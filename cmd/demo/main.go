// Command demo 用确定性假时钟与模拟总线端到端演示（无需真实硬件）：
//
//	上电 ESTOP -> 条件不全拒绝恢复 -> 全部满足+人工确认 ->
//	位置命令连续下发与反馈跟踪 -> 注入丢帧导致暂时失联(COMM_LOSS) ->
//	链路恢复但不自动复位 -> 显式恢复 -> 力矩超限锁存(TORQUE_FAULT) ->
//	恢复 -> V1 历史版本兼容 -> 急停与周期推进并发（结果确定）。
//
// 运行：go run ./cmd/demo
package main

import (
	"fmt"
	"io"
	"os"
	"sync"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/bus"
	"github.com/xingjianluo/robot-motion-control-go/internal/clock"
	"github.com/xingjianluo/robot-motion-control-go/internal/control"
	"github.com/xingjianluo/robot-motion-control-go/internal/device"
	"github.com/xingjianluo/robot-motion-control-go/internal/feedback"
	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
	"github.com/xingjianluo/robot-motion-control-go/internal/report"
	"github.com/xingjianluo/robot-motion-control-go/internal/safety"
	"github.com/xingjianluo/robot-motion-control-go/internal/stream"
	"github.com/xingjianluo/robot-motion-control-go/internal/trajectory"
)

func main() {
	if err := runDemo(os.Stdout); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

// runDemo 运行全部 12 个端到端场景，全部输出（含审计 Dump）写入 w。
// 固定起始时刻与固定参数 => 输出逐字节确定；任何不该发生的错误以
// 返回值（而非静默吞掉）上报，由 main 以非 0 退出码退出。
func runDemo(w io.Writer) error {
	t0 := time.Date(2026, 9, 22, 9, 0, 0, 0, time.UTC)
	fc := clock.NewFake(t0)
	period := 10 * time.Millisecond

	// 总线：容量 16，满时丢最旧；超过 200ms 未取走的帧按陈旧丢弃。
	b := bus.New(bus.Config{
		Capacity: 16, Policy: bus.DropOldest, MaxAge: 200 * time.Millisecond,
	}, fc)
	drv := device.New(device.Config{
		Version: protocol.V2, DT: period, Gain: 40, MaxVel: 2,
		HardMin: -2, HardMax: 2, StartPos: 0,
		// 连续流式轨迹使用带位置命令前馈的伺服模型跟踪（不影响单目标行为）。
		PosCmdFeedforward: true, PosFFGain: 40,
	}, fc, b)

	ctl, err := control.New(control.Config{
		ProtocolVersion: protocol.V2,
		Period:          period,
		JitterTolerance: 15 * time.Millisecond,
		MaxBacklog:      8,
		TrajCapacity:    32,
		Motion: trajectory.Limits{
			MinPositionRad: -2, MaxPositionRad: 2,
			MaxVelocityRadS: 2, MaxAccelRadS2: 200, MaxTorqueNm: 5,
		},
		SafetyLim: safety.Limits{
			MinPositionRad: -2, MaxPositionRad: 2, MaxTorqueNm: 5, SoftMarginRad: 0.1,
		},
		Feedback: feedback.Config{
			WatchdogTimeout: 35 * time.Millisecond, MaxStepRad: 1.0, MaxBackdateMs: 5,
		},
		Bus: bus.Config{Capacity: 16, Policy: bus.DropOldest, MaxAge: 200 * time.Millisecond},
		Stream: control.StreamConfig{
			Capacity: 32, FullPolicy: "reject_new", MinOverride: 0, MaxOverride: 2,
		},
		// 演示恢复必须以系统真实观测交叉核验为准，不能光信口头声明。
		RequireObservedRecovery: true,
	}, fc, b)
	if err != nil {
		return fmt.Errorf("demo failure: %w", err)
	}
	rec := report.NewRecorder(w)
	ctl.SetEventSink(rec)

	now := func() time.Time { return fc.Now() }
	// tick 顺序（确定）：设备先积分上一拍命令并产出状态帧（仍在旧时刻），
	// 随后推进时钟，控制器 Step 消费该反馈并下发“本拍”命令；
	// 设备在下一拍开头就能取到本拍命令。这是“反馈->决策->下发”的标准
	// 一拍延迟模型，链路顺序固定，不依赖 goroutine 调度。
	// demoErr 收集辅助闭包内发生的致命错误；闭包（tick/send*）签名不
	// 带返回值，捕获后由调用点检查并返回，保证异常以非 0 退出码退出而非 panic。
	var demoErr error
	tick := func(label string) {
		if derr := drv.Tick(); derr != nil {
			fmt.Fprintf(w, "[%s] 设备帧未入线（可观测通信故障）: %v\n", label, derr)
		}
		fc.Advance(period)
		if serr := ctl.Step(); serr != nil {
			demoErr = fmt.Errorf("cycle step (%s): %w", label, serr)
		}
	}
	header(w, "1. 上电默认 ESTOP，命令必须被拒绝（安全优先于控制）")
	if _, err := ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.8}); err != nil {
		fmt.Fprintln(w, "  命令被拒（预期）:", err)
	}
	header(w, "2. 恢复条件不全 => 明确拒绝，绝不静默恢复")
	partial := safety.RecoveryConditions{EstopReleased: true, OperatorAck: true}
	if err := ctl.Recover(partial); err != nil {
		fmt.Fprintln(w, "  恢复被拒（预期）:", err)
	}
	header(w, "3. 全部条件满足 + 人工确认 => 恢复 RUN")
	okCond := safety.RecoveryConditions{
		EstopReleased: true, CommHealthy: true, TorqueWithinLimit: true,
		PositionWithinLimit: true, FaultCodeClear: true, OperatorAck: true,
	}
	// 先让设备发两拍，保证“通信健康/位置正常”的恢复依据真实存在
	for i := 0; i < 2; i++ {
		tick("上电预热")
	}
	if err := ctl.Recover(okCond); err != nil {
		return fmt.Errorf("demo failure: %w", err)
	}
	fmt.Fprintln(w, "  已恢复:", short(ctl.Snapshot()))

	header(w, "4. 连续速度轨迹逼近 + 到位后位置保持（反馈闭环）")
	sendVelAt := func(v float64, at time.Time) {
		if demoErr != nil {
			return
		}
		if _, err := ctl.SubmitCommand(trajectory.Command{
			Mode: protocol.ModeVelocity, Velocity: v, IssuedAt: at,
		}); err != nil {
			demoErr = fmt.Errorf("velocity submit: %w", err)
		}
	}
	sendPosAt := func(target float64, at time.Time) {
		if demoErr != nil {
			return
		}
		if _, err := ctl.SubmitCommand(trajectory.Command{
			Mode: protocol.ModePosition, Position: target, IssuedAt: at,
		}); err != nil {
			demoErr = fmt.Errorf("position submit: %w", err)
		}
	}
	// 时序约定（确定、可复现）：
	//   每拍顺序为 提交本拍目标(IssuedAt=now) -> 设备tick(消费命令) ->
	//   推进时钟 -> 控制器Step(消费反馈)。同刻命令由轨迹缓冲做“替换”，
	//   因此连续重发恒定速度目标不会产生同刻冲突，设备每拍都收到命令。
	for i := 0; i < 30; i++ { // 1 rad/s × 30 × 10ms = 0.3 rad
		if ctl.Snapshot().SafetyState == safety.StateRun {
			sendVelAt(1.0, fc.Now())
		}
		tick("速度跟踪")
	}
	// 阶段二：平滑减速到零，再发位置保持。
	sendVelAt(0, fc.Now())
	tick("减速")
	for i := 0; i < 3; i++ {
		if ctl.Snapshot().SafetyState == safety.StateRun {
			sendPosAt(drv.Position(), fc.Now())
		}
		tick("位置保持")
	}
	if demoErr != nil {
		return demoErr
	}
	s4 := ctl.Snapshot()
	fmt.Fprintf(w, "  跟踪结果: %s 设备位置=%.3f (期望≈0.30) 速度跟踪误差=%.4f rad/s\n",
		short(s4), drv.Position(), s4.Tracking.ErrorRadS)

	header(w, "5. 越界/矛盾命令被拒绝且原因可区分")
	if _, err := ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 3}); err != nil {
		fmt.Fprintln(w, "  位置越界（预期）:", err)
	}
	if _, err := ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0, Velocity: 1}); err != nil {
		fmt.Fprintln(w, "  矛盾命令（预期）:", err)
	}

	header(w, "6. 连续流式喂点：上一段没走完就续点，一路连续走完、不在点间停顿")
	// 一次塞入三个点（各 0.2s，段速 0.5/1.0/0.5 rad/s），模拟手轮/示教
	// 边动边喂。到达拼接点不停车，末点到后持续下发位置保持。
	feedStream := func(pos float64, in time.Duration) {
		if demoErr != nil {
			return
		}
		if err := ctl.SubmitStreamPoint(stream.Point{
			At: fc.Now().Add(in), PositionRad: pos,
		}); err != nil {
			demoErr = fmt.Errorf("stream point submit: %w", err)
		}
	}
	feedStream(0.10, 200*time.Millisecond)
	feedStream(0.30, 400*time.Millisecond)
	feedStream(0.20, 600*time.Millisecond)
	fmt.Fprintf(w, "  已排队 %d 个流式点，开始连续推进...\n", ctl.StreamQueueLen())
	for i := 0; i < 90; i++ {
		tick("流式拼接")
	}
	if demoErr != nil {
		return demoErr
	}
	s6 := ctl.Snapshot()
	fmt.Fprintf(w, "  流式结果: 到达点=%d 设备位置=%.3f (期望≈0.20) queued=%d\n  %s\n",
		s6.StreamStats.Reached, drv.Position(), ctl.StreamQueueLen(), short(s6))

	header(w, "7. 在线速度倍率：50%->120%->暂停(0)->100%，越限倍率被明确拒绝")
	feedStream(0.40, 300*time.Millisecond) // 段速≈0.67，倍率 2 => 1.33 可行
	if err := ctl.SetSpeedOverride(3); err != nil {
		fmt.Fprintln(w, "  倍率越界被拒（预期）:", err)
	}
	if err := ctl.SetSpeedOverride(0.5); err != nil {
		return fmt.Errorf("demo failure: %w", err)
	}
	for i := 0; i < 80; i++ {
		switch i {
		case 20:
			if err := ctl.SetSpeedOverride(1.2); err != nil {
				return fmt.Errorf("demo failure: %w", err)
			}
		case 40:
			if err := ctl.SetSpeedOverride(0); err != nil {
				return fmt.Errorf("demo failure: %w", err)
			} // 暂停推进
		case 55:
			if err := ctl.SetSpeedOverride(1); err != nil {
				return fmt.Errorf("demo failure: %w", err)
			} // 恢复
		}
		tick("在线倍率")
	}
	if demoErr != nil {
		return demoErr
	}
	s7 := ctl.Snapshot()
	fmt.Fprintf(w, "  倍率结果: 当前倍率=%.2f 到达点=%d 设备位置=%.3f (期望≈0.40)\n  %s\n",
		s7.StreamFactor, s7.StreamStats.Reached, drv.Position(), short(s7))

	header(w, "8. 注入连续丢帧 => 看门狗判定‘暂时失联’(COMM_LOSS)，非跟踪误差")
	// 接下来 6 拍设备不产生帧：注入计划让总线丢弃（这里直接停发设备帧，
	// 等价于链路中断；也可以用 bus.Inject 在 Post 侧丢帧）。
	for i := 0; i < 5; i++ {
		fc.Advance(period)
		if err := ctl.Step(); err != nil {
			return fmt.Errorf("demo failure: %w", err)
		}
	}
	fmt.Fprintln(w, "  失联状态:", short(ctl.Snapshot()))

	header(w, "9. 链路自愈后状态不得自动回 RUN；必须显式恢复（且需真实观测核验）")
	for i := 0; i < 3; i++ {
		tick("链路恢复")
	}
	if demoErr != nil {
		return demoErr
	}
	fmt.Fprintln(w, "  有健康反馈但仍锁存:", short(ctl.Snapshot()))
	if err := ctl.Recover(okCond); err != nil {
		return fmt.Errorf("demo failure: %w", err)
	}
	fmt.Fprintln(w, "  显式恢复后:", short(ctl.Snapshot()))

	header(w, "10. V2 力矩命令下发；随后驱动器上报超力矩 => TORQUE_FAULT 锁存")
	if _, err := ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModeTorque, Torque: 2}); err != nil {
		return fmt.Errorf("demo failure: %w", err)
	}
	tick("力矩下发")
	drv.SetFaultCode(0)
	// 直接注入一帧超力矩反馈（模拟电流环/编码器独立上报）。序号必须远
	// 高于设备当前序号：恢复时再注入一帧序号严格更大的健康帧作为真实
	// 观测依据；若让设备直接重发小编号帧，会被反馈消费者按“序号回退”
	// 判乱序拒绝，最近接受样本将永远停在超力矩帧上。
	overSeq := uint64(100000)
	overLine, _ := protocol.EncodeStatus(protocol.V2, protocol.DriveStatus{
		PositionRad: drv.Position(), VelocityRadS: 0, TorqueNm: 7.5,
		Enabled: true, FaultCode: 0, Seq: overSeq, TsMs: now().UnixMilli(),
	})
	_ = b.Post(bus.Uplink, overLine, overSeq)
	fc.Advance(period)
	if serr := ctl.Step(); serr != nil {
		return fmt.Errorf("demo failure: %w", serr)
	}
	fmt.Fprintln(w, "  力矩故障:", short(ctl.Snapshot()))
	// 缺人工确认不得恢复
	bad := okCond
	bad.OperatorAck = false
	if err := ctl.Recover(bad); err != nil {
		fmt.Fprintln(w, "  无人工确认恢复被拒（预期）:", err)
	}
	// 恢复必须以系统“真实观测”交叉核验：电流环/编码器已确认力矩回到
	// 正常，这里注入一帧【序号严格更大】的健康 V2 状态帧供控制器消费
	// （设备自身序号还很小，直接让设备重发会因序号回退被判乱序拒绝，
	// 最近接受样本就永远是那条超力矩帧）。不放宽任何恢复校验：
	// 没有这帧真实健康观测，下面的恢复依旧必须被拒。
	healthyLine, _ := protocol.EncodeStatus(protocol.V2, protocol.DriveStatus{
		PositionRad: drv.Position(), VelocityRadS: 0, TorqueNm: 0,
		Enabled: true, FaultCode: 0, Seq: overSeq + 1, TsMs: now().UnixMilli(),
	})
	if err := b.Post(bus.Uplink, healthyLine, overSeq+1); err != nil {
		return fmt.Errorf("demo failure: %w", err)
	}
	fc.Advance(period)
	if serr := ctl.Step(); serr != nil {
		return fmt.Errorf("demo failure: %w", serr)
	}
	if err := ctl.Recover(okCond); err != nil {
		return fmt.Errorf("demo failure: %w", err)
	}
	fmt.Fprintln(w, "  人工确认恢复:", short(ctl.Snapshot()))

	header(w, "11. 急停与周期推进/命令提交并发：最终必须为 ESTOP 且无新命令")
	// 并发段使用【独立 rig】，且不挂审计记录器：goroutine 的锁竞争交错
	// 只改变“被拒命令/被处理周期”的次数，锁保证的最终状态（ESTOP、
	// 急停后下行计数不再增长）始终确定。把这些随调度变化的计数隔离在
	// 主记录器之外，主审计 Dump 才能逐字节一致。
	cc := clock.NewFake(t0)
	cb := bus.New(bus.Config{
		Capacity: 16, Policy: bus.DropOldest, MaxAge: 200 * time.Millisecond,
	}, cc)
	cdrv := device.New(device.Config{
		Version: protocol.V2, DT: period, Gain: 40, MaxVel: 2,
		HardMin: -2, HardMax: 2, StartPos: 0,
		PosCmdFeedforward: true, PosFFGain: 40,
	}, cc, cb)
	cctl, cerr := control.New(control.Config{
		ProtocolVersion: protocol.V2,
		Period:          period,
		JitterTolerance: 15 * time.Millisecond,
		MaxBacklog:      8,
		TrajCapacity:    32,
		Motion: trajectory.Limits{
			MinPositionRad: -2, MaxPositionRad: 2,
			MaxVelocityRadS: 2, MaxAccelRadS2: 200, MaxTorqueNm: 5,
		},
		SafetyLim: safety.Limits{
			MinPositionRad: -2, MaxPositionRad: 2, MaxTorqueNm: 5, SoftMarginRad: 0.1,
		},
		Feedback: feedback.Config{
			WatchdogTimeout: 35 * time.Millisecond, MaxStepRad: 1.0, MaxBackdateMs: 5,
		},
		Bus: bus.Config{Capacity: 16, Policy: bus.DropOldest, MaxAge: 200 * time.Millisecond},
		Stream: control.StreamConfig{
			Capacity: 32, FullPolicy: "reject_new", MinOverride: 0, MaxOverride: 2,
		},
		RequireObservedRecovery: false, // 并发 rig：先上电预热再显式恢复到 RUN
	}, cc, cb)
	if cerr != nil {
		return fmt.Errorf("demo failure: %w", cerr)
	}
	// 并发前把独立 rig 带到 RUN：设备两拍建立真实健康观测，再显式恢复。
	for i := 0; i < 2; i++ {
		if derr := cdrv.Tick(); derr != nil {
			return fmt.Errorf("demo failure: %w", derr)
		}
		cc.Advance(period)
		if serr := cctl.Step(); serr != nil {
			return fmt.Errorf("demo failure: %w", serr)
		}
	}
	if err := cctl.Recover(safety.RecoveryConditions{
		EstopReleased: true, CommHealthy: true, TorqueWithinLimit: true,
		PositionWithinLimit: true, FaultCodeClear: true, OperatorAck: true,
	}); err != nil {
		return fmt.Errorf("demo failure: %w", err)
	}
	var wg sync.WaitGroup
	wg.Add(3)
	go func() {
		defer wg.Done()
		for i := 0; i < 100; i++ {
			cc.Advance(period)
			_ = cctl.Step()
		}
	}()
	go func() {
		defer wg.Done()
		time.Sleep(500 * time.Microsecond)
		cctl.Estop("并发急停按钮")
	}()
	go func() {
		defer wg.Done()
		for i := 0; i < 100; i++ {
			_, _ = cctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.2})
		}
	}()
	wg.Wait()
	beforeDown := cb.Stats().DownEnqueued
	for i := 0; i < 3; i++ {
		cc.Advance(period)
		_ = cctl.Step()
	}
	if cb.Stats().DownEnqueued != beforeDown {
		return fmt.Errorf("急停后仍有命令下发，结果不确定")
	}
	fmt.Fprintln(w, "  并发结束:", short(cctl.Snapshot()), " 急停后新增下行命令数=0 ✓")

	header(w, "12. 可观测性：总线统计、积压丢弃、注入丢帧与周期抖动")
	snap := ctl.Snapshot()
	fmt.Fprintf(w, "  主链路 丢弃(满/陈旧): %d/%d  注入丢帧: %d  最大抖动: %v  抖动超限次数: %d\n",
		snap.BusStats.DroppedFull, snap.BusStats.DroppedStale,
		snap.BusStats.InjectedLoss, snap.MaxJitter, snap.JitterBreaches)

	// 独立演示：小容量总线 + drop_newest 策略 + 确定性故障注入。
	// 不接控制器，纯粹展示“积压有界、丢弃可观测、归属明确”。
	fc2 := clock.NewFake(t0)
	b2 := bus.New(bus.Config{Capacity: 2, Policy: bus.DropNewest, MaxAge: 20 * time.Millisecond}, fc2)
	b2.Inject(bus.Injection{DropAt: []int{1}})
	for seq := uint64(1); seq <= 4; seq++ {
		if perr := b2.Post(bus.Uplink, "frame", seq); perr != nil {
			fmt.Fprintf(w, "  Post#%d 被明确拒绝: %v\n", seq, perr)
		}
	}
	fc2.Advance(25 * time.Millisecond)
	if _, ok := b2.Poll(bus.Uplink); ok {
		return fmt.Errorf("陈旧帧应被淘汰")
	}
	st2 := b2.Stats()
	fmt.Fprintf(w, "  演示总线: 注入丢帧=%d 满丢弃=%d 陈旧丢弃=%d 最近丢弃原因=%s(seq=%d)\n",
		st2.InjectedLoss, st2.DroppedFull, st2.DroppedStale, st2.LastDropReason, st2.LastDropSeq)

	// 把安全状态机变迁历史同步到记录器，保证审计输出完整。
	rec.SyncSafety(ctl.SafetyEvents())

	fmt.Fprintln(w)
	rec.Dump()
	return nil
}

func header(w io.Writer, s string) {
	fmt.Fprintf(w, "\n================ %s ================\n", s)
}

func short(s control.Snapshot) string {
	return fmt.Sprintf("cycle=%d state=%s outputs=%v healthy=%v",
		s.Cycle, s.SafetyState, s.OutputsOn, s.Tracking.Healthy)
}
