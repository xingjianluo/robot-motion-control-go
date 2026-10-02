// Command demo 用确定性假时钟与模拟总线端到端演示（无需真实硬件）：
//
//	上电 ESTOP -> 条件不全拒绝恢复 -> 全部满足+人工确认 ->
//	连续速度轨迹与反馈闭环 -> 越界/矛盾命令区分拒绝 ->
//	连续流式三点拼接、末点保持 -> 在线倍率（越界倍率被拒）->
//	连续丢帧 COMM_LOSS -> 链路恢复不自动复位 -> 显式恢复 ->
//	力矩超限 TORQUE_FAULT 锁存 -> 缺人工确认拒绝 -> 观测刷新后恢复 ->
//	急停与周期/命令并发（确定性会合，结果固定）-> 可观测性统计。
//
// 运行：go run ./cmd/demo
//
// 全程只用手动推进的假时钟，不依赖 goroutine 调度时序与墙钟睡眠，
// 因此输出逐字节确定：连跑两次输出与退出码完全一致。
package main

import (
	"fmt"
	"io"
	"os"
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
	if err := run(os.Stdout); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

// run 执行全部演示场景。任何“预期被拒”都只打印、不中断；出现与预期
// 不符（例如急停后仍下发命令、陈旧帧未被淘汰）才以错误中断、非零退出。
// 所有输出写入 w，便于自动化测试捕获并做逐字节确定性比对。
func run(w io.Writer) error {
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
		return err
	}
	rec := report.NewRecorder(w)
	ctl.SetEventSink(rec)

	now := func() time.Time { return fc.Now() }
	// tick 顺序（确定）：设备先积分上一拍命令并产出状态帧（仍在旧时刻），
	// 随后推进时钟，控制器 Step 消费该反馈并下发“本拍”命令；
	// 设备在下一拍开头就能取到本拍命令。这是“反馈->决策->下发”的标准
	// 一拍延迟模型，链路顺序固定，不依赖 goroutine 调度。
	tick := func(label string) error {
		if derr := drv.Tick(); derr != nil {
			fmt.Fprintf(w, "[%s] 设备帧未入线（可观测通信故障）: %v\n", label, derr)
		}
		fc.Advance(period)
		return ctl.Step()
	}
	header := func(s string) {
		fmt.Fprintf(w, "\n================ %s ================\n", s)
	}
	short := func(s control.Snapshot) string {
		return fmt.Sprintf("cycle=%d state=%s outputs=%v healthy=%v",
			s.Cycle, s.SafetyState, s.OutputsOn, s.Tracking.Healthy)
	}
	header("1. 上电默认 ESTOP，命令必须被拒绝（安全优先于控制）")
	if _, err := ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.8}); err != nil {
		fmt.Fprintln(w, "  命令被拒（预期）:", err)
	}
	header("2. 恢复条件不全 => 明确拒绝，绝不静默恢复")
	partial := safety.RecoveryConditions{EstopReleased: true, OperatorAck: true}
	if err := ctl.Recover(partial); err != nil {
		fmt.Fprintln(w, "  恢复被拒（预期）:", err)
	}
	header("3. 全部条件满足 + 人工确认 => 恢复 RUN")
	okCond := safety.RecoveryConditions{
		EstopReleased: true, CommHealthy: true, TorqueWithinLimit: true,
		PositionWithinLimit: true, FaultCodeClear: true, OperatorAck: true,
	}
	// 先让设备发两拍，保证“通信健康/位置正常”的恢复依据真实存在
	for i := 0; i < 2; i++ {
		if err := tick("上电预热"); err != nil {
			return err
		}
	}
	if err := ctl.Recover(okCond); err != nil {
		return err
	}
	fmt.Fprintln(w, "  已恢复:", short(ctl.Snapshot()))

	header("4. 连续速度轨迹逼近 + 到位后位置保持（反馈闭环）")
	sendVelAt := func(v float64, at time.Time) error {
		_, err := ctl.SubmitCommand(trajectory.Command{
			Mode: protocol.ModeVelocity, Velocity: v, IssuedAt: at,
		})
		return err
	}
	sendPosAt := func(target float64, at time.Time) error {
		_, err := ctl.SubmitCommand(trajectory.Command{
			Mode: protocol.ModePosition, Position: target, IssuedAt: at,
		})
		return err
	}
	// 时序约定（确定、可复现）：
	//   每拍顺序为 提交本拍目标(IssuedAt=now) -> 设备tick(消费命令) ->
	//   推进时钟 -> 控制器Step(消费反馈)。同刻命令由轨迹缓冲做“替换”，
	//   因此连续重发恒定速度目标不会产生同刻冲突，设备每拍都收到命令。
	for i := 0; i < 30; i++ { // 1 rad/s × 30 × 10ms = 0.3 rad
		if ctl.Snapshot().SafetyState == safety.StateRun {
			if err := sendVelAt(1.0, fc.Now()); err != nil {
				return err
			}
		}
		if err := tick("速度跟踪"); err != nil {
			return err
		}
	}
	// 阶段二：平滑减速到零，再发位置保持。
	if err := sendVelAt(0, fc.Now()); err != nil {
		return err
	}
	if err := tick("减速"); err != nil {
		return err
	}
	for i := 0; i < 3; i++ {
		if ctl.Snapshot().SafetyState == safety.StateRun {
			if err := sendPosAt(drv.Position(), fc.Now()); err != nil {
				return err
			}
		}
		if err := tick("位置保持"); err != nil {
			return err
		}
	}
	s4 := ctl.Snapshot()
	fmt.Fprintf(w, "  跟踪结果: %s 设备位置=%.3f (期望≈0.30) 速度跟踪误差=%.4f rad/s\n",
		short(s4), drv.Position(), s4.Tracking.ErrorRadS)

	header("5. 越界/矛盾命令被拒绝且原因可区分")
	if _, err := ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 3}); err != nil {
		fmt.Fprintln(w, "  位置越界（预期）:", err)
	}
	if _, err := ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0, Velocity: 1}); err != nil {
		fmt.Fprintln(w, "  矛盾命令（预期）:", err)
	}

	header("6. 连续流式喂点：上一段没走完就续点，一路连续走完、不在点间停顿")
	// 一次塞入三个点（各 0.2s，段速 0.5/1.0/0.5 rad/s），模拟手轮/示教
	// 边动边喂。到达拼接点不停车，末点到后持续下发位置保持。
	feedStream := func(pos float64, in time.Duration) error {
		return ctl.SubmitStreamPoint(stream.Point{
			At: fc.Now().Add(in), PositionRad: pos,
		})
	}
	if err := feedStream(0.10, 200*time.Millisecond); err != nil {
		return err
	}
	if err := feedStream(0.30, 400*time.Millisecond); err != nil {
		return err
	}
	if err := feedStream(0.20, 600*time.Millisecond); err != nil {
		return err
	}
	fmt.Fprintf(w, "  已排队 %d 个流式点，开始连续推进...\n", ctl.StreamQueueLen())
	for i := 0; i < 90; i++ {
		if err := tick("流式拼接"); err != nil {
			return err
		}
	}
	s6 := ctl.Snapshot()
	fmt.Fprintf(w, "  流式结果: 到达点=%d 设备位置=%.3f (期望≈0.20) queued=%d\n  %s\n",
		s6.StreamStats.Reached, drv.Position(), ctl.StreamQueueLen(), short(s6))

	header("7. 在线速度倍率：50%->120%->暂停(0)->100%，越限倍率被明确拒绝")
	if err := feedStream(0.40, 300*time.Millisecond); err != nil { // 段速≈0.67，倍率 2 => 1.33 可行
		return err
	}
	if err := ctl.SetSpeedOverride(3); err != nil {
		fmt.Fprintln(w, "  倍率越界被拒（预期）:", err)
	}
	if err := ctl.SetSpeedOverride(0.5); err != nil {
		return err
	}
	for i := 0; i < 80; i++ {
		switch i {
		case 20:
			if err := ctl.SetSpeedOverride(1.2); err != nil {
				return err
			}
		case 40:
			if err := ctl.SetSpeedOverride(0); err != nil {
				return err
			} // 暂停推进
		case 55:
			if err := ctl.SetSpeedOverride(1); err != nil {
				return err
			} // 恢复
		}
		if err := tick("在线倍率"); err != nil {
			return err
		}
	}
	s7 := ctl.Snapshot()
	fmt.Fprintf(w, "  倍率结果: 当前倍率=%.2f 到达点=%d 设备位置=%.3f (期望≈0.40)\n  %s\n",
		s7.StreamFactor, s7.StreamStats.Reached, drv.Position(), short(s7))

	header("8. 注入连续丢帧 => 看门狗判定‘暂时失联’(COMM_LOSS)，非跟踪误差")
	// 接下来 6 拍设备不产生帧：注入计划让总线丢弃（这里直接停发设备帧，
	// 等价于链路中断；也可以用 bus.Inject 在 Post 侧丢帧）。
	for i := 0; i < 5; i++ {
		fc.Advance(period)
		if err := ctl.Step(); err != nil {
			return err
		}
	}
	fmt.Fprintln(w, "  失联状态:", short(ctl.Snapshot()))

	header("9. 链路自愈后状态不得自动回 RUN；必须显式恢复（且需真实观测核验）")
	for i := 0; i < 3; i++ {
		if err := tick("链路恢复"); err != nil {
			return err
		}
	}
	fmt.Fprintln(w, "  有健康反馈但仍锁存:", short(ctl.Snapshot()))
	if err := ctl.Recover(okCond); err != nil {
		return err
	}
	fmt.Fprintln(w, "  显式恢复后:", short(ctl.Snapshot()))

	header("10. V2 力矩命令下发；随后驱动器上报超力矩 => TORQUE_FAULT 锁存")
	if _, err := ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModeTorque, Torque: 2}); err != nil {
		return err
	}
	if err := tick("力矩下发"); err != nil {
		return err
	}
	drv.SetFaultCode(0)
	// 注入一帧超力矩反馈，触发安全联锁（设备模型自身不产生越限力矩，
	// 这里模拟电流环/编码器独立上报）。序号必须紧跟设备真实序号：
	// 若注入一个远大于设备序号的帧，设备后续的健康帧会被反馈层永久判为
	// “序号回退”而不被接受，恢复时系统的真实观测就一直停留在超力矩帧上，
	// 导致随后合法恢复也被误拒、演示中断。
	overSeq := drv.Seq() + 1
	overLine, _ := protocol.EncodeStatus(protocol.V2, protocol.DriveStatus{
		PositionRad: drv.Position(), VelocityRadS: 0, TorqueNm: 7.5,
		Enabled: true, FaultCode: 0, Seq: overSeq, TsMs: now().UnixMilli(),
	})
	if perr := b.Post(bus.Uplink, overLine, overSeq); perr != nil {
		return perr
	}
	fc.Advance(period)
	if serr := ctl.Step(); serr != nil {
		return serr
	}
	fmt.Fprintln(w, "  力矩故障:", short(ctl.Snapshot()))
	// 缺人工确认不得恢复：此刻系统最新观测仍是那帧超力矩反馈，
	// 因此该请求同时与观测矛盾；拒绝原因以安全/校验结果为准，不放宽。
	bad := okCond
	bad.OperatorAck = false
	if err := ctl.Recover(bad); err != nil {
		fmt.Fprintln(w, "  无人工确认恢复被拒（预期）:", err)
	}
	// 让设备再产出两拍真实健康帧（力矩 0、序号连续），把系统观测刷新为
	// 正常；此时“力矩正常”的恢复声明才有真实证据支撑。
	if err := tick("力矩恢复观测1"); err != nil {
		return err
	}
	if err := tick("力矩恢复观测2"); err != nil {
		return err
	}
	if err := ctl.Recover(okCond); err != nil {
		return fmt.Errorf("人工确认恢复应在健康观测下成功: %w", err)
	}
	fmt.Fprintln(w, "  人工确认恢复:", short(ctl.Snapshot()))

	header("11. 急停与周期推进/命令提交并发：最终必须为 ESTOP 且无新命令")
	// 并发的“数据竞争安全性”由 go test -race 的并发用例（200 周期四方
	// 竞争）覆盖；演示这里需要【逐字节确定】的输出，因此仍让操作真正
	// 发生在独立 goroutine 上（锁竞争与 -race 同样生效），但用无缓冲
	// channel 作为会合点，把交错顺序钉死成确定序列：
	//
	//	周期 1..10 ：每拍另一个 goroutine 先并发提交 1 条位置命令，
	//	             随后推进周期（命令必然入缓冲、周期必然下发）；
	//	周期 11    ：另一个 goroutine 并发按下急停后再推进周期；
	//	周期 12..100：只推进周期，不再有任何命令入线。
	//
	// 不使用墙钟 sleep：sleep 时长决定交错顺序正是旧版输出不确定的根因。
	commanderDone := make(chan struct{})
	stepperDone := make(chan struct{})
	submitKick := make(chan struct{})
	estopKick := make(chan struct{})
	acceptedBeforeEstop := 0
	go func() { // 命令/急停 goroutine（与周期 goroutine 真实并发）
		defer close(commanderDone)
		for i := 0; i < 10; i++ {
			<-submitKick
			if _, cerr := ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.2}); cerr == nil {
				acceptedBeforeEstop++
			}
			submitKick <- struct{}{} // 回执：命令提交临界区已完成
		}
		<-estopKick
		ctl.Estop("并发急停按钮")
		estopKick <- struct{}{} // 回执：急停临界区已完成
	}()
	go func() { // 周期 goroutine
		defer close(stepperDone)
		for i := 0; i < 100; i++ {
			if i < 10 {
				submitKick <- struct{}{}
				<-submitKick // 等命令提交完成，再推进本拍，顺序确定
			} else if i == 10 {
				estopKick <- struct{}{}
				<-estopKick // 急停先于本拍提交，本拍读到的必为 ESTOP
			}
			// 前 10 拍设备正常产出反馈，保证并发提交期间链路健康、
			// 命令确实在被接受并下发（而不是被看门狗 COMM_LOSS 抢先）；
			// 急停之后设备是否在帧不再影响结论。
			if i < 10 {
				if derr := drv.Tick(); derr != nil {
					fmt.Fprintf(w, "[并发] 设备帧未入线: %v\n", derr)
				}
			}
			fc.Advance(period)
			_ = ctl.Step()
		}
	}()
	<-commanderDone
	<-stepperDone
	if ctl.Snapshot().SafetyState != safety.StateEstop {
		return fmt.Errorf("并发结束后应为 ESTOP，实际 %s", ctl.Snapshot().SafetyState)
	}
	beforeDown := b.Stats().DownEnqueued
	for i := 0; i < 3; i++ {
		fc.Advance(period)
		_ = ctl.Step()
	}
	if b.Stats().DownEnqueued != beforeDown {
		return fmt.Errorf("急停后仍有 %d 条命令下发，结果不确定", b.Stats().DownEnqueued-beforeDown)
	}
	fmt.Fprintln(w, "  并发结束:", short(ctl.Snapshot()),
		fmt.Sprintf(" 急停前接受命令=%d 急停后新增下行命令数=0 ✓", acceptedBeforeEstop))

	header("12. 可观测性：总线统计、积压丢弃、注入丢帧与周期抖动")
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
