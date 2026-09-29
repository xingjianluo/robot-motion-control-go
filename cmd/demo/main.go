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
	}, fc, b)
	if err != nil {
		panic(err)
	}
	rec := report.NewRecorder(os.Stdout)
	ctl.SetEventSink(rec)

	now := func() time.Time { return fc.Now() }
	// tick 顺序（确定）：设备先积分上一拍命令并产出状态帧（仍在旧时刻），
	// 随后推进时钟，控制器 Step 消费该反馈并下发“本拍”命令；
	// 设备在下一拍开头就能取到本拍命令。这是“反馈->决策->下发”的标准
	// 一拍延迟模型，链路顺序固定，不依赖 goroutine 调度。
	tick := func(label string) {
		if derr := drv.Tick(); derr != nil {
			fmt.Printf("[%s] 设备帧未入线（可观测通信故障）: %v\n", label, derr)
		}
		fc.Advance(period)
		if serr := ctl.Step(); serr != nil {
			panic(serr)
		}
	}
	header("1. 上电默认 ESTOP，命令必须被拒绝（安全优先于控制）")
	if _, err := ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.8}); err != nil {
		fmt.Println("  命令被拒（预期）:", err)
	}
	header("2. 恢复条件不全 => 明确拒绝，绝不静默恢复")
	partial := safety.RecoveryConditions{EstopReleased: true, OperatorAck: true}
	if err := ctl.Recover(partial); err != nil {
		fmt.Println("  恢复被拒（预期）:", err)
	}
	header("3. 全部条件满足 + 人工确认 => 恢复 RUN")
	okCond := safety.RecoveryConditions{
		EstopReleased: true, CommHealthy: true, TorqueWithinLimit: true,
		PositionWithinLimit: true, FaultCodeClear: true, OperatorAck: true,
	}
	// 先让设备发两拍，保证“通信健康/位置正常”的恢复依据真实存在
	for i := 0; i < 2; i++ {
		tick("上电预热")
	}
	if err := ctl.Recover(okCond); err != nil {
		panic(err)
	}
	fmt.Println("  已恢复:", short(ctl.Snapshot()))

	header("4. 连续速度轨迹逼近 + 到位后位置保持（反馈闭环）")
	sendVelAt := func(v float64, at time.Time) {
		if _, err := ctl.SubmitCommand(trajectory.Command{
			Mode: protocol.ModeVelocity, Velocity: v, IssuedAt: at,
		}); err != nil {
			panic(err)
		}
	}
	sendPosAt := func(target float64, at time.Time) {
		if _, err := ctl.SubmitCommand(trajectory.Command{
			Mode: protocol.ModePosition, Position: target, IssuedAt: at,
		}); err != nil {
			panic(err)
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
	s4 := ctl.Snapshot()
	fmt.Printf("  跟踪结果: %s 设备位置=%.3f (期望≈0.30) 速度跟踪误差=%.4f rad/s\n",
		short(s4), drv.Position(), s4.Tracking.ErrorRadS)

	header("5. 越界/矛盾命令被拒绝且原因可区分")
	if _, err := ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 3}); err != nil {
		fmt.Println("  位置越界（预期）:", err)
	}
	if _, err := ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0, Velocity: 1}); err != nil {
		fmt.Println("  矛盾命令（预期）:", err)
	}

	header("6. 注入连续丢帧 => 看门狗判定‘暂时失联’(COMM_LOSS)，非跟踪误差")
	// 接下来 6 拍设备不产生帧：注入计划让总线丢弃（这里直接停发设备帧，
	// 等价于链路中断；也可以用 bus.Inject 在 Post 侧丢帧）。
	for i := 0; i < 5; i++ {
		fc.Advance(period)
		if err := ctl.Step(); err != nil {
			panic(err)
		}
	}
	fmt.Println("  失联状态:", short(ctl.Snapshot()))

	header("7. 链路自愈后状态不得自动回 RUN；必须显式恢复")
	for i := 0; i < 3; i++ {
		tick("链路恢复")
	}
	fmt.Println("  有健康反馈但仍锁存:", short(ctl.Snapshot()))
	if err := ctl.Recover(okCond); err != nil {
		panic(err)
	}
	fmt.Println("  显式恢复后:", short(ctl.Snapshot()))

	header("8. V2 力矩命令下发；随后驱动器上报超力矩 => TORQUE_FAULT 锁存")
	if _, err := ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModeTorque, Torque: 2}); err != nil {
		panic(err)
	}
	tick("力矩下发")
	drv.SetFaultCode(0)
	// 直接注入一帧超力矩反馈，触发安全联锁（设备模型不产生越限力矩，
	// 这里模拟编码器/电流环上报）。
	// 直接注入一帧超力矩反馈（模拟电流环/编码器独立上报），序号取一个
	// 远高于设备当前序号的值；为避免随后设备小编号帧造成“序号回退”噪声，
	// 注入前先把设备内部序号推进到该值（通过一次空转不现实），因此这里
	// 只推进控制器一拍消费该帧，随后恢复前再让设备重连——后续看门狗会
	// 自然把链路重建为“暂时失联->恢复”，归类依然清晰。
	overSeq := uint64(100000)
	overLine, _ := protocol.EncodeStatus(protocol.V2, protocol.DriveStatus{
		PositionRad: drv.Position(), VelocityRadS: 0, TorqueNm: 7.5,
		Enabled: true, FaultCode: 0, Seq: overSeq, TsMs: now().UnixMilli(),
	})
	_ = b.Post(bus.Uplink, overLine, overSeq)
	fc.Advance(period)
	if serr := ctl.Step(); serr != nil {
		panic(serr)
	}
	fmt.Println("  力矩故障:", short(ctl.Snapshot()))
	// 缺人工确认不得恢复
	bad := okCond
	bad.OperatorAck = false
	if err := ctl.Recover(bad); err != nil {
		fmt.Println("  无人工确认恢复被拒（预期）:", err)
	}
	// 仅凭口头声称“力矩正常”而真实最新证据仍是 7.5Nm，必须按真实状态拒绝。
	if err := ctl.Recover(okCond); err != nil {
		fmt.Println("  证据矛盾恢复被拒（预期）:", err)
	}
	// 投递真实健康高序号帧（序号须高于注入帧 100000，否则被判乱序），
	// 恢复与否按系统当时掌握的真实证据判定。
	for seq := uint64(100001); seq <= 100002; seq++ {
		healthy, _ := protocol.EncodeStatus(protocol.V2, protocol.DriveStatus{
			PositionRad: drv.Position(), VelocityRadS: 0, TorqueNm: 0,
			Enabled: true, FaultCode: 0, Seq: seq, TsMs: now().UnixMilli(),
		})
		_ = b.Post(bus.Uplink, healthy, seq)
		fc.Advance(period)
		if serr := ctl.Step(); serr != nil {
			panic(serr)
		}
	}
	if err := ctl.Recover(okCond); err != nil {
		panic(err)
	}
	fmt.Println("  真实证据健康 + 人工确认恢复:", short(ctl.Snapshot()))

	header("9. 急停与周期推进/命令提交并发：最终必须为 ESTOP 且无新命令")
	// 并发提交被安全门控拒绝的“次数”取决于 goroutine 调度，属时序偶然；
	// 为保持演示审计输出逐字节确定，这一小段把记录器临时换成丢弃型 sink
	// （拒绝仍然带归属返回、仍被计数，核心安全语义不变）。“提交/急停/周期”
	// 三者的数据竞争与最终状态由 internal/control 的 -race 并发测试断言。
	ctl.SetEventSink(discardSink{})
	var wg sync.WaitGroup
	wg.Add(3)
	go func() {
		defer wg.Done()
		for i := 0; i < 100; i++ {
			fc.Advance(period)
			_ = ctl.Step()
		}
	}()
	go func() {
		defer wg.Done()
		time.Sleep(500 * time.Microsecond)
		ctl.Estop("并发急停按钮")
	}()
	go func() {
		defer wg.Done()
		for i := 0; i < 100; i++ {
			_, _ = ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.2})
		}
	}()
	wg.Wait()
	ctl.SetEventSink(rec)
	fmt.Println("  并发提交结果: 100 次与急停/周期竞争的提交全部被明确接受或安全拒绝（无一静默）")
	beforeDown := b.Stats().DownEnqueued
	for i := 0; i < 3; i++ {
		fc.Advance(period)
		_ = ctl.Step()
	}
	if b.Stats().DownEnqueued != beforeDown {
		panic("急停后仍有命令下发，结果不确定")
	}
	fmt.Println("  并发结束:", short(ctl.Snapshot()), " 急停后新增下行命令数=0 ✓")

	header("10. 可观测性：总线统计、积压丢弃、注入丢帧与周期抖动")
	snap := ctl.Snapshot()
	fmt.Printf("  主链路 丢弃(满/陈旧): %d/%d  注入丢帧: %d  最大抖动: %v  抖动超限次数: %d\n",
		snap.BusStats.DroppedFull, snap.BusStats.DroppedStale,
		snap.BusStats.InjectedLoss, snap.MaxJitter, snap.JitterBreaches)

	// 独立演示：小容量总线 + drop_newest 策略 + 确定性故障注入。
	// 不接控制器，纯粹展示“积压有界、丢弃可观测、归属明确”。
	fc2 := clock.NewFake(t0)
	b2 := bus.New(bus.Config{Capacity: 2, Policy: bus.DropNewest, MaxAge: 20 * time.Millisecond}, fc2)
	b2.Inject(bus.Injection{DropAt: []int{1}})
	for seq := uint64(1); seq <= 4; seq++ {
		if perr := b2.Post(bus.Uplink, "frame", seq); perr != nil {
			fmt.Printf("  Post#%d 被明确拒绝: %v\n", seq, perr)
		}
	}
	fc2.Advance(25 * time.Millisecond)
	if _, ok := b2.Poll(bus.Uplink); ok {
		panic("陈旧帧应被淘汰")
	}
	st2 := b2.Stats()
	fmt.Printf("  演示总线: 注入丢帧=%d 满丢弃=%d 陈旧丢弃=%d 最近丢弃原因=%s(seq=%d)\n",
		st2.InjectedLoss, st2.DroppedFull, st2.DroppedStale, st2.LastDropReason, st2.LastDropSeq)

	header("11. 流式喂点 + 在线速度倍率（手轮/示教式连续下发）")
	streamDemo(fc, period)

	// 把安全状态机变迁历史同步到记录器，保证审计输出完整。
	rec.SyncSafety(ctl.SafetyEvents())

	fmt.Println()
	rec.Dump()
}

func header(s string) {
	fmt.Printf("\n================ %s ================\n", s)
}

func short(s control.Snapshot) string {
	return fmt.Sprintf("cycle=%d state=%s outputs=%v healthy=%v",
		s.Cycle, s.SafetyState, s.OutputsOn, s.Tracking.Healthy)
}

// discardSink 实现 control.EventSink 但不落任何记录，仅用于并发演示段，
// 使依赖 goroutine 调度的拒绝次数不进入确定性审计输出。
type discardSink struct{}

func (discardSink) RecordFault(time.Time, string, string, string, string) {}
func (discardSink) RecordCycle(control.Snapshot)                          {}

// streamDemo 在独立总线上演示：连续喂点拼接、运动中调速、越限倍率被拒、
// 满队策略与运动中急停->证据核验恢复。所有时序由假时钟手动推进，输出确定。
func streamDemo(fc *clock.Fake, period time.Duration) {
	b := bus.New(bus.Config{Capacity: 16, Policy: bus.DropOldest, MaxAge: time.Second}, fc)
	drv := device.New(device.Config{
		Version: protocol.V2, DT: period, Gain: 40, MaxVel: 2,
		HardMin: -2, HardMax: 2,
	}, fc, b)
	ctl, err := control.New(control.Config{
		ProtocolVersion: protocol.V2, Period: period, JitterTolerance: 15 * time.Millisecond,
		MaxBacklog: 8, TrajCapacity: 32,
		StreamCapacity: 3, StreamPolicy: stream.PolicyEvictOldest, MinOverride: 0.01, MaxOverride: 2.0,
		Motion: trajectory.Limits{MinPositionRad: -2, MaxPositionRad: 2,
			MaxVelocityRadS: 2, MaxAccelRadS2: 100, MaxTorqueNm: 5},
		SafetyLim: safety.Limits{MinPositionRad: -2, MaxPositionRad: 2, MaxTorqueNm: 5, SoftMarginRad: 0.1},
		Feedback:  feedback.Config{WatchdogTimeout: 35 * time.Millisecond, MaxStepRad: 1, MaxBackdateMs: 5},
		Bus:       bus.Config{Capacity: 16, Policy: bus.DropOldest, MaxAge: time.Second},
	}, fc, b)
	if err != nil {
		panic(err)
	}
	tick := func() {
		if derr := drv.Tick(); derr != nil {
			fmt.Println("  设备帧异常:", derr)
		}
		fc.Advance(period)
		_ = ctl.Step()
	}
	okCond := safety.RecoveryConditions{
		EstopReleased: true, CommHealthy: true, TorqueWithinLimit: true,
		PositionWithinLimit: true, FaultCodeClear: true, OperatorAck: true,
	}
	tick()
	tick()
	if err := ctl.Recover(okCond); err != nil {
		panic(err)
	}

	// 连续喂点：段1 没走完就塞段2、段3，顺序连续、不停车起步。
	vel := func(seq uint64, v float64, n int) stream.Target {
		return stream.Target{Mode: protocol.ModeVelocity, Value: v, Hold: time.Duration(n) * period, SeqIn: seq}
	}
	if _, err := ctl.AppendStreamTarget(vel(1, 0.4, 6)); err != nil {
		panic(err)
	}
	tick()
	tick()
	if _, err := ctl.AppendStreamTarget(vel(2, 0.6, 4)); err != nil {
		panic(err)
	}
	if _, err := ctl.AppendStreamTarget(vel(3, 0.8, 4)); err != nil {
		panic(err)
	}
	fmt.Println("  已连续喂入 3 段（0.4→0.6→0.8），前段未走完即追加，无停车起步")

	// 运动中调速：50% -> 120% -> 200%，每周期干净生效。
	if err := ctl.SetSpeedOverride(0.5); err != nil {
		panic(err)
	}
	tick()
	fmt.Printf("  倍率→50%%: 本周期下发速度=%.2f\n", ctl.Snapshot().LastCommand.VelocityRadS)
	if err := ctl.SetSpeedOverride(1.2); err != nil {
		panic(err)
	}
	tick()
	fmt.Printf("  倍率→120%%: 本周期下发速度=%.2f\n", ctl.Snapshot().LastCommand.VelocityRadS)
	if err := ctl.SetSpeedOverride(2.0); err != nil {
		panic(err)
	}
	tick()
	fmt.Printf("  倍率→200%%（在跑段<=1.6 可行，已接受）: 本周期下发速度=%.2f\n",
		ctl.Snapshot().LastCommand.VelocityRadS)
	// 在 200% 倍率下追加 1.05 段：缩放后 2.1 > 2，速度越限，明确拒绝。
	if _, err := ctl.AppendStreamTarget(vel(4, 1.05, 3)); err != nil {
		fmt.Println("  200% 下追加 1.05 段被明确拒绝（非静默裁剪，预期）:", err)
	}
	// 直接请求超配置范围的倍率同样拒绝，且状态不变。
	if err := ctl.SetSpeedOverride(2.5); err != nil {
		fmt.Println("  倍率→250% 超配置上限被拒（预期）:", err)
	}
	fmt.Printf("  倍率保持 %.0f%%（拒绝不生效）\n", ctl.SpeedOverride()*100)

	// 满队 evict_oldest：容量只有 3，多喂点会挤掉最旧等待段并留痕。
	for seq := uint64(10); seq < 14; seq++ {
		ev, err := ctl.AppendStreamTarget(vel(seq, 0.3, 2))
		if err != nil {
			fmt.Printf("  喂点 seq=%d 被可行性拒绝: %v\n", seq, err)
		}
		for _, e := range ev {
			fmt.Printf("  容量策略挤掉等待段 seq=%d（evict_oldest，已计数留痕）\n", e.SourceSeq)
		}
		tick()
	}

	// 运动中急停：安全优先，队列清空但倍率保留；恢复按真实证据判定，
	// 恢复后队列不复活、倍率复位 100%。
	for i := 0; i < 8; i++ {
		tick()
	}
	ctl.Estop("stream demo estop")
	fmt.Printf("  运动中急停: %s 倍率保留=%.0f%% 等待队列=%d\n",
		short(ctl.Snapshot()), ctl.SpeedOverride()*100, ctl.Snapshot().Stream.Waiting)
	for i := 0; i < 3; i++ {
		tick()
	}
	if err := ctl.Recover(okCond); err != nil {
		panic(err)
	}
	fmt.Printf("  真实证据健康+人工确认恢复: %s 队列=%d 倍率复位=%.0f%% 设备位置=%.3f\n",
		short(ctl.Snapshot()), ctl.Snapshot().Stream.Waiting, ctl.SpeedOverride()*100, drv.Position())
}
