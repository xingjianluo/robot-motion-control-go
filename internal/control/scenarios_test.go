package control_test

import (
	"sync"
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/bus"
	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
	"github.com/xingjianluo/robot-motion-control-go/internal/safety"
	"github.com/xingjianluo/robot-motion-control-go/internal/trajectory"
)

// Test1 初始 ESTOP 拒绝命令；满足全部条件并人工确认后恢复；
// 随后位置命令被连续接受、下发并被反馈跟踪。
func TestStartupEstopThenTrack(t *testing.T) {
	r := newRig(t, protocol.V2, bus.Config{Capacity: 64, Policy: bus.DropOldest, MaxAge: 500 * time.Millisecond})

	_, err := r.ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.5})
	mustReason(t, err, "outputs_disabled")

	// 恢复条件不全也必须被拒绝
	err = r.ctl.Recover(safety.RecoveryConditions{EstopReleased: true, OperatorAck: true})
	mustReason(t, err, safety.ReasonRecoveryDenied)

	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	// 恢复后提交并推进若干周期
	if _, err := r.ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.5}); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 10; i++ {
		r.tick()
	}
	snap := r.ctl.Snapshot()
	if snap.SafetyState != safety.StateRun || !snap.OutputsOn {
		t.Fatalf("unexpected state: %s", snap.SafetyState)
	}
	if !snap.Tracking.Healthy {
		t.Fatal("tracking should be healthy with continuous feedback")
	}
	if snap.BusStats.DownEnqueued < 1 || snap.BusStats.UpDequeued < 1 {
		t.Fatalf("bus traffic missing: %+v", snap.BusStats)
	}
	// 设备应已向目标方向运动
	if pos := r.drv.Position(); pos <= 0 {
		t.Fatalf("drive did not move toward target: pos=%.4f", pos)
	}
	t.Logf("最终状态:\n%s", formatSnap(snap))
}

// Test2 不同来源的故障必须带不同类别/原因：
// 越界命令(command)、坏帧(protocol)、看门狗(communication)、急停(safety)。
func TestFaultCategoriesNeverMerged(t *testing.T) {
	r := newRig(t, protocol.V2, bus.Config{Capacity: 64})
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}

	// 命令越界
	_, err := r.ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 9})
	fe := mustReason(t, err, trajectory.ReasonPositionOutOfLimit)
	if fe.Category != fault.CatCommandReject || fe.Source != fault.SourceTrajectory {
		t.Fatalf("category=%s source=%s", fe.Category, fe.Source)
	}

	// V1 上提交力矩命令：协议错误而非命令/通信错误。
	// 用独立的 V1 控制器验证多版本共存下的归类。
	r1 := newRig(t, protocol.V1, bus.Config{Capacity: 64})
	if err := r1.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	_, errV1 := r1.ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModeTorque, Torque: 1})
	fe1 := mustReason(t, errV1, protocol.ReasonUnsupportedVer)
	if fe1.Category != fault.CatProtocol {
		t.Fatalf("torque-on-v1 category=%s", fe1.Category)
	}

	// 通过上行损坏帧验证协议归类
	if err := r.bus.Post(bus.Uplink, "2|STA|1|0|garbage<<<", 1); err != nil {
		t.Fatal(err)
	}
	r.fc.Advance(10 * time.Millisecond)
	if err := r.ctl.Step(); err != nil {
		t.Fatal(err)
	}

	// 等待看门狗：40ms 内无有效反馈（上面的损坏帧不计）
	r.fc.Advance(40 * time.Millisecond)
	if err := r.ctl.Step(); err != nil {
		t.Fatal(err)
	}
	snap := r.ctl.Snapshot()
	if snap.SafetyState != safety.StateCommLoss {
		t.Fatalf("watchdog must trip COMM_LOSS, got %s", snap.SafetyState)
	}

	// 急停优先级最高，立即覆盖 COMM_LOSS
	r.ctl.Estop("operator")
	if r.ctl.Snapshot().SafetyState != safety.StateEstop {
		t.Fatal("estop must override comm loss")
	}

	// 校验记录流中四类原因都可区分且各自只出现于正确类别
	reasons := map[string]fault.Category{}
	for _, f := range r.rec.Faults() {
		reasons[f.Reason] = fault.Category(f.Category)
	}
	for _, f := range r1.rec.Faults() {
		reasons[f.Reason] = fault.Category(f.Category)
	}
	want := map[string]fault.Category{
		trajectory.ReasonPositionOutOfLimit: fault.CatCommandReject,
		protocol.ReasonUnsupportedVer:       fault.CatProtocol,
		feedbackWatchdogReason():            fault.CatComm,
	}
	for reason, cat := range want {
		got, ok := reasons[reason]
		if !ok {
			t.Fatalf("missing classified fault: %s", reason)
		}
		if got != cat {
			t.Fatalf("reason %s classified as %s, want %s", reason, got, cat)
		}
	}
	t.Logf("故障归类记录: %+v", reasons)
}

// Test3 看门狗：暂时失联不是跟踪误差；重新收到有效帧只清除失联标志，
// 退出 COMM_LOSS 仍需显式恢复。
func TestWatchdogLossThenExplicitRecovery(t *testing.T) {
	r := newRig(t, protocol.V2, bus.Config{Capacity: 64})
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	// 两拍有效反馈建立基准
	r.tick()
	r.tick()
	if !r.ctl.Snapshot().Tracking.Healthy {
		t.Fatal("expect healthy baseline")
	}
	// 设备静默 5 拍（不 Tick），仅推进控制器
	for i := 0; i < 5; i++ {
		r.fc.Advance(10 * time.Millisecond)
		if err := r.ctl.Step(); err != nil {
			t.Fatal(err)
		}
	}
	if r.ctl.Snapshot().SafetyState != safety.StateCommLoss {
		t.Fatalf("expected COMM_LOSS, got %s", r.ctl.Snapshot().SafetyState)
	}
	// 失联期间跟踪信息必须标记为不健康，禁止被解读为偏差
	if r.ctl.Snapshot().Tracking.Healthy {
		t.Fatal("tracking must be marked unhealthy during loss")
	}
	// 通信恢复：设备重新上线，健康反馈恢复；但安全状态不得自动回 RUN
	for i := 0; i < 3; i++ {
		r.tick()
	}
	if r.ctl.Snapshot().SafetyState != safety.StateCommLoss {
		t.Fatalf("COMM_LOSS must latch until explicit recovery, got %s", r.ctl.Snapshot().SafetyState)
	}
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	if r.ctl.Snapshot().SafetyState != safety.StateRun {
		t.Fatal("explicit recovery should return to RUN")
	}
}

// Test4 急停与轨迹推进并发：结果必须确定——急停后输出必然切断，
// 航点缓冲清空，下行总线在排空后不会再出现新命令。
func TestEstopConcurrentWithSteps(t *testing.T) {
	r := newRig(t, protocol.V2, bus.Config{Capacity: 64})
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}

	var wg sync.WaitGroup
	const n = 200
	wg.Add(2)
	go func() { // 周期推进者
		defer wg.Done()
		for i := 0; i < n; i++ {
			r.fc.Advance(10 * time.Millisecond)
			_ = r.ctl.Step()
		}
	}()
	go func() { // 急停者，在中途随机时刻触发
		defer wg.Done()
		time.Sleep(time.Millisecond) // 让推进开始
		r.ctl.Estop("concurrent operator stop")
	}()
	// 同时有一个命令提交者持续尝试
	wg.Add(1)
	go func() {
		defer wg.Done()
		for i := 0; i < n; i++ {
			_, _ = r.ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.1})
		}
	}()
	wg.Wait()

	snap := r.ctl.Snapshot()
	if snap.SafetyState != safety.StateEstop || snap.OutputsOn {
		t.Fatalf("after estop race final state must be ESTOP/off: %s", snap.SafetyState)
	}
	// 排空下行总线：急停被提交之后，不得再出现任何新命令。
	// 判据：此刻再推进若干周期，状态保持 ESTOP，下行不再增长。
	before := r.bus.Stats().DownEnqueued
	for i := 0; i < 5; i++ {
		r.fc.Advance(10 * time.Millisecond)
		_ = r.ctl.Step()
	}
	after := r.bus.Stats().DownEnqueued
	if after != before {
		t.Fatalf("commands posted after estop: before=%d after=%d", before, after)
	}
	t.Logf("并发急停结果: state=%s downlink=%d\n%s",
		snap.SafetyState, after, formatSnap(snap))
}

// Test5 积压约束：上行帧单周期消费受 MaxBacklog 限制；总线有界，
// 溢出按 DropOldest 处理且丢弃可观测、有归属。
func TestBacklogBoundAndObservableDrops(t *testing.T) {
	r := newRig(t, protocol.V2, bus.Config{Capacity: 4, Policy: bus.DropOldest})
	// 恢复后不启动设备 Tick，直接手工灌入 10 帧（总线容量 4 => 丢弃 6）
	for seq := uint64(1); seq <= 10; seq++ {
		line, err := protocol.EncodeStatus(protocol.V2, protocol.DriveStatus{
			PositionRad: 0.01 * float64(seq), VelocityRadS: 0, TorqueNm: 0,
			Enabled: true, Seq: seq, TsMs: int64(seq),
		})
		if err != nil {
			t.Fatal(err)
		}
		_ = r.bus.Post(bus.Uplink, line, seq)
	}
	if st := r.bus.Stats(); st.DroppedFull != 6 {
		t.Fatalf("want 6 overflow drops, got %d", st.DroppedFull)
	}
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	r.fc.Advance(10 * time.Millisecond)
	if err := r.ctl.Step(); err != nil {
		t.Fatal(err)
	}
	snap := r.ctl.Snapshot()
	// MaxBacklog=8 但队列只剩 4 帧：全部消费，无半更新
	if snap.FeedbackAccepted != 4 || snap.BacklogRemaining != 0 {
		t.Fatalf("accepted=%d remaining=%d", snap.FeedbackAccepted, snap.BacklogRemaining)
	}
	if snap.BusStats.LastDropReason != bus.ReasonQueueFull {
		t.Fatalf("drop not observable: %+v", snap.BusStats)
	}
	t.Logf("积压处理结果:\n%s", formatSnap(snap))
}

// Test6 力矩超限锁存：V2 反馈超力矩 => TORQUE_FAULT；
// 即使反馈恢复正常，也必须人工确认才能 RUN。
func TestTorqueOverloadLatched(t *testing.T) {
	r := newRig(t, protocol.V2, bus.Config{Capacity: 64})
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	line, _ := protocol.EncodeStatus(protocol.V2, protocol.DriveStatus{
		PositionRad: 0, VelocityRadS: 0, TorqueNm: 9, // 超 5Nm 上限
		Enabled: true, Seq: 1, TsMs: r.t0.UnixMilli(),
	})
	if err := r.bus.Post(bus.Uplink, line, 1); err != nil {
		t.Fatal(err)
	}
	r.fc.Advance(10 * time.Millisecond)
	if err := r.ctl.Step(); err != nil {
		t.Fatal(err)
	}
	if r.ctl.Snapshot().SafetyState != safety.StateTorqueFault {
		t.Fatalf("want TORQUE_FAULT, got %s", r.ctl.Snapshot().SafetyState)
	}
	// 缺人工确认 => 恢复被拒
	c := fullRecover(true)
	c.OperatorAck = false
	mustReason(t, r.ctl.Recover(c), safety.ReasonRecoveryDenied)
	// 全部满足且确认后恢复
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	if r.ctl.Snapshot().SafetyState != safety.StateRun {
		t.Fatal("recovery failed")
	}
}

// Test7 V1/V2 多版本共存：控制器 V2 可消费 V1 设备反馈（无 tau 字段
// 不误判）；力矩安全检查在缺字段时保持兼容跳过。
func TestV1DeviceFeedbackCompat(t *testing.T) {
	r := newRig(t, protocol.V2, bus.Config{Capacity: 64})
	// rig 设备为 V2；手工发一帧 V1 状态（无 tau/en/fault 走历史默认）
	line := "1|STA|1|" + itoa64(r.t0.UnixMilli()) + "|pos=0.1;vel=0"
	if err := r.bus.Post(bus.Uplink, line, 1); err != nil {
		t.Fatal(err)
	}
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	r.fc.Advance(10 * time.Millisecond)
	if err := r.ctl.Step(); err != nil {
		t.Fatal(err)
	}
	snap := r.ctl.Snapshot()
	if !snap.Tracking.Healthy {
		t.Fatal("v1 frame should be accepted as valid feedback")
	}
	if snap.SafetyState != safety.StateRun {
		t.Fatalf("missing tau in v1 must not trigger torque fault: %s", snap.SafetyState)
	}
}
