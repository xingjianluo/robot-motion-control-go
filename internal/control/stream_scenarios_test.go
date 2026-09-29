package control_test

import (
	"sync"
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/bus"
	"github.com/xingjianluo/robot-motion-control-go/internal/clock"
	"github.com/xingjianluo/robot-motion-control-go/internal/control"
	"github.com/xingjianluo/robot-motion-control-go/internal/device"
	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
	"github.com/xingjianluo/robot-motion-control-go/internal/feedback"
	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
	"github.com/xingjianluo/robot-motion-control-go/internal/report"
	"github.com/xingjianluo/robot-motion-control-go/internal/safety"
	"github.com/xingjianluo/robot-motion-control-go/internal/stream"
	"github.com/xingjianluo/robot-motion-control-go/internal/trajectory"
)

// newStreamRig 构造带自定义流式容量/策略的测试台（设备/总线/时钟与
// newRig 完全一致，便于与既有用例横向对比）。
func newStreamRig(t *testing.T, capacity int, policy stream.OverflowPolicy) *rig {
	t.Helper()
	t0 := time.Date(2026, 9, 22, 0, 0, 0, 0, time.UTC)
	fc := clock.NewFake(t0)
	b := bus.New(bus.Config{Capacity: 64, Policy: bus.DropOldest, MaxAge: time.Second}, fc)
	drv := device.New(device.Config{
		Version: protocol.V2, DT: 10 * time.Millisecond, Gain: 30,
		MaxVel: 2, HardMin: -2, HardMax: 2,
	}, fc, b)
	cfg := control.Config{
		ProtocolVersion: protocol.V2, Period: 10 * time.Millisecond,
		JitterTolerance: 15 * time.Millisecond, MaxBacklog: 8, TrajCapacity: 32,
		StreamCapacity: capacity, StreamPolicy: policy, MinOverride: 0.01, MaxOverride: 2.0,
		Motion: trajectory.Limits{MinPositionRad: -2, MaxPositionRad: 2,
			MaxVelocityRadS: 2, MaxAccelRadS2: 100, MaxTorqueNm: 5},
		SafetyLim: safety.Limits{MinPositionRad: -2, MaxPositionRad: 2, MaxTorqueNm: 5, SoftMarginRad: 0.1},
		Feedback:  feedback.Config{WatchdogTimeout: 35 * time.Millisecond, MaxStepRad: 1.0, MaxBackdateMs: 5},
		Bus:       bus.Config{Capacity: 64, Policy: bus.DropOldest, MaxAge: time.Second},
	}
	ctl, err := control.New(cfg, fc, b)
	if err != nil {
		t.Fatal(err)
	}
	rec := report.NewRecorder(&testWriter{t: t})
	ctl.SetEventSink(rec)
	return &rig{t: t, fc: fc, bus: b, drv: drv, rec: rec, ctl: ctl, t0: t0}
}

// vTarget/uTarget/pTarget 构造流式目标（外部序号=cycle-ish，仅透传）。
func vTarget(seq uint64, v float64, hold time.Duration) stream.Target {
	return stream.Target{Mode: protocol.ModeVelocity, Value: v, Hold: hold, SeqIn: seq}
}
func pTarget(seq uint64, pos float64, hold time.Duration) stream.Target {
	return stream.Target{Mode: protocol.ModePosition, Value: pos, Hold: hold, SeqIn: seq}
}

// S1 流式拼接连续性：上一段未走完就追加下一段；下行命令每周期连续，
// 没有“到点停车再起步”的空周期；下发速度符合给定倍率。
func TestStreamS1ConcatenationContinuity(t *testing.T) {
	r := newStreamRig(t, 8, stream.PolicyRejectNew)
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	period := 10 * time.Millisecond
	if _, err := r.ctl.AppendStreamTarget(vTarget(1, 0.5, 4*period)); err != nil {
		t.Fatal(err)
	}
	// 走 2 周期（段1 剩 2 周期），随即追加段2（拼接差 0.3=>30/s 可行）。
	r.tick()
	r.tick()
	if _, err := r.ctl.AppendStreamTarget(vTarget(2, 0.8, 3*period)); err != nil {
		t.Fatal(err)
	}
	before := r.bus.Stats().DownEnqueued
	// 再走 6 周期：段1 剩 2 + 段2 的 3 = 至少 5 个连续下行命令。
	var velocities []float64
	for i := 0; i < 6; i++ {
		r.tick()
		if r.bus.Stats().DownEnqueued > before {
			before = r.bus.Stats().DownEnqueued
			velocities = append(velocities, r.ctl.Snapshot().LastCommand.VelocityRadS)
		}
	}
	if len(velocities) < 5 {
		t.Fatalf("expected continuous commands across splice, got %d", len(velocities))
	}
	// 拼接处：先 0.5 后 0.8，顺序确定。
	foundSplice := false
	for i := 1; i < len(velocities); i++ {
		if velocities[i-1] == 0.5 && velocities[i] == 0.8 {
			foundSplice = true
		}
	}
	if !foundSplice {
		t.Fatalf("splice 0.5->0.8 not observed in order: %v", velocities)
	}
	st := r.ctl.Snapshot().Stream
	t.Logf("输入: 段1 vel=0.5(4周期) 未走完追加 段2 vel=0.8(3周期)\n判定: 拼接处下行命令连续、无空周期，顺序 0.5->0.8\n%s",
		formatSnap(r.ctl.Snapshot()))
	_ = st
}

// S2 在线倍率对在跑段与排队段同时生效，且单周期干净。
func TestStreamS2OnlineOverrideAppliesAndIsCycleClean(t *testing.T) {
	r := newStreamRig(t, 8, stream.PolicyRejectNew)
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	period := 10 * time.Millisecond
	if _, err := r.ctl.AppendStreamTarget(vTarget(1, 1.0, 4*period)); err != nil {
		t.Fatal(err)
	}
	if _, err := r.ctl.AppendStreamTarget(vTarget(2, 1.0, 4*period)); err != nil {
		t.Fatal(err)
	}
	r.tick()
	// 100% 时下发速度=1.0
	if got := r.ctl.Snapshot().LastCommand.VelocityRadS; got != 1.0 {
		t.Fatalf("pre-override vel=%.3f", got)
	}
	if err := r.ctl.SetSpeedOverride(0.5); err != nil {
		t.Fatal(err)
	}
	// 切换后每个周期下发速度必须恒为 0.5，绝不出现一个周期内混合。
	for i := 0; i < 6; i++ {
		r.tick()
		cmd := r.ctl.Snapshot().LastCommand
		if cmd.Mode != protocol.ModeVelocity || cmd.VelocityRadS != 0.5 {
			t.Fatalf("cycle %d after override dirty: %+v", i, cmd)
		}
	}
	if ov := r.ctl.SpeedOverride(); ov != 0.5 {
		t.Fatalf("override=%.3f", ov)
	}
	t.Logf("输入: 两个 1.0 rad/s 段在跑/排队，运行中倍率 100%%->50%%\n判定: 切换后所有周期下发恒为 0.5，活动段按比例延长\n%s",
		formatSnap(r.ctl.Snapshot()))
}

// S3 会越限的倍率：明确拒绝、原因区别于普通越界、原子不生效。
func TestStreamS3InfeasibleOverrideRejected(t *testing.T) {
	r := newStreamRig(t, 8, stream.PolicyRejectNew)
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	period := 10 * time.Millisecond
	// 先建立温和前驱，再排队同速 1.05 段，使 200% 仅触发速度越限（拼接差0）。
	if _, err := r.ctl.AppendStreamTarget(vTarget(1, 0.5, 6*period)); err != nil {
		t.Fatal(err)
	}
	r.tick()
	r.tick()
	if _, err := r.ctl.AppendStreamTarget(vTarget(2, 1.05, 6*period)); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 5; i++ {
		r.tick()
	}
	if _, err := r.ctl.AppendStreamTarget(vTarget(3, 1.05, 6*period)); err != nil {
		t.Fatal(err)
	}
	err := r.ctl.SetSpeedOverride(2.0) // 2.1 > 2
	fe := mustReason(t, err, stream.ReasonTargetVelocityOutOfLimit)
	if fe.Source != fault.SourceStream {
		t.Fatalf("source=%s want stream", fe.Source)
	}
	if r.ctl.SpeedOverride() != 1.0 {
		t.Fatalf("rejected override must not apply: %.3f", r.ctl.SpeedOverride())
	}
	r.tick()
	if got := r.ctl.Snapshot().LastCommand.VelocityRadS; got != 1.05 {
		t.Fatalf("motion must continue unchanged at 100%%, got %.3f", got)
	}
	t.Logf("输入: 1.05 rad/s 段在跑+排队, 倍率->200%%\n判定依据: %v\n最终: 倍率仍=1.00，运动按原速继续", err)
}

// S4 排队到上限：reject_new 拒新（有记录、可观测、内存有界）。
func TestStreamS4CapacityRejectNew(t *testing.T) {
	r := newStreamRig(t, 3, stream.PolicyRejectNew)
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	period := 10 * time.Millisecond
	// 全部排满等待队列（不 tick，故都在等待）。
	for seq := uint64(1); seq <= 3; seq++ {
		if _, err := r.ctl.AppendStreamTarget(vTarget(seq, 0.1, 4*period)); err != nil {
			t.Fatal(err)
		}
	}
	_, err := r.ctl.AppendStreamTarget(vTarget(4, 0.1, 4*period))
	mustReason(t, err, stream.ReasonOverflowRejectNew)
	if st := r.ctl.Snapshot().Stream; st.Waiting != 3 || st.RejectedCount < 1 {
		t.Fatalf("queue unbounded or rejection uncounted: %+v", st)
	}
	t.Logf("输入: 容量3 已排满再塞第4点\n判定: %v\n最终: waiting=%d rejected=%d（内存不增长）",
		err, r.ctl.Snapshot().Stream.Waiting, r.ctl.Snapshot().Stream.RejectedCount)
}

// S5 排队到上限：evict_oldest 挤掉最旧等待段，逐次留痕，活动段不被挤。
func TestStreamS5CapacityEvictOldestObservable(t *testing.T) {
	r := newStreamRig(t, 2, stream.PolicyEvictOldest)
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	period := 10 * time.Millisecond
	// 用可区分的命令速度标记段：活动段1=0.10，被挤段2=0.20，
	// 段3=0.30，段4=0.40（同值排队差均 <=100/s：按 0.10 起拼）。
	mk := func(seq uint64, v float64, hold time.Duration) stream.Target {
		return stream.Target{Mode: protocol.ModeVelocity, Value: v, Hold: hold, SeqIn: seq}
	}
	if _, err := r.ctl.AppendStreamTarget(mk(1, 0.10, 2*period)); err != nil {
		t.Fatal(err)
	}
	// 段2 与段1 拼接差 0.10 => 10/s 可行。
	if _, err := r.ctl.AppendStreamTarget(mk(2, 0.20, 1*period)); err != nil {
		t.Fatal(err)
	}
	r.tick()                                                                   // 段1 激活；等待=[段2]
	if _, err := r.ctl.AppendStreamTarget(mk(3, 0.30, 1*period)); err != nil { // 差0.10 可行
		t.Fatal(err)
	}
	// 等待满=[2,3]，再塞 4 => 挤掉 2（最旧等待），活动段1绝不受影响。
	ev, err := r.ctl.AppendStreamTarget(mk(4, 0.40, 1*period)) // 与段3 差 0.10 可行
	if err != nil || len(ev) != 1 || ev[0].SourceSeq != 2 {
		t.Fatalf("expect eviction of waiting seq2, ev=%v err=%v", ev, err)
	}
	st := r.ctl.Snapshot().Stream
	if st.EvictedCount != 1 || !st.HasLastEviction || st.LastEviction.SourceSeq != 2 {
		t.Fatalf("eviction not observable: %+v", st)
	}
	// 逐周期统计驱动器实际生效的命令速度。被挤掉的段2(0.20)永不出现，
	// 其余 0.10/0.30/0.40 都必须出现。
	seen := map[float64]bool{}
	for i := 0; i < 6; i++ {
		r.tick()
		if c := r.ctl.Snapshot().LastCommand; c.Mode == protocol.ModeVelocity {
			seen[c.VelocityRadS] = true
		}
	}
	for _, v := range []float64{0.10, 0.30, 0.40} {
		if !seen[v] {
			t.Fatalf("segment velocity %.2f must execute; seen=%v", v, seen)
		}
	}
	if seen[0.20] {
		t.Fatalf("evicted segment (0.20) must never be dispatched; seen=%v", seen)
	}
	t.Logf("输入: 容量2 evict_oldest，活动段1在跑时连塞到段4\n判定: 挤掉等待最旧段 seq=2，活动段未被挤，EvictedCount=1\n最终实际下发速度集合=%v（无0.20）", seen)
}

// S6 运动+调速过程中触发安全停机：安全优先、队列清空、倍率保留；
// 恢复后队列不恢复、倍率复位 100%；恢复声明与真实证据矛盾会被拒。
func TestStreamS6SafetyStopPurgeAndEvidenceRecovery(t *testing.T) {
	r := newStreamRig(t, 4, stream.PolicyRejectNew)
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	period := 10 * time.Millisecond
	if _, err := r.ctl.AppendStreamTarget(vTarget(1, 0.5, 8*period)); err != nil {
		t.Fatal(err)
	}
	if _, err := r.ctl.AppendStreamTarget(vTarget(2, 0.5, 4*period)); err != nil {
		t.Fatal(err)
	}
	r.tick()
	if err := r.ctl.SetSpeedOverride(0.8); err != nil {
		t.Fatal(err)
	}
	downBefore := r.bus.Stats().DownEnqueued
	r.ctl.Estop("operator pressed estop during streaming")
	if st := r.ctl.Snapshot().Stream; st.Active || st.Waiting != 0 {
		t.Fatalf("queue must be purged on estop: %+v", st)
	}
	if r.ctl.SpeedOverride() != 0.8 {
		t.Fatalf("operator override must be retained across estop: %.3f", r.ctl.SpeedOverride())
	}
	// 急停后任何周期都不得再下发；喂点也必须被安全优先拒绝。
	for i := 0; i < 3; i++ {
		r.tick()
	}
	if r.bus.Stats().DownEnqueued != downBefore {
		t.Fatalf("no downlink command after estop: before=%d after=%d", downBefore, r.bus.Stats().DownEnqueued)
	}
	if _, err := r.ctl.AppendStreamTarget(vTarget(9, 0.1, period)); err == nil {
		t.Fatal("stream target in ESTOP must be rejected")
	} else {
		mustReason(t, err, "outputs_disabled")
	}
	// 恢复声明与真实状态矛盾（设备仍在发健康帧，CommHealthy 谎报为否
	// 不算矛盾；这里谎报故障码清零而真实有故障码）。
	// 先手工注入一帧带故障码的真实反馈。
	badLine := "2|STA|500|" + itoa64(r.fc.Now().UnixMilli()) + "|en=1;fault=7;pos=0;tau=0;vel=0"
	if err := r.bus.Post(bus.Uplink, badLine, 500); err != nil {
		t.Fatal(err)
	}
	r.fc.Advance(period)
	if err := r.ctl.Step(); err != nil {
		t.Fatal(err)
	}
	lie := fullRecover(true) // 声称 FaultCodeClear=true，但真实证据 fault=7
	err := r.ctl.Recover(lie)
	mustReason(t, err, "recovery_evidence_contradicted")
	// 让系统继续收到“真实健康帧”（序号必须高于故障帧 500，否则会被
	// 反馈消费者判为乱序而不更新基准）：手工投递 fault=0 的高序号帧。
	for seq := uint64(501); seq <= 503; seq++ {
		okLine := "2|STA|" + itoa64(int64(seq)) + "|" + itoa64(r.fc.Now().UnixMilli()) +
			"|en=1;fault=0;pos=0;tau=0;vel=0"
		if err := r.bus.Post(bus.Uplink, okLine, seq); err != nil {
			t.Fatal(err)
		}
		r.fc.Advance(period)
		if err := r.ctl.Step(); err != nil {
			t.Fatal(err)
		}
	}
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	st := r.ctl.Snapshot().Stream
	if st.Active || st.Waiting != 0 {
		t.Fatalf("purged stream must not resurrect after recovery: %+v", st)
	}
	if r.ctl.SpeedOverride() != 1.0 {
		t.Fatalf("override must reset to 100%% after explicit recovery: %.3f", r.ctl.SpeedOverride())
	}
	// 安全清空必须在故障流里可观测，而非静默丢点。
	foundPurge := false
	for _, f := range r.rec.Faults() {
		if f.Reason == "stream_motion_purged" {
			foundPurge = true
		}
	}
	if !foundPurge {
		t.Fatal("stream purge by safety must be recorded")
	}
	t.Logf("输入: 流式运动+0.8倍率中急停，随后谎报故障码清零再恢复\n判定: 急停即清空队列/保留倍率; 谎报被 recovery_evidence_contradicted 拒; 健康证据后恢复且倍率复位\n%s",
		formatSnap(r.ctl.Snapshot()))
}

// S7 并发：Step / AppendStreamTarget / SetSpeedOverride / Estop 竞争下，
// 最终必为 ESTOP、急停后无新增下行、无数据竞争。
func TestStreamS7ConcurrentSafetyWins(t *testing.T) {
	r := newStreamRig(t, 16, stream.PolicyRejectNew)
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	period := 10 * time.Millisecond
	var wg sync.WaitGroup
	wg.Add(4)
	go func() { // 喂点者：温和目标，多数会被接受
		defer wg.Done()
		for i := 0; i < 300; i++ {
			_, _ = r.ctl.AppendStreamTarget(vTarget(uint64(i), 0.2, 4*period))
		}
	}()
	go func() { // 调速者：在安全范围内来回调
		defer wg.Done()
		for i := 0; i < 300; i++ {
			if i%2 == 0 {
				_ = r.ctl.SetSpeedOverride(0.5)
			} else {
				_ = r.ctl.SetSpeedOverride(1.0)
			}
		}
	}()
	go func() { // 周期推进
		defer wg.Done()
		for i := 0; i < 300; i++ {
			r.fc.Advance(period)
			_ = r.ctl.Step()
		}
	}()
	go func() { // 急停
		defer wg.Done()
		time.Sleep(2 * time.Millisecond)
		r.ctl.Estop("concurrent stream estop")
	}()
	wg.Wait()
	if r.ctl.Snapshot().SafetyState != safety.StateEstop {
		t.Fatalf("final state must be ESTOP, got %s", r.ctl.Snapshot().SafetyState)
	}
	before := r.bus.Stats().DownEnqueued
	for i := 0; i < 5; i++ {
		r.fc.Advance(period)
		_ = r.ctl.Step()
	}
	if r.bus.Stats().DownEnqueued != before {
		t.Fatalf("downlink grew after estop: %d -> %d", before, r.bus.Stats().DownEnqueued)
	}
}

// S8 旧版单目标用法与流式通道互斥：原因码可区分；流走空后旧版恢复可用。
func TestStreamS8LegacyAndStreamMutuallyExclusive(t *testing.T) {
	r := newStreamRig(t, 8, stream.PolicyRejectNew)
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	period := 10 * time.Millisecond
	// 旧版命令先行且尚未排空时，流式目标被拒。
	if _, err := r.ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.2}); err != nil {
		t.Fatal(err)
	}
	_, err := r.ctl.AppendStreamTarget(vTarget(1, 0.1, period))
	mustReason(t, err, "legacy_trajectory_busy")
	r.tick() // 排空旧版航点
	// 启动流式后，旧版命令被拒。
	if _, err := r.ctl.AppendStreamTarget(vTarget(2, 0.1, 3*period)); err != nil {
		t.Fatal(err)
	}
	_, err = r.ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.3})
	mustReason(t, err, "stream_active")
	// 流式走空后旧版命令恢复可用（兼容性）。
	for i := 0; i < 4; i++ {
		r.tick()
	}
	if _, err := r.ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.3}); err != nil {
		t.Fatalf("legacy command must work after stream drains: %v", err)
	}
}

// S9 位置流式段：以真实反馈位置为起点逐周期插值，末点精确；
// 目标进软限位带外/超硬限位按流原因拒绝，与普通越界区分。
func TestStreamS9PositionSegmentAndLimits(t *testing.T) {
	r := newStreamRig(t, 8, stream.PolicyRejectNew)
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	period := 10 * time.Millisecond
	// 先两拍建立真实反馈位置基准（设备在 0 附近）。
	r.tick()
	r.tick()
	// 0 -> 0.05 用 5 周期 => 1 rad/s 可行。
	if _, err := r.ctl.AppendStreamTarget(pTarget(1, 0.05, 5*period)); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 7; i++ { // 多跑两拍吸收“设备先积分、控制器后下发”的一拍延迟
		r.tick()
	}
	if c := r.ctl.Snapshot().LastCommand; c.Mode != protocol.ModePosition || c.PositionRad != 0.05 {
		t.Fatalf("final streamed position target must be exactly 0.05: %+v", c)
	}
	if pos := r.drv.Position(); pos < 0.035 {
		t.Fatalf("drive should follow interpolated position stream, pos=%.4f", pos)
	}
	// 超硬限位
	_, err := r.ctl.AppendStreamTarget(pTarget(2, 3.0, period))
	mustReason(t, err, stream.ReasonTargetPositionOutOfLimit)
	// 软限位带外（硬限位内）
	_, err = r.ctl.AppendStreamTarget(pTarget(3, 1.95, period))
	mustReason(t, err, stream.ReasonTargetSoftLimit)
	// 插值步长超速（0->0.5 仅 1 周期 => 50 rad/s）
	_, err = r.ctl.AppendStreamTarget(pTarget(4, 0.5, period))
	mustReason(t, err, stream.ReasonInterpStep)
	t.Logf("位置段跟踪: 设备位置=%.4f (期望≈0.05)；三类位置拒绝原因均可区分", r.drv.Position())
}
