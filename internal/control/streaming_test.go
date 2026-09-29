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

// newStreamingRig 构造一个开启“真实状态交叉核验恢复”的流式测试台，
// 流式排队策略/容量可配。
func newStreamingRig(t *testing.T, ver protocol.Version, policy stream.FullPolicy, capacity int) *rig {
	t.Helper()
	t0 := time.Date(2026, 9, 29, 0, 0, 0, 0, time.UTC)
	fc := clock.NewFake(t0)
	b := bus.New(bus.Config{Capacity: 64, Policy: bus.DropOldest, MaxAge: 500 * time.Millisecond}, fc)
	drv := device.New(device.Config{
		Version: ver, DT: 10 * time.Millisecond, Gain: 40, MaxVel: 2, HardMin: -2, HardMax: 2,
		PosCmdFeedforward: true, PosFFGain: 40,
	}, fc, b)
	cfg := control.Config{
		ProtocolVersion: ver, Period: 10 * time.Millisecond,
		JitterTolerance: 15 * time.Millisecond, MaxBacklog: 8, TrajCapacity: 32,
		Motion: trajectory.Limits{
			MinPositionRad: -2, MaxPositionRad: 2,
			MaxVelocityRadS: 2, MaxAccelRadS2: 100, MaxTorqueNm: 5,
		},
		SafetyLim: safety.Limits{
			MinPositionRad: -2, MaxPositionRad: 2, MaxTorqueNm: 5, SoftMarginRad: 0.1,
		},
		Feedback: feedback.Config{
			WatchdogTimeout: 35 * time.Millisecond, MaxStepRad: 1.0, MaxBackdateMs: 5,
		},
		Bus: bus.Config{Capacity: 64, Policy: bus.DropOldest, MaxAge: 500 * time.Millisecond},
		Stream: control.StreamConfig{
			Capacity: capacity, FullPolicy: policy, MinOverride: 0, MaxOverride: 2,
		},
		RequireObservedRecovery: true,
	}
	ctl, err := control.New(cfg, fc, b)
	if err != nil {
		t.Fatal(err)
	}
	rec := report.NewRecorder(&testWriter{t: t})
	ctl.SetEventSink(rec)
	return &rig{t: t, fc: fc, bus: b, drv: drv, rec: rec, ctl: ctl, t0: t0}
}

// feedStreamPoint 在当前时刻 +delay 处喂一个流式点。
func (r *rig) feedStreamPoint(pos float64, delay time.Duration) error {
	return r.ctl.SubmitStreamPoint(stream.Point{
		At:          r.fc.Now().Add(delay),
		PositionRad: pos,
	})
}

// TestStreamA 连续流式喂点端到端：上一段没走完就续点，一路连续走完，
// 不在每点停顿；最终到达最后一点，全程下行命令连续、跟踪健康，
// 输入/判定/最终状态全部打印。
func TestStreamEndToEndContinuousSplice(t *testing.T) {
	r := newStreamingRig(t, protocol.V2, stream.RejectNew, 32)
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	// 先两拍建立反馈基准（起始位置 0）。
	r.tick()
	r.tick()

	// 一次把三个点都塞进去（各 0.2s，段速 0.5/1.0/0.5 rad/s）。
	if err := r.feedStreamPoint(0.1, 200*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	if err := r.feedStreamPoint(0.3, 400*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	if err := r.feedStreamPoint(0.2, 600*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	if r.ctl.StreamQueueLen() != 3 {
		t.Fatalf("queued=%d want 3", r.ctl.StreamQueueLen())
	}

	before := r.bus.Stats().DownEnqueued
	idleCycles := 0
	for i := 0; i < 90; i++ {
		r.tick()
		if r.ctl.Snapshot().StreamActive && r.bus.Stats().DownEnqueued == before {
			idleCycles++
		}
		before = r.bus.Stats().DownEnqueued
	}
	snap := r.ctl.Snapshot()
	if snap.StreamStats.Reached != 3 {
		t.Fatalf("reached=%d want 3 (stats=%+v)", snap.StreamStats.Reached, snap.StreamStats)
	}
	if !snap.Tracking.Healthy {
		t.Fatal("tracking must stay healthy across streaming splices")
	}
	if pos := r.drv.Position(); pos < 0.19 || pos > 0.21 {
		t.Fatalf("drive final pos %.4f not near last point 0.2", pos)
	}
	if idleCycles != 0 {
		t.Fatalf("stream active but no downlink cmd in %d cycles (would imply stop/restart gaps)", idleCycles)
	}
	t.Logf("输入: 3 个流式点(0.1@.2s,0.3@.4s,0.2@.6s) 一次喂入\n判定: 全部接受, reached=%d\n最终状态:\n%s\n设备位置=%.4f",
		snap.StreamStats.Reached, formatSnap(snap), r.drv.Position())
}

// TestStreamB 在线倍率：50%->120%->0->100% 全程不越限；越界倍率与
// 会顶破限速的倍率分别以 override_out_of_range / override_infeasible
// 拒绝，绝不静默裁剪；非 RUN 下调倍率以 outputs_disabled 拒绝。
func TestStreamOnlineOverrideAndLimits(t *testing.T) {
	r := newStreamingRig(t, protocol.V2, stream.RejectNew, 32)
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	r.tick()
	r.tick()

	// 段速 1.0 rad/s（0.3rad/0.3s）；倍率 2 => 2.0 恰好不超，
	// 用 0.31/0.3 段构造 1.0333... 使倍率 2 必然超速。
	if err := r.feedStreamPoint(0.31, 300*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	mustReason(t, r.ctl.SetSpeedOverride(3), stream.ReasonOverrideOutOfRange)
	mustReason(t, r.ctl.SetSpeedOverride(2), stream.ReasonOverrideInfeasible)

	if err := r.ctl.SetSpeedOverride(0.5); err != nil {
		t.Fatal(err)
	}
	maxVel := 0.0
	for i := 0; i < 120; i++ {
		switch i {
		case 20:
			if err := r.ctl.SetSpeedOverride(1.2); err != nil {
				t.Fatal(err)
			}
		case 40:
			if err := r.ctl.SetSpeedOverride(0); err != nil {
				t.Fatal(err)
			}
		case 55:
			if err := r.ctl.SetSpeedOverride(1); err != nil {
				t.Fatal(err)
			}
		}
		r.tick()
		if v := r.drvVel(); v > maxVel {
			maxVel = v
		}
	}
	if maxVel > 2+1e-6 {
		t.Fatalf("drive velocity exceeded hard max during override changes: %.4f", maxVel)
	}
	if pos := r.drv.Position(); pos < 0.30 || pos > 0.32 {
		t.Fatalf("final pos %.4f not near 0.31", pos)
	}

	// 非 RUN 下调倍率：安全优先，outputs_disabled。
	r.ctl.Estop("override while estop")
	fe := mustReason(t, r.ctl.SetSpeedOverride(1.2), control.ReasonOutputsDisabled)
	if fe.Category != fault.CatSafety {
		t.Fatalf("want safety category, got %s", fe.Category)
	}
	t.Logf("在线倍率结果: maxDriveVel=%.4f finalPos=%.4f\n%s",
		maxVel, r.drv.Position(), formatSnap(r.ctl.Snapshot()))
}

// TestStreamC 排队到上限：reject_new 明确拒绝（内存有界）；
// evict_oldest 挤掉最旧排队点、活动点保留，挤掉动作进入可观测记录。
func TestStreamQueueAtCapacityPolicies(t *testing.T) {
	// reject_new
	r := newStreamingRig(t, protocol.V2, stream.RejectNew, 3)
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	r.tick()
	r.tick()
	for i := 1; i <= 3; i++ {
		if err := r.feedStreamPoint(0.05*float64(i), time.Duration(i)*200*time.Millisecond); err != nil {
			t.Fatal(err)
		}
	}
	mustReason(t, r.feedStreamPoint(0.2, 800*time.Millisecond), stream.ReasonQueueFull)

	// evict_oldest
	r2 := newStreamingRig(t, protocol.V2, stream.EvictOldest, 3)
	if err := r2.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	r2.tick()
	r2.tick()
	for i := 1; i <= 3; i++ {
		if err := r2.feedStreamPoint(0.05*float64(i), time.Duration(i)*200*time.Millisecond); err != nil {
			t.Fatal(err)
		}
	}
	if err := r2.feedStreamPoint(0.2, 800*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	if n := r2.ctl.StreamQueueLen(); n != 3 {
		t.Fatalf("queue must stay bounded, len=%d", n)
	}
	if st := r2.ctl.Snapshot().StreamStats; st.Evicted != 1 {
		t.Fatalf("want 1 observable eviction, got %+v", st)
	}
	// 被挤掉后第一个活动点（0.05）仍必须走到：推进 3 拍观察下行不中断。
	downBefore := r2.bus.Stats().DownEnqueued
	for i := 0; i < 3; i++ {
		r2.tick()
	}
	if r2.bus.Stats().DownEnqueued <= downBefore {
		t.Fatal("active point motion must continue after eviction")
	}
	t.Logf("容量策略结果(evict): %+v", r2.ctl.Snapshot().StreamStats)
}

// TestStreamD 运动/调速过程中触发安全停机：安全状态优先、结果确定；
// 未走完的流式队列与倍率被清空（不保留、且有记录）；恢复必须以系统
// 真实观测为准，口头声称健康不被接受；恢复后从干净状态重新喂点。
func TestStreamSafetyStopAndObservedRecovery(t *testing.T) {
	r := newStreamingRig(t, protocol.V2, stream.RejectNew, 16)
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 3; i++ {
		r.tick()
	}
	if err := r.feedStreamPoint(0.2, 300*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	if err := r.ctl.SetSpeedOverride(0.5); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 5; i++ {
		r.tick()
	}

	// 运动中触发看门狗 COMM_LOSS：停发设备帧 5 拍。
	for i := 0; i < 5; i++ {
		r.fc.Advance(10 * time.Millisecond)
		if err := r.ctl.Step(); err != nil {
			t.Fatal(err)
		}
	}
	if r.ctl.Snapshot().SafetyState != safety.StateCommLoss {
		t.Fatalf("want COMM_LOSS, got %s", r.ctl.Snapshot().SafetyState)
	}
	if r.ctl.StreamQueueLen() != 0 {
		t.Fatalf("stream queue must be discarded on safety stop, len=%d", r.ctl.StreamQueueLen())
	}
	if r.ctl.SpeedOverride() != 1.0 {
		t.Fatalf("override must reset to 1.0 on safety stop, got %.2f", r.ctl.SpeedOverride())
	}
	downBefore := r.bus.Stats().DownEnqueued

	// 链路尚未真正恢复，口头声称健康 => 必须拒绝（不能光信声明）。
	lie := fullRecover(true)
	mustReason(t, r.ctl.Recover(lie), control.ReasonRecoveryObservedMismatch)

	// COMM_LOSS 下喂点/调速都必须被安全门控拒绝。
	mustReason(t, r.feedStreamPoint(0.1, 200*time.Millisecond), control.ReasonOutputsDisabled)
	mustReason(t, r.ctl.SetSpeedOverride(1.2), control.ReasonOutputsDisabled)

	// 真正恢复链路：设备重新上线若干拍，看门狗观察到健康帧。
	for i := 0; i < 6; i++ {
		r.tick()
	}
	// 此时真实观测健康；显式恢复应被接受。
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatalf("recovery with healthy observation must succeed: %v", err)
	}
	if r.ctl.Snapshot().SafetyState != safety.StateRun {
		t.Fatal("state must be RUN after observed recovery")
	}
	// 恢复后队列空、倍率 1.0，从干净状态重新喂点可正常运动。
	if r.ctl.StreamQueueLen() != 0 || r.ctl.SpeedOverride() != 1.0 {
		t.Fatal("post-recovery stream state must be clean")
	}
	if err := r.feedStreamPoint(0.1, 200*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	downAt := r.bus.Stats().DownEnqueued
	for i := 0; i < 25; i++ {
		r.tick()
	}
	if r.bus.Stats().DownEnqueued <= downAt || r.bus.Stats().DownEnqueued <= downBefore {
		t.Fatal("stream must resume output after recovery")
	}
	t.Logf("安全停机/观测恢复结果:\n%s", formatSnap(r.ctl.Snapshot()))
}

// TestStreamE 并发：周期推进、流式喂点、在线倍率、急停同时竞争，
// race 检测下最终状态必须确定为 ESTOP 且急停后不再下发流式命令。
func TestStreamConcurrentFeedsOverrideEstop(t *testing.T) {
	r := newStreamingRig(t, protocol.V2, stream.EvictOldest, 64)
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	r.tick()
	r.tick()

	var wg sync.WaitGroup
	const n = 200
	wg.Add(4)
	go func() { // 周期
		defer wg.Done()
		for i := 0; i < n; i++ {
			r.fc.Advance(10 * time.Millisecond)
			_ = r.ctl.Step()
		}
	}()
	go func() { // 流式喂点（时间持续向后）
		defer wg.Done()
		base := r.fc.Now()
		for i := 0; i < n; i++ {
			_ = r.ctl.SubmitStreamPoint(stream.Point{
				At:          base.Add(time.Duration(i+2) * 100 * time.Millisecond),
				PositionRad: 0.05,
			})
		}
	}()
	go func() { // 在线倍率反复调整
		defer wg.Done()
		for i := 0; i < n; i++ {
			f := 1.0
			if i%2 == 0 {
				f = 0.5
			}
			_ = r.ctl.SetSpeedOverride(f)
		}
	}()
	go func() { // 中途急停
		defer wg.Done()
		time.Sleep(2 * time.Millisecond)
		r.ctl.Estop("concurrent stream stop")
	}()
	wg.Wait()

	snap := r.ctl.Snapshot()
	if snap.SafetyState != safety.StateEstop || snap.OutputsOn {
		t.Fatalf("final state must be ESTOP/off, got %s", snap.SafetyState)
	}
	if r.ctl.StreamQueueLen() != 0 || r.ctl.SpeedOverride() != 1.0 {
		t.Fatalf("estop must clear queue and reset override: len=%d factor=%.2f",
			r.ctl.StreamQueueLen(), r.ctl.SpeedOverride())
	}
	before := r.bus.Stats().DownEnqueued
	for i := 0; i < 5; i++ {
		r.fc.Advance(10 * time.Millisecond)
		_ = r.ctl.Step()
	}
	if after := r.bus.Stats().DownEnqueued; after != before {
		t.Fatalf("downlink grew after estop: before=%d after=%d", before, after)
	}
	t.Logf("并发流式结果: state=%s evicted=%d rejected=%d downlink=%d",
		snap.SafetyState, snap.StreamStats.Evicted,
		snap.StreamStats.Rejected+snap.StreamStats.OverrideRejects, before)
}
