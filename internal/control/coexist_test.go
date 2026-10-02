package control_test

import (
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
	"github.com/xingjianluo/robot-motion-control-go/internal/safety"
	"github.com/xingjianluo/robot-motion-control-go/internal/stream"
	"github.com/xingjianluo/robot-motion-control-go/internal/trajectory"
)

// runStreamHolding 喂一个流点并推进到排空后的“末点保持”，
// 返回进入保持时的下行命令计数。
func runStreamHolding(r *rig, pos float64, in, ticks time.Duration) int64 {
	if err := r.feedStreamPoint(pos, in); err != nil {
		r.t.Fatal(err)
	}
	n := int(in/r.period()) + 20
	for i := 0; i < n; i++ {
		r.tick()
	}
	if !r.ctl.Snapshot().StreamActive || r.ctl.StreamQueueLen() != 0 {
		r.t.Fatalf("want drained-and-holding, active=%v queued=%d",
			r.ctl.Snapshot().StreamActive, r.ctl.StreamQueueLen())
	}
	return r.bus.Stats().DownEnqueued
}

func (r *rig) period() time.Duration { return 10 * time.Millisecond }

// TestCoexistSingleTargetWorksAfterStream 流式跑完、机器人停在末点保持后，
// 原来的单目标下发必须照常生效；两种用法在同一会话里来回切换互不堵死。
func TestCoexistSingleTargetWorksAfterStream(t *testing.T) {
	r := newStreamingRig(t, protocol.V2, stream.RejectNew, 32)
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 3; i++ {
		r.tick()
	}

	// 阶段一：流式 0 -> 0.2，跑完进入末点保持。
	runStreamHolding(r, 0.2, 300*time.Millisecond, 600*time.Millisecond)
	holdPos := r.drv.Position()
	if holdPos < 0.19 || holdPos > 0.21 {
		t.Fatalf("stream end pos %.4f want ~0.2", holdPos)
	}

	// 阶段二：保持期间直接发单目标。回归前这里返回成功但机器人不动——
	// Planner 一直 holding 并在 Step 中接管下发，traj 航点永远发不出去。
	beforeDown := r.bus.Stats().DownEnqueued
	if _, err := r.ctl.SubmitCommand(trajectory.Command{
		Mode: protocol.ModePosition, Position: holdPos,
	}); err != nil {
		t.Fatalf("single-target after drained stream must be accepted: %v", err)
	}
	for i := 0; i < 3; i++ {
		r.tick()
	}
	if got := r.bus.Stats().DownEnqueued - beforeDown; got == 0 {
		t.Fatal("single-target command must actually be sent after drained stream, got 0 downlink cmds")
	}
	if snap := r.ctl.Snapshot(); snap.StreamActive {
		t.Fatalf("single-target acceptance must release stream path, snapshot:\n%s", formatSnap(snap))
	}
	if r.ctl.SpeedOverride() != 1.0 {
		t.Fatalf("override must reset to 1.0 after switching to single-target, got %.2f", r.ctl.SpeedOverride())
	}
	t.Logf("输入: 流末点=%.3f 保持后发单目标位置=%.3f\n判定: 接受(stream_released_by_command) 3 拍内下行新增=%d\n最终状态:\n%s",
		holdPos, holdPos, r.bus.Stats().DownEnqueued-beforeDown, formatSnap(r.ctl.Snapshot()))

	// 阶段三：单目标再跑一会儿（速度模式），命令继续真实下发。
	beforeDown = r.bus.Stats().DownEnqueued
	posBefore := r.drv.Position()
	for i := 0; i < 20; i++ {
		if _, err := r.ctl.SubmitCommand(trajectory.Command{
			Mode: protocol.ModeVelocity, Velocity: -0.5, IssuedAt: r.fc.Now(),
		}); err != nil {
			t.Fatal(err)
		}
		r.tick()
	}
	if r.bus.Stats().DownEnqueued <= beforeDown {
		t.Fatal("velocity single-target commands must keep going out after stream usage")
	}
	if r.drv.Position() >= posBefore {
		t.Fatalf("negative velocity must move device: before=%.4f after=%.4f", posBefore, r.drv.Position())
	}

	// 阶段四：再切回流式：新流必须从当前实际状态正常起步、正常走到。
	target := 0.1
	if err := r.feedStreamPoint(target, 300*time.Millisecond); err != nil {
		t.Fatalf("resuming stream after single-target usage must work: %v", err)
	}
	for i := 0; i < 60; i++ {
		r.tick()
	}
	if pos := r.drv.Position(); pos < target-0.02 || pos > target+0.02 {
		t.Fatalf("re-started stream must reach %.2f, got %.4f", target, pos)
	}
	t.Logf("切换回流式结果: 目标=%.2f 设备位置=%.4f reached=%d\n%s",
		target, r.drv.Position(), r.ctl.Snapshot().StreamStats.Reached, formatSnap(r.ctl.Snapshot()))
}

// TestCoexistRejectedStreamPointPreservesHolding 流排空停在末点保持时，
// 喂一个“按当前状态根本走不出来”的点（段速超速）必须被明确拒绝，
// 且：
//  1. 末点位置保持不被破坏——拒绝后每个周期仍持续下发保持位置；
//  2. 排队/保持等运动状态与拒绝前一致；
//  3. 该类拒绝(command_rejected|stream|stream_segment_too_fast)与普通
//     越界拒绝（控制器软限位预检 safety|safety|soft_limit_violation；
//     以及 planner 层硬限位 stream_position_out_of_limit）彼此可区分。
func TestCoexistRejectedStreamPointPreservesHolding(t *testing.T) {
	r := newStreamingRig(t, protocol.V2, stream.RejectNew, 32)
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 3; i++ {
		r.tick()
	}
	const hold = 0.2
	runStreamHolding(r, hold, 300*time.Millisecond, 600*time.Millisecond)

	// 被拒前状态基线。
	stBefore := r.ctl.Snapshot().StreamStats
	reachedBefore := stBefore.Reached
	downBefore := r.bus.Stats().DownEnqueued
	posBefore := r.drv.Position()

	// (a) 当前状态走不出来的点：保持在 0.2，要求 2ms 内到 0.3，
	//     段速 50 rad/s 远超上限 2 => command_rejected|stream|stream_segment_too_fast。
	//     这类拒绝不得触发任何安全状态变化（安全状态仍是 RUN）。
	infeasible := r.feedStreamPoint(hold+0.1, 2*time.Millisecond)
	feFast := mustReason(t, infeasible, stream.ReasonSegmentTooFast)
	if feFast.Category != "command_rejected" || feFast.Source != "stream" {
		t.Fatalf("want command_rejected|stream, got %s|%s", feFast.Category, feFast.Source)
	}
	if r.ctl.Snapshot().SafetyState != safety.StateRun {
		t.Fatalf("infeasible point must not trip safety state, got %s",
			r.ctl.Snapshot().SafetyState)
	}

	// (c) 迟到点同样不能破坏保持（另一类可区分的纯流式拒绝，RUN 不变）。
	late := r.ctl.SubmitStreamPoint(stream.Point{At: r.fc.Now(), PositionRad: hold + 0.05})
	mustReason(t, late, stream.ReasonLatePoint)
	if r.ctl.Snapshot().SafetyState != safety.StateRun {
		t.Fatalf("late point must not trip safety state, got %s", r.ctl.Snapshot().SafetyState)
	}

	// 拒绝之后多个周期：保持行为必须与拒绝前完全一致。
	const probe = 8
	for i := 0; i < probe; i++ {
		r.tick()
	}
	snap := r.ctl.Snapshot()
	if !snap.StreamActive || snap.StreamQueued != 0 {
		t.Fatalf("holding state changed by rejected points: active=%v queued=%d",
			snap.StreamActive, snap.StreamQueued)
	}
	if grew := r.bus.Stats().DownEnqueued - downBefore; grew != probe {
		t.Fatalf("want %d uninterrupted hold commands after rejection, got %d (hold was dropped)",
			probe, grew)
	}
	if st := snap.StreamStats; st.Reached != reachedBefore || st.Appended != stBefore.Appended {
		t.Fatalf("rejected point mutated queue/reach stats: before=%+v after=%+v", stBefore, st)
	}
	dPos := r.drv.Position() - posBefore
	// 允许伺服把极小自然偏差拉回保持点（量级 1e-6）；真正要防的是
	// “保持被丢掉后不再下发命令”，那会产生 1e-2 量级的漂移/失控。
	if dPos < -1e-4 || dPos > 1e-4 {
		t.Fatalf("held position must be unchanged: before=%.9f after=%.9f delta=%.2e",
			posBefore, r.drv.Position(), dPos)
	}
	t.Logf("输入: 保持点=%.2f; 被拒点=超速(2ms->%.1f)/迟到(同刻)\n判定依据: %s | %s\n最终状态: active=%v queued=%d 拒绝后保持命令数=%d 设备位置=%.4f state=%s\n%s",
		hold, hold+0.1, feFast.Reason, stream.ReasonLatePoint,
		snap.StreamActive, snap.StreamQueued, r.bus.Stats().DownEnqueued-downBefore,
		r.drv.Position(), snap.SafetyState, formatSnap(snap))

	// 被拒之后再喂一个合法续点：流必须照常恢复运动（拒绝没有毒化后续）。
	if err := r.feedStreamPoint(0.24, 200*time.Millisecond); err != nil {
		t.Fatalf("valid point after rejections must be accepted: %v", err)
	}
	for i := 0; i < 40; i++ {
		r.tick()
	}
	if pos := r.drv.Position(); pos < 0.23 || pos > 0.25 {
		t.Fatalf("stream must resume to 0.24 after rejected points, got %.4f", pos)
	}

	// (b) 普通越界点与“走不出来”必须能分开看出来：目标 5 在控制器
	//     软限位预检处即被拦为 safety|safety|soft_limit_violation
	//     （此时停在 0.25 保持、仍在 RUN）；类别/来源/原因都与 (a) 不同。
	//     硬限位级别的 stream_position_out_of_limit 在 planner 单测中覆盖。
	outOfSoft := r.feedStreamPoint(5, 100*time.Millisecond)
	feOOL := mustReason(t, outOfSoft, "soft_limit_violation")
	if (feOOL.Category == feFast.Category && feOOL.Source == feFast.Source) ||
		feOOL.Reason == feFast.Reason {
		t.Fatalf("ordinary out-of-limit must be distinguishable from infeasible: %s|%s|%s vs %s",
			feOOL.Category, feOOL.Source, feOOL.Reason, feFast.Reason)
	}
	t.Logf("普通越界可区分: 类别=%s 来源=%s 原因=%s（区别于 %s/%s）",
		feOOL.Category, feOOL.Source, feOOL.Reason, feFast.Category, feFast.Reason)
}

// TestCoexistRejectedSingleTargetDoesNotReleaseStream 被拒的单目标命令
// （非 RUN / 越界 / 矛盾字段）不能顺手把正在进行的流式运动退掉。
func TestCoexistRejectedSingleTargetDoesNotReleaseStream(t *testing.T) {
	r := newStreamingRig(t, protocol.V2, stream.RejectNew, 32)
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 3; i++ {
		r.tick()
	}
	// 流走到一半时发一条越界单目标：必须被 trajectory 层拒绝，
	// 且流式队列/活动状态原封不动、下行不中断。
	if err := r.feedStreamPoint(0.2, 600*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 10; i++ {
		r.tick()
	}
	beforeDown := r.bus.Stats().DownEnqueued
	queuedBefore := r.ctl.StreamQueueLen()
	if queuedBefore != 1 {
		t.Fatalf("precondition: want 1 active stream point, got %d", queuedBefore)
	}
	_, errBad := r.ctl.SubmitCommand(trajectory.Command{
		Mode: protocol.ModePosition, Position: 9,
	})
	mustReason(t, errBad, "position_out_of_limit")
	if r.ctl.StreamQueueLen() != 1 || !r.ctl.Snapshot().StreamActive {
		t.Fatal("rejected single-target must not release active stream")
	}
	for i := 0; i < 5; i++ {
		r.tick()
	}
	if r.bus.Stats().DownEnqueued <= beforeDown {
		t.Fatal("stream downlink must continue after rejected single-target")
	}
	// 非 RUN 下被 outputs_disabled 拒绝也同样不能改流式状态：
	// 急停本身会清队列（安全规则固定），这里只验证 RUN 下的拒绝路径。
	if r.ctl.Snapshot().SafetyState != safety.StateRun {
		t.Fatalf("precondition RUN, got %s", r.ctl.Snapshot().SafetyState)
	}
	t.Logf("被拒单目标不影响流式结果: queued=%d 新增下行=%d\n%s",
		r.ctl.StreamQueueLen(), r.bus.Stats().DownEnqueued-beforeDown, formatSnap(r.ctl.Snapshot()))
}
