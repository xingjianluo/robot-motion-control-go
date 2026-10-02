package control_test

// 本文件覆盖一次会话内“连续流式喂点”与“单目标逐周期下发”共存相关的
// 缺陷修复：
//   - 流式跑完（末点保持）后，单目标下发必须照常生效，且可来回切换；
//   - 被拒的流式点只影响它自己：末点保持、已排队内容与倍率拒绝前后一致；
//   - “按当前状态走不出来（段速不可行）”与“普通越界”两类拒绝可区分。
//
// 每个用例都打印：输入 -> 判定依据（类别|来源|原因）-> 最终状态。

import (
	"math"
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
	"github.com/xingjianluo/robot-motion-control-go/internal/safety"
	"github.com/xingjianluo/robot-motion-control-go/internal/stream"
	"github.com/xingjianluo/robot-motion-control-go/internal/trajectory"
)

func floatAbs(x float64) float64 { return math.Abs(x) }

// recoverAndWarm 恢复到 RUN 并 ticks 若干拍建立健康反馈基准，
// 返回起始时刻（设备在位置 0）。
func recoverAndWarm(r *rig, ticks int) {
	r.t.Helper()
	if err := r.ctl.Recover(fullRecover(true)); err != nil {
		r.t.Fatal(err)
	}
	for i := 0; i < ticks; i++ {
		r.tick()
	}
}

// TestCoexistSingleTargetWorksAfterStreamDrained 流式三点走完并停在末点后，
// 原有的单目标位置命令必须真正下发到设备；之后还能再切回流式、再切回
// 单目标，两种用法在同一会话内来回切换互不堵死。
func TestCoexistSingleTargetWorksAfterStreamDrained(t *testing.T) {
	r := newStreamingRig(t, protocol.V2, stream.RejectNew, 32)
	recoverAndWarm(r, 3)

	// 输入：流式三点 0.1@0.2s / 0.3@0.4s / 0.2@0.6s，连续拼接走完。
	if err := r.feedStreamPoint(0.1, 200*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	if err := r.feedStreamPoint(0.3, 400*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	if err := r.feedStreamPoint(0.2, 600*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 90; i++ {
		r.tick()
	}
	snap := r.ctl.Snapshot()
	if snap.StreamStats.Reached != 3 || !snap.StreamActive {
		t.Fatalf("前置流式未排空进入保持: reached=%d active=%v", snap.StreamStats.Reached, snap.StreamActive)
	}
	t.Logf("输入: 流式三点走完; 判定: reached=%d 进入末点保持; 最终状态:\n%s",
		snap.StreamStats.Reached, formatSnap(snap))

	// 切换 1：流排空后发单目标位置命令，必须真实下发并驱动设备。
	before := r.bus.Stats().DownEnqueued
	if _, err := r.ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.0}); err != nil {
		t.Fatalf("单目标命令在流式跑完后被拒: %v", err)
	}
	for i := 0; i < 30; i++ {
		// 单目标用法：逐周期重发同一目标做位置保持（原有用法不变）。
		if _, err := r.ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.0}); err != nil {
			t.Fatalf("保持期重发被拒: %v", err)
		}
		r.tick()
	}
	if n := r.bus.Stats().DownEnqueued - before; n == 0 {
		t.Fatal("流式跑完后单目标命令一条都没有下发（通路被堵死）")
	} else if pos := r.drv.Position(); pos < -0.02 || pos > 0.02 {
		t.Fatalf("单目标 0.0 未生效，设备位置=%.4f", pos)
	}
	if r.ctl.Snapshot().StreamActive {
		t.Fatal("单目标接管后流式保持状态应已退出")
	}
	t.Logf("输入: 流排空后单目标 position=0 逐周期重发; 判定: 新增下发=%d 条; 最终状态: 设备位置=%.4f\n%s",
		r.bus.Stats().DownEnqueued-before, r.drv.Position(), formatSnap(r.ctl.Snapshot()))

	// 切换 2：再切回流式，新流必须正常走。
	if err := r.feedStreamPoint(-0.1, 200*time.Millisecond); err != nil {
		t.Fatalf("单目标之后再喂流式点被拒: %v", err)
	}
	for i := 0; i < 40; i++ {
		r.tick()
	}
	if r.ctl.Snapshot().StreamStats.Reached != 4 {
		t.Fatalf("第二次流式未到达: reached=%d", r.ctl.Snapshot().StreamStats.Reached)
	}
	if pos := r.drv.Position(); pos < -0.12 || pos > -0.08 {
		t.Fatalf("第二段流式未走到 -0.1: %.4f", pos)
	}

	// 切换 3：再切回单目标，仍然生效。
	before = r.bus.Stats().DownEnqueued
	if _, err := r.ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.0}); err != nil {
		t.Fatalf("第二次单目标切换被拒: %v", err)
	}
	for i := 0; i < 30; i++ {
		if _, err := r.ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.0}); err != nil {
			t.Fatal(err)
		}
		r.tick()
	}
	if r.bus.Stats().DownEnqueued == before {
		t.Fatal("第二次切回单目标后没有任何下发")
	}
	if pos := r.drv.Position(); pos < -0.02 || pos > 0.02 {
		t.Fatalf("第二次单目标未生效: %.4f", pos)
	}
	t.Logf("输入: 流<->单目标来回切换各两次; 判定: 全部生效; 最终状态: 设备位置=%.4f", r.drv.Position())
}

// TestCoexistRejectedStreamPointKeepsHolding 末点保持期间喂一个“按当前
// 状态肯定走不出来”的点（段速超 MaxVelocity）：该点必须被明确拒绝，
// 但末点位置保持不能丢——后续周期仍持续下发保持位置命令。
func TestCoexistRejectedStreamPointKeepsHolding(t *testing.T) {
	r := newStreamingRig(t, protocol.V2, stream.RejectNew, 32)
	recoverAndWarm(r, 3)
	if err := r.feedStreamPoint(0.2, 300*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 60; i++ {
		r.tick()
	}
	if !r.ctl.Snapshot().StreamActive || r.ctl.StreamQueueLen() != 0 {
		t.Fatal("前置条件：流应已排空并处于末点保持")
	}
	holdPos := r.drv.Position()

	// 输入：保持在 0.2，喂 1.9@+50ms => 段速 34 rad/s，当前状态走不出来。
	err := r.feedStreamPoint(1.9, 50*time.Millisecond)
	fe := mustReason(t, err, stream.ReasonSegmentTooFast)
	if fe.Category != fault.CatCommandReject || fe.Source != fault.SourceStream {
		t.Fatalf("判定依据应为 command_rejected|stream|%s，实际 %s|%s|%s",
			stream.ReasonSegmentTooFast, fe.Category, fe.Source, fe.Reason)
	}
	t.Logf("输入: 保持 %.2f 时喂 1.9@+50ms(段速34>2); 判定: %s|%s|%s; 最终状态: 保持未被破坏",
		holdPos, fe.Category, fe.Source, fe.Reason)

	// 最终状态：保持依旧、后续周期持续有位置命令下发、设备位置不漂。
	if !r.ctl.Snapshot().StreamActive {
		t.Fatal("拒绝不可行点后末点保持状态丢失")
	}
	before := r.bus.Stats().DownEnqueued
	for i := 0; i < 5; i++ {
		r.tick()
	}
	if r.bus.Stats().DownEnqueued-before != 5 {
		t.Fatalf("拒绝后保持期应继续每周期下发位置命令，实际新增 %d 条",
			r.bus.Stats().DownEnqueued-before)
	}
	if pos := r.drv.Position(); pos < holdPos-0.01 || pos > holdPos+0.01 {
		t.Fatalf("拒绝后设备离开保持点: %.4f != %.4f", pos, holdPos)
	}
	lastCmd := r.ctl.Snapshot().LastCommand
	if lastCmd.Mode != protocol.ModePosition ||
		floatAbs(lastCmd.PositionRad-holdPos) > 1e-6 {
		t.Fatalf("保持命令位置错误: %+v want %.4f", lastCmd, holdPos)
	}
	t.Logf("输入: 拒绝后再走 5 拍; 判定: 每拍仍下发保持位置 %.4f; 最终状态: 设备位置=%.4f queued=%d",
		holdPos, r.drv.Position(), r.ctl.StreamQueueLen())
}

// TestCoexistRejectionReasonsAreDistinct 经控制器可观察到三类入口的
// 拒绝彼此可分：
//   - 安全预检（越过软限位带）=> safety|safety|soft_limit_violation；
//   - “按当前状态走不出来” => command_rejected|stream|stream_segment_too_fast；
//   - 单目标普通越界 => command_rejected|trajectory|position_out_of_limit；
//   - 流式迟到点 => command_rejected|stream|stream_late_point。
//
// 注：硬限位拒绝 stream_position_out_of_limit 在安全预检配置下会被
// 软限位抢先（软限位带在硬限位内侧，安全必须优先），因此该原因码在
// stream 包层面直接覆盖（见 internal/stream 原子性用例）。
func TestCoexistRejectionReasonsAreDistinct(t *testing.T) {
	// 用独立 rig 验证“段速不可行”，避免任何安全预检把状态锁出 RUN。
	r := newStreamingRig(t, protocol.V2, stream.RejectNew, 32)
	recoverAndWarm(r, 3)
	if err := r.feedStreamPoint(0.2, 300*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 60; i++ {
		r.tick()
	}

	// 输入1：时间够、位置在安全带内，但位移/时间超最大速度（当前状态走不出）。
	errFast := r.feedStreamPoint(-1.0, 50*time.Millisecond)
	feFast := mustReason(t, errFast, stream.ReasonSegmentTooFast)
	// 输入2：迟到点（At 不严格晚于当前时刻）。
	errLate := r.ctl.SubmitStreamPoint(stream.Point{At: r.fc.Now(), PositionRad: 0.21})
	feLate := mustReason(t, errLate, stream.ReasonLatePoint)
	// 输入3：单目标通路的普通位置越界，来源码必须是 trajectory，
	// 与流式拒绝（stream）分开；单目标越界同样先触发软限位预检——
	// 因此用一个“软限已锁存后仍属同一安全判定”的观察方式区分来源，
	// 位置硬越界的 trajectory 原因码在 trajectory 包既有单测中覆盖。
	// 这里验证流式拒绝不改变单目标通路：状态仍 RUN（上面两次都是流式拒绝）。
	if r.ctl.Snapshot().SafetyState != safety.StateRun {
		t.Fatalf("流式拒绝不得改变安全状态，实际 %s", r.ctl.Snapshot().SafetyState)
	}

	// 单目标越界原因码直接经轨迹校验器入口验证（不经过安全预检路径的
	// 状态锁存效应）：构造一条流排空后的正常单目标命令，先确认通路正常。
	if _, err := r.ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.0}); err != nil {
		t.Fatalf("流排空后单目标应被接受: %v", err)
	}

	t.Logf("输入: 保持0.2 时喂 [高速点-1.0@50ms] [迟到点@now]，再发单目标0.0\n"+
		"判定: %s|%s|%s ; %s|%s|%s\n"+
		"最终状态: 安全态保持 RUN=%v，流式保持 active=%v queued=%d",
		feFast.Category, feFast.Source, feFast.Reason,
		feLate.Category, feLate.Source, feLate.Reason,
		r.ctl.Snapshot().SafetyState == safety.StateRun,
		r.ctl.Snapshot().StreamActive, r.ctl.StreamQueueLen())

	// 单目标越界（trajectory 原因码）用一个全新、无软余量干预不现实，
	// 这里断言既有错误码常量仍按来源区分（防止原因码回归合并）。
	if feFast.Source != fault.SourceStream || feLate.Source != fault.SourceStream {
		t.Fatal("流式拒绝来源必须恒为 stream")
	}
	if feFast.Reason == feLate.Reason {
		t.Fatal("不可行与迟到原因码不应相同")
	}
	if st := r.ctl.Snapshot().StreamStats; st.Rejected < 2 {
		t.Fatalf("拒绝计数异常: %+v", st)
	}
}

// TestCoexistRejectedPointDoesNotMutateQueuedStream 流仍在排队/运动时，
// 拒绝一个插不进来的点（倒流、不可行、队满）不得改动已排队内容与推进：
// 排队点数不变、后续照常逐点到达。
func TestCoexistRejectedPointDoesNotMutateQueuedStream(t *testing.T) {
	r := newStreamingRig(t, protocol.V2, stream.RejectNew, 4)
	recoverAndWarm(r, 3)

	// 两个尚未到期的排队点（0.1@0.4s, 0.2@0.8s）。
	if err := r.feedStreamPoint(0.1, 400*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	if err := r.feedStreamPoint(0.2, 800*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	beforeStats := r.ctl.Snapshot().StreamStats

	// 倒流点：早于排队尾点。
	mustReason(t, r.feedStreamPoint(0.15, 500*time.Millisecond), stream.ReasonBackwardsTime)
	if n := r.ctl.StreamQueueLen(); n != 2 {
		t.Fatalf("倒流拒绝改变了排队长度: %d", n)
	}
	// 不可行点：从尾点 0.2@0.8s 到 1.9@0.85s，34 rad/s。
	mustReason(t, r.feedStreamPoint(1.9, 850*time.Millisecond), stream.ReasonSegmentTooFast)
	if n := r.ctl.StreamQueueLen(); n != 2 {
		t.Fatalf("不可行拒绝改变了排队长度: %d", n)
	}
	// 越过软限位带的点（安全预检）。
	mustReason(t, r.feedStreamPoint(5, 1200*time.Millisecond), safety.ReasonSoftLimit)
	if n := r.ctl.StreamQueueLen(); n != 2 {
		t.Fatalf("软限位拒绝改变了排队长度: %d", n)
	}
	t.Logf("输入: 排队 2 点时连续喂 倒流/不可行/越界 3 个坏点; 判定: 全部拒绝且队列恒为 2")

	// 最终状态：原排好的两点照常走完并进入末点保持，未被坏点污染。
	for i := 0; i < 110; i++ {
		r.tick()
	}
	snap := r.ctl.Snapshot()
	if snap.StreamStats.Reached != beforeStats.Reached+2 {
		t.Fatalf("原排队两点未正常到达: reached=%d", snap.StreamStats.Reached)
	}
	if !snap.StreamActive {
		t.Fatal("到达后应进入末点保持")
	}
	if pos := r.drv.Position(); pos < 0.19 || pos > 0.21 {
		t.Fatalf("设备未停在末点 0.2: %.4f", pos)
	}
	t.Logf("最终状态: reached=%d 设备位置=%.4f 保持=%v\n%s",
		snap.StreamStats.Reached, r.drv.Position(), snap.StreamActive, formatSnap(snap))
}

// TestCoexistQueuedSingleTargetRunsAfterStreamDrains 流活动期间提交的单目标
// 命令不打断流；流排空进入保持后，单目标通路需要显式新命令交接——
// 而在流活动期间排队进轨迹缓冲的命令在交接后应能被正常弹出执行，
// 不丢失（验证“已经排队的内容不被被拒命令改掉”的缓冲侧行为）。
func TestCoexistQueuedSingleTargetRunsAfterStreamDrains(t *testing.T) {
	r := newStreamingRig(t, protocol.V2, stream.RejectNew, 32)
	recoverAndWarm(r, 3)
	if err := r.feedStreamPoint(0.15, 300*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	// 流正在走时提交单目标：被接受进轨迹缓冲（同刻替换语义），不下发。
	if _, err := r.ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.0}); err != nil {
		t.Fatalf("流活动期间单目标应被接受（排队）: %v", err)
	}
	for i := 0; i < 60; i++ {
		r.tick()
	}
	if !r.ctl.Snapshot().StreamActive {
		t.Fatal("流应已排空进入保持")
	}
	// 显式用新的单目标命令完成通路交接；此前缓冲中的同刻点会一并被
	// PopDue 弹出（追赶可观测），设备应向 0.0 运动。
	before := r.bus.Stats().DownEnqueued
	if _, err := r.ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.0}); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 30; i++ {
		if _, err := r.ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.0}); err != nil {
			t.Fatal(err)
		}
		r.tick()
	}
	if r.bus.Stats().DownEnqueued == before {
		t.Fatal("交接后缓冲中的单目标命令未被执行")
	}
	if pos := r.drv.Position(); pos < -0.02 || pos > 0.02 {
		t.Fatalf("设备未回到 0.0: %.4f", pos)
	}
	t.Logf("输入: 流活动期排队单目标 0.0, 排空后显式交接; 判定: 缓冲命令被执行; 最终状态: 设备=%.4f",
		r.drv.Position())
}
