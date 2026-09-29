package stream

import (
	"math"
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
)

const period = 10 * time.Millisecond

func testCfg(capacity int, policy OverflowPolicy) Config {
	return Config{
		Capacity: capacity, Policy: policy, Period: period,
		MinOverride: 0.01, MaxOverride: 2.0,
		MinPositionRad: -2, MaxPositionRad: 2,
		SoftLowRad: -1.9, SoftHighRad: 1.9,
		MaxVelocityRadS: 2, MaxAccelRadS2: 100, MaxTorqueNm: 5,
	}
}

func mustAppend(t *testing.T, p *Player, tg Target, refPos float64) {
	t.Helper()
	if ev, err := p.Append(tg, refPos, true, time.Unix(0, 0)); err != nil {
		t.Fatalf("append %+v: %v (evicted=%v)", tg, err, ev)
	}
}

func runN(t *testing.T, p *Player, n int, refPos float64) []Dispatched {
	t.Helper()
	var out []Dispatched
	for i := 0; i < n; i++ {
		if d, ok := p.Step(time.Unix(0, 0).Add(time.Duration(i)*period), refPos, true); ok {
			out = append(out, d)
			if d.Mode == protocol.ModePosition {
				refPos = d.PositionRad
			}
		}
	}
	return out
}

// 连续速度段：在前段未走完时追加下一段，段间没有空周期（不停车起步），
// 且速度轮廓连续、单调顺序执行。
func TestVelocitySegmentsConcatenateWithoutGap(t *testing.T) {
	p := New(testCfg(8, PolicyRejectNew))
	now := time.Unix(0, 0)
	mustAppend(t, p, Target{Mode: protocol.ModeVelocity, Value: 1.0, Hold: 3 * period, SeqIn: 1}, 0)
	ds := runN(t, p, 2, 0) // 第 1 段还剩 1 周期时
	if len(ds) != 2 || ds[1].VelocityRadS != 1.0 {
		t.Fatalf("seg1 dispatch: %+v", ds)
	}
	// 上一段还没走完就塞下一段（顺序 1 -> 2），拼接加速度
	// |1.2-1.0|/0.01=20 <= 100，接受。
	if _, err := p.Append(Target{Mode: protocol.ModeVelocity, Value: 1.2, Hold: 2 * period, SeqIn: 2}, 0, true, now); err != nil {
		t.Fatal(err)
	}
	ds = runN(t, p, 3, 0)
	if len(ds) != 3 {
		t.Fatalf("expected no gap cycle, got %d dispatches", len(ds))
	}
	if ds[0].SourceSeq != 1 || ds[1].SourceSeq != 2 || ds[2].SourceSeq != 2 {
		t.Fatalf("segment order broken: %d,%d,%d", ds[0].SourceSeq, ds[1].SourceSeq, ds[2].SourceSeq)
	}
	if ds[1].VelocityRadS != 1.2 {
		t.Fatalf("seg2 velocity not continuous: %+v", ds[1])
	}
	// 下发序号必须严格单调（线上协议不允许回退）。
	for i := 1; i < len(ds); i++ {
		if ds[i].Seq != ds[i-1].Seq+1 {
			t.Fatalf("dispatch seq not monotonic: %d -> %d", ds[i-1].Seq, ds[i].Seq)
		}
	}
	st := p.Status()
	if st.CompletedCount != 2 || st.DispatchedCount != 5 || p.Active() {
		t.Fatalf("unexpected status: %+v", st)
	}
	t.Logf("输入: 速度段 seq=1(3周期,1.0) 未走完追加 seq=2(2周期,1.2); 判定: 全程无空周期; 最终: completed=%d dispatched=%d",
		st.CompletedCount, st.DispatchedCount)
}

// 位置段线性插值：起点取真实参考位置，末周期精确落在目标。
func TestPositionSegmentInterpolates(t *testing.T) {
	p := New(testCfg(8, PolicyRejectNew))
	// 0 -> 0.5 用 5 周期 => 每周期 0.1，即 10 rad/s，会超速被拒；
	// 用 0.05 目标 5 周期 => 1 rad/s，可行。
	mustAppend(t, p, Target{Mode: protocol.ModePosition, Value: 0.05, Hold: 5 * period, SeqIn: 7}, 0)
	ds := runN(t, p, 5, 0)
	if len(ds) != 5 {
		t.Fatalf("want 5 dispatches, got %d", len(ds))
	}
	for i, d := range ds {
		want := 0.01 * float64(i+1)
		if math.Abs(d.PositionRad-want) > 1e-12 {
			t.Fatalf("cycle %d pos=%.6f want %.4f", i, d.PositionRad, want)
		}
	}
	if last := ds[len(ds)-1].PositionRad; last != 0.05 {
		t.Fatalf("final must land exactly on target, got %.6f", last)
	}
}

// 插值步长超速：位置段按当前倍率走不出来 => 明确拒绝（非普通越界原因）。
func TestPositionStepTooFastRejected(t *testing.T) {
	p := New(testCfg(8, PolicyRejectNew))
	_, err := p.Append(Target{Mode: protocol.ModePosition, Value: 0.5, Hold: 5 * period}, 0, true, time.Unix(0, 0))
	fe, ok := fault.As(err)
	if !ok || fe.Reason != ReasonInterpStep || fe.Source != fault.SourceStream {
		t.Fatalf("want stream_interpolation_step_too_fast, got %v", err)
	}
}

// 在线倍率：对在跑段与排队段同时生效；切换后一个周期内只使用新倍率。
func TestOverrideAppliesToActiveAndQueued(t *testing.T) {
	p := New(testCfg(8, PolicyRejectNew))
	mustAppend(t, p, Target{Mode: protocol.ModeVelocity, Value: 1.0, Hold: 4 * period, SeqIn: 1}, 0)
	mustAppend(t, p, Target{Mode: protocol.ModeVelocity, Value: 1.0, Hold: 4 * period, SeqIn: 2}, 0)
	// 跑 2 个周期（100%），活动段应还剩 2 周期。
	d := runN(t, p, 2, 0)
	if d[1].VelocityRadS != 1.0 || d[1].AtOverride != 1.0 {
		t.Fatalf("pre-override dispatch: %+v", d[1])
	}
	if err := p.SetOverride(0.5, time.Unix(0, 0)); err != nil {
		t.Fatal(err)
	}
	if got := p.Override(); got != 0.5 {
		t.Fatalf("override=%v", got)
	}
	// 活动段剩余份额 50% 在 50% 速度下需要完整 4 周期；随后排队段也以 0.5 下发。
	d = runN(t, p, 6, 0)
	for i, e := range d {
		if e.VelocityRadS != 0.5 || e.AtOverride != 0.5 {
			t.Fatalf("cycle %d mixed override within a cycle: %+v", i, e)
		}
	}
	// 4 个周期完成活动段，剩余 2 个周期属于排队段。
	if d[3].SourceSeq != 1 || d[4].SourceSeq != 2 {
		t.Fatalf("segment boundary: %d then %d", d[3].SourceSeq, d[4].SourceSeq)
	}
	st := p.Status()
	if st.CompletedCount != 1 {
		t.Fatalf("active segment must finish under new override: %+v", st)
	}
	t.Logf("输入: 50%% 倍率在活动段中途; 判定: 每个周期 AtOverride 恒为 0.5，活动段按比例延长; 最终: completed=%d activeSeq=%d",
		st.CompletedCount, d[5].SourceSeq)
}

// 越限倍率（速度上限）：用隔离的判定依据覆盖——前驱段已建立轮廓，
// 在跑段与排队段同速（拼接差恒为 0，排除加速度原因），200% 把
// 1.05 rad/s 顶到 2.1 > 2，必须以速度越限拒绝且原子不生效。
func TestOverrideBeyondVelocityLimitRejectedAtomically(t *testing.T) {
	p := New(testCfg(8, PolicyRejectNew))
	mustAppend(t, p, Target{Mode: protocol.ModeVelocity, Value: 0.5, Hold: 6 * period, SeqIn: 1}, 0)
	_ = runN(t, p, 2, 0) // 段1 建立 0.5 前驱轮廓
	// 段2 与段1 的拼接差 0.55 => 55/s 可行；100% 速度 1.05 合法。
	mustAppend(t, p, Target{Mode: protocol.ModeVelocity, Value: 1.05, Hold: 6 * period, SeqIn: 2}, 0)
	_ = runN(t, p, 5, 0) // 跑完段1，段2 激活并建立 1.05 轮廓
	mustAppend(t, p, Target{Mode: protocol.ModeVelocity, Value: 1.05, Hold: 6 * period, SeqIn: 3}, 0)
	if err := p.SetOverride(1.5, time.Unix(0, 0)); err != nil { // 1.575 可行
		t.Fatalf("150%% feasible: %v", err)
	}
	err := p.SetOverride(2.0, time.Unix(0, 0)) // 2.10 > 2，同速段拼接差 0
	fe, ok := fault.As(err)
	if !ok || fe.Reason != ReasonTargetVelocityOutOfLimit {
		t.Fatalf("want velocity bound only, got %v", err)
	}
	if p.Override() != 1.5 {
		t.Fatalf("rejection must be atomic: override=%v", p.Override())
	}
	d := runN(t, p, 1, 0)
	if len(d) != 1 || d[0].AtOverride != 1.5 || math.Abs(d[0].VelocityRadS-1.05*1.5) > 1e-9 {
		t.Fatalf("state unchanged required: %+v", d)
	}
	t.Logf("输入: 同速 1.05 段(在跑+排队) + 倍率 200%%; 判定依据: %v; 最终倍率: 1.50", err)
}

// 倍率引起的拼接超加速也要拒绝，原因可与速度越限区分；拒绝为原子操作。
func TestOverrideCausingSpliceAccelRejected(t *testing.T) {
	p := New(testCfg(8, PolicyRejectNew))
	// 100% 下两段速度差 0.6 => 拼接加速度 60 <= 100，入队通过；
	// 150% 倍率下速度差 0.9 => 90 仍可行；200% => 差 1.2 => 120 > 100
	// 必须以 stream_splice_accel_out_of_limit 拒绝（而非速度越限：2*0.8=1.6<=2）。
	mustAppend(t, p, Target{Mode: protocol.ModeVelocity, Value: 0.2, Hold: 4 * period, SeqIn: 1}, 0)
	mustAppend(t, p, Target{Mode: protocol.ModeVelocity, Value: 0.8, Hold: 4 * period, SeqIn: 2}, 0)
	if err := p.SetOverride(1.5, time.Unix(0, 0)); err != nil {
		t.Fatalf("150%% should be feasible (splice 90/s): %v", err)
	}
	err := p.SetOverride(2.0, time.Unix(0, 0))
	fe, ok := fault.As(err)
	if !ok || fe.Reason != ReasonSpliceAccel {
		t.Fatalf("want splice accel reject at 200%%, got %v", err)
	}
	if p.Override() != 1.5 {
		t.Fatalf("rejected override must not apply atomically: %v", p.Override())
	}
	t.Logf("输入: 两段速度 0.2->0.8, 倍率 150%%(可行) 再调 200%%; 判定依据: %v; 最终倍率: %.2f", err, p.Override())
}

// 排队容量：reject_new 满则拒新（可计数观测），内存有界。
func TestQueueRejectNewAtCapacity(t *testing.T) {
	p := New(testCfg(2, PolicyRejectNew))
	mustAppend(t, p, Target{Mode: protocol.ModeVelocity, Value: 0.1, Hold: 1 * period, SeqIn: 1}, 0)
	mustAppend(t, p, Target{Mode: protocol.ModeVelocity, Value: 0.1, Hold: 1 * period, SeqIn: 2}, 0)
	// 容量 2 指“等待队列”；第 1 段在首个 Step 前仍是等待，故此时满。
	_, err := p.Append(Target{Mode: protocol.ModeVelocity, Value: 0.1, Hold: 1 * period, SeqIn: 3}, 0, true, time.Unix(0, 0))
	fe, ok := fault.As(err)
	if !ok || fe.Reason != ReasonOverflowRejectNew {
		t.Fatalf("want overflow reject, got %v", err)
	}
	if p.WaitingDepth() != 2 {
		t.Fatalf("queue must remain bounded at 2, got %d", p.WaitingDepth())
	}
	if p.Status().RejectedCount < 1 {
		t.Fatal("rejection not counted")
	}
}

// 排队容量：evict_oldest 挤掉最旧等待段并留痕；活动段永不被挤。
func TestQueueEvictOldest(t *testing.T) {
	p := New(testCfg(2, PolicyEvictOldest))
	mustAppend(t, p, Target{Mode: protocol.ModeVelocity, Value: 0.1, Hold: 2 * period, SeqIn: 1}, 0)
	mustAppend(t, p, Target{Mode: protocol.ModeVelocity, Value: 0.1, Hold: 1 * period, SeqIn: 2}, 0)
	_ = runN(t, p, 1, 0) // 段1 成为活动段；等待队列里只有段2
	mustAppend(t, p, Target{Mode: protocol.ModeVelocity, Value: 0.1, Hold: 1 * period, SeqIn: 3}, 0)
	// 再追加：等待队列满（段3），挤掉段3？不——段2 在段1 完成前一直在等待。
	// 此刻等待=[段2,段3]已满；追加段4 挤掉段2。
	ev, err := p.Append(Target{Mode: protocol.ModeVelocity, Value: 0.1, Hold: 1 * period, SeqIn: 4}, 0, true, time.Unix(0, 0))
	if err != nil || len(ev) != 1 || ev[0].SourceSeq != 2 {
		t.Fatalf("expect eviction of seq2, ev=%v err=%v", ev, err)
	}
	st := p.Status()
	if st.EvictedCount != 1 || !st.HasLastEviction || st.LastEviction.SourceSeq != 2 {
		t.Fatalf("eviction not observable: %+v", st)
	}
	// 执行剩余：活动段1 -> 段3 -> 段4；段2 永不出现。
	var seqs []uint64
	for i := 0; i < 4; i++ {
		if d, ok := p.Step(time.Unix(0, 0).Add(time.Duration(i)*period), 0, true); ok {
			seqs = append(seqs, d.SourceSeq)
		}
	}
	want := []uint64{1, 3, 4}
	if len(seqs) != len(want) {
		t.Fatalf("executed seqs=%v want %v (evicted seq2 must not run)", seqs, want)
	}
	for i := range want {
		if seqs[i] != want[i] {
			t.Fatalf("executed seqs=%v want %v", seqs, want)
		}
	}
}

// 力矩段不被倍率缩放幅值；仅持续周期数随倍率变化。
func TestTorqueSegmentAmplitudeUnscaled(t *testing.T) {
	p := New(testCfg(8, PolicyRejectNew))
	mustAppend(t, p, Target{Mode: protocol.ModeTorque, Value: 3.0, Hold: 2 * period, SeqIn: 1}, 0)
	if err := p.SetOverride(0.5, time.Unix(0, 0)); err != nil {
		t.Fatal(err)
	}
	d := runN(t, p, 1, 0)
	if d[0].TorqueNm != 3.0 {
		t.Fatalf("torque must not be scaled: %+v", d[0])
	}
	// 超力矩目标按原幅值拒绝，原因区别于速度越限。
	_, err := p.Append(Target{Mode: protocol.ModeTorque, Value: 9, Hold: 1 * period, SeqIn: 9}, 0, true, time.Unix(0, 0))
	if fe, _ := fault.As(err); fe == nil || fe.Reason != ReasonTargetTorqueOutOfLimit {
		t.Fatalf("want torque out of limit, got %v", err)
	}
}

// Clear/Reset 语义：安全停机清队列但保留倍率与计数；Reset 复位倍率。
func TestClearAndResetSemantics(t *testing.T) {
	p := New(testCfg(8, PolicyRejectNew))
	mustAppend(t, p, Target{Mode: protocol.ModeVelocity, Value: 0.1, Hold: 4 * period, SeqIn: 1}, 0)
	_ = runN(t, p, 1, 0)
	if err := p.SetOverride(0.8, time.Unix(0, 0)); err != nil {
		t.Fatal(err)
	}
	if n := p.Clear(); n != 1 {
		t.Fatalf("clear reports %d segments (1 active)", n)
	}
	if p.Active() || p.Override() != 0.8 {
		t.Fatal("clear must drop queue but keep override")
	}
	if p.Status().DispatchedCount != 1 {
		t.Fatal("clear must keep audit counters")
	}
	p.Reset()
	if p.Override() != 1.0 || p.Status().DispatchedCount != 0 || p.Active() {
		t.Fatal("reset must restore override=1 and zero counters")
	}
}

// 非法段：负 Hold / 量化为零的短 Hold / 未知模式。
func TestInvalidSegmentRejected(t *testing.T) {
	p := New(testCfg(8, PolicyRejectNew))
	bad := []Target{
		{Mode: protocol.ModeVelocity, Value: 0, Hold: -1},
		{Mode: protocol.ModeVelocity, Value: 0, Hold: time.Microsecond}, // 量化为 0
		{Mode: "fly", Value: 0, Hold: period},
	}
	for _, tg := range bad {
		if _, err := p.Append(tg, 0, true, time.Unix(0, 0)); err == nil {
			t.Fatalf("bad target accepted: %+v", tg)
		} else if fe, _ := fault.As(err); fe.Reason != ReasonInvalidSegment {
			t.Fatalf("want invalid_segment, got %v", err)
		}
	}
}
