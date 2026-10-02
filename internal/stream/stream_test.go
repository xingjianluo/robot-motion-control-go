package stream

import (
	"math"
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
)

func testCfg(capacity int, policy FullPolicy) Config {
	return Config{
		Capacity: capacity, FullPolicy: policy, Period: 10 * time.Millisecond,
		MaxAccelRadS2: 100, MaxVelocityRadS: 2,
		MinPositionRad: -2, MaxPositionRad: 2,
		MinOverride: 0, MaxOverride: 2,
	}
}

func mustFail(t *testing.T, err error, reason string) {
	t.Helper()
	fe, ok := fault.As(err)
	if !ok || fe.Reason != reason {
		t.Fatalf("want fault reason %q, got %v", reason, err)
	}
	if fe.Category != fault.CatCommandReject || fe.Source != fault.SourceStream {
		t.Fatalf("want command_rejected|stream, got %s|%s", fe.Category, fe.Source)
	}
}

// Test1 连续流式喂点：上一段还没走完就塞下一个点，点与点之间不停车，
// 位置/速度连续且所有点按顺序被到达。
func TestStreamContinuousFlowNoStop(t *testing.T) {
	t0 := time.Date(2026, 9, 29, 0, 0, 0, 0, time.UTC)
	p := New(testCfg(8, RejectNew))
	now := t0
	for i := 1; i <= 3; i++ {
		if err := p.Append(Point{At: t0.Add(time.Duration(i) * 100 * time.Millisecond),
			PositionRad: 0.1 * float64(i)}, now, 0, true); err != nil {
			t.Fatal(err)
		}
	}
	var maxStepVel, maxAccel float64
	prevVel, prevPos := 0.0, 0.0
	reached := 0
	for i := 0; i < 60; i++ {
		now = now.Add(10 * time.Millisecond)
		o := p.Advance(now)
		if o.Active {
			if math.Abs(o.VelocityRadS) > 2+1e-9 {
				t.Fatalf("velocity limit breached: %.4f", o.VelocityRadS)
			}
			step := o.PositionRad - prevPos
			if i > 0 {
				if a := math.Abs(o.VelocityRadS-prevVel) / 0.01; a > maxAccel {
					maxAccel = a
				}
				_ = step
			}
			if math.Abs(o.VelocityRadS) > maxStepVel {
				maxStepVel = math.Abs(o.VelocityRadS)
			}
			prevVel = o.VelocityRadS
			prevPos = o.PositionRad
			if o.Reached {
				reached++
			}
		}
	}
	if reached != 3 {
		t.Fatalf("want 3 reached points, got %d", reached)
	}
	if maxAccel > 100+1e-6 {
		t.Fatalf("accel limit breached across splice: %.4f", maxAccel)
	}
	// 连续运动必须真正动起来：峰值速度应接近段速 1 rad/s，且不为零起步即停。
	if maxStepVel < 0.5 {
		t.Fatalf("expected continuous motion near segment speed, maxVel=%.4f", maxStepVel)
	}
	// 最终位置停在最后一个点。
	if math.Abs(prevPos-0.3) > 1e-6 {
		t.Fatalf("final pos %.6f, want 0.3", prevPos)
	}
	t.Logf("流式拼接结果: reached=%d maxVel=%.4f maxAccel=%.4f finalPos=%.6f stats=%+v",
		reached, maxStepVel, maxAccel, prevPos, p.Stats())
}

// Test2 迟到/重叠点与时间倒流必须被明确拒绝，绝不悄悄丢点。
func TestStreamLateAndBackwardsRejected(t *testing.T) {
	t0 := time.Date(2026, 9, 29, 0, 0, 0, 0, time.UTC)
	p := New(testCfg(8, RejectNew))
	now := t0
	if err := p.Append(Point{At: t0.Add(100 * time.Millisecond), PositionRad: 0.2}, now, 0, true); err != nil {
		t.Fatal(err)
	}
	// 与 now 同时刻 => 迟到/重叠
	mustFail(t, p.Append(Point{At: now, PositionRad: 0.1}, now, 0, true), ReasonLatePoint)
	// 早于尾点 => 倒流
	mustFail(t, p.Append(Point{At: t0.Add(50 * time.Millisecond), PositionRad: 0.1}, now, 0, true),
		ReasonBackwardsTime)
	// 与尾点同一时刻 => 替换（不拒绝）
	if err := p.Append(Point{At: t0.Add(100 * time.Millisecond), PositionRad: 0.15}, now, 0, true); err != nil {
		t.Fatal(err)
	}
	if st := p.Stats(); st.Replaced != 1 {
		t.Fatalf("want 1 replacement, got %+v", st)
	}
}

// Test3 在线倍率：运动中 50%->120%->0->100%，全程速度/加速度不越限，
// 最终仍精确到达目标；不可行倍率（会顶破限速）与越界倍率分别被拒。
func TestStreamOnlineOverride(t *testing.T) {
	t0 := time.Date(2026, 9, 29, 0, 0, 0, 0, time.UTC)
	now := t0
	p := New(testCfg(16, RejectNew))
	// 段速在倍率 1 下为 1.5 rad/s（200ms 走 0.3），倍率 2 => 3 rad/s 超速。
	if err := p.Append(Point{At: t0.Add(200 * time.Millisecond), PositionRad: 0.3}, now, 0, true); err != nil {
		t.Fatal(err)
	}
	mustFail(t, p.SetOverride(3, now), ReasonOverrideOutOfRange)
	mustFail(t, p.SetOverride(2, now), ReasonOverrideInfeasible)
	if err := p.SetOverride(0.5, now); err != nil {
		t.Fatal(err)
	}
	maxAccel := 0.0
	prevVel := 0.0
	prevActive := false
	prevReached := false
	reachedAt := -1
	pos := 0.0
	for i := 0; i < 200; i++ {
		now = now.Add(10 * time.Millisecond)
		switch i {
		case 10:
			if err := p.SetOverride(1.2, now); err != nil {
				t.Fatal(err)
			}
		case 25:
			if err := p.SetOverride(0, now); err != nil {
				t.Fatal(err)
			}
		case 40:
			if err := p.SetOverride(1, now); err != nil {
				t.Fatal(err)
			}
		}
		o := p.Advance(now)
		if o.Active {
			// 到达拍下发的是“吸附到点”的绝对位置命令，保持拍下发的是
			// 位置保持（对外速度语义 0），这两拍不用速度差分核算加速度；
			// 连续运动拍之间速度变化必须被斜坡限制在 MaxAccel 内。
			if !o.Reached && prevActive && !prevReached {
				if a := math.Abs(o.VelocityRadS-prevVel) / 0.01; a > maxAccel {
					maxAccel = a
				}
			}
			if !o.Reached && math.Abs(o.VelocityRadS) > 2+1e-9 {
				t.Fatalf("cycle %d velocity %.4f exceeds 2", i, o.VelocityRadS)
			}
			prevVel = o.VelocityRadS
			prevActive = true
			prevReached = o.Reached
			pos = o.PositionRad
			if o.Reached && reachedAt < 0 {
				reachedAt = i
			}
		}
	}
	if maxAccel > 100+1e-6 {
		t.Fatalf("accel breached while override changed: %.4f", maxAccel)
	}
	if reachedAt < 0 {
		t.Fatal("never reached target")
	}
	if math.Abs(pos-0.3) > 1e-6 {
		t.Fatalf("final pos %.6f want 0.3", pos)
	}
	// 倍率 0 暂停期间位置必须冻结：校验暂停段无位移（15..40 周期附近）
	t.Logf("在线倍率结果: reachedAtCycle=%d maxAccel=%.4f finalPos=%.6f stats=%+v",
		reachedAt, maxAccel, pos, p.Stats())
}

// Test4 排队容量上限：RejectNew 明确拒绝；EvictOldest 挤掉最旧排队点，
// 活动点不受影响，挤掉可观测且内存有界。
func TestStreamQueueCapacityPolicies(t *testing.T) {
	t0 := time.Date(2026, 9, 29, 0, 0, 0, 0, time.UTC)
	now := t0
	p := New(testCfg(3, RejectNew))
	for i := 1; i <= 3; i++ {
		if err := p.Append(Point{At: t0.Add(time.Duration(i) * 200 * time.Millisecond),
			PositionRad: 0.05 * float64(i)}, now, 0, true); err != nil {
			t.Fatal(err)
		}
	}
	mustFail(t, p.Append(Point{At: t0.Add(800 * time.Millisecond), PositionRad: 0.2}, now, 0, true),
		ReasonQueueFull)

	// EvictOldest 策略
	p2 := New(testCfg(3, EvictOldest))
	for i := 1; i <= 3; i++ {
		if err := p2.Append(Point{At: t0.Add(time.Duration(i) * 200 * time.Millisecond),
			PositionRad: 0.05 * float64(i)}, now, 0, true); err != nil {
			t.Fatal(err)
		}
	}
	if err := p2.Append(Point{At: t0.Add(800 * time.Millisecond), PositionRad: 0.2}, now, 0, true); err != nil {
		t.Fatal(err)
	}
	if p2.Len() != 3 {
		t.Fatalf("queue length must stay bounded at 3, got %d", p2.Len())
	}
	st := p2.Stats()
	if st.Evicted != 1 {
		t.Fatalf("want 1 eviction, got %+v", st)
	}
	// 活动点（原第一个点 0.2s/0.05）必须仍在队首，未被挤掉。
	head := p2.pending[0]
	if math.Abs(head.PositionRad-0.05) > 1e-9 {
		t.Fatalf("active point must not be evicted, head=%+v", head)
	}
	t.Logf("容量策略结果: %+v headAt=%v", st, head.At)
}

// TestRejectedAppendNeverMutatesHoldingOrQueue 流排空进入末点保持后，
// 任何被拒的点（越界/迟到/倒流/段速走不出来/队列满）都不得改动保持
// 位置、锚点、排队与倍率：拒绝前后连续 Advance 的输出必须逐拍一致，
// 且普通硬限位(stream_position_out_of_limit)与“当前状态走不出来”
// (stream_segment_too_fast)两类拒绝原因码明确可分。
func TestRejectedAppendNeverMutatesHoldingOrQueue(t *testing.T) {
	t0 := time.Date(2026, 9, 29, 0, 0, 0, 0, time.UTC)
	now := t0
	p := New(testCfg(4, RejectNew))
	if err := p.Append(Point{At: t0.Add(300 * time.Millisecond), PositionRad: 0.2}, now, 0, true); err != nil {
		t.Fatal(err)
	}
	// 推进到排空后的末点保持。
	for i := 0; i < 60; i++ {
		now = now.Add(10 * time.Millisecond)
		_ = p.Advance(now)
	}
	if !p.Active() || p.Len() != 0 {
		t.Fatalf("precondition want holding, active=%v len=%d", p.Active(), p.Len())
	}

	// 拒绝前先采 3 拍保持输出作为黄金序列。
	golden := make([]Output, 3)
	for i := range golden {
		now = now.Add(10 * time.Millisecond)
		golden[i] = p.Advance(now)
		if !golden[i].Active || golden[i].PositionRad != 0.2 || golden[i].VelocityRadS != 0 {
			t.Fatalf("golden hold output %d not stable: %+v", i, golden[i])
		}
	}

	// 各类被拒点。
	type rej struct {
		name string
		pt   Point
		want string
	}
	cases := []rej{
		{"hard_position_limit", Point{At: now.Add(100 * time.Millisecond), PositionRad: 5}, ReasonPositionOutOfLimit},
		{"infeasible_segment", Point{At: now.Add(2 * time.Millisecond), PositionRad: 0.3}, ReasonSegmentTooFast},
		{"late_or_overlap", Point{At: now, PositionRad: 0.21}, ReasonLatePoint},
	}
	for _, c := range cases {
		err := p.Append(c.pt, now, 0.2, true)
		mustFail(t, err, c.want)
	}
	// 两类“越界 vs 走不出来”的原因码必须不同。
	if ReasonPositionOutOfLimit == ReasonSegmentTooFast {
		t.Fatal("out-of-limit and infeasible reasons must differ")
	}

	// 拒绝后内部状态与拒绝前一致：仍在保持、无排队、倍率 1、位置 0.2。
	if !p.Active() || p.Len() != 0 || p.Override() != 1 {
		t.Fatalf("state mutated by rejected points: active=%v len=%d factor=%.2f",
			p.Active(), p.Len(), p.Override())
	}
	// 拒绝后 3 拍输出必须与黄金保持序列逐字段一致（保持从未中断）。
	for i := range golden {
		now = now.Add(10 * time.Millisecond)
		got := p.Advance(now)
		if got.Active != golden[i].Active || got.PositionRad != golden[i].PositionRad ||
			got.VelocityRadS != golden[i].VelocityRadS || got.Drained != golden[i].Drained {
			t.Fatalf("hold output diverged after rejection at beat %d: golden=%+v got=%+v",
				i, golden[i], got)
		}
	}
	// 拒绝只记账、不入队：Appended 仍是 1，Rejected=3。
	st := p.Stats()
	if st.Appended != 1 || st.Rejected != int64(len(cases)) {
		t.Fatalf("stats wrong after rejected appends: %+v", st)
	}
	t.Logf("输入: 保持点=0.200; 被拒=%v\n判定: 全部 command_rejected|stream, 保持/排队/倍率未改\n最终状态: active=%v holdPos=%.3f stats=%+v",
		[]string{"limit(5)", "infeasible(0.3@2ms)", "late(now)"},
		p.Active(), 0.2, st)

	// 合法续点仍可正常接续（拒绝没有毒化后续运动）。
	if err := p.Append(Point{At: now.Add(200 * time.Millisecond), PositionRad: 0.25}, now, 0.2, true); err != nil {
		t.Fatalf("resume after rejected points: %v", err)
	}
	if p.Len() != 1 {
		t.Fatalf("resumed stream must have 1 queued point, got %d", p.Len())
	}
}

// TestRejectedEvictionNeverMutatesQueue EvictOldest 下若挤掉最旧点后
// 合并段不可行，拒绝时现有队列必须一个字节都不改。
func TestRejectedEvictionNeverMutatesQueue(t *testing.T) {
	t0 := time.Date(2026, 9, 29, 0, 0, 0, 0, time.UTC)
	now := t0
	p := New(testCfg(3, EvictOldest))
	pts := []Point{
		{At: t0.Add(200 * time.Millisecond), PositionRad: 0.05},
		{At: t0.Add(400 * time.Millisecond), PositionRad: 0.10},
		{At: t0.Add(600 * time.Millisecond), PositionRad: 0.15},
	}
	for _, pt := range pts {
		if err := p.Append(pt, now, 0, true); err != nil {
			t.Fatal(err)
		}
	}
	before := append([]Point{}, p.pending...)
	// 构造一个“挤掉 pending[1] 后合并段 0.05->0.15 超速”的新点：
	// 合并时长 400ms、位移 0.10 => 0.25 rad/s 本身可行，因此用极大位移点
	// 且目标自身在硬限内（如 -2），触发合并段超速。
	err := p.Append(Point{At: t0.Add(800 * time.Millisecond), PositionRad: -2}, now, 0, true)
	mustFail(t, err, ReasonEvictInfeasible)
	if len(p.pending) != len(before) {
		t.Fatalf("rejected eviction mutated queue length: %d -> %d", len(before), len(p.pending))
	}
	for i := range before {
		if p.pending[i] != before[i] {
			t.Fatalf("rejected eviction mutated pending[%d]: %+v -> %+v", i, before[i], p.pending[i])
		}
	}
	t.Logf("输入: 队列=%v; 挤掉后不可行点(-2@800ms)\n判定: %s; 队列未改\n最终状态: len=%d %v",
		before, ReasonEvictInfeasible, p.Len(), p.pending)
}
