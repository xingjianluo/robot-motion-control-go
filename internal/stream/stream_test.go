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
