package trajectory

import (
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
)

func testLimits() Limits {
	return Limits{
		MinPositionRad: -1.0, MaxPositionRad: 1.0,
		MaxVelocityRadS: 2.0, MaxAccelRadS2: 10.0, MaxTorqueNm: 5.0,
	}
}

func TestRejectReasonsDistinguishable(t *testing.T) {
	v := NewValidator(testLimits())
	t0 := time.Unix(0, 0)

	cases := []struct {
		name   string
		cmd    Command
		reason string
	}{
		{"position high", Command{Mode: protocol.ModePosition, Position: 1.5, IssuedAt: t0}, ReasonPositionOutOfLimit},
		{"position low", Command{Mode: protocol.ModePosition, Position: -1.5, IssuedAt: t0}, ReasonPositionOutOfLimit},
		{"velocity", Command{Mode: protocol.ModeVelocity, Velocity: 3.0, IssuedAt: t0}, ReasonVelocityOutOfLimit},
		{"torque", Command{Mode: protocol.ModeTorque, Torque: -9.0, IssuedAt: t0}, ReasonTorqueOutOfLimit},
		{"unknown mode", Command{Mode: "fly", IssuedAt: t0}, ReasonContradictoryMode},
		{"contradiction", Command{Mode: protocol.ModePosition, Position: 0.1, Velocity: 0.2, IssuedAt: t0}, ReasonContradictoryMode},
	}
	for _, c := range cases {
		if _, err := v.Validate(c.cmd); err == nil {
			t.Fatalf("%s: expected rejection", c.name)
		} else {
			fe, ok := fault.As(err)
			if !ok || fe.Reason != c.reason || fe.Category != fault.CatCommandReject {
				t.Fatalf("%s: got %v", c.name, err)
			}
		}
	}
}

func TestAccelContinuityAccepted(t *testing.T) {
	v := NewValidator(testLimits())
	t0 := time.Unix(0, 0)
	// 第一周期静止 -> 1 rad/s，dt=0.2s => accel=5 <= 10 接受
	sp1, err := v.Validate(Command{Mode: protocol.ModeVelocity, Velocity: 1, IssuedAt: t0})
	if err != nil {
		t.Fatal(err)
	}
	// dt=0.05s 内跳到 2 rad/s => accel=20 > 10 拒绝（原因不同于速度超限）
	_, err = v.Validate(Command{Mode: protocol.ModeVelocity, Velocity: 2, IssuedAt: t0.Add(50 * time.Millisecond)})
	if fe, ok := fault.As(err); !ok || fe.Reason != ReasonAccelOutOfLimit {
		t.Fatalf("want accel reject, got %v", err)
	}
	// 同一时刻重试一个温和目标 => accel 以“最后接受的目标”为基准：
	// (1.2-1)/0.1 = 2 <= 10 接受
	sp2, err := v.Validate(Command{Mode: protocol.ModeVelocity, Velocity: 1.2, IssuedAt: t0.Add(100 * time.Millisecond)})
	if err != nil || sp2.Seq != sp1.Seq+1 {
		t.Fatalf("gentle target should be accepted: %v", err)
	}
}

func TestNonMonotonicTimestamp(t *testing.T) {
	v := NewValidator(testLimits())
	t0 := time.Unix(0, 0)
	if _, err := v.Validate(Command{Mode: protocol.ModePosition, Position: 0.1, IssuedAt: t0.Add(time.Second)}); err != nil {
		t.Fatal(err)
	}
	_, err := v.Validate(Command{Mode: protocol.ModePosition, Position: 0.2, IssuedAt: t0})
	if fe, ok := fault.As(err); !ok || fe.Reason != ReasonNonMonotonicTime {
		t.Fatalf("want non-monotonic reject, got %v", err)
	}
}

func TestBufferBoundedAndDue(t *testing.T) {
	buf := NewBuffer(2)
	t0 := time.Unix(0, 0)
	mk := func(seq uint64, at time.Time) BufferedPoint {
		return BufferedPoint{Setpoint: Setpoint{Seq: seq, Mode: protocol.ModePosition, PositionRad: 0.1}, At: at}
	}
	if err := buf.Append(mk(1, t0)); err != nil {
		t.Fatal(err)
	}
	if err := buf.Append(mk(2, t0.Add(10*time.Millisecond))); err != nil {
		t.Fatal(err)
	}
	if err := buf.Append(mk(3, t0.Add(20*time.Millisecond))); err == nil {
		t.Fatal("buffer overflow must be rejected")
	} else if fe, _ := fault.As(err); fe.Reason != ReasonBufferOverflow {
		t.Fatalf("got %v", err)
	}
	if err := buf.Append(mk(9, t0.Add(5*time.Millisecond))); err == nil {
		t.Fatal("earlier waypoint must be rejected")
	}

	// 同一执行时刻（最后一个航点）=> 替换而非并存。
	buf2 := NewBuffer(2)
	if err := buf2.Append(mk(1, t0)); err != nil {
		t.Fatal(err)
	}
	if err := buf2.Append(mk(8, t0)); err != nil {
		t.Fatalf("same-time waypoint should replace: %v", err)
	}
	if buf2.Len() != 1 {
		t.Fatal("same-time append must not grow buffer")
	}
	if p, _, _ := buf2.PopDue(t0); p.Seq != 8 {
		t.Fatalf("same-time replacement wrong: seq=%d", p.Seq)
	}

	p, skipped, ok := buf.PopDue(t0.Add(15 * time.Millisecond))
	if !ok || p.Seq != 2 || skipped != 1 {
		t.Fatalf("pop due = %+v skipped=%d ok=%v", p, skipped, ok)
	}
	buf.Clear()
	if buf.Len() != 0 {
		t.Fatal("clear failed")
	}
}
