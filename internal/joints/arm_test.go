package joints_test

import (
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/bus"
	"github.com/xingjianluo/robot-motion-control-go/internal/clock"
	"github.com/xingjianluo/robot-motion-control-go/internal/control"
	"github.com/xingjianluo/robot-motion-control-go/internal/feedback"
	"github.com/xingjianluo/robot-motion-control-go/internal/joints"
	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
	"github.com/xingjianluo/robot-motion-control-go/internal/safety"
	"github.com/xingjianluo/robot-motion-control-go/internal/trajectory"
)

func jointCfg() control.Config {
	return control.Config{
		ProtocolVersion: protocol.V2, Period: 10 * time.Millisecond,
		JitterTolerance: 15 * time.Millisecond, MaxBacklog: 8, TrajCapacity: 16,
		Motion: trajectory.Limits{MinPositionRad: -2, MaxPositionRad: 2,
			MaxVelocityRadS: 2, MaxAccelRadS2: 200, MaxTorqueNm: 5},
		SafetyLim: safety.Limits{MinPositionRad: -2, MaxPositionRad: 2,
			MaxTorqueNm: 5, SoftMarginRad: 0.1},
		Feedback: feedback.Config{WatchdogTimeout: 35 * time.Millisecond, MaxStepRad: 1, MaxBackdateMs: 5},
		Bus:      bus.Config{Capacity: 16, Policy: bus.DropOldest},
	}
}

func TestAddAndUnknownJoint(t *testing.T) {
	fc := clock.NewFake(time.Unix(0, 0))
	arm := joints.NewArm(fc)
	if _, err := arm.AddJoint("j1", jointCfg()); err != nil {
		t.Fatal(err)
	}
	if _, err := arm.AddJoint("j1", jointCfg()); err == nil {
		t.Fatal("duplicate joint name must be rejected")
	}
	if err := arm.Submit("nope", trajectory.Command{}); err == nil {
		t.Fatal("unknown joint submit must error")
	}
}

// 任一关节进入 ESTOP，同一周期边界结束后所有关节必须都 ESTOP，
// 不允许出现“有的 RUN、有的停”的半更新。
func TestInterlockedEstopAcrossJoints(t *testing.T) {
	fc := clock.NewFake(time.Unix(0, 0))
	arm := joints.NewArm(fc)
	for _, n := range []string{"j1", "j2", "j3"} {
		if _, err := arm.AddJoint(n, jointCfg()); err != nil {
			t.Fatal(err)
		}
	}
	cond := safety.RecoveryConditions{
		EstopReleased: true, CommHealthy: true, TorqueWithinLimit: true,
		PositionWithinLimit: true, FaultCodeClear: true, OperatorAck: true,
	}
	// 全部恢复
	if err := arm.RecoverAll(cond); err != nil {
		t.Fatal(err)
	}
	for _, s := range arm.Snapshot() {
		if s.SafetyState != safety.StateRun {
			t.Fatalf("all should RUN, got %s", s.SafetyState)
		}
	}
	// 中间关节急停；推进一个共享周期边界
	if c, ok := arm.Joint("j2"); !ok {
		t.Fatal("j2 missing")
	} else {
		c.Estop("j2 local stop")
	}
	src := arm.StepAll("arm interlock")
	if src != "j2" {
		t.Fatalf("estop source = %q, want j2", src)
	}
	for name, s := range arm.Snapshot() {
		if s.SafetyState != safety.StateEstop || s.OutputsOn {
			t.Fatalf("joint %s not interlocked: %s", name, s.SafetyState)
		}
	}
}

// 恢复是全有或全无：一个关节条件不满足，则所有关节都不得停留在 RUN。
func TestRecoverAllOrNothing(t *testing.T) {
	fc := clock.NewFake(time.Unix(0, 0))
	arm := joints.NewArm(fc)
	for _, n := range []string{"j1", "j2"} {
		if _, err := arm.AddJoint(n, jointCfg()); err != nil {
			t.Fatal(err)
		}
	}
	bad := safety.RecoveryConditions{EstopReleased: true} // 条件不全
	if err := arm.RecoverAll(bad); err == nil {
		t.Fatal("partial recovery must be rejected")
	}
	for name, s := range arm.Snapshot() {
		if s.SafetyState != safety.StateEstop {
			t.Fatalf("joint %s must remain ESTOP, got %s", name, s.SafetyState)
		}
	}
}
