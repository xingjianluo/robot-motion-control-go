package safety

import (
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
	"github.com/xingjianluo/robot-motion-control-go/internal/feedback"
	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
	"github.com/xingjianluo/robot-motion-control-go/internal/trajectory"
)

func TestPriorityAndSuppression(t *testing.T) {
	t0 := time.Unix(0, 0)
	m := NewMachine(StateRun)

	// 低优先级不得覆盖高优先级状态
	m.TriggerEvent(t0, Trigger{Reason: ReasonSoftLimit})
	st, changed := m.TriggerEvent(t0.Add(time.Millisecond), Trigger{Reason: ReasonCommLoss})
	if !changed || st != StateCommLoss {
		t.Fatalf("higher priority must win: %s %v", st, changed)
	}
	st, changed = m.TriggerEvent(t0.Add(2*time.Millisecond), Trigger{Reason: ReasonSoftLimit})
	if changed || st != StateCommLoss {
		t.Fatalf("lower trigger must be suppressed: %s %v", st, changed)
	}
	if sup, _, ok := m.LastSuppressed(); !ok || sup != ReasonSoftLimit {
		t.Fatalf("suppression not observable: %s %v", sup, ok)
	}
	// 同级重复触发也不得再产生变迁
	if _, changed := m.TriggerEvent(t0.Add(3*time.Millisecond), Trigger{Reason: ReasonTorqueOverload}); changed {
		t.Fatal("same/lower priority repeated trigger must not transition")
	}
}

func TestEstopAlwaysWinsAndRecoveryConditions(t *testing.T) {
	t0 := time.Unix(0, 0)
	m := NewMachine(StateRun)
	m.TriggerEvent(t0, Trigger{Reason: ReasonSoftLimit})
	m.Estop(t0.Add(time.Millisecond), "button")
	if m.State() != StateEstop || m.OutputsEnabled() {
		t.Fatal("estop must cut outputs")
	}
	// 急停期间任何非恢复触发都无法覆盖
	m.TriggerEvent(t0.Add(2*time.Millisecond), Trigger{Reason: ReasonSoftLimitClear})
	if m.State() != StateEstop {
		t.Fatal("no trigger may leave ESTOP except explicit recovery")
	}

	// 条件不全 => 拒绝恢复并列出缺失项，绝不静默
	err := m.RequestRecover(t0.Add(3*time.Millisecond), RecoveryConditions{
		EstopReleased: true, CommHealthy: true,
	})
	fe, ok := fault.As(err)
	if !ok || fe.Reason != ReasonRecoveryDenied || fe.Category != fault.CatSafety {
		t.Fatalf("partial recovery: %v", err)
	}

	// 全部条件 + 人工确认 => 恢复且留痕
	if err := m.RequestRecover(t0.Add(4*time.Millisecond), RecoveryConditions{
		EstopReleased: true, CommHealthy: true, TorqueWithinLimit: true,
		PositionWithinLimit: true, FaultCodeClear: true, OperatorAck: true,
	}); err != nil {
		t.Fatal(err)
	}
	if m.State() != StateRun || !m.OutputsEnabled() {
		t.Fatal("recovery should enable outputs")
	}
	evs := m.Events()
	last := evs[len(evs)-1]
	if last.Reason != ReasonManualRecovery || last.From != StateEstop || last.To != StateRun {
		t.Fatalf("recovery event wrong: %+v", last)
	}
}

func TestSoftLimitExplicitClear(t *testing.T) {
	t0 := time.Unix(0, 0)
	m := NewMachine(StateRun)
	mon := NewMonitor(m, Limits{MinPositionRad: -1, MaxPositionRad: 1, MaxTorqueNm: 5, SoftMarginRad: 0.1})

	// 位置 0.95 在软限位带 [ -0.9, 0.9 ] 之外
	mon.Check(t0, feedback.Sample{PositionRad: 0.95, TorqueKnown: true})
	if m.State() != StateSoftLimit {
		t.Fatalf("want SOFT_LIMIT, got %s", m.State())
	}
	// 仍在带外：重复样本不得重复触发/抖动
	mon.Check(t0.Add(time.Millisecond), feedback.Sample{PositionRad: 0.96, TorqueKnown: true})
	nEvents := len(m.Events())
	mon.Check(t0.Add(2*time.Millisecond), feedback.Sample{PositionRad: 0.95, TorqueKnown: true})
	if len(m.Events()) != nEvents {
		t.Fatal("soft limit must latch while outside the band")
	}
	// 回到带内 => 显式清除事件，恢复 RUN（有留痕，非静默）
	mon.Check(t0.Add(3*time.Millisecond), feedback.Sample{PositionRad: 0.5, TorqueKnown: true})
	if m.State() != StateRun {
		t.Fatalf("soft clear: %s", m.State())
	}
	if last := m.Events()[len(m.Events())-1]; last.Reason != ReasonSoftLimitClear {
		t.Fatalf("clear event: %+v", last)
	}
}

func TestTorqueAndDriveFaultAndMissingTorqueField(t *testing.T) {
	t0 := time.Unix(0, 0)
	m := NewMachine(StateRun)
	mon := NewMonitor(m, Limits{MinPositionRad: -1, MaxPositionRad: 1, MaxTorqueNm: 5, SoftMarginRad: 0.1})

	// V1 反馈无力矩字段：不得按零力矩处理，也不得误触发
	mon.Check(t0, feedback.Sample{PositionRad: 0, TorqueKnown: false, TorqueNm: 99})
	if m.State() != StateRun {
		t.Fatalf("missing torque field must not trigger: %s", m.State())
	}
	// V2 明确报告超限力矩 => 锁存故障
	mon.Check(t0.Add(time.Millisecond), feedback.Sample{PositionRad: 0, TorqueKnown: true, TorqueNm: -6})
	if m.State() != StateTorqueFault {
		t.Fatalf("got %s", m.State())
	}

	m2 := NewMachine(StateRun)
	mon2 := NewMonitor(m2, Limits{MinPositionRad: -1, MaxPositionRad: 1, MaxTorqueNm: 5, SoftMarginRad: 0.1})
	mon2.Check(t0, feedback.Sample{PositionRad: 0, TorqueKnown: true, DriveFault: 7})
	if m2.State() != StateTorqueFault {
		t.Fatalf("drive fault code: %s", m2.State())
	}
}

func TestSetpointSoftLimitPreCheck(t *testing.T) {
	t0 := time.Unix(0, 0)
	m := NewMachine(StateRun)
	mon := NewMonitor(m, Limits{MinPositionRad: -1, MaxPositionRad: 1, MaxTorqueNm: 5, SoftMarginRad: 0.1})

	reasons := mon.CheckSetpoint(t0, trajectory.Setpoint{Mode: protocol.ModePosition, PositionRad: 0.95})
	if len(reasons) != 1 || reasons[0] != ReasonSoftLimit || m.State() != StateSoftLimit {
		t.Fatalf("pre-check: %v state=%s", reasons, m.State())
	}
}
