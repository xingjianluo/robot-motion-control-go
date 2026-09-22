package control_test

import (
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
	"github.com/xingjianluo/robot-motion-control-go/internal/trajectory"
)

type rig struct {
	t   *testing.T
	fc  *clock.Fake
	bus *bus.Bus
	drv *device.Drive
	rec *report.Recorder
	ctl *control.Controller
	t0  time.Time
}

func newRig(t *testing.T, ver protocol.Version, busCfg bus.Config) *rig {
	t.Helper()
	t0 := time.Date(2026, 9, 22, 0, 0, 0, 0, time.UTC)
	fc := clock.NewFake(t0)
	if busCfg.Capacity == 0 {
		busCfg.Capacity = 64
	}
	b := bus.New(busCfg, fc)
	drv := device.New(device.Config{
		Version: ver, DT: 10 * time.Millisecond, Gain: 30,
		MaxVel: 2, HardMin: -2, HardMax: 2,
	}, fc, b)
	cfg := control.Config{
		ProtocolVersion: ver,
		Period:          10 * time.Millisecond,
		JitterTolerance: 15 * time.Millisecond,
		MaxBacklog:      8,
		TrajCapacity:    32,
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
		Bus: busCfg,
	}
	ctl, err := control.New(cfg, fc, b)
	if err != nil {
		t.Fatal(err)
	}
	rec := report.NewRecorder(&testWriter{t: t})
	ctl.SetEventSink(rec)
	return &rig{t: t, fc: fc, bus: b, drv: drv, rec: rec, ctl: ctl, t0: t0}
}

type testWriter struct{ t *testing.T }

func (w *testWriter) Write(p []byte) (int, error) { w.t.Log(string(p)); return len(p), nil }

func (r *rig) tick() {
	if err := r.drv.Tick(); err != nil {
		r.t.Fatalf("drive tick: %v", err)
	}
	r.fc.Advance(10 * time.Millisecond)
	if err := r.ctl.Step(); err != nil {
		panic(err)
	}
}

func fullRecover(estopReleased bool) safety.RecoveryConditions {
	return safety.RecoveryConditions{
		EstopReleased:       estopReleased,
		CommHealthy:         true,
		TorqueWithinLimit:   true,
		PositionWithinLimit: true,
		FaultCodeClear:      true,
		OperatorAck:         true,
	}
}

func mustReason(t *testing.T, err error, want string) *fault.Error {
	t.Helper()
	fe, ok := fault.As(err)
	if !ok || fe.Reason != want {
		t.Fatalf("want reason %q, got %v", want, err)
	}
	return fe
}
