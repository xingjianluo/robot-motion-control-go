package control_test

import (
	"bytes"
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/bus"
	"github.com/xingjianluo/robot-motion-control-go/internal/clock"
	"github.com/xingjianluo/robot-motion-control-go/internal/control"
	"github.com/xingjianluo/robot-motion-control-go/internal/device"
	"github.com/xingjianluo/robot-motion-control-go/internal/feedback"
	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
	"github.com/xingjianluo/robot-motion-control-go/internal/report"
	"github.com/xingjianluo/robot-motion-control-go/internal/safety"
	"github.com/xingjianluo/robot-motion-control-go/internal/trajectory"
)

// TestExplainableDecisionFlow 是“输入 -> 判定依据 -> 最终状态”
// 全链路可解释用例：每一步都打印输入条件、系统给出的归类原因与
// 最终状态快照，便于人工复核与离线复现。
func TestExplainableDecisionFlow(t *testing.T) {
	var buf bytes.Buffer
	t0 := time.Date(2026, 9, 22, 0, 0, 0, 0, time.UTC)
	fc := clock.NewFake(t0)
	period := 10 * time.Millisecond
	b := bus.New(bus.Config{Capacity: 16, Policy: bus.DropOldest, MaxAge: 200 * time.Millisecond}, fc)
	drv := device.New(device.Config{
		Version: protocol.V2, DT: period, Gain: 40, MaxVel: 2,
		HardMin: -2, HardMax: 2,
	}, fc, b)

	ctl, err := control.New(control.Config{
		ProtocolVersion: protocol.V2, Period: period, JitterTolerance: 15 * time.Millisecond,
		MaxBacklog: 8, TrajCapacity: 32,
		Motion: trajectory.Limits{MinPositionRad: -2, MaxPositionRad: 2,
			MaxVelocityRadS: 2, MaxAccelRadS2: 200, MaxTorqueNm: 5},
		SafetyLim: safety.Limits{MinPositionRad: -2, MaxPositionRad: 2,
			MaxTorqueNm: 5, SoftMarginRad: 0.1},
		Feedback: feedback.Config{WatchdogTimeout: 35 * time.Millisecond, MaxStepRad: 1, MaxBackdateMs: 5},
		Bus:      bus.Config{Capacity: 16, Policy: bus.DropOldest, MaxAge: 200 * time.Millisecond},
	}, fc, b)
	if err != nil {
		t.Fatal(err)
	}
	rec := report.NewRecorder(&buf)
	ctl.SetEventSink(rec)

	step := func(tag string) {
		if derr := drv.Tick(); derr != nil {
			t.Logf("[%s] 设备帧丢失(注入/链路): %v", tag, derr)
		}
		fc.Advance(period)
		if serr := ctl.Step(); serr != nil {
			t.Fatal(serr)
		}
	}

	// 输入1：上电即提交位置命令
	t.Log("输入: 上电默认 ESTOP + 位置命令 pos=0.5")
	if _, err := ctl.SubmitCommand(trajectory.Command{Mode: protocol.ModePosition, Position: 0.5}); err != nil {
		t.Logf("判定: 拒绝，依据(类别|来源|原因) = %v", err)
	}
	t.Logf("最终状态: state=%s outputs=%v\n", ctl.Snapshot().SafetyState, ctl.Snapshot().OutputsOn)

	// 输入2：恢复条件不全
	t.Log("输入: 恢复但缺少大部分前置条件")
	if err := ctl.Recover(safety.RecoveryConditions{EstopReleased: true}); err != nil {
		t.Logf("判定: %v", err)
	}

	// 输入3：建立反馈后满足全部条件
	step("预热1")
	step("预热2")
	cond := safety.RecoveryConditions{
		EstopReleased: true, CommHealthy: true, TorqueWithinLimit: true,
		PositionWithinLimit: true, FaultCodeClear: true, OperatorAck: true,
	}
	if err := ctl.Recover(cond); err != nil {
		t.Fatal(err)
	}
	t.Log("输入: 全部恢复条件满足 + 人工确认")
	t.Logf("最终状态: state=%s outputs=%v healthy=%v\n",
		ctl.Snapshot().SafetyState, ctl.Snapshot().OutputsOn, ctl.Snapshot().Tracking.Healthy)

	// 输入4：合法速度轨迹，连续 5 拍
	t.Log("输入: 连续速度命令 vel=1 rad/s × 5 周期")
	for i := 0; i < 5; i++ {
		if _, err := ctl.SubmitCommand(trajectory.Command{
			Mode: protocol.ModeVelocity, Velocity: 1, IssuedAt: fc.Now(),
		}); err != nil {
			t.Fatal(err)
		}
		step("匀速")
	}
	s4 := ctl.Snapshot()
	t.Logf("判定: 全部接受(feedbackAccepted)；最终状态: %s 速度误差=%.4f 设备位置=%.3f (期望≈0.05)\n",
		report.FmtSnapshot(s4), s4.Tracking.ErrorRadS, drv.Position())

	// 输入5：超加速命令必须被拒绝（原因不同于速度越限）。
	// 注：速度模式下 IssuedAt 为零也能做加速度核算（按当前时刻），
	// 这里显式给出 5ms 前的时间戳使所需加速度=|2.5-1|/0.005=300 > 200。
	t.Log("输入: 5ms 内速度 1->2.5 rad/s，所需加速度 300 > 上限 200 rad/s²")
	_, err = ctl.SubmitCommand(trajectory.Command{
		Mode: protocol.ModeVelocity, Velocity: 2.5,
		IssuedAt: fc.Now().Add(-5 * time.Millisecond),
	})
	if err != nil {
		t.Logf("判定: %v", err)
	} else {
		t.Fatal("超加速命令必须被拒绝")
	}

	// 输入6：急停
	t.Log("输入: 人工急停")
	ctl.Estop("explain test")
	t.Logf("最终状态: %s\n", report.FmtSnapshot(ctl.Snapshot()))

	// 汇总输出到测试日志：输入/依据/状态的完整审计轨迹
	rec.SyncSafety(ctl.SafetyEvents())
	rec.Dump()
	t.Log("\n" + buf.String())
}
