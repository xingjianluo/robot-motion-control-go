package controller_test

import (
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/pkg/controller"
	"github.com/xingjianluo/robot-motion-control-go/pkg/feedback"
	"github.com/xingjianluo/robot-motion-control-go/pkg/frame"
	"github.com/xingjianluo/robot-motion-control-go/pkg/limits"
	"github.com/xingjianluo/robot-motion-control-go/pkg/safety"
)

const period10 = 10 * time.Millisecond

func warmup(t *testing.T, c *controller.Controller, d *fakeDrive,
	clk *clkT, cycles int) {
	t.Helper()
	for i := 0; i < cycles; i++ {
		now := clk.Now()
		d.respond(now, nil, false)
		d.emit(t, now)
		if _, err := c.Tick(); err != nil {
			t.Fatal(err)
		}
		clk.Advance(period10)
	}
}

// 包装 clk 类型名以缩短测试代码
type clkT = clkAlias
type clkAlias = manualClock

type manualClock = clockManual

// TestFeedbackDuplicateAndReorder 重复帧与序号回退：
// 必须被判为链路异常（WATCHDOG），且不得当作跟踪误差（FAULT_STOP/TRACKING）。
func TestFeedbackDuplicateAndReorder(t *testing.T) {
	c, d, clk, _, _ := testSetup(t, frame.V2)
	warmup(t, c, d, clk, 5)

	// 注入重复帧
	d.emitDuplicate(t, clk.Now(), 0)
	snap, _ := c.Tick()
	if snap.Joints[0].FbCondition != feedback.CondDuplicate {
		t.Fatalf("期望 DUPLICATE，实际 %s", snap.Joints[0].FbCondition)
	}
	if snap.SafetyState != safety.StateWatchdog {
		t.Fatalf("链路异常应进入 WATCHDOG，实际 %s", snap.SafetyState)
	}
	t.Logf("输入重复帧 seq -> 判定 %s：%s；安全=%s",
		snap.Joints[0].FbCondition, snap.Joints[0].FbReason, snap.SafetyState)

	clk.Advance(period10)
	// 序号回退
	d.emitOldSeq(t, clk.Now(), 0, 1)
	snap, _ = c.Tick()
	if snap.Joints[0].FbCondition != feedback.CondOutOfOrder {
		t.Fatalf("期望 OUT_OF_ORDER，实际 %s", snap.Joints[0].FbCondition)
	}
	// 跟踪误差必须保持 0（异常帧不采信）
	if snap.Joints[0].TrackingError != 0 {
		t.Fatalf("异常帧不得计入跟踪误差，实际 %.4f", snap.Joints[0].TrackingError)
	}
	t.Logf("输入旧序号帧 -> 判定 %s：%s；跟踪误差保持 %.2f（未被误判）",
		snap.Joints[0].FbCondition, snap.Joints[0].FbReason,
		snap.Joints[0].TrackingError)
}

// TestFeedbackLossStages 连续丢帧：先 STALE（暂时失联），再 LOST（丢失），
// 恢复后显式复位才能运动。
func TestFeedbackLossStages(t *testing.T) {
	c, d, clk, _, _ := testSetup(t, frame.V2)
	warmup(t, c, d, clk, 5)

	// 停止 joint0 的反馈（持续丢帧）
	var sawStale, sawLost bool
	for k := 0; k < 12; k++ {
		now := clk.Now()
		d.respond(now, nil, false)
		// joint0 丢帧，joint1 正常
		d.dropNext(0)
		d.emit(t, now)
		snap, _ := c.Tick()
		ageMs := snap.Joints[0].FbCondition
		switch ageMs {
		case feedback.CondStale:
			sawStale = true
		case feedback.CondLost:
			sawLost = true
		}
		if snap.SafetyState == safety.StateWatchdog &&
			snap.Joints[0].FbCondition == feedback.CondStale {
			t.Logf("周期 %d: 年龄超看门狗窗口 -> STALE（暂时失联，区别于真实偏差）",
				snap.Cycle)
		}
		clk.Advance(period10)
	}
	if !sawStale || !sawLost {
		t.Fatalf("应依次经历 STALE/LOST，得到 stale=%v lost=%v", sawStale, sawLost)
	}
	snap, _ := c.Tick()
	// Tick 重复一次不会清状态；检查提交命令被安全状态拒绝
	err := c.SubmitCommand(limits.Command{Joint: 0, Mode: limits.ModePosition, Position: 0.1})
	if err == nil {
		t.Fatal("WATCHDOG 状态必须拒绝运动命令")
	}
	t.Logf("丢失后提交命令被拒：%v", err)

	// 反馈恢复，但锁存不会静默清除
	d.respond(clk.Now(), nil, false)
	d.emit(t, clk.Now())
	clk.Advance(period10)
	d.respond(clk.Now(), nil, false)
	d.emit(t, clk.Now())
	snap, _ = c.Tick()
	if snap.SafetyState != safety.StateWatchdog {
		t.Fatalf("故障锁存，反馈恢复后仍应 WATCHDOG，实际 %s", snap.SafetyState)
	}
	if err := c.Reset(); err != nil {
		t.Fatalf("条件已消失，Reset 应成功: %v", err)
	}
	snap = c.Snapshot()
	if snap.SafetyState != safety.StateOperational {
		t.Fatalf("Reset 后应 OPERATIONAL，实际 %s", snap.SafetyState)
	}
	t.Logf("反馈恢复 + 显式 Reset 后最终状态=%s（未静默恢复）", snap.SafetyState)
}

// TestRealTrackingErrorVsLoss 真实位置偏差必须归类为 TRACKING_ERROR ->
// FAULT_STOP，而与 WATCHDOG（失联）可区分。
func TestRealTrackingErrorVsLoss(t *testing.T) {
	c, d, clk, _, _ := testSetup(t, frame.V2)
	// 先下发一个 0 位保持命令，使关节进入位置模式（跟踪监测仅在有命令时生效）
	if err := c.SubmitCommand(limits.Command{
		Joint: 0, Mode: limits.ModePosition, Position: 0,
	}); err != nil {
		t.Fatal(err)
	}
	warmup(t, c, d, clk, 5)

	t.Log("输入: 给 joint0 斜坡注入位置偏差至 0.5rad（每帧 +0.05，不触发跳变）")
	bias := 0.0
	var snap controller.Snapshot
	for k := 0; k < 12; k++ {
		if k < 10 {
			bias += 0.05
			d.setBias(0, bias)
		}
		now := clk.Now()
		d.respond(now, nil, false)
		d.emit(t, now)
		snap, _ = c.Tick()
		clk.Advance(period10)
	}
	if snap.Joints[0].FbCondition != feedback.CondOK {
		t.Fatalf("反馈帧应始终有效，实际 %s（%s）",
			snap.Joints[0].FbCondition, snap.Joints[0].FbReason)
	}
	if snap.SafetyState != safety.StateFaultStop {
		t.Fatalf("真实偏差应进入 FAULT_STOP，实际 %s（%s）",
			snap.SafetyState, snap.SafetyReason.Detail)
	}
	if snap.SafetyReason.Kind != "TRACKING_ERROR" {
		t.Fatalf("故障来源应为 TRACKING_ERROR，实际 %s", snap.SafetyReason.Kind)
	}
	t.Logf("判定: 反馈 %s 但跟踪误差=%.3f -> %s/%s（与失联 WATCHDOG 明确区分）",
		snap.Joints[0].FbCondition, snap.Joints[0].TrackingError,
		snap.SafetyState, snap.SafetyReason.Kind)
}

// TestPositionJump 编码器位置跳变不得被当作跟踪误差处理。
func TestPositionJump(t *testing.T) {
	c, d, clk, _, _ := testSetup(t, frame.V2)
	warmup(t, c, d, clk, 5)

	t.Log("输入: joint0 编码器读数单周期从 0 跳到 1.0 rad（阈值 0.2）")
	d.jump(0, 1.0)
	now := clk.Now()
	d.respond(now, nil, false)
	d.emit(t, now)
	snap, _ := c.Tick()
	if snap.Joints[0].FbCondition != feedback.CondPositionJump {
		t.Fatalf("期望 POSITION_JUMP，实际 %s", snap.Joints[0].FbCondition)
	}
	if snap.Joints[0].TrackingError != 0 {
		t.Fatalf("跳变帧不得计入跟踪误差，实际 %.4f",
			snap.Joints[0].TrackingError)
	}
	if snap.SafetyState != safety.StateWatchdog {
		t.Fatalf("跳变属链路不可信，应 WATCHDOG，实际 %s", snap.SafetyState)
	}
	t.Logf("判定: %s（%s）；跟踪误差保持 %.2f，安全=%s",
		snap.Joints[0].FbCondition, snap.Joints[0].FbReason,
		snap.Joints[0].TrackingError, snap.SafetyState)
}
