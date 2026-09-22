package controller_test

import (
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/pkg/feedback"
	"github.com/xingjianluo/robot-motion-control-go/pkg/frame"
	"github.com/xingjianluo/robot-motion-control-go/pkg/safety"
)

// TestTimestampJump 反馈时间戳前跳超过阈值 -> TIMESTAMP_JUMP，归为链路异常。
func TestTimestampJump(t *testing.T) {
	c, d, clk, _, _ := testSetup(t, frame.V2)
	warmup(t, c, d, clk, 3)

	fb := frame.FeedbackData{
		Joint: 0, Seq: d.peekNextSeq(0),
		Timestamp: clk.Now().Add(5 * time.Second).UnixNano(),
		Position:  d.peekPos(0),
	}
	d.emitCustom(t, fb)
	snap, _ := c.Tick()
	if snap.Joints[0].FbCondition != feedback.CondTimestampJump {
		t.Fatalf("期望 TIMESTAMP_JUMP，实际 %s（%s）",
			snap.Joints[0].FbCondition, snap.Joints[0].FbReason)
	}
	if snap.SafetyState != safety.StateWatchdog {
		t.Fatalf("时间戳跳变应判 WATCHDOG，实际 %s", snap.SafetyState)
	}
	if snap.Joints[0].TrackingError != 0 {
		t.Fatal("时间戳异常帧不得计入跟踪误差")
	}
	t.Logf("判定: %s（%s），未当作跟踪误差",
		snap.Joints[0].FbCondition, snap.Joints[0].FbReason)
}
