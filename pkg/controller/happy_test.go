package controller_test

import (
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/pkg/bus"
	clkpkg "github.com/xingjianluo/robot-motion-control-go/pkg/clock"
	"github.com/xingjianluo/robot-motion-control-go/pkg/controller"
	"github.com/xingjianluo/robot-motion-control-go/pkg/feedback"
	"github.com/xingjianluo/robot-motion-control-go/pkg/frame"
	"github.com/xingjianluo/robot-motion-control-go/pkg/limits"
	"github.com/xingjianluo/robot-motion-control-go/pkg/safety"
)

func testSetup(t *testing.T, v frame.Version) (*controller.Controller, *fakeDrive, *clkpkg.Manual, *bus.Bus, time.Time) {
	t.Helper()
	start := time.Date(2026, 9, 22, 10, 0, 0, 0, time.UTC)
	clk := clkpkg.NewManual(start)
	b := bus.New(bus.Config{Capacity: 32, Policy: bus.DropOldest})
	period := 10 * time.Millisecond
	cfg := controller.Config{
		Joints: twoJointLimits(), Period: period, QueueDepth: 8,
		DropStrategy: controller.RejectNew, Protocol: v,
		Feedback: feedback.Config{
			StaleTimeout: 35 * time.Millisecond, LostTimeout: 80 * time.Millisecond,
			MaxPosJump: 0.2, MaxForwardSkew: time.Second,
		},
		MaxTrackingErr: 0.3, Clk: clk,
	}
	c, err := controller.New(cfg, b)
	if err != nil {
		t.Fatalf("controller.New: %v", err)
	}
	d := newFakeDrive(b, v, len(twoJointLimits()))
	return c, d, clk, b, start
}

// TestHappyPath 位置命令连续下发：轨迹逐周期推进、约束始终满足、最终到位。
func TestHappyPath(t *testing.T) {
	c, d, clk, _, _ := testSetup(t, frame.V2)
	const period = 10 * time.Millisecond

	t.Log("输入: joint0 位置目标 1.0 rad（限位 [-1.57,1.57], vmax=1, amax=2）")
	if err := c.SubmitCommand(limits.Command{
		Joint: 0, Mode: limits.ModePosition, Position: 1.0,
	}); err != nil {
		t.Fatalf("提交被拒: %v", err)
	}

	maxVel, maxAcc := 0.0, 0.0
	prevRef, prevVel := 0.0, 0.0
	finalSnap := controller.Snapshot{}
	for k := 0; k < 200; k++ {
		now := clk.Now()
		d.respond(now, nil, false)
		d.emit(t, now)
		snap, err := c.Tick()
		if err != nil {
			t.Fatalf("tick: %v", err)
		}
		finalSnap = snap
		ref := snap.Joints[0].RefPosition
		vel := (ref - prevRef) / period.Seconds()
		acc := (vel - prevVel) / period.Seconds()
		if a := abs(vel); a > maxVel {
			maxVel = a
		}
		if a := abs(acc); a > maxAcc {
			maxAcc = a
		}
		if ref < -1.57-1e-9 || ref > 1.57+1e-9 {
			t.Fatalf("参考位置越硬限位: %.4f", ref)
		}
		if snap.SafetyState != safety.StateOperational {
			t.Fatalf("期望 OPERATIONAL，实际 %s（%s）",
				snap.SafetyState, snap.SafetyReason.Detail)
		}
		prevRef, prevVel = ref, vel
		clk.Advance(period)
		if snap.Joints[0].TrajDone {
			break
		}
	}

	if !finalSnap.Joints[0].TrajDone {
		t.Fatalf("轨迹未在限定周期内完成")
	}
	if e := abs(finalSnap.Joints[0].RefPosition - 1.0); e > 1e-6 {
		t.Fatalf("最终参考位置 %.6f != 1.0", finalSnap.Joints[0].RefPosition)
	}
	// 允许一点离散化裕量
	if maxVel > 1.0+1e-9 {
		t.Fatalf("速度约束被突破: vmax=%.4f > 1.0", maxVel)
	}
	if maxAcc > 2.0+1e-6 {
		t.Fatalf("加速度约束被突破: amax=%.4f > 2.0", maxAcc)
	}
	t.Logf("判定: 全程 OPERATIONAL；实测最大速度=%.4f<=1.0 最大加速度=%.4f<=2.0",
		maxVel, maxAcc)
	t.Logf("最终状态: ref=%.6f fb=%.6f safety=%s 周期=%d",
		finalSnap.Joints[0].RefPosition, finalSnap.Joints[0].FbPosition,
		finalSnap.SafetyState, finalSnap.Cycle)
}

func abs(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}
