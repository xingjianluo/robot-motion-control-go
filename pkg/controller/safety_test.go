package controller_test

import (
	"testing"

	"github.com/xingjianluo/robot-motion-control-go/pkg/controller"
	"github.com/xingjianluo/robot-motion-control-go/pkg/frame"
	"github.com/xingjianluo/robot-motion-control-go/pkg/limits"
	"github.com/xingjianluo/robot-motion-control-go/pkg/safety"
)

// TestEStopPriorityAndRecovery 急停高于一切；释放后若其他故障仍在，不静默恢复；
// 故障未消失时 Reset 必须被拒绝并给出原因。
func TestEStopPriorityAndRecovery(t *testing.T) {
	c, d, clk, _, _ := testSetup(t, frame.V2)
	warmup(t, c, d, clk, 5)

	// 1) 急停
	if err := c.EStop("操作员拍下急停"); err != nil {
		t.Fatal(err)
	}
	snap := c.Snapshot()
	if snap.SafetyState != safety.StateEStop {
		t.Fatalf("期望 ESTOP，实际 %s", snap.SafetyState)
	}
	err := c.SubmitCommand(limits.Command{Joint: 0, Mode: limits.ModePosition, Position: 0.1})
	if err == nil {
		t.Fatal("急停期间必须拒绝命令")
	}
	t.Logf("急停后命令被拒: %v", err)

	// 2) 未释放急停直接 Reset -> 拒绝
	if err := c.Reset(); err == nil {
		t.Fatal("急停未释放时 Reset 必须失败")
	} else {
		t.Logf("Reset 被拒（条件未满足）: %v", err)
	}

	// 3) 释放急停；先让 joint0 持续丢失并跑若干周期，使看门狗锁存，
	//    释放后必须停留在 WATCHDOG，而不是静默恢复
	for k := 0; k < 9; k++ {
		d.dropNext(0)
		now := clk.Now()
		d.respond(now, nil, false)
		d.emit(t, now)
		if _, err := c.Tick(); err != nil {
			t.Fatal(err)
		}
		clk.Advance(period10)
	}
	if err := c.ReleaseEStop(); err != nil {
		t.Fatalf("释放急停失败: %v", err)
	}
	// 释放后立刻再评估一个周期：关节 0 仍在丢帧，Active 必须保持成立
	d.dropNext(0)
	now := clk.Now()
	d.respond(now, nil, false)
	d.emit(t, now)
	if _, err := c.Tick(); err != nil {
		t.Fatal(err)
	}
	snap = c.Snapshot()
	if snap.SafetyState == safety.StateOperational {
		t.Fatal("存在丢失故障时释放急停不得静默恢复 OPERATIONAL")
	}
	t.Logf("释放急停时反馈丢失 -> 停留在 %s（原因: %s）",
		snap.SafetyState, snap.SafetyReason.Detail)

	// 4) 反馈仍未恢复，Reset -> 拒绝并列明条件（在推进时钟前立即检验）
	if err := c.Reset(); err == nil {
		t.Fatal("看门狗条件仍成立时 Reset 必须失败")
	} else {
		t.Logf("条件未消失 Reset 再被拒: %v", err)
	}
	clk.Advance(period10)

	// 5) 反馈恢复后需先跑一个周期让事件重新评估为“条件消失”，再 Reset
	for k := 0; k < 2; k++ {
		now := clk.Now()
		d.respond(now, nil, false)
		d.emit(t, now)
		if _, err := c.Tick(); err != nil {
			t.Fatal(err)
		}
		clk.Advance(period10)
	}
	if err := c.Reset(); err != nil {
		t.Fatalf("条件消失后 Reset 应成功: %v", err)
	}
	snap = c.Snapshot()
	if snap.SafetyState != safety.StateOperational {
		t.Fatalf("最终应 OPERATIONAL，实际 %s", snap.SafetyState)
	}
	t.Logf("最终状态 %s：恢复路径显式、可解释", snap.SafetyState)
}

// TestSoftLimit 实际位置越过软限位 -> SOFT_LIMIT，退回到软限位内后需显式复位。
func TestSoftLimit(t *testing.T) {
	c, d, clk, _, _ := testSetup(t, frame.V2)
	if err := c.SubmitCommand(limits.Command{
		Joint: 0, Mode: limits.ModePosition, Position: 0,
	}); err != nil {
		t.Fatal(err)
	}
	warmup(t, c, d, clk, 5)

	// 软限位: 上限 1.57-0.05=1.52。用每周期 +0.1 的偏置斜坡模拟真实位置漂移
	// （单帧变化 <0.2 跳变阈值，保证帧始终有效；跟踪偏差会先报 FAULT_STOP，
	// 越过软限位后 SOFT_LIMIT 优先级更高）
	t.Log("输入: joint0 反馈位置斜坡越过软限位 1.52")
	bias := 0.0
	var snap controller.Snapshot
	for k := 0; k < 16; k++ {
		if bias < 1.54 {
			bias += 0.1
			d.setBias(0, bias)
		}
		now := clk.Now()
		d.respond(now, nil, false)
		d.emit(t, now)
		snap, _ = c.Tick()
		clk.Advance(period10)
	}
	if snap.SafetyState != safety.StateSoftLimit {
		t.Fatalf("期望 SOFT_LIMIT，实际 %s", snap.SafetyState)
	}
	t.Logf("判定: %s —— %s", snap.SafetyState, snap.SafetyReason.Detail)

	// 连续周期斜坡退回（每周期仅 1 步、时钟同步推进），保证帧链路有效；
	// 锁存不自动清除，必须后续显式 Reset。
	b := 1.54
	for b > 0 {
		b -= 0.1
		if b < 0 {
			b = 0
		}
		d.setBias(0, b)
		now := clk.Now()
		d.respond(now, nil, false)
		d.emit(t, now)
		snap, _ = c.Tick()
		clk.Advance(period10)
	}
	if snap.SafetyState != safety.StateSoftLimit {
		t.Fatalf("软限位触发已锁存，位置退回后仍应 SOFT_LIMIT，实际 %s",
			snap.SafetyState)
	}
	if err := c.Reset(); err != nil {
		t.Fatalf("条件消失后 Reset 应成功: %v", err)
	}
	if s := c.Snapshot().SafetyState; s != safety.StateOperational {
		t.Fatalf("Reset 后应 OPERATIONAL，实际 %s", s)
	}
}

// TestTorqueOverlimit V2 实测力矩超限 -> TORQUE_FAULT。
func TestTorqueOverlimit(t *testing.T) {
	c, d, clk, _, _ := testSetup(t, frame.V2)
	warmup(t, c, d, clk, 5)

	// joint0 TorqueMax=10
	d.setTorque(0, 12.5)
	now := clk.Now()
	d.respond(now, nil, false)
	d.emit(t, now)
	snap, _ := c.Tick()
	if snap.SafetyState != safety.StateTorqueFault {
		t.Fatalf("期望 TORQUE_FAULT，实际 %s（%s）",
			snap.SafetyState, snap.SafetyReason.Detail)
	}
	t.Logf("判定: 实测力矩 12.5 > 限值 10 -> %s（%s）",
		snap.SafetyState, snap.SafetyReason.Detail)

	d.setTorque(0, 0)
	clk.Advance(period10)
	now = clk.Now()
	d.respond(now, nil, false)
	d.emit(t, now)
	snap, _ = c.Tick()
	if snap.SafetyState != safety.StateTorqueFault {
		t.Fatal("力矩正常后故障仍应锁存")
	}
	if err := c.Reset(); err != nil {
		t.Fatalf("条件消失后 Reset 应成功: %v", err)
	}
}

// TestV1NoTorqueAndNoSoftFalsePositive V1 无力矩字段：
// 力矩超限监测永不误触发；V1 下发力矩命令被协议层拒绝。
func TestV1NoTorqueAndNoSoftFalsePositive(t *testing.T) {
	c, d, clk, _, _ := testSetup(t, frame.V1)
	warmup(t, c, d, clk, 5)

	err := c.SubmitCommand(limits.Command{Joint: 0, Mode: limits.ModeTorque, Torque: 1})
	if err == nil {
		t.Fatal("V1 控制器必须拒绝力矩命令")
	}
	t.Logf("V1 力矩命令被拒: %v", err)

	// V1 反馈 Torque 恒为 0，跑若干周期不应误报 TORQUE_FAULT
	for k := 0; k < 5; k++ {
		now := clk.Now()
		d.respond(now, nil, false)
		d.emit(t, now)
		snap, _ := c.Tick()
		if snap.SafetyState == safety.StateTorqueFault {
			t.Fatal("V1 无力矩字段不得误报力矩超限")
		}
		clk.Advance(period10)
	}
}
