package safety_test

import (
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/pkg/safety"
)

func TestPriorityAndLatching(t *testing.T) {
	m := safety.New()
	at := time.Date(2026, 9, 22, 10, 0, 0, 0, time.UTC)

	// 同时上报力矩超限与软限位：软限位优先
	m.Evaluate([]safety.Event{
		{Kind: safety.KindTorque, Active: true, Detail: "tau=12"},
		{Kind: safety.KindSoftLimit, Active: true, Detail: "pos=1.6"},
	}, at)
	if st, _ := m.Snapshot(); st != safety.StateSoftLimit {
		t.Fatalf("期望 SOFT_LIMIT，实际 %s", st)
	}

	// 急停覆盖一切
	m.EStop(at.Add(time.Second), "人工急停")
	if st, _ := m.Snapshot(); st != safety.StateEStop {
		t.Fatalf("期望 ESTOP，实际 %s", st)
	}

	// 释放急停后回到次高优先级（软限位仍锁存），不静默恢复
	if err := m.ReleaseEStop(at.Add(2 * time.Second)); err != nil {
		t.Fatal(err)
	}
	if st, _ := m.Snapshot(); st != safety.StateSoftLimit {
		t.Fatalf("释放急停应回到 SOFT_LIMIT，实际 %s", st)
	}

	// 条件仍成立（未上报 false），Reset 被拒
	if err := m.Reset(at.Add(3 * time.Second)); err == nil {
		t.Fatal("条件未消失时 Reset 必须失败")
	}
	// 上报条件全部消失
	m.Evaluate([]safety.Event{
		{Kind: safety.KindTorque, Active: false},
		{Kind: safety.KindSoftLimit, Active: false},
	}, at.Add(4*time.Second))
	if err := m.Reset(at.Add(5 * time.Second)); err != nil {
		t.Fatalf("条件消失后 Reset 应成功: %v", err)
	}
	if st, _ := m.Snapshot(); st != safety.StateOperational {
		t.Fatalf("期望 OPERATIONAL，实际 %s", st)
	}

	tr := m.Transitions()
	if len(tr) < 4 {
		t.Fatalf("迁移留痕不完整: %d", len(tr))
	}
	for _, x := range tr {
		t.Logf("迁移 %s -> %s (%s): %s @%s",
			x.From, x.To, x.Kind, x.Detail, x.At.Format("15:04:05"))
	}
}

func TestMultiJointORAggregation(t *testing.T) {
	m := safety.New()
	at := time.Now()
	// joint0 故障，joint1 正常；事件顺序 false 在前、true 在后 以及 反向都测
	m.Evaluate([]safety.Event{
		{Kind: safety.KindWatchdog, Active: false, Detail: "joint1 ok"},
		{Kind: safety.KindWatchdog, Active: true, Detail: "joint0 lost"},
	}, at)
	if st, _ := m.Snapshot(); st != safety.StateWatchdog {
		t.Fatalf("任一关节触发即应 WATCHDOG，实际 %s", st)
	}
	m.Reset(time.Time{}) // 急停未置，条件仍 active -> 应失败
	if err := m.Reset(at); err == nil {
		t.Fatal("joint0 仍 active 时 Reset 必须失败")
	}
}
