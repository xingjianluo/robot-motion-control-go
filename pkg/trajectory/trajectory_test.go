package trajectory_test

import (
	"math"
	"testing"

	"github.com/xingjianluo/robot-motion-control-go/pkg/limits"
	"github.com/xingjianluo/robot-motion-control-go/pkg/trajectory"
)

func TestTrapezoidConstraints(t *testing.T) {
	l := limits.JointLimits{ID: 0, PosMin: -2, PosMax: 2, VelMax: 1, AccMax: 2}
	tr, err := trajectory.Plan(0, 0, 1.5, 0.01, l)
	if err != nil {
		t.Fatal(err)
	}
	steps := 0
	var last trajectory.Point
	for !tr.Done() {
		_, p, err := tr.Sample()
		if err != nil {
			t.Fatal(err)
		}
		// 使用轨迹报告的速度/加速度（与实际下发参考一致）
		if math.Abs(p.Velocity) > 1.0+1e-9 {
			t.Fatalf("速度超限 %.4f @step %d", p.Velocity, steps)
		}
		if math.Abs(p.Accel) > 2.0+1e-6 {
			t.Fatalf("加速度超限 %.4f @step %d", p.Accel, steps)
		}
		if p.Position < -2-1e-9 || p.Position > 2+1e-9 {
			t.Fatalf("位置越限 %.4f", p.Position)
		}
		last = p
		steps++
		if steps > 10000 {
			t.Fatal("轨迹不收敛")
		}
	}
	if math.Abs(last.Position-1.5) > 1e-9 {
		t.Fatalf("终点 %.6f != 1.5", last.Position)
	}
	if math.Abs(last.Velocity) > 1e-12 {
		t.Fatalf("终点未静止 v=%.2e", last.Velocity)
	}
	t.Logf("输入 0->1.5 (vmax=1,amax=2,dt=10ms)：%d 步收敛到 %.4f，末速度 %.2e",
		steps, last.Position, last.Velocity)
}

func TestPlanRejectsOutOfLimits(t *testing.T) {
	l := limits.JointLimits{PosMin: -1, PosMax: 1, VelMax: 1, AccMax: 2}
	if _, err := trajectory.Plan(0, 0, 2, 0.01, l); err == nil {
		t.Fatal("目标越限必须拒绝")
	}
	if _, err := trajectory.Plan(0, 3, 0.5, 0.01, l); err == nil {
		t.Fatal("起始超速必须拒绝")
	}
}
