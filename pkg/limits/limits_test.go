package limits_test

import (
	"testing"

	myerr "github.com/xingjianluo/robot-motion-control-go/pkg/errors"
	"github.com/xingjianluo/robot-motion-control-go/pkg/limits"
)

func TestRejectionReasons(t *testing.T) {
	l := limits.JointLimits{ID: 0, PosMin: -1, PosMax: 1, VelMax: 2,
		AccMax: 4, TorqueMax: 5, SoftMargin: 0.1}

	cases := []struct {
		name string
		cmd  limits.Command
		code string
	}{
		{"位置越上限", limits.Command{Joint: 0, Mode: limits.ModePosition, Position: 1.5}, "POS_LIMIT"},
		{"速度越限", limits.Command{Joint: 0, Mode: limits.ModeVelocity, Velocity: 3}, "VEL_LIMIT"},
		{"力矩越限", limits.Command{Joint: 0, Mode: limits.ModeTorque, Torque: 9}, "TORQUE_LIMIT"},
		{"位置模式夹带速度", limits.Command{Joint: 0, Mode: limits.ModePosition, Position: 0, Velocity: 1}, "CONTRADICTORY_COMMAND"},
		{"未知模式", limits.Command{Joint: 0, Mode: limits.Mode(99)}, "UNKNOWN_MODE"},
		{"合法位置命令", limits.Command{Joint: 0, Mode: limits.ModePosition, Position: 0.5}, ""},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			err := l.Validate(tc.cmd)
			if tc.code == "" {
				if err != nil {
					t.Fatalf("应接受但被拒: %v", err)
				}
				return
			}
			if err == nil || err.Code != tc.code {
				t.Fatalf("期望 %s，得到 %v", tc.code, err)
			}
			if err.Cat != myerr.CatCommand {
				t.Fatalf("限位拒绝必须归类 COMMAND，得到 %s", err.Cat)
			}
			t.Logf("输入 %+v -> 拒绝码 %s：%s", tc.cmd, err.Code, err.Detail)
		})
	}
}
