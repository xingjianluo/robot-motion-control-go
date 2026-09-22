// Package limits 定义关节限位（位置/速度/力矩）与命令校验，
// 越界与自相矛盾的命令被拒绝并返回可区分的原因码。
package limits

import (
	"math"

	myerr "github.com/xingjianluo/robot-motion-control-go/pkg/errors"
)

// JointLimits 单关节约束。
type JointLimits struct {
	ID         int
	PosMin     float64 // 硬位置限（弧度）
	PosMax     float64
	VelMax     float64 // 最大绝对速度（rad/s）
	AccMax     float64 // 最大绝对加速度（rad/s^2）
	TorqueMax  float64 // 最大绝对力矩（Nm）
	SoftMargin float64 // 软限位距硬限位的余量（rad）
}

// SoftMin / SoftMax 返回软限位。
func (l JointLimits) SoftMin() float64 { return l.PosMin + l.SoftMargin }
func (l JointLimits) SoftMax() float64 { return l.PosMax - l.SoftMargin }

// Mode 命令模式，三种运动模式互斥。
type Mode uint8

const (
	ModeInvalid  Mode = 0
	ModePosition Mode = 1
	ModeVelocity Mode = 2
	ModeTorque   Mode = 3
)

// Command 下发给单关节的运动命令。
// 约定：按 Mode 仅解释对应字段，其余字段必须为 0，否则判定为自相矛盾。
type Command struct {
	Joint    int
	Mode     Mode
	Position float64
	Velocity float64
	Torque   float64
}

// Validate 校验命令是否满足约束。返回 nil 表示接受。
func (l JointLimits) Validate(c Command) *myerr.Error {
	switch c.Mode {
	case ModePosition:
		if c.Velocity != 0 || c.Torque != 0 {
			return myerr.New(myerr.CatCommand, "CONTRADICTORY_COMMAND",
				"位置模式下 Velocity/Torque 必须为 0（三种模式互斥，不得同时指定）")
		}
		if badFloat(c.Position) {
			return myerr.New(myerr.CatCommand, "MALFORMED_VALUE", "目标位置非有限数值")
		}
		if c.Position < l.PosMin || c.Position > l.PosMax {
			return myerr.New(myerr.CatCommand, "POS_LIMIT",
				"目标位置 "+ff(c.Position)+" 超出硬限位 ["+ff(l.PosMin)+", "+ff(l.PosMax)+"]")
		}
	case ModeVelocity:
		if c.Position != 0 || c.Torque != 0 {
			return myerr.New(myerr.CatCommand, "CONTRADICTORY_COMMAND",
				"速度模式下 Position/Torque 必须为 0")
		}
		if badFloat(c.Velocity) {
			return myerr.New(myerr.CatCommand, "MALFORMED_VALUE", "目标速度非有限数值")
		}
		if math.Abs(c.Velocity) > l.VelMax {
			return myerr.New(myerr.CatCommand, "VEL_LIMIT",
				"目标速度 "+ff(c.Velocity)+" 超出 |v|<= "+ff(l.VelMax))
		}
	case ModeTorque:
		if c.Position != 0 || c.Velocity != 0 {
			return myerr.New(myerr.CatCommand, "CONTRADICTORY_COMMAND",
				"力矩模式下 Position/Velocity 必须为 0")
		}
		if badFloat(c.Torque) {
			return myerr.New(myerr.CatCommand, "MALFORMED_VALUE", "目标力矩非有限数值")
		}
		if math.Abs(c.Torque) > l.TorqueMax {
			return myerr.New(myerr.CatCommand, "TORQUE_LIMIT",
				"目标力矩 "+ff(c.Torque)+" 超出 |tau|<= "+ff(l.TorqueMax))
		}
	default:
		return myerr.New(myerr.CatCommand, "UNKNOWN_MODE",
			"未知命令模式 "+itoa(int(c.Mode)))
	}
	return nil
}

func badFloat(f float64) bool { return math.IsNaN(f) || math.IsInf(f, 0) }

func ff(f float64) string {
	// 轻量格式化，避免引入 strconv 以外的依赖争议
	const digits = "0123456789"
	if f == 0 {
		return "0"
	}
	neg := f < 0
	if neg {
		f = -f
	}
	whole := int64(f)
	frac := f - float64(whole)
	var b []byte
	if whole == 0 {
		b = append(b, '0')
	} else {
		var w [20]byte
		i := len(w)
		for whole > 0 {
			i--
			w[i] = digits[whole%10]
			whole /= 10
		}
		b = append(b, w[i:]...)
	}
	if frac > 0 {
		b = append(b, '.')
		for i := 0; i < 4; i++ {
			frac *= 10
			d := int64(frac)
			b = append(b, digits[d])
			frac -= float64(d)
		}
	}
	if neg {
		return "-" + string(b)
	}
	return string(b)
}

func itoa(n int) string {
	if n == 0 {
		return "0"
	}
	neg := n < 0
	if neg {
		n = -n
	}
	var b [20]byte
	i := len(b)
	for n > 0 {
		i--
		b[i] = byte('0' + n%10)
		n /= 10
	}
	if neg {
		i--
		b[i] = '-'
	}
	return string(b[i:])
}
