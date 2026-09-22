// Package trajectory 负责运动命令的合法性校验、限幅、连续性检查
// 与轨迹缓冲推进。任何越界或自相矛盾的命令都以可区分原因拒绝。
package trajectory

import (
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
)

// Limits 是单关节的硬约束（位置范围、速度、加速度、力矩）。
type Limits struct {
	MinPositionRad  float64
	MaxPositionRad  float64
	MaxVelocityRadS float64
	MaxAccelRadS2   float64
	MaxTorqueNm     float64
}

// Command 是待校验的上层命令。
type Command struct {
	Mode     protocol.CommandMode
	Position float64
	Velocity float64
	Torque   float64
	// IssuedAt 为命令时间戳，用于按控制周期做加速度连续性核算。
	IssuedAt time.Time
}

// 拒绝原因码（CatCommandReject）。
const (
	ReasonPositionOutOfLimit = "position_out_of_limit"
	ReasonVelocityOutOfLimit = "velocity_out_of_limit"
	ReasonTorqueOutOfLimit   = "torque_out_of_limit"
	ReasonAccelOutOfLimit    = "accel_out_of_limit"
	ReasonContradictoryMode  = "contradictory_mode"
	ReasonEmptyTrajectory    = "empty_trajectory"
	ReasonBufferOverflow     = "trajectory_buffer_overflow"
	ReasonNonMonotonicTime   = "non_monotonic_time"
)

// Setpoint 是校验后在某周期下发的目标。
type Setpoint struct {
	Mode         protocol.CommandMode
	PositionRad  float64
	VelocityRadS float64
	TorqueNm     float64
	Seq          uint64
}

// Validator 维护“上一周期目标”，从而校验速度/加速度连续性。
type Validator struct {
	limits  Limits
	prev    Setpoint
	prevAt  time.Time
	hasPrev bool
	seq     uint64
}

// NewValidator 创建校验器。
func NewValidator(limits Limits) *Validator { return &Validator{limits: limits} }

// Validate 校验单条命令相对上一已接受目标是否可接受。
//
// 判定依据（顺序固定，保证拒绝原因可复现）：
//  1. 模式必须是已知模式，否则 contradictory_mode；
//  2. 各模式的主物理量先做硬限位检查（位置/速度/力矩各有独立原因码）；
//  3. 时间戳不得早于上一条已接受命令（non_monotonic_time）；
//  4. 相邻周期按 dt 估算加速度，超限 => accel_out_of_limit。
//     位置模式用 (v_cmd - v_prev)/dt 近似；跨模式切换时，
//     要求新目标速度轮廓相对上一目标满足同一加速度约束；
//  5. 全部通过才更新“上一目标”记忆并分配序号。
//
// 越界绝不自动裁剪：裁剪会把危险命令伪装成正常命令。
func (v *Validator) Validate(cmd Command) (Setpoint, error) {
	sp := Setpoint{Mode: cmd.Mode}
	switch cmd.Mode {
	case protocol.ModePosition:
		sp.PositionRad = cmd.Position
		// 位置模式下速度/力矩字段必须保持零值；携带非零的其它模式
		// 字段视为自相矛盾的命令。
		if cmd.Velocity != 0 || cmd.Torque != 0 {
			return Setpoint{}, reject(ReasonContradictoryMode,
				"position command must not carry velocity/torque values")
		}
	case protocol.ModeVelocity:
		sp.VelocityRadS = cmd.Velocity
		if cmd.Position != 0 || cmd.Torque != 0 {
			return Setpoint{}, reject(ReasonContradictoryMode,
				"velocity command must not carry position/torque values")
		}
	case protocol.ModeTorque:
		sp.TorqueNm = cmd.Torque
		if cmd.Position != 0 || cmd.Velocity != 0 {
			return Setpoint{}, reject(ReasonContradictoryMode,
				"torque command must not carry position/velocity values")
		}
	default:
		return Setpoint{}, reject(ReasonContradictoryMode, "unknown mode: "+string(cmd.Mode))
	}

	if cmd.Mode == protocol.ModePosition {
		if cmd.Position < v.limits.MinPositionRad || cmd.Position > v.limits.MaxPositionRad {
			return Setpoint{}, reject(ReasonPositionOutOfLimit,
				formatLimit("position", cmd.Position, v.limits.MinPositionRad, v.limits.MaxPositionRad))
		}
	}
	if cmd.Mode == protocol.ModeVelocity {
		if abs(cmd.Velocity) > v.limits.MaxVelocityRadS {
			return Setpoint{}, reject(ReasonVelocityOutOfLimit,
				formatLimit("velocity", cmd.Velocity, -v.limits.MaxVelocityRadS, v.limits.MaxVelocityRadS))
		}
	}
	if cmd.Mode == protocol.ModeTorque {
		if abs(cmd.Torque) > v.limits.MaxTorqueNm {
			return Setpoint{}, reject(ReasonTorqueOutOfLimit,
				formatLimit("torque", cmd.Torque, -v.limits.MaxTorqueNm, v.limits.MaxTorqueNm))
		}
	}

	if v.hasPrev {
		if !cmd.IssuedAt.IsZero() && !v.prevAt.IsZero() && cmd.IssuedAt.Before(v.prevAt) {
			return Setpoint{}, reject(ReasonNonMonotonicTime,
				"command timestamp before previous accepted command")
		}
		dt := 0.0
		if !cmd.IssuedAt.IsZero() && !v.prevAt.IsZero() {
			dt = cmd.IssuedAt.Sub(v.prevAt).Seconds()
		}
		if dt > 0 && v.limits.MaxAccelRadS2 > 0 {
			// 以两目标的“速度轮廓量”之差估算所需加速度。
			prevVel := v.effectiveVelocity(v.prev)
			thisVel := v.effectiveVelocity(sp)
			if accel := abs(thisVel-prevVel) / dt; accel > v.limits.MaxAccelRadS2 {
				return Setpoint{}, reject(ReasonAccelOutOfLimit,
					formatAccel(accel, v.limits.MaxAccelRadS2, dt))
			}
		}
	}

	v.seq++
	sp.Seq = v.seq
	v.prev = sp
	v.prevAt = cmd.IssuedAt
	v.hasPrev = true
	return sp, nil
}

// effectiveVelocity 给出目标在速度维度上的轮廓量：
// 速度模式取命令速度；位置/力矩模式没有直接速度承诺，取 0，
// 因此由静止切入超速目标也会被加速度约束拦截。
func (v *Validator) effectiveVelocity(sp Setpoint) float64 {
	if sp.Mode == protocol.ModeVelocity {
		return sp.VelocityRadS
	}
	return 0
}

// Prev 返回最近已接受目标（控制器用于跟踪误差基准与连续性）。
func (v *Validator) Prev() (Setpoint, time.Time, bool) {
	return v.prev, v.prevAt, v.hasPrev
}

// Reset 在安全动作/重新使能后清空连续性记忆。
func (v *Validator) Reset() {
	v.prev = Setpoint{}
	v.prevAt = time.Time{}
	v.hasPrev = false
}
