// Package trajectory 生成逐控制周期连续、且始终满足速度/加速度约束的
// 位置参考轨迹（梯形速度曲线）。参考位置始终位于硬限位内；
// 软限位由 safety 层依据实际反馈独立判定，二者职责分离。
package trajectory

import (
	"math"

	myerr "github.com/xingjianluo/robot-motion-control-go/pkg/errors"
	"github.com/xingjianluo/robot-motion-control-go/pkg/limits"
)

// Point 一个控制周期的参考点。
type Point struct {
	Position float64
	Velocity float64
	Accel    float64
	Done     bool // 是否到达目标并静止
}

// Trajectory 单关节参考轨迹。以固定周期 dt 前向积分，
// 每个周期加速度幅值 <= AccMax、速度幅值 <= VelMax。
type Trajectory struct {
	lim    limits.JointLimits
	dt     float64
	pos    float64
	vel    float64
	target float64
	done   bool
}

// Plan 从当前状态规划到目标位置。
// 若初速度已越界（与限位自相矛盾）或目标越硬限位，返回带分类的错误，
// 调用方原有状态保持不变（此处不修改任何接收方状态）。
func Plan(startPos, startVel, target float64, dtSeconds float64, lim limits.JointLimits) (*Trajectory, error) {
	if dtSeconds <= 0 {
		return nil, myerr.New(myerr.CatCommand, "BAD_PERIOD",
			"控制周期必须为正数")
	}
	if math.IsNaN(startPos) || math.IsNaN(startVel) || math.IsNaN(target) ||
		math.IsInf(startPos, 0) || math.IsInf(startVel, 0) || math.IsInf(target, 0) {
		return nil, myerr.New(myerr.CatCommand, "MALFORMED_VALUE", "轨迹输入含非有限数值")
	}
	if math.Abs(startVel) > lim.VelMax*(1+1e-9) {
		return nil, myerr.New(myerr.CatCommand, "VEL_LIMIT",
			"起始速度已超出速度限位，无法生成连续轨迹")
	}
	if target < lim.PosMin || target > lim.PosMax {
		return nil, myerr.New(myerr.CatCommand, "POS_LIMIT",
			"目标位置超出硬限位")
	}
	if startPos < lim.PosMin || startPos > lim.PosMax {
		return nil, myerr.New(myerr.CatCommand, "POS_LIMIT",
			"起始位置已在硬限位之外，拒绝以越界状态启动轨迹")
	}
	return &Trajectory{
		lim:    lim,
		dt:     dtSeconds,
		pos:    startPos,
		vel:    clamp(startVel, -lim.VelMax, lim.VelMax),
		target: target,
		done:   false,
	}, nil
}

// Sample 前进一个控制周期并返回该周期的位置命令与参考点。
// 连续性由“先更新速度、再用周期平均速度更新位置”保证。
func (t *Trajectory) Sample() (limits.Command, Point, error) {
	if t.done {
		p := Point{Position: t.pos, Velocity: 0, Accel: 0, Done: true}
		return limits.Command{Joint: t.lim.ID, Mode: limits.ModePosition, Position: t.pos}, p, nil
	}

	dist := t.target - t.pos
	dir := 0.0
	if dist > 0 {
		dir = 1
	} else if dist < 0 {
		dir = -1
	}

	// 到达死区：若残余距离小于一个最小周期可达步长，直接静止吸附，
	// 避免离散步长下的极限环抖动。
	stepMin := 0.5 * t.lim.AccMax * t.dt * t.dt
	if math.Abs(dist) <= math.Max(stepMin, 1e-9) && math.Abs(t.vel) <= 1e-9 {
		t.pos = t.target
		t.vel = 0
		t.done = true
		p := Point{Position: t.pos, Velocity: 0, Accel: 0, Done: true}
		return limits.Command{Joint: t.lim.ID, Mode: limits.ModePosition, Position: t.pos}, p, nil
	}

	// 制动距离 v^2/(2a)：若当前速度下剩余距离不足以安全减速，
	// 本周期必须以 -dir 方向的最大加速度减速。
	brakeDist := (t.vel * t.vel) / (2 * t.lim.AccMax)
	needBrake := dir != 0 && brakeDist >= math.Abs(dist)-stepMin

	desiredVel := 0.0
	var accel float64
	if needBrake {
		accel = -dir * t.lim.AccMax
	} else if dir != 0 {
		desiredVel = dir * t.lim.VelMax
		accel = clamp(desiredVel-t.vel, -t.lim.AccMax*t.dt, t.lim.AccMax*t.dt) / t.dt
	} else {
		// 在目标点附近但仍有速度：向速度零值减速
		accel = clamp(-t.vel, -t.lim.AccMax*t.dt, t.lim.AccMax*t.dt) / t.dt
	}

	newVel := clamp(t.vel+accel*t.dt, -t.lim.VelMax, t.lim.VelMax)
	// 若速度越过零值而距离已很小，钳到 0，防止反向超调振荡
	if dir != 0 && (dir > 0) == (newVel < 0) && brakeDist >= math.Abs(dist)-stepMin {
		newVel = 0
	}

	avgVel := 0.5 * (t.vel + newVel)
	newPos := t.pos + avgVel*t.dt

	// 到达判定：本周期步长跨过目标（含恰好在目标点减速）即吸附。
	// 仅当剩余速度可在一个周期内以 <=AccMax 归零才真正停止，
	// 保证“瞬间停止”在物理上仍满足加速度约束；否则继续减速。
	if (newPos-t.target)*(t.pos-t.target) <= 0 &&
		math.Abs(newVel) <= t.lim.AccMax*t.dt+1e-12 {
		newPos = t.target
		newVel = 0
	}
	// 参考位置始终在硬限位内
	newPos = clamp(newPos, t.lim.PosMin, t.lim.PosMax)

	realAccel := (newVel - t.vel) / t.dt
	t.vel = newVel
	t.pos = newPos

	if math.Abs(t.target-t.pos) <= 1e-9 && math.Abs(t.vel) <= 1e-9 {
		t.pos = t.target
		t.vel = 0
		t.done = true
	}

	p := Point{Position: t.pos, Velocity: t.vel, Accel: realAccel, Done: t.done}
	return limits.Command{Joint: t.lim.ID, Mode: limits.ModePosition, Position: t.pos}, p, nil
}

// Done 轨迹是否完成。
func (t *Trajectory) Done() bool { return t.done }

// Target / Pos / Vel 暴露当前规划状态（只读用途）。
func (t *Trajectory) Target() float64 { return t.target }
func (t *Trajectory) Pos() float64    { return t.pos }
func (t *Trajectory) Vel() float64    { return t.vel }

func clamp(x, lo, hi float64) float64 {
	if x < lo {
		return lo
	}
	if x > hi {
		return hi
	}
	return x
}
