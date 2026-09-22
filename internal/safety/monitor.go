package safety

import (
	"fmt"
	"sync"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/feedback"
	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
	"github.com/xingjianluo/robot-motion-control-go/internal/trajectory"
)

// Limits 是监视器使用的约束：硬位置范围、力矩上限与软限位余量。
// （与 trajectory.Limits 字段保持数值一致，但类型独立，避免安全包
// 反向依赖命令校验语义。）
type Limits struct {
	MinPositionRad float64
	MaxPositionRad float64
	MaxTorqueNm    float64
	SoftMarginRad  float64 // 软限位距硬限位的余量；必须 > 0
}

// Monitor 周期检查反馈样本与待发目标，只产出“带明确原因的触发”，
// 自身绝不修改任何运动状态。
type Monitor struct {
	mu          sync.Mutex
	machine     *Machine
	limits      Limits
	softActive  bool
	lastReasons []string // 最近一次 Check 实际产生/尝试的触发原因，便于观测
}

// NewMonitor 创建监视器。
func NewMonitor(m *Machine, l Limits) *Monitor {
	if l.SoftMarginRad < 0 {
		l.SoftMarginRad = 0
	}
	return &Monitor{machine: m, limits: l}
}

// Check 针对一条已接受样本执行全部安全检查，返回本次提交给状态机的
// 触发原因（可能为空）。检查顺序固定，保证并发场景结论可复现：
//  1. 驱动器故障码（锁存故障，优先级高）；
//  2. 力矩超限（仅当该版本反馈中确实存在力矩字段；V1 缺字段不得
//     被误判为零力矩，因此跳过而不是放行/误触发）；
//  3. 位置进入软限位带；
//  4. 位置回到软限位带内且软限位由本监视器置位 => 显式清除并留痕。
func (mon *Monitor) Check(now time.Time, s feedback.Sample) []string {
	mon.mu.Lock()
	defer mon.mu.Unlock()
	mon.lastReasons = mon.lastReasons[:0]

	if s.DriveFault != 0 {
		mon.fire(now, ReasonDriveFaultCode, fmt.Sprintf("drive reported fault code %d", s.DriveFault))
		return mon.snapshotReasons() // 锁存故障下继续检查无意义
	}
	if s.TorqueKnown {
		if t := s.TorqueNm; t > mon.limits.MaxTorqueNm || t < -mon.limits.MaxTorqueNm {
			mon.fire(now, ReasonTorqueOverload,
				fmt.Sprintf("torque %.3f Nm exceeds limit %.3f", t, mon.limits.MaxTorqueNm))
		}
	}

	softLow := mon.limits.MinPositionRad + mon.limits.SoftMarginRad
	softHigh := mon.limits.MaxPositionRad - mon.limits.SoftMarginRad
	outside := s.PositionRad < softLow || s.PositionRad > softHigh
	if outside {
		if !mon.softActive {
			mon.softActive = true
			mon.fire(now, ReasonSoftLimit,
				fmt.Sprintf("position %.4f outside soft band [%.4f, %.4f]",
					s.PositionRad, softLow, softHigh))
		}
	} else if mon.softActive {
		mon.softActive = false
		mon.fire(now, ReasonSoftLimitClear,
			fmt.Sprintf("position %.4f back inside soft band", s.PositionRad))
	}
	return mon.snapshotReasons()
}

// CheckSetpoint 在下发前检查目标：位置模式目标落入软限位带外时
// 提前触发软限位（由控制器拒绝该命令）。速度/力矩模式没有承诺位置，
// 交由反馈侧检查。
func (mon *Monitor) CheckSetpoint(now time.Time, sp trajectory.Setpoint) []string {
	mon.mu.Lock()
	defer mon.mu.Unlock()
	mon.lastReasons = mon.lastReasons[:0]
	if sp.Mode != protocol.ModePosition {
		return nil
	}
	softLow := mon.limits.MinPositionRad + mon.limits.SoftMarginRad
	softHigh := mon.limits.MaxPositionRad - mon.limits.SoftMarginRad
	if sp.PositionRad < softLow || sp.PositionRad > softHigh {
		mon.softActive = true
		mon.fire(now, ReasonSoftLimit,
			fmt.Sprintf("setpoint %.4f targets soft-limit band [%.4f, %.4f]",
				sp.PositionRad, softLow, softHigh))
	}
	return mon.snapshotReasons()
}

// fire 调用状态机（锁顺序：Monitor.mu -> Machine）。
func (mon *Monitor) fire(now time.Time, reason, detail string) {
	mon.machine.TriggerEvent(now, Trigger{Reason: reason, Detail: detail})
	mon.lastReasons = append(mon.lastReasons, reason)
}

func (mon *Monitor) snapshotReasons() []string {
	out := make([]string, len(mon.lastReasons))
	copy(out, mon.lastReasons)
	return out
}
