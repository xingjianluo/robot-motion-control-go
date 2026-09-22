// Package safety 实现安全联锁状态机：
// 安全状态优先级高于一切普通控制命令；进入/退出安全状态都必须显式、
// 带原因、满足明确条件，禁止静默降级或自动恢复。
package safety

import (
	"fmt"
	"strings"
	"sync"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
)

// State 是安全状态。
type State string

const (
	// StateRun 正常运行，唯一允许功率输出的状态。
	StateRun State = "RUN"
	// StateSoftLimit 软限位：触发期间命令被拒绝/冻结，
	// 条件解除后由监视器显式清除（有事件记录，不是静默恢复）。
	StateSoftLimit State = "SOFT_LIMIT"
	// StateTorqueFault 力矩超限/驱动器故障码，锁存，必须人工确认恢复。
	StateTorqueFault State = "TORQUE_FAULT"
	// StateCommLoss 通信看门狗超时（暂时失联），冻结输出，
	// 链路恢复后仍必须显式恢复，禁止自动重新使能。
	StateCommLoss State = "COMM_LOSS"
	// StateEstop 急停，最高优先级，输出切断，锁存。
	StateEstop State = "ESTOP"
)

// Priority 返回状态优先级，数值越大优先级越高。
// 并发到达的多个触发在同一把锁下按最高优先级确定最终状态。
func (s State) Priority() int {
	switch s {
	case StateRun:
		return 0
	case StateSoftLimit:
		return 10
	case StateTorqueFault:
		return 20
	case StateCommLoss:
		return 30
	case StateEstop:
		return 40
	}
	return -1
}

// 安全原因码（CatSafety）。
const (
	ReasonEstopPressed   = "estop_pressed"
	ReasonSoftLimit      = "soft_limit_violation"
	ReasonSoftLimitClear = "soft_limit_cleared"
	ReasonTorqueOverload = "torque_overload"
	ReasonDriveFaultCode = "drive_fault_code"
	ReasonCommLoss       = "communication_loss"
	ReasonRecoveryDenied = "recovery_denied"
	ReasonManualRecovery = "manual_recovery"
)

// Event 记录一次状态变迁，保证可追溯、可复现。
type Event struct {
	At     time.Time
	From   State
	To     State
	Reason string
	Detail string
}

// Trigger 是一次外部/内部安全输入。Reason 决定目标状态。
type Trigger struct {
	Reason string
	Detail string
}

// targetOf 把触发原因映射为目标状态；未知原因不触发任何变迁。
func targetOf(reason string) (State, bool) {
	switch reason {
	case ReasonEstopPressed:
		return StateEstop, true
	case ReasonSoftLimit, ReasonSoftLimitClear:
		return StateSoftLimit, true
	case ReasonTorqueOverload, ReasonDriveFaultCode:
		return StateTorqueFault, true
	case ReasonCommLoss:
		return StateCommLoss, true
	}
	return StateRun, false
}

// Machine 是并发安全的联锁状态机。
type Machine struct {
	mu               sync.Mutex
	state            State
	events           []Event
	lastSuppressed   string
	lastSuppressedAt time.Time
}

// NewMachine 创建状态机。系统上电默认 ESTOP：必须经一次满足全部
// 条件的显式恢复才能使能，杜绝“上电即出力”。
func NewMachine(initial State) *Machine {
	if initial == "" {
		initial = StateEstop
	}
	return &Machine{state: initial}
}

// State 返回当前状态。
func (m *Machine) State() State {
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.state
}

// TriggerEvent 尝试注入一个安全触发。
//
// 判定规则（在单把锁内完成，与周期推进/急停并发时结果确定）：
//   - 软限位“清除”是唯一允许从 SOFT_LIMIT 回到 RUN 的触发，且仅当
//     当前状态正是 SOFT_LIMIT；它由监视器在条件明确解除时发出并留痕；
//   - 其余触发按优先级：仅当目标状态优先级严格更高时才变迁；
//     同级/低级触发被抑制，原因记录在 LastSuppressed 中可观测；
//   - 未知原因一律不改变状态。
//
// 返回最终状态与本次是否发生变迁。
func (m *Machine) TriggerEvent(now time.Time, t Trigger) (State, bool) {
	m.mu.Lock()
	defer m.mu.Unlock()

	if t.Reason == ReasonSoftLimitClear {
		if m.state == StateSoftLimit {
			m.transitionLocked(now, StateRun, t.Reason, t.Detail)
			return StateRun, true
		}
		m.suppressLocked(now, t.Reason)
		return m.state, false
	}

	target, known := targetOf(t.Reason)
	if !known {
		m.suppressLocked(now, "unknown_trigger:"+t.Reason)
		return m.state, false
	}
	if target.Priority() > m.state.Priority() {
		m.transitionLocked(now, target, t.Reason, t.Detail)
		return target, true
	}
	m.suppressLocked(now, t.Reason)
	return m.state, false
}

// Estop 是急停快捷入口：任何状态下无条件进入 ESTOP。
// 该方法可与控制周期并发调用；急停提交之后的周期绝不可能再下发出力命令。
func (m *Machine) Estop(now time.Time, detail string) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.state != StateEstop {
		m.transitionLocked(now, StateEstop, ReasonEstopPressed, detail)
		return
	}
	m.suppressLocked(now, ReasonEstopPressed)
}

// RequestRecover 显式请求恢复到 RUN。
// 必须满足全部前置条件（任一不满足 => fault(CatSafety, recovery_denied)，
// 并逐条列出缺失项）；OperatorAck 强制要求人工确认，因此任何锁存
// 故障都不会静默/自动恢复。
func (m *Machine) RequestRecover(now time.Time, c RecoveryConditions) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	var missing []string
	if !c.EstopReleased {
		missing = append(missing, "estop_not_released")
	}
	if !c.CommHealthy {
		missing = append(missing, "comm_unhealthy")
	}
	if !c.TorqueWithinLimit {
		missing = append(missing, "torque_over_limit")
	}
	if !c.PositionWithinLimit {
		missing = append(missing, "position_out_of_limit")
	}
	if !c.FaultCodeClear {
		missing = append(missing, "drive_fault_code_active")
	}
	if !c.OperatorAck {
		missing = append(missing, "operator_ack_required")
	}
	if len(missing) > 0 {
		detail := "recovery blocked by: " + strings.Join(missing, ",")
		m.suppressLocked(now, ReasonRecoveryDenied)
		return fault.New(fault.CatSafety, fault.SourceSafety, ReasonRecoveryDenied, detail)
	}
	if m.state == StateRun {
		return nil
	}
	m.transitionLocked(now, StateRun, ReasonManualRecovery,
		fmt.Sprintf("recovered from %s with all conditions met", m.state))
	return nil
}

func (m *Machine) transitionLocked(now time.Time, to State, reason, detail string) {
	m.events = append(m.events, Event{At: now, From: m.state, To: to, Reason: reason, Detail: detail})
	m.state = to
}

func (m *Machine) suppressLocked(now time.Time, reason string) {
	m.lastSuppressed = reason
	m.lastSuppressedAt = now
}

// Events 返回变迁历史快照。
func (m *Machine) Events() []Event {
	m.mu.Lock()
	defer m.mu.Unlock()
	out := make([]Event, len(m.events))
	copy(out, m.events)
	return out
}

// LastSuppressed 返回最近一次被抑制触发的原因与时刻（无则 ok=false）。
func (m *Machine) LastSuppressed() (string, time.Time, bool) {
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.lastSuppressed, m.lastSuppressedAt, m.lastSuppressed != ""
}

// OutputsEnabled 表示当前状态是否允许功率输出；仅 RUN 允许。
func (m *Machine) OutputsEnabled() bool {
	return m.State() == StateRun
}

// RecoveryConditions 是恢复所需的全部明确条件。
type RecoveryConditions struct {
	EstopReleased       bool
	CommHealthy         bool
	TorqueWithinLimit   bool
	PositionWithinLimit bool
	FaultCodeClear      bool
	OperatorAck         bool // 必须有人工确认，禁止自动恢复
}
