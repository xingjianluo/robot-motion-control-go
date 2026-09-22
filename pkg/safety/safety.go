// Package safety 实现安全联锁状态机。
//
// 优先级（高 -> 低）：
//
//	ESTOP(急停) > SOFT_LIMIT(软限位) > WATCHDOG(通信看门狗) >
//	TORQUE_FAULT(力矩超限) > FAULT_STOP(驱动器故障/反馈不可信) > OPERATIONAL
//
// 规则：
//   - 安全状态优先级始终高于普通命令：非 OPERATIONAL 一律拒绝运动下发；
//   - 所有安全触发均“锁存”，不会因瞬时条件消失而静默自动恢复；
//   - 恢复条件显式且可检验：急停必须先 ReleaseEStop，其余故障必须在
//     触发条件确已消失后显式 Reset，Reset 失败会返回未满足条件的原因；
//   - 所有状态迁移记录时间戳与原因，结果可复现、可解释。
package safety

import (
	"strings"
	"time"
)

// State 安全状态。
type State string

const (
	StateOperational State = "OPERATIONAL"
	StateEStop       State = "ESTOP"
	StateSoftLimit   State = "SOFT_LIMIT"
	StateWatchdog    State = "WATCHDOG"
	StateTorqueFault State = "TORQUE_FAULT"
	StateFaultStop   State = "FAULT_STOP"
)

// Kind 安全触发来源（比 State 更细，便于区分故障来源）。
type Kind string

const (
	KindEStop      Kind = "ESTOP"
	KindSoftLimit  Kind = "SOFT_LIMIT"
	KindWatchdog   Kind = "WATCHDOG"
	KindTorque     Kind = "TORQUE_FAULT"
	KindDriveFault Kind = "DRIVE_FAULT"
	KindTracking   Kind = "TRACKING_ERROR" // 反馈有效但实际位置偏离参考（真实偏差，非失联）
)

// Cause 单个安全触发的当前情况。
type Cause struct {
	Kind    Kind
	Active  bool      // 仅反映“最近一次评估”时刻条件是否成立
	Latched bool      // 是否已锁存（发生过且未复位）
	Detail  string    // 最近一次判定依据
	At      time.Time // 最近一次触发时间
}

// Transition 状态迁移留痕。
type Transition struct {
	At     time.Time
	From   State
	To     State
	Kind   Kind
	Detail string
}

// Event 提交给监控器的安全输入。
type Event struct {
	Kind   Kind
	Active bool
	Detail string
	At     time.Time
}

// Monitor 安全联锁监控器。非并发安全，由 controller 统一加锁。
type Monitor struct {
	causes      map[Kind]*Cause
	estate      bool // 急停标志（独立显式释放）
	state       State
	transitions []Transition
}

// New 创建安全监控器。
func New() *Monitor {
	return &Monitor{causes: map[Kind]*Cause{}, state: StateOperational}
}

// Priority 返回状态优先级（数值越大优先级越高）。
func Priority(s State) int {
	switch s {
	case StateEStop:
		return 50
	case StateSoftLimit:
		return 40
	case StateWatchdog:
		return 30
	case StateTorqueFault:
		return 20
	case StateFaultStop:
		return 10
	default:
		return 0
	}
}

func kindState(k Kind) State {
	switch k {
	case KindEStop:
		return StateEStop
	case KindSoftLimit:
		return StateSoftLimit
	case KindWatchdog:
		return StateWatchdog
	case KindTorque:
		return StateTorqueFault
	case KindDriveFault, KindTracking:
		return StateFaultStop
	default:
		return StateOperational
	}
}

// Evaluate 每个控制周期汇总各触发条件：
// active=true 立即锁存；active=false 仅表示条件消失，锁存仍需显式 Reset。
func (m *Monitor) Evaluate(events []Event, at time.Time) State {
	// Active 只在“被评估的那一周期”有效：若某触发源本周期没有事件
	// （例如安全停机后控制器不再产生该类事件），不得沿用陈旧的 Active=true
	// 阻止复位；是否仍故障由下一周期重新上报。锁存 Latched 不受影响。
	for _, c := range m.causes {
		c.Active = false
	}
	for _, e := range events {
		c, ok := m.causes[e.Kind]
		if !ok {
			c = &Cause{Kind: e.Kind}
			m.causes[e.Kind] = c
		}
		// 同一 Kind 在一个周期内可能由多个关节上报（每关节一条事件），
		// 必须做“或”聚合：任一关节触发即 Active=true，不得被后续关节的
		// false 覆盖，否则会静默吞掉另一关节的故障。
		if e.Active {
			if !c.Active {
				c.Detail = e.Detail
			} else if e.Detail != c.Detail && len(c.Detail) < 300 {
				c.Detail += "; " + e.Detail
			}
			c.Active = true
			if !c.Latched {
				c.Latched = true
				c.At = at
			}
		}
	}
	m.recompute(at)
	return m.state
}

// EStop 显式急停（最高优先级，任何状态均可触发）。
func (m *Monitor) EStop(at time.Time, reason string) {
	if !m.estate {
		from := m.current()
		m.estate = true
		m.record(from, StateEStop, at, KindEStop, reason)
	}
	m.recompute(at)
}

// ReleaseEStop 释放急停。仅撤销急停锁存；若仍有其他锁存故障，
// 系统不会静默恢复到 OPERATIONAL，而是停留在对应故障状态。
func (m *Monitor) ReleaseEStop(at time.Time) error {
	if !m.estate {
		return errReason("ESTOP_NOT_ACTIVE", "当前未处于急停状态，无需释放")
	}
	from := m.current()
	m.estate = false
	m.record(from, m.peekAfterEStopRelease(), at, KindEStop, "急停已显式释放")
	m.recompute(at)
	return nil
}

// Reset 故障复位：仅清除“触发条件已经消失”的锁存。
// 急停未释放或仍有条件成立时拒绝复位并列出未满足项，绝不静默成功。
func (m *Monitor) Reset(at time.Time) error {
	if m.estate {
		return errReason("ESTOP_ACTIVE", "急停仍生效，必须先 ReleaseEStop 才能复位")
	}
	var blocked []string
	for _, c := range m.causes {
		if c.Latched && c.Active {
			blocked = append(blocked, string(c.Kind)+"("+c.Detail+")")
		}
	}
	if len(blocked) > 0 {
		return errReason("RESET_CONDITIONS_UNMET",
			"以下安全触发条件仍成立，拒绝复位: "+strings.Join(blocked, ", "))
	}
	from := m.state
	for _, c := range m.causes {
		c.Latched = false
	}
	m.recompute(at)
	if from != m.state {
		m.record(from, m.state, at, "", "显式故障复位，所有触发条件均已消失")
	}
	return nil
}

// AllowsMotion 当前状态是否允许下发运动命令。
func (m *Monitor) AllowsMotion() bool { return m.state == StateOperational }

// Snapshot 返回当前安全状态与最高优先级原因。
func (m *Monitor) Snapshot() (State, Cause) {
	st := m.current()
	best := Cause{}
	for _, c := range m.causes {
		if (c.Active || c.Latched) && Priority(kindState(c.Kind)) >= Priority(kindState(best.Kind)) {
			best = *c
		}
	}
	if m.estate {
		best = Cause{Kind: KindEStop, Active: true, Latched: true,
			Detail: "急停生效", At: m.eStopAt()}
	}
	return st, best
}

// Transitions 返回迁移留痕副本（最多保留最近 128 条）。
func (m *Monitor) Transitions() []Transition {
	out := make([]Transition, len(m.transitions))
	copy(out, m.transitions)
	return out
}

func (m *Monitor) current() State {
	if m.estate {
		return StateEStop
	}
	best := StateOperational
	for _, c := range m.causes {
		if c.Active || c.Latched {
			if s := kindState(c.Kind); Priority(s) > Priority(best) {
				best = s
			}
		}
	}
	return best
}

func (m *Monitor) peekAfterEStopRelease() State {
	saved := m.estate
	m.estate = false
	s := m.current()
	m.estate = saved
	return s
}

func (m *Monitor) recompute(at time.Time) {
	newState := m.current()
	if newState != m.state {
		m.record(m.state, newState, at, "", "安全状态重算迁移")
	}
}

func (m *Monitor) record(from, to State, at time.Time, k Kind, detail string) {
	if from == to {
		return
	}
	m.transitions = append(m.transitions, Transition{At: at, From: from, To: to,
		Kind: k, Detail: detail})
	if len(m.transitions) > 128 {
		m.transitions = m.transitions[len(m.transitions)-128:]
	}
	m.state = to
}

func (m *Monitor) eStopAt() time.Time {
	for i := len(m.transitions) - 1; i >= 0; i-- {
		if m.transitions[i].Kind == KindEStop && m.transitions[i].To == StateEStop {
			return m.transitions[i].At
		}
	}
	return time.Time{}
}
