// Package feedback 消费编码器/驱动器反馈：
// 区分反馈丢失（暂时失联 STALE / 确定丢失 LOST）、序号与时间戳乱序、
// 重复帧、时间戳前跳、位置跳变。异常帧不更新“最新有效反馈”，
// 因此绝不会被当作正常跟踪误差处理。
package feedback

import (
	"math"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/pkg/frame"
)

// Condition 反馈质量分类。
type Condition string

const (
	CondOK            Condition = "OK"
	CondStale         Condition = "STALE"          // 暂时失联（看门狗窗口内未收到新帧）
	CondLost          Condition = "LOST"           // 超过丢失阈值，判定为反馈丢失
	CondOutOfOrder    Condition = "OUT_OF_ORDER"   // 序号或时间戳回退
	CondDuplicate     Condition = "DUPLICATE"      // 序号重复
	CondTimestampJump Condition = "TIMESTAMP_JUMP" // 时间戳前向跳变超阈值
	CondPositionJump  Condition = "POSITION_JUMP"  // 位置单周期跳变超阈值
)

// Config 监控参数。
type Config struct {
	StaleTimeout   time.Duration // 超过该时长无新帧 -> STALE（暂时失联）
	LostTimeout    time.Duration // 超过该时长 -> LOST（反馈丢失）
	MaxPosJump     float64       // 单周期允许的最大位置跳变（rad）
	MaxForwardSkew time.Duration // 允许的反馈时间戳前向跳变上限
}

// State 监控器输出（值类型，便于无锁快照拷贝）。
type State struct {
	Cond       Condition
	LastSeq    uint32
	HasData    bool
	LastPos    float64
	LastVel    float64
	LastTorque float64
	LastAt     time.Time
	Age        time.Duration // 距上次“有效”帧的时间
	Reason     string        // 判定依据
}

// Monitor 单关节反馈质量监控器。本身不做并发保护，由 controller 统一加锁。
type Monitor struct {
	joint   int
	cfg     Config
	has     bool
	seq     uint32
	pos     float64
	vel     float64
	torque  float64
	ts      int64
	at      time.Time
	anomaly Condition // 粘滞异常：收到下一帧有效帧前持续上报
	reason  string
}

// NewMonitor 创建监控器。
func NewMonitor(joint int, cfg Config) *Monitor {
	return &Monitor{joint: joint, cfg: cfg}
}

// Observe 消费一帧反馈。
// 返回的 State 描述本帧判定；异常帧不会替换最新有效反馈，
// 也不参与跟踪误差，跟踪误差只能来自 CondOK 的反馈。
func (m *Monitor) Observe(fb frame.FeedbackData, at time.Time) State {
	if !m.has {
		// 首帧只建立基线，不做差分判定
		m.has = true
		m.seq, m.ts = fb.Seq, fb.Timestamp
		m.pos, m.vel, m.torque, m.at = fb.Position, fb.Velocity, fb.Torque, at
		m.anomaly, m.reason = "", ""
		return m.snapshot(at, CondOK, "首帧建立基线 seq="+u32(fb.Seq))
	}

	// 1) 序号重复
	if fb.Seq == m.seq {
		m.anomaly = CondDuplicate
		m.reason = "重复帧：seq=" + u32(fb.Seq) + " 与最新有效帧相同，丢弃不更新状态"
		return m.snapshot(at, CondDuplicate, m.reason)
	}
	// 2) 序号回退（乱序）
	if fb.Seq < m.seq {
		m.anomaly = CondOutOfOrder
		m.reason = "序号回退：收到 seq=" + u32(fb.Seq) + " < 已接受 seq=" + u32(m.seq)
		return m.snapshot(at, CondOutOfOrder, m.reason)
	}
	// 3) 时间戳回退（乱序）
	if fb.Timestamp < m.ts {
		m.anomaly = CondOutOfOrder
		m.reason = "时间戳回退：收到 ts=" + i64(fb.Timestamp) + " < 已接受 ts=" + i64(m.ts)
		return m.snapshot(at, CondOutOfOrder, m.reason)
	}
	// 4) 时间戳前向跳变
	if d := time.Duration(fb.Timestamp - m.ts); d > m.cfg.MaxForwardSkew {
		m.anomaly = CondTimestampJump
		m.reason = "时间戳前跳 " + d.String() + " 超过阈值 " + m.cfg.MaxForwardSkew.String()
		return m.snapshot(at, CondTimestampJump, m.reason)
	}
	// 5) 位置跳变：扣除按已知速度的预期运动后仍超阈值
	dtSec := at.Sub(m.at).Seconds()
	expected := math.Abs(m.vel) * math.Max(dtSec, 0)
	if jump := math.Abs(fb.Position-m.pos) - expected; jump > m.cfg.MaxPosJump {
		m.anomaly = CondPositionJump
		m.reason = "位置跳变残余 " + f64(jump) + " rad 超过阈值 " + f64(m.cfg.MaxPosJump)
		return m.snapshot(at, CondPositionJump, m.reason)
	}

	// 有效帧：推进基线并清除粘滞异常
	m.seq, m.ts = fb.Seq, fb.Timestamp
	m.pos, m.vel, m.torque, m.at = fb.Position, fb.Velocity, fb.Torque, at
	m.anomaly, m.reason = "", ""
	return m.snapshot(at, CondOK, "有效帧 seq="+u32(fb.Seq)+" 位置="+f64(fb.Position))
}

// Tick 每个控制周期调用：仅依据距最新有效帧的年龄判定暂时失联/丢失。
// 失联判定不改变最新有效反馈值，是否采信该旧值由上层依据 Cond 决定。
func (m *Monitor) Tick(now time.Time) State {
	if !m.has {
		return State{Cond: CondLost, HasData: false, Reason: "从未收到该关节反馈"}
	}
	age := now.Sub(m.at)
	switch {
	case age > m.cfg.LostTimeout:
		return m.snapshot(now, CondLost,
			"已 "+age.String()+" 无有效反馈，超过丢失阈值 "+m.cfg.LostTimeout.String())
	case age > m.cfg.StaleTimeout:
		return m.snapshot(now, CondStale,
			"已 "+age.String()+" 无有效反馈，超过看门狗窗口 "+m.cfg.StaleTimeout.String())
	default:
		if m.anomaly != "" {
			return m.snapshot(now, m.anomaly, m.reason)
		}
		return m.snapshot(now, CondOK, "反馈新鲜")
	}
}

func (m *Monitor) snapshot(at time.Time, cond Condition, reason string) State {
	s := State{
		Cond: cond, LastSeq: m.seq, HasData: m.has,
		LastPos: m.pos, LastVel: m.vel, LastTorque: m.torque,
		LastAt: m.at, Age: at.Sub(m.at), Reason: reason,
	}
	return s
}
