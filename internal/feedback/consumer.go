// Package feedback 消费驱动器/编码器状态帧，区分以下情形：
//
//   - 正常：序号递增、时间戳单调、位置跳变在物理可信范围内
//   - 重复帧：序号重复
//   - 乱序帧：序号或时间戳回退
//   - 跳变：单周期位置增量超出物理上限（编码器异常/真实失控需区别对待）
//   - 失联：超过看门狗阈值没有收到任何正常帧（区别于真实跟踪误差）
package feedback

import (
	"fmt"
	"math"
	"sync"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
)

// Sample 是一帧被接受的反馈。
type Sample struct {
	PositionRad  float64
	VelocityRadS float64
	TorqueNm     float64
	Enabled      bool
	DriveFault   int
	Seq          uint64
	TsMs         int64
	ReceivedAt   time.Time
	TorqueKnown  bool // V1 状态帧无力矩字段时为 false
}

// Classification 是反馈判定结论。
type Classification string

const (
	ClassAccepted   Classification = "accepted"
	ClassDuplicate  Classification = "duplicate"
	ClassOutOfOrder Classification = "out_of_order"
	ClassJump       Classification = "position_jump"
	ClassStale      Classification = "stale_timestamp"
)

// 通信/反馈原因码：注意它们与协议错误、安全触发完全不同。
const (
	ReasonWatchdogTimeout = "feedback_watchdog_timeout"
	ReasonDuplicateSeq    = "duplicate_seq"
	ReasonOutOfOrder      = "out_of_order_frame"
	ReasonPositionJump    = "position_jump"
	ReasonStaleTimestamp  = "stale_timestamp"
	ReasonLostLink        = "link_temporarily_lost"
)

// Config 是反馈消费参数。
type Config struct {
	WatchdogTimeout time.Duration // 超过此时长无正常帧 => 暂时失联
	MaxStepRad      float64       // 单周期最大可信位置增量（绝对值）
	MaxBackdateMs   int64         // 允许的设备时间戳回退（毫秒）；超过即乱序
}

// Consumer 维护最近正常反馈并做分类。所有方法并发安全。
type Consumer struct {
	mu       sync.Mutex
	cfg      Config
	last     Sample
	hasLast  bool
	lostFlag bool // 已判定失联，恢复前不再重复报看门狗
}

// New 创建反馈消费者。
func New(cfg Config) *Consumer { return &Consumer{cfg: cfg} }

// Ingest 消费一条原始线上帧（解码失败时 decodeErr 非 nil）。
//
// 归类优先级（结果确定、可复现）：
//  1. decodeErr != nil：原样透传协议错误（CatProtocol），
//     绝不把坏帧当反馈接受；
//  2. seq < last.seq：序号回退 => out_of_order；
//     seq == last.seq：重复 => duplicate（时间戳等字段完全一致才算良性，
//     但即便内容不同也只是重复序号的重传，归类不变）；
//  3. 设备时间戳相对上一正常帧回退超过 MaxBackdateMs => out_of_order；
//  4. 位置单步增量超过 MaxStepRad => position_jump（既不是正常跟踪误差
//     也不是失联：这是需要安全联锁处理的可疑样本，标记但不更新基准）；
//  5. 其余 => accepted，并作为新的基准。
//
// 除 accepted 外的分类都返回非 nil 的 fault(CatComm)，
// 且不更新 last——可疑数据绝不污染连续性基准。
func (c *Consumer) Ingest(line string, receivedAt time.Time, decodeErr error) (Sample, Classification, error) {
	if decodeErr != nil {
		// 保持协议归类（CatProtocol），调用方据此区分“链路通但帧坏”。
		return Sample{}, "", decodeErr
	}
	frame, derr := protocol.Decode(line)
	if derr != nil {
		return Sample{}, "", derr
	}
	st, torquePresent, perr := protocol.DecodeStatus(frame)
	if perr != nil {
		return Sample{}, "", perr
	}
	s := Sample{
		PositionRad: st.PositionRad, VelocityRadS: st.VelocityRadS,
		TorqueNm: st.TorqueNm, Enabled: st.Enabled, DriveFault: st.FaultCode,
		Seq: st.Seq, TsMs: st.TsMs, ReceivedAt: receivedAt, TorqueKnown: torquePresent,
	}

	c.mu.Lock()
	defer c.mu.Unlock()

	if c.hasLast {
		switch {
		case s.Seq < c.last.Seq:
			return s, ClassOutOfOrder, commFault(ReasonOutOfOrder,
				fmt.Sprintf("seq %d < last %d", s.Seq, c.last.Seq))
		case s.Seq == c.last.Seq:
			return s, ClassDuplicate, commFault(ReasonDuplicateSeq,
				fmt.Sprintf("seq %d already accepted", s.Seq))
		}
		if c.last.TsMs > 0 && s.TsMs > 0 && c.last.TsMs-s.TsMs > c.cfg.MaxBackdateMs {
			return s, ClassOutOfOrder, commFault(ReasonOutOfOrder,
				fmt.Sprintf("device timestamp backdate %dms > allowed %dms",
					c.last.TsMs-s.TsMs, c.cfg.MaxBackdateMs))
		}
		if step := math.Abs(s.PositionRad - c.last.PositionRad); step > c.cfg.MaxStepRad {
			return s, ClassJump, commFault(ReasonPositionJump,
				fmt.Sprintf("position step %.4f rad exceeds max %.4f", step, c.cfg.MaxStepRad))
		}
	}

	c.last = s
	c.hasLast = true
	c.lostFlag = false
	return s, ClassAccepted, nil
}

// WatchdogCheck 在每个控制周期调用：距上次正常帧超时则返回
// fault(CatComm, feedback_watchdog_timeout) 表示“暂时失联”。
// 失联期间持续调用只返回同一结论一次置位、之后返回带
// link_temporarily_lost 的稳定结论，直到新正常帧到达清除。
// 这两类结论都不是位置偏差：调用方禁止据此计算跟踪误差。
func (c *Consumer) WatchdogCheck(now time.Time) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	if !c.hasLast {
		return commFault(ReasonWatchdogTimeout, "no feedback frame ever received")
	}
	if now.Sub(c.last.ReceivedAt) <= c.cfg.WatchdogTimeout {
		return nil
	}
	if c.lostFlag {
		return commFault(ReasonLostLink, "link still lost, waiting for valid frame")
	}
	c.lostFlag = true
	return commFault(ReasonWatchdogTimeout,
		fmt.Sprintf("no valid frame for >%v", c.cfg.WatchdogTimeout))
}

// IsLost 返回当前是否处于失联状态。
func (c *Consumer) IsLost() bool {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.lostFlag
}

// Last 返回最近被接受的样本；从未收到时 ok=false。
func (c *Consumer) Last() (Sample, bool) {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.last, c.hasLast
}

// Reset 清空历史（重新使能后调用）。
func (c *Consumer) Reset() {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.last = Sample{}
	c.hasLast = false
	c.lostFlag = false
}

func commFault(reason, detail string) error {
	return fault.New(fault.CatComm, fault.SourceFeedback, reason, detail)
}
