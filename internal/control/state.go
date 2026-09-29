package control

import (
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/bus"
	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
	"github.com/xingjianluo/robot-motion-control-go/internal/safety"
)

// TrackingInfo 是跟踪误差观测；只有链路健康、样本被接受时才有意义。
type TrackingInfo struct {
	Healthy   bool
	ErrorRad  float64
	ErrorRadS float64
	LastSeq   uint64
	LastTsMs  int64
}

// FaultRef 是最近一次被归类故障的结构化归属，三类来源绝不混用。
type FaultRef struct {
	Category fault.Category
	Source   fault.Source
	Reason   string
	Message  string
	At       time.Time
}

func (f FaultRef) String() string {
	if f.Reason == "" {
		return ""
	}
	return string(f.Category) + "|" + string(f.Source) + "|" + f.Reason + ": " + f.Message
}

// Snapshot 是一个控制周期边界上的完整状态。
// 所有字段在控制器单把锁下一次性填充，因此读取者要么看到完整的
// 上一周期状态，要么看到完整的当前周期状态，不存在半更新。
type Snapshot struct {
	At          time.Time
	Cycle       uint64
	SafetyState safety.State
	OutputsOn   bool
	LastCommand protocol.MotorCommand
	Tracking    TrackingInfo
	BusStats    bus.Stats
	// 周期抖动观测：实际周期相对配置周期的偏差与超限次数。
	LastJitter     time.Duration
	MaxJitter      time.Duration
	JitterBreaches int64
	// 本周期内各类反馈分类计数（accepted/duplicate/...）。
	FeedbackAccepted int
	FeedbackRejected int
	BacklogRemaining int
	LastFault        FaultRef
	// Stream 是流式喂点/在线速度倍率的可观测状态（纯增量字段，
	// 不影响原有快照读取方）。
	Stream StreamInfo
}
