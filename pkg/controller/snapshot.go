package controller

import (
	"strconv"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/pkg/feedback"
	"github.com/xingjianluo/robot-motion-control-go/pkg/limits"
	"github.com/xingjianluo/robot-motion-control-go/pkg/safety"
)

// JointState 单关节对外状态快照（值拷贝，无半更新）。
type JointState struct {
	ID            int
	CmdMode       limits.Mode
	CmdPosition   float64
	RefPosition   float64
	FbPosition    float64
	FbVelocity    float64
	FbTorque      float64
	FbCondition   feedback.Condition
	FbReason      string
	TrackingError float64 // 仅在反馈有效时有意义，失联时恒为 0
	TrajDone      bool
}

// Snapshot 整机状态快照（原子获取）。
type Snapshot struct {
	At           time.Time
	Cycle        uint64
	SafetyState  safety.State
	SafetyReason safety.Event
	Joints       []JointState
	QueueLen     int
	CmdDropped   uint64
	FbBadFrames  uint64
	CommBad      uint64
	LastJitter   time.Duration
	LastPeriod   time.Duration
}

func itoa(n int) string     { return strconv.Itoa(n) }
func ftoa(f float64) string { return strconv.FormatFloat(f, 'f', 4, 64) }
func hex2(b byte) string {
	const h = "0123456789ABCDEF"
	return string([]byte{h[b>>4], h[b&0x0F]})
}
