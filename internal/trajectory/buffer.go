package trajectory

import (
	"fmt"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
)

// BufferedPoint 是轨迹缓冲中的一个航点。
type BufferedPoint struct {
	Setpoint
	At time.Time
}

// Buffer 是有界轨迹缓冲：积压超限时明确拒绝（不静默裁剪/丢弃），
// 内存使用永远不超过 capacity 个航点。
type Buffer struct {
	capacity int
	items    []BufferedPoint
}

// NewBuffer 创建缓冲。
func NewBuffer(capacity int) *Buffer {
	if capacity <= 0 {
		capacity = 1
	}
	return &Buffer{capacity: capacity}
}

// Append 追加航点。规则：
//   - 缓冲满 => fault(command_rejected/trajectory_buffer_overflow)；
//   - 时间戳早于缓冲中最后一个航点 => non_monotonic_time（轨迹倒流）；
//   - 与最后一个航点同一执行时刻 => 视为对该时刻目标的“替换”：
//     一个执行时刻只允许一个目标，更新它而不是报错或并存。
func (b *Buffer) Append(p BufferedPoint) error {
	if len(b.items) >= b.capacity {
		return fault.New(fault.CatCommandReject, fault.SourceTrajectory, ReasonBufferOverflow,
			fmt.Sprintf("trajectory buffer full (%d points)", b.capacity))
	}
	if n := len(b.items); n > 0 && p.At.Before(b.items[n-1].At) {
		return fault.New(fault.CatCommandReject, fault.SourceTrajectory, ReasonNonMonotonicTime,
			"waypoint timestamp must not be earlier than last buffered waypoint")
	}
	if n := len(b.items); n > 0 && p.At.Equal(b.items[n-1].At) {
		b.items[n-1] = p // 替换同一执行时刻的目标
		return nil
	}
	b.items = append(b.items, p)
	return nil
}

// PopDue 弹出 t 时刻应执行的航点中“最新的一个”（条件 At <= t）。
// 若一次追赶跳过了若干更早的到期航点，skipped 为被跳过数量，调用方
// 据此可观测“周期追赶”而非静默吞点。无到期航点时 ok=false。
func (b *Buffer) PopDue(t time.Time) (p BufferedPoint, skipped int, ok bool) {
	due := -1
	for i, it := range b.items {
		if it.At.After(t) {
			break
		}
		due = i
	}
	if due < 0 {
		return BufferedPoint{}, 0, false
	}
	p = b.items[due]
	b.items = b.items[due+1:]
	return p, due, true
}

// Len 返回缓冲长度。
func (b *Buffer) Len() int { return len(b.items) }

// Clear 清空缓冲（急停时调用）。
func (b *Buffer) Clear() { b.items = b.items[:0] }
