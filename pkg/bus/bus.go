// Package bus 提供本地模拟总线：确定性收发、故障注入与有界积压。
// 积压超出 Capacity 时按既定策略处理，绝不无界占用内存；
// 丢弃事件带帧归属（方向/关节/类型），可观测、可追溯。
package bus

import (
	"sync"

	"github.com/xingjianluo/robot-motion-control-go/pkg/frame"
)

// DropPolicy 积压超限时的处理策略。
type DropPolicy int

const (
	DropOldest DropPolicy = iota // 丢弃最旧帧
	DropNewest                   // 拒绝/丢弃新帧
)

// DropEvent 记录一次丢弃的归属信息。
type DropEvent struct {
	Dir     string // "RX" 或 "TX"
	Version frame.Version
	Type    frame.Type
	Joint   int
	Reason  string
}

// Stats 总线可观测统计。
type Stats struct {
	Sent     uint64
	Received uint64
	Dropped  uint64
	Events   []DropEvent // 最近丢弃记录（有界，默认保留 64 条）
}

// Config 总线配置。
type Config struct {
	Capacity int
	Policy   DropPolicy
	// FaultInject 在入队前对帧做故障注入：返回 false 模拟该帧在总线上丢失；
	// 可返回篡改后的帧模拟线路损坏。
	FaultInject func(f frame.Frame) (frame.Frame, bool)
}

// Bus 模拟本地总线。内部为单端环形缓冲，控制器从同一队列收发。
type Bus struct {
	mu     sync.Mutex
	cfg    Config
	q      []frame.Frame
	head   int
	size   int
	closed bool
	stats  Stats
}

// New 创建总线。Capacity<=0 时默认为 16。
func New(cfg Config) *Bus {
	if cfg.Capacity <= 0 {
		cfg.Capacity = 16
	}
	return &Bus{cfg: cfg}
}

// Send 发送一帧。积压满时按策略丢弃并计入统计，返回带归属的丢弃说明。
func (b *Bus) Send(f frame.Frame) error {
	if b.cfg.FaultInject != nil {
		injected, ok := b.cfg.FaultInject(f)
		if !ok {
			// 模拟线路丢帧：发送方认为“已发出”，但总线上不存在该帧，
			// 因此由接收方看门狗表现为通信故障，而非发送错误。
			b.mu.Lock()
			b.stats.Sent++
			b.record(DropEvent{
				Dir: "TX", Version: f.Version, Type: f.Type, Joint: f.Joint,
				Reason: "FAULT_INJECT_DROP",
			})
			b.mu.Unlock()
			return nil
		}
		f = injected
	}

	b.mu.Lock()
	defer b.mu.Unlock()
	b.stats.Sent++
	if b.size == b.cfg.Capacity {
		switch b.cfg.Policy {
		case DropOldest:
			old := b.q[b.head]
			b.q[b.head] = frame.Frame{}
			b.head = (b.head + 1) % b.cfg.Capacity
			b.size--
			b.record(DropEvent{
				Dir: "TX", Version: old.Version, Type: old.Type, Joint: old.Joint,
				Reason: "OVERFLOW_DROP_OLDEST",
			})
		case DropNewest:
			b.record(DropEvent{
				Dir: "TX", Version: f.Version, Type: f.Type, Joint: f.Joint,
				Reason: "OVERFLOW_REJECT_NEWEST",
			})
			return errDropped(f)
		}
	}
	b.q = ensureRing(b.q, b.cfg.Capacity)
	if b.size == 0 {
		// 空队列时重置 head，保证行为确定
		b.head = 0
	}
	tail := (b.head + b.size) % b.cfg.Capacity
	b.q[tail] = f
	b.size++
	return nil
}

func ensureRing(q []frame.Frame, cap int) []frame.Frame {
	if len(q) < cap {
		return make([]frame.Frame, cap)
	}
	return q
}

// Recv 非阻塞接收一帧；无帧时返回 ok=false。
func (b *Bus) Recv() (frame.Frame, bool) {
	b.mu.Lock()
	defer b.mu.Unlock()
	if b.size == 0 {
		return frame.Frame{}, false
	}
	f := b.q[b.head]
	b.q[b.head] = frame.Frame{}
	b.head = (b.head + 1) % b.cfg.Capacity
	b.size--
	b.stats.Received++
	return f, true
}

// Pending 返回当前积压帧数。
func (b *Bus) Pending() int {
	b.mu.Lock()
	defer b.mu.Unlock()
	return b.size
}

// Stats 返回可观测统计快照（含丢弃归属）。
func (b *Bus) Stats() Stats {
	b.mu.Lock()
	defer b.mu.Unlock()
	ev := make([]DropEvent, len(b.stats.Events))
	copy(ev, b.stats.Events)
	return Stats{
		Sent: b.stats.Sent, Received: b.stats.Received,
		Dropped: b.stats.Dropped, Events: ev,
	}
}

func (b *Bus) record(e DropEvent) {
	b.stats.Dropped++
	b.stats.Events = append(b.stats.Events, e)
	if len(b.stats.Events) > 64 {
		b.stats.Events = b.stats.Events[len(b.stats.Events)-64:]
	}
}

type droppedError struct{ e DropEvent }

func (d *droppedError) Error() string {
	return "总线积压丢弃: " + d.e.Dir + " joint=" + itoa(d.e.Joint) + " reason=" + d.e.Reason
}

func errDropped(f frame.Frame) error {
	return &droppedError{DropEvent{Dir: "TX", Version: f.Version, Type: f.Type,
		Joint: f.Joint, Reason: "OVERFLOW_REJECT_NEWEST"}}
}

func itoa(n int) string {
	if n == 0 {
		return "0"
	}
	neg := n < 0
	if neg {
		n = -n
	}
	var buf [20]byte
	i := len(buf)
	for n > 0 {
		i--
		buf[i] = byte('0' + n%10)
		n /= 10
	}
	if neg {
		i--
		buf[i] = '-'
	}
	return string(buf[i:])
}
