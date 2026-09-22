// Package bus 提供本地进程内的“总线”模拟：
// 有界队列 + 可配置丢弃策略 + 截止期陈旧帧淘汰 + 故障注入，
// 用来模拟驱动器/编码器链路的积压、丢失与重复。
package bus

import (
	"sync"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/clock"
	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
)

// DropPolicy 队列满时的策略。
type DropPolicy string

const (
	// DropOldest 丢弃最旧消息，保证新消息入队（有界内存）。
	DropOldest DropPolicy = "drop_oldest"
	// DropNewest 丢弃新到消息并返回错误，由调用方决定如何降级。
	DropNewest DropPolicy = "drop_newest"
)

// Config 是总线参数，均可配置以便复现不同积压场景。
type Config struct {
	Capacity     int
	Policy       DropPolicy
	MaxAge       time.Duration // 超过该年龄的帧在出队时判定为陈旧并丢弃；0 表示不检查
	TickInterval time.Duration // 仅用于真实运行时轮询节拍参考，模拟逻辑不依赖它
}

// Direction 表示消息方向。
type Direction string

const (
	Downlink Direction = "downlink" // 控制器 -> 驱动器
	Uplink   Direction = "uplink"   // 编码器/驱动器 -> 控制器
)

// 通信原因码（CatComm）。
const (
	ReasonQueueFull       = "queue_full_dropped"
	ReasonStaleFrame      = "stale_frame_dropped"
	ReasonInjectedLoss    = "injected_frame_loss"
	ReasonInjectedCorrupt = "injected_frame_corrupt"
)

// Stats 暴露可观测的积压/丢弃/超时归属；字段按方向拆开。
type Stats struct {
	UpEnqueued      int64
	UpDequeued      int64
	DownEnqueued    int64
	DownDequeued    int64
	DroppedFull     int64
	DroppedStale    int64
	InjectedLoss    int64
	InjectedDup     int64
	InjectedCorrupt int64
	// LastDropReason 为最近一次丢弃/损坏的稳定原因码与归属。
	LastDropSeq    uint64
	LastDropReason string
	LastDropDir    Direction
}

type entry struct {
	line       string
	seq        uint64
	enqueuedAt time.Time
}

type dirQueue struct {
	items []entry
}

// Bus 是有界、线程安全的模拟总线。
type Bus struct {
	cfg  Config
	clk  clock.Clock
	mu   sync.Mutex
	up   dirQueue
	down dirQueue

	postCount int
	inj       Injection
	stats     Stats
}

// New 创建总线；容量非法时按 1 处理，保证内存永远有界。
func New(cfg Config, clk clock.Clock) *Bus {
	if cfg.Capacity <= 0 {
		cfg.Capacity = 1
	}
	if cfg.Policy == "" {
		cfg.Policy = DropOldest
	}
	return &Bus{cfg: cfg, clk: clk}
}

// Post 向指定方向投递一帧。
//
// 处理顺序（每一步都可观测、有归属）：
//  1. 故障注入计划命中：丢帧直接返回 CatComm(injected_frame_loss)；
//     损坏改写内容（接收方将归类为协议错误）；复制命中会多投一份；
//  2. 队列满：按 DropOldest/DropNewest 处理，计入 DroppedFull；
//  3. 正常入队，长度永不超过 Capacity。
func (b *Bus) Post(dir Direction, line string, seq uint64) error {
	b.mu.Lock()
	defer b.mu.Unlock()

	b.postCount++
	n := b.postCount
	lost := containsInt(b.inj.DropAt, n)
	dup := containsInt(b.inj.DuplicateAt, n)
	corrupt := containsInt(b.inj.CorruptAt, n)

	if lost {
		b.stats.InjectedLoss++
		b.recordDropLocked(dir, seq, ReasonInjectedLoss)
		return fault.New(fault.CatComm, fault.SourceBus, ReasonInjectedLoss,
			"frame lost by fault injection")
	}
	if corrupt {
		b.stats.InjectedCorrupt++
		b.recordDropLocked(dir, seq, ReasonInjectedCorrupt)
		line = line + "<<<CORRUPT"
	}
	if dup {
		b.stats.InjectedDup++
	}
	// 复制命中时投递两份完全相同的条目（同序号、同入队时刻）。
	copies := 1
	if dup {
		copies = 2
	}
	for i := 0; i < copies; i++ {
		if err := b.enqueueLocked(dir, entry{line: line, seq: seq, enqueuedAt: b.clk.Now()}); err != nil {
			return err
		}
	}
	return nil
}

func (b *Bus) enqueueLocked(dir Direction, e entry) error {
	q := b.queueLocked(dir)
	if len(q.items) >= b.cfg.Capacity {
		b.stats.DroppedFull++
		switch b.cfg.Policy {
		case DropNewest:
			b.recordDropLocked(dir, e.seq, ReasonQueueFull)
			return fault.New(fault.CatComm, fault.SourceBus, ReasonQueueFull,
				"backlog at capacity, newest frame rejected by policy")
		default: // DropOldest
			dropped := q.items[0]
			q.items = q.items[1:]
			b.recordDropLocked(dir, dropped.seq, ReasonQueueFull)
		}
	}
	q.items = append(q.items, e)
	if dir == Uplink {
		b.stats.UpEnqueued++
	} else {
		b.stats.DownEnqueued++
	}
	b.putQueueLocked(dir, q)
	return nil
}

// Poll 取走一条消息；无消息时返回 ok=false，绝不阻塞。
// 出队时若帧年龄超过 MaxAge，则计为超时丢弃（DroppedStale）并继续
// 向后取，直到取到新鲜帧或队列空——超时帧不会被伪装成正常反馈。
func (b *Bus) Poll(dir Direction) (string, bool) {
	b.mu.Lock()
	defer b.mu.Unlock()
	now := b.clk.Now()
	q := b.queueLocked(dir)
	for len(q.items) > 0 {
		head := q.items[0]
		q.items = q.items[1:]
		if b.cfg.MaxAge > 0 && now.Sub(head.enqueuedAt) > b.cfg.MaxAge {
			b.stats.DroppedStale++
			b.recordDropLocked(dir, head.seq, ReasonStaleFrame)
			continue
		}
		if dir == Uplink {
			b.stats.UpDequeued++
		} else {
			b.stats.DownDequeued++
		}
		b.putQueueLocked(dir, q)
		return head.line, true
	}
	b.putQueueLocked(dir, q)
	return "", false
}

// Pending 返回某方向当前排队长度（骨架测试用）。
func (b *Bus) Pending(dir Direction) int {
	b.mu.Lock()
	defer b.mu.Unlock()
	return len(b.queueLocked(dir).items)
}

func (b *Bus) queueLocked(dir Direction) dirQueue {
	if dir == Uplink {
		return b.up
	}
	return b.down
}

func (b *Bus) putQueueLocked(dir Direction, q dirQueue) {
	if dir == Uplink {
		b.up = q
	} else {
		b.down = q
	}
}

func (b *Bus) recordDropLocked(dir Direction, seq uint64, reason string) {
	b.stats.LastDropSeq = seq
	b.stats.LastDropReason = reason
	b.stats.LastDropDir = dir
}

// Stats 返回统计快照（值拷贝，调用方无需持锁）。
func (b *Bus) Stats() Stats {
	b.mu.Lock()
	defer b.mu.Unlock()
	return b.stats
}

// Close 释放总线资源（模拟实现无后台资源）。
func (b *Bus) Close() {}

func containsInt(xs []int, v int) bool {
	for _, x := range xs {
		if x == v {
			return true
		}
	}
	return false
}
