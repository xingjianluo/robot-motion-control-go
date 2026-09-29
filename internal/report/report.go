// Package report 收集状态快照、安全变迁与带归属故障，并以可读、
// 可复现的方式输出，供测试打印“输入、判定依据、最终状态”。
package report

import (
	"fmt"
	"io"
	"strings"
	"sync"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/control"
	"github.com/xingjianluo/robot-motion-control-go/internal/safety"
)

// FaultEntry 是一条带归属的故障记录。
type FaultEntry struct {
	At       time.Time
	Category string
	Source   string
	Reason   string
	Detail   string
}

// Recorder 线程安全地记录快照/安全事件/故障，并实现 control.EventSink。
type Recorder struct {
	mu     sync.Mutex
	w      io.Writer
	cycles []control.Snapshot
	faults []FaultEntry
	safety []safety.Event
}

// NewRecorder 创建记录器；w 用于 Dump 输出。
func NewRecorder(w io.Writer) *Recorder { return &Recorder{w: w} }

// RecordCycle 实现 control.EventSink。
func (r *Recorder) RecordCycle(s control.Snapshot) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.cycles = append(r.cycles, s)
}

// RecordSnapshot 直接记录一个快照（与 RecordCycle 等价，便于外部使用）。
func (r *Recorder) RecordSnapshot(s control.Snapshot) { r.RecordCycle(s) }

// RecordFault 实现 control.EventSink。
func (r *Recorder) RecordFault(at time.Time, category, source, reason, detail string) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.faults = append(r.faults, FaultEntry{
		At: at, Category: category, Source: source, Reason: reason, Detail: detail,
	})
}

// RecordSafety 记录安全状态机变迁。
func (r *Recorder) RecordSafety(e safety.Event) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.safety = append(r.safety, e)
}

// Cycles 返回周期快照副本。
func (r *Recorder) Cycles() []control.Snapshot {
	r.mu.Lock()
	defer r.mu.Unlock()
	out := make([]control.Snapshot, len(r.cycles))
	copy(out, r.cycles)
	return out
}

// Faults 返回故障记录副本。
func (r *Recorder) Faults() []FaultEntry {
	r.mu.Lock()
	defer r.mu.Unlock()
	out := make([]FaultEntry, len(r.faults))
	copy(out, r.faults)
	return out
}

// SafetyEvents 返回安全变迁副本。
func (r *Recorder) SafetyEvents() []safety.Event {
	r.mu.Lock()
	defer r.mu.Unlock()
	out := make([]safety.Event, len(r.safety))
	copy(out, r.safety)
	return out
}

// AttachSafety 返回一个回调，供外部把安全事件同步进记录器。
// （测试中可在每个 Step 后手工 RecordSafety(machine.Events() 的新增项)。）
func (r *Recorder) SyncSafety(events []safety.Event) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.safety = append(r.safety, events...)
}

// FmtSnapshot 把快照格式化为多行可读文本。
func FmtSnapshot(s control.Snapshot) string {
	var b strings.Builder
	fmt.Fprintf(&b, "cycle=%d at=%v state=%s outputs=%v", s.Cycle, s.At.Format("15:04:05.000"),
		s.SafetyState, s.OutputsOn)
	fmt.Fprintf(&b, "\n  feedback: accepted=%d rejected=%d backlog=%d tracking_healthy=%v lastSeq=%d",
		s.FeedbackAccepted, s.FeedbackRejected, s.BacklogRemaining, s.Tracking.Healthy, s.Tracking.LastSeq)
	if s.Tracking.Healthy {
		fmt.Fprintf(&b, " errPos=%.4f errVel=%.4f", s.Tracking.ErrorRad, s.Tracking.ErrorRadS)
	}
	fmt.Fprintf(&b, "\n  bus: enq(up/down)=%d/%d deq=%d/%d dropFull=%d dropStale=%d injLoss=%d injCorrupt=%d",
		s.BusStats.UpEnqueued, s.BusStats.DownEnqueued, s.BusStats.UpDequeued, s.BusStats.DownDequeued,
		s.BusStats.DroppedFull, s.BusStats.DroppedStale, s.BusStats.InjectedLoss, s.BusStats.InjectedCorrupt)
	if s.BusStats.LastDropReason != "" {
		fmt.Fprintf(&b, " lastDrop=%s(seq=%d,dir=%s)", s.BusStats.LastDropReason, s.BusStats.LastDropSeq, s.BusStats.LastDropDir)
	}
	fmt.Fprintf(&b, "\n  jitter: last=%v max=%v breaches=%d", s.LastJitter, s.MaxJitter, s.JitterBreaches)
	if s.StreamActive || s.StreamQueued > 0 {
		fmt.Fprintf(&b, "\n  stream: active=%v queued=%d override=%.2f reached=%d appended=%d rejected=%d evicted=%d overrideRejects=%d",
			s.StreamActive, s.StreamQueued, s.StreamFactor,
			s.StreamStats.Reached, s.StreamStats.Appended, s.StreamStats.Rejected,
			s.StreamStats.Evicted, s.StreamStats.OverrideRejects)
		if s.StreamStats.LastEvent != "" {
			fmt.Fprintf(&b, " lastEvent=%s", s.StreamStats.LastEvent)
		}
	}
	if s.LastFault.Reason != "" {
		fmt.Fprintf(&b, "\n  LAST_FAULT: %s", s.LastFault.String())
	}
	return b.String()
}

// Dump 输出全部记录，顺序为：安全变迁、故障归属流、每周期最终状态。
func (r *Recorder) Dump() {
	r.mu.Lock()
	defer r.mu.Unlock()
	fmt.Fprintln(r.w, "================ SAFETY TRANSITIONS ================")
	for _, e := range r.safety {
		fmt.Fprintf(r.w, "%v %s -> %s  reason=%s  %s\n",
			e.At.Format("15:04:05.000"), e.From, e.To, e.Reason, e.Detail)
	}
	fmt.Fprintln(r.w, "================ CLASSIFIED FAULTS ================")
	for _, f := range r.faults {
		fmt.Fprintf(r.w, "%v [%s|%s|%s] %s\n",
			f.At.Format("15:04:05.000"), f.Category, f.Source, f.Reason, f.Detail)
	}
	fmt.Fprintln(r.w, "================ CYCLE SNAPSHOTS ================")
	for _, s := range r.cycles {
		fmt.Fprintln(r.w, FmtSnapshot(s))
	}
}
