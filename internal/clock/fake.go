package clock

import (
	"sort"
	"sync"
	"time"
)

// waiter 表示一个被 Sleep 或 Timer 挂起的等待者。
type waiter struct {
	deadline time.Time
	seq      int
	ch       chan time.Time
	active   bool // 是否仍在等待集合中（未 Stop/未触发）
	fired    bool
}

// Fake 是可手动推进的确定性时钟。
//
// 语义：
//   - Now 始终返回最近一次 Advance 之后的时刻；
//   - Sleep(d) 与 NewTimer(d) 注册一个截止时刻 now+d 的等待者；
//   - Advance(d) 先把时钟一次性推进到 now+d，再按 (截止时刻, 注册顺序)
//     依次唤醒所有已到期等待者，同刻事件的唤醒顺序完全确定；
//   - d<=0 的 Sleep 立即返回；定时器即使时长为 0 也在下次 Advance 时触发，
//     从而避免测试里出现依赖调度器的“立即”竞态。
type Fake struct {
	mu      sync.Mutex
	now     time.Time
	seq     int
	waiters map[*waiter]struct{}
}

// NewFake 创建起始时刻为 start 的假时钟。
func NewFake(start time.Time) *Fake {
	return &Fake{now: start, waiters: map[*waiter]struct{}{}}
}

// Now 返回假时钟当前时刻。
func (f *Fake) Now() time.Time {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.now
}

// Sleep 阻塞直到假时钟被推进至少 d；d<=0 立即返回。
func (f *Fake) Sleep(d time.Duration) {
	if d <= 0 {
		return
	}
	t := f.NewTimer(d).(*fakeTimer)
	<-t.C()
}

// NewTimer 返回由假时钟驱动的定时器。
func (f *Fake) NewTimer(d time.Duration) Timer {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.seq++
	w := &waiter{
		deadline: f.now.Add(d),
		seq:      f.seq,
		ch:       make(chan time.Time, 1),
		active:   true,
	}
	f.waiters[w] = struct{}{}
	return &fakeTimer{f: f, w: w}
}

// remove 把等待者摘出集合，加锁调用。
func (f *Fake) removeLocked(w *waiter) {
	delete(f.waiters, w)
	w.active = false
}

// Advance 将时钟向前推进 d（d<=0 不做事），并确定性地唤醒到期者。
func (f *Fake) Advance(d time.Duration) {
	if d <= 0 {
		return
	}
	f.mu.Lock()
	f.now = f.now.Add(d)
	due := make([]*waiter, 0, len(f.waiters))
	for w := range f.waiters {
		if !w.deadline.After(f.now) {
			due = append(due, w)
		}
	}
	sort.Slice(due, func(i, j int) bool {
		if due[i].deadline.Equal(due[j].deadline) {
			return due[i].seq < due[j].seq
		}
		return due[i].deadline.Before(due[j].deadline)
	})
	for _, w := range due {
		f.removeLocked(w)
		w.fired = true
	}
	// 在锁外发送，避免等待方接收后回调时钟造成死锁。
	f.mu.Unlock()
	for _, w := range due {
		w.ch <- w.deadline
	}
}

// fakeTimer 实现 Timer。
type fakeTimer struct {
	f *Fake
	w *waiter
}

func (t *fakeTimer) C() <-chan time.Time { return t.w.ch }

// Stop 阻止定时器触发；返回调用前定时器是否仍处于活动状态。
func (t *fakeTimer) Stop() bool {
	t.f.mu.Lock()
	defer t.f.mu.Unlock()
	wasActive := t.w.active
	if t.w.active {
		t.f.removeLocked(t.w)
	}
	return wasActive
}

// Reset 修改截止时刻。若定时器已触发或已停止，则重新放回等待集合。
// 返回它在 Reset 前是否活动（与 time.Timer.Reset 语义一致）。
func (t *fakeTimer) Reset(d time.Duration) bool {
	t.f.mu.Lock()
	defer t.f.mu.Unlock()
	wasActive := t.w.active
	t.w.fired = false
	t.w.active = true
	t.w.deadline = t.f.now.Add(d)
	// 清空可能残留的触发值。
	select {
	case <-t.w.ch:
	default:
	}
	t.f.waiters[t.w] = struct{}{}
	return wasActive
}
