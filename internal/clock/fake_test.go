package clock

import (
	"testing"
	"time"
)

func TestFakeTimerOrdering(t *testing.T) {
	base := time.Date(2026, 9, 22, 0, 0, 0, 0, time.UTC)
	f := NewFake(base)

	// t1 后注册但与 t2 同刻 => 先注册者先触发；t3 截止更晚。
	t2 := f.NewTimer(10 * time.Millisecond)
	t1 := f.NewTimer(10 * time.Millisecond)
	t3 := f.NewTimer(20 * time.Millisecond)

	// 同刻事件均未到期前不触发。
	f.Advance(9 * time.Millisecond)
	select {
	case <-t1.C():
		t.Fatal("t1 fired early")
	default:
	}
	// 在单个 goroutine 内顺序接收：发送顺序严格等于
	// (deadline, 注册序号) 排序，不依赖运行时调度。
	f.Advance(1 * time.Millisecond)
	if got := <-t2.C(); !got.Equal(base.Add(10 * time.Millisecond)) {
		t.Fatalf("t2 deadline = %v", got)
	}
	if got := <-t1.C(); !got.Equal(base.Add(10 * time.Millisecond)) {
		t.Fatalf("t1 deadline = %v", got)
	}
	f.Advance(10 * time.Millisecond)
	if got := <-t3.C(); !got.Equal(base.Add(20 * time.Millisecond)) {
		t.Fatalf("t3 deadline = %v", got)
	}
}

func TestFakeSleepAndStop(t *testing.T) {
	base := time.Unix(0, 0)
	f := NewFake(base)
	ready := make(chan struct{})
	woke := make(chan struct{})
	go func() {
		// 先把定时器注册好，再允许主 goroutine 推进时钟：
		// NewTimer 的截止时刻锚定注册时的 now。
		tm := f.NewTimer(5 * time.Millisecond)
		close(ready)
		<-tm.C()
		close(woke)
	}()
	<-ready
	f.Advance(4 * time.Millisecond)
	select {
	case <-woke:
		t.Fatal("sleeper woke too early")
	default:
	}
	f.Advance(1 * time.Millisecond)
	<-woke

	tm := f.NewTimer(time.Second)
	if !tm.Stop() {
		t.Fatal("active timer should report Stop=true")
	}
	f.Advance(2 * time.Second)
	select {
	case <-tm.C():
		t.Fatal("stopped timer must not fire")
	default:
	}
}

func TestFakeReset(t *testing.T) {
	base := time.Unix(0, 0)
	f := NewFake(base)
	tm := f.NewTimer(10 * time.Millisecond)
	tm.Reset(20 * time.Millisecond)
	f.Advance(15 * time.Millisecond)
	select {
	case <-tm.C():
		t.Fatal("timer fired before reset deadline")
	default:
	}
	f.Advance(5 * time.Millisecond)
	<-tm.C()
}
