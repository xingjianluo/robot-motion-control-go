package bus

import (
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/clock"
	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
)

func TestDropOldestBounded(t *testing.T) {
	fc := clock.NewFake(time.Unix(0, 0))
	b := New(Config{Capacity: 2, Policy: DropOldest}, fc)
	for i := 1; i <= 4; i++ {
		if err := b.Post(Uplink, "frame", uint64(i)); err != nil {
			t.Fatalf("post %d: %v", i, err)
		}
	}
	st := b.Stats()
	if st.DroppedFull != 2 || st.UpEnqueued != 4 || b.Pending(Uplink) != 2 {
		t.Fatalf("stats wrong: %+v pending=%d", st, b.Pending(Uplink))
	}
	// 容量 2 => 队列中应是 seq 3、4，即最旧的 1、2 被丢弃。
	_, ok1 := b.Poll(Uplink) // seq 3
	_, ok2 := b.Poll(Uplink) // seq 4
	_, ok3 := b.Poll(Uplink) // 空
	if !ok1 || !ok2 || ok3 {
		t.Fatal("queue length after overflow wrong")
	}
	if st.LastDropReason != ReasonQueueFull || st.LastDropSeq != 2 {
		t.Fatalf("drop attribution wrong: %+v", st)
	}
}

func TestDropNewestRejectsWithReason(t *testing.T) {
	fc := clock.NewFake(time.Unix(0, 0))
	b := New(Config{Capacity: 1, Policy: DropNewest}, fc)
	if err := b.Post(Uplink, "a", 1); err != nil {
		t.Fatal(err)
	}
	err := b.Post(Uplink, "b", 2)
	fe, ok := fault.As(err)
	if !ok || fe.Reason != ReasonQueueFull || fe.Category != fault.CatComm {
		t.Fatalf("want CatComm/%s, got %v", ReasonQueueFull, err)
	}
	if b.Pending(Uplink) != 1 {
		t.Fatal("newest must be rejected, oldest retained")
	}
}

func TestStaleFrameExpiry(t *testing.T) {
	fc := clock.NewFake(time.Unix(0, 0))
	b := New(Config{Capacity: 4, MaxAge: 10 * time.Millisecond}, fc)
	if err := b.Post(Uplink, "old", 1); err != nil {
		t.Fatal(err)
	}
	fc.Advance(15 * time.Millisecond)
	if err := b.Post(Uplink, "fresh", 2); err != nil {
		t.Fatal(err)
	}
	line, ok := b.Poll(Uplink)
	if !ok || line != "fresh" {
		t.Fatalf("stale frame must be skipped, got %q %v", line, ok)
	}
	if st := b.Stats(); st.DroppedStale != 1 || st.LastDropSeq != 1 ||
		st.LastDropReason != ReasonStaleFrame {
		t.Fatalf("stale stats wrong: %+v", st)
	}
}

func TestInjectionLossDupCorrupt(t *testing.T) {
	fc := clock.NewFake(time.Unix(0, 0))
	b := New(Config{Capacity: 8}, fc)
	b.Inject(Injection{DropAt: []int{1}, DuplicateAt: []int{2}, CorruptAt: []int{3}})

	if err := b.Post(Uplink, "x", 1); err == nil {
		t.Fatal("injected loss must surface CatComm error")
	} else if fe, ok := fault.As(err); !ok || fe.Reason != ReasonInjectedLoss {
		t.Fatalf("loss classification: %v", err)
	}

	if err := b.Post(Uplink, "dup", 2); err != nil {
		t.Fatal(err)
	}
	l1, _ := b.Poll(Uplink)
	l2, _ := b.Poll(Uplink)
	if l1 != "dup" || l2 != "dup" {
		t.Fatalf("duplicate injection wrong: %q %q", l1, l2)
	}
	_, extra := b.Poll(Uplink)
	if extra {
		t.Fatal("exactly two copies expected")
	}

	if err := b.Post(Uplink, "good", 3); err != nil {
		t.Fatal(err)
	}
	l3, _ := b.Poll(Uplink)
	if l3 != "good<<<CORRUPT" {
		t.Fatalf("corrupt injection wrong: %q", l3)
	}
	st := b.Stats()
	if st.InjectedLoss != 1 || st.InjectedDup != 1 || st.InjectedCorrupt != 1 {
		t.Fatalf("injection stats: %+v", st)
	}
}
