package stream

import (
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
)

// 挤兑后若新目标拼接仍不可行被拒：被挤段不恢复（固定规则），
// 但队列保持自洽（剩余段仍能连续执行、内存有界）。
func TestEdgeEvictThenRejectKeepsQueueConsistent(t *testing.T) {
	p := New(testCfg(2, PolicyEvictOldest))
	mk := func(seq uint64, v float64) Target {
		return Target{Mode: protocol.ModeVelocity, Value: v, Hold: period, SeqIn: seq}
	}
	mustAppend(t, p, mk(1, 0.1), 0)
	mustAppend(t, p, mk(2, 0.1), 0)
	_ = runN(t, p, 1, 0)            // 段1 激活；等待=[2]
	mustAppend(t, p, mk(3, 0.1), 0) // 等待=[2,3]
	// 段4 与队尾段3 拼接超速（差1.9 =>190/s>100）：先挤掉段2腾位，
	// 随后拼接检查失败 -> 段4 被拒；段2 不恢复，段1/段3 仍可执行。
	ev, err := p.Append(mk(4, 2.0), 0, true, time.Unix(0, 0))
	if err == nil {
		t.Fatal("infeasible-after-eviction target must still be rejected")
	}
	if len(ev) != 1 || ev[0].SourceSeq != 2 {
		t.Fatalf("eviction of seq2 expected, got %v", ev)
	}
	if p.WaitingDepth() != 1 { // 只剩段3
		t.Fatalf("waiting depth=%d, want 1 (seg3)", p.WaitingDepth())
	}
	// 段1 剩 0（baseN=1 已跑完）-> 段3 接着执行，不出现段4。
	d := runN(t, p, 3, 0)
	for _, e := range d {
		if e.SourceSeq == 4 || e.SourceSeq == 2 {
			t.Fatalf("rejected/evicted segment dispatched: %+v", e)
		}
	}
}

// Reset 后倍率=1、队列空；再次 SetOverride 到 200% 只对新流负责。
func TestEdgeResetThenOverrideFreshContext(t *testing.T) {
	p := New(testCfg(8, PolicyRejectNew))
	mustAppend(t, p, Target{Mode: protocol.ModeVelocity, Value: 0.5, Hold: 4 * period, SeqIn: 1}, 0)
	_ = runN(t, p, 1, 0)
	if err := p.SetOverride(0.7, time.Unix(0, 0)); err != nil {
		t.Fatal(err)
	}
	p.Reset()
	if p.Override() != 1.0 || p.Active() || p.Status().DispatchedCount != 0 {
		t.Fatalf("reset not clean: %+v", p.Status())
	}
	// 重置后 seq 不复位：继续单调。
	d := runN(t, p, 0, 0)
	_ = d
	mustAppend(t, p, Target{Mode: protocol.ModeVelocity, Value: 1.0, Hold: 1 * period, SeqIn: 99}, 0)
	out := runN(t, p, 1, 0)
	if len(out) != 1 || out[0].Seq <= 1 || out[0].AtOverride != 1.0 {
		t.Fatalf("post-reset dispatch must use fresh override 1.0 and monotonic seq: %+v", out)
	}
}
