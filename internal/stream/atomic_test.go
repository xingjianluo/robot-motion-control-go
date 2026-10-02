package stream

// 本文件覆盖 Append 的“事务性”与通路交接原语：
//   - 任何校验拒绝都不得改动 pending/gaps/cruise/锚点/保持状态，
//     即被拒命令只影响它自己；
//   - 末点保持期间被拒（迟到/倒流/不可行/越界/队满）后，Advance 仍
//     持续输出保持位置，运动状态与拒绝前一致；
//   - 两类拒绝原因可分：stream_position_out_of_limit（普通越界）与
//     stream_segment_too_fast（按当前状态走不出来）；
//   - ReleaseHold 只在流排空后退出保持，活动流上为空操作。

import (
	"testing"
	"time"
)

// drainPoint 推进到第一个点（0.2@+0.3s）被到达并进入末点保持。
func drainToHolding(t *testing.T, p *Planner, t0 time.Time) {
	t.Helper()
	now := t0
	if err := p.Append(Point{At: t0.Add(300 * time.Millisecond), PositionRad: 0.2}, now, 0, true); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 60; i++ {
		now = now.Add(10 * time.Millisecond)
		p.Advance(now)
	}
	if !p.Active() || p.Len() != 0 {
		t.Fatalf("前置：应排空并保持, active=%v len=%d", p.Active(), p.Len())
	}
}

// TestAppendRejectionAtomicWhileHolding 末点保持期喂坏点：拒绝前后
// pending/保持位置/Advance 输出必须完全不变。
func TestAppendRejectionAtomicWhileHolding(t *testing.T) {
	t0 := time.Date(2026, 9, 29, 0, 0, 0, 0, time.UTC)
	p := New(testCfg(8, RejectNew))
	now := t0.Add(600 * time.Millisecond)
	drainToHolding(t, p, t0)
	// drainToHolding 内部时钟已推进到 0.6s；把外部 now 对齐。
	now = t0.Add(600 * time.Millisecond)

	holdOut := p.Advance(now)
	if !holdOut.Active || holdOut.PositionRad != 0.2 {
		t.Fatalf("保持输出错误: %+v", holdOut)
	}

	cases := []struct {
		name   string
		pt     Point
		reason string
	}{
		{"迟到点(At=now)", Point{At: now, PositionRad: 0.21}, ReasonLatePoint},
		{"不可行段速", Point{At: now.Add(50 * time.Millisecond), PositionRad: 1.9}, ReasonSegmentTooFast},
		{"普通位置越界", Point{At: now.Add(300 * time.Millisecond), PositionRad: 2.5}, ReasonPositionOutOfLimit},
	}
	for _, c := range cases {
		err := p.Append(c.pt, now, 0.2, true)
		mustFail(t, err, c.reason)
		if p.Len() != 0 {
			t.Fatalf("%s: 拒绝后 pending 非空 len=%d", c.name, p.Len())
		}
		if !p.Active() {
			t.Fatalf("%s: 拒绝破坏了保持状态", c.name)
		}
		out := p.Advance(now.Add(10 * time.Millisecond))
		if !out.Active || out.PositionRad != 0.2 || out.VelocityRadS != 0 {
			t.Fatalf("%s: 拒绝后保持输出被改变: %+v", c.name, out)
		}
		now = now.Add(10 * time.Millisecond)
		t.Logf("输入: 保持0.2 喂 %s; 判定: command_rejected|stream|%s; 最终状态: 继续保持 0.2",
			c.name, c.reason)
	}
}

// TestAppendRejectionAtomicWithPending 有排队点时，坏点（倒流/不可行/
// 越界/迟到）不得改动队列；随后好点仍可追加，全部原排点照常到达。
func TestAppendRejectionAtomicWithPending(t *testing.T) {
	t0 := time.Date(2026, 9, 29, 0, 0, 0, 0, time.UTC)
	p := New(testCfg(8, RejectNew))
	now := t0
	if err := p.Append(Point{At: t0.Add(400 * time.Millisecond), PositionRad: 0.1}, now, 0, true); err != nil {
		t.Fatal(err)
	}
	if err := p.Append(Point{At: t0.Add(800 * time.Millisecond), PositionRad: 0.2}, now, 0, true); err != nil {
		t.Fatal(err)
	}
	bad := []struct {
		pt     Point
		reason string
	}{
		{Point{At: t0.Add(500 * time.Millisecond), PositionRad: 0.15}, ReasonBackwardsTime},
		{Point{At: t0.Add(850 * time.Millisecond), PositionRad: 1.9}, ReasonSegmentTooFast},
		{Point{At: t0.Add(1200 * time.Millisecond), PositionRad: 3}, ReasonPositionOutOfLimit},
		{Point{At: now, PositionRad: 0.1}, ReasonLatePoint},
	}
	for i, c := range bad {
		if err := p.Append(c.pt, now, 0, true); err == nil {
			t.Fatalf("case %d 应被拒绝", i)
		} else {
			mustFail(t, err, c.reason)
		}
		if p.Len() != 2 {
			t.Fatalf("case %d(%s) 拒绝改变了队列长度: %d", i, c.reason, p.Len())
		}
	}
	t.Logf("输入: 队列2点时连续喂4类坏点; 判定: 全部拒绝, 队列恒为2")

	// 原排点照常走完。
	reached := 0
	for i := 0; i < 100; i++ {
		now = now.Add(10 * time.Millisecond)
		out := p.Advance(now)
		if out.Reached {
			reached++
		}
	}
	if reached != 2 {
		t.Fatalf("原排队点未全部到达: %d", reached)
	}
	if !p.Active() {
		t.Fatal("排空后应保持")
	}
	out := p.Advance(now.Add(10 * time.Millisecond))
	if out.PositionRad != 0.2 {
		t.Fatalf("末点保持位置错误: %+v", out)
	}
	t.Logf("最终状态: reached=%d 保持位置=%.2f", reached, out.PositionRad)
}

// TestAppendQueueFullAtomic reject_new 下队满拒绝不得改动任何状态。
func TestAppendQueueFullAtomic(t *testing.T) {
	t0 := time.Date(2026, 9, 29, 0, 0, 0, 0, time.UTC)
	p := New(testCfg(2, RejectNew))
	now := t0
	for i := 1; i <= 2; i++ {
		if err := p.Append(Point{At: t0.Add(time.Duration(i) * 200 * time.Millisecond),
			PositionRad: 0.05 * float64(i)}, now, 0, true); err != nil {
			t.Fatal(err)
		}
	}
	before := p.Stats()
	mustFail(t, p.Append(Point{At: t0.Add(600 * time.Millisecond), PositionRad: 0.2}, now, 0, true),
		ReasonQueueFull)
	if p.Len() != 2 {
		t.Fatalf("队满拒绝改变了队列: %d", p.Len())
	}
	if p.Stats().Appended != before.Appended || p.Stats().Evicted != before.Evicted {
		t.Fatalf("队满拒绝污染了计数: before=%+v after=%+v", before, p.Stats())
	}
}

// TestRejectionReasonsDistinct 硬限位越界与段速不可行是两类不同拒绝。
func TestRejectionReasonsDistinct(t *testing.T) {
	t0 := time.Date(2026, 9, 29, 0, 0, 0, 0, time.UTC)
	p := New(testCfg(8, RejectNew))
	now := t0
	// 普通越界：位置超出 [-2,2]，但时间给得很宽（段速本身可行）。
	mustFail(t, p.Append(Point{At: now.Add(10 * time.Second), PositionRad: 2.01}, now, 0, true),
		ReasonPositionOutOfLimit)
	// 按当前状态走不出来：位置在限内，但时间太短、段速超过最大速度。
	mustFail(t, p.Append(Point{At: now.Add(10 * time.Millisecond), PositionRad: 1.0}, now, 0, true),
		ReasonSegmentTooFast)
	// 两次拒绝都没有建立锚点/队列（流从未启动）。
	if p.Active() || p.Len() != 0 {
		t.Fatalf("流未成功启动时不应有活动状态: active=%v len=%d", p.Active(), p.Len())
	}
	// 随后一个正常点应能建立流并正常推进。
	if err := p.Append(Point{At: now.Add(300 * time.Millisecond), PositionRad: 0.2}, now, 0, true); err != nil {
		t.Fatalf("拒绝后正常点应被接受: %v", err)
	}
	if !p.Active() || p.Len() != 1 {
		t.Fatalf("正常点未建立流: active=%v len=%d", p.Active(), p.Len())
	}
	t.Log("输入: 先越界、再段速不可行、最后正常点; 判定: 原因码可分且拒绝不污染; 最终状态: 正常点入队")
}

// TestReleaseHoldSemantics ReleaseHold 只在排空后交接，活动流上空操作；
// 交接后空闲 Advance 不再产生位置设定。
func TestReleaseHoldSemantics(t *testing.T) {
	t0 := time.Date(2026, 9, 29, 0, 0, 0, 0, time.UTC)

	// 活动流：ReleaseHold 必须是空操作。
	p := New(testCfg(8, RejectNew))
	now := t0
	if err := p.Append(Point{At: now.Add(300 * time.Millisecond), PositionRad: 0.2}, now, 0, true); err != nil {
		t.Fatal(err)
	}
	if p.ReleaseHold() {
		t.Fatal("活动流上 ReleaseHold 不应生效")
	}
	if p.Len() != 1 {
		t.Fatalf("活动流被 ReleaseHold 改动: len=%d", p.Len())
	}

	// 排空保持：ReleaseHold 生效，之后 Advance 不再下发保持位置。
	drainToHolding(t, p, t0)
	if !p.ReleaseHold() {
		t.Fatal("排空保持期 ReleaseHold 应返回 true")
	}
	if p.Active() {
		t.Fatal("交接后不应再报告 active（含保持）")
	}
	now = t0.Add(700 * time.Millisecond)
	out := p.Advance(now)
	if out.Active {
		t.Fatalf("交接后不应再产生流式设定: %+v", out)
	}
	// 幂等：再调一次无效果。
	if p.ReleaseHold() {
		t.Fatal("空闲状态 ReleaseHold 应返回 false")
	}
	t.Log("输入: 活动流/排空保持/空闲 三种状态下调用 ReleaseHold; 判定: 仅排空保持时生效且幂等")
}

// TestSameTimestampReplaceRejectionAtomic 与尾点同时刻的替换点若校验失败
// （前驱->替换点段速不可行），既不能替换掉原尾点，也不能改 gap/cruise，
// 原尾点必须照常到达。
func TestSameTimestampReplaceRejectionAtomic(t *testing.T) {
	t0 := time.Date(2026, 9, 29, 0, 0, 0, 0, time.UTC)
	p := New(testCfg(8, RejectNew))
	now := t0
	// 正常排队点：0.1@0.2s。
	if err := p.Append(Point{At: t0.Add(200 * time.Millisecond), PositionRad: 0.1}, now, 0, true); err != nil {
		t.Fatal(err)
	}
	// 同时刻替换到 0.5：dt=0.2s, 速度 2.5 > 2 => 不可行，必须拒绝。
	err := p.Append(Point{At: t0.Add(200 * time.Millisecond), PositionRad: 0.5}, now, 0, true)
	mustFail(t, err, ReasonSegmentTooFast)
	if p.Len() != 1 {
		t.Fatalf("替换被拒后队列长度变化: %d", p.Len())
	}
	// 原尾点位置仍是 0.1。
	for i := 0; i < 30; i++ {
		now = now.Add(10 * time.Millisecond)
		out := p.Advance(now)
		if out.Reached {
			if out.PositionRad != 0.1 {
				t.Fatalf("原尾点被污染: 到达位置 %.4f", out.PositionRad)
			}
			t.Logf("输入: 同时刻替换到0.5(段速2.5超限) 被拒; 判定: 原尾点保留; 最终状态: 到达 0.1")
			return
		}
	}
	t.Fatal("原排队尾点未到达（可能已被失败的替换污染）")
}
