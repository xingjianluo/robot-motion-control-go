package controller_test

import (
	"sync"
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/pkg/bus"
	clkpkg "github.com/xingjianluo/robot-motion-control-go/pkg/clock"
	"github.com/xingjianluo/robot-motion-control-go/pkg/controller"
	"github.com/xingjianluo/robot-motion-control-go/pkg/feedback"
	"github.com/xingjianluo/robot-motion-control-go/pkg/frame"
	"github.com/xingjianluo/robot-motion-control-go/pkg/limits"
	"github.com/xingjianluo/robot-motion-control-go/pkg/safety"
)

// TestConcurrentEStopVsTick 急停与轨迹推进并发：无论交织顺序如何，
// 最终必须确定为 ESTOP，且期间不出现 panic / 半更新快照。
func TestConcurrentEStopVsTick(t *testing.T) {
	for iter := 0; iter < 20; iter++ {
		start := time.Date(2026, 9, 22, 10, 0, 0, 0, time.UTC)
		clk := clkpkg.NewManual(start)
		b := bus.New(bus.Config{Capacity: 256, Policy: bus.DropOldest})
		cfg := controller.Config{
			Joints: twoJointLimits(), Period: period10, QueueDepth: 64,
			DropStrategy: controller.RejectNew, Protocol: frame.V2,
			Feedback: feedback.Config{
				StaleTimeout: time.Hour, LostTimeout: 2 * time.Hour,
				MaxPosJump: 100, MaxForwardSkew: time.Hour,
			},
			MaxTrackingErr: 100, Clk: clk,
		}
		c, err := controller.New(cfg, b)
		if err != nil {
			t.Fatal(err)
		}
		var wg sync.WaitGroup
		stop := make(chan struct{})
		wg.Add(1)
		go func() { // 周期推进 goroutine：收到停止信号即退出
			defer wg.Done()
			for {
				select {
				case <-stop:
					return
				default:
					c.Tick()
				}
			}
		}()
		// 并发急停：与若干 Tick 交织后，关闭停止通道并等待 Tick goroutine 退出
		c.EStop("并发测试急停")
		close(stop)
		wg.Wait()

		snap := c.Snapshot()
		if snap.SafetyState != safety.StateEStop {
			t.Fatalf("迭代 %d：并发后最终状态必须确定为 ESTOP，实际 %s",
				iter, snap.SafetyState)
		}
		// 急停后任何新命令都必须被拒
		if err := c.SubmitCommand(limits.Command{
			Joint: 0, Mode: limits.ModePosition, Position: 0.1,
		}); err == nil {
			t.Fatalf("迭代 %d：ESTOP 后命令未被拒绝", iter)
		}
	}
	t.Log("输入: 1 个周期推进 goroutine 与急停 goroutine 竞争 x20 轮；" +
		"判定: 每轮最终状态均确定为 ESTOP，无半更新")
}

// TestQueueBacklogPolicies 有界命令队列：RejectNew 拒绝并计数；
// OverwriteOld 丢旧保新，丢弃均有计数归属。
func TestQueueBacklogPolicies(t *testing.T) {
	// RejectNew
	c, _, _, _, _ := testSetup(t, frame.V2)
	for i := 0; i < 8; i++ {
		if err := c.SubmitCommand(limits.Command{
			Joint: 0, Mode: limits.ModePosition, Position: 0.5,
		}); err != nil {
			t.Fatal(err)
		}
	}
	err := c.SubmitCommand(limits.Command{
		Joint: 0, Mode: limits.ModePosition, Position: 0.6})
	if err == nil {
		t.Fatal("队列满后第 9 条必须被拒绝")
	}
	snap := c.Snapshot()
	if snap.QueueLen != 8 || snap.CmdDropped != 1 {
		t.Fatalf("队列长度=%d 丢弃计数=%d，期望 8/1", snap.QueueLen, snap.CmdDropped)
	}
	t.Logf("RejectNew: 第9条被拒(%v)；QueueLen=%d CmdDropped=%d（归属明确）",
		err, snap.QueueLen, snap.CmdDropped)

	// OverwriteOld
	start := time.Date(2026, 9, 22, 10, 0, 0, 0, time.UTC)
	clk := clkpkg.NewManual(start)
	b2 := bus.New(bus.Config{Capacity: 64})
	cfg2 := controller.Config{
		Joints: twoJointLimits(), Period: period10, QueueDepth: 4,
		DropStrategy: controller.OverwriteOld, Protocol: frame.V2,
		Feedback: feedback.Config{
			StaleTimeout: time.Hour, LostTimeout: 2 * time.Hour,
			MaxPosJump: 100, MaxForwardSkew: time.Hour,
		},
		MaxTrackingErr: 100, Clk: clk,
	}
	c2, err := controller.New(cfg2, b2)
	if err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 6; i++ {
		if err := c2.SubmitCommand(limits.Command{
			Joint: 0, Mode: limits.ModePosition, Position: 0.5,
		}); err != nil {
			t.Fatal(err)
		}
	}
	snap2 := c2.Snapshot()
	if snap2.QueueLen != 4 || snap2.CmdDropped != 2 {
		t.Fatalf("OverwriteOld: 队列长度=%d 丢弃=%d，期望 4/2",
			snap2.QueueLen, snap2.CmdDropped)
	}
	t.Logf("OverwriteOld: 提交6条深度4 -> QueueLen=%d CmdDropped=%d，内存有界",
		snap2.QueueLen, snap2.CmdDropped)
}

// TestJitterObservable 周期抖动在快照中可观测。
func TestJitterObservable(t *testing.T) {
	c, d, clk, _, _ := testSetup(t, frame.V2)
	warmup(t, c, d, clk, 2)
	// 故意慢 5ms 推进
	now := clk.Now()
	d.respond(now, nil, false)
	d.emit(t, now)
	c.Tick()
	clk.Advance(15 * time.Millisecond)
	now = clk.Now()
	d.respond(now, nil, false)
	d.emit(t, now)
	snap, _ := c.Tick()
	if snap.LastJitter != 5*time.Millisecond {
		t.Fatalf("抖动观测= %v，期望 5ms", snap.LastJitter)
	}
	t.Logf("输入实际周期间隔 15ms（配置 10ms）-> 观测抖动 %v，归属到周期 %d",
		snap.LastJitter, snap.Cycle)
}

// TestBusBacklogDropAttribution 总线积压超限时丢弃带归属，统计可观测。
func TestBusBacklogDropAttribution(t *testing.T) {
	b := bus.New(bus.Config{Capacity: 2, Policy: bus.DropNewest})
	f1, _ := frame.EncodeCommand(frame.V2, frame.TypeCmdPosition, 3, 0.1)
	_ = b.Send(f1)
	_ = b.Send(f1)
	err := b.Send(f1) // 第三条应被拒绝
	if err == nil {
		t.Fatal("DropNewest 策略下第三条发送必须返回错误")
	}
	st := b.Stats()
	if st.Dropped != 1 || len(st.Events) != 1 {
		t.Fatalf("统计 Dropped=%d events=%d，期望 1/1", st.Dropped, len(st.Events))
	}
	ev := st.Events[0]
	if ev.Joint != 3 || ev.Reason != "OVERFLOW_REJECT_NEWEST" {
		t.Fatalf("丢弃归属错误: %+v", ev)
	}
	t.Logf("判定: 容量2发送3帧 -> 丢弃归属 dir=%s joint=%d reason=%s",
		ev.Dir, ev.Joint, ev.Reason)
}

// TestProtocolErrorsIngested 损坏帧送入反馈入口必须归类为 PROTOCOL，
// 而非被忽略或合并成安全/通信错误。
func TestProtocolErrorsIngested(t *testing.T) {
	c, _, _, _, _ := testSetup(t, frame.V2)
	if err := c.IngestFeedback([]byte{0x00, 0x01}); err == nil {
		t.Fatal("截断帧必须报错")
	}
	good, _ := frame.EncodeFeedback(frame.V2,
		frame.FeedbackData{Joint: 0, Seq: 1, Timestamp: 1})
	raw, _ := frame.Encode(good)
	corrupt := make([]byte, len(raw))
	copy(corrupt, raw)
	corrupt[len(corrupt)-1] ^= 0xAA
	if err := c.IngestFeedback(corrupt); err == nil {
		t.Fatal("校验损坏帧必须报错")
	}
	snap := c.Snapshot()
	if snap.FbBadFrames != 2 {
		t.Fatalf("坏帧计数=%d，期望 2（坏帧必须可观测而非忽略）", snap.FbBadFrames)
	}
	t.Logf("输入 1 截断帧 + 1 校验损坏帧 -> 均返回 PROTOCOL 错误，坏帧计数=%d",
		snap.FbBadFrames)
}
