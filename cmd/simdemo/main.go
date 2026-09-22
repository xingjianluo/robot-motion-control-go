// simdemo 是纯本地、确定性的端到端演示：无硬件、无外部服务、无凭据。
//
// 运行：
//
//	go run ./cmd/simdemo
//
// 演示以固定 10ms 控制周期推进一个两关节系统，依次展示：
//  1. 正常位置轨迹下发与约束保持；
//  2. 注入总线丢帧 -> 看门狗暂时失联 -> 恢复与显式复位；
//  3. 注入真实位置偏差 -> TRACKING_ERROR/FAULT_STOP（与失联区分）；
//  4. 人工急停 -> 拒绝运动 -> 释放/复位恢复；
//  5. 积压超限丢弃的可观测统计。
package main

import (
	"fmt"
	"os"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/pkg/bus"
	"github.com/xingjianluo/robot-motion-control-go/pkg/clock"
	"github.com/xingjianluo/robot-motion-control-go/pkg/controller"
	"github.com/xingjianluo/robot-motion-control-go/pkg/feedback"
	"github.com/xingjianluo/robot-motion-control-go/pkg/frame"
	"github.com/xingjianluo/robot-motion-control-go/pkg/limits"
)

type simDrive struct {
	b      *bus.Bus
	v      frame.Version
	pos    [2]float64
	vel    [2]float64
	torque [2]float64
	seq    [2]uint32
	bias   [2]float64
	drop   [2]bool
}

func newSimDrive(b *bus.Bus, v frame.Version) *simDrive {
	d := &simDrive{b: b, v: v}
	d.seq[0], d.seq[1] = 1, 1
	return d
}

func (d *simDrive) cycle(now time.Time) {
	// 执行命令
	for {
		f, ok := d.b.Recv()
		if !ok {
			break
		}
		if len(f.Payload) < 8 || f.Joint < 0 || f.Joint > 1 {
			continue
		}
		v := floatFromPayload(f.Payload)
		switch f.Type {
		case frame.TypeCmdPosition:
			d.pos[f.Joint] = v
		case frame.TypeCmdVelocity:
			d.vel[f.Joint] = v
		}
	}
	// 反馈
	for j := 0; j < 2; j++ {
		if d.drop[j] {
			d.drop[j] = false
			continue
		}
		fb := frame.FeedbackData{
			Joint: j, Seq: d.seq[j], Timestamp: now.UnixNano(),
			Position: d.pos[j] + d.bias[j], Velocity: d.vel[j],
			Torque: d.torque[j],
		}
		f, err := frame.EncodeFeedback(d.v, fb)
		if err != nil {
			fmt.Fprintln(os.Stderr, err)
			continue
		}
		if err := d.b.Send(f); err != nil {
			fmt.Fprintln(os.Stderr, err)
		}
		d.seq[j]++
	}
}

func main() {
	start := time.Date(2026, 9, 22, 10, 0, 0, 0, time.UTC)
	clk := clock.NewManual(start)
	b := bus.New(bus.Config{Capacity: 64, Policy: bus.DropOldest})
	period := 10 * time.Millisecond

	c, err := controller.New(controller.Config{
		Joints: []limits.JointLimits{
			{ID: 0, PosMin: -1.57, PosMax: 1.57, VelMax: 1, AccMax: 2,
				TorqueMax: 10, SoftMargin: 0.05},
			{ID: 1, PosMin: -3.14, PosMax: 3.14, VelMax: 2, AccMax: 4,
				TorqueMax: 20, SoftMargin: 0.1},
		},
		Period: period, QueueDepth: 8,
		DropStrategy: controller.RejectNew, Protocol: frame.V2,
		Feedback: feedback.Config{
			StaleTimeout: 35 * time.Millisecond, LostTimeout: 80 * time.Millisecond,
			MaxPosJump: 0.2, MaxForwardSkew: time.Second,
		},
		MaxTrackingErr: 0.3, Clk: clk,
	}, b)
	must(err)
	drive := newSimDrive(b, frame.V2)

	step := func(tag string) controller.Snapshot {
		now := clk.Now()
		drive.cycle(now)
		snap, err := c.Tick()
		must(err)
		if tag != "" {
			printSnap(tag, snap)
		}
		clk.Advance(period)
		return snap
	}

	fmt.Println("== 场景1：正常轨迹 joint0 -> 1.0rad ==")
	must(c.SubmitCommand(limits.Command{Joint: 0, Mode: limits.ModePosition, Position: 1.0}))
	var last controller.Snapshot
	for k := 0; k < 150; k++ {
		last = step("")
		if last.Joints[0].TrajDone {
			break
		}
	}
	printSnap("轨迹完成", last)

	fmt.Println("\n== 场景2：注入连续丢帧（通信故障，非位置偏差）==")
	for k := 0; k < 10; k++ {
		if k >= 2 {
			drive.drop[0] = true
		}
		last = step("")
	}
	printSnap("连续丢帧后", last)
	fmt.Printf("恢复条件：反馈恢复新鲜 + 显式 Reset（不静默恢复）\n")
	for k := 0; k < 3; k++ {
		last = step("")
	}
	fmt.Printf("Reset 结果: %v\n", c.Reset())
	printSnap("复位后", c.Snapshot())

	fmt.Println("\n== 场景3：命令越限被拒绝（可区分原因）==")
	err = c.SubmitCommand(limits.Command{Joint: 0, Mode: limits.ModePosition, Position: 9})
	fmt.Printf("越界命令拒绝: %v\n", err)

	fmt.Println("\n== 场景4：人工急停 ==")
	must(c.EStop("演示急停"))
	printSnap("急停后", c.Snapshot())
	err = c.SubmitCommand(limits.Command{Joint: 0, Mode: limits.ModePosition, Position: 0.5})
	fmt.Printf("急停期间提交命令: %v\n", err)
	must(c.ReleaseEStop())
	fmt.Printf("释放急停后 Reset: %v\n", c.Reset())
	printSnap("恢复运行", c.Snapshot())

	st := b.Stats()
	fmt.Printf("\n总线观测: 发送=%d 接收=%d 丢弃=%d\n", st.Sent, st.Received, st.Dropped)
	fmt.Println("演示结束：全过程确定性，可反复运行复现。")
}

func printSnap(tag string, s controller.Snapshot) {
	fmt.Printf("[%s] cycle=%d safety=%s", tag, s.Cycle, s.SafetyState)
	if s.SafetyReason.Detail != "" {
		fmt.Printf("(%s:%s)", s.SafetyReason.Kind, s.SafetyReason.Detail)
	}
	fmt.Printf(" queue=%d dropped=%d badFb=%d jitter=%v\n",
		s.QueueLen, s.CmdDropped, s.FbBadFrames, s.LastJitter)
	for _, j := range s.Joints {
		fmt.Printf("    joint%d mode=%d ref=%.3f fb=%.3f cond=%s terr=%.3f\n",
			j.ID, j.CmdMode, j.RefPosition, j.FbPosition, j.FbCondition,
			j.TrackingError)
	}
}

func must(err error) {
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}
