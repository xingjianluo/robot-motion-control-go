// tick.go 实现控制周期 Tick 与状态快照：所有步骤在同一把锁内原子完成。
package controller

import (
	"math"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/pkg/feedback"
	"github.com/xingjianluo/robot-motion-control-go/pkg/frame"
	"github.com/xingjianluo/robot-motion-control-go/pkg/limits"
	"github.com/xingjianluo/robot-motion-control-go/pkg/safety"
	"github.com/xingjianluo/robot-motion-control-go/pkg/trajectory"
)

// Tick 推进一个控制周期。周期内严格按以下顺序执行且全程持锁：
//  1. 排空总线：反馈帧交给监控器，非反馈帧归类计数；
//  2. 各反馈监控器按年龄 Tick，区分 STALE（暂时失联）/ LOST（丢失）；
//  3. 汇总安全事件（看门狗、软限位、力矩超限、驱动器故障、真实跟踪偏差），
//     由安全机按优先级裁决；
//  4. 允许运动时：每关节取一条排队命令并推进轨迹/保持直驱，下发总线；
//     不允许时：停止一切下发，排队命令保留（恢复需显式 Reset，可解释）；
//  5. 生成整机快照（值拷贝），记录周期时刻用于抖动可观测。
func (c *Controller) Tick() (Snapshot, error) {
	c.mu.Lock()
	defer c.mu.Unlock()
	now := c.cfg.Clk.Now()
	c.cycle++

	// ---- 1. 排空总线积压（防止无界积压：每周期尽力排空，总线自身也有界）----
	drainBad := 0
	for {
		f, ok := c.bus.Recv()
		if !ok {
			break
		}
		if f.Type != frame.TypeFeedback {
			drainBad++
			continue
		}
		fb, derr := frame.DecodeFeedback(f)
		if derr != nil {
			c.fbBad++
			continue
		}
		if fb.Joint < 0 || fb.Joint >= len(c.joints) {
			c.fbBad++
			continue
		}
		j := c.joints[fb.Joint]
		j.fb = j.mon.Observe(fb, now)
		if j.fb.Cond != feedback.CondOK {
			c.fbBad++
		}
	}
	if drainBad > 0 {
		c.commBad += uint64(drainBad)
	}

	// ---- 2/3. 反馈时间判定 + 安全事件汇总 ----
	events := make([]safety.Event, 0, len(c.joints)*3)
	for _, j := range c.joints {
		j.fb = j.mon.Tick(now)
		switch j.fb.Cond {
		case feedback.CondStale:
			events = append(events, safety.Event{
				Kind: safety.KindWatchdog, Active: true,
				Detail: "joint " + itoa(j.lim.ID) + " 暂时失联: " + j.fb.Reason})
		case feedback.CondLost:
			events = append(events, safety.Event{
				Kind: safety.KindWatchdog, Active: true,
				Detail: "joint " + itoa(j.lim.ID) + " 反馈丢失: " + j.fb.Reason})
		case feedback.CondOutOfOrder, feedback.CondDuplicate,
			feedback.CondTimestampJump, feedback.CondPositionJump:
			// 异常帧不计为跟踪误差，但通信看门狗判定“反馈链路不可信”
			events = append(events, safety.Event{
				Kind: safety.KindWatchdog, Active: true,
				Detail: "joint " + itoa(j.lim.ID) + " 反馈链路异常: " + j.fb.Reason})
		default:
			events = append(events, safety.Event{
				Kind: safety.KindWatchdog, Active: false,
				Detail: "joint " + itoa(j.lim.ID) + " 反馈正常"})
		}

		if j.fb.HasData && j.fb.Cond == feedback.CondOK {
			// 软限位（依据“真实位置”，且仅在允许反馈时）
			if j.fb.LastPos < j.lim.SoftMin() || j.fb.LastPos > j.lim.SoftMax() {
				events = append(events, safety.Event{
					Kind: safety.KindSoftLimit, Active: true,
					Detail: "joint " + itoa(j.lim.ID) + " 实际位置 " +
						ftoa(j.fb.LastPos) + " 越过软限位 [" + ftoa(j.lim.SoftMin()) +
						", " + ftoa(j.lim.SoftMax()) + "]"})
			} else {
				events = append(events, safety.Event{Kind: safety.KindSoftLimit, Active: false})
			}
			// 力矩超限（V1 反馈无力矩字段，LastTorque 恒为 0）
			if math.Abs(j.fb.LastTorque) > j.lim.TorqueMax {
				events = append(events, safety.Event{
					Kind: safety.KindTorque, Active: true,
					Detail: "joint " + itoa(j.lim.ID) + " 实测力矩 " +
						ftoa(j.fb.LastTorque) + " 超过限值 " + ftoa(j.lim.TorqueMax)})
			} else {
				events = append(events, safety.Event{Kind: safety.KindTorque, Active: false})
			}
			// 真实跟踪偏差：参考与实测之差；仅在反馈 OK 时统计，
			// 因而与“失联”严格区分
			if c.cfg.MaxTrackingErr > 0 && j.mode != limits.ModeInvalid {
				if terr := math.Abs(j.refPos - j.fb.LastPos); terr > c.cfg.MaxTrackingErr {
					events = append(events, safety.Event{
						Kind: safety.KindTracking, Active: true,
						Detail: "joint " + itoa(j.lim.ID) + " 跟踪误差 " +
							ftoa(terr) + " 超过阈值 " + ftoa(c.cfg.MaxTrackingErr)})
				} else {
					events = append(events, safety.Event{Kind: safety.KindTracking, Active: false})
				}
			}
		}
	}
	c.safety.Evaluate(events, now)

	// ---- 4. 允许运动才推进与下发 ----
	if c.safety.AllowsMotion() {
		for _, j := range c.joints {
			c.advanceJoint(j, now)
		}
	}
	// 非 OPERATIONAL 时不下发任何帧（安全优先于普通命令）

	// ---- 5. 周期记录与快照 ----
	c.tickTimes = append(c.tickTimes, now)
	if len(c.tickTimes) > 64 {
		c.tickTimes = c.tickTimes[len(c.tickTimes)-64:]
	}
	return c.snapshotLocked(now), nil
}

// advanceJoint 取出一条排队命令并推进，或将轨迹/直驱命令下发总线。
func (c *Controller) advanceJoint(j *jointCtx, now time.Time) {
	q := c.queue[j.lim.ID]
	if len(q) > 0 {
		next := q[0]
		c.queue[j.lim.ID] = q[1:]
		// 以当前参考位置启动；反馈位置仅用于监测跟踪误差。
		// 若直接以反馈值重设参考，反而会在控制器侧制造参考跳变，
		// 真实偏差应通过跟踪误差安全事件体现。
		startPos := j.refPos
		switch next.cmd.Mode {
		case limits.ModePosition:
			tr, err := trajectory.Plan(startPos, 0, next.cmd.Position,
				c.cfg.Period.Seconds(), j.lim)
			if err != nil {
				// 规划失败（理论上提交时已校验）：不得静默，记入命令丢弃计数
				c.cmdDropped++
				return
			}
			j.traj = tr
			j.mode = limits.ModePosition
		case limits.ModeVelocity, limits.ModeTorque:
			j.traj = nil
			j.mode = next.cmd.Mode
			j.hold = next.cmd
		}
	}

	switch j.mode {
	case limits.ModePosition:
		if j.traj == nil {
			return
		}
		cmd, pt, err := j.traj.Sample()
		if err != nil {
			c.cmdDropped++
			return
		}
		j.refPos = pt.Position
		j.trajDone = pt.Done
		c.sendCmd(j, cmd)
	case limits.ModeVelocity:
		j.refPos = j.fb.LastPos
		j.trajDone = false
		c.sendCmd(j, j.hold)
	case limits.ModeTorque:
		j.refPos = j.fb.LastPos
		j.trajDone = false
		c.sendCmd(j, j.hold)
	}
}

func (c *Controller) sendCmd(j *jointCtx, cmd limits.Command) {
	var typ frame.Type
	var value float64
	switch cmd.Mode {
	case limits.ModePosition:
		typ, value = frame.TypeCmdPosition, cmd.Position
	case limits.ModeVelocity:
		typ, value = frame.TypeCmdVelocity, cmd.Velocity
	case limits.ModeTorque:
		typ, value = frame.TypeCmdTorque, cmd.Torque
	default:
		return
	}
	f, err := frame.EncodeCommand(c.cfg.Protocol, typ, j.lim.ID, value)
	if err != nil {
		// 例如 V1 力矩：协议不兼容必须可见
		c.commBad++
		return
	}
	if err := c.bus.Send(f); err != nil {
		// 总线积压拒绝（DropNewest）：计入命令丢弃，归属可经总线 Stats 查询
		c.cmdDropped++
	}
}

// Snapshot 原子读取当前状态。
func (c *Controller) Snapshot() Snapshot {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.snapshotLocked(c.cfg.Clk.Now())
}

func (c *Controller) snapshotLocked(now time.Time) Snapshot {
	st, cause := c.safety.Snapshot()
	snap := Snapshot{
		At: now, Cycle: c.cycle,
		SafetyState: st, SafetyReason: safety.Event{
			Kind: cause.Kind, Active: cause.Active, Detail: cause.Detail,
			At: cause.At,
		},
		CmdDropped: c.cmdDropped, FbBadFrames: c.fbBad,
		CommBad: c.commBad,
	}
	queueLen := 0
	for _, j := range c.joints {
		queueLen += len(c.queue[j.lim.ID])
		js := JointState{
			ID: j.lim.ID, CmdMode: j.mode,
			RefPosition: j.refPos,
			FbPosition:  j.fb.LastPos,
			FbVelocity:  j.fb.LastVel,
			FbTorque:    j.fb.LastTorque,
			FbCondition: j.fb.Cond,
			FbReason:    j.fb.Reason,
			TrajDone:    j.trajDone,
		}
		if j.fb.HasData && j.fb.Cond == feedback.CondOK && j.mode != limits.ModeInvalid {
			js.TrackingError = math.Abs(j.refPos - j.fb.LastPos)
		}
		snap.Joints = append(snap.Joints, js)
	}
	snap.QueueLen = queueLen
	if n := len(c.tickTimes); n >= 2 {
		// 最近一个周期间隔相对配置周期的抖动
		got := c.tickTimes[n-1].Sub(c.tickTimes[n-2])
		d := got - c.cfg.Period
		if d < 0 {
			d = -d
		}
		snap.LastJitter = d
		snap.LastPeriod = got
	}
	return snap
}
