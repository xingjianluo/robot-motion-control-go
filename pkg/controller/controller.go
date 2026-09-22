// Package controller 编排命令下发、反馈接收、安全联锁与状态上报。
//
// 并发模型：
//   - 所有共享状态（轨迹、反馈、安全机、命令队列、计数）均由单把 mu 保护，
//     一个控制周期 Tick 在锁内原子完成“反馈消费 -> 安全判定 -> 轨迹推进 ->
//     命令下发”，因此不会出现半更新或相互覆盖；
//   - 急停 EStop 与 Tick 并发时，二者串行，急停始终在获取锁的那一刻生效，
//     周期结束状态唯一确定（若 Tick 先拿锁，该周期正常推进；若 EStop 先拿锁，
//     该周期停止下发），不存在交叉状态；
//   - 命令队列为有界深度，积压超限按 RejectNew/OverwriteOld 策略处理，
//     每次拒绝/丢弃均计数并带关节与原因归属。
package controller

import (
	"sync"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/pkg/bus"
	"github.com/xingjianluo/robot-motion-control-go/pkg/clock"
	"github.com/xingjianluo/robot-motion-control-go/pkg/errors"
	"github.com/xingjianluo/robot-motion-control-go/pkg/feedback"
	"github.com/xingjianluo/robot-motion-control-go/pkg/frame"
	"github.com/xingjianluo/robot-motion-control-go/pkg/limits"
	"github.com/xingjianluo/robot-motion-control-go/pkg/safety"
	"github.com/xingjianluo/robot-motion-control-go/pkg/trajectory"
)

// DropStrategy 命令积压策略。
type DropStrategy int

const (
	RejectNew    DropStrategy = iota // 队列满则拒绝新命令（返回 COMMAND 错误）
	OverwriteOld                     // 丢弃该关节最旧的未执行命令
)

// Config 控制器配置。
type Config struct {
	Joints       []limits.JointLimits
	Period       time.Duration
	QueueDepth   int
	DropStrategy DropStrategy
	Protocol     frame.Version // 下发命令使用的协议版本（V1 关节禁止力矩命令）
	Feedback     feedback.Config
	// MaxTrackingErr 反馈有效时允许的跟踪误差（rad）；超限按 FAULT_STOP 处理
	MaxTrackingErr float64
	Clk            clock.Clock
}

type queuedCmd struct {
	cmd limits.Command
	at  time.Time
}

type jointCtx struct {
	lim      limits.JointLimits
	mon      *feedback.Monitor
	traj     *trajectory.Trajectory
	hold     limits.Command // 当前保持的直驱命令（速度/力矩模式）或最新位置命令
	mode     limits.Mode
	refPos   float64
	fb       feedback.State
	trajDone bool
}

// Controller 多关节控制器。
type Controller struct {
	cfg Config
	bus *bus.Bus

	mu         sync.Mutex
	joints     []*jointCtx
	safety     *safety.Monitor
	queue      map[int][]queuedCmd // 每关节独立有界队列
	cycle      uint64
	cmdDropped uint64
	fbBad      uint64
	commBad    uint64
	tickTimes  []time.Time // 最近周期时刻（用于抖动统计，有界）
}

// New 创建控制器并校验配置。
func New(cfg Config, b *bus.Bus) (*Controller, error) {
	if len(cfg.Joints) == 0 {
		return nil, errors.New(errors.CatCommand, "NO_JOINTS", "至少需要一个关节")
	}
	if cfg.Period <= 0 {
		return nil, errors.New(errors.CatCommand, "BAD_PERIOD", "控制周期必须为正数")
	}
	if cfg.QueueDepth <= 0 {
		cfg.QueueDepth = 8
	}
	if cfg.Protocol == 0 {
		cfg.Protocol = frame.V2
	}
	if cfg.Clk == nil {
		cfg.Clk = clock.Real{}
	}
	c := &Controller{
		cfg: cfg, bus: b,
		safety: safety.New(),
		queue:  map[int][]queuedCmd{},
	}
	ok := false
	for _, v := range frame.SupportedVersions {
		if v == cfg.Protocol {
			ok = true
		}
	}
	if !ok {
		return nil, errors.New(errors.CatProtocol, "UNSUPPORTED_VERSION",
			"控制器协议版本不受支持")
	}
	for _, jl := range cfg.Joints {
		c.joints = append(c.joints, &jointCtx{
			lim:  jl,
			mon:  feedback.NewMonitor(jl.ID, cfg.Feedback),
			mode: limits.ModeInvalid,
		})
	}
	return c, nil
}

func (c *Controller) joint(id int) (*jointCtx, error) {
	if id < 0 || id >= len(c.joints) {
		return nil, errors.New(errors.CatCommand, "UNKNOWN_JOINT",
			"未知关节编号")
	}
	return c.joints[id], nil
}

// SubmitCommand 提交命令。以下情况均返回带分类的错误且不改变系统状态：
// 安全状态不允许运动（SAFETY）、V1 发力矩（PROTOCOL）、
// 限位拒绝（COMMAND）、队列满且策略为 RejectNew（COMMAND: QUEUE_FULL）。
func (c *Controller) SubmitCommand(cmd limits.Command) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	if !c.safety.AllowsMotion() {
		st, cause := c.safety.Snapshot()
		return errors.New(errors.CatSafety, "MOTION_NOT_ALLOWED",
			"安全状态 "+string(st)+"（"+string(cause.Kind)+"）下拒绝运动命令")
	}
	j, err := c.joint(cmd.Joint)
	if err != nil {
		return err
	}
	if cmd.Mode == limits.ModeTorque && c.cfg.Protocol == frame.V1 {
		return errors.New(errors.CatProtocol, "CMD_UNSUPPORTED_VERSION",
			"力矩命令需要 V2，当前控制器按 V1 下发")
	}
	if e := j.lim.Validate(cmd); e != nil {
		return e
	}

	q := c.queue[cmd.Joint]
	if len(q) >= c.cfg.QueueDepth {
		switch c.cfg.DropStrategy {
		case OverwriteOld:
			c.queue[cmd.Joint] = append(q[1:], queuedCmd{cmd: cmd, at: c.cfg.Clk.Now()})
			c.cmdDropped++
			// 覆盖旧命令是既定策略，提交仍成功，丢弃可经 Snapshot 观测
			return nil
		default:
			c.cmdDropped++
			return errors.New(errors.CatCommand, "QUEUE_FULL",
				"关节命令队列已满（深度 "+itoa(c.cfg.QueueDepth)+"），按 RejectNew 策略拒绝")
		}
	}
	c.queue[cmd.Joint] = append(q, queuedCmd{cmd: cmd, at: c.cfg.Clk.Now()})
	return nil
}

// EStop 急停（并发安全，立即锁内生效）。
func (c *Controller) EStop(reason string) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.safety.EStop(c.cfg.Clk.Now(), reason)
	return nil
}

// ReleaseEStop 释放急停。
func (c *Controller) ReleaseEStop() error {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.safety.ReleaseEStop(c.cfg.Clk.Now())
}

// Reset 故障复位。
func (c *Controller) Reset() error {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.safety.Reset(c.cfg.Clk.Now())
}

// IngestFeedback 解析并消费一帧原始反馈。
// 帧协议错误（PROTOCOL）与反馈质量异常（记录但不产生跟踪误差）严格区分；
// 关节编号非法归为 PROTOCOL:UNKNOWN_JOINT。
func (c *Controller) IngestFeedback(raw []byte) error {
	f, err := frame.Decode(raw)
	if err != nil {
		c.mu.Lock()
		c.fbBad++
		c.mu.Unlock()
		return err
	}
	if f.Type != frame.TypeFeedback {
		c.mu.Lock()
		c.fbBad++
		c.mu.Unlock()
		return errors.New(errors.CatProtocol, "FRAME_TYPE_MISMATCH",
			"IngestFeedback 只接受反馈帧，收到 0x"+hex2(uint8(f.Type)))
	}
	fb, err := frame.DecodeFeedback(f)
	if err != nil {
		c.mu.Lock()
		c.fbBad++
		c.mu.Unlock()
		return err
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	if fb.Joint < 0 || fb.Joint >= len(c.joints) {
		c.fbBad++
		return errors.New(errors.CatProtocol, "UNKNOWN_JOINT",
			"反馈帧携带未知关节编号")
	}
	j := c.joints[fb.Joint]
	j.fb = j.mon.Observe(fb, c.cfg.Clk.Now())
	if j.fb.Cond != feedback.CondOK {
		c.fbBad++
	}
	return nil
}
