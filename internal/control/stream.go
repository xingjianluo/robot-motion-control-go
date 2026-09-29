package control

import (
	"fmt"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/bus"
	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
	"github.com/xingjianluo/robot-motion-control-go/internal/safety"
	"github.com/xingjianluo/robot-motion-control-go/internal/stream"
)

// 流式喂点 / 在线倍率相关原因码。
const (
	// ReasonLegacyTrajectoryBusy 流式目标在旧版单目标轨迹尚未排空时被拒。
	ReasonLegacyTrajectoryBusy = "legacy_trajectory_busy"
	// ReasonStreamActive 旧版单目标命令在流式段尚未走完时被拒。
	ReasonStreamActive = "stream_active"
	// ReasonStreamModeMismatch 追加目标的模式与当前在跑/排队流模式不一致；
	// 跨模式拼接要求先回到空闲，避免不确定的混合轮廓。
	ReasonStreamModeMismatch = "stream_mode_mismatch"
	// ReasonStreamPurged 安全动作（周期内安全门控/急停）清空了尚未走完的
	// 流式段。属安全类别：这是安全优先级的确定性结果，可观测、非静默丢点。
	ReasonStreamPurged = "stream_motion_purged"
	// ReasonLegacyTrajectoryPurged 安全动作清空了旧版单目标轨迹缓冲。
	ReasonLegacyTrajectoryPurged = "legacy_trajectory_purged"
	// ReasonRecoveryEvidenceContradicted 恢复请求声称的条件与系统当时掌握的
	// 真实反馈证据相矛盾（恢复与否按真实状态判定，不信口头条件）。
	ReasonRecoveryEvidenceContradicted = "recovery_evidence_contradicted"
)

// StreamInfo 是流式喂点与在线倍率在周期边界上的可观测状态（值语义）。
type StreamInfo struct {
	stream.Status
}

// refPositionLocked 返回当前可用的参考位置：优先最近一条被接受反馈样本
// （系统当时的真实状态），没有时退回最近下发的位置目标；都没有则
// has=false，位置段的可行性核算推迟到激活瞬间。
func (c *Controller) refPositionLocked() (float64, bool) {
	if sample, ok := c.consumer.Last(); ok {
		return sample.PositionRad, true
	}
	if c.hasCmd && c.lastCmd.Mode == protocol.ModePosition {
		return c.lastCmd.PositionRad, true
	}
	return 0, false
}

// AppendStreamTarget 追加一个流式目标（手轮/示教式连续喂点）。
//
// 与旧版 SubmitCommand 的兼容规则：两者是互斥的单条运动通道——
//   - 旧版单目标轨迹缓冲里还有未排空航点时，流式目标以
//     command_rejected|controller|legacy_trajectory_busy 拒绝；
//   - 流式段（在跑或排队）尚未走完时，旧版命令以
//     command_rejected|controller|stream_active 拒绝。
//
// 这样原有单目标用法、协议 V1/V2、既有拒绝顺序全部保持不变；新能力是
// 纯增量。任何拒绝都返回带归属错误，绝不静默丢点。
//
// 安全规则与 SubmitCommand 相同：非 RUN 状态一律安全优先拒绝
// (safety|controller|outputs_disabled)，不进入任何流合法性核算。
func (c *Controller) AppendStreamTarget(t stream.Target) (evicted []stream.Eviction, err error) {
	c.mu.Lock()
	defer c.mu.Unlock()
	now := c.clk.Now()

	if state := c.machine.State(); state != safety.StateRun {
		err := fault.New(fault.CatSafety, fault.SourceController, ReasonOutputsDisabled,
			"stream target rejected: outputs disabled in safety state "+string(state))
		c.recordFaultLocked(now, err, "stream target not accepted in "+string(state))
		return nil, err
	}
	if c.traj.Len() > 0 {
		err := fault.New(fault.CatCommandReject, fault.SourceController, ReasonLegacyTrajectoryBusy,
			"legacy single-target trajectory still pending; drain it before streaming")
		c.recordFaultLocked(now, err, "")
		return nil, err
	}
	if m := c.player.Mode(); m != "" && m != t.Mode {
		err := fault.New(fault.CatCommandReject, fault.SourceController, ReasonStreamModeMismatch,
			fmt.Sprintf("stream mode %s cannot splice into active/queued mode %s; return to idle first", t.Mode, m))
		c.recordFaultLocked(now, err, "")
		return nil, err
	}

	// 协议版本入口预检（与 SubmitCommand 一致）：例如 V1 上的流式力矩
	// 目标必须在入口以 protocol_error|version_unsupported 拒绝。
	probe := protocol.MotorCommand{Mode: t.Mode}
	if _, perr := protocol.EncodeCommand(c.cfg.ProtocolVersion, probe); perr != nil {
		c.recordFaultLocked(now, perr, "stream target incompatible with configured protocol version")
		return nil, perr
	}

	refPos, hasRef := c.refPositionLocked()
	ev, aerr := c.player.Append(t, refPos, hasRef, now)
	for _, e := range ev {
		fe := fault.New(fault.CatCommandReject, fault.SourceController, "stream_waiting_evicted",
			fmt.Sprintf("waiting segment seq=%d (%s %.4f) evicted by overflow policy at %s",
				e.SourceSeq, e.Mode, e.Value, e.At.Format("15:04:05.000")))
		c.recordFaultLocked(now, fe, "evict_oldest made room for a newer stream target")
	}
	if aerr != nil {
		c.recordFaultLocked(now, aerr, "stream target rejected by player feasibility checks")
		return ev, aerr
	}
	return ev, nil
}

// SetSpeedOverride 在线设置整体速度倍率（1=100%，如 0.5 / 1.2）。
//
// 可在任意安全状态下调用（停机期间操作员也可预调倍率），但：
//   - 倍率超出配置范围 => command_rejected|stream|stream_override_out_of_range；
//   - 若新倍率会把“正在执行的段”或任一排队段顶出速度/插值步长/拼接
//     加速度边界 => 整体拒绝，播放器状态与倍率保持不变（原子生效）。
//
// 倍率的生效点固定在两个控制周期之间：Step 在同一临界区内读倍率、
// 推进播放器、编码并投递，因此一个周期内绝不可能半周期旧倍率、
// 半周期新倍率。
func (c *Controller) SetSpeedOverride(scale float64) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	now := c.clk.Now()
	if err := c.player.SetOverride(scale, now); err != nil {
		c.recordFaultLocked(now, err, "speed override rejected")
		return err
	}
	return nil
}

// SpeedOverride 返回当前速度倍率（快照读，加控制器锁）。
func (c *Controller) SpeedOverride() float64 {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.player.Override()
}

// StreamStatus 返回流式播放器可观测状态。
func (c *Controller) StreamStatus() stream.Status {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.player.Status()
}

// advanceStreamLocked 在输出使能时推进一个周期的流式轨迹。
// 播放器在一个周期内只按“当前这一个倍率”产出一个目标，编码/投递同样
// 发生在本临界区，保证倍率切换对单个周期干净。
func (c *Controller) advanceStreamLocked(now time.Time) {
	refPos, hasRef := c.refPositionLocked()
	d, ok := c.player.Step(now, refPos, hasRef)
	for _, dropped := range c.player.TakeDropped() {
		err := fault.New(fault.CatCommandReject, fault.SourceStream, dropped.Reason,
			fmt.Sprintf("stream segment seq=%d dropped at activation: %s", dropped.SourceSeq, dropped.Detail))
		c.recordFaultLocked(now, err, "infeasible segment dropped by fixed rule, not silently lost")
	}
	if !ok {
		return
	}
	cmd := protocol.MotorCommand{
		Mode: d.Mode, PositionRad: d.PositionRad, VelocityRadS: d.VelocityRadS,
		TorqueNm: d.TorqueNm, Seq: d.Seq, IssuedAt: now,
	}
	line, err := protocol.EncodeCommand(c.cfg.ProtocolVersion, cmd)
	if err != nil {
		c.recordFaultLocked(now, err, "stream command encoding failed")
		return
	}
	if perr := c.bus.Post(bus.Downlink, line, cmd.Seq); perr != nil {
		c.recordFaultLocked(now, perr, "stream downlink post failed")
		return
	}
	c.lastCmd = cmd
	c.hasCmd = true
}
