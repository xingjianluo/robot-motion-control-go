// Package control 把轨迹校验/缓冲、反馈消费、安全联锁与模拟总线
// 组合成单关节运动控制器。
//
// 并发正确性：
//   - SubmitCommand / Estop / Recover / Step / Snapshot 全部走同一把锁，
//     每个方法都在一个临界区里完成全部状态读写与总线投递，不存在
//     半更新与相互覆盖；
//   - 急停与周期推进并发时：拿到锁的先执行，另一个进入后读到的是
//     ESTOP 状态，因此急停提交之后的任何周期都不会再下发出力命令；
//   - 锁序固定为 控制器锁 -> 总线锁 -> 状态机锁（经 Monitor），无环。
package control

import (
	"fmt"
	"sync"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/bus"
	"github.com/xingjianluo/robot-motion-control-go/internal/clock"
	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
	"github.com/xingjianluo/robot-motion-control-go/internal/feedback"
	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
	"github.com/xingjianluo/robot-motion-control-go/internal/safety"
	"github.com/xingjianluo/robot-motion-control-go/internal/stream"
	"github.com/xingjianluo/robot-motion-control-go/internal/trajectory"
)

// EventSink 接收故障记录与周期快照（由 report.Recorder 实现；
// 仅依赖最小接口以避免 control<->report 循环依赖）。
type EventSink interface {
	RecordFault(at time.Time, category, source, reason, detail string)
	RecordCycle(s Snapshot)
}

// 控制器自身产生的原因码。
const (
	// ReasonOutputsDisabled 安全状态不允许出力时拒绝命令。
	ReasonOutputsDisabled = "outputs_disabled"
	// ReasonSkippedWaypoints 周期追赶丢弃了过时航点（可观测）。
	ReasonSkippedWaypoints = "waypoints_skipped_by_cycle_catchup"
	// ReasonStreamEvicted 流式排队挤掉旧点（可观测事件，非命令拒绝）。
	ReasonStreamEvicted = "stream_evicted_oldest"
	// ReasonStreamDrained 一段流式喂点全部走完（可观测事件）。
	ReasonStreamDrained = "stream_drained"
	// ReasonRecoveryObservedMismatch 调用方声称的恢复条件与系统真实
	// 观测相矛盾（不能光信口头声明）。
	ReasonRecoveryObservedMismatch = "recovery_conditions_contradict_observation"
)

// StreamConfig 是连续流式喂点与在线倍率的控制器侧配置。
type StreamConfig struct {
	Capacity    int               // 排队深度上限（含活动点），默认 32
	FullPolicy  stream.FullPolicy // 满时策略：reject_new / evict_oldest
	MinOverride float64           // 允许的最小倍率（0 表示可暂停），默认 0
	MaxOverride float64           // 允许的最大倍率，默认 2
}

// Config 汇总控制器配置。
type Config struct {
	ProtocolVersion protocol.Version
	Period          time.Duration
	JitterTolerance time.Duration // 实际周期偏差绝对值超过此值计入 JitterBreaches；0 不检查
	MaxBacklog      int           // 单周期最多消费的上行帧数，防止单周期被积压饿死
	TrajCapacity    int
	Motion          trajectory.Limits
	SafetyLim       safety.Limits
	Feedback        feedback.Config
	Bus             bus.Config
	Stream          StreamConfig
	// RequireObservedRecovery 为 true 时，Recover 不仅检查调用方给出的
	// RecoveryConditions，还要与系统当前真实观测（最近反馈样本/失联状态）
	// 交叉核验：口头声称健康但系统观测不健康 => 拒绝恢复。
	// 默认 false 以保持对既有调用方/用例的兼容。
	RequireObservedRecovery bool
}

// Controller 是并发安全的控制器核心。
type Controller struct {
	cfg       Config
	clk       clock.Clock
	bus       *bus.Bus
	mu        sync.Mutex
	validator *trajectory.Validator
	traj      *trajectory.Buffer
	planner   *stream.Planner
	consumer  *feedback.Consumer
	machine   *safety.Machine
	monitor   *safety.Monitor
	sink      EventSink

	cycle        uint64
	lastStepAt   time.Time
	lastCmd      protocol.MotorCommand
	hasCmd       bool
	maxJitter    time.Duration
	jitterBreach int64
	lastFault    FaultRef
	lastAccepted int
	lastRejected int
	streamSeq    uint64
}

// New 创建控制器并校验配置中的协议版本。
// 初始安全状态为 ESTOP：必须显式满足条件恢复后才允许出力。
func New(cfg Config, clk clock.Clock, b *bus.Bus) (*Controller, error) {
	if !protocol.SupportedVersions[cfg.ProtocolVersion] {
		return nil, fault.New(fault.CatProtocol, fault.SourceController, protocol.ReasonUnsupportedVer,
			fmt.Sprintf("controller configured with unsupported protocol version %d", cfg.ProtocolVersion))
	}
	if cfg.MaxBacklog <= 0 {
		cfg.MaxBacklog = 16
	}
	if cfg.TrajCapacity <= 0 {
		cfg.TrajCapacity = 16
	}
	machine := safety.NewMachine(safety.StateEstop)
	if cfg.Stream.Capacity <= 0 {
		cfg.Stream.Capacity = 32
	}
	if cfg.Stream.FullPolicy == "" {
		cfg.Stream.FullPolicy = stream.RejectNew
	}
	if cfg.Stream.MaxOverride <= 0 {
		cfg.Stream.MaxOverride = 2
	}
	planner := stream.New(stream.Config{
		Capacity:        cfg.Stream.Capacity,
		FullPolicy:      cfg.Stream.FullPolicy,
		Period:          cfg.Period,
		MaxAccelRadS2:   cfg.Motion.MaxAccelRadS2,
		MaxVelocityRadS: cfg.Motion.MaxVelocityRadS,
		MinPositionRad:  cfg.Motion.MinPositionRad,
		MaxPositionRad:  cfg.Motion.MaxPositionRad,
		MinOverride:     cfg.Stream.MinOverride,
		MaxOverride:     cfg.Stream.MaxOverride,
	})
	c := &Controller{
		cfg:       cfg,
		clk:       clk,
		bus:       b,
		validator: trajectory.NewValidator(cfg.Motion),
		traj:      trajectory.NewBuffer(cfg.TrajCapacity),
		planner:   planner,
		consumer:  feedback.New(cfg.Feedback),
		machine:   machine,
		monitor:   safety.NewMonitor(machine, cfg.SafetyLim),
	}
	return c, nil
}

// SetEventSink 安装事件/快照记录器（通常在启动前调用一次）。
func (c *Controller) SetEventSink(s EventSink) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.sink = s
}

// SubmitCommand 提交一条运动命令。
//
// 判定顺序（全部在同一临界区内，拒绝原因彼此可区分）：
//  1. 安全状态不允许出力 => CatSafety/outputs_disabled，
//     安全优先级最高，绝不落入普通校验；
//  2. 轨迹连续性/限位/矛盾字段 => CatCommandReject（原因码由轨迹层给出）；
//  3. 目标软限位预检 => 触发 SOFT_LIMIT 并以 CatSafety 拒绝；
//  4. 进入有界轨迹缓冲；缓冲满 => trajectory_buffer_overflow。
//
// 命令不会被静默丢弃：任何拒绝都原样返回带归属的错误。
func (c *Controller) SubmitCommand(cmd trajectory.Command) (protocol.MotorCommand, error) {
	c.mu.Lock()
	defer c.mu.Unlock()
	now := cmd.IssuedAt
	if now.IsZero() {
		now = c.clk.Now()
		cmd.IssuedAt = now
	}

	if state := c.machine.State(); state != safety.StateRun {
		err := fault.New(fault.CatSafety, fault.SourceController, ReasonOutputsDisabled,
			"command rejected: outputs disabled in safety state "+string(state))
		c.recordFaultLocked(now, err, "command not accepted in "+string(state))
		return protocol.MotorCommand{}, err
	}

	sp, err := c.validator.Validate(cmd)
	if err != nil {
		c.recordFaultLocked(now, err, "trajectory validation rejected command")
		return protocol.MotorCommand{}, err
	}

	c.monitor.CheckSetpoint(now, sp)
	if state := c.machine.State(); state != safety.StateRun {
		err := fault.New(fault.CatSafety, fault.SourceSafety, safety.ReasonSoftLimit,
			"setpoint enters soft-limit band, command held")
		c.recordFaultLocked(now, err, "setpoint pre-check tripped soft limit")
		return protocol.MotorCommand{}, err
	}

	// 协议版本预检：例如 V1 上请求力矩模式必须在入口处即以
	// CatProtocol 拒绝，不能先入缓冲再在周期内静默失败。
	probe := protocol.MotorCommand{Mode: sp.Mode, PositionRad: sp.PositionRad,
		VelocityRadS: sp.VelocityRadS, TorqueNm: sp.TorqueNm, Seq: sp.Seq, IssuedAt: now}
	if _, perr := protocol.EncodeCommand(c.cfg.ProtocolVersion, probe); perr != nil {
		c.recordFaultLocked(now, perr, "command incompatible with configured protocol version")
		return protocol.MotorCommand{}, perr
	}

	if err := c.traj.Append(trajectory.BufferedPoint{Setpoint: sp, At: now}); err != nil {
		c.recordFaultLocked(now, err, "trajectory buffer rejected waypoint")
		return protocol.MotorCommand{}, err
	}
	return protocol.MotorCommand{
		Mode: sp.Mode, PositionRad: sp.PositionRad,
		VelocityRadS: sp.VelocityRadS, TorqueNm: sp.TorqueNm,
		Seq: sp.Seq, IssuedAt: now,
	}, nil
}

// StreamPoint 是一个流式喂点：名义时刻 At 到达 PositionRad。
type StreamPoint = stream.Point

// SubmitStreamPoint 连续流式喂入一个目标点。允许在上一段还没走完时
// 就把下一个点塞进来，系统按提交顺序连续走完，不在每个点停车再起。
//
// 判定顺序（同一把锁，与 Step/Estop/SetSpeedOverride 并发时结论确定）：
//  1. 非 RUN => CatSafety/outputs_disabled，绝不落入流式校验；
//  2. 目标软限位预检（位置点会直接承诺位置）；
//  3. V1 兼容预检（流式点恒为位置模式，V1/V2 都支持）；
//  4. 交 Planner：位置限位、迟到/重叠、倒流、段速、容量策略，
//     原因码均为 command_rejected|stream，与普通单目标越界可区分；
//     任何拒绝都原样返回，绝不悄悄丢点。
//
// 起始位置取最近被接受的反馈样本；从未收到反馈时按 0 建锚点。
func (c *Controller) SubmitStreamPoint(pt stream.Point) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	now := c.clk.Now()
	if pt.At.IsZero() {
		// 未显式给时刻的点不允许“立即到期”，默认下一周期时刻，
		// 保证 late_point 语义严格（必须严格晚于当前时刻）。
		pt.At = now.Add(c.cfg.Period)
	}

	if state := c.machine.State(); state != safety.StateRun {
		err := fault.New(fault.CatSafety, fault.SourceController, ReasonOutputsDisabled,
			"stream point rejected: outputs disabled in safety state "+string(state))
		c.recordFaultLocked(now, err, "stream point not accepted in "+string(state))
		return err
	}

	sp := trajectory.Setpoint{Mode: protocol.ModePosition, PositionRad: pt.PositionRad}
	c.monitor.CheckSetpoint(now, sp)
	if state := c.machine.State(); state != safety.StateRun {
		err := fault.New(fault.CatSafety, fault.SourceSafety, safety.ReasonSoftLimit,
			"stream point enters soft-limit band, point held")
		c.recordFaultLocked(now, err, "stream point pre-check tripped soft limit")
		return err
	}

	startPos := 0.0
	known := false
	if sample, ok := c.consumer.Last(); ok {
		startPos, known = sample.PositionRad, true
	}

	if err := c.planner.Append(pt, now, startPos, known); err != nil {
		c.recordFaultLocked(now, err, "stream point rejected")
		return err
	}
	return nil
}

// SetSpeedOverride 在线调整整体速度倍率（例如 0.5 -> 1.2 -> 1.0）。
//
// 生效边界：仅改倍率状态；下一个完整控制周期才使用新倍率，任何周期都
// 不会出现一半旧倍率、一半新倍率。倍率对“正在走的段”和“后面排队的段”
// 同时生效。
//
// 拒绝规则（command_rejected|stream，区别于普通越界）：
//   - 非 RUN => outputs_disabled；
//   - 超出配置范围/非有限值 => override_out_of_range；
//   - 新倍率会把任一排队段顶破限速 => override_infeasible，绝不静默裁剪。
func (c *Controller) SetSpeedOverride(factor float64) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	now := c.clk.Now()
	if state := c.machine.State(); state != safety.StateRun {
		err := fault.New(fault.CatSafety, fault.SourceController, ReasonOutputsDisabled,
			"override rejected: outputs disabled in safety state "+string(state))
		c.recordFaultLocked(now, err, "override not accepted in "+string(state))
		return err
	}
	if err := c.planner.SetOverride(factor, now); err != nil {
		c.recordFaultLocked(now, err, "speed override rejected")
		return err
	}
	return nil
}

// SpeedOverride 返回当前生效倍率。
func (c *Controller) SpeedOverride() float64 {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.planner.Override()
}

// StreamQueueLen 返回流式排队深度（含活动点）。
func (c *Controller) StreamQueueLen() int {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.planner.Len()
}

// Estop 可从任意 goroutine 调用；与周期推进并发时结果确定：
// 急停提交后的任何周期都不会再下发出力命令，待发航点同时清空。
func (c *Controller) Estop(reason string) {
	c.mu.Lock()
	defer c.mu.Unlock()
	now := c.clk.Now()
	c.machine.Estop(now, reason)
	c.traj.Clear()
	// 安全优先：未走完的流式队列与在线倍率一律不保留，规则固定可复现。
	if c.planner.Len() > 0 {
		c.recordFaultLocked(now,
			fault.New(fault.CatSafety, fault.SourceController, ReasonStreamEvicted,
				"streaming queue discarded by safety stop"),
			"stream queue cleared on safety action")
	}
	c.planner.Reset()
}

// Recover 显式恢复。条件不满足时状态机返回 CatSafety/recovery_denied，
// 绝不静默恢复；成功时清空连续性记忆/反馈基准/待发航点与流式队列，
// 倍率复位为 1.0，防止失联前的陈旧数据在恢复后造成跳变。
//
// 当配置 RequireObservedRecovery=true 时，除调用方给出的
// RecoveryConditions 外，还必须与系统当时的【真实观测】交叉核验：
// 口头声称健康但系统实际仍失联/超力矩/位置超限位/故障码未清，
// 一律拒绝（safety|controller|recovery_conditions_contradict_observation），
// 不能光信调用方口头说条件满足。
func (c *Controller) Recover(cond safety.RecoveryConditions) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	now := c.clk.Now()

	if c.cfg.RequireObservedRecovery {
		if mismatch := c.observationMismatchLocked(cond); mismatch != "" {
			err := fault.New(fault.CatSafety, fault.SourceController,
				ReasonRecoveryObservedMismatch, mismatch)
			c.recordFaultLocked(now, err, "recovery denied by real system observation")
			return err
		}
	}

	if err := c.machine.RequestRecover(now, cond); err != nil {
		c.recordFaultLocked(now, err, "explicit recovery request denied")
		return err
	}
	c.validator.Reset()
	c.consumer.Reset()
	c.traj.Clear()
	c.planner.Reset()
	return nil
}

// observationMismatchLocked 用系统真实观测核验调用方声称的恢复条件。
// 仅在“有观测证据”时反驳：从未收到反馈时无法反驳健康类声明
// （此时由调用方条件与 OperatorAck 负责），避免无证据的误拒。
// EstopReleased/OperatorAck 属物理/人工输入，无系统内观测，不核验。
func (c *Controller) observationMismatchLocked(cond safety.RecoveryConditions) string {
	if c.consumer.IsLost() && cond.CommHealthy {
		return "caller claims comm healthy but watchdog still observes link lost"
	}
	sample, has := c.consumer.Last()
	if has {
		if cond.TorqueWithinLimit && sample.TorqueKnown &&
			(sample.TorqueNm > c.cfg.SafetyLim.MaxTorqueNm ||
				sample.TorqueNm < -c.cfg.SafetyLim.MaxTorqueNm) {
			return "caller claims torque within limit but latest feedback exceeds torque limit"
		}
		if cond.PositionWithinLimit &&
			(sample.PositionRad < c.cfg.SafetyLim.MinPositionRad ||
				sample.PositionRad > c.cfg.SafetyLim.MaxPositionRad) {
			return "caller claims position within limit but latest feedback position is outside hard limits"
		}
		if cond.FaultCodeClear && sample.DriveFault != 0 {
			return "caller claims drive fault code clear but latest feedback reports a fault code"
		}
	}
	return ""
}

// Step 推进一个控制周期。正常返回 nil；本周期内发生的一切归类
// 结论写入 Snapshot（LastFault / BusStats / 分类计数）与 EventSink，
// 调用方据此观测，而不会打断周期循环本身。
//
// 反馈的“接收与消费”独立于安全状态：即使处于 ESTOP/COMM_LOSS，
// 仍持续消费上行帧，否则恢复条件（链路健康、力矩正常、故障码清除）
// 永远无从判定。只有“轨迹推进/命令下发”受 OutputsEnabled 门控。
func (c *Controller) Step() error {
	c.mu.Lock()
	defer c.mu.Unlock()

	c.cycle++
	now := c.clk.Now()
	if !c.lastStepAt.IsZero() && c.cfg.Period > 0 {
		elapsed := now.Sub(c.lastStepAt)
		jitter := elapsed - c.cfg.Period
		if jitter < 0 {
			jitter = -jitter
		}
		if jitter > c.maxJitter {
			c.maxJitter = jitter
		}
		if c.cfg.JitterTolerance > 0 && jitter > c.cfg.JitterTolerance {
			c.jitterBreach++
		}
	}

	// 1) 反馈消费与安全监视——任何安全状态下都执行。
	c.lastAccepted, c.lastRejected = c.consumeUplinkLocked(now)

	// 2) 看门狗：暂时失联 => COMM_LOSS（CatComm，不是跟踪误差）。
	if werr := c.consumer.WatchdogCheck(now); werr != nil {
		c.recordFaultLocked(now, werr, "feedback watchdog tripped")
		if fe, ok := fault.As(werr); ok && fe.Reason == feedback.ReasonWatchdogTimeout {
			c.machine.TriggerEvent(now, safety.Trigger{
				Reason: safety.ReasonCommLoss,
				Detail: "no valid feedback within watchdog window",
			})
		}
	}

	// 3) 只有输出使能时才推进轨迹/下发命令；否则清空待发航点。
	//    安全动作（任意非 RUN 状态，含看门狗 COMM_LOSS、软限位）下，
	//    流式队列与倍率同样被确定性清空/复位，绝不静默丢点或静默恢复。
	if c.machine.OutputsEnabled() {
		c.advanceStreamLocked(now)
		c.advanceTrajectoryLocked(now)
	} else {
		c.traj.Clear()
		if c.planner.Len() > 0 {
			c.recordFaultLocked(now,
				fault.New(fault.CatSafety, fault.SourceController, ReasonStreamEvicted,
					"streaming queue discarded while outputs disabled"),
				"stream queue cleared: outputs disabled")
		}
		c.planner.Reset()
	}

	c.lastStepAt = now
	snap := c.buildSnapshotLocked(now)
	if c.sink != nil {
		c.sink.RecordCycle(snap)
	}
	return nil
}

// consumeUplinkLocked 每周期至多消费 MaxBacklog 帧，
// 防止消息积压时单个周期无界占用 CPU；未消费部分留在有界总线中，
// 总线自身按既定丢弃策略保证内存有界，丢弃在 BusStats 中可观测。
func (c *Controller) consumeUplinkLocked(now time.Time) (accepted, rejected int) {
	for i := 0; i < c.cfg.MaxBacklog; i++ {
		line, ok := c.bus.Poll(bus.Uplink)
		if !ok {
			return
		}
		sample, _, ierr := c.consumer.Ingest(line, now, nil)
		if ierr != nil {
			rejected++
			c.recordFaultLocked(now, ierr, "uplink frame not accepted")
			continue
		}
		accepted++
		// 已接受样本进入安全监视（故障码/力矩/软限位）。
		for _, r := range c.monitor.Check(now, sample) {
			c.recordFaultLocked(now,
				fault.New(fault.CatSafety, fault.SourceSafety, r, "safety monitor trigger on feedback"),
				"safety trigger: "+r)
		}
	}
	return
}

// advanceStreamLocked 推进流式运动：每周期取一个整形后的位置设定并
// 下发。流活动期间本方法“接管”下发，单目标缓冲本周期不再发令
// （两条通路互不混用，避免一周期两条位置命令）。
func (c *Controller) advanceStreamLocked(now time.Time) {
	out := c.planner.Advance(now)
	if !out.Active {
		return
	}
	c.streamSeq++
	cmd := protocol.MotorCommand{
		Mode: protocol.ModePosition, PositionRad: out.PositionRad,
		Seq: c.streamSeq, IssuedAt: now,
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
	if out.Drained {
		c.recordFaultLocked(now,
			fault.New(fault.CatComm, fault.SourceStream, ReasonStreamDrained,
				"stream reached last point and queue drained"),
			"stream drained; holding position")
	}
}

// advanceTrajectoryLocked 取出截至当前到期的最新航点并下发。
func (c *Controller) advanceTrajectoryLocked(now time.Time) {
	// 流式运动活动期间由 advanceStreamLocked 负责本周期下发。
	if c.planner.Active() {
		return
	}
	p, skipped, ok := c.traj.PopDue(now)
	if !ok {
		return
	}
	if skipped > 0 {
		err := fault.New(fault.CatComm, fault.SourceController, ReasonSkippedWaypoints,
			fmt.Sprintf("%d stale waypoint(s) skipped at cycle %d", skipped, c.cycle))
		c.recordFaultLocked(now, err, "")
	}
	cmd := protocol.MotorCommand{
		Mode: p.Mode, PositionRad: p.PositionRad, VelocityRadS: p.VelocityRadS,
		TorqueNm: p.TorqueNm, Seq: p.Seq, IssuedAt: now,
	}
	line, err := protocol.EncodeCommand(c.cfg.ProtocolVersion, cmd)
	if err != nil {
		// 例如在 V1 上请求力矩模式：明确协议归类，绝不下发半个命令。
		c.recordFaultLocked(now, err, "command encoding failed")
		return
	}
	if perr := c.bus.Post(bus.Downlink, line, cmd.Seq); perr != nil {
		// 队列满（drop_newest）或注入丢帧：带归属的通信故障。
		c.recordFaultLocked(now, perr, "downlink post failed")
		return
	}
	c.lastCmd = cmd
	c.hasCmd = true
}

// Snapshot 返回周期边界上的完整状态（值拷贝）。
func (c *Controller) Snapshot() Snapshot {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.buildSnapshotLocked(c.clk.Now())
}

func (c *Controller) buildSnapshotLocked(now time.Time) Snapshot {
	snap := Snapshot{
		At: now, Cycle: c.cycle,
		SafetyState:      c.machine.State(),
		OutputsOn:        c.machine.OutputsEnabled(),
		LastCommand:      c.lastCmd,
		BusStats:         c.bus.Stats(),
		LastJitter:       0,
		MaxJitter:        c.maxJitter,
		JitterBreaches:   c.jitterBreach,
		FeedbackAccepted: c.lastAccepted,
		FeedbackRejected: c.lastRejected,
		BacklogRemaining: c.bus.Pending(bus.Uplink),
		LastFault:        c.lastFault,
		StreamActive:     c.planner.Active(),
		StreamQueued:     c.planner.Len(),
		StreamFactor:     c.planner.Override(),
		StreamStats:      c.planner.Stats(),
	}
	if !c.lastStepAt.IsZero() && c.cfg.Period > 0 {
		if j := now.Sub(c.lastStepAt) - c.cfg.Period; j >= 0 {
			snap.LastJitter = j
		} else {
			snap.LastJitter = -j
		}
	}
	last, hasSample := c.consumer.Last()
	healthy := !c.consumer.IsLost() && hasSample
	snap.Tracking = TrackingInfo{
		Healthy:  healthy,
		LastSeq:  last.Seq,
		LastTsMs: last.TsMs,
	}
	if healthy && c.hasCmd {
		switch c.lastCmd.Mode {
		case protocol.ModePosition:
			snap.Tracking.ErrorRad = c.lastCmd.PositionRad - last.PositionRad
		case protocol.ModeVelocity:
			snap.Tracking.ErrorRadS = c.lastCmd.VelocityRadS - last.VelocityRadS
		}
	}
	return snap
}

// recordFaultLocked 记录最近故障并转发给 sink；err 必须是 *fault.Error，
// 这里透传其类别/来源/原因码，保证通信、协议、安全三类不被合并。
func (c *Controller) recordFaultLocked(now time.Time, err error, detail string) {
	fe, ok := fault.As(err)
	if !ok {
		fe = fault.Wrap(err, fault.CatComm, fault.SourceController, "unclassified", detail)
	}
	if now.IsZero() {
		now = c.clk.Now()
	}
	c.lastFault = FaultRef{
		Category: fe.Category, Source: fe.Source, Reason: fe.Reason,
		Message: fe.Message, At: now,
	}
	if c.sink != nil {
		c.sink.RecordFault(now, string(fe.Category), string(fe.Source), fe.Reason, detail)
	}
}

// Close 释放控制器关联资源。
func (c *Controller) Close() error {
	c.bus.Close()
	return nil
}

// SafetyEvents 返回安全状态机变迁历史（供上报/审计）。
func (c *Controller) SafetyEvents() []safety.Event {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.machine.Events()
}
