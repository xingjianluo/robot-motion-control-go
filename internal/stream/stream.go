// Package stream 在既有单目标命令能力之上提供“流式喂点 + 在线速度倍率”
// 的轨迹播放器（segment player）。
//
// 语义总览（详见各方法文档）：
//
//   - 上层像手轮/示教一样持续 Append 目标；每个目标（Target）带有一个
//     标称保持时长 Hold（倍率=1 时的持续时间），播放器按提交顺序把各段
//     首尾相接连续执行，段与段之间不会停下重新起步。
//   - SetOverride 在任意时刻调整整体速度倍率（如 0.5 / 1.2），对“正在
//     执行的段”和“排队中的段”同时生效；倍率切换发生在两个控制周期之间，
//     切换后的那个周期从头到尾只使用新倍率，不存在半周期旧倍率、半周期
//     新倍率。
//   - 每次 Append/SetOverride 都会按“当前真实状态”做完整可行性核算
//     （位置硬限位/软限位、插值周期速度、拼接加速度、力矩），不可行一律
//     明确拒绝并给出可区分原因，绝不静默裁剪或伪装成正常命令。
//   - 排队长度有上限（不含正在执行的段），满时按显式策略 RejectNew 或
//     EvictOldest 处理，内存永远有界，所有挤兑都计数并可观测。
//   - Player 自身不加锁；并发安全由 control.Controller 的单把锁保证，
//     使“提交/调速/周期推进”三者的临界区顺序固定、结论可复现。
package stream

import (
	"fmt"
	"math"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
)

// 稳定原因码（CatCommandReject / SourceStream），与普通单目标越界原因
// 明确区分。
const (
	// ReasonInvalidSegment 模式未知或标称保持时长非法（< 一个周期且非 0）。
	ReasonInvalidSegment = "stream_invalid_segment"
	// ReasonOverflowRejectNew 等待队列已满且策略为 reject_new。
	ReasonOverflowRejectNew = "stream_overflow_reject_new"
	// ReasonInfeasibleNow 段在激活瞬间按真实状态判定不可行（如位置段
	// 找不到位置基准、实际插值步长超速/拼接超加速），按固定规则丢弃该段。
	ReasonInfeasibleNow = "stream_segment_infeasible_now"
	// ReasonOverrideOutOfRange 倍率超出配置允许范围或非正数/NaN。
	ReasonOverrideOutOfRange = "stream_override_out_of_range"
	// ReasonTargetPositionOutOfLimit 位置目标超出硬位置范围。
	ReasonTargetPositionOutOfLimit = "stream_target_position_out_of_limit"
	// ReasonTargetVelocityOutOfLimit 速度目标按当前倍率缩放后超速。
	ReasonTargetVelocityOutOfLimit = "stream_target_velocity_out_of_limit"
	// ReasonTargetTorqueOutOfLimit 力矩目标超力矩上限（力矩幅值不被倍率缩放）。
	ReasonTargetTorqueOutOfLimit = "stream_target_torque_out_of_limit"
	// ReasonTargetSoftLimit 位置目标落入软限位带外。
	ReasonTargetSoftLimit = "stream_target_soft_limit"
	// ReasonSpliceAccel 相邻两段（或静止→首段）轮廓速度跳变要求的加速度超限。
	ReasonSpliceAccel = "stream_splice_accel_out_of_limit"
	// ReasonInterpStep 位置段每个周期的插值步长对应的速度超速度上限。
	ReasonInterpStep = "stream_interpolation_step_too_fast"
)

// TargetMode 是流式目标的模式（与协议命令模式一致的类型别名）。
type TargetMode = protocol.CommandMode

// Target 是一个流式目标点：在该段标称保持时长内，播放器逐周期下发对应
// 目标。位置段在 [起点, 目标位置] 间做逐周期线性插值，速度段逐周期下发
// 倍率缩放后的恒定速度，力矩段逐周期下发原幅值力矩（力矩不是“快慢”，
// 不被倍率缩放，只随倍率缩短/拉长保持时间）。
type Target struct {
	Mode  TargetMode
	Value float64 // 位置(rad) / 速度(rad/s) / 力矩(Nm)，由 Mode 决定
	// Hold 为倍率=1 时的标称持续时间，按控制周期四舍五入量化为整周期数，
	// 至少 1 个周期；Hold==0 表示恰好 1 个周期；Hold<0 或不足半个周期
	// （量化为 0）的非零值按非法段拒绝。
	Hold  time.Duration
	SeqIn uint64 // 调用方可携带的外部序号（仅透传，不参与内部判定）
}

// Dispatched 是一个周期内实际下发给驱动器的目标（已施加倍率）。
type Dispatched struct {
	Mode         protocol.CommandMode
	PositionRad  float64
	VelocityRadS float64
	TorqueNm     float64
	Seq          uint64 // 播放器分配的单调下发序号（不随恢复复位）
	SourceSeq    uint64 // 对应 Target.SeqIn
	AtOverride   float64
}

// OverflowPolicy 是等待队列（不含正在执行的段）满时的策略。
type OverflowPolicy string

const (
	// PolicyRejectNew：满则拒绝新目标（默认，安全优先，绝不挤掉已排队动作）。
	PolicyRejectNew OverflowPolicy = "reject_new"
	// PolicyEvictOldest：挤掉等待队列中最旧的段以接纳新目标；
	// 正在执行的段永远不会被挤掉。每次挤兑都计数、可观测。
	PolicyEvictOldest OverflowPolicy = "evict_oldest"
)

// Config 是播放器参数。
type Config struct {
	// Capacity 等待队列容量（不含正在执行的段）；<=0 时按 16 处理。
	Capacity int
	// Policy 满队策略；空值按 PolicyRejectNew 处理。
	Policy OverflowPolicy
	// Period 控制周期；段时长按整周期量化。<=0 时按 10ms 处理。
	Period time.Duration
	// MinOverride/MaxOverride 允许的倍率范围（1=100%）；零值取默认 1%~200%。
	MinOverride float64
	MaxOverride float64
	// 运动学硬约束（由控制器从 trajectory/safety 配置填入，保持单一事实来源）。
	MinPositionRad  float64
	MaxPositionRad  float64
	SoftLowRad      float64 // 软限位带下界；位置段目标越过即拒
	SoftHighRad     float64 // 软限位带上界
	MaxVelocityRadS float64
	MaxAccelRadS2   float64
	MaxTorqueNm     float64
}

// segment 是内部段表示。
type segment struct {
	mode      protocol.CommandMode
	value     float64 // 原始（未缩放）目标值
	sourceSeq uint64

	baseN int // 倍率=1 时的整周期数

	// 以下仅对“正在执行的段”有意义：
	effN     int     // 当前倍率下的规划总周期数
	elapsed  int     // 已执行周期数
	startPos float64 // 位置段起点
	hasStart bool    // 起点在入队时是否已知（位置链可推算）
	override float64

	stepVel  float64 // 位置段每周期轮廓速度（含符号）
	velConst float64 // 速度段每周期下发速度
	torque   float64 // 力矩段每周期下发力矩（不缩放）
}

// Eviction 记录一次因容量策略挤掉等待段的事件。
type Eviction struct {
	At        time.Time
	SourceSeq uint64
	Mode      protocol.CommandMode
	Value     float64
}

// DroppedSegment 记录一段在激活瞬间被判定不可行而丢弃的段（固定规则留痕）。
type DroppedSegment struct {
	At        time.Time
	SourceSeq uint64
	Mode      protocol.CommandMode
	Reason    string
	Detail    string
}

// Status 是播放器在周期边界上的可观测状态（值语义）。
type Status struct {
	Active          bool
	ActiveMode      protocol.CommandMode
	ActiveSourceSeq uint64
	ActiveElapsed   int
	ActiveTotal     int
	Waiting         int
	Capacity        int
	Policy          OverflowPolicy
	Override        float64
	DispatchedCount uint64
	CompletedCount  uint64
	RejectedCount   uint64
	EvictedCount    uint64
	LastEviction    Eviction
	HasLastEviction bool
}

// Player 是流式轨迹播放器（非并发安全，由控制器单把锁串行化）。
type Player struct {
	cfg      Config
	active   *segment
	waiting  []*segment
	override float64
	seq      uint64

	dispatched uint64
	completed  uint64
	rejected   uint64
	evicted    uint64
	lastEvict  Eviction
	hasEvict   bool
	dropped    []DroppedSegment

	// 上一周期实际下发的轮廓速度与位置（位置段为规划下发位置），
	// 用于拼接加速度核算与位置段起点连续性。
	lastContour float64
	hasContour  bool
	lastPos     float64
	hasPos      bool
	lastMode    protocol.CommandMode
}

// New 创建播放器；初始倍率为 1（100%）。
func New(cfg Config) *Player {
	if cfg.Capacity <= 0 {
		cfg.Capacity = 16
	}
	if cfg.Policy == "" {
		cfg.Policy = PolicyRejectNew
	}
	if cfg.MinOverride <= 0 {
		cfg.MinOverride = 0.01
	}
	if cfg.MaxOverride < cfg.MinOverride {
		cfg.MaxOverride = 2.0
	}
	return &Player{cfg: cfg, override: 1}
}

func streamFault(reason, detail string) error {
	return fault.New(fault.CatCommandReject, fault.SourceStream, reason, detail)
}

// effCycles 给出标称 baseN 个周期在倍率 scale 下的有效整周期数
// （倍率越大走得越快、周期数越少），至少 1 个。
func effCycles(baseN int, scale float64) int {
	n := int(math.Round(float64(baseN) / scale))
	if n < 1 {
		return 1
	}
	return n
}

// contourAt 计算段在给定倍率下的“轮廓速度量”（用于加速度拼接核算）。
// 位置段需要已知起点；起点未知（前面隔着速度/力矩段）时 ok=false，
// 相关检查推迟到激活瞬间按真实位置执行。
func contourAt(s *segment, scale float64, period time.Duration) (c float64, ok bool) {
	switch s.mode {
	case protocol.ModeVelocity:
		return s.value * scale, true
	case protocol.ModeTorque:
		return 0, true
	case protocol.ModePosition:
		if !s.hasStart {
			return 0, false
		}
		n := effCycles(s.baseN, scale)
		return (s.value - s.startPos) / float64(n) / period.Seconds(), true
	}
	return 0, false
}

// Append 追加一个流式目标到等待队列尾部。refPos/hasRef 为当前参考位置
// （通常取最近一条被接受反馈的位置），用于位置段起点连续性。
// 返回因 evict_oldest 策略被挤掉的段（调用方负责逐条留痕）。
//
// 判定顺序固定（任一步失败都不会让队列进入半更新状态；容量挤兑发生在
// 纯合法性检查之后、拼接检查之前，挤掉的段即使随后新目标被拒也不恢复，
// 这是确定且可观测的固定规则）：
//  1. 模式已知、Hold 可量化 => 否则 stream_invalid_segment；
//  2. 与在跑/在排队的模式一致（跨模式必须先回到空闲）=> stream 模式冲突
//     由控制器以独立原因拒绝（本层只暴露当前 Mode）；
//  3. 物理量按当前倍率核算：位置硬限位/软限位带、速度、力矩；
//  4. 按容量策略腾位（reject_new 直接拒；evict_oldest 挤队首并计数）；
//  5. 拼接可行性：位置段插值步长速度 + 与前驱段的轮廓加速度跳变。
func (p *Player) Append(t Target, refPos float64, hasRef bool, at time.Time) (evicted []Eviction, err error) {
	if t.Mode != protocol.ModePosition && t.Mode != protocol.ModeVelocity && t.Mode != protocol.ModeTorque {
		p.rejected++
		return nil, streamFault(ReasonInvalidSegment, "unknown stream target mode: "+string(t.Mode))
	}
	nBase := 1
	if t.Hold < 0 {
		p.rejected++
		return nil, streamFault(ReasonInvalidSegment, "hold duration must not be negative")
	}
	if t.Hold > 0 {
		nBase = int(math.Round(float64(t.Hold) / float64(p.cfg.period())))
		if nBase < 1 {
			p.rejected++
			return nil, streamFault(ReasonInvalidSegment,
				"hold duration quantizes to zero cycles; minimum is one control period")
		}
	}

	// 3. 物理量硬约束（速度按“当前倍率”缩放后核算；后续 SetOverride
	// 会对全部排队段重新核算，因此这里只对当前倍率负责）。
	switch t.Mode {
	case protocol.ModePosition:
		if t.Value < p.cfg.MinPositionRad || t.Value > p.cfg.MaxPositionRad {
			p.rejected++
			return nil, streamFault(ReasonTargetPositionOutOfLimit,
				fmtLimit(t.Value, p.cfg.MinPositionRad, p.cfg.MaxPositionRad))
		}
		if t.Value < p.cfg.SoftLowRad || t.Value > p.cfg.SoftHighRad {
			p.rejected++
			return nil, streamFault(ReasonTargetSoftLimit,
				fmtLimit(t.Value, p.cfg.SoftLowRad, p.cfg.SoftHighRad))
		}
	case protocol.ModeVelocity:
		if v := math.Abs(t.Value * p.override); v > p.cfg.MaxVelocityRadS {
			p.rejected++
			return nil, streamFault(ReasonTargetVelocityOutOfLimit,
				fmtScaled(t.Value, p.override, v, p.cfg.MaxVelocityRadS))
		}
	case protocol.ModeTorque:
		// 力矩幅值不随倍率缩放：倍率是“快慢”，不是“出力大小”。
		if v := math.Abs(t.Value); v > p.cfg.MaxTorqueNm {
			p.rejected++
			return nil, streamFault(ReasonTargetTorqueOutOfLimit,
				fmtLimit(t.Value, -p.cfg.MaxTorqueNm, p.cfg.MaxTorqueNm))
		}
	}

	s := &segment{mode: t.Mode, value: t.Value, sourceSeq: t.SeqIn, baseN: nBase}

	// 4. 容量策略。
	for len(p.waiting) >= p.cfg.Capacity {
		switch p.cfg.Policy {
		case PolicyEvictOldest:
			old := p.waiting[0]
			p.waiting = p.waiting[1:]
			ev := Eviction{At: at, SourceSeq: old.sourceSeq, Mode: old.mode, Value: old.value}
			p.evicted++
			p.lastEvict = ev
			p.hasEvict = true
			evicted = append(evicted, ev)
		default: // PolicyRejectNew
			p.rejected++
			return nil, streamFault(ReasonOverflowRejectNew,
				fmt.Sprintf("stream waiting queue full (%d), newest target rejected by policy", p.cfg.Capacity))
		}
	}

	// 位置段起点：前驱（活动段或队尾段）是位置段则沿位置链取其目标值；
	// 队列为空时用参考位置；前驱是速度/力矩段时起点在激活前未知。
	if t.Mode == protocol.ModePosition {
		if pred, ok := p.predecessorSegment(); ok {
			if pred.mode == protocol.ModePosition {
				s.startPos, s.hasStart = pred.value, true
			}
		} else if hasRef {
			s.startPos, s.hasStart = refPos, true
		}
	}

	// 5. 拼接/步长可行性（按当前倍率）。
	if c, known := contourAt(s, p.override, p.cfg.period()); known {
		if s.mode == protocol.ModePosition && math.Abs(c) > p.cfg.MaxVelocityRadS {
			p.rejected++
			return evicted, streamFault(ReasonInterpStep,
				fmt.Sprintf("interpolation step speed %.4f rad/s exceeds limit %.4f", c, p.cfg.MaxVelocityRadS))
		}
		if predC, predKnown := p.predecessorContour(); predKnown {
			if accel := math.Abs(c-predC) / p.cfg.period().Seconds(); accel > p.cfg.MaxAccelRadS2 {
				p.rejected++
				return evicted, streamFault(ReasonSpliceAccel,
					fmt.Sprintf("splice requires accel %.3f rad/s^2 > limit %.3f", accel, p.cfg.MaxAccelRadS2))
			}
		}
	}

	p.waiting = append(p.waiting, s)
	return evicted, nil
}

// predecessorSegment 返回新段的直接前驱段：优先活动段，否则队尾等待段。
func (p *Player) predecessorSegment() (*segment, bool) {
	if p.active != nil {
		return p.active, true
	}
	if n := len(p.waiting); n > 0 {
		return p.waiting[n-1], true
	}
	return nil, false
}

// predecessorContour 返回当前拼接点的轮廓速度：活动段按其激活时量化的
// 轮廓；否则队尾等待段按当前倍率折算；队列全空时取上一周期下发轮廓，
// 从未下发过则按静止 0 处理（与既有 trajectory.Validator 约定一致）。
func (p *Player) predecessorContour() (float64, bool) {
	if p.active != nil {
		switch p.active.mode {
		case protocol.ModePosition:
			return p.active.stepVel, true
		case protocol.ModeVelocity:
			return p.active.velConst, true
		default:
			return 0, true
		}
	}
	if n := len(p.waiting); n > 0 {
		return contourAt(p.waiting[n-1], p.override, p.cfg.period())
	}
	if p.hasContour {
		return p.lastContour, true
	}
	return 0, true
}

// SetOverride 在线调整速度倍率（1=100%）。对正在执行的段与排队段同时
// 生效。若新倍率会把任一段顶出速度/插值步长/拼接加速度边界，整体拒绝，
// 播放器状态（含活动段进度、队列）保持不变——原子生效，绝不半应用。
// 空闲时调用只更新倍率（无可行性对象），立即生效并对随后入队的段负责。
func (p *Player) SetOverride(scale float64, at time.Time) error {
	if math.IsNaN(scale) || math.IsInf(scale, 0) || scale < p.cfg.MinOverride || scale > p.cfg.MaxOverride {
		p.rejected++
		return streamFault(ReasonOverrideOutOfRange,
			fmt.Sprintf("override %.4f outside allowed [%.4f, %.4f]", scale, p.cfg.MinOverride, p.cfg.MaxOverride))
	}
	if scale == p.override {
		return nil
	}
	if p.active == nil && len(p.waiting) == 0 {
		p.override = scale
		return nil
	}

	// 在临时变量上完成全部核算，全部通过后才提交，保证原子性。
	type cand struct {
		s     *segment
		c     float64
		known bool
	}
	cands := make([]cand, 0, 1+len(p.waiting))
	if p.active != nil {
		newN := effCycles(p.active.baseN, scale)
		c, known := contourAt(&segment{
			mode: p.active.mode, value: p.active.value, baseN: p.active.baseN,
			startPos: p.active.startPos, hasStart: true,
		}, scale, p.cfg.period())
		_ = newN
		cands = append(cands, cand{p.active, c, known})
	}
	for _, w := range p.waiting {
		c, known := contourAt(w, scale, p.cfg.period())
		cands = append(cands, cand{w, c, known})
	}

	periodSec := p.cfg.period().Seconds()
	predC, predKnown := p.lastContour, p.hasContour
	for _, cd := range cands {
		if cd.known {
			switch cd.s.mode {
			case protocol.ModeVelocity, protocol.ModePosition:
				if math.Abs(cd.c) > p.cfg.MaxVelocityRadS {
					p.rejected++
					return streamFault(overrideBoundReason(cd.s.mode),
						fmt.Sprintf("override %.3f would require %.4f rad/s over limit %.4f (seq=%d)",
							scale, cd.c, p.cfg.MaxVelocityRadS, cd.s.sourceSeq))
				}
			}
			if predKnown {
				if accel := math.Abs(cd.c-predC) / periodSec; accel > p.cfg.MaxAccelRadS2 {
					p.rejected++
					return streamFault(ReasonSpliceAccel,
						fmt.Sprintf("override %.3f at seq=%d requires splice accel %.3f > limit %.3f",
							scale, cd.s.sourceSeq, accel, p.cfg.MaxAccelRadS2))
				}
			}
		}
		predC, predKnown = cd.c, cd.known
	}

	// 全部通过：提交。活动段按已完成比例把剩余周期重新量化到新倍率，
	// 已走过的位置份额保持不变，因此切换周期不会产生位置跳变。
	if p.active != nil {
		a := p.active
		newN := effCycles(a.baseN, scale)
		frac := float64(a.elapsed) / float64(a.effN)
		ne := int(math.Round(frac * float64(newN)))
		if ne < 0 {
			ne = 0
		}
		if ne > newN-1 {
			ne = newN - 1
		}
		a.effN = newN
		a.elapsed = ne
		a.override = scale
		switch a.mode {
		case protocol.ModePosition:
			a.stepVel = (a.value - a.startPos) / float64(newN) / periodSec
		case protocol.ModeVelocity:
			a.velConst = a.value * scale
		}
	}
	p.override = scale
	return nil
}

func overrideBoundReason(m protocol.CommandMode) string {
	if m == protocol.ModePosition {
		return ReasonInterpStep
	}
	return ReasonTargetVelocityOutOfLimit
}

// Step 推进一个控制周期，返回本周期应下发的目标；无段在跑时 ok=false。
// 若等待中的首段在激活瞬间按真实状态（refPos/hasRef，通常来自反馈）被
// 判定为不可行，按固定规则丢弃该段并继续尝试后续段，丢弃事件可经
// TakeDropped 取走留痕——绝不静默丢点。
func (p *Player) Step(now time.Time, refPos float64, hasRef bool) (Dispatched, bool) {
	for p.active == nil && len(p.waiting) > 0 {
		s := p.waiting[0]
		p.waiting = p.waiting[1:]

		if s.mode == protocol.ModePosition && !s.hasStart {
			if !hasRef {
				p.dropLocked(now, s, ReasonInfeasibleNow, "position segment activated without any position reference")
				continue
			}
			s.startPos, s.hasStart = refPos, true
		}
		s.override = p.override
		s.effN = effCycles(s.baseN, p.override)
		s.elapsed = 0
		switch s.mode {
		case protocol.ModePosition:
			s.stepVel = (s.value - s.startPos) / float64(s.effN) / p.cfg.period().Seconds()
		case protocol.ModeVelocity:
			s.velConst = s.value * p.override
		case protocol.ModeTorque:
			s.torque = s.value // 力矩幅值不被倍率缩放
		}

		// 激活瞬间按真实状态再做一次边界核算（主要覆盖入队时起点未定、
		// 以及 evict_oldest 改变拼接边界的情形）。
		c, known := contourAt(s, p.override, p.cfg.period())
		if known {
			bad := ""
			if s.mode != protocol.ModeTorque && math.Abs(c) > p.cfg.MaxVelocityRadS {
				bad = ReasonInfeasibleNow
			} else if p.hasContour && math.Abs(c-p.lastContour)/p.cfg.period().Seconds() > p.cfg.MaxAccelRadS2 {
				bad = ReasonInfeasibleNow
			}
			if bad != "" {
				p.dropLocked(now, s, bad, fmt.Sprintf(
					"segment infeasible against real state at activation: contour=%.4f prev=%.4f", c, p.lastContour))
				continue
			}
		}
		p.active = s
	}
	if p.active == nil {
		return Dispatched{}, false
	}

	a := p.active
	a.elapsed++
	d := Dispatched{Mode: a.mode, Seq: p.nextSeq(), SourceSeq: a.sourceSeq, AtOverride: a.override}
	switch a.mode {
	case protocol.ModePosition:
		pos := a.startPos + (a.value-a.startPos)*float64(a.elapsed)/float64(a.effN)
		if a.elapsed >= a.effN {
			pos = a.value // 最后一个周期精确落在目标，消除浮点残差
		}
		d.PositionRad = pos
		p.lastPos, p.hasPos = pos, true
		p.lastContour, p.hasContour = a.stepVel, true
	case protocol.ModeVelocity:
		d.VelocityRadS = a.velConst
		p.lastContour, p.hasContour = a.velConst, true
	case protocol.ModeTorque:
		d.TorqueNm = a.torque
		p.lastContour, p.hasContour = 0, true
	}
	p.lastMode = a.mode
	p.dispatched++
	if a.elapsed >= a.effN {
		p.completed++
		p.active = nil
	}
	return d, true
}

func (p *Player) dropLocked(at time.Time, s *segment, reason, detail string) {
	p.rejected++
	p.dropped = append(p.dropped, DroppedSegment{
		At: at, SourceSeq: s.sourceSeq, Mode: s.mode, Reason: reason, Detail: detail,
	})
}

func (p *Player) nextSeq() uint64 {
	p.seq++
	return p.seq
}

// TakeDropped 取走并清空自上次调用以来激活时被丢弃的段事件（控制器逐周期
// 取走并记录为带归属故障）。
func (p *Player) TakeDropped() []DroppedSegment {
	out := p.dropped
	p.dropped = nil
	return out
}

// Clear 清空正在执行的段与等待队列（安全停机时调用）；倍率与统计计数器
// 保留——倍率是操作员设置，其复位规则见 Reset，且计数用于审计不断档。
func (p *Player) Clear() (cleared int) {
	cleared = len(p.waiting)
	if p.active != nil {
		cleared++
	}
	p.active = nil
	p.waiting = p.waiting[:0]
	p.hasContour = false
	p.hasPos = false
	p.lastContour = 0
	p.lastPos = 0
	p.lastMode = ""
	return cleared
}

// Reset 在显式恢复后调用：清空队列，并把倍率与统计计数器复位到初始
// （全新运动上下文：防止恢复后首个动作以停机前的未知倍率冲出；下发序号
// 不复位，避免线上序号回退被对端误判为重复/乱序）。
func (p *Player) Reset() {
	p.Clear()
	p.override = 1
	p.dispatched = 0
	p.completed = 0
	p.rejected = 0
	p.evicted = 0
	p.hasEvict = false
	p.lastEvict = Eviction{}
	p.dropped = nil
}

// Active 报告是否有段在执行或排队。
func (p *Player) Active() bool { return p.active != nil || len(p.waiting) > 0 }

// Mode 返回当前流的模式（活动段优先，否则队首段）；空闲返回空串。
func (p *Player) Mode() protocol.CommandMode {
	if p.active != nil {
		return p.active.mode
	}
	if len(p.waiting) > 0 {
		return p.waiting[0].mode
	}
	return ""
}

// WaitingDepth 返回等待队列长度（不含正在执行的段）。
func (p *Player) WaitingDepth() int { return len(p.waiting) }

// Override 返回当前倍率（1=100%）。
func (p *Player) Override() float64 { return p.override }

// Status 返回可观测状态快照。
func (p *Player) Status() Status {
	st := Status{
		Active:          p.Active(),
		Waiting:         len(p.waiting),
		Capacity:        p.cfg.Capacity,
		Policy:          p.cfg.Policy,
		Override:        p.override,
		DispatchedCount: p.dispatched,
		CompletedCount:  p.completed,
		RejectedCount:   p.rejected,
		EvictedCount:    p.evicted,
		LastEviction:    p.lastEvict,
		HasLastEviction: p.hasEvict,
	}
	if p.active != nil {
		st.ActiveMode = p.active.mode
		st.ActiveSourceSeq = p.active.sourceSeq
		st.ActiveElapsed = p.active.elapsed
		st.ActiveTotal = p.active.effN
	}
	return st
}
