// Package stream 实现“手轮/示教”式连续流式喂点与在线速度倍率。
//
// 与单目标轨迹缓冲（internal/trajectory + Buffer）并存、互不影响：
// 原有的 SubmitCommand 单目标用法完全保留；本包提供按顺序连续走完、
// 不在每个目标点停顿的流式运动，并支持运动过程中整体调快调慢。
//
// 安全边界：
//   - 位置点受硬位置限位约束；
//   - 段速（按提交时当前倍率核算）受最大速度约束；
//   - 每周期速度变化被“一周期前瞻斜坡”限制在 MaxAccel*Period 内，
//     因此无论怎样接点、怎样在线改倍率（包括突变到 0 再拉回），
//     实际下发的速度/加速度都不会越限；
//   - 排队深度有界，满时策略明确（拒绝新点 / 挤掉最旧排队点），
//     挤掉动作有计数、有最近事件，可观测，内存永不越界。
//
// 时间语义：点上的 At 是“名义排程时刻”，只用于排队顺序、段速核算与
// 迟到判定；倍率缩放的是实际推进速度，因此倍率>1 可能提前到、
// 倍率<1（含 0 暂停）会延后到，各段速度始终按 段长/名义时长×倍率 取期望，
// 再由加速度斜坡整形。任何时刻喂入的新点 At 必须严格晚于当前时刻，
// 否则以 stream_late_point 明确拒绝，绝不悄悄丢点。
package stream

import (
	"fmt"
	"math"
	"sync"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
)

// FullPolicy 是流式排队达到容量上限时的策略。
type FullPolicy string

const (
	// RejectNew 拒绝新点并向调用方返回 queue_full 错误（不丢任何旧点）。
	RejectNew FullPolicy = "reject_new"
	// EvictOldest 挤掉队列中最旧的“尚未执行”排队点以接纳新点；
	// 当前正在执行的活动点绝不被挤掉，避免轨迹不连续。
	EvictOldest FullPolicy = "evict_oldest"
)

// 拒绝原因码（均为 command_rejected|stream，与普通单目标校验原因码可区分）。
const (
	// ReasonPositionOutOfLimit 目标点超出硬位置范围。
	ReasonPositionOutOfLimit = "stream_position_out_of_limit"
	// ReasonSegmentTooFast 相邻点段速按当前倍率核算超过最大速度。
	ReasonSegmentTooFast = "stream_segment_too_fast"
	// ReasonLatePoint 新点时间已被当前运动时刻越过（迟到/与现在重叠），明确拒绝。
	ReasonLatePoint = "stream_late_point"
	// ReasonBackwardsTime 新点时间早于队列中最后一个点（倒流）。
	ReasonBackwardsTime = "stream_backwards_time"
	// ReasonQueueFull 排队到上限且策略为 RejectNew。
	ReasonQueueFull = "stream_queue_full"
	// ReasonEvictInfeasible EvictOldest 下挤掉最旧点后剩余段仍不可行。
	ReasonEvictInfeasible = "stream_evict_infeasible"
	// ReasonOverrideOutOfRange 倍率超出配置允许范围（含非有限值）。
	ReasonOverrideOutOfRange = "override_out_of_range"
	// ReasonOverrideInfeasible 该倍率会把某个排队段顶到限速以外。
	ReasonOverrideInfeasible = "override_infeasible"
)

// Point 是一个流式目标点：名义时刻 At 应到达 PositionRad。
type Point struct {
	At          time.Time
	PositionRad float64
}

// Config 是 Planner 参数。
type Config struct {
	Capacity        int        // 排队容量上限（含当前活动点）
	FullPolicy      FullPolicy // 满时策略，默认 RejectNew
	Period          time.Duration
	MaxAccelRadS2   float64
	MaxVelocityRadS float64
	MinPositionRad  float64
	MaxPositionRad  float64
	MinOverride     float64 // 允许的最小倍率（0 表示可暂停推进）
	MaxOverride     float64 // 允许的最大倍率
}

// Output 是一个控制周期的流式推进结果。
type Output struct {
	// Active 为本周期是否真的产生了流式位置设定（运动中每周期都为 true，
	// 包括在某点上等待后续点的周期）。
	Active       bool
	PositionRad  float64
	VelocityRadS float64
	Factor       float64
	// Reached 为本周期恰好到达到期点（拼接点）。
	Reached bool
	// Drained 为到达后队列已空：本段流喂结束，未再续点则保持位置。
	Drained bool
}

// Stats 暴露流式运动的可观测计数与最近事件。
type Stats struct {
	Appended        int64
	Rejected        int64
	Replaced        int64 // 同一名义时刻替换排队尾点的次数
	Evicted         int64 // 挤掉旧点总数
	Reached         int64 // 到达点总数
	OverrideChanges int64
	OverrideRejects int64
	LastEvent       string
	LastEventAt     time.Time
}

// Planner 是流式喂点 + 在线倍率的纯逻辑核心（自身互斥，线程安全）。
// 不直接触碰总线/协议：控制器每周期调用 Advance 取出本周期设定。
type Planner struct {
	mu  sync.Mutex
	cfg Config

	factor float64

	// pending 为尚未到达的点，pending[0] 是当前段的目标（活动点）。
	pending []Point
	// gaps[i] 为 pending[i] 相对其前驱（锚点或 pending[i-1]）的名义时长，
	// 与 pending 等长；倍率伸缩后据此把后续链重钉到实际时刻。
	gaps []time.Duration
	// cruise[i] 为 pending[i] 提交时设计的【基准巡航速度】（带符号、倍率=1），
	// 与 pending 等长；改倍率重钉时据此求段时长，不受滞后/暂停污染。
	cruise []float64

	// 当前段起点（上一个到达点 / 流的起始位置）与当前积分状态。
	anchorPos float64
	anchorAt  time.Time
	hasAnchor bool
	curPos    float64
	curVel    float64
	active    bool
	// 排空后保持：流喂完后不立即停发，而是持续下发末点位置、把内部
	// 期望速度斜坡收敛到零，等静止后才停发，防止下游带着残余速度跑飞。
	holding bool
	holdPos float64

	stats Stats
}

// New 创建 Planner 并归一化配置（非法值给安全默认，保证内存有界）。
func New(cfg Config) *Planner {
	if cfg.Capacity <= 0 {
		cfg.Capacity = 32
	}
	if cfg.FullPolicy == "" {
		cfg.FullPolicy = RejectNew
	}
	if cfg.Period <= 0 {
		cfg.Period = 10 * time.Millisecond
	}
	if cfg.MaxAccelRadS2 <= 0 {
		cfg.MaxAccelRadS2 = 100
	}
	if cfg.MaxVelocityRadS <= 0 {
		cfg.MaxVelocityRadS = 2
	}
	if cfg.MaxOverride <= 0 {
		cfg.MaxOverride = 2
	}
	if cfg.MinOverride < 0 {
		cfg.MinOverride = 0
	}
	if cfg.MinOverride > cfg.MaxOverride {
		cfg.MinOverride = 0
	}
	return &Planner{cfg: cfg, factor: 1}
}

// Append 追加一个流式目标点（now 为控制器当前时刻）。
// startPos/known 为流首次启动时的实际位置（控制器用最近反馈位置；
// known=false 时按 0 建立锚点）。
//
// 判定顺序固定（拒绝原因彼此可区分，绝不静默丢点）：
//  1. 位置硬限位 => stream_position_out_of_limit；
//  2. p.At 必须严格晚于 now => stream_late_point（迟到/与当前重叠）；
//  3. 相对排队尾点时间只能递增；相等 => “替换”该时刻目标（有计数）；
//     早于尾点 => stream_backwards_time；
//  4. 相对上一点（无排队时相对流锚点）的段速按【当前倍率】核算，
//     超过最大速度 => stream_segment_too_fast——当前状态走不出来的点
//     在这里就被拒，而不是等到周期内顶到限速；
//  5. 容量上限：RejectNew => stream_queue_full；EvictOldest =>
//     挤掉最早的非活动点并重算受影响段，不可行 => stream_evict_infeasible。
func (p *Planner) Append(pt Point, now time.Time, startPos float64, known bool) error {
	p.mu.Lock()
	defer p.mu.Unlock()

	// rejectLocked 只更新“观测计数/最近事件”，绝不触碰任何运动状态
	// （队列/锚点/保持/倍率均保持拒绝前原样）：被拒的点只影响它自己。
	rejectLocked := func(reason, msg string) error {
		err := fault.New(fault.CatCommandReject, fault.SourceStream, reason, msg)
		p.stats.Rejected++
		p.noteLocked(now, reason)
		return err
	}
	// rejectErrLocked 对校验子过程返回的既有 fault 做同样的“只记账”处理，
	// 并原样把错误返回调用方（原因码保持可区分）。
	rejectErrLocked := func(err error) error {
		fe, ok := fault.As(err)
		if !ok {
			err = fault.New(fault.CatCommandReject, fault.SourceStream,
				ReasonSegmentTooFast, err.Error())
			fe, _ = fault.As(err)
		}
		p.stats.Rejected++
		p.noteLocked(now, fe.Reason)
		return err
	}

	if pt.PositionRad < p.cfg.MinPositionRad || pt.PositionRad > p.cfg.MaxPositionRad {
		return rejectLocked(ReasonPositionOutOfLimit, fmt.Sprintf(
			"stream point %.6f outside [%.6f, %.6f]",
			pt.PositionRad, p.cfg.MinPositionRad, p.cfg.MaxPositionRad))
	}
	if !pt.At.After(now) {
		return rejectLocked(ReasonLatePoint,
			"stream point timestamp must be strictly later than now (no overlap with current motion)")
	}

	// 流从未启动时的候选起始位置；仅用于校验计算，提交成功后才写入锚点。
	if !p.hasAnchor && !known {
		startPos = 0
	}

	// 本次追加成功后将生效的“段前驱基点”。全部校验通过前只做计算，
	// 绝不修改 holding/anchor/curPos 等运动记忆——尤其在末点保持期
	// 喂入一个走不到的点时，拒绝后必须继续按保持点逐周期下发。
	baseAt := now
	basePos := startPos
	resumeHold := false
	switch {
	case len(p.pending) > 0:
		tail := p.pending[len(p.pending)-1]
		baseAt, basePos = tail.At, tail.PositionRad
		// 时间倒流先于同刻替换判定。
		if pt.At.Before(tail.At) {
			return rejectLocked(ReasonBackwardsTime,
				"stream point timestamp earlier than last queued point")
		}
	case p.holding:
		// 上一段排空后的末点保持期：新段从保持点起算、从 now 起算。
		baseAt, basePos, resumeHold = now, p.holdPos, true
	case p.hasAnchor:
		baseAt, basePos = now, p.anchorPos
	}
	firstStart := !p.hasAnchor && !resumeHold

	// 同一执行时刻的点 => 替换排队尾点。先完成全部校验，成功后才一次性
	// 提交；拒绝时队列与保持状态原封不动（被拒点不改动任何既有状态）。
	if len(p.pending) > 0 && pt.At.Equal(p.pending[len(p.pending)-1].At) {
		n := len(p.pending)
		var gapPrev time.Duration
		var posPrev float64
		if n >= 2 {
			gapPrev = pt.At.Sub(p.pending[n-2].At)
			posPrev = p.pending[n-2].PositionRad
		} else if p.hasAnchor {
			gapPrev = pt.At.Sub(p.anchorAt)
			posPrev = p.anchorPos
		}
		if gapPrev > 0 {
			if err := p.checkSegmentSpeedLocked(
				Point{At: pt.At.Add(-gapPrev), PositionRad: posPrev}, pt,
				(pt.PositionRad-posPrev)/gapPrev.Seconds()); err != nil {
				return rejectErrLocked(err)
			}
		}
		var cruiseNew float64
		if gs := gapPrev.Seconds(); gs > 0 {
			cruiseNew = (pt.PositionRad - posPrev) / gs
		}
		p.pending[n-1] = pt
		p.gaps[n-1] = gapPrev
		p.cruise[n-1] = cruiseNew
		p.stats.Replaced++
		p.noteLocked(now, "same_timestamp_replaced")
		return nil
	}

	// 容量策略：EvictOldest 时先在本地副本上完成“挤掉 + 合并段重新校验”，
	// 任何一步不可行都直接拒绝，现有队列一个字节都不改。
	evicted := false
	if len(p.pending) >= p.cfg.Capacity {
		switch p.cfg.FullPolicy {
		case EvictOldest:
			// 活动点 pending[0] 不能挤（会改变正在走的路径）；只能挤 pending[1]。
			if len(p.pending) < 2 {
				return rejectLocked(ReasonEvictInfeasible,
					"only the active point remains; nothing safe to evict")
			}
			trialP := append(append([]Point{}, p.pending[:1]...), p.pending[2:]...)
			merged := p.gaps[1] + p.gaps[2]
			ms := merged.Seconds()
			if ms <= 0 {
				return rejectLocked(ReasonEvictInfeasible,
					"evicting oldest queued point would collapse segment time")
			}
			// 合并后新段的基准巡航速度 = 合并位移 / 合并时长，并必须可行。
			mergedCruise := (trialP[1].PositionRad - trialP[0].PositionRad) / ms
			if err := p.checkSegmentSpeedLocked(trialP[0], trialP[1], mergedCruise); err != nil {
				return rejectLocked(ReasonEvictInfeasible,
					"evicting oldest queued point would make remaining segment infeasible: "+err.Error())
			}
			// 挤掉后新点相对“挤掉后的尾点（活动点）”这一段同样必须可行。
			if err := p.checkSegmentLocked(trialP[len(trialP)-1], pt); err != nil {
				return rejectLocked(ReasonEvictInfeasible,
					"new point infeasible after oldest queued point evicted: "+err.Error())
			}
			trialG := append(append([]time.Duration{}, p.gaps[:1]...), p.gaps[2:]...)
			trialG[1] = merged
			trialV := append(append([]float64{}, p.cruise[:1]...), p.cruise[2:]...)
			trialV[1] = mergedCruise
			p.pending = trialP
			p.gaps = trialG
			p.cruise = trialV
			evicted = true
		default:
			return rejectLocked(ReasonQueueFull, fmt.Sprintf(
				"stream queue at capacity %d", p.cfg.Capacity))
		}
	}

	// 挤掉发生后，基点变为挤掉后的新尾点。
	if evicted {
		baseAt = p.pending[len(p.pending)-1].At
		basePos = p.pending[len(p.pending)-1].PositionRad
	}
	// 普通追加段（基点 -> 新点）可行性。
	if pt.At.After(baseAt) {
		if err := p.checkSegmentLocked(Point{At: baseAt, PositionRad: basePos}, pt); err != nil {
			return rejectErrLocked(err)
		}
	}

	// ---- 全部校验通过，以下为唯一提交区：任何拒绝都不会走到这里 ----
	gapPrevAppend := pt.At.Sub(baseAt)
	var cruiseNew float64
	if gs := gapPrevAppend.Seconds(); gs > 0 {
		cruiseNew = (pt.PositionRad - basePos) / gs
	}
	if resumeHold {
		// 保持期续点：以保持点为新锚点并退出保持，保持期逐周期下发不中断。
		p.anchorAt = now
		p.anchorPos = p.holdPos
		p.curPos = p.holdPos
		p.holding = false
	}
	if firstStart {
		// 流首次启动：校验全部通过后才建立锚点，被拒点不污染运动记忆。
		p.hasAnchor = true
		p.anchorAt = now
		p.anchorPos = startPos
		p.curPos = startPos
	}
	p.pending = append(p.pending, pt)
	p.gaps = append(p.gaps, gapPrevAppend)
	p.cruise = append(p.cruise, cruiseNew)
	p.active = true
	p.stats.Appended++
	if evicted {
		p.stats.Evicted++
		p.noteLocked(now, "evicted_oldest_point")
	}
	return nil
}

// checkSegmentLocked 校验 from->to 段在当前倍率下不超速度、时长为正。
func (p *Planner) checkSegmentLocked(from, to Point) error {
	dt := to.At.Sub(from.At).Seconds()
	if dt <= 0 {
		return fault.New(fault.CatCommandReject, fault.SourceStream, ReasonLatePoint,
			"segment duration must be positive")
	}
	baseCruise := (to.PositionRad - from.PositionRad) / dt
	return p.checkSegmentSpeedLocked(from, to, baseCruise)
}

// checkSegmentSpeedLocked 按显式给出的【基准巡航速度】（倍率=1）与当前
// 倍率核算 from->to 段速是否超限。挤掉合并段时合并巡航速度不等于按
// from/to 时刻差反推的值，因此需要显式传入。
func (p *Planner) checkSegmentSpeedLocked(from, to Point, baseCruise float64) error {
	dt := to.At.Sub(from.At).Seconds()
	if dt <= 0 {
		return fault.New(fault.CatCommandReject, fault.SourceStream, ReasonLatePoint,
			"segment duration must be positive")
	}
	speed := math.Abs(baseCruise) * p.factor
	if speed > p.cfg.MaxVelocityRadS {
		return fault.New(fault.CatCommandReject, fault.SourceStream, ReasonSegmentTooFast,
			fmt.Sprintf("segment speed %.4f at factor %.2f exceeds max %.4f",
				speed, p.factor, p.cfg.MaxVelocityRadS))
	}
	return nil
}

// SetOverride 在线调整整体倍率，对当前段和后续排队段同时生效。
//
// 生效边界：仅修改倍率状态，下一个完整控制周期的 Advance 才使用新值，
// 因此任何周期都不可能“半周期旧倍率、半周期新倍率”。
//
// 拒绝规则（与普通越界原因码不同）：
//   - 非有限值或超出 [MinOverride, MaxOverride] => override_out_of_range；
//   - 新倍率会使任一排队段速超过最大速度 => override_infeasible
//     （绝不靠静默裁剪倍率把危险调速伪装成接受）。
func (p *Planner) SetOverride(factor float64, now time.Time) error {
	p.mu.Lock()
	defer p.mu.Unlock()

	if math.IsNaN(factor) || math.IsInf(factor, 0) ||
		factor < p.cfg.MinOverride || factor > p.cfg.MaxOverride {
		err := fault.New(fault.CatCommandReject, fault.SourceStream, ReasonOverrideOutOfRange,
			fmt.Sprintf("override %.4f outside [%.2f, %.2f]",
				factor, p.cfg.MinOverride, p.cfg.MaxOverride))
		p.stats.OverrideRejects++
		p.noteLocked(now, ReasonOverrideOutOfRange)
		return err
	}

	// 可行性核算：用“改倍率瞬间的真实锚点（当前实际位置/当前时刻）”
	// 重排整条剩余链，检查每段在新倍率下是否超最大速度。
	// 任一排队段超速即拒，绝不静默裁剪倍率。
	chain, chainAt := p.rescaleChainLocked(now, factor)
	for i := 0; i+1 < len(chain); i++ {
		dt := chainAt[i+1].Sub(chainAt[i]).Seconds()
		if dt <= 0 {
			continue
		}
		speed := math.Abs(chain[i+1]-chain[i]) / dt
		if speed > p.cfg.MaxVelocityRadS+1e-9 {
			err := fault.New(fault.CatCommandReject, fault.SourceStream, ReasonOverrideInfeasible,
				fmt.Sprintf("override %.2f would drive a queued segment speed %.4f past max %.4f",
					factor, speed, p.cfg.MaxVelocityRadS))
			p.stats.OverrideRejects++
			p.noteLocked(now, ReasonOverrideInfeasible)
			return err
		}
	}

	if factor != p.factor {
		// 提交重排：活动段锚点钉到当前实际位置/时刻，整链时刻按新倍率重钉，
		// 段的期望速度与排程时刻从此严格一致（下一完整周期生效，
		// 不存在半周期混用两种倍率）。
		if p.active && len(p.pending) > 0 {
			p.anchorPos = p.curPos
			p.anchorAt = now
			// chainAt[0] 是锚点 now，第 i 个排队点对应 chainAt[i+1]。
			for i := range p.pending {
				p.pending[i].At = chainAt[i+1]
			}
			p.rebuildGapsLocked()
		}
		p.factor = factor
		p.stats.OverrideChanges++
		p.noteLocked(now, fmt.Sprintf("override=%.2f", factor))
	}
	return nil
}

// rescaleChainLocked 计算在 now 时刻把倍率改为 factor 后，剩余链各点
// 相对当前实际位置的【目标位置序列】与【重钉后的到达时刻序列】。
// 规则：每段时长 = |位移| / (基准段速×factor)，其中基准段速按各点
// 提交时的名义排程（gaps 与原位置差）确定。
func (p *Planner) rescaleChainLocked(now time.Time, factor float64) (positions []float64, ats []time.Time) {
	// 基准（factor=1）段速：来自现有 gaps（活动段 gap 可能因滞后已不
	// 准确），因此活动段用“当前位置->活动点”的剩余位移与剩余名义时长。
	n := len(p.pending)
	positions = make([]float64, 0, n+1)
	ats = make([]time.Time, 0, n+1)
	positions = append(positions, p.curPos)
	ats = append(ats, now)
	curPos := p.curPos
	curAt := now
	for i := 0; i < n; i++ {
		target := p.pending[i].PositionRad
		// 基准（倍率=1）巡航速度直接取提交时记录的 cruise[i]，
		// 不受斜坡滞后/暂停把锚点钉到当前位置的影响。
		baseSpeed := math.Abs(p.cruise[i])
		if factor == 0 {
			// 暂停：把整链推到“远未来”（足够远，使暂停期间位移判据
			// 不可能满足），期望速度=0、dx=0，于是持续下发当前位置保持；
			// 恢复到正倍率时再正常重钉。farHorizon 只需大于任何真实运动时长。
			far := now.Add(time.Hour)
			positions = append(positions, target)
			ats = append(ats, far)
			continue
		}
		// 段时长按【重排起点(now,curPos)→本点】位移 / (基准巡航×factor)。
		// SetOverride 提交时锚点即钉到 (now,curPos)，与 Advance 期望速度匹配。
		moveLen := math.Abs(target - curPos)
		dur := 0.0
		if baseSpeed > 0 {
			dur = moveLen / (baseSpeed * factor)
		}
		curPos = target
		if dur > 0 {
			curAt = curAt.Add(time.Duration(dur * float64(time.Second)))
		}
		positions = append(positions, target)
		ats = append(ats, curAt)
	}
	return positions, ats
}

// rebuildGapsLocked 依据当前 anchorAt 与 pending 时刻重建 gaps。
func (p *Planner) rebuildGapsLocked() {
	p.gaps = p.gaps[:0]
	prev := p.anchorAt
	for _, pt := range p.pending {
		p.gaps = append(p.gaps, pt.At.Sub(prev))
		prev = pt.At
	}
}

// Advance 推进一个控制周期。锚点在首个点 Append 成功时已建立，
// 因此这里只需周期时刻 now。
//
// 安全整形规则：
//   - 巡航期望 = 提交时设计的基准巡航速度 × 当前倍率（改倍率时整链
//     排程时刻已由 SetOverride 重钉，到达点时由 rescheduleLocked 顺延）；
//   - “剩余位移/剩余墙钟时间”兜底项保证在 head.At 精确到点；
//   - 实际下发速度每周期最多变化 MaxAccel×Period，绝对值不超 MaxVelocity，
//     所以接点拼接与倍率突变（含 0 暂停）都不会把速度/加速度顶到限外；
//   - 到期点在周期内被精确吸附；排空后进入末点保持（无限期下发末点
//     位置，直到续点或安全动作清除），杜绝下游带残余速度跑飞。
func (p *Planner) Advance(now time.Time) Output {
	p.mu.Lock()
	defer p.mu.Unlock()

	out := Output{Factor: p.factor}
	if !p.active || len(p.pending) == 0 {
		if p.holding {
			// 末点保持：无限期持续下发末点位置，直到续点或安全动作清除。
			// 不能在固定窗口后停发——下游一旦收不到新位置命令就可能维持
			// 残余速度漂移。对外速度语义为 0（纯位置保持，内部 curVel
			// 仅平滑退出用）。
			p.curVel = ramp(p.curVel, 0, p.cfg.MaxAccelRadS2*p.cfg.Period.Seconds(),
				p.cfg.MaxVelocityRadS)
			if math.Abs(p.curVel) < 1e-12 {
				p.curVel = 0
			}
			out.Active = true
			out.PositionRad = p.holdPos
			out.VelocityRadS = 0
			return out
		}
		// 空闲期（等待续点）：把残余速度斜坡收敛到零，避免下一段启动时
		// 沿用过冲的历史速度；本周期不下发位置设定。
		if p.curVel != 0 {
			p.curVel = ramp(p.curVel, 0, p.cfg.MaxAccelRadS2*p.cfg.Period.Seconds(),
				p.cfg.MaxVelocityRadS)
			if math.Abs(p.curVel) < 1e-12 {
				p.curVel = 0
			}
		}
		return out
	}
	head := p.pending[0]

	periodS := p.cfg.Period.Seconds()
	dvMax := p.cfg.MaxAccelRadS2 * periodS

	// 零长度段（同位等待点）：锚点已在该位置，立即视为到达并拼接下一点。
	if head.PositionRad == p.anchorPos {
		return p.arriveLocked(now, head, out)
	}

	// 段期望速度（带符号）。
	//
	// 巡航期望 = 提交时设计的基准巡航速度 cruise[0] × 当前倍率
	// （SetOverride 已把排程时刻按倍率重钉，这里用保存的 cruise 才不会
	// 被“暂停时锚点钉到当前位置”污染）；同时用“剩余位移/剩余墙钟时间”
	// 得到为准时到点所需的兜底速度，取二者沿运动方向的更小者：
	//   - 正常时贴巡航速度连续运动，接点不停车；
	//   - 斜坡起步/暂停恢复滞后时兜底项自然减速，保证在 head.At 精确到点
	//     （时间与位置都自洽，不冲点、不拖点）；
	//   - 每周期速度变化由 ramp 钳在 dvMax，绝对值不超 MaxVelocity。
	cruise := 0.0
	if len(p.cruise) > 0 {
		cruise = p.cruise[0] * p.factor
	}
	desired := cruise
	remTime := head.At.Sub(now).Seconds()
	remPos := head.PositionRad - p.curPos
	if remTime > 0 {
		catchup := remPos / remTime
		if desired == 0 {
			desired = catchup
		} else if desired*catchup > 0 { // 同向才比较，避免反向拉扯
			if (desired > 0 && catchup < desired) || (desired < 0 && catchup > desired) {
				desired = catchup
			}
		}
	}
	// factor=0（暂停）显式归零，确保位置保持。
	if p.factor == 0 {
		desired = 0
	}
	p.curVel = ramp(p.curVel, desired, dvMax, p.cfg.MaxVelocityRadS)

	dx := p.curVel * periodS
	rem := head.PositionRad - p.curPos
	// 到达严格以位移判据为准：本周期步进覆盖剩余位移即精确吸附到点。
	reached := rem != 0 && ((rem > 0 && dx >= rem) || (rem < 0 && dx <= rem))
	if reached {
		return p.arriveLocked(now, head, out)
	}

	p.curPos += dx
	// 等待/暂停期间（倍率=0，dx=0）也必须持续下发“当前位置保持”设定，
	// 绝不能停发——否则下游会沿用上一条高速命令继续冲。
	out.Active = true
	out.PositionRad = p.curPos
	out.VelocityRadS = p.curVel
	return out
}

// arriveLocked 处理到达到期点：弹出活动点、推进锚点、按当前倍率重钉
// 后续排程、统计与排空判定。
func (p *Planner) arriveLocked(now time.Time, head Point, out Output) Output {
	p.curPos = head.PositionRad
	p.anchorPos = head.PositionRad
	p.anchorAt = now
	p.pending = p.pending[1:]
	p.gaps = p.gaps[1:]
	p.cruise = p.cruise[1:]
	p.stats.Reached++
	p.noteLocked(now, fmt.Sprintf("reached_point_at=%v", head.At))

	// 重钉后续点：保持各点之间的【名义间隔】不变，把整条剩余链平移到
	// 从 now 起算。于是低倍率/加速滞后造成的晚到只顺延后续段；高倍率
	// 早到则压缩前段、后段仍按自身名义时长走。期望速度与积分位移自洽。
	p.rescheduleLocked(now)

	out.Active = true
	out.PositionRad = head.PositionRad
	out.VelocityRadS = p.curVel
	out.Reached = true
	if len(p.pending) == 0 {
		// 流喂结束：进入“末点保持”。末段速度立即按斜坡收敛到零
		// （排空后由保持期持续下发末点位置），避免到达后仍带残余速度。
		p.curVel = ramp(p.curVel, 0, p.cfg.MaxAccelRadS2*p.cfg.Period.Seconds(),
			p.cfg.MaxVelocityRadS)
		if math.Abs(p.curVel) < 1e-12 {
			p.curVel = 0
		}
		p.active = false
		p.holding = true
		p.holdPos = head.PositionRad
		out.Drained = true
	}
	return out
}

// rescheduleLocked 把排队链以 base 为起点、按原相邻名义间隔重新钉时刻。
func (p *Planner) rescheduleLocked(base time.Time) {
	if len(p.pending) == 0 {
		return
	}
	cur := base
	for i := range p.pending {
		cur = cur.Add(p.gaps[i])
		p.pending[i].At = cur
	}
}

// ramp 把 cur 朝 target 每周期最多移动 dvMax，并钳在 maxVel 内。
func ramp(cur, target, dvMax, maxVel float64) float64 {
	v := cur
	if target > v+dvMax {
		v += dvMax
	} else if target < v-dvMax {
		v -= dvMax
	} else {
		v = target
	}
	if v > maxVel {
		v = maxVel
	} else if v < -maxVel {
		v = -maxVel
	}
	return v
}

// Deactivate 退出流式运动（含排空后的末点保持）并回到干净空闲状态：
// 清空排队与运动记忆、倍率复位 1.0。当一条单目标命令被成功接受时由
// 控制器调用，表示本会话从“流式通路”明确切回“单目标通路”。
//
// 与 Reset 的区别只在语义：Reset 是安全动作（强制、留安全类记录），
// Deactivate 是用户显式接管（正常模式切换）；二者对运动状态的清理一致，
// 保证切回单目标后 Step 不再被流式保持期“接管”下发。
func (p *Planner) Deactivate() {
	p.Reset()
}

// Reset 清空排队与运动记忆，并把倍率恢复为 1.0（安全动作/恢复时调用，
// 规则固定：安全优先，未走完的队列一律不保留）。
func (p *Planner) Reset() {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.pending = p.pending[:0]
	p.gaps = p.gaps[:0]
	p.cruise = p.cruise[:0]
	p.active = false
	p.hasAnchor = false
	p.anchorPos = 0
	p.anchorAt = time.Time{}
	p.curPos = 0
	p.curVel = 0
	p.holding = false
	p.holdPos = 0
	p.factor = 1
}

// Override 返回当前倍率。
func (p *Planner) Override() float64 {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.factor
}

// Len 返回排队点数（含活动点）。
func (p *Planner) Len() int {
	p.mu.Lock()
	defer p.mu.Unlock()
	return len(p.pending)
}

// Active 返回是否有流式运动在进行（含排空后的末点保持期）。
func (p *Planner) Active() bool {
	p.mu.Lock()
	defer p.mu.Unlock()
	return (p.active && len(p.pending) > 0) || p.holding
}

// Stats 返回统计快照（值拷贝）。
func (p *Planner) Stats() Stats {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.stats
}

func (p *Planner) noteLocked(now time.Time, event string) {
	p.stats.LastEvent = event
	p.stats.LastEventAt = now
}
