// Package device 是一个纯本地的单轴模拟驱动器/编码器：
// 它消费控制器下发的命令帧，按一个确定性的一阶模型更新关节状态，
// 再把状态帧发回上行总线。所有行为只依赖注入的时钟与总线，
// 不需要真实硬件。
package device

import (
	"sync"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/bus"
	"github.com/xingjianluo/robot-motion-control-go/internal/clock"
	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
)

// Config 是模拟驱动器参数。
type Config struct {
	Version protocol.Version // 驱动器协议版本，可与控制器版本不同以验证共存
	DT      time.Duration    // 设备内部积分步长
	Gain    float64          // 位置环一阶增益：vel = gain*(cmdPos-pos)
	MaxVel  float64
	HardMin float64
	HardMax float64
	// ReportTorqueV1 无意义（V1 线格式本就无 tau）；保留字段以显式表达。
	StartPos float64
	// PosCmdFeedforward 为 true 时，位置模式使用“相邻位置命令差分得到的
	// 前馈速度 + 位置误差修正”，模拟带轨迹前馈的真实伺服，用于连续流式
	// 轨迹的端到端跟踪；false（默认）保持历史的纯比例位置环行为，
	// 不影响任何既有用法与测试。
	PosCmdFeedforward bool
	// PosFFGain 是前馈模式下位置误差修正项的增益（默认等于 Gain）。
	PosFFGain float64
}

// Drive 是模拟驱动器。
type Drive struct {
	cfg     Config
	clk     clock.Clock
	bus     *bus.Bus
	mu      sync.Mutex
	pos     float64
	vel     float64
	tau     float64
	en      bool
	fault   int
	seq     uint64
	lastCmd protocol.MotorCommand
	// prevPosCmd / hasPrevPosCmd 用于位置命令速度前馈（相邻命令差分）。
	prevPosCmd    float64
	hasPrevPosCmd bool
	prevPosCmdAt  time.Time
	// 若解码器无法解释最新命令，记录原因（设备侧观测）。
	lastDecodeIssue string
}

// New 创建模拟驱动器。
func New(cfg Config, clk clock.Clock, b *bus.Bus) *Drive {
	if cfg.Gain == 0 {
		cfg.Gain = 20
	}
	if cfg.DT == 0 {
		cfg.DT = 10 * time.Millisecond
	}
	return &Drive{cfg: cfg, clk: clk, bus: b, pos: cfg.StartPos, en: true}
}

// SetEnabled 模拟使能/失能状态上报。
func (d *Drive) SetEnabled(en bool) {
	d.mu.Lock()
	defer d.mu.Unlock()
	d.en = en
}

// SetFaultCode 注入驱动器故障码（0 清除）。
func (d *Drive) SetFaultCode(code int) {
	d.mu.Lock()
	defer d.mu.Unlock()
	d.fault = code
}

// Position 返回当前模拟位置（测试断言用）。
func (d *Drive) Position() float64 {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.pos
}

// Velocity 返回当前模拟速度（测试断言用）。
func (d *Drive) Velocity() float64 {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.vel
}

// Seq 返回设备最近一帧状态的序号（测试/演示中按真实序号注入额外反馈帧时
// 使用，避免高序号注入帧把设备后续帧永久变成“序号回退”）。
func (d *Drive) Seq() uint64 {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.seq
}

// Tick 执行一拍：排空下行命令（取最新一条生效，其余标记积压丢弃的
// 责任由总线容量策略承担），积分运动学，再发一帧状态。
// 返回总线下发状态帧时的错误（例如注入丢帧），调用方可观测。
func (d *Drive) Tick() error {
	d.mu.Lock()
	dt := d.cfg.DT.Seconds()
	var cmd *protocol.MotorCommand
	// 排空本拍到达的命令，最后一条有效命令生效。
	for {
		line, ok := d.bus.Poll(bus.Downlink)
		if !ok {
			break
		}
		frame, err := protocol.Decode(line)
		if err != nil {
			d.lastDecodeIssue = err.Error()
			continue
		}
		c, err := protocol.DecodeCommand(frame)
		if err != nil {
			d.lastDecodeIssue = err.Error()
			continue
		}
		cmd = &c
	}
	if cmd != nil {
		d.lastCmd = *cmd
		switch cmd.Mode {
		case protocol.ModePosition:
			if d.cfg.PosCmdFeedforward {
				ffGain := d.cfg.PosFFGain
				if ffGain == 0 {
					ffGain = d.cfg.Gain
				}
				if d.hasPrevPosCmd && cmd.PositionRad == d.prevPosCmd {
					// 连续相同位置命令 = 显式位置保持：前馈速度为 0，
					// 由位置误差修正把残余速度拉回（不产生差分伪前馈）。
					d.vel = clamp(ffGain*(cmd.PositionRad-d.pos), -d.cfg.MaxVel, d.cfg.MaxVel)
				} else {
					// 前馈速度 = 相邻位置命令差分 / 命令时间差；
					// 再叠加误差修正，模拟带轨迹前馈的伺服，稳态无差、
					// 能平滑跟踪连续流式轨迹。
					var ff float64
					if d.hasPrevPosCmd && cmd.IssuedAt.After(d.prevPosCmdAt) {
						ff = (cmd.PositionRad - d.prevPosCmd) / cmd.IssuedAt.Sub(d.prevPosCmdAt).Seconds()
					}
					corr := ffGain * (cmd.PositionRad - d.pos)
					d.vel = clamp(ff+corr, -d.cfg.MaxVel, d.cfg.MaxVel)
				}
				d.prevPosCmd = cmd.PositionRad
				d.prevPosCmdAt = cmd.IssuedAt
				d.hasPrevPosCmd = true
			} else {
				d.vel = clamp(d.cfg.Gain*(cmd.PositionRad-d.pos), -d.cfg.MaxVel, d.cfg.MaxVel)
			}
		case protocol.ModeVelocity:
			d.vel = cmd.VelocityRadS
		case protocol.ModeTorque:
			// 简化模型：力矩经单位惯量直接给加速度，速度做硬限速。
			d.vel = clamp(d.vel+cmd.TorqueNm*dt, -d.cfg.MaxVel, d.cfg.MaxVel)
		}
	}
	d.pos = clamp(d.pos+d.vel*dt, d.cfg.HardMin, d.cfg.HardMax)
	d.seq++
	st := protocol.DriveStatus{
		PositionRad: d.pos, VelocityRadS: d.vel, TorqueNm: d.tau,
		Enabled: d.en, FaultCode: d.fault, Seq: d.seq,
		TsMs: d.clk.Now().UnixMilli(),
	}
	line, err := protocol.EncodeStatus(d.cfg.Version, st)
	d.mu.Unlock()
	if err != nil {
		return err
	}
	return d.bus.Post(bus.Uplink, line, st.Seq)
}

func clamp(x, lo, hi float64) float64 {
	if hi <= lo {
		return x
	}
	if x < lo {
		return lo
	}
	if x > hi {
		return hi
	}
	return x
}
