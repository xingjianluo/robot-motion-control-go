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
			d.vel = clamp(d.cfg.Gain*(cmd.PositionRad-d.pos), -d.cfg.MaxVel, d.cfg.MaxVel)
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
