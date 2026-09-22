package controller_test

// testkit 提供确定性时钟下的“虚拟驱动器”：
// 它按控制周期产生编码器/驱动器反馈帧到本地模拟总线，
// 位置跟随总线收到的位置命令，可注入延迟、乱序、丢帧、跳变等故障。
import (
	"sync"
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/pkg/bus"
	"github.com/xingjianluo/robot-motion-control-go/pkg/frame"
	"github.com/xingjianluo/robot-motion-control-go/pkg/limits"
)

type fakeDrive struct {
	mu      sync.Mutex
	b       *bus.Bus
	v       frame.Version
	n       int
	pos     []float64
	vel     []float64
	torque  []float64
	seq     []uint32
	dropped map[int]bool // 本周期该关节是否注入丢帧
	delayQ  []frame.Frame
	posBias map[int]float64 // 注入恒定位置偏差（模拟真实跟踪误差）
	faultB  map[int]bool
	sentRaw int
}

func newFakeDrive(b *bus.Bus, v frame.Version, n int) *fakeDrive {
	d := &fakeDrive{b: b, v: v, n: n,
		dropped: map[int]bool{}, posBias: map[int]float64{}, faultB: map[int]bool{}}
	for i := 0; i < n; i++ {
		d.seq = append(d.seq, 1)
		d.pos = append(d.pos, 0)
		d.vel = append(d.vel, 0)
		d.torque = append(d.torque, 0)
	}
	return d
}

// respond 消费总线上的命令帧（模拟驱动器执行），随后发反馈帧。
func (d *fakeDrive) respond(ts time.Time, drop []bool, reorder bool) {
	d.mu.Lock()
	defer d.mu.Unlock()
	// 1) 执行命令
	for {
		f, ok := d.b.Recv()
		if !ok {
			break
		}
		if f.Joint < 0 || f.Joint >= d.n {
			continue
		}
		// 命令负载为 f64
		if len(f.Payload) >= 8 {
			v := floatFromPayload(f.Payload)
			switch f.Type {
			case frame.TypeCmdPosition:
				d.pos[f.Joint] = v
			case frame.TypeCmdVelocity:
				d.vel[f.Joint] = v
			}
		}
	}
	_ = drop
	_ = reorder
}

// emit 发送一周期反馈（可注入故障）。
func (d *fakeDrive) emit(t *testing.T, ts time.Time) {
	t.Helper()
	d.mu.Lock()
	defer d.mu.Unlock()
	var frames []frame.Frame
	for j := 0; j < d.n; j++ {
		if d.dropped[j] {
			continue
		}
		fb := frame.FeedbackData{
			Joint: j, Seq: d.seq[j], Timestamp: ts.UnixNano(),
			Position: d.pos[j] + d.posBias[j],
			Velocity: d.vel[j],
			Torque:   d.torque[j],
			Fault:    d.faultB[j],
		}
		f, err := frame.EncodeFeedback(d.v, fb)
		if err != nil {
			t.Fatalf("emit encode: %v", err)
		}
		frames = append(frames, f)
		d.seq[j]++
	}
	for _, f := range frames {
		if err := d.b.Send(f); err != nil {
			t.Fatalf("emit send: %v", err)
		}
		d.sentRaw++
	}
	d.dropped = map[int]bool{}
}

func (d *fakeDrive) dropNext(j int) {
	d.mu.Lock()
	d.dropped[j] = true
	d.mu.Unlock()
}

func (d *fakeDrive) setBias(j int, b float64) {
	d.mu.Lock()
	d.posBias[j] = b
	d.mu.Unlock()
}

func (d *fakeDrive) setFault(j int, on bool) {
	d.mu.Lock()
	d.faultB[j] = on
	d.mu.Unlock()
}

func (d *fakeDrive) setTorque(j int, tau float64) {
	d.mu.Lock()
	d.torque[j] = tau
	d.mu.Unlock()
}

// jump 直接修改上报位置（不经过命令），用于模拟编码器跳变
func (d *fakeDrive) jump(j int, pos float64) {
	d.mu.Lock()
	d.pos[j] = pos
	d.mu.Unlock()
}

func (d *fakeDrive) emitDuplicate(t *testing.T, ts time.Time, j int) {
	t.Helper()
	d.mu.Lock()
	defer d.mu.Unlock()
	seq := d.seq[j] - 1 // 上一次已发出的序号
	fb := frame.FeedbackData{Joint: j, Seq: seq, Timestamp: ts.UnixNano(),
		Position: d.pos[j], Velocity: d.vel[j], Torque: d.torque[j]}
	f, err := frame.EncodeFeedback(d.v, fb)
	if err != nil {
		t.Fatal(err)
	}
	if err := d.b.Send(f); err != nil {
		t.Fatal(err)
	}
}

// emitStaleSeq 发送一个序号回退的帧
func (d *fakeDrive) emitOldSeq(t *testing.T, ts time.Time, j int, oldSeq uint32) {
	t.Helper()
	d.mu.Lock()
	defer d.mu.Unlock()
	fb := frame.FeedbackData{Joint: j, Seq: oldSeq, Timestamp: ts.UnixNano(),
		Position: d.pos[j], Velocity: d.vel[j], Torque: d.torque[j]}
	f, err := frame.EncodeFeedback(d.v, fb)
	if err != nil {
		t.Fatal(err)
	}
	if err := d.b.Send(f); err != nil {
		t.Fatal(err)
	}
}

func twoJointLimits() []limits.JointLimits {
	return []limits.JointLimits{
		{ID: 0, PosMin: -1.57, PosMax: 1.57, VelMax: 1.0, AccMax: 2.0,
			TorqueMax: 10, SoftMargin: 0.05},
		{ID: 1, PosMin: -3.14, PosMax: 3.14, VelMax: 2.0, AccMax: 4.0,
			TorqueMax: 20, SoftMargin: 0.1},
	}
}

// emitCustom 发送任意构造的反馈帧（用于时间戳异常等注入）。
func (d *fakeDrive) emitCustom(t *testing.T, fb frame.FeedbackData) {
	t.Helper()
	d.mu.Lock()
	defer d.mu.Unlock()
	f, err := frame.EncodeFeedback(d.v, fb)
	if err != nil {
		t.Fatal(err)
	}
	if err := d.b.Send(f); err != nil {
		t.Fatal(err)
	}
}

// peekNextSeq / peekPos 供测试在持锁外读取下一个序号与位置。
func (d *fakeDrive) peekNextSeq(j int) uint32 {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.seq[j]
}

func (d *fakeDrive) peekPos(j int) float64 {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.pos[j]
}
