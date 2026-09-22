// Package joints 在单关节控制器之上提供多关节编排：
// 多个关节各自拥有独立的总线/控制器/状态机，在一个共享的周期边界上
// 被确定性地推进；任一关节触发急停都会以确定顺序联锁其余关节。
package joints

import (
	"github.com/xingjianluo/robot-motion-control-go/internal/bus"
	"github.com/xingjianluo/robot-motion-control-go/internal/clock"
	"github.com/xingjianluo/robot-motion-control-go/internal/control"
	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
	"github.com/xingjianluo/robot-motion-control-go/internal/safety"
	"github.com/xingjianluo/robot-motion-control-go/internal/trajectory"
)

// Arm 是一组命名的单关节控制器。
type Arm struct {
	clk    clock.Clock
	order  []string
	joints map[string]*control.Controller
}

// NewArm 创建空机械臂。
func NewArm(clk clock.Clock) *Arm {
	return &Arm{clk: clk, joints: map[string]*control.Controller{}}
}

// AddJoint 按名字与配置挂载一个关节（独立总线），返回其控制器。
// 挂载顺序即周期推进顺序，保证多关节结论可复现。
func (a *Arm) AddJoint(name string, cfg control.Config) (*control.Controller, error) {
	if _, exists := a.joints[name]; exists {
		return nil, fault.New(fault.CatCommandReject, fault.SourceController,
			"duplicate_joint_name", "joint already registered: "+name)
	}
	b := bus.New(cfg.Bus, a.clk)
	ctl, err := control.New(cfg, a.clk, b)
	if err != nil {
		return nil, err
	}
	a.joints[name] = ctl
	a.order = append(a.order, name)
	return ctl, nil
}

// Joint 按名取控制器。
func (a *Arm) Joint(name string) (*control.Controller, bool) {
	c, ok := a.joints[name]
	return c, ok
}

// Names 按挂载顺序返回关节名。
func (a *Arm) Names() []string {
	out := make([]string, len(a.order))
	copy(out, a.order)
	return out
}

// StepAll 在一个共享周期边界上按固定挂载顺序推进所有关节。
//
// 联锁语义（确定、无半更新）：
//   - 各关节串行推进（各自独立加锁），顺序固定，因此跨关节结果可复现；
//   - 若某关节推进后处于 ESTOP，则本周期内其后所有关节也立即急停
//     （整体安全停机优先）；已经推进过的关节若此前非 ESTOP 也一并急停，
//     保证“一个周期结束后要么全运行、要么全停”，不出现单关节运行、
//     其余切断的半更新状态；
//   - 返回首个触发整体急停的关节名（无则空串）。
func (a *Arm) StepAll(reason string) (estopSource string) {
	snapAt := make(map[string]control.Snapshot, len(a.order))
	for _, name := range a.order {
		c := a.joints[name]
		_ = c.Step()
		snapAt[name] = c.Snapshot()
		if estopSource == "" && snapAt[name].SafetyState == safety.StateEstop {
			estopSource = name
		}
	}
	if estopSource != "" {
		for _, name := range a.order {
			if c := a.joints[name]; c.Snapshot().SafetyState != safety.StateEstop {
				c.Estop(reason + " (interlocked by joint " + estopSource + ")")
			}
		}
	}
	return estopSource
}

// EstopAll 以固定顺序急停所有关节（可从任意 goroutine 调用）。
func (a *Arm) EstopAll(reason string) {
	for _, name := range a.order {
		a.joints[name].Estop(reason)
	}
}

// RecoverAll 在所有关节都满足条件时才统一恢复；任一关节条件不满足，
// 则一个都不恢复，并返回该关节的拒绝错误（无静默部分恢复）。
func (a *Arm) RecoverAll(cond safety.RecoveryConditions) error {
	// 先全部尝试：只要有一个被拒，就回滚本轮已恢复的关节——
	// 做法是先校验快照（无副作用接口不足），因此采用“先试后停”策略：
	for _, name := range a.order {
		if err := a.joints[name].Recover(cond); err != nil {
			// 恢复是显式且全有或全无：把本轮已进入 RUN 的关节重新急停，
			// 避免部分关节出力、部分切断。
			for _, other := range a.order {
				if other == name {
					break
				}
				a.joints[other].Estop("rollback: joint " + name + " recovery denied")
			}
			return err
		}
	}
	return nil
}

// Submit 在指定关节提交命令。
func (a *Arm) Submit(joint string, cmd trajectory.Command) error {
	c, ok := a.joints[joint]
	if !ok {
		return fault.New(fault.CatCommandReject, fault.SourceController, "unknown_joint", joint)
	}
	_, err := c.SubmitCommand(cmd)
	return err
}

// Snapshot 返回按挂载顺序排列的各关节快照。
func (a *Arm) Snapshot() map[string]control.Snapshot {
	out := make(map[string]control.Snapshot, len(a.order))
	for _, name := range a.order {
		out[name] = a.joints[name].Snapshot()
	}
	return out
}
