package control_test

import (
	"strconv"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/control"
	"github.com/xingjianluo/robot-motion-control-go/internal/feedback"
	"github.com/xingjianluo/robot-motion-control-go/internal/report"
)

func feedbackWatchdogReason() string { return feedback.ReasonWatchdogTimeout }

func itoa64(v int64) string { return strconv.FormatInt(v, 10) }

var _ = time.Second

func formatSnap(s control.Snapshot) string { return report.FmtSnapshot(s) }

// drvVel 读取测试设备当前速度（越限倍率测试用）。
func (r *rig) drvVel() float64 { return r.drv.Velocity() }
