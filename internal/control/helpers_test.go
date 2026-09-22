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
