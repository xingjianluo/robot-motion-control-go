package stream

import (
	"fmt"
	"time"
)

// period 返回控制周期；Config.Period 未设置（<=0）时按 10ms 处理，
// 与控制层默认节拍一致。
func (cfg Config) period() time.Duration {
	if cfg.Period > 0 {
		return cfg.Period
	}
	return 10 * time.Millisecond
}

func fmtLimit(v, lo, hi float64) string {
	return fmt.Sprintf("value %.6f outside [%.6f, %.6f]", v, lo, hi)
}

func fmtScaled(raw, scale, scaled, limit float64) string {
	return fmt.Sprintf("raw %.6f * override %.3f = %.6f exceeds limit %.6f", raw, scale, scaled, limit)
}
