package trajectory

import (
	"fmt"
	"math"

	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
)

func reject(reason, detail string) error {
	return fault.New(fault.CatCommandReject, fault.SourceTrajectory, reason, detail)
}

func abs(x float64) float64 { return math.Abs(x) }

func formatLimit(name string, v, lo, hi float64) string {
	return fmt.Sprintf("%s=%.6f outside [%.6f, %.6f]", name, v, lo, hi)
}

func formatAccel(got, limit, dt float64) string {
	return fmt.Sprintf("required accel %.3f rad/s^2 over dt=%.4fs exceeds limit %.3f", got, dt, limit)
}
