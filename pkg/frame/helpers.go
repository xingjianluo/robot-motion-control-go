package frame

import "math"

func floatBits(f float64) uint64     { return math.Float64bits(f) }
func floatFromBits(b uint64) float64 { return math.Float64frombits(b) }
