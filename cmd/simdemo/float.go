package main

import (
	"encoding/binary"
	"math"
)

func floatFromPayload(p []byte) float64 {
	return math.Float64frombits(binary.LittleEndian.Uint64(p))
}
