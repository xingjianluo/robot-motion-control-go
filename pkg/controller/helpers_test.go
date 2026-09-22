package controller_test

import "encoding/binary"

func floatFromPayload(p []byte) float64 {
	return floatFromBits(binary.LittleEndian.Uint64(p))
}

func floatFromBits(b uint64) float64 {
	return float64frombits(b)
}
