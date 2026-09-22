// Package clock 提供可替换的时钟抽象，测试中使用确定性手动时钟。
package clock

import "time"

// Clock 时钟抽象。
type Clock interface {
	Now() time.Time
}

// Real 真实时钟。
type Real struct{}

func (Real) Now() time.Time { return time.Now() }

// Manual 确定性手动时钟，仅在被显式推进时变化。
type Manual struct {
	now time.Time
}

func NewManual(start time.Time) *Manual { return &Manual{now: start} }

func (m *Manual) Now() time.Time { return m.now }

// Advance 推进时钟。
func (m *Manual) Advance(d time.Duration) { m.now = m.now.Add(d) }
