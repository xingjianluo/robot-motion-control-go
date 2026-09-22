package clock

import "time"

// Clock 抽象时间来源，测试中可替换为确定性的 Fake 时钟。
type Clock interface {
	Now() time.Time
	Sleep(d time.Duration)
	NewTimer(d time.Duration) Timer
}

// Timer 是可停止/重置的定时器句柄。
type Timer interface {
	C() <-chan time.Time
	Stop() bool
	Reset(d time.Duration) bool
}

type realClock struct{}

// Real 返回基于墙钟的真实时钟。
func Real() Clock { return realClock{} }

func (realClock) Now() time.Time                 { return time.Now() }
func (realClock) Sleep(d time.Duration)          { time.Sleep(d) }
func (realClock) NewTimer(d time.Duration) Timer { return realTimer{time.NewTimer(d)} }

type realTimer struct{ t *time.Timer }

func (r realTimer) C() <-chan time.Time        { return r.t.C }
func (r realTimer) Stop() bool                 { return r.t.Stop() }
func (r realTimer) Reset(d time.Duration) bool { return r.t.Reset(d) }
