package feedback

import (
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
	"github.com/xingjianluo/robot-motion-control-go/internal/protocol"
)

func statusLine(v protocol.Version, seq uint64, tsMs int64, pos, vel, tau float64, en bool) string {
	s := protocol.DriveStatus{PositionRad: pos, VelocityRadS: vel, TorqueNm: tau, Seq: seq, TsMs: tsMs, Enabled: en}
	line, err := protocol.EncodeStatus(v, s)
	if err != nil {
		panic(err)
	}
	return line
}

func newTestConsumer() *Consumer {
	return New(Config{WatchdogTimeout: 30 * time.Millisecond, MaxStepRad: 0.5, MaxBackdateMs: 5})
}

func feed(t *testing.T, c *Consumer, line string, at time.Time) (Sample, Classification) {
	t.Helper()
	s, class, err := c.Ingest(line, at, nil)
	if err != nil {
		t.Fatalf("unexpected ingest error: %v", err)
	}
	return s, class
}

func TestAcceptedAndTrackingBaseline(t *testing.T) {
	c := newTestConsumer()
	t0 := time.Unix(0, 0)
	feed(t, c, statusLine(protocol.V2, 1, 0, 0.1, 0, 0, true), t0)
	s, class := feed(t, c, statusLine(protocol.V2, 2, 10, 0.2, 0, 0, true), t0.Add(10*time.Millisecond))
	if class != ClassAccepted || s.Seq != 2 || !s.TorqueKnown {
		t.Fatalf("accept wrong: %+v %s", s, class)
	}
	last, ok := c.Last()
	if !ok || last.PositionRad != 0.2 {
		t.Fatalf("last baseline wrong: %+v %v", last, ok)
	}
}

func TestDuplicateAndOutOfOrder(t *testing.T) {
	c := newTestConsumer()
	t0 := time.Unix(0, 0)
	feed(t, c, statusLine(protocol.V2, 5, 100, 0.1, 0, 0, true), t0)

	_, _, err := c.Ingest(statusLine(protocol.V2, 5, 100, 0.1, 0, 0, true), t0.Add(5*time.Millisecond), nil)
	if fe, ok := fault.As(err); !ok || fe.Reason != ReasonDuplicateSeq || fe.Category != fault.CatComm {
		t.Fatalf("dup classify: %v", err)
	}
	_, _, err = c.Ingest(statusLine(protocol.V2, 4, 90, 0.1, 0, 0, true), t0.Add(6*time.Millisecond), nil)
	if fe, ok := fault.As(err); !ok || fe.Reason != ReasonOutOfOrder {
		t.Fatalf("seq rollback classify: %v", err)
	}
	// 时间戳大幅回退但 seq 正常增长 => 仍归类乱序
	_, _, err = c.Ingest(statusLine(protocol.V2, 6, 90, 0.12, 0, 0, true), t0.Add(7*time.Millisecond), nil)
	if fe, ok := fault.As(err); !ok || fe.Reason != ReasonOutOfOrder {
		t.Fatalf("timestamp backdate classify: %v", err)
	}
	// 异常帧不得污染基准：last 仍是 seq 5
	if last, _ := c.Last(); last.Seq != 5 {
		t.Fatalf("baseline polluted: %d", last.Seq)
	}
}

func TestPositionJumpIsNotTrackingError(t *testing.T) {
	c := newTestConsumer()
	t0 := time.Unix(0, 0)
	feed(t, c, statusLine(protocol.V2, 1, 0, 0.0, 0, 0, true), t0)
	_, class, err := c.Ingest(statusLine(protocol.V2, 2, 10, 5.0, 0, 0, true), t0.Add(10*time.Millisecond), nil)
	if fe, ok := fault.As(err); !ok || fe.Reason != ReasonPositionJump || class != ClassJump {
		t.Fatalf("jump classify: class=%s err=%v", class, err)
	}
	// 基准不被跳变样本更新：接着 seq3 相对 seq1 的增量正常即可接受
	_, class, err = c.Ingest(statusLine(protocol.V2, 3, 20, 0.05, 0, 0, true), t0.Add(20*time.Millisecond), nil)
	if err != nil || class != ClassAccepted {
		t.Fatalf("valid frame after jump should accept: %v", err)
	}
}

func TestWatchdogDistinguishesLossFromDeviation(t *testing.T) {
	c := newTestConsumer()
	t0 := time.Unix(0, 0)
	// 从未收到帧 => 看门狗超时
	err0 := c.WatchdogCheck(t0)
	if fe, ok := fault.As(err0); !ok || fe.Reason != ReasonWatchdogTimeout {
		t.Fatalf("initial watchdog: %v", err0)
	}
	feed(t, c, statusLine(protocol.V2, 1, 0, 0.1, 0, 0, true), t0)
	if err := c.WatchdogCheck(t0.Add(20 * time.Millisecond)); err != nil {
		t.Fatalf("within window must be healthy: %v", err)
	}
	// 超时：暂时失联（CatComm），不是跟踪误差
	err := c.WatchdogCheck(t0.Add(31 * time.Millisecond))
	fe, ok := fault.As(err)
	if !ok || fe.Reason != ReasonWatchdogTimeout || fe.Category != fault.CatComm {
		t.Fatalf("watchdog expiry: %v", err)
	}
	// 持续失联给出稳定的“仍失联”结论
	err = c.WatchdogCheck(t0.Add(40 * time.Millisecond))
	if fe, _ = fault.As(err); fe.Reason != ReasonLostLink {
		t.Fatalf("persistent loss: %v", err)
	}
	// 一帧正常反馈到达即清除失联
	feed(t, c, statusLine(protocol.V2, 2, 50, 0.11, 0, 0, true), t0.Add(40*time.Millisecond))
	if c.IsLost() || c.WatchdogCheck(t0.Add(40*time.Millisecond)) != nil {
		t.Fatal("link must be healthy after valid frame")
	}
}

func TestProtocolErrorNotConsumedAsFeedback(t *testing.T) {
	c := newTestConsumer()
	t0 := time.Unix(0, 0)
	// 线路损坏帧：协议错误必须保持 CatProtocol，不能被当跟踪/失联
	_, _, err := c.Ingest("2|STA|1|0|pos=0.1<<<CORRUPT", t0, nil)
	fe, ok := fault.As(err)
	if !ok || fe.Category != fault.CatProtocol {
		t.Fatalf("corrupt frame category: %v", err)
	}
	// V1 缺力矩字段在 V1 是合法帧（兼容），不应报错
	_, _, err = c.Ingest("1|STA|1|0|pos=0.1;vel=0", t0, nil)
	if err != nil {
		t.Fatalf("v1 frame without tau must remain compatible: %v", err)
	}
}
