package protocol

import (
	"strings"
	"testing"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
)

func mustFault(t *testing.T, err error, wantReason string) *fault.Error {
	t.Helper()
	fe, ok := fault.As(err)
	if !ok {
		t.Fatalf("want fault error, got %v", err)
	}
	if fe.Reason != wantReason {
		t.Fatalf("want reason %s, got %s", wantReason, fe.Reason)
	}
	return fe
}

func TestRoundTripV1V2(t *testing.T) {
	ts := time.UnixMilli(1234)
	// V1 位置命令
	cmd := MotorCommand{Mode: ModePosition, PositionRad: 0.5, Seq: 7, IssuedAt: ts}
	line, err := EncodeCommand(V1, cmd)
	if err != nil {
		t.Fatal(err)
	}
	f, err := Decode(line)
	if err != nil {
		t.Fatal(err)
	}
	back, err := DecodeCommand(f)
	if err != nil {
		t.Fatal(err)
	}
	if back.PositionRad != 0.5 || back.Seq != 7 || !back.IssuedAt.Equal(ts) {
		t.Fatalf("v1 round trip mismatch: %+v", back)
	}

	// V2 力矩命令
	tau := MotorCommand{Mode: ModeTorque, TorqueNm: -1.25, Seq: 8}
	line2, err := EncodeCommand(V2, tau)
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(line2, "tau=-1.25") {
		t.Fatalf("v2 torque frame missing tau: %s", line2)
	}
	back2, err := DecodeCommand(mustFrame(t, line2))
	if err != nil || back2.TorqueNm != -1.25 {
		t.Fatalf("v2 torque round trip: %+v err=%v", back2, err)
	}

	// 历史兼容：V1 不得出现力矩模式
	if _, err := EncodeCommand(V1, MotorCommand{Mode: ModeTorque, TorqueNm: 1}); err == nil {
		t.Fatal("v1 torque command must be rejected")
	} else {
		fe := mustFault(t, err, ReasonUnsupportedVer)
		if fe.Category != fault.CatProtocol {
			t.Fatalf("category=%s", fe.Category)
		}
	}
}

func mustFrame(t *testing.T, line string) Frame {
	t.Helper()
	f, err := Decode(line)
	if err != nil {
		t.Fatalf("decode %q: %v", line, err)
	}
	return f
}

func TestStatusV1MissingTorqueIsNotZeroTorque(t *testing.T) {
	line := "1|STA|3|100|en=1;fault=0;pos=0.1;vel=0"
	s, present, err := DecodeStatus(mustFrame(t, line))
	if err != nil {
		t.Fatal(err)
	}
	if present {
		t.Fatal("v1 status must report torque absent")
	}
	if !s.Enabled || s.PositionRad != 0.1 || s.Seq != 3 {
		t.Fatalf("v1 status decode wrong: %+v", s)
	}

	// V2 缺 tau => 明确的协议错误，而不是被当零力矩
	lineV2 := "2|STA|3|100|en=1;fault=0;pos=0.1;vel=0"
	if _, _, err := DecodeStatus(mustFrame(t, lineV2)); err == nil {
		t.Fatal("v2 status missing tau must be classified")
	} else {
		mustFault(t, err, ReasonMissingField)
	}

	lineV2ok := "2|STA|4|100|en=0;fault=2;pos=0.1;tau=3.3;vel=0"
	s2, present, err := DecodeStatus(mustFrame(t, lineV2ok))
	if err != nil || !present || s2.TorqueNm != 3.3 || s2.Enabled || s2.FaultCode != 2 {
		t.Fatalf("v2 status decode wrong: %+v present=%v err=%v", s2, present, err)
	}
}

func TestDecodeErrorClassification(t *testing.T) {
	cases := []struct {
		line   string
		reason string
	}{
		{"", ReasonEmptyFrame},
		{"garbage", ReasonMalformedFrame},
		{"1|CMD", ReasonMalformedFrame},
		{"9|STA|1|0|pos=1", ReasonUnknownVersion},
		{"1|WAT|1|0|", ReasonUnknownFrameType},
		{"x|STA|1|0|", ReasonBadFieldValue},
		{"1|STA|a|0|", ReasonBadFieldValue},
		{"1|STA|1|0|badpair", ReasonMalformedFrame},
		{"1|STA|1|0|pos=1;pos=2", ReasonMalformedFrame},
	}
	for _, c := range cases {
		_, err := Decode(c.line)
		if err == nil {
			t.Fatalf("decode(%q) expected error", c.line)
		}
		fe := mustFault(t, err, c.reason)
		if fe.Category != fault.CatProtocol || fe.Source != fault.SourceProtocol {
			t.Fatalf("decode(%q) classification wrong: %+v", c.line, fe)
		}
	}
}

func TestCommandMissingFieldAndDeterministicEncoding(t *testing.T) {
	// 缺模式字段
	if _, err := DecodeCommand(mustFrame(t, "2|CMD|1|0|pos=1")); err == nil {
		t.Fatal("missing mode must error")
	} else {
		mustFault(t, err, ReasonMissingField)
	}
	// 速度模式缺 vel
	if _, err := DecodeCommand(mustFrame(t, "1|CMD|1|0|mode=velocity")); err == nil {
		t.Fatal("missing vel must error")
	} else {
		mustFault(t, err, ReasonMissingField)
	}
	// 字段排序确定：两次编码字节一致
	a, _ := EncodeCommand(V2, MotorCommand{Mode: ModeTorque, TorqueNm: 1, Seq: 1})
	b, _ := EncodeCommand(V2, MotorCommand{Mode: ModeTorque, TorqueNm: 1, Seq: 1})
	if a != b {
		t.Fatalf("nondeterministic encoding:\n%s\n%s", a, b)
	}
}
