package frame_test

import (
	"testing"

	myerr "github.com/xingjianluo/robot-motion-control-go/pkg/errors"
	"github.com/xingjianluo/robot-motion-control-go/pkg/frame"
)

func mustCode(t *testing.T, err error, wantCode string) {
	t.Helper()
	if err == nil {
		t.Fatalf("期望错误 %s，实际 nil", wantCode)
	}
	e, ok := err.(*myerr.Error)
	if !ok {
		t.Fatalf("非分类错误: %v", err)
	}
	if e.Code != wantCode {
		t.Fatalf("错误码 = %s，期望 %s（%v）", e.Code, wantCode, err)
	}
}

// TestV1BackwardCompat V1 历史版本行为：力矩/状态字段被裁剪，
// 编解码结果固定且可复现。
func TestV1BackwardCompat(t *testing.T) {
	fb := frame.FeedbackData{Joint: 2, Seq: 42, Timestamp: 123456789,
		Position: 0.5, Velocity: -0.25, Torque: 9.9, Fault: true}
	f, err := frame.EncodeFeedback(frame.V1, fb)
	if err != nil {
		t.Fatal(err)
	}
	if len(f.Payload) != 28 {
		t.Fatalf("V1 负载应为 28B，实际 %d", len(f.Payload))
	}
	got, err := frame.DecodeFeedback(f)
	if err != nil {
		t.Fatal(err)
	}
	if got.HasTorque || got.Fault {
		t.Fatalf("V1 不应包含力矩/故障字段: %+v", got)
	}
	if got.Position != 0.5 || got.Velocity != -0.25 || got.Seq != 42 {
		t.Fatalf("V1 字段不兼容: %+v", got)
	}
	// 线缆字节可复现
	raw1, _ := frame.Encode(f)
	raw2, _ := frame.Encode(f)
	if string(raw1) != string(raw2) {
		t.Fatalf("V1 编码不确定")
	}
	t.Logf("输入 V2 视图(torque=9.9,fault=true) -> V1 裁剪后 %d 字节；结论可复现",
		len(raw1))
}

func TestV2RoundTrip(t *testing.T) {
	fb := frame.FeedbackData{Joint: 1, Seq: 7, Timestamp: 99,
		Position: 1.25, Velocity: 0.5, Torque: -3.5, Fault: true}
	f, err := frame.EncodeFeedback(frame.V2, fb)
	if err != nil {
		t.Fatal(err)
	}
	got, err := frame.DecodeFeedback(f)
	if err != nil {
		t.Fatal(err)
	}
	if !got.HasTorque || got.Torque != -3.5 || !got.Fault {
		t.Fatalf("V2 字段丢失: %+v", got)
	}
}

func TestUnsupportedVersion(t *testing.T) {
	_, err := frame.Encode(frame.Frame{Version: 9, Type: frame.TypeFeedback})
	mustCode(t, err, "UNSUPPORTED_VERSION")
	if !myerr.IsCategory(err, myerr.CatProtocol) {
		t.Fatal("版本错误必须归为 PROTOCOL")
	}
}

func TestUnknownFrameType(t *testing.T) {
	_, err := frame.Encode(frame.Frame{Version: frame.V2, Type: 0x66})
	mustCode(t, err, "UNKNOWN_FRAME_TYPE")
	// V1 出现 V2 专属类型也算未知
	_, err = frame.Encode(frame.Frame{Version: frame.V1, Type: frame.TypeCmdTorque})
	mustCode(t, err, "UNKNOWN_FRAME_TYPE")
}

func TestV1RejectsTorqueCommand(t *testing.T) {
	_, err := frame.EncodeCommand(frame.V1, frame.TypeCmdTorque, 0, 1.0)
	mustCode(t, err, "CMD_UNSUPPORTED_VERSION")
}

func TestCorruptAndTruncated(t *testing.T) {
	good, _ := frame.EncodeFeedback(frame.V2,
		frame.FeedbackData{Joint: 0, Seq: 1, Timestamp: 1})
	raw, _ := frame.Encode(good)

	// 截断
	_, err := frame.Decode(raw[:5])
	mustCode(t, err, "FRAME_TRUNCATED")

	// 校验和损坏
	bad := make([]byte, len(raw))
	copy(bad, raw)
	bad[len(bad)-1] ^= 0xFF
	_, err = frame.Decode(bad)
	mustCode(t, err, "FRAME_CORRUPT")

	// 起始字节损坏
	bad2 := make([]byte, len(raw))
	copy(bad2, raw)
	bad2[0] = 0x5A
	_, err = frame.Decode(bad2)
	mustCode(t, err, "FRAME_CORRUPT")
}

func TestFieldMissing(t *testing.T) {
	// V2 帧但只带 V1 长度负载 -> 字段缺失
	f := frame.Frame{Version: frame.V2, Type: frame.TypeFeedback, Joint: 0,
		Payload: make([]byte, 28)}
	_, err := frame.DecodeFeedback(f)
	mustCode(t, err, "FIELD_MISSING")
}
