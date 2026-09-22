// Package frame 定义本地总线帧结构与多版本协议编解码。
// 未知帧、版本不支持、字段缺失必须被明确归类，不得静默忽略。
//
// 线缆格式（小端）：
//
//	0xA5 | Version(1) | Type(1) | Joint(2) | PayloadLen(4) | Payload... | XOR校验(1)
//
// V1 反馈 Payload(24B): Seq(u32) Timestamp(i64) Position(f64) Velocity(f64)
// V2 反馈 Payload(33B): V1 字段 + Torque(f64) + Status(1, bit0=驱动器故障)
package frame

import (
	"encoding/binary"

	myerr "github.com/xingjianluo/robot-motion-control-go/pkg/errors"
)

const sof = 0xA5

// Version 协议版本。
type Version uint8

const (
	V1 Version = 1 // 历史版本：仅位置/速度反馈与位置/速度命令
	V2 Version = 2 // 当前版本：增加力矩命令/反馈与状态字
)

// SupportedVersions 本实现支持的协议版本集合。历史版本 V1 行为长期保持兼容。
var SupportedVersions = []Version{V1, V2}

// Type 帧类型。
type Type uint8

const (
	TypeCmdPosition Type = 0x01
	TypeCmdVelocity Type = 0x02
	TypeCmdTorque   Type = 0x03 // 仅 V2
	TypeFeedback    Type = 0x10
	TypeStatus      Type = 0x11 // 仅 V2
	TypeEStop       Type = 0x7F
)

// Frame 总线帧。Version 决定 Payload 的编解码方式。
type Frame struct {
	Version Version
	Type    Type
	Joint   int
	Payload []byte
}

// FeedbackData 解码后的反馈内容（V1/V2 公共视图）。
type FeedbackData struct {
	Joint     int
	Seq       uint32 // 单调递增序号，用于乱序/重复检测
	Timestamp int64  // 发送方时间戳（纳秒）
	Position  float64
	Velocity  float64
	Torque    float64 // 仅 V2 有效
	HasTorque bool
	Fault     bool // 驱动器故障位（V1 无此字段，恒为 false）
}

func isSupportedVersion(v Version) bool {
	for _, sv := range SupportedVersions {
		if sv == v {
			return true
		}
	}
	return false
}

// knownType 报告某版本是否认识该帧类型；未知类型不得被静默忽略。
func knownType(v Version, t Type) bool {
	switch t {
	case TypeCmdPosition, TypeCmdVelocity, TypeFeedback, TypeEStop:
		return true
	case TypeCmdTorque, TypeStatus:
		return v == V2
	default:
		return false
	}
}

// Encode 将帧编码为字节流。版本不受支持或类型与版本不匹配时返回协议错误。
func Encode(f Frame) ([]byte, error) {
	if !isSupportedVersion(f.Version) {
		return nil, myerr.New(myerr.CatProtocol, "UNSUPPORTED_VERSION",
			"协议版本 "+itoa(int(f.Version))+" 不受支持")
	}
	if !knownType(f.Version, f.Type) {
		return nil, myerr.New(myerr.CatProtocol, "UNKNOWN_FRAME_TYPE",
			"帧类型 0x"+hex2(uint8(f.Type))+" 在 V"+itoa(int(f.Version))+" 中未知或不受支持")
	}
	out := make([]byte, 0, 9+len(f.Payload)+1)
	out = append(out, sof, byte(f.Version), byte(f.Type))
	out = binary.LittleEndian.AppendUint16(out, uint16(f.Joint))
	out = binary.LittleEndian.AppendUint32(out, uint32(len(f.Payload)))
	out = append(out, f.Payload...)
	var chk byte
	for _, b := range out {
		chk ^= b
	}
	out = append(out, chk)
	return out, nil
}

// Decode 将字节流解码为帧。
// 帧损坏、版本不支持、未知帧类型分别返回不同原因码的 PROTOCOL 错误。
func Decode(raw []byte) (Frame, error) {
	if len(raw) < 9 {
		return Frame{}, myerr.New(myerr.CatProtocol, "FRAME_TRUNCATED",
			"帧长度不足帧头要求: "+itoa(len(raw))+"B")
	}
	if raw[0] != sof {
		return Frame{}, myerr.New(myerr.CatProtocol, "FRAME_CORRUPT", "起始字节错误")
	}
	// 校验和位于最后一个字节
	var chk byte
	for _, b := range raw[:len(raw)-1] {
		chk ^= b
	}
	if chk != raw[len(raw)-1] {
		return Frame{}, myerr.New(myerr.CatProtocol, "FRAME_CORRUPT", "校验和不匹配")
	}
	v := Version(raw[1])
	t := Type(raw[2])
	joint := int(binary.LittleEndian.Uint16(raw[3:5]))
	plen := int(binary.LittleEndian.Uint32(raw[5:9]))
	if 9+plen+1 != len(raw) {
		return Frame{}, myerr.New(myerr.CatProtocol, "FRAME_TRUNCATED",
			"声明负载长度 "+itoa(plen)+" 与实际不符")
	}
	if !isSupportedVersion(v) {
		return Frame{}, myerr.New(myerr.CatProtocol, "UNSUPPORTED_VERSION",
			"协议版本 "+itoa(int(v))+" 不受支持")
	}
	if !knownType(v, t) {
		return Frame{}, myerr.New(myerr.CatProtocol, "UNKNOWN_FRAME_TYPE",
			"帧类型 0x"+hex2(uint8(t))+" 在 V"+itoa(int(v))+" 中未知或不受支持")
	}
	pl := make([]byte, plen)
	copy(pl, raw[9:9+plen])
	return Frame{Version: v, Type: t, Joint: joint, Payload: pl}, nil
}

const (
	v1FeedbackLen = 28 // u32(4)+i64(8)+f64(8)+f64(8)
	v2FeedbackLen = 37 // + f64(8) + u8 状态字(1)
)

// EncodeFeedback 按版本编码反馈帧：V1 裁剪力矩/状态字，V2 保留全部字段。
func EncodeFeedback(v Version, fb FeedbackData) (Frame, error) {
	var pl []byte
	pl = binary.LittleEndian.AppendUint32(pl, fb.Seq)
	pl = binary.LittleEndian.AppendUint64(pl, uint64(fb.Timestamp))
	pl = binary.LittleEndian.AppendUint64(pl, floatBits(fb.Position))
	pl = binary.LittleEndian.AppendUint64(pl, floatBits(fb.Velocity))
	if v == V2 {
		pl = binary.LittleEndian.AppendUint64(pl, floatBits(fb.Torque))
		var status byte
		if fb.Fault {
			status = 0x01
		}
		pl = append(pl, status)
	}
	f := Frame{Version: v, Type: TypeFeedback, Joint: fb.Joint, Payload: pl}
	raw, err := Encode(f)
	if err != nil {
		return Frame{}, err
	}
	// Encode 已校验版本；重新解析出最终帧
	dec, err := Decode(raw)
	return dec, err
}

// DecodeFeedback 解码反馈帧负载。
// 负载长度不符（字段缺失/粘连）按 FIELD_MISSING 或 PAYLOAD_MALFORMED 明确归类。
func DecodeFeedback(f Frame) (FeedbackData, error) {
	if f.Type != TypeFeedback {
		return FeedbackData{}, myerr.New(myerr.CatProtocol, "FRAME_TYPE_MISMATCH",
			"期望反馈帧(0x10)，实际 0x"+hex2(uint8(f.Type)))
	}
	want := v1FeedbackLen
	if f.Version == V2 {
		want = v2FeedbackLen
	}
	if len(f.Payload) < want {
		return FeedbackData{}, myerr.New(myerr.CatProtocol, "FIELD_MISSING",
			"V"+itoa(int(f.Version))+" 反馈需要 "+itoa(want)+"B，实际 "+itoa(len(f.Payload))+"B")
	}
	if len(f.Payload) > want {
		return FeedbackData{}, myerr.New(myerr.CatProtocol, "PAYLOAD_MALFORMED",
			"V"+itoa(int(f.Version))+" 反馈负载过长: "+itoa(len(f.Payload))+"B > "+itoa(want)+"B")
	}
	fb := FeedbackData{
		Joint:     f.Joint,
		Seq:       binary.LittleEndian.Uint32(f.Payload[0:4]),
		Timestamp: int64(binary.LittleEndian.Uint64(f.Payload[4:12])),
		Position:  floatFromBits(binary.LittleEndian.Uint64(f.Payload[12:20])),
		Velocity:  floatFromBits(binary.LittleEndian.Uint64(f.Payload[20:28])),
	}
	if f.Version == V2 {
		fb.Torque = floatFromBits(binary.LittleEndian.Uint64(f.Payload[28:36]))
		fb.HasTorque = true
		fb.Fault = f.Payload[36]&0x01 != 0
	}
	return fb, nil
}

// EncodeCommand 将位置/速度/力矩命令编码为对应帧。
func EncodeCommand(v Version, typ Type, joint int, value float64) (Frame, error) {
	if typ != TypeCmdPosition && typ != TypeCmdVelocity && typ != TypeCmdTorque {
		return Frame{}, myerr.New(myerr.CatProtocol, "UNKNOWN_FRAME_TYPE",
			"0x"+hex2(uint8(typ))+" 不是命令帧类型")
	}
	if typ == TypeCmdTorque && v == V1 {
		return Frame{}, myerr.New(myerr.CatProtocol, "CMD_UNSUPPORTED_VERSION",
			"力矩命令需要 V2 驱动器，V1 不支持")
	}
	f := Frame{Version: v, Type: typ, Joint: joint,
		Payload: binary.LittleEndian.AppendUint64(nil, floatBits(value))}
	raw, err := Encode(f)
	if err != nil {
		return Frame{}, err
	}
	return Decode(raw)
}

func itoa(n int) string {
	if n == 0 {
		return "0"
	}
	neg := n < 0
	if neg {
		n = -n
	}
	var b [20]byte
	i := len(b)
	for n > 0 {
		i--
		b[i] = byte('0' + n%10)
		n /= 10
	}
	if neg {
		i--
		b[i] = '-'
	}
	return string(b[i:])
}

func hex2(b byte) string {
	const h = "0123456789ABCDEF"
	return string([]byte{h[b>>4], h[b&0x0F]})
}
