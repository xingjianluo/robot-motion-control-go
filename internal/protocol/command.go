package protocol

import (
	"fmt"
	"strconv"
	"time"

	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
)

// MotorCommand 是解码后的运动命令（与协议版本无关的内部表示）。
type MotorCommand struct {
	Mode         CommandMode
	PositionRad  float64
	VelocityRadS float64
	TorqueNm     float64
	Seq          uint64
	IssuedAt     time.Time
}

// CommandMode 命令模式。
type CommandMode string

const (
	ModePosition CommandMode = "position"
	ModeVelocity CommandMode = "velocity"
	ModeTorque   CommandMode = "torque"
)

// 命令帧字段名（pos/vel/tau 见 frame.go 共享定义）。
const FieldMode = "mode"

// EncodeCommand 按指定版本编码命令帧。
//
// 版本兼容规则（历史行为必须保持不变）：
//   - V1：只有位置/速度两种模式，命令帧不含力矩字段；
//     在 V1 上请求力矩模式 => fault(ReasonUnsupportedVer) 而非静默改写。
//   - V2：新增力矩模式与 tau 字段。
//
// 自相矛盾（未知模式、模式与字段不一致）在本层不会发生——本层只负责
// 忠实编码；限位/连续性拒绝由 trajectory 层给出。
func EncodeCommand(v Version, c MotorCommand) (string, error) {
	switch c.Mode {
	case ModePosition, ModeVelocity:
	case ModeTorque:
		if v < V2 {
			return "", fault.New(fault.CatProtocol, fault.SourceProtocol, ReasonUnsupportedVer,
				fmt.Sprintf("torque mode requires protocol v2+, frame requested v%d", v))
		}
	default:
		return "", fault.New(fault.CatProtocol, fault.SourceProtocol, ReasonBadFieldValue,
			"unknown command mode: "+string(c.Mode))
	}
	fields := map[string]string{FieldMode: string(c.Mode)}
	switch c.Mode {
	case ModePosition:
		fields[FieldPos] = strconv.FormatFloat(c.PositionRad, 'f', -1, 64)
	case ModeVelocity:
		fields[FieldVel] = strconv.FormatFloat(c.VelocityRadS, 'f', -1, 64)
	case ModeTorque:
		fields[FieldTau] = strconv.FormatFloat(c.TorqueNm, 'f', -1, 64)
	}
	tsMs := int64(0)
	if !c.IssuedAt.IsZero() {
		tsMs = c.IssuedAt.UnixMilli()
	}
	return Encode(Frame{Version: v, Type: FrameCommand, Seq: c.Seq, TsMs: tsMs, Fields: fields})
}

// DecodeCommand 把通用帧解释为内部命令；
// 缺失模式字段、模式与字段不匹配都返回带原因的 fault。
func DecodeCommand(f Frame) (MotorCommand, error) {
	if f.Type != FrameCommand {
		return MotorCommand{}, fault.New(fault.CatProtocol, fault.SourceProtocol, ReasonUnknownFrameType,
			"expected CMD frame, got "+string(f.Type))
	}
	modeRaw, ok := f.Get(FieldMode)
	if !ok {
		return MotorCommand{}, fault.New(fault.CatProtocol, fault.SourceProtocol, ReasonMissingField, FieldMode)
	}
	mode := CommandMode(modeRaw)
	switch mode {
	case ModePosition, ModeVelocity:
	case ModeTorque:
		if f.Version < V2 {
			return MotorCommand{}, fault.New(fault.CatProtocol, fault.SourceProtocol, ReasonUnsupportedVer,
				"torque mode on v1 frame")
		}
	default:
		return MotorCommand{}, fault.New(fault.CatProtocol, fault.SourceProtocol, ReasonBadFieldValue,
			"unknown mode: "+modeRaw)
	}
	cmd := MotorCommand{Mode: mode, Seq: f.Seq}
	if f.TsMs > 0 {
		cmd.IssuedAt = time.UnixMilli(f.TsMs)
	}
	// 每个模式都必须携带其专有字段，缺失即协议错误。
	required := map[CommandMode]string{
		ModePosition: FieldPos, ModeVelocity: FieldVel, ModeTorque: FieldTau,
	}[mode]
	v, present, err := parseFloatField(f, required)
	if err != nil {
		return MotorCommand{}, err
	}
	if !present {
		return MotorCommand{}, fault.New(fault.CatProtocol, fault.SourceProtocol, ReasonMissingField, required)
	}
	switch mode {
	case ModePosition:
		cmd.PositionRad = v
	case ModeVelocity:
		cmd.VelocityRadS = v
	case ModeTorque:
		cmd.TorqueNm = v
	}
	return cmd, nil
}
