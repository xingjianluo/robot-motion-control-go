package protocol

import (
	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
)

// DriveStatus 是解码后的驱动器/编码器反馈（版本无关内部表示）。
type DriveStatus struct {
	PositionRad  float64
	VelocityRadS float64
	TorqueNm     float64
	Enabled      bool
	FaultCode    int
	Seq          uint64
	TsMs         int64
}

// 状态帧专有字段名（pos/vel/tau 见 frame.go 共享定义）。
const (
	FieldEnabled = "en"
	FieldFault   = "fault"
)

// EncodeStatus 按版本编码状态帧：
// V1 仅输出 pos/vel/en/fault（保持历史兼容，绝不增加力矩）；
// V2 额外输出 tau。
func EncodeStatus(v Version, s DriveStatus) (string, error) {
	fields := map[string]string{
		FieldPos:     formatFloat(s.PositionRad),
		FieldVel:     formatFloat(s.VelocityRadS),
		FieldEnabled: boolStr(s.Enabled),
		FieldFault:   itoa(s.FaultCode),
	}
	if v >= V2 {
		fields[FieldTau] = formatFloat(s.TorqueNm)
	}
	return Encode(Frame{Version: v, Type: FrameStatus, Seq: s.Seq, TsMs: s.TsMs, Fields: fields})
}

// DecodeStatus 把已解析的通用帧按其版本解释为驱动器状态。
//
// 兼容约定（可复现）：
//   - pos/vel 为所有版本必填，缺失 => protocol_error/missing_field；
//   - tau 仅 V2+ 存在；V1 帧缺失 tau 时 torquePresent=false，
//     调用方必须区分“字段缺失”与“值为 0”，不得把缺失当零力矩；
//   - en/fault 缺失时按 V1 历史默认（en=1, fault=0）解释，
//     但 V2 帧缺失 en 属于字段缺失（新版本要求显式上报）。
func DecodeStatus(f Frame) (s DriveStatus, torquePresent bool, err error) {
	if f.Type != FrameStatus {
		return DriveStatus{}, false, fault.New(fault.CatProtocol, fault.SourceProtocol,
			ReasonUnknownFrameType, "expected STA frame, got "+string(f.Type))
	}
	s.Seq, s.TsMs = f.Seq, f.TsMs

	pos, present, perr := parseFloatField(f, FieldPos)
	if perr != nil {
		return DriveStatus{}, false, perr
	}
	if !present {
		return DriveStatus{}, false, fault.New(fault.CatProtocol, fault.SourceProtocol,
			ReasonMissingField, FieldPos)
	}
	vel, present, verr := parseFloatField(f, FieldVel)
	if verr != nil {
		return DriveStatus{}, false, verr
	}
	if !present {
		return DriveStatus{}, false, fault.New(fault.CatProtocol, fault.SourceProtocol,
			ReasonMissingField, FieldVel)
	}
	s.PositionRad, s.VelocityRadS = pos, vel

	if tau, present, terr := parseFloatField(f, FieldTau); terr != nil {
		return DriveStatus{}, false, terr
	} else if present {
		s.TorqueNm, torquePresent = tau, true
	} else if f.Version >= V2 {
		// V2 要求显式力矩；缺失是协议错误。
		return DriveStatus{}, false, fault.New(fault.CatProtocol, fault.SourceProtocol,
			ReasonMissingField, FieldTau)
	}

	if raw, ok := f.Get(FieldEnabled); ok {
		switch raw {
		case "1":
			s.Enabled = true
		case "0":
			s.Enabled = false
		default:
			return DriveStatus{}, false, fault.New(fault.CatProtocol, fault.SourceProtocol,
				ReasonBadFieldValue, FieldEnabled+"="+raw)
		}
	} else {
		if f.Version >= V2 {
			return DriveStatus{}, false, fault.New(fault.CatProtocol, fault.SourceProtocol,
				ReasonMissingField, FieldEnabled)
		}
		s.Enabled = true // V1 历史默认
	}

	if fc, ok, ferr := parseIntField(f, FieldFault); ferr != nil {
		return DriveStatus{}, false, ferr
	} else if ok {
		s.FaultCode = int(fc)
	}
	return s, torquePresent, nil
}
