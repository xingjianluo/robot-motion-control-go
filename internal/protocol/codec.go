package protocol

import (
	"fmt"
	"sort"
	"strconv"
	"strings"

	"github.com/xingjianluo/robot-motion-control-go/internal/fault"
)

// Reason 码：协议错误使用稳定原因码，禁止静默忽略。
const (
	ReasonEmptyFrame       = "empty_frame"
	ReasonMalformedFrame   = "malformed_frame"
	ReasonUnknownVersion   = "unknown_version"
	ReasonUnsupportedVer   = "version_unsupported"
	ReasonUnknownFrameType = "unknown_frame_type"
	ReasonMissingField     = "missing_field"
	ReasonBadFieldValue    = "bad_field_value"
)

const sep = "|"
const kvSep = ";"
const kvAssign = "="

// Encode 把帧编码为线上文本：VERSION|TYPE|SEQ|TS_MS|k=v;k=v
// 字段按 key 排序输出，保证历史结论可复现。
func Encode(f Frame) (string, error) {
	if f.Type != FrameCommand && f.Type != FrameStatus && f.Type != FrameAck {
		return "", fault.New(fault.CatProtocol, fault.SourceProtocol, ReasonUnknownFrameType,
			fmt.Sprintf("cannot encode type %q", f.Type))
	}
	if !SupportedVersions[f.Version] {
		return "", fault.New(fault.CatProtocol, fault.SourceProtocol, ReasonUnsupportedVer,
			fmt.Sprintf("version %d not supported", f.Version))
	}
	if f.TsMs < 0 {
		return "", fault.New(fault.CatProtocol, fault.SourceProtocol, ReasonBadFieldValue, "ts_ms must be >= 0")
	}
	keys := make([]string, 0, len(f.Fields))
	for k := range f.Fields {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	parts := make([]string, 0, len(keys))
	for _, k := range keys {
		v := f.Fields[k]
		if strings.ContainsAny(k, sep+kvSep+kvAssign) || strings.ContainsAny(v, sep+kvSep) {
			return "", fault.New(fault.CatProtocol, fault.SourceProtocol, ReasonMalformedFrame,
				fmt.Sprintf("field %q contains reserved character", k))
		}
		parts = append(parts, k+kvAssign+v)
	}
	return fmt.Sprintf("%d|%s|%d|%d|%s",
		f.Version, f.Type, f.Seq, f.TsMs, strings.Join(parts, kvSep)), nil
}

// Decode 解析线上文本为帧。空帧、结构损坏、未知版本、未知帧类型
// 均返回 fault(CatProtocol) 且各带不同原因码，绝不返回零值帧让上层猜测。
func Decode(line string) (Frame, error) {
	raw := line
	line = strings.TrimSpace(line)
	if line == "" {
		return Frame{}, fault.New(fault.CatProtocol, fault.SourceProtocol, ReasonEmptyFrame, "empty frame")
	}
	segments := strings.Split(line, sep)
	if len(segments) != 5 {
		return Frame{}, fault.New(fault.CatProtocol, fault.SourceProtocol, ReasonMalformedFrame,
			fmt.Sprintf("expected 5 segments, got %d", len(segments)))
	}
	verN, err := strconv.Atoi(segments[0])
	if err != nil || verN <= 0 {
		return Frame{}, fault.Wrap(err, fault.CatProtocol, fault.SourceProtocol, ReasonBadFieldValue,
			"bad version: "+segments[0])
	}
	ver := Version(verN)
	// 能解析但不在支持列表：与“根本不是版本号”明确区分。
	if _, known := knownVersions[ver]; !known {
		return Frame{}, fault.New(fault.CatProtocol, fault.SourceProtocol, ReasonUnknownVersion,
			fmt.Sprintf("version %d is not a known protocol version", ver))
	}
	if !SupportedVersions[ver] {
		return Frame{}, fault.New(fault.CatProtocol, fault.SourceProtocol, ReasonUnsupportedVer,
			fmt.Sprintf("version %d known but not supported by this build", ver))
	}
	ft := FrameType(segments[1])
	if ft != FrameCommand && ft != FrameStatus && ft != FrameAck {
		return Frame{}, fault.New(fault.CatProtocol, fault.SourceProtocol, ReasonUnknownFrameType,
			fmt.Sprintf("frame type %q", segments[1]))
	}
	seq, err := strconv.ParseUint(segments[2], 10, 64)
	if err != nil {
		return Frame{}, fault.Wrap(err, fault.CatProtocol, fault.SourceProtocol, ReasonBadFieldValue, "bad seq")
	}
	ts, err := strconv.ParseInt(segments[3], 10, 64)
	if err != nil {
		return Frame{}, fault.Wrap(err, fault.CatProtocol, fault.SourceProtocol, ReasonBadFieldValue, "bad ts_ms")
	}
	fields := map[string]string{}
	if payload := segments[4]; payload != "" {
		for _, kv := range strings.Split(payload, kvSep) {
			eq := strings.Index(kv, kvAssign)
			if eq <= 0 || eq == len(kv)-1 {
				return Frame{}, fault.New(fault.CatProtocol, fault.SourceProtocol, ReasonMalformedFrame,
					"bad key/value pair: "+kv)
			}
			k, v := kv[:eq], kv[eq+1:]
			if strings.Contains(k, kvAssign) {
				return Frame{}, fault.New(fault.CatProtocol, fault.SourceProtocol, ReasonMalformedFrame,
					"duplicate '=' in pair: "+kv)
			}
			if _, dup := fields[k]; dup {
				return Frame{}, fault.New(fault.CatProtocol, fault.SourceProtocol, ReasonMalformedFrame,
					"duplicate field: "+k)
			}
			fields[k] = v
		}
	}
	return Frame{Version: ver, Type: ft, Seq: seq, TsMs: ts, Fields: fields, Raw: raw}, nil
}

// knownVersions 与 SupportedVersions 分离：未知版本（从未存在）
// 与已知但本构建不支持（升级/降级场景）给出不同结论。
var knownVersions = map[Version]bool{V1: true, V2: true}

// parseFloatField 读取并解析浮点字段，缺失与值损坏原因码不同。
func parseFloatField(f Frame, key string) (float64, bool, error) {
	raw, ok := f.Get(key)
	if !ok {
		return 0, false, nil
	}
	v, err := strconv.ParseFloat(raw, 64)
	if err != nil {
		return 0, true, fault.Wrap(err, fault.CatProtocol, fault.SourceProtocol, ReasonBadFieldValue,
			"field "+key+"="+raw)
	}
	return v, true, nil
}

// parseIntField 读取并解析整数字段。
func parseIntField(f Frame, key string) (int64, bool, error) {
	raw, ok := f.Get(key)
	if !ok {
		return 0, false, nil
	}
	v, err := strconv.ParseInt(raw, 10, 64)
	if err != nil {
		return 0, true, fault.Wrap(err, fault.CatProtocol, fault.SourceProtocol, ReasonBadFieldValue,
			"field "+key+"="+raw)
	}
	return v, true, nil
}
