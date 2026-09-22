// Package protocol 定义驱动器/编码器本地总线帧与多版本编解码规则。
//
// 线格式（基于文本、便于本地复现）:
//
//	<VERSION>|<TYPE>|<SEQ>|<TS_MS>|<PAYLOAD...>
//
// 不同版本下同一帧类型允许的字段集合不同；解码器必须对未知帧、
// 字段缺失、未知版本给出可区分的原因。
package protocol

// Version 是协议版本号。
type Version int

const (
	V1 Version = 1
	V2 Version = 2
)

// SupportedVersions 为当前构建支持的版本集合（历史行为必须保持兼容）。
var SupportedVersions = map[Version]bool{V1: true, V2: true}

// FrameType 标识帧类型。
type FrameType string

const (
	FrameCommand FrameType = "CMD" // 控制器 -> 驱动器
	FrameStatus  FrameType = "STA" // 驱动器/编码器 -> 控制器
	FrameAck     FrameType = "ACK" // 驱动器命令应答
)

// Frame 是解析后的通用帧。字段含义:
//
//	Version 帧版本
//	Type    帧类型
//	Seq     发送序号（接收侧据此识别重复/跳变）
//	TsMs    设备侧时间戳（毫秒），0 表示设备未提供
//	Fields  有效负载键值对
type Frame struct {
	Version Version
	Type    FrameType
	Seq     uint64
	TsMs    int64
	Fields  map[string]string
	Raw     string
}

// Get 读取字段，缺失时返回 ("", false)，调用方必须显式处理缺失。
func (f Frame) Get(key string) (string, bool) {
	v, ok := f.Fields[key]
	return v, ok
}

// 共享字段名（命令帧与状态帧复用相同物理量的键名）。
const (
	FieldPos = "pos"
	FieldVel = "vel"
	FieldTau = "tau"
)
