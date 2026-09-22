// Package fault 定义跨组件统一的可追溯错误类型，
// 显式区分命令拒绝、协议错误、通信故障与安全触发四类来源。
package fault

import (
	"errors"
	"strings"
)

// Category 是故障大类，禁止把不同来源合并成同一个笼统错误。
type Category string

const (
	// CatCommandReject 命令因越界/矛盾/不安全被主动拒绝。
	CatCommandReject Category = "command_rejected"
	// CatProtocol 帧可到达但无法被协议接受（坏帧、未知帧、字段缺失、版本不支持）。
	CatProtocol Category = "protocol_error"
	// CatComm 链路问题：丢帧、积压丢弃、超时、暂时失联。
	CatComm Category = "communication_fault"
	// CatSafety 安全联锁触发或恢复被拒。
	CatSafety Category = "safety"
)

// Source 标识故障首次产生的组件。
type Source string

const (
	SourceTrajectory Source = "trajectory"
	SourceProtocol   Source = "protocol"
	SourceBus        Source = "bus"
	SourceFeedback   Source = "feedback"
	SourceSafety     Source = "safety"
	SourceController Source = "controller"
)

// Error 携带类别、来源组件、机器可判定的原因码与可读说明。
type Error struct {
	Category Category
	Source   Source
	// Reason 是稳定的机器可读原因码，测试依据该码断言而非错误文案。
	Reason  string
	Message string
	Err     error
}

func (e *Error) Error() string {
	var b strings.Builder
	b.WriteString(string(e.Category))
	b.WriteString("|")
	b.WriteString(string(e.Source))
	b.WriteString("|")
	b.WriteString(e.Reason)
	if e.Message != "" {
		b.WriteString(": ")
		b.WriteString(e.Message)
	}
	if e.Err != nil {
		b.WriteString(": ")
		b.WriteString(e.Err.Error())
	}
	return b.String()
}

// Unwrap 支持 errors.Is/As 穿透。
func (e *Error) Unwrap() error { return e.Err }

// New 构造一个故障错误。
func New(cat Category, src Source, reason, message string) *Error {
	return &Error{Category: cat, Source: src, Reason: reason, Message: message}
}

// Wrap 包装下层错误并附加分类信息。
func Wrap(err error, cat Category, src Source, reason, message string) *Error {
	return &Error{Category: cat, Source: src, Reason: reason, Message: message, Err: err}
}

// As 尝试把 err 解析为 *fault.Error。
func As(err error) (*Error, bool) {
	var fe *Error
	if errors.As(err, &fe) {
		return fe, true
	}
	return nil, false
}
