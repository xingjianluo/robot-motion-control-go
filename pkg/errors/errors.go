// Package errors 定义系统中可区分、可追溯的故障分类。
// 通信故障、协议错误、安全触发与命令拒绝分别归类，绝不合并为笼统错误。
package errors

import "fmt"

// Category 故障来源分类。
type Category string

const (
	CatComm     Category = "COMM"     // 通信故障：总线超时、帧丢失、看门狗
	CatProtocol Category = "PROTOCOL" // 协议错误：未知帧、版本不支持、字段缺失
	CatSafety   Category = "SAFETY"   // 安全触发：急停、软限位、力矩超限
	CatCommand  Category = "COMMAND"  // 命令拒绝：越界、自相矛盾、状态不允许
)

// Error 携带分类与原因的可追溯错误。
type Error struct {
	Cat    Category
	Code   string // 细粒度原因码，如 "ESTOP_ACTIVE"、"POS_LIMIT"
	Detail string // 人类可读的判定依据
}

func (e *Error) Error() string {
	return fmt.Sprintf("[%s:%s] %s", e.Cat, e.Code, e.Detail)
}

// New 构造分类错误。
func New(cat Category, code, detail string) *Error {
	return &Error{Cat: cat, Code: code, Detail: detail}
}

// IsCategory 判断错误是否属于指定分类。
func IsCategory(err error, cat Category) bool {
	if e, ok := err.(*Error); ok {
		return e.Cat == cat
	}
	return false
}
