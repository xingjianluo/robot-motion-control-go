package controller_test

import (
	"testing"

	clkpkg "github.com/xingjianluo/robot-motion-control-go/pkg/clock"
)

// 测试文件内部别名，避免多处重名导入冲突
type clockManual = clkpkg.Manual

var _ = (*testing.T)(nil)
