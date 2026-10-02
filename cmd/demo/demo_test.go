package main

// 端到端演示的确定性验证：run 必须完整跑完全部场景、正常结束（nil），
// 且连跑两次输出逐字节一致——对应文档承诺的“make demo 可稳定复现”。

import (
	"bytes"
	"strings"
	"testing"
)

func TestDemoRunsAllScenariosAndExitsCleanly(t *testing.T) {
	var buf bytes.Buffer
	if err := run(&buf); err != nil {
		t.Fatalf("demo 必须完整跑完且正常退出，实际中断: %v", err)
	}
	out := buf.String()

	// 关键场景/段落都必须出现（历史缺陷：场景 10 崩溃导致 11/12 缺席）。
	for _, want := range []string{
		"1. 上电默认 ESTOP",
		"6. 连续流式喂点",
		"7. 在线速度倍率",
		"10. V2 力矩命令下发",
		"11. 急停与周期推进/命令提交并发",
		"12. 可观测性",
		"SAFETY TRANSITIONS",
		"CLASSIFIED FAULTS",
		"CYCLE SNAPSHOTS",
	} {
		if !strings.Contains(out, want) {
			t.Fatalf("演示输出缺少场景/段落 %q\n（末尾输出）:\n%s", want, tail(out, 800))
		}
	}
	// 关键安全判定不能因为“跑通演示”被放宽。
	for _, want := range []string{
		"state=TORQUE_FAULT",
		"无人工确认恢复被拒（预期）",
		"人工确认恢复: ",
		"state=ESTOP outputs=false",
		"急停后新增下行命令数=0",
		"倍率越界被拒（预期）",
		"位置越界（预期）",
		"矛盾命令（预期）",
		"state=COMM_LOSS",
	} {
		if !strings.Contains(out, want) {
			t.Fatalf("演示丢失安全判定 %q", want)
		}
	}
	t.Logf("演示完整跑完，输出 %d 字节；末段:\n%s", len(out), tail(out, 600))
}

func TestDemoDeterministicAcrossRuns(t *testing.T) {
	var a, b bytes.Buffer
	if err := run(&a); err != nil {
		t.Fatalf("第一次运行失败: %v", err)
	}
	if err := run(&b); err != nil {
		t.Fatalf("第二次运行失败: %v", err)
	}
	if a.Len() != b.Len() {
		t.Fatalf("两次输出长度不一致: %d vs %d", a.Len(), b.Len())
	}
	if !bytes.Equal(a.Bytes(), b.Bytes()) {
		// 找到首个差异位置，便于定位不确定源。
		ab, bb := a.Bytes(), b.Bytes()
		for i := 0; i < len(ab) && i < len(bb); i++ {
			if ab[i] != bb[i] {
				t.Fatalf("两次输出在字节 %d 处首次不同:\n--- run1\n%s\n--- run2\n%s",
					i, tail(string(ab[i:]), 300), tail(string(bb[i:]), 300))
			}
		}
		t.Fatal("两次输出内容不同")
	}
	t.Logf("连跑两次输出逐字节一致（%d 字节），退出码均为 0", a.Len())
}

func tail(s string, n int) string {
	if len(s) <= n {
		return s
	}
	return "..." + s[len(s)-n:]
}
