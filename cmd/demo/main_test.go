package main

import (
	"bytes"
	"strings"
	"testing"
)

// TestDemoEndToEndDeterministic 端到端演示必须：
//  1. 完整跑完全部 12 个场景（含并发急停、可观测性统计），正常返回 nil
//     （等价 main 进程退出码 0）；
//  2. 连跑多次输出逐字节一致（并发段不向审计流注入调度相关噪声）；
//  3. 关键安全判定结论（拒绝/联锁/恢复）在输出中真实出现，说明演示
//     不是靠放宽校验跑通的。
func TestDemoEndToEndDeterministic(t *testing.T) {
	const runs = 3
	var prev []byte
	for i := 0; i < runs; i++ {
		var buf bytes.Buffer
		if err := runDemo(&buf); err != nil {
			t.Fatalf("demo run %d must complete without error (exit code 0), got: %v", i+1, err)
		}
		out := buf.Bytes()
		if i > 0 && !bytes.Equal(out, prev) {
			t.Fatalf("demo output not byte-identical across runs (run %d vs %d)", i, i+1)
		}
		prev = out
	}
	text := string(prev)

	// 12 个场景标题全部跑到。
	for _, want := range []string{
		"1. 上电默认 ESTOP",
		"2. 恢复条件不全",
		"3. 全部条件满足",
		"4. 连续速度轨迹",
		"5. 越界/矛盾命令",
		"6. 连续流式喂点",
		"7. 在线速度倍率",
		"8. 注入连续丢帧",
		"9. 链路自愈后状态不得自动回 RUN",
		"10. V2 力矩命令下发",
		"11. 急停与周期推进/命令提交并发",
		"12. 可观测性",
	} {
		if !strings.Contains(text, want) {
			t.Fatalf("demo must reach scenario %q; output tail:\n%s", want, tail(text, 800))
		}
	}

	// 安全恢复/拒绝判定该拒的仍然拒（不能为跑通而放宽）。
	mustContain := []string{
		"safety|controller|outputs_disabled",
		"safety|safety|recovery_denied",
		"command_rejected|trajectory|position_out_of_limit",
		"command_rejected|trajectory|contradictory_mode",
		"command_rejected|stream|override_out_of_range",
		"safety|safety|torque_overload",
		"recovery_conditions_contradict_observation", // 无真实健康观测仍被拒
		"state=COMM_LOSS outputs=false",
		"state=TORQUE_FAULT outputs=false",
		"state=ESTOP outputs=false",
		"急停后新增下行命令数=0 ✓",
		"================ SAFETY TRANSITIONS ================",
		"================ CLASSIFIED FAULTS ================",
		"================ CYCLE SNAPSHOTS ================",
	}
	for _, want := range mustContain {
		if !strings.Contains(text, want) {
			t.Fatalf("demo output missing expected safety decision %q", want)
		}
	}

	// 业务修复在端到端链路里也真实可见：
	// 第 6/7 段流排空后，第 8 段失联清空流，第 10 段单目标力矩命令在
	// 恢复后照常下发（证明同一会话单目标/流式可切换）。
	if !strings.Contains(text, "stream_drained") {
		t.Fatal("stream drain/hold event must be observable in demo")
	}
	t.Logf("演示输出 %d 字节，连续 %d 次逐字节一致，12 个场景与全部关键拒绝均出现",
		len(prev), runs)
}

func tail(s string, n int) string {
	if len(s) <= n {
		return s
	}
	return s[len(s)-n:]
}
