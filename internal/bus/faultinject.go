package bus

// Injection 描述一次确定性的链路故障注入。
// 命中计数按 Post 调用次数（不分方向，1-based），
// 因此同一个测试脚本的注入结论完全可复现。
type Injection struct {
	// DropAt 在指定的第 N 次 Post 丢弃消息（模拟丢帧/暂时失联）。
	DropAt []int
	// DuplicateAt 将命中的消息额外复制投递一次（模拟重复帧）。
	DuplicateAt []int
	// CorruptAt 将命中的消息内容损坏（模拟线路损坏 -> 协议错误）。
	CorruptAt []int
}

// Inject 配置故障注入计划（替换既有计划）。
func (b *Bus) Inject(in Injection) {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.inj = in
}
