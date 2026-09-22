package bus_test

import (
	"testing"

	"github.com/xingjianluo/robot-motion-control-go/pkg/bus"
	"github.com/xingjianluo/robot-motion-control-go/pkg/frame"
)

func TestFaultInjectionDropsFrame(t *testing.T) {
	var dropped bool
	b := bus.New(bus.Config{
		Capacity: 4, Policy: bus.DropOldest,
		FaultInject: func(f frame.Frame) (frame.Frame, bool) {
			return f, false // 模拟所有帧在总线上丢失
		},
	})
	f, _ := frame.EncodeCommand(frame.V2, frame.TypeCmdPosition, 1, 0.5)
	if err := b.Send(f); err != nil {
		t.Fatalf("线路丢帧对发送方不应是错误（应由看门狗体现）: %v", err)
	}
	if _, ok := b.Recv(); ok {
		t.Fatal("故障注入丢帧后不应收到帧")
	}
	st := b.Stats()
	if st.Sent != 1 || st.Received != 0 || st.Dropped != 1 {
		t.Fatalf("统计错误: %+v", st)
	}
	if st.Events[0].Reason != "FAULT_INJECT_DROP" {
		t.Fatalf("丢弃原因应可追溯，得到 %s", st.Events[0].Reason)
	}
	_ = dropped
	t.Logf("注入线路丢帧：Sent=%d 但 Received=%d，丢弃原因=%s（由看门狗表现为通信故障）",
		st.Sent, st.Received, st.Events[0].Reason)
}

func TestDropOldestPolicy(t *testing.T) {
	b := bus.New(bus.Config{Capacity: 2, Policy: bus.DropOldest})
	mk := func(joint int) frame.Frame {
		f, _ := frame.EncodeCommand(frame.V2, frame.TypeCmdPosition, joint, 0.1)
		return f
	}
	if err := b.Send(mk(0)); err != nil {
		t.Fatal(err)
	}
	if err := b.Send(mk(1)); err != nil {
		t.Fatal(err)
	}
	if err := b.Send(mk(2)); err != nil { // 挤掉 joint0 最旧帧
		t.Fatal("DropOldest 不应向发送方报错")
	}
	got := []int{}
	for {
		f, ok := b.Recv()
		if !ok {
			break
		}
		got = append(got, f.Joint)
	}
	if len(got) != 2 || got[0] != 1 || got[1] != 2 {
		t.Fatalf("最旧帧应被丢弃，实际收到关节 %v", got)
	}
	st := b.Stats()
	if st.Dropped != 1 || st.Events[0].Joint != 0 {
		t.Fatalf("丢弃归属错误: %+v", st.Events)
	}
	t.Logf("容量2发送3帧(DropOldest) -> 保留 %v，丢弃归属 joint=%d", got, st.Events[0].Joint)
}
