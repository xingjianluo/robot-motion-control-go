# 机器人多关节运动控制与安全联锁系统（本地模拟）

纯本地、确定性、无硬件依赖的多关节运动控制原型：以本地总线帧模拟驱动器与编码器，
验证轨迹下发、反馈闭环、安全停机与故障恢复在**异常与并发**场景下仍有
**可预期且可解释**的结果。

仅依赖 Go 标准库，无需真实硬件、外部服务或凭据。

## 快速验证

```bash
go test ./... -race -count=1     # 全量测试（含竞态检测），应全部 PASS
go run ./cmd/simdemo             # 确定性端到端演示，输出可反复复现
go test ./pkg/controller/ -v     # 查看关键用例打印的“输入/判定依据/最终状态”
```

要求：Go 1.25+（见 `go.mod`）。所有时间源可替换（`pkg/clock`），测试使用
**手动确定性时钟**，不依赖 wall-clock 与 sleep，因此结果稳定可复现。

## 组件划分（职责单一，避免集中）

| 包 | 职责 |
|---|---|
| `pkg/errors` | 故障分类：`COMM` / `PROTOCOL` / `SAFETY` / `COMMAND` 四类不合并 |
| `pkg/clock` | 时钟抽象；`Manual` 手动时钟供确定性测试 |
| `pkg/frame` | 总线帧与多版本协议（V1/V2）编解码；未知帧/版本/缺字段明确归类 |
| `pkg/bus` | 本地模拟总线：有界队列、积压策略、故障注入、丢弃归属统计 |
| `pkg/limits` | 关节位置/速度/力矩限位与命令校验，拒绝原因可区分 |
| `pkg/trajectory` | 逐周期连续的梯形速度轨迹；速度/加速度/硬限位约束始终成立 |
| `pkg/feedback` | 反馈质量监控：丢失分级、乱序/重复/时间戳跳变/位置跳变 |
| `pkg/safety` | 安全联锁状态机：优先级、锁存、显式恢复条件、迁移留痕 |
| `pkg/controller` | 并发编排：原子控制周期、有界命令队列、状态快照、计数归属 |
| `cmd/simdemo` | 纯本地端到端演示程序 |

## 控制周期（单把锁内原子完成，无半更新）

每个 `Tick()` 周期在同一互斥锁内严格按序执行：

1. **排空总线**：反馈帧解码后交给质量监控；坏帧计 `FbBadFrames`，不中断周期；
2. **反馈时间判定**：按年龄区分 `STALE`（暂时失联）与 `LOST`（确定丢失）；
3. **安全事件汇总**：每关节上报看门狗/软限位/力矩/跟踪偏差，同类型事件做
   **“或”聚合**（任一关节触发即激活，不会被另一关节的 false 覆盖）；
4. **安全裁决**：非 `OPERATIONAL` 时**停止一切命令下发**，排队命令保留；
5. **运动推进**（仅在允许时）：每关节取一条排队命令，位置模式规划梯形轨迹，
   速度/力矩模式保持直驱，逐周期编码下发；
6. **生成值拷贝快照**并记录周期间隔（抖动可观测）。

**并发确定性**：命令下发、反馈接收、安全联锁、状态上报共享同一把锁。
急停 `EStop()` 与 `Tick()` 并发时二者串行：`Tick` 先拿锁则该周期完整推进，
`EStop` 先拿锁则该周期停止下发——不存在交叉或半更新状态，最终状态唯一确定。
`TestConcurrentEStopVsTick` 以 20 轮竞争验证。

## 控制状态与安全优先级

优先级从高到低（高优先级覆盖低优先级，但低优先级故障仍被锁存）：

| 状态 | 触发来源（Kind） | 触发条件 |
|---|---|---|
| `ESTOP` | `ESTOP` | 显式人工急停，任意状态可触发，最高优先级 |
| `SOFT_LIMIT` | `SOFT_LIMIT` | **真实位置**越过软限位（硬限位内缩 `SoftMargin`） |
| `WATCHDOG` | `WATCHDOG` | 反馈暂时失联/丢失、乱序、重复、时间戳或位置跳变（链路不可信） |
| `TORQUE_FAULT` | `TORQUE_FAULT` | V2 实测力矩绝对值超限 |
| `FAULT_STOP` | `DRIVE_FAULT` / `TRACKING_ERROR` | 驱动器故障位；或反馈有效但位置真实偏差超阈值 |
| `OPERATIONAL` | — | 唯一允许下发运动命令的状态 |

关键规则（见 `pkg/safety`）：

- **安全优先于普通命令**：非 `OPERATIONAL` 一律拒绝运动下发，返回
  `SAFETY:MOTION_NOT_ALLOWED`；
- **全部锁存，不静默恢复**：瞬时条件消失不会自动回到 `OPERATIONAL`；
- **显式恢复条件**：
  - 急停必须先 `ReleaseEStop()`；若释放时仍有其他锁存故障，系统停留在
    对应故障状态而非静默恢复；
  - 其余故障必须在触发条件确已消失后显式 `Reset()`；条件仍成立时
    `Reset` 返回 `SAFETY:RESET_CONDITIONS_UNMET` 并列明未满足项；
- 所有状态迁移记录时间戳、来源与原因（`Transitions()`，最近 128 条）。

## 反馈可信度：失联 ≠ 真实位置偏差

`pkg/feedback` 只在帧通过全部质量检查时才更新“最新有效反馈”：

| 条件 | 结论 | 安全归属 |
|---|---|---|
| 超过 `StaleTimeout` 无新帧 | `STALE` 暂时失联 | WATCHDOG |
| 超过 `LostTimeout` | `LOST` 反馈丢失 | WATCHDOG |
| 序号回退 / 时间戳回退 | `OUT_OF_ORDER` | WATCHDOG |
| 序号重复 | `DUPLICATE` | WATCHDOG |
| 时间戳前向跳变超阈值 | `TIMESTAMP_JUMP` | WATCHDOG |
| 位置单周期跳变（扣除预期运动）超阈值 | `POSITION_JUMP` | WATCHDOG |
| 帧有效但参考与实测偏差超阈值 | 跟踪误差 | **FAULT_STOP / TRACKING_ERROR** |

异常帧**不更新基线、不计入跟踪误差**（快照中 `TrackingError` 恒为 0），
因此乱序/丢帧绝不会被误报为真实位置偏差；反之反馈新鲜且连续时的偏差
也不会被通信故障掩盖。两者来源码分别是 `WATCHDOG` 与 `TRACKING_ERROR`。

## 命令校验与轨迹连续性

- 位置/速度/力矩三种模式**互斥**：夹带其他字段返回
  `COMMAND:CONTRADICTORY_COMMAND`；
- 越硬限位 `POS_LIMIT`、超速 `VEL_LIMIT`、超力矩 `TORQUE_LIMIT`，
  非有限数值 `MALFORMED_VALUE`，原因码互不相同；
- 位置目标经梯形速度曲线分解：逐周期加速度 ≤ `AccMax`、速度 ≤ `VelMax`，
  参考位置被钳制在硬限位内，终点离散吸附时仍保证减速度约束（见
  `TestTrapezoidConstraints` 打印的实测峰值）。

## 协议兼容范围（`pkg/frame`）

线缆格式：`0xA5 | Ver | Type | Joint(u16) | Len(u32) | Payload | XOR`（小端）。

| 版本 | 支持内容 | 兼容承诺 |
|---|---|---|
| **V1** | 位置/速度命令与反馈（Seq/时间戳/位置/速度，28B 负载） | 历史版本长期兼容：不含力矩/状态字字段，解码时这两项恒为零值/ false，不会误报力矩超限 |
| **V2** | 新增力矩命令 `0x03`、状态帧 `0x11`、反馈力矩与故障位（37B 负载） | 当前版本 |

明确归类而不忽略的错误：`UNSUPPORTED_VERSION`、`UNKNOWN_FRAME_TYPE`、
`FRAME_CORRUPT`（起始字节/XOR 校验）、`FRAME_TRUNCATED`、
`FIELD_MISSING`（负载短缺）、`PAYLOAD_MALFORMED`（负载粘连）、
`CMD_UNSUPPORTED_VERSION`（V1 下发力矩命令）。同一输入编码字节确定，可复现。

## 积压与抖动（有界、可观测、有归属）

- 命令队列：每关节有界深度（默认 8），满时按配置策略处理：
  `RejectNew` 返回 `COMMAND:QUEUE_FULL`；`OverwriteOld` 丢最旧命令；
  均计入 `Snapshot.CmdDropped`，**绝不无界占用内存**；
- 总线队列：固定容量环形缓冲，`DropOldest` / `DropNewest`；
  `bus.Stats().Events` 记录每次丢弃的方向、关节、类型与原因
  （`OVERFLOW_*` / `FAULT_INJECT_DROP`）；
- 周期抖动：`Snapshot.LastJitter` / `LastPeriod` 报告最近一个周期间隔
  相对配置周期的偏差，可定位到周期号 `Cycle`；
- 故障注入：`bus.Config.FaultInject` 可丢帧/篡改帧，模拟线路故障；
  线路丢帧对发送方不是错误，而由接收方看门狗体现为通信故障。

## 故障来源不合并

所有错误均为 `pkg/errors.Error{Cat, Code, Detail}`：

- `COMM`：总线/看门狗类通信问题；
- `PROTOCOL`：帧格式、版本、字段问题；
- `SAFETY`：安全触发与非法恢复；
- `COMMAND`：限位、矛盾命令、队列拒绝。

用 `errors.IsCategory(err, errors.CatXxx)` 编程判别，`Code` 提供细粒度原因，
`Detail` 给出人类可读的判定依据。

## 关键测试索引

| 测试 | 覆盖点 |
|---|---|
| `frame_test.go` | V1 向后兼容（字段裁剪/字节可复现）、V2 往返、版本/类型/校验/缺字段 |
| `limits_test.go` | 各类拒绝原因码互异 |
| `trajectory_test.go` | 梯形曲线全程 v/a/位置约束、终点静止 |
| `TestHappyPath` | 端到端轨迹约束与闭环 |
| `TestFeedbackDuplicateAndReorder` / `TestFeedbackLossStages` / `TestPositionJump` / `TestTimestampJump` | 乱序、重复、丢失分级、跳变不计跟踪误差 |
| `TestRealTrackingErrorVsLoss` | 真实偏差归 `TRACKING_ERROR`，与失联区分 |
| `TestEStopPriorityAndRecovery` / `TestSoftLimit` / `TestTorqueOverlimit` | 优先级、锁存、恢复条件不满足被拒 |
| `TestV1NoTorqueAndNoSoftFalsePositive` | V1 兼容边界 |
| `TestConcurrentEStopVsTick` | 急停与周期推进并发的确定性（-race 验证） |
| `TestQueueBacklogPolicies` / `TestBusBacklogDropAttribution` / `TestFaultInjectionDropsFrame` | 有界积压策略与丢弃归属 |
| `TestJitterObservable` | 周期抖动可观测 |
| `TestProtocolErrorsIngested` | 坏帧明确归类为 PROTOCOL 且计数 |
