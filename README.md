# 机器人多关节运动控制与安全联锁系统（本地模拟）

一套**完全本地、确定性**的单关节运动控制与安全联锁参考实现。用进程内有界
“总线”模拟驱动器/编码器，用可手动推进的假时钟驱动控制周期，使**轨迹下发、
反馈闭环、安全停机、故障恢复**在异常与并发场景下都能给出**可预期、可解释、
可复现**的结果。

> 无需真实硬件、外部服务或任何凭据；仅依赖 Go 标准库（零第三方依赖）。

---

## 1. 快速开始

```bash
go version            # 需要 Go 1.25+
make test             # 全量单测（含 -race）
make demo             # 端到端确定性演示
make verbose          # 打印关键测试的输入 / 判定依据 / 最终状态
```

或直接：

```bash
go test ./... -race -count=1
go run ./cmd/demo
go test ./internal/control/ -run TestExplainableDecisionFlow -v
```

`make demo` 用固定起始时刻 `2026-09-22 09:00:00 UTC` 和固定参数运行，
**每次输出逐字节一致**，便于稳定复现。

---

## 2. 系统全景与组件职责

```
                ┌─────────────────────────────────────────────┐
                │                Controller                   │
                │  (internal/control, 单把锁保证原子周期)       │
 上层命令 ─────► │  Validator → Setpoint 预检 → Buffer(有界)    │
                │        │                                     │
                │        ▼                                     │
                │  Step(): 消费反馈 → 看门狗 → 安全门控 → 下发   │
                └───▲───────────────────────┬──────────────────┘
                    │ 上行 STA               │ 下行 CMD
            ┌───────┴─────────┐     ┌───────┴──────────┐
            │ feedback        │     │ trajectory       │
            │ Consumer(分类)  │     │ Validator+Buffer │
            └───────▲─────────┘     └──────────────────┘
                    │
            ┌───────┴───────────────────────────────┐
            │  Bus (internal/bus) 有界队列+丢弃策略   │
            │  + 故障注入(丢帧/重复/损坏) + 陈旧淘汰   │
            └───────▲───────────────────────┬───────┘
                    │ 上行 STA               │ 下行 CMD
            ┌───────┴─────────┐     ┌───────┴──────────┐
            │  Device         │     │ safety Monitor   │
            │ (模拟驱动器/    │     │（软限位/力矩/    │
            │  编码器,V1/V2)  │     │  故障码检查）     │
            └─────────────────┘     └──────────────────┘
```

| 包 | 职责 |
|----|------|
| `internal/clock` | 时钟抽象与确定性 `Fake` 时钟；同刻定时器按注册顺序触发 |
| `internal/fault` | 统一错误类型：**类别 Category × 来源 Source × 原因码 Reason** |
| `internal/protocol` | V1/V2 帧编解码；未知帧/缺字段/版本不支持各自独立归类 |
| `internal/bus` | 有界双工队列、`drop_oldest/newest`、MaxAge 陈旧淘汰、故障注入 |
| `internal/trajectory` | 位置/速度/力矩命令的限位、矛盾校验、加速度连续性、有界轨迹缓冲 |
| `internal/feedback` | 反馈分类：接受/重复/乱序/跳变/陈旧；看门狗区分“失联 vs 偏差” |
| `internal/safety` | 优先级联锁状态机 + 软限位/力矩/故障码监视器；显式恢复条件 |
| `internal/control` | 组合各组件，**一把锁**串行化提交/急停/恢复/周期，输出完整快照 |
| `internal/report` | 快照、安全变迁、带归属故障的记录与可读输出 |
| `internal/device` | 纯软件驱动器/编码器，一阶模型，可按 V1/V2 发状态帧 |
| `internal/joints` | 多关节编排：共享周期边界原子推进，单关节急停即整体联锁 |
| `cmd/demo` | 10 段端到端场景的确定性演示 |

---

## 3. 控制状态与安全优先级

### 3.1 安全状态机（数值越大优先级越高）

| 状态 | 优先级 | 输出 | 是否锁存 | 含义 |
|------|:---:|:---:|:---:|------|
| `RUN` | 0 | 使能 | 否 | 唯一允许功率输出的状态 |
| `SOFT_LIMIT` | 10 | 切断 | 软锁存 | 进入软限位带；位置回到带内由监视器**显式清除并留痕** |
| `TORQUE_FAULT` | 20 | 切断 | 锁存 | 力矩超限或驱动器故障码 |
| `COMM_LOSS` | 30 | 切断 | 锁存 | 反馈看门狗超时（**暂时失联**） |
| `ESTOP` | 40 | 切断 | 锁存 | 急停，最高优先级，可由任意 goroutine 触发 |

**关键规则**

- **安全优先于普通控制**：非 `RUN` 状态下任何运动命令都以
  `safety|controller|outputs_disabled` 拒绝，绝不落入普通校验。
- **低优先级触发不能覆盖高优先级状态**；被抑制的触发可通过
  `LastSuppressed()` 观测，不静默丢弃。
- **没有静默降级/自动恢复**：`ESTOP / TORQUE_FAULT / COMM_LOSS` 退出到
  `RUN` 只能通过显式 `Recover(RecoveryConditions)`，且必须**同时**满足：
  急停释放、通信健康、力矩正常、位置在限内、故障码清零、**人工确认**；
  任一不满足返回 `safety|safety|recovery_denied` 并逐条列出缺失项。
- 即使 `COMM_LOSS` 后链路已自愈、健康反馈恢复，状态也**保持锁存**，
  直到人工确认——防止“掉线—恢复”导致的意外出力。

### 3.2 一个控制周期（`Step`）的确定性顺序

1. 周期计数与**抖动观测**（实际周期偏差、超限次数）。
2. **消费上行反馈**（任何安全状态下都消费，否则恢复条件无从判定）：
   每周期最多消费 `MaxBacklog` 帧；坏帧/重复/乱序/跳变各自归类，
   异常帧**不更新**连续性基准。
3. **看门狗**：超过 `WatchdogTimeout` 无有效帧 ⇒ 触发 `COMM_LOSS`
   （`communication_fault`，**不是**跟踪误差）。
4. **安全门控**：仅 `OutputsEnabled()` 时推进轨迹、编码并投递命令；
   非 RUN 状态清空待发航点。
5. 生成并上报**完整快照**（安全状态、输出、跟踪、总线统计、最近故障）。

### 3.3 并发正确性

`SubmitCommand / Estop / Recover / Step / Snapshot` 全部走控制器**同一把
锁**，每个方法在一个临界区内完成所有状态读写与总线投递：

- 一个周期内外部要么看到**完整上周期状态**，要么看到**完整本周期状态**，
  不存在半更新或相互覆盖；
- 急停与周期推进并发时，先拿锁者先执行，后拿锁者读到的已是 `ESTOP`，
  因此**急停提交之后的任何周期都不会再下发出力命令**（有并发测试断言
  “急停后下行命令计数不再增长”）；
- 锁序固定为 `控制器锁 → 总线锁 → 状态机锁`，无死锁环。

### 3.4 多关节联锁（`internal/joints`）

`Arm` 挂载多个独立单关节控制器（各有独立总线与状态机），在**共享周期边界**
`StepAll` 上按固定挂载顺序串行推进：

- 任一关节在本周期进入 `ESTOP`，周期结束时**所有关节都被联锁急停**，
  不会出现“部分关节 RUN、部分切断”的半更新状态；返回首个触发关节名；
- `RecoverAll` 是**全有或全无**：任一关节恢复条件不满足，则本轮已恢复的
  关节回滚为急停，杜绝部分出力；
- `EstopAll` 以固定顺序急停，跨关节推进顺序确定、结论可复现。

---

## 4. 命令合法性与轨迹连续性

位置 / 速度 / 力矩三类命令在进入轨迹缓冲前依次经过固定顺序的检查，
**拒绝原因彼此可区分**（类别恒为 `command_rejected`）：

| 原因码 | 触发条件 |
|--------|----------|
| `contradictory_mode` | 未知模式；或位置命令携带非零速度/力矩等“自相矛盾”字段 |
| `position_out_of_limit` | 位置目标超出硬位置范围 |
| `velocity_out_of_limit` | `|vel|` 超过速度上限 |
| `torque_out_of_limit` | `|tau|` 超过力矩上限 |
| `accel_out_of_limit` | 相邻命令按时间差估算所需加速度超限（连续性被破坏） |
| `non_monotonic_time` | 命令/航点时间戳早于上一条 |
| `trajectory_buffer_overflow` | 有界轨迹缓冲已满 |

- **越界绝不静默裁剪**：裁剪会把危险命令伪装成正常命令，因此一律拒绝。
- 同一执行时刻重复提交目标时，轨迹缓冲做**替换**（一个时刻只允许一个目标），
  而不是并存或报错。
- 协议层不兼容（如 V1 上请求力矩模式）在命令**入口**即以
  `protocol_error|version_unsupported` 拒绝，不会先入缓冲再在周期内失败。

---

## 5. 反馈消费：失联、乱序、跳变都不是“跟踪误差”

反馈消费者维护“最近一条被接受样本”作为基准，对每帧给出确定分类：

| 分类 | 原因码 | 处理 |
|------|--------|------|
| accepted | — | 更新基准，参与跟踪/安全计算 |
| duplicate | `duplicate_seq` | 序号重复，拒绝，不更新基准 |
| out_of_order | `out_of_order_frame` | 序号回退或设备时间戳回退超 `MaxBackdateMs` |
| position_jump | `position_jump` | 单周期位置增量超 `MaxStepRad`，标记可疑但不当偏差 |
| （协议坏帧） | `protocol_error/*` | 解码失败，保持协议类别，绝不被当反馈接受 |

- **看门狗**：超过 `WatchdogTimeout` 无**有效帧** ⇒
  `communication_fault|feedback|feedback_watchdog_timeout`（暂时失联），
  持续失联给 `link_temporarily_lost`。这两类都不是位置/速度偏差，
  快照中 `Tracking.Healthy=false` 且误差字段不具意义。
- V1 状态帧没有力矩字段时 `TorqueKnown=false`：系统区分“字段缺失”与
  “值为 0”，缺字段时安全侧**跳过力矩检查**，既不误判零力矩也不触发超限。

---

## 6. 协议兼容范围（V1 / V2 共存）

线格式（文本，便于本地复现）：

```
<VERSION>|<TYPE>|<SEQ>|<TS_MS>|k=v;k=v
```

- `TYPE ∈ {CMD, STA, ACK}`；字段按 key 排序编码，**输出确定可复现**。
- **V1**：位置/速度命令；状态帧含 `pos/vel`，`en/fault` 缺省按历史默认
  （`en=1,fault=0`）；**无力矩**。
- **V2**：新增力矩命令模式与 `tau`；状态帧**强制要求** `tau` 与 `en`，
  缺失即 `protocol_error|missing_field`。
- 解析层把三种情况明确区分：
  - `unknown_version`：版本号根本不被认识；
  - `version_unsupported`：版本已知但本构建不支持；
  - `unknown_frame_type` / `missing_field` / `bad_field_value` /
    `malformed_frame` / `empty_frame`：结构与字段问题。
- 控制器可配置为 V2 同时接收 V1 设备的反馈（多版本共存，有兼容测试）。

---

## 7. 积压、丢弃与可观测性

- 总线两个方向各持有界 FIFO，容量满时按策略处理，**内存永远有界**：
  - `drop_oldest`：丢最旧，接纳新帧（丢弃计入 `DroppedFull`）；
  - `drop_newest`：拒绝新帧并向调用方返回
    `communication_fault|bus|queue_full_dropped`，由上层决定降级。
- 帧在队列中停留超过 `MaxAge`，出队时判为陈旧丢弃
  （`stale_frame_dropped`），不会被伪装成新鲜反馈。
- 控制器每周期最多消费 `MaxBacklog` 帧，避免积压饿死单周期；未消费帧
  留在有界总线中。追赶周期若跳过若干到期航点，记录
  `waypoints_skipped_by_cycle_catchup`。
- 所有丢弃/超时/注入都在 `bus.Stats`（含最近丢弃的原因码、序号、方向）
  与快照中可观测，**归属明确**。
- 确定性故障注入 `bus.Inject{DropAt, DuplicateAt, CorruptAt}` 按 Post
  调用序号命中：丢帧→`injected_frame_loss`；重复→投递两份相同帧；
  损坏→接收方归类为协议错误。

---

## 8. 故障分类：三类来源绝不合并

每个错误都携带 `Category | Source | Reason`：

| Category | 含义 | 典型原因码 |
|----------|------|-----------|
| `command_rejected` | 命令主动被拒（越界/矛盾/不安全） | `position_out_of_limit`、`accel_out_of_limit` … |
| `protocol_error` | 帧到达但协议不接受 | `unknown_frame_type`、`missing_field`、`version_unsupported` … |
| `communication_fault` | 链路问题 | `feedback_watchdog_timeout`、`queue_full_dropped`、`injected_frame_loss`、`position_jump` … |
| `safety` | 安全联锁触发或恢复被拒 | `estop_pressed`、`torque_overload`、`recovery_denied` … |

`Source` 进一步指明首次产生组件（trajectory/protocol/bus/feedback/
safety/controller）。测试依据**稳定原因码**断言，而不依赖错误文案。

---

## 9. 验证方法（本地稳定复现）

### 9.1 单元 / 并发 / 故障注入测试

```bash
make test           # 等价：go test ./... -race -count=1
```

覆盖要点（均使用确定性假时钟与模拟总线）：

- **时钟**：同刻定时器按注册顺序触发、Stop/Reset 语义；
- **协议**：V1/V2 往返、缺字段、未知帧、版本不支持、编码确定性；
- **总线**：两种丢弃策略、陈旧淘汰、丢帧/重复/损坏注入的归属；
- **轨迹**：各类拒绝原因、加速度连续性、时间戳单调、缓冲有界；
- **反馈**：重复/乱序/跳变不污染基准、看门狗“失联≠偏差”、V1 兼容；
- **安全**：优先级与抑制、急停恒赢、恢复条件全列举、软限位显式清除；
- **控制器场景**（`internal/control/scenarios_test.go`）：
  上电 ESTOP→恢复→跟踪、四类故障不合并、看门狗失联与显式恢复、
  **急停并发**（200 周期竞争 + race 检测）、积压上限、力矩锁存、V1/V2 共存；
- **可解释用例** `TestExplainableDecisionFlow`：逐步打印
  **输入 → 判定依据（类别|来源|原因）→ 最终状态快照**，并 Dump 完整审计轨迹。

### 9.2 端到端演示

```bash
make demo
```

10 个场景顺序演示：上电 ESTOP 拒绝命令 → 条件不全拒绝恢复 → 满足条件+人工
确认 → 连续速度轨迹与反馈闭环（1 rad/s×0.3s=0.3 rad，误差 0）→
越界/矛盾命令被区分拒绝 → 连续丢帧触发看门狗 COMM_LOSS → 链路自愈不自动
复位 → 显式恢复 → 力矩超限 TORQUE_FAULT 锁存 → 缺人工确认被拒 → 恢复 →
急停与周期/命令并发（最终必为 ESTOP 且无新命令）→ 积压/注入/抖动可观测性。

### 9.3 确定性与无外部依赖说明

- 所有时间来自 `clock.Fake`（手动 `Advance`），不依赖墙钟与 goroutine
  调度时序；并发用例验证的是**锁保证的最终状态**，而非时序巧合。
- 总线、设备全在进程内；无网络、无文件 IO 依赖、无真实凭据。
- 唯一第三方“依赖”是 Go 工具链本身。

---

## 10. 关键参数（均可在 `control.Config` 配置）

| 参数 | 含义 |
|------|------|
| `Period` | 控制周期（演示为 10ms） |
| `JitterTolerance` | 周期抖动告警阈值，超限计入 `JitterBreaches` |
| `MaxBacklog` | 单周期最多消费的上行帧数 |
| `TrajCapacity` | 轨迹缓冲容量（满则拒绝，不无界增长） |
| `Motion` | 位置范围、最大速度/加速度/力矩 |
| `SafetyLim` | 硬限位、力矩上限、软限位余量 `SoftMarginRad` |
| `Feedback` | 看门狗超时、最大可信单步位移、允许时间戳回退 |
| `Bus` | 容量、丢弃策略、最大帧龄 |
