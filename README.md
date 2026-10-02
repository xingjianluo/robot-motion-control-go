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
| `internal/stream` | **连续流式喂点队列 + 在线速度倍率**：有界排队、段速可行性、加速度斜坡整形、倍率原子生效、挤掉策略 |
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

## 4A. 连续流式喂点与在线速度倍率（手轮/示教模式）

在原有“单目标、逐周期下发”能力之外，新增一条**并存且互不影响**的
流式通路，对应 `internal/stream` 与控制器三个新入口：

| API | 作用 |
|-----|------|
| `Controller.SubmitStreamPoint(Point{At, PositionRad})` | 连续喂入一个位置目标点（名义时刻 + 位置） |
| `Controller.SetSpeedOverride(factor)` | 在线调整整体倍率（如 0.5 → 1.2 → 0 → 1.0） |
| `Controller.SpeedOverride() / StreamQueueLen()` | 观测当前倍率与排队深度 |

原有的 `SubmitCommand / Estop / Recover / Step` 完全保留，**单目标用法、
线协议帧格式、V1/V2 行为均不变**（详见第 6 章）。

### 4A.1 连续拼接，不在每个点停车

可以在上一段还没走完时就把下一个点塞进来，系统按提交顺序一路连续
走完：每个控制周期，`stream.Planner` 输出一个**整形后的绝对位置设定**，
相邻周期之间由“一周期前瞻斜坡”限速：

- 巡航期望速度 = 提交时设计的段速 × 当前倍率；
- 实际下发速度每周期最多变化 `MaxAccel × Period`，且绝对值钳在
  `MaxVelocity` 内——**无论怎么接点、怎么突变倍率，速度/加速度都不会
  被顶到限值以外**；
- “剩余位移 / 剩余墙钟时间”兜底项保证在点的名义时刻**精确到点**，
  不冲过、不拖尾；
- 到达拼接点不停车；走完整条流后进入**末点保持**：无限期持续下发
  末点位置（对外速度语义为 0），直到续点或安全动作清除，防止下游
  带着残余速度漂移。

### 4A.2 时间重叠 / 中途插入：行为说清楚，绝不悄悄丢点

点的 `At` 是名义排程时刻，只用于排队顺序、段速核算与迟到判定。
流式校验顺序固定，拒绝原因**恒为 `command_rejected|stream|…`**，
与普通单目标越界原因码可区分：

| 原因码 | 触发条件 |
|--------|----------|
| `stream_position_out_of_limit` | 目标点超出硬位置范围 |
| `stream_late_point` | 点时刻不严格晚于当前时刻（迟到 / 与当前时刻重叠） |
| `stream_backwards_time` | 点时刻早于排队尾点（倒流） |
| `stream_segment_too_fast` | 相邻点段速按**当前倍率**核算超过最大速度（当前状态根本走不出来） |
| `stream_queue_full` | 排队到上限且策略为 `reject_new` |
| `stream_evict_infeasible` | `evict_oldest` 挤掉最旧点后剩余段仍不可行 |
| `override_out_of_range` | 倍率非有限值或超出 `[MinOverride, MaxOverride]` |
| `override_infeasible` | 该倍率会把任一排队段顶破限速（不靠静默裁剪蒙混） |

> 上表是 Planner 自身的拒绝码（控制器安全门控通过后才会走到）。配置了
> 软限位余量（`SoftMarginRad>0`）时，越过软限位带的位置点会**先**被控制
> 器安全预检以 `safety|safety|soft_limit_violation` 拦下——软限位带在硬
> 限位内侧，因此安全预检优先于 `stream_position_out_of_limit`，这是
> “安全优先于普通控制”的固定顺序，不影响两类原因码本身的可区分性
> （见 4A.7）。

- 与排队尾点**同一时刻**的点做**替换**（一个执行时刻只允许一个目标），
  计入 `StreamStats.Replaced`，不是报错也不是静默丢点；
- 倍率缩放的是实际推进速度：倍率 > 1 可能提前到、< 1（含 **0 = 暂停
  推进但持续下发当前位置保持**）会延后到，各段设计巡航速度不变；
  改倍率时整链排程时刻被重新钉定，段速与积分位移始终自洽。

### 4A.3 在线倍率：原子生效、当前段与排队段同时作用

- `SetSpeedOverride` 只改倍率状态，**下一个完整控制周期**才采用新值，
  因此任何周期都不可能“半周期旧倍率、半周期新倍率”；
- 改倍率瞬间，活动段锚点钉到“当前实际位置/当前时刻”，剩余整链按新
  倍率重钉时刻，所以对**正在走的段和后面排队的段同时生效**；
- 提交前先做整链段速可行性核算：任一段在新倍率下超速即
  `override_infeasible` 拒绝，绝不把危险调速伪装成接受；
- 非 RUN（安全门控）下调倍率 / 喂点一律 `safety|controller|outputs_disabled`。

### 4A.4 排队容量与溢出策略（内存永远有界、可观测）

流式队列深度上限为 `Config.Stream.Capacity`（含当前活动点），满时策略由
`Config.Stream.FullPolicy` 明确指定：

- **`reject_new`（默认）**：拒绝新点（`stream_queue_full`），不丢任何旧点；
- **`evict_oldest`**：挤掉队列中**最早的非活动点**（当前活动点绝不被挤，
  避免轨迹不连续），合并相邻两段并重新校验；不可行则
  `stream_evict_infeasible`。挤掉计入 `StreamStats.Evicted` 与
  `LastEvent`，可观测、可复现。

`Snapshot.StreamStats` 暴露 `Appended/Rejected/Replaced/Evicted/Reached/
OverrideChanges/OverrideRejects/LastEvent`，内存占用永不超过容量。

### 4A.5 与安全状态、恢复的互动（结果确定、可复现）

- 任意安全动作（`ESTOP / COMM_LOSS / TORQUE_FAULT / SOFT_LIMIT`，即
  任意非 RUN 状态）下，**未走完的流式队列一律清空、倍率一律复位 1.0**，
  并记录 `safety|controller|stream_evicted_oldest`——规则固定：
  安全优先，不保留任何待执行运动，也不静默丢点；
- 恢复后从**干净状态**开始（队列空、倍率 1.0），需要重新喂点；
  不存在静默恢复或静默丢点；
- `Config.RequireObservedRecovery=true` 时，`Recover` 除检查调用方给的
  `RecoveryConditions` 外，还必须与系统当时的**真实观测**交叉核验：
  看门狗仍判失联却口头声称健康、最近反馈仍超力矩/越硬限位/带故障码却
  声称正常，一律以
  `safety|controller|recovery_conditions_contradict_observation` 拒绝。
  无观测证据时不反驳（由显式条件与人工确认负责），避免无证据误拒。
  该开关默认 `false`，**保持对既有调用方/用例的兼容**。

### 4A.6 流式与单目标在同一会话内如何共存（可来回切换）

两条通路共用同一把控制器锁与同一条下行总线，但**任一周期只由其中一条
通路下发**，规则固定、可随时来回切换：

| 当前情形 | 提交 `SubmitCommand`（单目标） | 提交 `SubmitStreamPoint`（流式） |
|----------|-------------------------------|----------------------------------|
| 空闲（两条通路都没用过） | 正常入轨迹缓冲、逐周期下发 | 以当前反馈位置建锚点，开始流式运动 |
| 流正在走（队列非空） | **接受并入轨迹缓冲排队**，不打断流；本周期仍由流式下发 | 正常续点 / 替换 / 拒绝（规则见 4A.2） |
| 流已排空、处于**末点保持** | **接受即完成“通路交接”**：退出流式保持，下一完整周期起单目标缓冲正常弹出下发 | 以保持点为新锚点续一段流 |
| 任意非 RUN 安全状态 | `safety\|…\|outputs_disabled` 拒绝 | `safety\|…\|outputs_disabled` 拒绝 |

- **交接原语**：流排空后 Planner 处于“末点保持”（持续下发末点位置）。
  单目标命令**被接受的那一刻**调用内部 `ReleaseHold` 退出保持、清空流的
  运动记忆（不改倍率、不记故障）；此后 `advanceTrajectoryLocked` 不再被
  流式保持挡住。活动流（仍有排队点）上 `ReleaseHold` 是**空操作**，
  正在走的流绝不会被单目标命令打断，单目标命令在轨迹缓冲中等待，流排空
  后再由一条新的单目标命令完成交接即可生效。
- **被拒命令不触发交接**：安全门控、轨迹校验、软限位预检任一不通过时
  命令根本不会入缓冲，也就不会触碰流式保持状态——因此“命令被拒”和
  “通路切换”永远不会同时发生。
- 原有单目标用法（`SubmitCommand` 逐周期重发、同刻替换语义）、线协议
  帧格式与 V1/V2 行为**完全不变**；流式只发 `mode=position` 帧，不引入
  新的线格式字段。

### 4A.7 为什么被拒的流式点不会改动现有运动状态（拒绝是“事务”）

`Planner.Append` 的所有校验（位置硬限位、迟到、倒流、段速可行性、容量
策略——包括 `evict_oldest` 的合并试算）都**先在副本上完成**，只有全部
通过后才有唯一的提交点去改 `pending/gaps/cruise/锚点/保持` 状态：

- 被拒点只影响它自己：仅留下 `Rejected/LastEvent` 这类**有意的观测
  计数**；正在保持的末点位置、已排队的点、当前倍率、安全状态在拒绝
  前后**逐字节一致**，后续周期照常下发保持位置命令；
- 两类“走不到”的拒绝在原因码上明确分开，且都恒为
  `command_rejected|stream|…`：
  - `stream_segment_too_fast`：位置在限内、时间也合法，但按当前倍率
    核算的段速超过最大速度——**“按当前状态这个点根本走不出来”**，
    在入口就拒，而不是等周期里顶到限速；
  - `stream_position_out_of_limit`：**“普通越界”**，目标位置超出硬
    位置范围（注意：配置了软限位余量时，越过软限位带的位置点会被安全
    预检以 `safety|safety|soft_limit_violation` 抢先拦下——安全优先，
    不落入普通流式校验）；
- 同理，`SetOverride` 的可行性核算被拒时不会提交重排，活动段锚点与
  排程时刻保持不变。

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
- **流式喂点只使用位置模式**（`mode=position;pos=…`），V1 与 V2 均支持，
  不引入任何新的线格式字段；在线倍率是控制器内部状态、**不在线上传输**。
  因此流式能力对既有 V1/V2 部署与抓包完全向后兼容。

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
| `command_rejected` | 命令主动被拒（越界/矛盾/不安全） | `position_out_of_limit`、`accel_out_of_limit`、`stream_segment_too_fast`、`stream_queue_full`、`override_infeasible` … |
| `protocol_error` | 帧到达但协议不接受 | `unknown_frame_type`、`missing_field`、`version_unsupported` … |
| `communication_fault` | 链路问题 | `feedback_watchdog_timeout`、`queue_full_dropped`、`injected_frame_loss`、`position_jump`、`stream_drained` … |
| `safety` | 安全联锁触发或恢复被拒 | `estop_pressed`、`torque_overload`、`recovery_denied`、`recovery_conditions_contradict_observation`、`stream_evicted_oldest` … |

`Source` 进一步指明首次产生组件（trajectory/**stream**/protocol/bus/
feedback/safety/controller）。流式相关拒绝恒为 `command_rejected|stream|…`，
与普通单目标越界（`…|trajectory|…`）明确区分。测试依据**稳定原因码**
断言，而不依赖错误文案。

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
- **流式喂点/倍率**（`internal/stream/stream_test.go`、
  `internal/stream/atomic_test.go` 与 `internal/control/streaming_test.go`、
  `internal/control/coexist_test.go`）：连续拼接不停车、拼接处速度/
  加速度不越限、迟到/重叠/倒流与同时刻替换、在线倍率（0.5→1.2→0→1.0）
  全程不越限且精确到点、越界倍率 `override_out_of_range` 与会顶破限速的
  `override_infeasible`、排队 `reject_new`/`evict_oldest` 两种容量策略、
  运动/调速中触发安全停机后队列与倍率被确定性清空、恢复必须真实观测核验、
  **Append 拒绝的事务性**（末点保持/有排队点/队满三类情形下，迟到、倒流、
  段速不可行、硬限位越界等被拒点不改动保持位置、队列与倍率；
  `stream_segment_too_fast` 与 `stream_position_out_of_limit` 原因码可分）、
  **流式跑完后单目标照常生效并可来回切换**（流→单目标→流→单目标、
  流活动期排队的单目标在交接后被执行）、
  以及 **200 周期 喂点/调速/急停/Step 四方并发 + race 检测**；
- **反馈**：重复/乱序/跳变不污染基准、看门狗“失联≠偏差”、V1 兼容；
- **安全**：优先级与抑制、急停恒赢、恢复条件全列举、软限位显式清除；
- **控制器场景**（`internal/control/scenarios_test.go`）：
  上电 ESTOP→恢复→跟踪、四类故障不合并、看门狗失联与显式恢复、
  **急停并发**（200 周期竞争 + race 检测）、积压上限、力矩锁存、V1/V2 共存；
- **可解释用例** `TestExplainableDecisionFlow`：逐步打印
  **输入 → 判定依据（类别|来源|原因）→ 最终状态快照**，并 Dump 完整审计轨迹。

### 9.2 端到端演示

```bash
make demo            # 或 go run ./cmd/demo；退出码 0 表示全部场景正常跑完
echo $?              # 期望：0
go run ./cmd/demo > /tmp/run1.txt && go run ./cmd/demo > /tmp/run2.txt
diff /tmp/run1.txt /tmp/run2.txt && echo "逐字节一致"
# 等价自动化校验：
go test ./cmd/demo/ -run 'TestDemo' -count=1 -v
```

12 个场景顺序演示：上电 ESTOP 拒绝命令 → 条件不全拒绝恢复 → 满足条件+人工
确认 → 连续速度轨迹与反馈闭环（1 rad/s×0.3s=0.3 rad，误差 0）→
越界/矛盾命令被区分拒绝 → **连续流式三点拼接（0.1/0.3/0.2，不在点间停车，
末点 0.20 精确保持）** → **在线倍率 50%→120%→暂停→100%（越界倍率被拒，
末点 0.40 精确保持）** → 连续丢帧触发看门狗 COMM_LOSS → 链路自愈不自动
复位 → 显式恢复 → 力矩超限 TORQUE_FAULT 锁存 → 缺人工确认被拒 →
**两拍真实健康帧刷新观测后**人工确认恢复 → 急停与周期/命令并发
（10 拍并发提交命令均被接受并下发，随后并发急停，最终必为 ESTOP
且急停后新增下行命令为 0）→ 积压/注入/抖动可观测性。

确定性保证：

- 全程只用手动推进的假时钟；并发场景里命令 goroutine、急停 goroutine
  与周期 goroutine **仍然真实并发**（`-race` 照常检测数据竞争），但用
  无缓冲 channel 作为会合点把交错顺序钉死——**不使用墙钟 `Sleep`**
  （旧版用 `time.Sleep` 决定急停先后，正是输出不确定的根源）；
- 场景 10 注入超力矩反馈帧时序号**紧跟设备真实序号**，随后两拍设备
  健康帧（力矩 0、序号连续）能正常被接受，恢复时“力矩正常”的声明才有
  真实观测证据；校验没有任何放宽：该拒的（软/硬限位、矛盾命令、越界
  倍率、COMM_LOSS/TORQUE_FAULT 下的命令、缺人工确认的恢复）依旧被拒。

连续运行 `make demo` 两次输出**逐字节一致、退出码均为 0**，便于稳定
复现；`go test ./cmd/demo/` 会在进程内直接断言“全部场景跑到 + 退出
nil + 两次输出逐字节相等”。

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
| `Stream.Capacity` | 流式排队深度上限（含活动点，默认 32；满则按策略拒绝/挤掉，内存有界） |
| `Stream.FullPolicy` | `reject_new`（默认，拒新不丢旧）/ `evict_oldest`（挤掉最早非活动点） |
| `Stream.MinOverride` / `MaxOverride` | 允许的在线倍率范围（默认 0 ~ 2；**0 = 暂停推进但持续位置保持**） |
| `RequireObservedRecovery` | 恢复是否必须与系统真实观测交叉核验（默认 `false` 保持兼容；流式演示置 `true`） |
