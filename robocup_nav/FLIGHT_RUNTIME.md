# EV、起降与故障监督集成运行链

最新实机记录见汇总第29节：2026-09-25仅模式入口已实际发EV/地面保持流，但拨档超时，模式命令0；未完成握手，最终恢复Position已确认。新增GCS监看门禁和显式60 s有界等待候选，全量141单测；60 s选项尚未实机使用。下文“未运行”是旧阶段记录，不能覆盖本次真实流量。

## 最新：受限台架握手出口已实现但未运行（汇总第 28 节）

新增独立 `handshake-bench` 入口及 `HANDSHAKE_BENCH_REVIEW.md`。
它与默认影子入口不同：明确授权后可实际发送 EV、地面保持心跳/设定点和一次 OFFBOARD 请求，**不能解锁/起飞**。
两段式 Position 基线→预流→提示拨档解决 RC 与 PX4 状态到达顺序问题；来源组件 191、独立发送边界、退出后人工恢复 Position 检查已实现。
20 个两段式隔离 DDS 场景分两批通过，全量 139 单测；真实域出口、实机模式应答和恢复步骤未验收。
仅运行 `bash run.sh handshake-bench --help` 不接硬件；执行真实入口需要另行授权，当前未运行。
下面第 27 节“尚无真实出口”等是历史状态，以本节为准；live=false 仍保持，不能理解成新台架入口不会发送模式控制。

## 最新：不解锁握手影子/DDS 入口（汇总第 27 节）

`bash run.sh handshake-runtime --aux-params /home/cfly/param.params.txt --duration 30`
只读真实遥测、输出固定影子话题；`--exercise-controller` 也不会发送真实模式命令。
`bash run.sh handshake-dds-check` 在 domain 177 合成输入上验证实际订阅、序列化与状态机。
匹配的成功 ACK 和请求后的新鲜 Offboard 状态必须同时存在；完成或故障后停止全部影子 EV/控制输出。
模式转换期间由独立协调层管理 EV，禁止复用真实 EV-only 入口来执行握手。
本轮 23 个 DDS 场景分两批通过，全量 130 单测；尚无真实控制出口或实机握手验收。
实际 RC 先切入 Offboard 与候选要求 Position 基线的启动顺序、独立命令身份等仍需在真实出口设计中闭合。

下面保留此前阶段记录；“尚未接到 runtime/DDS”等历史表述以本节和汇总第 27 节为准。

最新补充见汇总第 26 节：最终 EV 门禁/共享锁版本实机 45 s 复跑通过（1285 帧，独立连续预飞 41.91 s）。
新增纯逻辑 `disarmed_handshake.py` 及 `bash run.sh disarmed-handshake-check`，10 个离线场景、全量 121 单测通过。
该候选只生成原地保持/OFFBOARD 意图，禁止 ARM/LAND/执行器命令；尚未接到 runtime、DDS 或任何真实控制话题。
后续接入必须解决源时间戳/ACK/EV 门禁协同，不能简单复用要求 Position 的 EV-only 边界。

最新状态（2026-09-24）：默认入口仍为影子控制；新增独立 EV-only 卸桨入口，详见汇总第 25 节。
`bash run.sh flight-bench-check --prop-off --params /home/cfly/param.params.txt --duration 45`
协调现有 Agent、相机、真实 EV 与独立只读预飞观察器；Agent 和容器须已运行。
单独 `flight-bench --prop-off --aux-params /home/cfly/param.params.txt --duration 45`
要求已有相机输入。两个入口只放行真实 EV，绝不放行真实命令、心跳、设定点；live 必须 false。
协调入口与默认 runtime 共用文件锁。门禁检查未解锁、Position、急停关、新鲜遥测及发布者所有权。
113 单测通过；实机 EV-only 1224 帧、独立预飞连续 41.55 s 通过。
实机后新增 nav_state==2 门禁/共享锁已在第 26 节完成最终版本卸桨复跑。不是起降许可。

2026-09-23。当前是完整的**影子运行入口**，没有 live 模式，不解锁真机。
平台保持 `live_flight_enabled=false`；若不是显式 false，CLI 拒绝运行。

2026-09-24 更新：开关双时间戳校验、Offboard 档启动意图、预热期间接管退出已补齐；
全量 90 单测、15 个隔离 DDS 场景通过。原生 DDS 遥测改动仅准备到只读候选补丁，
未编译/刷写、未在真实飞控接通开关输入。详见 EV_BENCH_SUMMARY 第 22 节。
操作员随后暂不接受固件改动，替代路径评估见该汇总第 23 节；优先评估现有 DDS
AUX 镜像，须先核对最新映射及执行器/云台副作用，尚未改参或接入，不能当作已可用输入。

后续实机四步核验已完成，操作员已调 AUX，源码已加入可选 `--aux-params` 影子输入。
默认仍为原生 DDS；下面的 AUX 入口只作影子控制，不创建 native 开关话题或真实飞控输入。
当前导出文件是否已更新必须实际核对，不能直接沿用调整前 AUX=0 的快照。

最新已收到调整后导出（SHA256 `c1f8b262759170ee02fd4827dffb1fd3a1d973d92f3f59a1145fe62a3dcddfdd`）。
`flight_runtime_20260924_202658_d31cc3` 实际 AUX+VIO 输入 20 s，494 帧影子 EV、779 个派生开关样本，
无故障/真实输入发布者，Position/未解锁条件下 WAIT；AUX 17 个 DDS 场景、全量 104 单测通过。
持续开关影子接入已完成，但没有真实控制出口或放飞批准。

## 组成与输出

`flight_runtime_core.py` 将 FlightEvAdapter 与 FlightSupervisor 放在同一会话内，
`flight_runtime.py` 负责 ROS 输入、时间检查、发布者审计和消息序列化。

| 输入/组件 | 处理 | 输出/后果 |
|---|---|---|
| VIO + tracking | 地面会话预热、FRD 转换、质量/跳变/超时检查 | 影子 VehicleOdometry；视觉失效锁存停发 |
| PX4 status/local/flags/land/RC | 每路源时间戳与接收时间分别检查 | 新鲜原子控制快照；重复源包不续期，时钟回退锁存 |
| manual_control_switches | 模式切换、kill 输入及 2 s 新鲜度 | 接管退出；急停进入 KILLED；缺失时不启动 |
| 发布者审计 | 0.1 s 审计，0.5 s 租期，冲突锁存 | 冲突时退出控制并停影子 EV，不竞争发 LAND |
| 起降监督 | 保持当前 PX4 local XY/heading、限速爬升、悬停、LAND | 影子心跳、设定点、命令 |
| vehicle_command_ack | 按命令及发送时间匹配 | 拒绝应答终止；接受应答不能替代实际 mode/armed/landed 状态 |
| 电池告警 | 记录 warning 数值 | 仅告警，不额外触发 LAND |

真实输入模式只订阅 `/fmu/out/*`、VIO 与 tracking。
默认 `flight-runtime` 所有发布话题固定在 `/robocup/flight_runtime_shadow/*`，不创建 `/fmu/in` 发布者；独立 EV-only 入口仅例外放行真实 EV。
节点禁用全局 ROS remap 参数，CLI 不提供任意话题前缀/ROS 参数或 live 开关。
本地文件锁阻止两个同模式 CLI 同时运行；DDS 审计进一步检查输出/输入重复发布者。
审计属于尽力而为的 ROS 图检查，不是分布式控制权的原子保证。
当前真实输入模式要求 `/fmu/in` 发布者为零，不能与台架 EV 注入同时运行。

隔离模式固定 domain 177、localhost、`/robocup/runtime_test/input/*` 和
`/robocup/runtime_test/output/*`，同样不会出现 `/fmu/in` 发布器。

## 故障语义

- 视觉失效：EV 质量/时间/跳变锁存停发；停止位置流/Offboard 心跳，状态新鲜且
  已解锁时输出 LAND 意图。画面恢复不会恢复旧会话。
- 通信失联：停止控制输出；状态过期时不声称 LAND 可达。EV 在安全遥测过期后
  锁存停发。依赖飞控自身保护，恢复链路也不自动续飞。
- 人工接管：模式输入或 PX4 离开 Offboard 可触发 HANDOVER；不抢回控制。
- 急停：kill 输入触发 KILLED，无后续自主心跳/设定点/命令；不会把物理 kill
  转换为 LAND 或重启电机。电机实际切断由飞控和已配置的物理开关负责。
- 命令拒绝：永久拒绝/不支持/失败/取消触发终止；临时拒绝仍受原状态机时限约束。
  ACK accepted 不会单独推进解锁/起飞；必须看到新鲜对应飞控状态。
- 低电：按操作员明确选择，仅记录告警，COM_LOW_BAT_ACT 保持 0。
  不干预 PX4 已经实际触发的其他 failsafe，不保证耗尽电量仍可飞。
- 所有输出目前均为影子消息；不实现电机控制、舵机驱动或参数写入。

## 运行

```bash
cd /home/cfly/ros2_ws
# 无硬件动作的单元测试
bash robocup_nav/run.sh flight-test

# 完整运行节点 + DDS 合成机体，默认十个场景
bash robocup_nav/run.sh flight-runtime-check

# 相机/纯视觉 VIO 与飞控遥测已运行时，55 秒影子观察
bash robocup_nav/run.sh flight-runtime --duration 55

# 允许影子控制器尝试启动；仍没有任何真实飞控发布器
bash robocup_nav/run.sh flight-runtime --duration 55 --exercise-controller
```

`--exercise-controller` 只是请求在所有新鲜证据就绪时启动一次影子状态机；
它不会提供假预飞/假开关数据，也不会绕过检查。不带此选项只观察并发布影子 EV。
2026-09-24 起还要求新鲜 `mode_slot=6`、`kill_switch=3`、`offboard_switch=0`。
Position 档不会启动；STREAM 预热期间拨回 Position 同样锁存 HANDOVER。
相机、XRCE Agent 由已有有界流程管理，此入口不会启动/重启/停止它们。
报告保存到 `evidence/flight_runtime_*/report.json`，包含输入计数、状态变更、
ACK、EV 故障时刻、输出计数和启动阻塞原因。
CLI 返回 0 只表示该窗口影子 EV 正常且无运行错误，不是起降通过；务必同时看
state、start_blockers 和 exercise_requested。

### 可选 AUX 镜像输入（仍然 shadow-only）

```bash
# 必须使用操作员调整后的真实参数导出；不会从此文件写入飞控
bash robocup_nav/run.sh flight-runtime --duration 30 --aux-params /home/cfly/param.params.txt
# 隔离合成机体，只使用显式测试 fixture，不读取真实飞控
bash robocup_nav/run.sh flight-runtime-check --switch-source aux
```

参数契约要求 AUX1=CH6、AUX2=CH5、原模式 CH6/急停 CH5/独立 Offboard 开关未分配、
kill 阈值 0.75、RC 输入模式及六档位匹配；不匹配直接拒启。报告保存导出文件 SHA256。
实体 RC 来源、发布/采样双时间戳、重复包、断流、无效值和阈值歧义由独立解码器校验。
派生正常状态需连续新样本稳定 0.1 s；明确急停开不增加稳定等待；AUX 开关新鲜度上限 0.5 s。
报告 `switch_source=derived_aux_mirror`，不伪装 native。选择 AUX 后不订阅原生开关，
图中出现原生开关发布者亦视为来源冲突，不自动回退。所有故障和接管退出策略继续保留。
即使 AUX 门限通过，没有 VIO/实际 EV 融合、预飞条件或 Offboard 档启动意图时仍保持 WAIT。
没有实际 EV/VIO 的单独开关接入检查可能退出非零，应看具体字段，不能称完整运行链健康。

## 已执行的验收

| 证据 | 结果 |
|---|---|
| `flight_runtime_check_20260923_215227_8c7c26` | 八种整链 DDS 场景通过 |
| `flight_runtime_check_20260923_215359_6d7995` | 加强通信失联检查通过：确认 EV 因 PX4 safety telemetry stale 锁存 |
| `flight_runtime_check_20260923_215551_14dbba` | 重复发布者退出、ACK 接受但无状态变化两项通过 |
| `flight_runtime_20260923_215548_a6fd38` | 真实输入 55 s，1310 帧影子 EV，无故障，状态 WAIT，真实发布者 0 |
| `alignment_preview_20260923_215452` | 上述真实 VIO 输入来源，纯视觉、有界预览，无真实 EV 注入 |

十种隔离场景：正常起降、视觉丢失、通信失联、人工接管、急停、命令拒绝、
低电仅告警、缺少开关输入、发布者冲突、ACK 接受但状态不变。
所有感知/飞控数据均通过 ROS 订阅进入实际运行节点，没有直接塞入健康快照。
机体动力学、估计器融合和飞控失联反应依然是合成的，不是真 PX4/Gazebo SITL。

单元测试新增 11 项，连同原测试全量 **82 项通过**。首轮整链测试曾发现
审计时间晚于本周期时间导致误判过期的问题，已修正；上述成功证据来自修正后。
第一次真实输入试验 `flight_runtime_20260923_215403_7820b0` 生成 1389 帧后因
tracking 超时锁存，会话时序与有界相机先退出相符；不作为健康窗口通过。
调整两进程启动时序后，1310 帧的完整健康窗口通过。

## 当前明确边界

真实输入报告没有收到 `/fmu/out/manual_control_switches`：默认 DDS 未导出此路。
QGC 三次静态样本已证明映射正确，但不能当持续、可供机载监督使用的开关输入。
本轮无真实 EV 注入，所以 PX4 水平定位/预飞未通过也是预期结果；之前有 EV 的
63.59 s 预飞窗口证据仍有效。真实运行节点保持 WAIT，既没有伪造开关关闭状态，
也没有自动启动控制。

下一步真实控制握手之前，需要提供持续模式/急停输入通道并验证源时间与语义，
完成实际飞控命令握手/独占输出入口的单独审查。不要将 shadow 话题直接重映射到
`/fmu/in`。外参、急停电机效果及空中失效落地的未验收项仍保留。
