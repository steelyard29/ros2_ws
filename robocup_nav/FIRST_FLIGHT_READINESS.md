# 首次起降测试准备与验收记录

**2026-09-28更新（第44节）：修复READY边沿重复回调竞争及就绪撤销处理，217项单测与正常/缺ARM ACK/急停三项隔离DDS回归通过。** 软件入口保留；目前等待今天现场状态确认，校准后新EV会话和现场放行仍待完成。第43节15场景为9月25日版本历史证据。live=false及四项verified=false不变。

**当前以第43节软件交付为准：受保护的真实无货起降入口已实现，现场校准、真实验收及当次放行由操作员完成。** 新操作说明 `FIRST_FLIGHT_OPERATOR_RUNBOOK.md` 包含离线release检查和明确标注的真实执行命令。默认live=false及四项verified=false未改；历史“真实出口尚未实现”不再是软件待办。不要把软件完成或合成测试当作当前飞行许可。

最终验证：215项单测、同一代码版本15项隔离合成DDS场景通过，报告 `evidence/live_flight_check_20260925_213544_d9af98/report.json`；不是PX4 SITL或实机飞行验收。

**第42节更新：起降输出候选检查器已实现并接入合成DDS路径，184项单测及五项全链场景通过。** 详见 `FLIGHT_DISPATCH_REVIEW.md`。这是发送边界的软件准备，不是已完成的真实ARM/LAND入口；未开展新的硬件验收，不能将剩余工作压缩为“只剩校准”。第41节清单继续有效，live=false及四项verified=false保持。

**最新状态以汇总第41节及 `FIRST_FLIGHT_RELEASE_CHECKLIST.md` 为准。** 三项收尾的离线准备已推进：175项单测、五项隔离合成DDS全链场景通过；新增校准参数基线保全/差异审查，独立预飞观察器强化源时间与最终状态门禁。用户当前无法校准，独立舵机测试已停止；未开展校准后实机复核或新的硬件故障验收。真实ARM/LAND执行出口仍未实现/审查，不能仅改live开关起飞。live=false及四个verified=false保持。下方为历史阶段记录，不能将历史“最新”或“仅剩”表述当作当前放行结论。外围网格/横杆已由用户确认，安装一致性已记录，不重复要求同项确认。

**第39节：用户确认的首飞场地已完成60秒独立静置视觉检查。** VIO约29.40Hz、跟踪全1、最大观测位置变化1.48cm；IMU约199.98Hz，启动消息验证通过。实际IR图像有接缝/黑色图案/远处标志，不是全白无纹理；网格/横杆与净空关系仍待确认。Motion Module告警仍有，地面静置不替代升高/运动验证。检查已退出，无飞控控制输出，live=false保持。

**第37节：IR稳定条件、参数响应超时、实际组合IMU启动验证已实现并通过独立相机复测，全量165单测通过。** 新检查确认403帧连续有限IMU/2.001秒后才报告成功；45秒采集约200Hz。Motion Module告警本轮复现但数据持续，根因未解决；左IR最大间隔约200ms也须保留。不是飞行放行，live=false保持。第36节“本次无告警”仅适用于其当次记录。

**第36节最新状态：用户再次确认现场急停功能测试无问题，按操作员确认记录，不重复同项物理测试。** 程序第35节窗口超时事实不改写为通过。独立45秒相机检查收到7597帧组合IMU，约200Hz、最大间隔10ms、时间戳无倒退；VIO约29.7Hz且跟踪全有效，本次无Motion Module failure。仅确认本次采集正常，不代表间歇告警根治或标定完成。相机已停止，live=false不变。

**第35节更新：首轮Position急停停发实机入口已运行，但60秒内无有效急停事件，按设计超时停发，验收未通过。** 实发1894帧EV与3102组地面保持流，命令0；退出后433帧只读确认Position/急停关/未解锁。本轮相机已停止，不自动重试。下述第33节“尚未执行硬件”为历史记录；正常Offboard模式握手已通过的结论保持不变。

**第33节最新进展：Position地面保持流下的独立急停停发验收入口已实现，尚未执行硬件。** 四类隔离DDS回归符合预期，全量156单测通过；真实入口需单独授权，不请求Offboard且禁止全部模式/解锁/降落命令。故障入口目前仅支持kill，其他场景待准备；不把软件结果计为实机或飞行保护验收，live=false不变。下文第32节“尚待实现”已由此更新。

第32节补充：故障分层验收清单见 `FAULT_ACCEPTANCE.md`；五项隔离DDS故障回归及全量142单测通过。正常握手提示已修正，真实故障验收入口尚待实现/审查，未进行硬件故障注入。四个物理verified仍为false，不因测试通过而放飞。

**最新状态以汇总第31节为准（2026-09-25）：第三轮卸桨、未解锁真实模式握手已通过。** 证据 `handshake_bench_20260925_144040_40cccc`：唯一一次OFFBOARD请求获接受ACK，确认新鲜Offboard状态并人工恢复Position，passed=true；追加10秒只读391帧确认Position/急停关/未解锁。不是起降验收，live_flight_enabled=false保持；故障保护实机验收、飞控标定及标定后EV/预飞、外参/场地等缺口仍在。相机Motion Module告警仍未解决。下述第29节等为历史状态。

**第29节最新实机结果（2026-09-25）：真实台架入口发出720帧 EV 和1101组地面保持流，等待拨档超时，模式命令0、未解锁，握手未通过。晚到拨档后已用原生开关及最终DDS采样确认恢复 Position/急停关/未解锁。**
相机/控制已退出，Agent11225保留，不自动重试。新增60 s有界等待候选、GCS监看门禁，全量141单测通过；相机日志的Motion Module failure需单独复核，不能记IMU正常。最新状态以汇总第29节为准，以下为历史记录。

第 28.6 节覆盖下述 GCS 失联停点：重新连接 QGC 后，10 s 只读采样 450 帧 RC 有效，gcs_connection_lost=false，未解锁、Position/急停关；尚未发送模式握手输入。QGC 页面刷新需以消息频率/计数核实。

当前现场停点（第 28.5 节）：用户已限定卸桨/不解锁并确认命令来源独占；10 s 只读基线 451 帧 RC 有效、未解锁、Position/急停关，但 GCS 持续失联。需先恢复 QGC 实时监看并只读确认；真实模式握手尚未执行。

**第 28 节更新：受限卸桨模式握手真机入口已实现但未执行。两段式顺序/组件 191/发送边界通过 20 个隔离 DDS 场景与全量 139 单测。**
下一步需单独授权仅模式握手并确认现场条件；它不会解锁或起飞。真实模式应答、人工恢复、硬件故障保护及标定等仍待验收，live=false。

**最新软件进展见汇总第 27 节：不解锁握手影子/DDS 接入已完成，23 个隔离 DDS 场景分两批通过，全量 130 单测。**
尚无真实控制出口、真实模式握手或起降；不将此作为准确 PX4 固件验收。标定后置安排不变，解锁前必须完成。

操作员最新确认（第 26.4 节）：QGC 加速度计/陀螺仪/水平标定组尚未完成，按要求留到后续现场收尾，任何解锁/起飞前必须完成；D435i 与飞控安装自此前位移/偏航核验后未变化。先推进影子/DDS 软件，不以“最后再做”绕过飞控门禁。标定后重开 EV 会话并复核方向及预飞；安装未变化不等于旋转外参已校准。

第 26 节更新：最终 EV-only 门禁/共享锁实机复跑已通过（1285 帧，独立连续预飞 41.91 s），该复跑不再待办。
用户确认独立舵机测试不移动机身/遮挡/释放，保持其运行；本轮未操作 GPIO。
新增不解锁握手纯逻辑，10 个合成场景及全量 121 单测通过；其影子/DDS 接入、真实模式握手仍未完成。
标定核验、真实控制出口审查、固件故障验收和装桨/解锁单独授权仍是首飞前置条件；live=false。

**最新状态以 EV_BENCH_SUMMARY 第 28 节为准，下面保留旧阶段记录；“最终版本待复跑”“握手未接 DDS”“真实出口未实现”等旧待办已被覆盖，但真机握手仍未执行。**
操作员确认电机编号/旋向、独立急停实际切断、防护罩安装均已验证；不等于其余标定通过。
EV-only 45 s 真机联调通过：1224 帧实际 EV，独立预飞连续 41.55 s，通过且始终未解锁。
最新 113 单测通过；实际控制发布者仍为 0。实机后补的 Position 状态门禁/共享锁待最终版本复跑。
首飞还需：物理/标定/坐标核验、独立受限真实控制出口及不解锁握手、准确固件故障保护验收、
场地和装桨/解锁单独授权。四个 verified 标志 false，live=false，不能据本轮预飞通过直接起飞。

2026-09-24 续进：统一台账见 `EV_BENCH_SUMMARY.md` 第 22 节。
开关双时间戳、Offboard 档启动意图和预热期接管已补齐，90 单测及 15 隔离 DDS 场景通过。
真实开关输入仍缺原生 DDS 导出；本地 PX4 工作树为 1.18 开发版，不可直接替换现场 1.16.2。
已提供只读 `flight-switch-contract` 报告/补丁准备工具，未编译/刷写。真实控制入口与物理验收未完成。

后续操作员不接受固件改动，改走 AUX 镜像并完成四步静态对照；最新汇总第 24.4 节覆盖上述
“开关输入缺失”停点。真实 AUX+VIO 影子接入通过（494 帧影子 EV、779 个派生开关样本），
104 单测、17 个 AUX DDS 场景通过。仍未做真实控制握手、物理急停或实际起降，live=false。

2026-09-23。范围：软件候选、隔离验证、真机卸桨遥测、首飞配置审查。
仍未授权装桨/解锁，`config/platform.yaml` 的所有核验标志保持原值，
`live_flight_enabled=false`。本机未写飞控参数。

## 1. 软件闭环：候选已实现，真实飞行集成尚未放行

- `flight_ev_adapter.py`：独立飞行 EV 策略，复用已验收的 FRD、时间戳、
  NaN 速度、质量与跳变保护。只允许地面建立会话和预热；正常解锁后继续输出。
  视觉异常锁存后不恢复。旧 `EvAdapter/ev_bridge` 仍一解锁就停。
- `flight_ev_shadow.py`：候选 ROS 发布器，输出固定为
  `/robocup/flight_shadow/vehicle_visual_odometry`。当前没有 live 开关。
- `flight_supervisor.py`：预流 2 s → Offboard 状态握手 → 解锁状态握手 →
  起飞 → 连续悬停 → LAND。默认测试目标 0.5 m、悬停 3 s，上升设定点速率
  0.2 m/s；这是未来测试候选值，本轮没有执行真实起降。
- `flight_messages.py`：将决策序列化成 PX4 消息；无串口/发布副作用。
  位置控制保持速度、加速度、yaw rate 为 NaN；LAND 经纬度等为 NaN，
  防止无意使用 (0,0)。无 force-disarm 路径。
- 垂直测试直接记录 PX4 当前 local x/y/z/heading，保持 XY/heading，向上减 Z。
  不假定 PX4 本地航向是地理北，不使用测距顶 Z。这不等于完成后续水平导航的
  ROS→PX4 坐标对齐。任何 XY/Z/heading/速度 reset counter 改变均终止。
- 视觉/估计器异常：停止 Offboard 心跳/位置流，状态新鲜且已解锁时请求 LAND；
  链路/状态过期则不声称命令可达，依赖 PX4 内部失效保护。恢复后仍锁存在 ABORT。
- 离开 Offboard 或人工接管后不重新抢控制。新鲜 land detector + 未解锁才能
  判定落地；不按低高度强制停电机。
- 实测 VehicleStatus 约 2 Hz、land/flags 约 1 Hz，因此状态有效期 1.5 s、
  land/flags 2 s；local/RC 为 0.5 s，控制循环停顿 >0.3 s 终止。
  这些是工程候选门限，未经过空中验证。

验证证据：

| 检查 | 结果 | 范围 |
|---|---|---|
| 新增飞行控制/EV 单元测试 | 14 项通过 | 包括非零原点/航向、上升限速、模式拒绝、失联/接管、EV 不恢复 |
| 配置审查单元测试 | 3 项通过 | 缺失数据、共享通道、RC exception、伪真值拒绝 |
| 全部 `test_*.py` | 70 项通过 | 不含真 PX4 动力学 |
| `evidence/flight_dds_20260923_210527` | 5 场景通过 | domain 177，仅 shadow 话题；真实 ROS DDS/消息类型，合成机体 |
| `evidence/flight_ev_shadow_20260923_210651` | 1349 帧影子 EV，quality 全 100，无故障 | 真实 VIO 输入；实际 EV 发布 0，未解锁 |

DDS 场景：正常起降、视觉丢失、链路丢失、人工接管、模式拒绝。
链路丢失场景的合成机体按测试假定自行降落，控制器仍为 ABORT，因为没有新鲜
遥测，不能将其写成真实断链保护通过。本机未发现已构建的 PX4 SITL 二进制，
此次不是 Gazebo，也不是 PX4 1.16.2 固件 SITL。

后续已将候选控制器、EV 发布器、命令应答与发布者审计接成统一**影子运行入口**，
见 `FLIGHT_RUNTIME.md`。新增 11 项单元测试，全量 82 项；十种整链 DDS 场景通过，
真实 VIO/飞控输入下 55 s 生成 1310 帧影子 EV，无故障，没有真实控制输出。
仍缺真实飞行运行入口的单独放行、持续模式/急停遥测、真 PX4 的 mode/arm/land
应答及失联动作验证、空中定位失效降落验收。不能只将 shadow 话题改名到
`/fmu/in` 就用于飞行。

## 2. 有 EV 的卸桨预飞：已通过本轮遥测门限

操作员明确确认：桨叶拆除、飞控上电未解锁、遥控与 QGC 就绪。
XRCE Agent 本轮启动 pid 18935，并保留运行，不依赖历史 pid。

- 只读基线 `evidence/px4_telemetry_20260923_205637`：无 `/fmu/in` 发布者；
  RC 有效、未解锁，气压融合开、测距高度关；无 EV 时 XY 无效、预飞不过。
  初始 GCS lost=true，后续有 EV 会话已为 false。
- 有界 EV `evidence/ev_inject_20260923_210046`：75 s，EV **1916** 帧，
  quality 全 100；VIO 约 29.4 Hz、相机 IMU 约 194 Hz；
  `pre_flight_checks_pass=true`、RC 有效、GCS 正常、arming_state=1。
- 独立只读 `evidence/preflight_readonly_20260923_210222`：100 s 观察，
  **连续 63.59 s** 满足新鲜预飞、落地、RC、位置/速度有效性、EV 融合和
  独占输入门限。只有 1 个 `vehicle_visual_odometry` 发布者，没有命令/设定点。
- EV 位置/高度/偏航融合开，速度融合关；气压辅助开，测距高度关。
- 会话结束后 EV 停发，XY/预飞恢复为无效；观察器最终 snapshot 是停发后的状态，
  report.passed 表示中间有满足持续时间的有效窗口，不表示退出时仍可起飞。
- 后续 `alignment_preview_20260923_210555` 仅为新候选影子验证提供输入，
  没有向飞控注入 EV；相机有界会话已结束。

这一项没有验证电机、控制方向、急停开关或真机 Offboard 握手。

## 3. 首飞配置与几何：现场资料待补，不能标通过

操作员本轮从 QGC 确认：

| 参数 | 修改前报告 | 修改后操作员确认 |
|---|---|---|
| COM_OBL_RC_ACT | Position Mode (0) | Land Mode (4) |
| NAV_RCL_ACT | Return Mode (2) | Land Mode (3) |
| COM_RC_OVERRIDE | 1 | 3，允许自动及 Offboard 摇杆接管 |
| RC_MAP_FLTMODE | Channel 5 | Channel 6；新截图核对通过 |
| RC_MAP_KILL_SW | Channel 5 | 保留 Channel 5，专用急停；新截图核对通过 |
| RC_MAP_OFFB_SW | Channel 6 | Unassigned (0)；新截图核对通过 |
| 模式档位 | 1/4 Position、6 Return，其余未分配 | 1–5 Position、6 Offboard；新截图核对通过 |

操作员随后确认：前三项已在 QGC 修改为候选值，且 **PX4 已重启**。
这是操作员回报，尚无本轮新参数导出做独立回读；重启后另做只读遥测复查。
操作员确认独立急停开关输出 Channel 5，模式/Offboard 实体开关为 Channel 6。
已由操作员完成上表映射调整并再次重启 PX4；更新后的 `/home/cfly/image.png`
逐项确认映射已分离。原图的“飞行模式 6”是模式槽位，不是 Channel 6。
随后操作员提供 `manual_control_switches` 三组记录，未解锁输入检查通过；
证据见 `evidence/rc_switch_input_20260923/operator_record.md`。
不能将输入检查等同于电机实际急停切断验证。

枚举已核对官方
[PX4 v1.16.2 commander_params.c](https://github.com/PX4/PX4-Autopilot/blob/v1.16.2/src/modules/commander/commander_params.c)。
`COM_RC_OVERRIDE=3` 是 bit 0+bit 1；摇杆超过 `COM_RC_STICK_OV` 会切位置模式，
位置无效时回高度模式。接管意味着终止比赛任务，不代表自动降落。
Land 动作仍依赖姿态/高度可用和实际固件处理，设置枚举不能代替试验。

操作员后续 QGC 控制台回读确认：COM_RCL_EXCEPT=0、COM_OF_LOSS_T=1 s、
COM_RC_LOSS_T=0.5 s、COM_RC_STICK_OV=30%，这四项保持。
MPC_LAND_SPEED=0.7 m/s、COM_LOW_BAT_ACT=0（Warning）。原始数值及审查见
`evidence/first_flight_params_20260923/operator_readback.md`。
已核对 v1.16.2 官方 MPC_LAND_SPEED 声明最小值为 0.6 m/s，不能直接推荐
0.2/0.3 m/s。操作员随后确认 MPC_LAND_SPEED 已改为 **0.6 m/s**。
操作员明确选择 **COM_LOW_BAT_ACT=0（Warning）保持不变**，比赛低电仅告警，
不要求自动降落；此项不再作为必改条件。新集成控制链只记录电池告警，不额外
添加电池告警触发的 LAND。PX4 若因其他条件实际进入 failsafe，仍按既定失效策略处理。
控制器 0.2 m/s 上升限制不约束 AUTO_LAND，近地测距不可用时也不能依赖
MPC_LAND_CRWL。配置更改不等于低高度着陆或电池标定验收。

现场几何与遥控验收待操作员完成：

1. 操作员本轮实测：相对飞控中心最高点 +11.5 cm、**空载**最低点 -15 cm，
   空载总高 26.5 cm。已写入 platform.yaml 的 above_center_m=0.115 和新增
   below_center_without_payload_m=0.15。带货物最低点未测，原字段保持 null。
   首次空载起降可采用空载尺寸；载货必须补测。已有约 0.45×0.45 m 外廓报告，
   姿态/桨保间隙和整体 geometry_verified 仍未通过。
2. 已有相机 XYZ 尺量保留；roll/pitch/yaw、IMU 标定和水平导航坐标对齐未核，
   不能直接将对应 verified 改为 true。
3. **已通过**未解锁开关输入检查：Position/急停关为 mode_slot=1、kill=3；
   仅急停开为 mode_slot=1、kill=1；急停关/Offboard 档为 mode_slot=6、kill=3。
   三次 offboard_switch 均为 0，模式与急停互不串扰。
4. 接收机失联行为、急停实际切断和飞行模式接管效果仍未实测。
   本阶段不从 Jetson 解锁，也不通过杀 XRCE Agent 伪造断链试验。

未解锁的开关输入检查已发给操作员：每个位置执行
`listener manual_control_switches -n 1`，核对 mode_slot / kill_switch / offboard_switch。
Position+急停关应为对应模式槽位 / 3 / 0；仅急停开为同槽位 / 1 / 0；
急停关+Offboard 档为 6 / 3 / 0。最后恢复 Position+急停关。
此时没有 Offboard 心跳，不能将模式被拒绝当作开关输入失败。

加入控制权丢失退出和独立 Offboard 通道冲突检查后，最新离线测试总数为 **71**；
DDS 五场景复测证据为 `flight_dds_20260923_211226`。
首次参数修改重启后的只读证据 `px4_telemetry_20260923_211219`：
RC/GCS 正常、未解锁、无飞控输入、气压开/测距高度关；无 EV 时预飞 false。
模式/急停映射调整并再次重启后的只读复查为
`px4_telemetry_20260923_211757`，采集正常；后续三组开关输入记录已通过。
最后提供的样本是 Offboard 档，仍需操作员保持/恢复 Position+急停关；
不根据缺失的恢复样本推断当前开关位置。

## 可重复执行入口

```bash
cd /home/cfly/ros2_ws
bash robocup_nav/run.sh flight-test
bash robocup_nav/run.sh flight-dds-check
bash robocup_nav/run.sh flight-review --params /absolute/current.params
bash robocup_nav/run.sh flight-review --params /absolute/current.params --payload none
bash robocup_nav/run.sh preflight-observe --duration 30
bash robocup_nav/run.sh flight-ev-shadow --duration 60
```

后两项需要既有遥测/视觉数据；自身不启动飞控 Agent 或相机。
`flight-review` 只读导出与平台配置，不写文件、不写参，也从不签发 flight approval。
