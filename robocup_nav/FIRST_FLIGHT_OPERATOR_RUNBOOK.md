# 无货单次起降：软件交付与操作员收尾

本入口会真正发送OFFBOARD、ARM和LAND。本文不是当前飞行许可。当前仓库仍为 `live_flight_enabled=false`、四项verified=false；没有为本次开发创建放行记录，没有执行真实起降。仅适用于已审查的PX4 FMUv6C v1.16.2、现有AUX映射、无货竖直起降，不是完整比赛自主任务入口。

软件交付验证：215项单元测试通过；同一版本15项隔离DDS场景通过，证据 `evidence/live_flight_check_20260925_213544_d9af98/report.json`。真实发送路径的授权前置条件在无ROS导入的测试中检查；运行链、序列化、ACK和异常停发在隔离话题上验证，没有开展真实飞控命令发送或实机飞行验收。

## 软件入口

- `run.sh live-flight-check`：domain177合成飞控全链验证，永不创建真实/fmu/in发布者，不是PX4 SITL。
- `run.sh flight-release`：离线放行记录格式、哈希和证据检查，不接飞控。
- `run.sh live-flight`：唯一新增的真实起降入口；需完整放行记录、一次性会话、交互终端确认、连续Position基线及新Offboard拨档。原flight-runtime默认影子模式、EV-only、模式握手/故障台架权限保持。

执行高度为相对当前位置上升0.2–0.5m、悬停1–5秒，水平坐标/偏航保持；实际机体净空必须由现场审查决定。0.5m默认值不是现场推荐值或安全保证。软件不强制解锁、不发DISARM/电机测试/舵机/参数命令，不自动启动或停止Agent、相机或其他程序。

## 1. 操作员完成现场工作

保持卸桨和未解锁，先完成确实需要的PX4校准及最终安装/方向/几何审查。记录校准完成时间、结果和现场证据，导出新的完整PX4参数。保留现有模式/急停、视觉丢失/失联/接管处理和低电警告策略。校准移动机身后，不延用旧EV会话。

先保持live=false，按 `FIRST_FLIGHT_RELEASE_CHECKLIST.md` 完成校准后EV-only复核。新 `preflight_observer.py` 报告必须passed=true、final_ready=true、ever_armed=false、无source_fault，并有recorded_at_unix。该报告应在校准之后、正式启动前24小时内取得。对应方向核对仍需现场证据，不能只看静态预飞通过。

故障保护和运动/升高视野、场地、电池、人工处置位置由操作员现场验收。已有的电机/物理急停/防护罩、安装一致性、外围设施确认不要求重复，除非实际发生改变。不要为满足文件格式伪造测试或将历史台架超时改成通过。准确固件保护验收需要专业人员按场地条件安排；未解锁试验不能证明空中行为。

现场完成后，操作员审核证据，才可把 `config/platform.yaml` 对应的四个verified设为true。真实飞行开启必须单独明确决定，最后才将live_flight_enabled设为true。校准后EV-only工具要求live=false，故不要提前打开。参数或TF改变后需相应重新核验；相机必须运行实际审查过的配置，不能沿用修改TF之前启动的节点。

## 2. 制作当次放行记录（离线）

在Cursor终端查看未批准的JSON模板，另存为本次release文件，按实际情况填写：

```bash
bash /home/cfly/ros2_ws/robocup_nav/run.sh flight-release --template
```

字段说明：

- session_id：模板生成的UUID，每次尝试一个新ID；operator填写现场责任人。
- issued_at_unix / expires_at_unix：UTC Unix秒，必须当前有效，间隔不超过900秒。不知道当前值可运行 `date +%s`。过期或系统时间倒退后不能发ARM。
- payload仅支持none；height_m/hover_s按现场审查填写，不超上述软件限制。
- 四项operator确认必须据实填写：导出与当前板载一致、system1/component191命令来源独占、确认FMUv6C v1.16.2、知晓保留低电仅警告。磁盘导出不证明当前板载一致，软件不远程写参数或验证全部板载参数。
- evidence七项：calibration、post_calibration_ev、geometry、extrinsics、frame_alignment、fault_acceptance、site_and_battery。每项填写非空证据文件的**绝对路径**、SHA256、实际完成的Unix秒和reviewed=true。现场文字记录/照片/原始日志均可作为操作员审查证据；程序只验证文件与声明的一致性，不代替真实性或适航审查。post_calibration_ev必须是上述新版观察器JSON报告。site_and_battery必须为启动前15分钟内的当次确认。
- parameter_sha256、platform_sha256、software_sha256必须绑定当前最终版本。完成配置修改后取得：

```bash
bash /home/cfly/ros2_ws/robocup_nav/run.sh flight-release --fingerprints --params /absolute/current.params
sha256sum /absolute/evidence-file
```

再执行离线一致性检查：

```bash
bash /home/cfly/ros2_ws/robocup_nav/run.sh flight-release --release /absolute/release.json --params /absolute/current.params
```

`release_consistent=true`仍不是飞行许可，不会连接硬件。模板不自动填写完成项，不自动启用live标志。任何代码/配置/参数/证据变化都会使旧哈希失效；不可只重算哈希而不重新审查改动。

## 3. 当次真实入口（仅现场放行后由操作员执行）

按现场验收流程完成装桨与清场。飞控应未解锁，遥控在Position、急停关闭，QGC在线监看，操作者可立即接管。不要为了通过启动检查在本程序之外先行解锁。

Agent及相机/VIO/tracking须已按现有部署运行在domain0；不要误用domain174相机诊断会话。不得同时运行EV-only、旧EV桥或其他向/fmu/in写入的控制程序。QGC/RC保留；实际命令源独占需要操作员确认，因为DDS图检查不能枚举所有MAVLink发送方。入口不替你管理这些进程。

相机侧需可见唯一的 `/visual_slam/tracking/odometry` 与 `/robocup/alignment/tracking`；仅启动 `perception.launch.py` 不一定包含tracking转接，须保留现有已验证的相机+tracking部署。看不到这些话题时入口拒绝启动，不要绕过检查。

若相机尚未运行，现有容器已启动且没有其他相机占用者时，可在**独立终端**使用与既有EV-only台架相同的相机/relay入口（本轮未执行）：

```bash
docker exec -it -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=1 \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/ros2_ws/config/fastdds_bridge.xml \
  isaac_ros_dev bash -c 'source /workspaces/ros2_ws/isaac_vio_debug/scripts/container_env.sh; exec python3 /workspaces/ros2_ws/robocup_nav/scripts/alignment_camera.py --visual-only-diagnostic --duration 900'
```

它只管理自身相机/VIO/状态relay，不发飞控输入。该旧监督器有900秒寿命，**应在完成记录准备后新启动，并在启动后5分钟内进入本次起降入口**，给初始化、最多175秒运行与处置留足余量；若拖延，必须在未解锁且控制程序未运行时结束旧相机会话并重开，不能在空中重启相机。飞行期间保持该终端和Agent运行；关闭相机终端会造成视觉丢失。软件入口不会自动重启相机或Agent。

**下面是会导致真实解锁的命令，不属于本轮开发已执行内容：**

```bash
bash /home/cfly/ros2_ws/robocup_nav/run.sh live-flight \
  --release /absolute/release.json --params /absolute/current.params \
  --authorize-real-flight
```

1. 在30秒内手工输入终端显示的 `AUTHORIZE FLIGHT <session_id>` 完整文字；不能通过管道自动确认。
2. 会话ID立即被一次性消耗，失败也不复用。只读预检后才建立真实发布者。
3. 保持Position，程序先发布EV、等待连续5秒健康融合/预飞。没有READY提示不要拨档。已在Offboard或提前拨档会终止，不自动重试。
4. 看到READY后60秒内，操作员决定是否从Position新拨到Offboard。**这个动作会使程序自动完成模式请求、解锁、起飞、悬停与降落；并非此前的“只模式握手”。** 不再需要聊天回复，终端与现场操作直接同步。
5. 模式必须有匹配component191的接受ACK和实际Offboard状态，解锁必须有接受ACK及实际解锁状态。ACK不是动作完成证明。ACK丢失/拒绝、状态不匹配或超时会中止，不强行继续爬升。
6. 正常结束需观察到Auto Land、相关ACK，以及连续2秒新鲜落地且未解锁状态；不自动发DISARM。查看本次report.json，不能只看进程退出或飞机暂时不动。

## 中止与故障

- 急停/人工接管：停止EV、心跳、位置与命令，不自动重入、不转为额外LAND抢控制。独立急停的实际电机切断由飞控/遥控通道负责。
- 视觉失效：停止EV和控制流；新鲜且已解锁、仍有控制权的状态下请求LAND。
- 失联：停止控制流，LAND意图不代表送达；依靠已验收的PX4板载失联策略。不要通过杀Agent/乱拔共享USB临时模拟验收。
- Ctrl-C/SIGTERM：请求中止，条件允许时继续短时间LAND/落地观察，最多20秒；不是立即强制停机。接管/急停始终优先。不要以为按Ctrl-C即电机停止。
- Python发送异常、进程崩溃/强杀、断电：无法保证补发LAND；入口会尽可能关闭自身所有发布者，最终必须由板载保护与现场人员处理。不能把程序退出等同于安全落地。
- 低电仍仅警告，软件不自动改参或新增低电降落；电池耗尽无法保证维持飞行。

任何失败/超时/接管后不复用本会话。恢复Position、确认实际状态和故障原因，重新审查后才能新开会话。一次性记录位于 `evidence/live_release_consumed/`，不要删除来复用旧许可。测试结束后操作员应关闭live标志并保留日志。

## 交付边界

软件入口和隔离测试完成不等于飞机已获飞行放行；真实传感器质量、准确固件行为、标定、装桨与人员净空由现场验收覆盖。首次竖直起降通过也不代表完整比赛导航、取放货或带货几何已验收。
