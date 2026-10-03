# 2026 比赛任务核心：离线实现

2026-09-23。已逐页阅读 `/home/cfly/2026比赛规则.pdf` 19 页扫描件。
本模块不导入 ROS、GPIO 或飞控通信库。`live_flight_enabled` 仍为 false。

## 已实现范围

`scripts/competition_mission.py` 是确定性的任务决策层：

READY → TAKEOFF → SEARCH → ALIGN → RELEASE → SEARCH（最多三件）
→ RETURN → LAND → DONE。

首版实现规则允许的直接回起飞区路线，不实现两门路线，不宣称降落区得分。
输出只代表意图，不会解锁、发送设定点、驱动舵机或启动任何其他节点。
SEARCH 是给未来搜索适配器的请求，**不是已经实现覆盖搜索或绕障**。

- 起飞前要求新鲜预飞、定位、链路、控制权、落地且未解锁证据。
- 从一次 start 授权计时；30 s 未离地终止；高度 >1 m 且稳定连续 >10 s
  才进入搜索。稳定性由适配器按速度/姿态判断，本模块不伪造测量门限。
- 600 s 截止；保守地从 start 而非实际离地计时。540 s 开始返航，
  返航超过 40 s、降落超过 30 s 进入终止降落意图。
- 不配置靶标坐标。接收实时检测标识与类别；目标证据有效期 0.3 s，
  置信度至少 0.8（工程初值，尚无真实模型标定）。对准持续 1 s 后发单次释放意图。
- 三个槽位 0/1/2；每次必须收到当前 request_id 的物理释放反馈才记入
  confirmed_releases。3 s 未确认即标记库存不确定并返航，不自动重发。
  释放后失联/定位失败/接管也会标记不确定槽位。
- 已尝试的目标不重复投放，是首版保守策略，不是规则禁止同靶投放的结论。
  当前按首次有效检测顺序处理，不实现得分权重优化。
- 视觉失败、失联、失去控制权、非法高度/超过 4 m、监督停顿 >0.5 s、
  遥测过期 >0.5 s 均锁存 ABORT；恢复不会续赛。新鲜人工接管证据进入
  HANDOVER，之后不再输出任何自主控制意图。
- 树木等接触取消避障资格；墙/网/地面等接触终止任务。本版不通过门，
  对 gate 接触也采用终止策略。避障资格仅表示未因碰撞取消，不能当作得分。
- 只有新鲜 land detector 落地且未解锁才认定完成；高度为零不等于落地。

## 运行与验证

```bash
cd /home/cfly/ros2_ws
bash robocup_nav/run.sh mission-test
bash robocup_nav/run.sh mission-replay --demo nominal
bash robocup_nav/run.sh mission-replay --demo vision-loss
bash robocup_nav/run.sh mission-replay --demo link-loss
bash robocup_nav/run.sh mission-replay --demo release-timeout
bash robocup_nav/run.sh mission-replay --demo manual-takeover
bash robocup_nav/run.sh mission-replay --demo liftoff-timeout
bash robocup_nav/run.sh mission-replay --input snapshots.jsonl
```

演示的运动、识别、释放与落地反馈全部为合成输入。演示约 14 s 完成不是
真实飞行性能。尤其 link-loss 演示的后续落地反馈是假适配器提供的，不能说明
断链后 LAND 能发到飞控。真实断链依赖飞控内部失效保护，仍未验收。
报告的 complete 表示回放进入终态（含终止/接管），不是比赛成功或飞行通过。

JSONL 每行格式：

```json
{"now":0.0,"start":true,"evidence":{"at":0.0,"height_m":0.0,"landed":true,"armed":false,"preflight_ok":true,"localization_ok":true,"link_ok":true,"control_owned":true}}
```

完整字段见 Evidence dataclass。时钟为同一单调时钟，正常 10 Hz 调用；
evidence.at 是真实采样/聚合时间，不能每次拿缓存重打时间戳。
适配器需要先逐项校验原始话题新鲜度，再产生原子快照。搜索完成、对准、
返航完成及释放确认必须带当前 request_id，防止旧应答推进状态。
这些 ID 仅在单次进程会话有效；当前无跨重启库存持久化，禁止中途重启续投。

## 与现有代码的关系

`src/uav_task/scripts/servo_control_node.py` 已有单路 GPIO 舵机控制；
`circle_landing_target_node.py` 已有圆形检测。它们不是三槽释放确认或
按四电机中心圆内含关系验收的黑环降落实现，本次未运行或修改。

旧 `competition_obstacle_course.yaml` 的固定航点延伸至 x=12 m，且安全注释要求
EV 速度和 TFmini 高度融合，与现用 EV_CTRL=11、RNG_CTRL=0 不一致。
不能直接接入此核心作为比赛导航/飞行适配器。原文件保留未修改。

## 下一阶段依赖

1. 飞行监督：将 EV 锁存状态、新鲜 PX4 状态和控制权接入影子适配器，
   在隔离环境验证；独立飞行 EV 发布器及断链回退仍待实现/验收。
2. 搜索：接入 nvblox/Nav2 的在线覆盖搜索，未知格不可视为可飞空间；
   验证机体与货物三维包络、水平绕行、路径和盲区。
3. 感知：真实靶标检测与跟踪 ID、目标坐标、对准证据及落点估计。
   当前接受 tank 类别，但规则未列其权重，不推定分数。
4. 投放：确定三槽机构、接口和物理释放反馈；跨重启库存与动作去重。
5. 扩展两门路线及黑环精准降落；当前不存在过门或落点得分证据。

19 项任务/回放测试通过；含既有 EV、坐标和几何测试的全量
`python3 -m unittest discover -s robocup_nav/tests -p 'test_*.py'` 为 53 项。
测试是逻辑验证，不是真 PX4 SITL、实机 DDS 时序或飞行验收。
