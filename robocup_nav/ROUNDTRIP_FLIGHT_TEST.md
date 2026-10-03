# 1 m / 2.5 m 往返测试候选代码

本入口目前是任务控制代码与离线回放，**不是可直接装桨运行的真实飞行入口**。
此描述/回放入口不创建真实PX4发布器，没有解锁权限，不要把隔离话题改名到 `/fmu/in`。
2026-09-30新增专用真实接口与独立任务许可入口，详见[TASK_HANDOFF.md](TASK_HANDOFF.md)；
已经通过隔离DDS交接，未完成真实链路/实飞验收，本地实飞许可仍关闭。

## 航线与高度定义

- 用户已明确高度为**PX4中心距地面1.0 m**，不是海拔，也不是TFmini近地读数。
  PX4中心未载货正常落地离地0.14 m，因此相对爬升量为**0.86 m**。
  代码同时检查巡航高度相对起点0.86 m、相对同一地面1.0 m；不会把里程计原点当成地面。
- 前方为捕获的起飞航向，目标距离2.5 m；全程不要求转机头。
- 终点与起点均要求位置进入8 cm、水平速度低于5 cm/s、连续2秒稳定。
  这些是候选测试判据，不是已测得的实机精度。
- OUTBOUND到RETURN时更新目标ID；新目标尚未产生匹配的规划速度时保持零水平候选。
- 回到起点后才进入已有H对准/降落流程；缺少H、地图/制动或定位证据时不盲降。

## 代码与无硬件验证

`scripts/roundtrip_flight_test.py`中的`RoundTripMission`继承现有任务，
通过`planner_goal()`向导航适配器提供当前目标。适配器必须把匹配的目标ID写入
`Scene.navigation_goal_id`，并仍提供真实A*＋APF规划速度与地图制动检查。
真实适配代码现为`TaskSceneInput`/`TaskFlightCore`。现有`SortieShadow`和`LandingSequenceShadow`
仍用于原隔离下降试验；当前首次真实任务采用H对准后交回原PX4 LAND，不将两种下降方式混用。

查看配置（不连接硬件）：

```bash
python3 /home/cfly/ros2_ws/robocup_nav/scripts/roundtrip_flight_test.py --describe
```

完整理想回放（只使用PX4消息类型，不创建ROS发布器，不接飞控）：

```bash
source /opt/ros/humble/setup.bash
source /home/cfly/ros2_ws/install/setup.bash
python3 /home/cfly/ros2_ws/robocup_nav/tests/roundtrip_sortie_replay.py
```

回放中的规划速度是理想目标跟随夹具，不是实际NavFn/APF节点，地图与H也是合成数据。
回放成功只能证明两航段、起降交接和状态逻辑，不能证明真实避障、真实H识别或动力学稳定。

## 真机前尚缺

恢复定位/PX4链，验收已实现的真实任务输入/唯一输出适配和目标ID关联、地图/雷达有效性与失效接管，
以及针对本条航线的现场操作确认。旧普通起降入口的0.3 m横向保护不修改。
不要运行去年含舵机/自动解锁的整套投递入口来代替此测试。
