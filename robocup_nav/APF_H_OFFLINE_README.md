# A* + APF + H 降落：离线验证入口

截至2026-09-30，本方案完成部分L1/L3软件验证，**没有完成实机自主避障或H降落**。
不改变已成功的起降入口、不启动相机/雷达/Agent、不操作PX4或舵机。
已有容器 `isaac_ros_dev` 必须运行；本入口不会创建、重建或启动容器。

## 实际运行内容

真实NavFn A*、当前 `uav_task` 的C++ APF及独立包装，与合成地图/雷达、
简化运动模型连接。`path-apf`采用A*方向前进、APF侧向避让，后接地图制动检查。
`legacy-apf`保留原向量作为对照；不是将新包装冒充去年原机实飞软件。
默认实飞控制器未更换；旧平台T265且无TFmini，不等同于当前D435i平台。

H完整链使用真实K/D和合成H图像、理想光学外参、完美定位、假设1mm/s漂移界。
不按H字母尺寸测距；60×40cm只用于生成测试图像。合成巡航升高1.1m不是比赛高度批准。
所有结果均不是PX4固件SITL，也不包含真实电机、气流、地效或传感器噪声验证。

## 可复现命令

以下测试**依次运行**，不能同时占用ROS隔离域176；不要与任何实机桥接该域。
正常运行每项约1～2分钟，结果在 `robocup_nav/evidence/` 的独立时间戳目录。

```bash
# 1. 两箱通道，包含H图像识别、对准及模拟触地/未解锁确认
bash /home/cfly/ros2_ws/robocup_nav/run.sh offline-apf-h --controller path-apf --scene two-boxes --scan-beams 720 --initial-y-offset -0.05

# 2. 绕墙完整链
bash /home/cfly/ros2_ws/robocup_nav/run.sh offline-apf-h --controller path-apf --scene wall --scan-beams 720

# 3. 单独导航到点，要求误差<0.10m、速度<0.03m/s持续2秒
bash /home/cfly/ros2_ws/robocup_nav/run.sh offline-navigation --controller path-apf --scene two-boxes --scan-beams 720 --initial-y-offset 0.05

# 4. 实际C++适配器的合成坏数据/断流测试，正常对照必须同时通过
bash /home/cfly/ros2_ws/robocup_nav/run.sh offline-apf-faults

# 5. 从地面起步：原起降监督逻辑→显式交接→避障→H→触地
# 经唯一ROS候选输出驱动简化机体；仍不是真实PX4 SITL/实机入口。
bash /home/cfly/ros2_ws/robocup_nav/run.sh offline-sortie-h --controller path-apf --scene two-boxes --scan-beams 720

# 6. 新接口组合：真实PX4消息格式经DDS回调进入原READY/AUX/EV策略，
# 再经唯一候选出口完成起飞/任务/触地。输入、场景与ACK仍全部合成。
bash /home/cfly/ros2_ws/robocup_nav/run.sh offline-ready-sortie-h --controller path-apf --scene two-boxes --scan-beams 720
```

切回 `--controller legacy-apf` 只改变隔离对照，不改原包；DWB对照使用
`offline-navigation --controller dwb --generator StandardTrajectoryGenerator`。
不要因为一个场景通过就认为局部极小、动态障碍、窄门、过期地图或实机停止均已解决。

## 当前证据与边界

- 两箱完整链：`nvblox_closed_loop_20260930_105735/report.json`，通过。
- 绕墙完整链：`nvblox_closed_loop_20260930_111423/report.json`，通过。
- 通道到点驻留：`nvblox_closed_loop_20260930_111622/report.json`，通过。
- 雷达/定位输入失效：`path_apf_fault_20260930_111923/report.json`，28段通过。
- 地面起步完整DDS候选链：`nvblox_closed_loop_20260930_114013/report.json`，通过；
  含独立输出边界检查和12项源码哈希，运行期间源码未变。
- READY/AUX/EV策略＋DDS输入的新组合：两箱`nvblox_closed_loop_20260930_123542/report.json`通过；
  冻结版本绕墙`nvblox_closed_loop_20260930_124457/report.json`通过，新增逐周期H时序诊断。
  旧慢轮`...123750`仍保留false；H等待波动未被一次通过否定。
- 冻结后相关单元回归80项通过；所有结果仍不是真实PX4固件SITL或实机任务许可。

制动使用保守包络及假设延迟/制动力；选择零候选不等于证明飞机已经停止。
异常测试证明适配器不继续生成前进候选，未验证PX4响应。
禁止缩小包络、清空未知格、放宽时效或删除原起降横移保护来获得通过结果。

## 接实机前还缺什么

1. 新增独立任务执行器已经完成原起降监督阶段→导航/H阶段的唯一候选ROS输出，
   新增`ReadySortieShadow`组合原READY/AUX/EV策略，`SortieInputShadow`仅接收域176的固定合成输入，
   不等于生产`flight_runtime.create_node`接入或独立飞行许可完成。其输入状态/ACK/触地仍是合成，
   不得把固定的 `/robocup/sortie_shadow/output` 路由改为 `/fmu/in` 来启动实机。
   实际PX4停止/接管/自动上锁响应和固件SITL尚待验证。
2. 实际地图起点与升降扫掠覆盖、相机米制误差及目标出视野后的独立漂移界。
3. 拆桨状态下接口验收及另行批准的操作员实机测试。

`landing_candidate.yaml` 中真实外参/米制/边界/下降开关继续关闭。
棋盘格最终实测边长12.5mm（用户纠正此前17mm读数）、板厚约10mm，镜头最低点到PX4为55mm。
近期起终点图按12.5mm得到离地约83.5mm，与名义85mm接近；局部位移尺度矛盾已关闭。
早前“仅差5mm”的单图结论因采用错误格长而撤回，保留历史报告。
当前局部一致性不自动成为已验证的全高度误差上界；旧内参及其19mm标定资料未改写。
