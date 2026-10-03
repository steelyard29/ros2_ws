# cuVSLAM + nvblox + Nav2（2026-09-16）

已安装并部署独立的**固定高度、影子输出**原型；这不是可直接起飞的无人机控制系统。
旧 RTAB-Map 原型和飞控入口未替换。最新测试见 [NVBLOX_PROGRESS.md](NVBLOX_PROGRESS.md)。

## 分工与可行性

```text
D435i 双目视觉-only → cuVSLAM → odom → base_link
（D435i IMU 采集但不融合；PX4 IMU 留给后续 EKF，尚未接入外部视觉）
D435i 原始深度 + 相机内参 + TF → nvblox TSDF / ESDF
    → static_map_slice → Nav2 全局 / 局部 costmap + inflation
    → NavFn A* → DWB LimitedAccelGenerator → 新鲜度/姿态/高度检查 → 影子速度
```

cuVSLAM 的稀疏特征地图不是障碍代价地图。nvblox 生成三维重建及指定高度带的二维距离切片，
官方 `nvblox::nav2::NvbloxCostmapLayer` 转为 Nav2 障碍层，再由 inflation 层生成缓冲代价。
DWB 是 ROS2 的局部轨迹采样/评分框架；本配置使用动态窗口式 `LimitedAccelGenerator`，
不是直接移植 ROS1 的 `dwa_local_planner`。Nav2 输出平面速度，不负责多旋翼姿态/推力控制。

本机 Ubuntu22.04、L4T36.4.4、Orin Nano、约 8 GB 共享内存，采用现有 Isaac ROS3.2 容器。
保留 cuVSLAM3.2.6 / Nav2 1.1.18，新增 nvblox3.2.5 官方 ROS 接口，而不是编译最新 core/master。
5 cm 体素、深度积分10 Hz、ESDF5 Hz；不开 RGB、彩色融合、网格可视化，降低负载。
长时间运动建图的内存增长、散热、功耗和加入目标识别后的性能尚需测试。

## RoboCup 任务适配

已核对 `/home/cfly/yun_ws/RoboCup规则.pdf`：场地10×10×4 m，走廊宽1.5 m，门宽约0.8 m；
进入障碍区须水平进入且高度不超过1.5 m。2026 年不提供靶标大致坐标，须主动搜索；
起飞得分要求离地超过1 m并稳定超过10 s。不能靠飞到障碍上方代替水平避障。

因此“固定安全高度 + 平面在线绕障”适合部分水平航段；起降、跨高度航段、上下碰撞检查、
目标搜索识别、三件货物投送、精准降落和任务状态机需另行实现。Nav2 本身不提供完整比赛行为。
未知空间保持未知，`allow_unknown=false`；未探索区域的远距离目标可能规划失败，需独立探索逻辑。
45×45 cm 外廓在80 cm门中仅有理论单侧17.5 cm净余量，尚未扣姿态、定位、深度误差和货物摆动。

## 坐标、重定位和高度限制

- nvblox、两个 Nav2 costmap 和导航目标均使用连续 `odom`；cuVSLAM 可单独发布 `map→odom`。
- cuVSLAM 有保存地图及 `LocalizeInMap` 服务能力，但**本次没有验证跨会话保存/重定位**。
  发布 map TF 不代表重定位验收通过。nvblox 历史体素不会自动随着 cuVSLAM 回环整体重积分。
  不要直接将两者都切换为 map 后宣称地图一致，也不要独立加载不匹配原点的稠密/稀疏地图。
- 每次启动创建新的会话地图。odom 重置/跳变后应停止该栈、重新建立地图；不自动清图继续跑。
- `center_z` 是 odom 中**固定**的切片中心，不是飞控起飞高度，也不是自动跟随无人机的高度。
  高度带暂定 `[center_z-0.4, center_z+0.4]`，仅为影子试验的保守参数。
  高度偏离中心8 cm或倾角超过10度时抑制影子速度。实际竖向外廓/载荷仍未确认，不能据此放行飞行。
- 水平 footprint 使用用户提供的0.45×0.45 m，padding3 cm。base_link/PX4参考点及上下包络尚未验收。
- 局部5×5 m滚动窗口10 Hz更新，全局30×30 m固定窗口2 Hz更新；输入 ESDF 目标5 Hz。
  当前是 static_tsdf，传感器会持续更新，但移动障碍消失后的清除时延及动态物体模式尚未验证。
  前向相机存在侧后方、细网、无纹理/反光等盲区；现有 RPLidar 未在新栈启动或融合。

## 启动、停止、测试

保持拆桨、无飞控输入。不要同时运行旧相机/VIO/飞控导航入口。
所有新组件在现有 `isaac_ros_dev` 容器运行，原生主机无需安装 nvblox。

```bash
# 仅启动现有容器，不使用会重建容器的 reset 命令
docker start isaac_ros_dev

# 交互式终端，一次启动相机、cuVSLAM、nvblox、Nav2、影子保护节点
bash /home/cfly/ros2_ws/robocup_nav/run.sh nvblox
# Ctrl-C 停止本次启动的节点；没有后台飞行进程。

# 只看建图，不启动 Nav2
bash /home/cfly/ros2_ws/robocup_nav/run.sh nvblox navigation:=false

# 离线：实际 GPU nvblox + 合成深度
bash /home/cfly/ros2_ws/robocup_nav/run.sh nvblox-smoke
# 离线：实际 Nav2/nvblox 插件 + 合成 ESDF，含 A*/DWB 和断流测试
bash /home/cfly/ros2_ws/robocup_nav/run.sh nvblox-test
# 实机：60秒自动结束，含 Nav2 代价地图，无目标/移动指令
bash /home/cfly/ros2_ws/robocup_nav/run.sh nvblox-smoke --live --navigation --duration 60
```

交互栈/离线导航使用 ROS_DOMAIN_ID=176、ROS_LOCALHOST_ONLY=1；有界建图/整栈测试内部使用175。
RViz/检查命令须在同一容器、同一域，source `isaac_vio_debug/scripts/container_env.sh`。
检查 `/nvblox_node/static_map_slice`、`/global_costmap/costmap`、`/local_costmap/costmap`、
`/visual_slam/tracking/odometry`、`/robocup/nvblox/readiness`。

唯一受保护输出 `/robocup/nav/cmd_vel_raw` **仍只是影子话题**，禁止直接接飞控。
上游 `/robocup/nvblox/cmd_vel_unchecked` 未受保护，更不能接飞控。
本栈不启动 PX4 bridge、Offboard 心跳、解锁或舵机节点，不发布 `/fmu/in/*`。
保护节点检查深度/ESDF/位姿/跟踪状态/命令新鲜度、平面速度界限，及短间隔大幅位姿跳变锁存。
它只是影子验证保护，不是已认证的飞行失效保护；飞行中发零速度不等同于可靠刹停或降落。

## 安装记录与回退

容器新增以下六个包，各为 `3.2.5-0jammy`：

- ros-humble-nvblox-ros
- ros-humble-nvblox-nav2
- ros-humble-nvblox-msgs
- ros-humble-nvblox-ros-common
- ros-humble-nvblox-ros-python-utils
- ros-humble-nvblox-rviz-plugin

共下载约24.4 MB、安装增加约176 MB，零升级/卸载。源码核查副本在 `vendor/isaac_ros_nvblox`，
release-3.2 commit `7908a183acf84f4f1ab3fda7b6d6caf3eefc1f78`；`vendor/COLCON_IGNORE` 防止被误编译。
包保存在现有容器可写层，删除/重建容器会丢失，尚未提交新的容器镜像。
如需重装，先在容器内 `apt-get -s install --no-upgrade` 模拟上述精确版本，再确认依赖无升级/移除。

回退只需停止新栈并使用旧入口；不要求卸载新包。若以后要卸载，仅模拟并逐项确认这六个新包，
不要运行全局 autoremove，不删除原容器、地图、录包或标定结果。旧 RTAB-Map 崩溃问题仍单独保留。

## 下一阶段验收（本次未执行飞行）

1. 拆桨、已知距离/转角的移动测试：核验坐标、尺度、延迟、闭环漂移与重定位后的地图一致性。
2. 确认机体参考点、最高点/最低货物点及倾斜包络；实测门框、地面和头顶障碍。
3. 按比赛时长持续建图+Nav2负载测试，补地图未知区/旧观测、动态障碍和传感器盲区处理。
4. 独立 PX4 控制适配和安全监督器、ENU/NED及速度参考系验证、接管/急停/失联策略、SITL。
5. 用户另行明确授权后，分阶段验证飞控位姿输入、台架及实机；不自动启用飞行。

参考：[nvblox core](https://github.com/nvidia-isaac/nvblox)、
[Isaac ROS nvblox release-3.2](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/tree/release-3.2)。
