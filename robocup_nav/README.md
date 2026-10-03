# RoboCup 导航部署

> 2026-09-16：最新 cuVSLAM + nvblox + Nav2 A*/DWB 部署与实测请看
> [NVBLOX_README.md](NVBLOX_README.md) 和 [NVBLOX_PROGRESS.md](NVBLOX_PROGRESS.md)。
> 下文是保留的旧 RTAB-Map 路线历史说明，包括早期传感器故障状态；新栈仍为影子输出，未放行飞行。

本目录是独立部署，不替换旧的 `run_slam_px4.sh`，不重建容器。当前交付为
**定位/建图配置 + Nav2 水平导航 + PX4 消息预览适配层**。
不含自动解锁、飞控位姿注入、实机 Offboard 使能、舵机输出或比赛完整任务状态机。
不能将预览话题 remap/relay 到飞控。实际飞行仍需补齐以下验收条件。

## 架构与坐标

1. 现有 `isaac_ros_dev` 内唯一 D435i 驱动采集双 IR 640×480@30、accel 250 Hz、gyro 200 Hz。
   `rgbd:=true` 另开 RGB 640×480@15、深度 640×480@30，并对齐到 RGB。
   不改变 EEPROM；不启动第二个驱动；关闭 IR 投射器，保留对其深度质量的实测要求。
2. cuVSLAM 默认视觉-only（`imu_fusion:=false`），发布连续 `odom → base_link` 和
   `/visual_slam/tracking/odometry`。D435i IMU 仍按 IR-then-IMU 采集，但不送入
   cuVSLAM；后续候选是 PX4 自身 IMU 做 EKF。关闭其 `map → odom` 发布；噪声参数沿用
   已有基线，不宣称已做 Allan 标定。卸桨台架可将同一 FRD 契约注入
   `/fmu/in/vehicle_visual_odometry`（`probe --ev-inject` 或 `run.sh ev-bridge --prop-off`）。
   禁止使用旧 `vslam_odom_bridge`（测距顶替 Z、timesync 偏移、`align_yaw_to_px4`）。
   `live_flight_enabled` 仍为 false；这不是飞行批准。
3. RTAB-Map 只使用该外部里程计和 RGB-D，发布唯一 `map → odom` 与 `/map`。
   使用三维姿态图/深度建图，不使用 `stereo_odometry`，不强制三自由度。
   每次建图必须指定新的数据库路径，拒绝覆盖已有地图。
4. 几何桥发布 yaw-only `odom → base_footprint` 和 `/robocup/odom`。`base_footprint`
   是高度为零的水平投影，与 `base_link` 是兄弟节点，不重复发布其 TF。
   深度云保留相机光学坐标和时间戳；依据整机/货物高度范围与倾斜余量筛选障碍。
5. Nav2 全局 NavFn `use_astar=true`，局部 DWB `LimitedAccelGenerator`，代价地图分辨率
   5 cm，滚动局部窗口 5×5 m。全局和局部均包含静态、实时障碍和膨胀层。
   未知格不视为空闲。当前只准许前向速度，横移/后退需侧后方感知覆盖验收后再开放。
6. 预览桥把机体水平速度旋转到 odom 世界坐标，再演示 ENU→NED 变换。
   **VIO 世界系与 PX4 local NED 的航向/原点对齐尚未实现或验证**；仅转换轴顺序不足以
   建立两者对应关系。预览消息在 `/robocup/preview/trajectory_setpoint`，没有 `/fmu/in/*`
   发布者。输入里程计或指令超过 0.3 秒停止预览，不伪称这等同真实飞控失效保护。

## 已记录的硬件信息

`config/platform.yaml`：用户报告保护罩外尺寸 0.45×0.45 m，机体总高约 0.25–0.30 m。
2026-09-18 尺量：镜头玻璃在 PX4 前 0.20 m、相机中心低于 PX4 0.05 m、红外投影器在
PX4 右侧约 0.015 m，且 PX4 与 D435i 模组中心重合。换算到 `camera_link`（左 IR 光心）
为前 0.196 m、左 0.025 m、下 0.05 m；姿态角仍沿用旧值。雷达前后/左右 0、上 0.10 m。
机体上端/带货下端相对 base_link 的距离保留 null。PX4 安装点也不能未经确认等同于重心。
`geometry_verified=false`，`sensor_extrinsics_verified=false`，实机导航入口拒绝启动。

默认导航 YAML 的 0.70 m 方形只是离线测试夹具。只有设置实测平台几何并完成确认后，
导航入口才会生成对应的 footprint。`demo_geometry:=true` 只用于隔离合成测试。
传感器内参标定不等于相机/雷达到机体外参标定。

## 使用（宿主机，互不重复启动）

离线实插件测试，不读取相机、不连接 PX4；固定 ROS_DOMAIN_ID=173、localhost only：

```bash
bash /home/cfly/ros2_ws/robocup_nav/run.sh test
```

检查现有容器状态，然后在没有其他相机驱动使用设备时启动已存在的容器：

```bash
docker ps -a --filter name=isaac_ros_dev
docker start isaac_ros_dev
```

独立有界联合采集验证（45 秒采样，另加初始化/关闭时间），结果写入新 evidence 子目录，
固定 ROS_DOMAIN_ID=174、localhost only。无需启动几何桥或飞控。

```bash
bash /home/cfly/ros2_ws/robocup_nav/run.sh smoke --stereo-only   # step 1: dual-IR+IMU+VIO
bash /home/cfly/ros2_ws/robocup_nav/run.sh smoke --rgbd-no-map   # step 2: add RGB-D, no RTAB-Map
bash /home/cfly/ros2_ws/robocup_nav/run.sh smoke                 # step 3: RGB-D+IMU+VIO+mapping
```

交互式连续感知，仅在相机联合流故障排除后使用；终端 Ctrl-C 结束此次启动：

```bash
bash /home/cfly/ros2_ws/robocup_nav/run.sh perception rgbd:=true
```

另一个宿主机终端启动建图；路径必须尚不存在，所有终端 ROS_DOMAIN_ID 必须一致：

```bash
bash /home/cfly/ros2_ws/robocup_nav/run.sh mapping database:=/home/cfly/ros2_ws/robocup_nav/maps/session01/map.db
```

实测尺寸/外参通过后，几何桥和导航可做真实数据上的影子测试，仍只有预览输出：

```bash
bash /home/cfly/ros2_ws/robocup_nav/run.sh geometry
bash /home/cfly/ros2_ws/robocup_nav/run.sh navigation
bash /home/cfly/ros2_ws/robocup_nav/run.sh preview
```

三个命令分别在三个终端运行。导航入口只支持 NavigateToPose；不提供 NavigateThroughPoses
任务编排。行为树每秒重规划，无自动旋转/倒退/清图恢复，失败返回上层。当前没有
自动搜索任务生成，不能把未知区都改成 free 来假装具备探索功能。

## 已验证与限制

- 四项纯几何测试覆盖 ENU/NED、旋转、非法输入和缺测深度不生成自由空间。
- 实际 Nav2 插件在合成场景中完成生命周期激活、A* 绕墙、DWB 非零速度输出；全封闭墙
  导致规划失败；停止里程计后预览停止；测试 ROS 域不存在 `/fmu/in/*` 话题。
  这是软件集成测试，不是 Gazebo/PX4 SITL 动力学仿真，也不是飞行测试。
- IMU SDK 已读到与保存结果一致的加速度内参，运动校正开启。独立 30 秒出流证据在
  `evidence/imu_20260914_130628/`；存在最长约 50 ms 间隔和明显波动，未通过静置质量验收。
- 联合流测试 `evidence/perception_20260914_132042/` 失败：驱动报告 Motion Module failure，
  RGB-D/双目有数据但 IMU/VIO/地图为空。该目录中的 map.db 只是数据库文件，不能当可用地图。
  测试进程已停止，等待物理重插后按最小双目+IMU组合复测；不要启动飞行。
- 陀螺仪 SDK 偏置为 JSON 值乘 π/180；本地 SDK 解析器存在角度到弧度转换。
  标定脚本直接使用 motion samples 的均值，单位一致性还需核对。未擅自重写 EEPROM。
- 深度下采样、前方视野与测距无效区域仍可能漏掉细枝、保护网、侧后方障碍。
  RPLidar 的实际串口、出流和旧 TF 尚未验证，暂不自动启动或纳入控制有效性判定。
- RTAB-Map 全局栅格是地面分割后的保守二维投影；它不是经过验证的全高度碰撞检测器。
  地面分割阈值基于初始化坐标，起飞地面坐标与地图注册需现场验证。局部高度筛选也不能
  消除深度遮挡。需要实测 0.8 m 门及全机体/货物通过余量。
- 参数以低速原型为起点：0.25 m/s、0.30 m/s²、0.35 rad/s，不构成已验证制动能力。
  后续需要定位失效、地图新鲜度、障碍数据有效性、离地高度、PX4 状态、急停与接管的
  独立飞行监督器，以及唯一控制权仲裁。
- 完整比赛还缺靶标识别模型/数据、覆盖搜索、三件货物机械接口、释放确认、精确降落
  及 SITL/台架/实机分级验收。Nav2 本身不会自动实现这些任务。

## 回退与清理记录

本目录独立新增。停止对应进程后，旧启动入口和现有系统环境可继续使用；不要同时启动
旧栈与本栈抢占相机/TF。未升级系统、安装新包、刷写固件或修改飞控参数。
用户确认的 A+B+C 已删除：指定缓存、9.93 GB 长录包数据库、OpenVINS 备份；
实验统计/metadata 保留，但该长录包不可回放，OpenVINS 本地回退副本已不存在。
其他录包、标定原始数据、旧 Isaac 工作区、镜像、容器和卷保留。
详见 `PROGRESS.md`，测试证据在 `evidence/`。
