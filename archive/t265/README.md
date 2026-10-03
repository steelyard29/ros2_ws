# T265 架构参考（2025 年比赛用，去年方案）

## 数据链路

```
T265 硬件 VIO（专用 VPU）
  → /t265/odom 或 /t265/pose/sample
  → t265_odom_publisher.py（外参补偿：camera→base_link）
  → /visual_slam/tracking/odometry + TF odom→base_link
  → vslam_odom_bridge（与 D435i 共享同一 bridge）
  → /fmu/in/vehicle_visual_odometry → PX4 EKF2
```

## 核心差异 T265 vs D435i

| | T265 | D435i |
|------|------|------|
| VIO 方式 | 硬件 VPU 芯片 | 软件 cuVSLAM/stereo_odometry |
| 稳定性 | 振动不敏感 | 振动下漂移 3-15m |
| 外参 | 需标定 camera_pose_frame→base_link | 已标定 camera_link→base_link |
| RTAB-Map | 可选（激光辅助回环） | 必需（修正 VIO 漂移） |

## 启动方式

```bash
# 当前支持的统一启动（run_slam_px4.sh 已内置 T265 路径）
ODOM_SOURCE=t265 RUN_MODE=debug ./scripts/run_slam_px4.sh --bg

# 去年原始方式（start_robot.sh，tmux 分窗）
./scripts/start_robot.sh
```

## 飞行控制

```bash
# 去年的一键起飞（简单 NED setpoint，不检查 EKF 融合状态）
python3 ~/ros2_ws/scripts/px4_fly.py takeoff 1.0
python3 ~/ros2_ws/scripts/px4_fly.py land

# 今年的起飞控制（安全检查多，需 cs_ev_pos=true）
ros2 run uav_task takeoff_control.py
```

## 注意事项

- 外参是占位值（x=0.15, z=-0.04, rpy=0），需按实机重新标定
- 当前 librealsense 2.58 不含 TM2（T265）支持 → 需旧版驱动
- T265 USB ID: 8086:0b37
- `t265_odom_publisher.py` 有三种数据源优先级：ROS topic → pyrealsense2 直读 → 无数据警告
