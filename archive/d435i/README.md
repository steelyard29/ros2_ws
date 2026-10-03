# D435i 架构备份

## 数据链路

```
D435i → Isaac cuVSLAM (GPU) 或 RTAB-Map stereo_odometry (CPU)
      → /visual_slam/tracking/odometry
      → vslam_odom_bridge → /fmu/in/vehicle_visual_odometry → PX4 EKF2
```

## 关键改动（2026-08-05 调试记录）

1. **相机外参标定**：pitch=-0.0523, roll=-0.0131（3000 采样）
2. **use_isaac_vslam 开关**：false=RTAB-Map stereo_odometry (CPU), true=cuVSLAM (GPU)
3. **激光 ICP**：`laser_icp_odom.py` 改订阅 `/scan_planar`（IMU 拉平），绕过 relay
4. **RUN_MODE**：debug（不用 RTAB-Map）/ mapping（边飞边建图）/ production（定位）
5. **relay degraded 模式**：无 geometric match 时仍发布（时间戳用当前时间）
6. **takeoff_control.py**：min_vslam_odom_hz 15→8

## 最佳配置（2026-08-05 已验证）

```bash
USE_ISAAC_VSLAM=0 USE_LIDAR_SLAM=1 RUN_MODE=debug ./scripts/run_slam_px4.sh --bg
# EKF2_EV_CTRL=13, laser_icp_odom → /laser_odom → bridge → PX4
```

## 漂移总结

| 方案 | XY 漂移 | 原因 |
|------|---------|------|
| cuVSLAM | 15m | 振动导致特征跟踪失败 |
| stereo_odometry | 3.7m | 同上，CPU 算法略好 |
| 激光 ICP | 0.5m | 2D 扫描面高度变化导致爬升时漂移 |

## 不保留的文件

以下文件已废弃（引用链断裂或配置过时）：
- `launch/vslam_realsense_fixed.launch.py`
- `launch/rtabmap_realsense.launch.py`
- `scripts/launch_vslam.sh`
- `scripts/RTAB_MAP.sh`（引用不存在的 imu_rtabmap.launch.py）
- `scripts/launchmethod.sh`
