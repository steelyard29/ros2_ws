# D435i + Isaac ROS VIO（独立拆桨验证栈）

此目录只启动 D435i 双 IR/IMU 与 Isaac ROS VIO，不启动 PX4 bridge，不发布 `/fmu/in/*`，也不包含解锁、执行器或飞行命令。运行前由现场人员确认桨已拆除、相机固定且 USB/电源稳定。

## 常用命令（Jetson 宿主机）

启动已有容器和独立相机/VIO：

```bash
/home/cfly/ros2_ws/isaac_vio_debug/vio_stack.sh start
```

查看本脚本管理的进程：

```bash
/home/cfly/ros2_ws/isaac_vio_debug/vio_stack.sh status
```

在线统计双 IR 配对、IMU/VIO频率、时间戳和位姿变化（例：60 秒）：

```bash
/home/cfly/ros2_ws/isaac_vio_debug/vio_stack.sh validate 60
```

录制并自动执行 `ros2 bag info`（例：15 秒；传 `0` 则直到 Ctrl+C）：

```bash
/home/cfly/ros2_ws/isaac_vio_debug/vio_stack.sh record 15
```

停止本脚本启动的 VIO 与相机：

```bash
/home/cfly/ros2_ws/isaac_vio_debug/vio_stack.sh stop
```

脚本没有 reset/rebuild 路径。若容器已停止，只有 `start` 会执行 `docker start isaac_ros_dev`；若容器不存在，脚本直接失败，不会创建容器。停止逻辑只向当前 run 目录记录且 PID=PGID 的进程组发信号，不使用 `pkill`。

## 配置与证据

- `config/d435i_ir_imu.yaml`：D435i `912112073953`，双 IR `640×480@30`，gyro 200 Hz，accel 250 Hz，合并 IMU 线性插值，RGB/深度/点云/投射器关闭。
- `config/isaac_vio.yaml`：Isaac ROS 3.2.6 / cuVSLAM 12.6 的 VIO-only 参数；输入是实际 `/camera/camera/...` 话题，不启动第二相机。
- `config/rosbag_qos.yaml`：传感器 best-effort 与 `/tf_static` transient-local 录制策略。
- `scripts/container_env.sh`：为当前容器补齐实际已有的 RSUSB 2.58.2 与 GXF 库路径。
- `PROGRESS.md`：根因、修改、回退点、测试结果和未通过项。
- `evidence/<时间戳>_vio/`：每次启动的新目录，包含 camera/VIO日志、在线 JSON 与完整 bag。不会覆盖旧试验。

当前已验证 bag：

`/home/cfly/ros2_ws/isaac_vio_debug/evidence/20260911_111114_vio/bag_20260911_111331`

## 已知限制

- D435i 报告 IMU calibration 不可用并采用默认内/外参；这不是已完成的相机—IMU标定。
- 长测中右 IR 稳定约 30 Hz，左 IR 偶有单帧缺失，VIO 会报告约 66.68 ms 间隔；暂不满足“零丢帧”验收。
- `unite_imu_method=2` 时合并 `/camera/camera/imu` 稳定约 200 Hz，但当前 VIO运行期间原始 `/camera/camera/accel/sample` 没有消息；完整 bag 中仍有合并 IMU 线加速度。
- 当前只完成静置意图基线；软件不能确认现场完全未移动，也没有卷尺真值或同步视频。1 m 手持往返与坐标/尺度验收仍须现场动作后另录新 run。
- 不要在相机节点占用 D435i 时并行运行 `rs-enumerate-devices`；出现 `RS2_USB_STATUS_BUSY` 属设备独占，先执行本脚本 `stop`。

