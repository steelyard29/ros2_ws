# D435i / Isaac ROS VIO 调试进度

试验编号：`20260911_isaac_vio_recovery`

本文件位于宿主机 `/home/cfly/ros2_ws/isaac_vio_debug/PROGRESS.md`，并通过现有 bind mount 映射到容器 `/workspaces/ros2_ws/isaac_vio_debug/PROGRESS.md`。所有新增配置与证据保存在该目录；不直接修改 `/opt/ros/humble` 的 launch 文件。

## 安全边界

- 当前阶段仅拆桨相机/VIO调试，不启动 PX4 bridge，不向飞控输入位姿。
- 未获授权：容器删除/重建、系统升级、飞控固件或参数修改、解锁、执行器和飞行操作。
- 不调用旧的 `run_slam_px4.sh`；其会启动 PX4 bridge，且停止逻辑含宽泛 `pkill`。
- 原有工作树已有大量修改与未跟踪文件，全部视为用户资产，不覆盖、不清理。

## Phase 0：接管、规则与现状盘点（完成）

时间：2026-09-11（Asia/Shanghai）

已执行：

- 完整读取 `/home/cfly/无人机资料收集与Jetson联调步骤.md`（301 行）。
- 完整读取 `/home/cfly/Jetson_Codex调试交接报告.md`（180 行），并读取同目录现有原始检查记录 `more.txt`。
- 搜索 `/home/cfly` 下 `AGENTS.md`；工程路径 `/home/cfly`、`/home/cfly/ros2_ws`、`/home/cfly/workspaces/isaac_ros` 均无适用项目级文件。命中项仅在 `.codex` 插件内部，不适用于本工程。
- 确认当前在 Jetson 宿主机：hostname `yahboom`，架构 `aarch64`，内核 `5.15.148-tegra`，`/.dockerenv` 不存在，`systemd-detect-virt --container` 返回 `none`。
- 盘点现有容器（未删除/重建）：`isaac_ros_dev`，镜像 `isaac_ros_dev-ros2ws:latest`，镜像 ID `sha256:dc4f356c918edc765da61bf4ce6b31d704fe4aea65284aed7fed0843f0a1f288`，NVIDIA runtime，host network，private IPC，privileged。
- 确认持久挂载：宿主机 `/home/cfly/ros2_ws` → 容器 `/workspaces/ros2_ws`；另有 `/dev` 和 `/tmp/.X11-unix` 挂载。
- 读取 `/home/cfly/ros2_ws/scripts/run_isaac_dev.sh`、`/home/cfly/workspaces/isaac_ros/run_dev.sh`、`launch_vslam.sh`、`run_slam_px4.sh` 与 `slam-px4.service`。两套容器入口使用不同默认镜像/挂载，当前实际容器以 inspect 结果为准。
- 宿主机确认 D435i `8086:0b3a` 以 USB 5 Gbit/s 连接；Microdia `0c45:6366` 在 USB 2 总线上。诊断前未发现正在运行的 RealSense/Visual SLAM 进程。
- 仅启动已有已停止容器以便诊断；未启动任何相机、VIO 或 PX4 节点。

回退点：容器原先状态为 `Exited (143)`。全部验收结束后已停止相机/VIO并执行 `docker stop isaac_ros_dev`，最终恢复为 `Exited (143)`。

## Phase A：RealSense 运行库修复（完成）

修复前证据：

- 首选工具为 `/opt/ros/humble/bin/rs-enumerate-devices`，归属 `ros-humble-librealsense2`。
- `ldd`：`librealsense2.so.2.58 => not found`。
- 已安装 Deb：`ros-humble-librealsense2 2.58.2-1jammy.20260615.124805`、`ros-humble-realsense2-camera 4.58.2-1jammy.20260615.141035`、`ros-humble-realsense2-camera-msgs 4.58.2-1jammy.20260615.120152`、Isaac Visual SLAM `3.2.6-0jammy`。
- 动态链接缓存仅有源码安装的 `/usr/local/lib/librealsense2.so.2.51.1`（SONAME 2.51）；禁止将其软链接伪装为 2.58。
- `/opt/ros/humble`、`/usr/lib` 中未找到真实 `librealsense2.so.2.58`。初步结论：2.58 Deb 的库文件与包数据库不一致/缺失，而非单纯搜索路径问题。

进一步诊断与修复：

- `dpkg -V` 确认 Deb 版 2.58 的库、SONAME 链接与 CMake 配置确实被删除；`docker diff` 同样显示这些路径为 `D`。
- 精确同版本重装模拟结果为 `0 upgraded, 0 newly installed, 1 reinstalled, 0 to remove`。实际获取因 ROS 主仓库已轮换该 Deb 而返回 HTTP 404，事务在下载阶段终止，没有安装/删除/升级任何包。
- 找到容器中已有 Jetson RSUSB 构建 `/opt/realsense_rsusb/lib/librealsense2.so.2.58.2`；其 SHA-256 为 `1d814bf8e9f206e924419647798f6c2e7a808607bfb124f2404cd6b92b4461c2`，与持久工作区构建产物逐字节哈希一致，ELF SONAME 为 `librealsense2.so.2.58`，工具报告版本 `2.58.2.0`。
- 临时 `LD_LIBRARY_PATH` 验证通过后，新增项目文件 `config/realsense_rsusb.conf`，并安装到当前容器 `/etc/ld.so.conf.d/99-realsense-rsusb.conf` 后执行 `ldconfig`。
- 修复后裸 `ldd /opt/ros/humble/bin/rs-enumerate-devices` 与 `ldd /opt/ros/humble/lib/librealsense2_camera.so` 都解析到 `/opt/realsense_rsusb/lib/librealsense2.so.2.58`，无 RealSense `not found`。
- 修复后裸枚举成功：`Intel RealSense D435I / 912112073953 / 5.13.0.55`。

保留项：旧 `/usr/local/lib/librealsense2.so.2.51.1` 未删除，但不同 SONAME 不会替代 2.58；未伪造软链接，未升级系统或 ROS 包。

回退：

```bash
docker exec isaac_ros_dev rm /etc/ld.so.conf.d/99-realsense-rsusb.conf
docker exec isaac_ros_dev ldconfig
```

回退后会恢复原先的 2.58 加载失败，原文件仍保留在持久工作区。

## Phase B：独立 D435i 双 IR + IMU（完成，带已知缺口）

新增配置：`config/d435i_ir_imu.yaml`。

- 固定序列号 `912112073953`；真实节点名 `/camera/camera`。
- 实际打开模式经启动日志确认：Infra1/2 均为 Y8 `640×480@30`，Accel `250 Hz`，Gyro `200 Hz`；RGB、深度和点云关闭，`unite_imu_method=2`。
- 实际话题前缀为 `/camera/camera`，不是安装示例假定的 `/camera`。
- image frame 分别为 `camera_infra1_optical_frame` 与 `camera_infra2_optical_frame`；合并 IMU frame 为 `camera_imu_optical_frame`。静态 TF 可查询，左右相机约 50 mm 基线。
- CameraInfo 为 640×480 rectified `plumb_bob`；右目 P 矩阵含 `Tx=-19.0158`。两目 CameraInfo header 都使用左目 optical frame，而两路 Image header 各自使用左右 optical frame，已按实际记录，不擅自改写。
- 相机日志确认 wrapper/build/runtime 都为 `4.58.2 / 2.58.2 / 2.58.2`。

已知缺口：每次启动出现一项 `IMU Calibration is not available, default intrinsic and extrinsic will be used`，并出现一次 Motion Module hardware notification（日志中以 XXX 和普通格式各写一行）；但合并 IMU 与 gyro 持续出流。该告警保留为标定/硬件检查项。

## Phase C：独立 Isaac ROS VIO（完成，未桥接 PX4）

新增 `config/isaac_vio.yaml`、`scripts/container_env.sh`、`scripts/container_stack.sh` 与宿主机入口 `vio_stack.sh`。

- 安装版 VSLAM 首次探测还暴露 `libgxf_serialization.so` 不在动态链接路径；文件属于已安装 `ros-humble-isaac-ros-gxf 3.2.5`。新增脚本仅在本流程中收集 GXF `lib` 子目录进入 `LD_LIBRARY_PATH`，没有全局升级或复制库。
- VIO-only 使用已运行相机的真实话题，未启动第二个相机节点；`imu_frame` 使用实际合并消息 frame `camera_imu_optical_frame`。
- 30 Hz 周期为 33.33 ms，`image_jitter_threshold_ms=40`，使约 66.67 ms 的漏帧仍报警而不是被大阈值掩盖。
- 日志确认 `cuVSLAM version: 12.6`、GPU warm-up、`Enable IMU Fusion: true` 和 tracker successful initialization。
- `/visual_slam/status.vo_state=1`；连续里程计为 `/visual_slam/tracking/odometry`，frame `odom`、child `camera_link`，约 30 Hz。未向 PX4 发布任何位姿。
- 第一次手工后台测试的 CLI PID 不是进程组长，已在核实成员后精确停止对应 PGID。新脚本用 `setsid`，实际验收 PID=PGID，并成功执行两轮精确 stop，无残留。
- 第二轮在完整 `docker stop`/现有容器重启后仍成功启动相机、初始化 tracker 并输出 odometry，证明配置跨容器重启可复现。未删除或重建容器；最终容器恢复停止状态。

## Phase D：频率、时间戳与 rosbag 验证（完成，零丢帧与真值运动未通过）

在线 20 秒 JSON：`evidence/20260911_111114_vio/online_validation_20260911_111814.json`。

- 所有收到的 header 时间戳单调。
- Infra2 586 条，29.996 Hz，最大间隔 33.340 ms；Infra1 582 条，29.791 Hz，最大间隔 66.677 ms。
- 582 对左右图像时间戳精确相等；4 个右目帧没有左目配对。
- 合并 IMU 3906 条，199.911 Hz，最大间隔 5.004 ms；gyro 3904 条，199.911 Hz。
- odometry/status 各 535 条，29.884 Hz；535 条 status 全部 `vo_state=1`。

在线 60 秒静置意图 JSON：`evidence/20260911_111114_vio/online_validation_20260911_111944.json`。

- Infra2 1799 条，29.995 Hz，无 >33.345 ms 间隔；Infra1 1785 条，29.762 Hz，最大 66.686 ms；1785 对时间戳精确匹配，14 个右目帧没有左目配对。
- 合并 IMU 11985 条，199.890 Hz，最大间隔 10.006 ms；gyro 11984 条，199.907 Hz。所有时间戳单调。
- odometry/status 各 1788 条，29.828 Hz，全部 `vo_state=1`；窗口内终点相对起点位移 0.023022 m，最大偏移 0.068646 m。软件不能独立确认现场无人触碰，因此不升级为严格静态漂移真值。
- 第一次验证器使用 ROS timer，在高频回调下未结束并生成 0 字节文件 `online_validation_20260911_111601.json`；已核实并精确停止其 PGID，修复为墙钟 `spin_once`，保留失败文件与说明，不覆盖证据。

已完成短包：`evidence/20260911_111114_vio/bag_20260911_111331/`。

- SQLite3，14.3006 秒，251.5 MiB，9547 条消息，`metadata.yaml` 和数据库完整闭合；`bag_info.txt` 已持久保存。
- VIO status 419、odometry 420、TF 838、static TF 1；Infra2/Image+Info 各 429，Infra1 image 417、Info 429；合并 IMU 2861、gyro 2861；IMUInfo/各外参均有 1 条。
- 原始 accel topic 被请求并订阅但计数为 0；在线 VIO运行期间也无法获取该 topic，而合并 IMU 约 200 Hz 且含有效线加速度。标记为未通过项，不删除空 topic，不伪报完整。
- VIO日志长期累计 86 次帧间隔告警，其中多数约 66.68 ms，另有 133/166/366/766 ms 个例；采集与 Python 验证订阅会增加负载，仍需在后续 5 分钟基线中区分驱动、USB与订阅负载影响。

未执行且仍需现场真值动作：卷尺 1 m 前后/左右/上下往返、同步视频、严格无人触碰静置。未执行 PX4通信、外部位姿、参数修改、解锁或飞行。

## 最终离线验收与交还状态（完成）

- `d435i_ir_imu.yaml`、`isaac_vio.yaml`、`rosbag_qos.yaml` 均通过 Python `yaml.safe_load`。
- `validate_online.py` 通过 `py_compile`；`vio_stack.sh`、`container_env.sh`、`container_stack.sh` 均通过 `bash -n`，入口文件具有执行权限。
- 配置与脚本 SHA-256 固化在 `MANIFEST.sha256`；可在本目录执行 `sha256sum -c MANIFEST.sha256` 离线复核。
- bag 目录包含 263757824 字节 DB、9527 字节 `metadata.yaml` 与 2699 字节 `bag_info.txt`。
- 当前 `isaac_ros_dev` 状态为 `Exited (143)`，restart policy `no`，持久挂载仍为 `/home/cfly/ros2_ws:/workspaces/ros2_ws`。
- `slam-px4.service` 未安装到 `/etc/systemd/system` 或 `/lib/systemd/system`，systemd 报 inactive；home 中的旧服务文件未启用。
- `git status --short -- isaac_vio_debug` 只显示新增独立目录 `?? isaac_vio_debug/`；未清理或覆盖原工作树的既有修改。

## Phase E：5 分钟静置与负载复测（完成，未达到飞行放行条件）

时间：2026-09-11（Asia/Shanghai）

证据目录：`evidence/20260911_113449_vio/`。

### 5 分钟在线验证

`online_validation_20260911_113523.json` 完整结束，墙钟 300 秒：

- 全部流的 header 时间戳单调，VIO status 共 8887 条且全部 `vo_state=1`。
- odometry 约 29.722 Hz；静置意图窗口终点相对起点三维位移 0.043473 m，分量为 `[-0.035697, 0.012126, 0.021647] m`，最大三维偏移 0.051980 m。仅水平终点位移约 0.03770 m。
- Infra1 8768 条、约 29.482 Hz、最大间隔 100.087 ms；Infra2 8910 条、约 29.956 Hz、最大间隔 66.741 ms。精确同步对 8759，左目比右目少 142 条；丢帧仍未通过。
- 合并 IMU 59435 条、约 199.833 Hz；gyro 59440 条、约 199.846 Hz；raw accel 本轮恢复为 75923 条、约 255.267 Hz。此前 raw accel 为 0 不是稳定复现故障。

### rosbag

两个包均通过 SIGINT 正常闭合并生成 `.db3`、`metadata.yaml` 和 `bag_info.txt`：

- `bag_20260911_113536/`：298.145 秒，5.2 GiB，275252 条消息。Infra1 8745、Infra2 8932，相差 187（约占右目 2.09%）；odometry/status 各约 8828/8827；raw accel 76095，合并 IMU 59575。
- `bag_20260911_114334/`：531.564 秒，9.2 GiB，491069 条消息。Infra1 15751、Infra2 15934，相差 183（约占右目 1.15%）；odometry/status 各 15773；raw accel 135655，合并 IMU 106221。
- 第二包原为无限时长 `record 0`，检查时仍在录制且仅有 DB。为避免填满磁盘，仅向该次 rosbag 的容器内精确 PGID 354 发送 SIGINT；未停止或影响相机/VIO，随后确认元数据完整。录包后根文件系统剩余约 72 GiB。

第一包同时有在线验证器额外订阅，第二包无验证器。左右差值比例由约 2.09% 降至约 1.15%，说明额外订阅/拷贝负载会放大丢帧，但该对比仍同时受运行时段影响，不能据此唯一归因为 CPU、USB 或驱动。尚缺同等时长的“仅相机”和“相机 + VIO、不录包”基线。

### 日志与硬件风险

- 本轮 VIO 日志有 211 次帧间隔超过 40 ms：192 次小于 80 ms、15 次为 80--120 ms、4 次为 120--180 ms，最大 166.843 ms。
- 相机日志再次出现 `IMU Calibration is not available` 1 次、Motion Module failure 2 行（同一通知的两种日志格式）、libusb control transfer `Resource temporarily unavailable` 82 次，以及 `Asic Temperature value is not valid` 9 次。
- 同期内核日志未发现 RealSense 对应的 USB reset、UVC 断连、带宽不足或重连记录。因此不能把问题定性为整条 USB 链路断连；设备控制通道、固件/硬件状态和订阅负载仍需分别排除。
- 对第一份 5 分钟静置 bag 直接只读反序列化：raw accel 均值 `[-0.006989, -9.920314, 0.015425] m/s^2`，模长均值 9.920352、标准差 0.019047 m/s^2；raw gyro 均值 `[-0.000208, -0.000966, -0.001649] rad/s`，各轴标准差约 0.00221--0.00232 rad/s。数据连续且未显示失控噪声，但重力模长较标准值高约 1.1%，gyro 静态偏置模长约 0.00192 rad/s（约 0.11 deg/s）；结合缺失标定与硬件通知，仍不能视为已通过 IMU 硬件验收。

### 当前结论和下一闸门

- 5 分钟静置 VIO 漂移量不算失控，但它只证明相机坐标下 VIO 能持续跟踪；没有验证相机到机体外参、ENU/NED 与 FLU/FRD 转换、延迟、PX4 EKF 视觉融合/拒绝状态、起桨振动或磁航向，因此**不构成飞行放行**，也**不能确定历史水平漂移的唯一原因**。
- 下一步先在拆桨且不连接 PX4 位姿输入的条件下完成四组单轴真值动作，每组独立录包并同步录像：水平前后 1.00 m、水平左右 1.00 m、竖直上下 0.50 m、原地偏航 90 度并返回。平移时机体保持水平且不改变朝向；旋转时相机光心尽量不平移。每组按“起点静止 10 秒 -> 5 秒匀速到终点 -> 终点静止 10 秒 -> 5 秒匀速返回 -> 起点静止 15 秒”执行。
- 真值动作通过标准：VIO 轴向/符号可重复；1 m 平移尺度误差建议先控制在 10% 内、0.5 m 竖直误差在 0.05 m 内、90 度偏航误差在 5 度内；回到起点后水平残差不大于 0.05 m；全过程保持 `vo_state=1`，不得出现重定位跳变或持续增长的离轴漂移。该阈值是进入 PX4 只读/台架融合检查的工程闸门，不是飞行安全认证。
- 真值通过后再进行 PX4 **只读**检查：串口与 uXRCE-DDS Agent、`px4_msgs` 匹配、主 EKF 实例、local position validity、heading validity、估计器创新/拒绝状态、时间同步及当前外参/坐标约定。向 `/fmu/in/vehicle_visual_odometry` 实际发布仍须另行给出话题、变换、时间戳、协方差、失效停止和回退方案，并等待明确授权。
- 本轮分析结束后，精确停止本 run 的相机/VIO 进程组并确认容器内无残留 ROS 进程；随后仅停止现有 `isaac_ros_dev` 容器，恢复为 Exited 状态。未删除或重建容器。

## Phase F：D435i IMU 六面标定准备（阻塞于 Motion Module 当前无数据）

时间：2026-09-11（Asia/Shanghai）

- 用户指定参考 CSDN 文章 `weixin_42705114/article/details/109721864`。CSDN 在本机直连和既有代理下均于 TLS 握手阶段提前断开；改为完整读取工作区同版本 librealsense 2.58.2 官方 `tools/rs-imu-calibration/README.md` 和 `rs-imu-calibration.py`。官方流程与该类文章一致：六面静置采集 -> 保存 accel/gyro 原始数据与 `calibration.json`/`calibration.bin` -> 用户确认后通过调试协议命令 `0x50` 写入 Motion Module EEPROM。
- 容器原先缺少 `pyrealsense2`。新增独立、可回退构建目录 `tools/librealsense_py_build/`，从工作区 librealsense 2.58.2 源码以 `BUILD_PYTHON_BINDINGS=ON`、`FORCE_RSUSB_BACKEND=ON` 构建；未安装系统包、未覆盖 `/opt/ros` 或现有构建。模块验证版本为 2.58.2，并能唯一枚举 D435i 序列号 `912112073953`、固件 `5.13.0.55`。
- 建立标定证据目录 `evidence/20260911_d435i_imu_calibration_01/`。尝试官方 `MMER (0x4F)` 只读备份当前 Motion Module EEPROM，设备返回错误 `0xFFFFFFEB`，未获得 520 字节备份；没有发送 EEPROM 写命令。
- 第一次运行官方六面脚本时可枚举 Motion Module profiles，但在第一姿态前因没有收到任何 IMU callback 报 `No IMU data. Check connectivity.` 并安全退出；未生成或写入标定值。
- 随后分别用 Python pipeline、Motion Module sensor callback 和原生 `frame_queue` 指定 accel 250 Hz / gyro 200 Hz 做 5 秒读取，计数均为 0。
- 用原先已验证的 ROS 配置进行独立 10 秒对照，证据为 `evidence/20260911_123658_vio/online_validation_20260911_123714.json`：双 IR/CameraInfo 正常约 30 Hz，但 raw accel、raw gyro、合并 IMU、VIO odometry/status 全为 0。由此确认当前不是 Python 回调兼容问题，而是本次连接状态下 Motion Module 未出流；与此前相机日志中的 `Motion Module failure` 相吻合。
- 已精确停止对照测试的相机/VIO 进程。当前阻塞动作是现场物理断电：从 Jetson 端拔出 D435i USB，等待至少 30 秒后直接接回 USB 3.x 端口。只有复测 accel/gyro 恢复后才继续六面采集；在拟合检查、原始样本保存和 EEPROM 无备份风险确认之前不得写入设备。
