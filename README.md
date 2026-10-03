# ROS2 UAV Workspace

这是一个ROS2工作空间，用于无人机（UAV）任务开发和测试。包含PX4接口、任务执行和第三方库。

> 此上传版本是当前工作区的源码快照，子目录源码直接包含在仓库中。原始依赖来源见 `SOURCE_REPOSITORIES.json`，排除项见 `UPLOAD_NOTES.md`。

## 项目结构

```
ros2_ws/
├── src/
│   ├── px4_interface/     # PX4飞控接口包
│   ├── uav_task/         # 无人机任务包
│   ├── uav_tui_dashboard/# 终端仪表盘/状态监控
│   ├── realsense-ros/   # RealSense ROS驱动
│   └── third_party/     # 第三方依赖包
│       ├── PX4-Autopilot/          # PX4固件源码
│       ├── px4_msgs/               # PX4消息定义
│       ├── px4_ros_com/            # PX4 ROS通信示例/工具
│       ├── rplidar_ros/            # RPLIDAR ROS驱动
│       ├── isaac_ros_common/       # NVIDIA Isaac ROS通用组件
│       ├── isaac_ros_nitros/       # NVIDIA NITROS/GXF组件
│       └── isaac_ros_visual_slam/  # NVIDIA Visual SLAM
├── launch/               # 顶层SLAM/RTAB-Map启动文件
├── config/               # FastDDS、PX4参数和SD卡配置
├── build/                # 编译输出目录（已忽略）
├── install/              # 安装目录（已忽略）
├── log/                  # 日志目录（已忽略）
└── scripts/              # 启动脚本
```

## 依赖要求

- ROS 2 Humble Hawksbill
- PX4飞控固件
- Ubuntu 22.04 LTS

## 构建步骤

1. 安装依赖：
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install python3-colcon-common-extensions
   ```

2. 克隆并构建工作空间：
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

3. 构建特定包：
   ```bash
   colcon build --packages-select uav_task
   ```

## 使用方法

### 运行无人机任务测试

'''
cd ~/ros2_ws
colcon build --packages-select uav_task(选择不同依赖)
source install/setup.bash
'''

```bash
ros2 launch uav_task simple_flight_test.launch.py
```

### 运行SLAM到PX4链路
```bash
cd ~/ros2_ws
./scripts/run_isaac_dev.sh reset    # 首次迁移后建议重建容器挂载
./scripts/run_slam_px4.sh --bg
./scripts/run_slam_px4.sh --check-arm
```

容器内工作区路径统一为`/workspaces/ros2_ws`，宿主机路径统一为`~/ros2_ws`。

### 查看可用节点
```bash
ros2 pkg list
ros2 node list
```

## 开发指南

- 使用`colcon build`编译所有包
- 使用`source install/setup.bash`设置环境
- 日志文件位于`log/`目录
- 数据库文件`rtabmap.db`用于SLAM
- 使用`clangd-format`作为代码格式化标准

## 安全提醒

⚠️ **重要安全警告**:
- 无人机测试前确保安全环境
- 检查PX4配置和传感器校准
- 保持足够的安全距离
- 建议先在仿真环境中测试

## 贡献

1. Fork项目
2. 创建功能分支
3. 提交更改
4. 发起Pull Request

## 许可证

请查看各包的许可证信息。
