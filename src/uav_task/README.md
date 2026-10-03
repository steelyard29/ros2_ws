# UAV Task 包

一个简单的无人机飞行测试包，演示基本的起飞、悬停和降落功能。

## 功能描述

这个包实现了一个简单的无人机飞行任务：
1. 解锁无人机
2. 切换到offboard模式
3. 起飞到1米高度
4. 在1米高度悬停3秒
5. 降落
6. 上锁无人机

## 依赖

- PX4 飞控固件
- px4_msgs
- ROS 2 Humble
- rclcpp

## 编译

```bash
cd /home/cfly/ros2_ws
colcon build --packages-select uav_task
source install/setup.bash
```

## 使用方法

### 方法1: 直接运行节点
```bash
ros2 run uav_task simple_flight_test
```

### 方法2: 使用launch文件
```bash
ros2 launch uav_task simple_flight_test.launch.py
```

### competition_obstacle_course.launch.py
'''
source /opt/ros/humble/setup.bash
source /home/cfly/ros2_ws/install/setup.bash
ros2 launch uav_task /home/cfly/ros2_ws/src/uav_task/launch/competition_obstacle_course.launch.py
'''

## 重要说明

⚠️ **安全提醒**: 
- 确保无人机在安全的环境中测试
- 确保PX4飞控已正确配置
- 确保有足够的起飞和降落空间
- 建议先在仿真环境中测试

## 程序逻辑

程序使用状态机实现飞行控制：

1. **INIT**: 初始化状态，发布几次setpoint后切换到解锁状态
2. **ARMING**: 发送解锁命令，等待无人机解锁成功
3. **TAKEOFF**: 发送起飞到1米高度的position setpoint
4. **HOVER**: 在1米高度悬停3秒
5. **LANDING**: 发送降落到地面的position setpoint
6. **DISARMING**: 发送上锁命令
7. **COMPLETED**: 任务完成

## 话题接口

### 发布的话题：
- `/fmu/in/offboard_control_mode` - Offboard控制模式
- `/fmu/in/trajectory_setpoint` - 轨迹设定点
- `/fmu/in/vehicle_command` - 飞行器命令

### 订阅的话题：
- `/fmu/out/vehicle_local_position` - 本地位置信息
- `/fmu/out/vehicle_status` - 飞行器状态

## 参数配置

- `TARGET_HEIGHT`: 目标起飞高度（默认1米，在NED坐标系中为-1.0）
- `POSITION_TOLERANCE`: 位置到达误差容忍度（默认0.1米）

## 坐标系说明

程序使用PX4的NED坐标系：
- X: 北（前）
- Y: 东（右）  
- Z: 下（向下为正）

因此1米高度在代码中表示为z = -1.0

## 故障排除

如果程序无法正常工作，请检查：
1. PX4飞控是否正常运行
2. ROS 2与PX4的通信是否正常
3. 相关话题是否存在：`ros2 topic list | grep fmu`
4. 无人机是否在合适的环境中（GPS信号等）
