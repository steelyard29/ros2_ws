# PX4 Interface

PX4 Interface 是构建在 ROS 2 (Humble+) 与 PX4 飞控之间的数据桥接与控制层。
它在保持 PX4 原生 Offboard 接口能力的同时，提供线程安全的状态缓存、
清晰的外部话题暴露以及简化的控制 API，便于高层算法与地面站快速集成。

## 📌 功能概览

- **PX4Gateway 节点**：订阅 `/fmu/out/*` 消息、维护心跳，并向 PX4 发布控制命令。
- **Px4MsgsCache 缓存层**：线程安全地缓存车辆状态、位置、电池信息，支持高并发访问。
- **统一话题接口**：将原始 PX4 消息转换为 `px4_interface/msg/*` 自定义消息，供上层模块消费。
- **Offboard 控制辅助**：封装解锁、切换模式、紧急停机等常用命令，并持续发布 Offboard 心跳。
- **完备测试**：自带多线程压力与竞态场景的 gtest 测试，附 `run_tests.sh` 脚本一键运行。

## 🏗️ 架构概览

```mermaid
flowchart LR
	PX4[PX4 Flight Controller]
	PX4 -->|/fmu/out/vehicle_status\n/fmu/out/vehicle_odometry\n/fmu/out/battery_status| Gateway
	Gateway[[PX4Gateway (rclcpp::Node)]]
	Gateway -->|更新| Cache[Px4MsgsCache]
	Cache -->|线程安全读取| Gateway
	Gateway -->|/cache/vehicle_status\n/cache/vehicle_odometry\n/cache/battery_status| ROSApp[上层 ROS 2 节点]
	ROSApp -->|调用 API| Gateway
	Gateway -->|/fmu/in/offboard_control_mode\n/fmu/in/trajectory_setpoint\n/fmu/in/vehicle_command| PX4
```

组件职责简述：

| 组件 | 主要职责 | 关键实现 |
| --- | --- | --- |
| `PX4Gateway` | ROS 2 节点，桥接 PX4 ↔ ROS 2 数据流；封装控制命令 | `src/px4_gateway.cpp`
| `Px4MsgsCache` | 线程安全缓存位姿、电池、飞行状态 | `src/px4_msgs_cache.cpp`
| 自定义消息 | 对外发布聚合后的状态信息 | `msg/*.msg`
| 单元测试 | 并发、压力测试覆盖缓存一致性 | `test/test_px4_msgs_cache.cpp`

## 🔄 话题与消息

### PX4Gateway 订阅 / 发布话题

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/fmu/out/vehicle_status` | `px4_msgs/msg/VehicleStatus` | PX4 提供的飞控状态 |
| 订阅 | `/fmu/out/vehicle_odometry` | `px4_msgs/msg/VehicleOdometry` | 位姿与速度（NED 坐标 + 姿态） |
| 订阅 | `/fmu/out/battery_status` | `px4_msgs/msg/BatteryStatus` | 电池电量、电压、告警 |
| 发布 | `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | 50 Hz 心跳，声明控制模式 |
| 发布 | `/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` | Offboard 位置/速度设定点 |
| 发布 | `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | 解锁、模式切换等命令 |
| 发布 | `/cache/vehicle_status` | `px4_interface/msg/VehicleStatus` | 缓存后的飞控状态，时间戳自节点时钟 |
| 发布 | `/cache/vehicle_odometry` | `px4_interface/msg/PoseNED` | 缓存后的位姿信息（含位置、速度、姿态） |
| 发布 | `/cache/battery_status` | `px4_interface/msg/BatteryStatus` | 缓存后的电池信息 |

### 自定义消息字段

#### `px4_interface/msg/VehicleStatus`

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| `valid` | `bool` | 数据是否有效 |
| `latest_timestamp` | `builtin_interfaces/Time` | Gateway 接收到数据的本地时间 |
| `arming_state` | `uint8` | PX4 解锁状态，参考 `px4_msgs::msg::VehicleStatus` |
| `nav_state` | `uint8` | 导航状态（`px4Enum::NavState`） |
| `failsafe` | `bool` | 是否处于故障保护 |
| `pre_flight_checks_pass` | `bool` | 飞行前自检是否通过 |

#### `px4_interface/msg/PoseNED`

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| `valid` | `bool` | `vehicle_odometry` 消息转换后是否有效 |
| `translation` | `float64[3]` | NED 坐标位置 (m) |
| `velocity` | `float64[3]` | NED 线速度 (m/s) |
| `orientation` | `float64[4]` | 四元数 `(w, x, y, z)`；PX4 默认无姿态信息时为单位四元数 |
| `timestamp` | `builtin_interfaces/Time` | Gateway 获取数据时间 |

#### `px4_interface/msg/BatteryStatus`

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| `valid` | `bool` | 数据是否有效 |
| `timestamp` | `builtin_interfaces/Time` | Gateway 获取数据时间 |
| `voltage_v` | `float32` | 电池电压 |
| `current_a` | `float32` | 电流 |
| `remaining` | `float32` | 剩余电量比例（0~1） |
| `warning` | `uint8` | 告警级别，与 PX4 枚举一致 |

## 🧠 控制 API 摘要

`PX4Gateway` 作为 rclcpp 节点提供以下高层接口，可在自定义控制器或任务管理器中直接调用：

| 方法 | 功能 |
| --- | --- |
| `bool checkPx4Publishers()` | 启动时检查 PX4 是否已经在关键话题上发布消息，判断连接是否正常 |
| `void chooseControlMethod(bool position_control, bool velocity_control)` | 选择 Offboard 控制维度（位置 / 速度，可同时开启） |
| `void setTarget(const px4GatewayTypes::setpoint &target)` | 使用简化结构发送位置 + 速度 + 偏航指令，内部自动填充时间戳 |
| `void setTarget(const px4_msgs::msg::TrajectorySetpoint &target)` | 直接转发自定义的 TrajectorySetpoint |
| `void publishVehicleCommand(px4Enum::VehicleCommand cmd, float param1, float param2)` | 发送任意 PX4 VehicleCommand |
| `setArmMode()` / `setDisarmMode()` | 常用 ARM / DISARM 命令封装 |
| `setOffboardMode()` | 切换到 Offboard 控制模式（`VEHICLE_CMD_DO_SET_MODE`） |
| `setLandMode()` | 触发自动降落 |
| `triggerEmergencyStop()` | 紧急停机（`VEHICLE_CMD_DO_FLIGHTTERMINATION`） |

缓存访问：

```cpp
auto cache = std::make_shared<Px4MsgsCache>();
auto status = cache->getVehicleStatus();
cache->accessPositionNED([](const px4Position::PositionNED &ned) {
	if (ned.valid) {
		// 在锁保护下安全访问数据
	}
});
```

## 🚀 快速上手

### 环境要求

- ROS 2 Humble (或更新版本)
- PX4 Autopilot 1.13+，开启 MAVLink/RTPS 桥接
- `px4_msgs` 对应版本（应与飞控固件匹配）
- Eigen3

在 ROS 2 工作区 `ros2_ws` 中，确保已初始化并导入 PX4 使用的 RTPS IDL。

### 构建

```bash
cd /home/cfly/ros2_ws
colcon build --packages-select px4_interface
source install/setup.bash
```

### 启动 PX4Gateway 示例

该仓库未内置 launch 文件，可在自定义包或临时脚本中启动：

```cpp
#include <rclcpp/rclcpp.hpp>
#include <px4_interface/px4_gateway.hpp>
#include <px4_interface/px4_msgs_cache.hpp>

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto cache = std::make_shared<Px4MsgsCache>();
	auto node = std::make_shared<PX4Gateway>(rclcpp::NodeOptions(), cache);
	if (!node->checkPx4Publishers()) {
		RCLCPP_WARN(node->get_logger(), "PX4 通讯未就绪");
	}
	node->chooseControlMethod(true, true);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
```

部署到自定义可执行文件后，即可通过 ROS 2 话题订阅缓存数据、并调用接口下发控制命令。

### 常见使用流程

1. 启动 PX4（仿真或实机），确认 RTPS Agent 正常转发。  
2. 启动 `PX4Gateway`，观察日志确认订阅/发布建立。  
3. 调用 `setArmMode()` 解锁，随后 `setOffboardMode()` 切换模式。  
4. 通过 `setTarget()` 周期性发送轨迹设定点（至少 2 Hz，推荐 ≥10 Hz）。  
5. 若需降落或紧急停机，调用对应封装方法即可。  

> ⚠️ **安全提示**：Offboard 模式下需要持续发送设定点和心跳；若出现通信中断，PX4 将进入 failsafe。

## 🧪 测试

运行单元测试（包含多线程竞态场景）：

```bash
cd /home/cfly/ros2_ws/src/px4_interface
./run_tests.sh
```

脚本将调用 `colcon build`、`colcon test` 并输出结果摘要。
也可手动执行：

```bash
colcon test --packages-select px4_interface --event-handlers console_direct+
colcon test-result --all --verbose
```

