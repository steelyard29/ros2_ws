# UAV Task API Documentation

本文档描述了无人机任务系统的三个核心组件的API接口。

## 目录

- [PX4Communicator](#px4communicator) - PX4飞控通信接口
- [VelocityAPFController](#velocityapfcontroller) - 速度人工势场控制器
- [VisualServoController](#visualservocontroller) - 视觉伺服控制器

---

## PX4Communicator

PX4Communicator是与PX4飞控系统通信的核心接口，提供安全的飞行控制功能。

### 概述

该类封装了所有与PX4飞控的通信，包括位置/速度控制、状态监控和安全保护机制。它实现了多层安全检查，确保在各种异常情况下无人机的安全。

### 核心特性

- **多种控制模式**：支持位置控制、速度控制、混合控制
- **安全状态机**：自动处理通信超时、数据丢失等异常情况
- **线程安全**：所有公共接口都是线程安全的
- **自动保护**：主程序超时自动悬停，紧急情况自动降落

### 数据结构

#### Position
```cpp
struct Position {
    bool valid = false;        // 数据是否有效
    double x = 0.0;           // X坐标 (米)
    double y = 0.0;           // Y坐标 (米) 
    double z = 0.0;           // Z坐标 (米，负值表示高度)
    float yaw = 0.0f;         // 偏航角 (弧度)
    rclcpp::Time timestamp;   // 时间戳
};
```

#### ControlStatus
```cpp
struct ControlStatus {
    bool valid = false;       // 数据是否有效
    bool is_armed = false;    // 是否解锁
    bool is_offboard = false; // 是否在Offboard模式
    rclcpp::Time timestamp;   // 时间戳
};
```

#### CommandResult
```cpp
struct CommandResult {
    bool accepted = false;    // 命令是否被接受
    std::string reason;       // 失败原因（如果适用）
};
```

#### CommState (通信状态)
```cpp
enum class CommState {
    NORMAL,                   // 正常状态
    MAIN_TIMEOUT_HOLD,       // 主程序超时悬停
    PX4_DATA_TIMEOUT_HOLD,   // PX4数据超时悬停
    EMERGENCY_LAND           // 紧急降落
};
```

### 公共接口

#### 构造函数
```cpp
explicit PX4Communicator(rclcpp::Node* node);
```
- **参数**：`node` - 指向ROS节点的指针
- **说明**：初始化通信接口，创建发布者和订阅者

#### 核心更新函数
```cpp
void update();
```
- **说明**：必须在主循环中定期调用（建议50-100Hz）
- **功能**：处理安全状态机、超时检查、紧急保护逻辑

#### 位置控制
```cpp
bool setPositionSetpoint(float x, float y, float z, float yaw);
```
- **参数**：
  - `x, y, z`：目标位置 (米)
  - `yaw`：目标偏航角 (弧度)
- **返回值**：成功返回true，紧急状态下返回false
- **说明**：设置位置设定点，飞控将自动规划路径

#### 速度控制
```cpp
bool setVelocitySetpoint(float vx, float vy, float vz, float yaw);
```
- **参数**：
  - `vx, vy, vz`：目标速度 (米/秒)
  - `yaw`：目标偏航角 (弧度)
- **返回值**：成功返回true，紧急状态下返回false
- **说明**：设置速度设定点，需要上层算法持续更新

#### 混合控制
```cpp
bool setPositionAndVelocitySetpoint(float x, float y, float z, 
                                   float vx, float vy, float vz, float yaw);
```
- **参数**：同时包含位置和速度参数
- **返回值**：成功返回true，紧急状态下返回false
- **说明**：同时控制位置和速度，提供更精确的控制

#### 飞行命令
```cpp
CommandResult arm();                // 解锁
CommandResult setOffboardMode();    // 切换到Offboard模式
void disarm();                      // 上锁
void land();                        // 降落
```

#### 状态查询
```cpp
Position getCurrentPosition() const;         // 获取当前位置
ControlStatus getControlStatus() const;      // 获取控制状态
CommState getCommunicationState() const;    // 获取通信状态
```

### 安全机制

1. **主程序超时保护**：如果500ms内未收到控制指令，自动悬停
2. **PX4数据超时保护**：如果500ms内未收到PX4数据，进入紧急状态
3. **紧急悬停超时**：紧急悬停10秒后自动降落
4. **状态机保护**：一旦进入紧急状态，拒绝所有控制指令

### 使用示例

```cpp
// 初始化
auto node = std::make_shared<rclcpp::Node>("test_node");
PX4Communicator px4(node.get());

// 主循环
rclcpp::Rate rate(50); // 50Hz
while (rclcpp::ok()) {
    px4.update(); // 必须调用
    
    // 检查状态
    if (px4.getCommunicationState() == PX4Communicator::CommState::NORMAL) {
        // 发送控制指令
        px4.setPositionSetpoint(1.0f, 2.0f, -3.0f, 0.0f);
    }
    
    rclcpp::spin_some(node);
    rate.sleep();
}
```

---

## VelocityAPFController

VelocityAPFController是一个基于人工势场法的速度控制器，集成了PID航点跟踪和激光雷达避障功能。

### 概述

该控制器实现了智能的航点序列飞行，能够在遇到障碍物时自动避障，同时保持向目标航点的前进。它使用PID控制器进行精确的位置控制，并使用人工势场法实现实时避障。

### 核心特性

- **航点序列管理**：支持多航点顺序飞行
- **PID位置控制**：精确的三维位置控制
- **实时避障**：基于激光雷达的人工势场避障
- **配置化设计**：支持YAML配置文件
- **状态反馈**：提供详细的控制状态信息

### 数据结构

#### Waypoint
```cpp
struct Waypoint {
    float x, y, z;              // 坐标 (米)
    std::string description;    // 描述信息
    
    Waypoint(float x = 0.0f, float y = 0.0f, float z = 0.0f, 
             const std::string& desc = "");
};
```

#### InputData
```cpp
struct InputData {
    // 当前状态
    float current_x, current_y, current_z;  // 当前位置
    float current_yaw;                      // 当前偏航角
    
    // 激光雷达数据
    sensor_msgs::msg::LaserScan::SharedPtr scan_data;
    
    // 控制标志
    bool avoidance_enabled;                 // 是否启用避障
};
```

#### OutputVelocity
```cpp
struct OutputVelocity {
    float vx, vy, vz;                    // 输出速度 (米/秒)
    bool is_pid_active;                  // PID控制是否激活
    bool is_avoidance_active;            // 避障是否激活
    bool waypoint_reached;               // 当前航点是否到达
    bool sequence_completed;             // 序列是否完成
    size_t current_waypoint_index;       // 当前航点索引
};
```

#### PIDConfig
```cpp
struct PIDConfig {
    float kp_x, kp_y, kp_z;           // 比例增益
    float kd_x, kd_y, kd_z;           // 微分增益
    float ki_x, ki_y, ki_z;           // 积分增益
    float max_velocity_xy;            // 水平最大速度
    float max_velocity_z;             // 垂直最大速度
    float derivative_filter_alpha;    // 微分滤波系数
    float position_tolerance;         // 位置容差
};
```

#### AvoidanceConfig
```cpp
struct AvoidanceConfig {
    std::string laser_topic;              // 激光雷达话题
    float obstacle_distance_threshold;    // 障碍物距离阈值
    float avoidance_fov_degrees;         // 避障视场角
    float avoidance_strength_gain;       // 斥力增益
    float max_avoidance_velocity;        // 最大避障速度
    int scan_timeout_ms;                 // 扫描超时时间
    float min_valid_ranges_ratio;        // 最小有效点比例
};
```

### 公共接口

#### 构造函数
```cpp
explicit VelocityAPFController(rclcpp::Node* node);
```
- **参数**：`node` - 指向ROS节点的指针
- **说明**：初始化控制器，加载配置文件

#### 航点序列管理
```cpp
void setWaypointSequence(const std::vector<Waypoint>& waypoints);
```
- **参数**：`waypoints` - 航点序列
- **说明**：设置新的航点序列，重置控制状态

```cpp
void resetSequence();
```
- **说明**：重置到第一个航点，清除所有状态

#### 核心控制接口
```cpp
OutputVelocity calculateVelocity(const InputData& input);
```
- **参数**：`input` - 当前状态和环境信息
- **返回值**：计算得到的安全速度指令
- **说明**：核心算法接口，集成PID控制和避障

#### 状态查询
```cpp
const Waypoint& getCurrentWaypoint() const;     // 当前目标航点
const PIDConfig& getPIDConfig() const;          // PID配置
const AvoidanceConfig& getAvoidanceConfig() const; // 避障配置
size_t getWaypointCount() const;                 // 航点总数
size_t getCurrentWaypointIndex() const;          // 当前航点索引
bool isSequenceCompleted() const;               // 序列是否完成
```

### 控制算法

#### PID位置控制
- 使用三维PID控制器计算到达目标航点的速度
- 支持可配置的PID参数
- 包含微分项低通滤波，减少噪声影响

#### 人工势场避障
- 基于激光雷达数据计算斥力场
- 可配置的视场角和距离阈值
- 斥力速度与吸引力速度（PID输出）叠加

#### 航点序列管理
- 自动检测航点到达（基于距离阈值）
- 每个航点悬停2秒后前进到下一个
- 完成所有航点后发布完成信号

### 配置文件

控制器支持YAML配置文件，位置：`config/obstacle_avoidance_config.yaml`

```yaml
position_controller:
  kp_x: 0.8
  kp_y: 0.8
  kp_z: 0.6
  kd_x: 0.1
  kd_y: 0.1
  kd_z: 0.05
  max_velocity_xy: 0.4
  max_velocity_z: 0.3
  position_tolerance: 0.2

obstacle_avoidance:
  laser_topic: "/scan"
  obstacle_distance_threshold: 1.0
  avoidance_fov_degrees: 360.0
  avoidance_strength_gain: 0.6
  max_avoidance_velocity: 0.3
  scan_timeout_ms: 200
  min_valid_ranges_ratio: 0.5
```

### 使用示例

```cpp
// 初始化
auto node = std::make_shared<rclcpp::Node>("controller_node");
VelocityAPFController controller(node.get());

// 设置航点序列
std::vector<VelocityAPFController::Waypoint> waypoints = {
    {0.0f, 0.0f, -2.0f, "起飞点"},
    {3.0f, 0.0f, -2.0f, "航点1"},
    {3.0f, 3.0f, -2.0f, "航点2"},
    {0.0f, 3.0f, -2.0f, "航点3"}
};
controller.setWaypointSequence(waypoints);

// 控制循环
while (!controller.isSequenceCompleted()) {
    // 准备输入数据
    VelocityAPFController::InputData input;
    input.current_x = getCurrentX();  // 获取当前位置
    input.current_y = getCurrentY();
    input.current_z = getCurrentZ();
    input.scan_data = getLatestScan(); // 获取激光雷达数据
    input.avoidance_enabled = true;
    
    // 计算控制指令
    auto output = controller.calculateVelocity(input);
    
    // 发送速度指令
    if (output.is_pid_active) {
        px4.setVelocitySetpoint(output.vx, output.vy, output.vz, 0.0f);
    }
}
```

---

## VisualServoController

VisualServoController是基于视觉反馈的精确定位控制器，使用YOLO目标检测进行精确的目标跟踪和对准。

### 概述

该控制器接收YOLO目标检测结果，使用PD控制算法计算精确的速度指令，使无人机能够精确对准并跟踪目标。它包含相机偏移补偿、检测丢失处理、搜索模式等高级功能。

### 核心特性

- **视觉伺服控制**：基于图像反馈的精确控制
- **相机偏移补偿**：自动补偿相机与飞机中心的偏移
- **检测丢失处理**：智能的目标丢失恢复机制
- **搜索模式**：目标丢失时的主动搜索
- **配置化参数**：支持YAML配置文件

### 数据结构

#### InputData
```cpp
struct InputData {
    // YOLO检测数据 (FRD坐标系)
    float detection_x;          // 目标X位置误差
    float detection_y;          // 目标Y位置误差
    bool detection_valid;       // 检测是否有效
    float detection_confidence; // 检测置信度
    
    // 当前无人机状态
    float current_x, current_y, current_z;  // 当前位置
    float current_yaw;                      // 当前偏航角
    
    // 时间戳
    std::chrono::steady_clock::time_point timestamp;
};
```

#### OutputVelocity
```cpp
struct OutputVelocity {
    float vx, vy, vz;          // 输出速度指令
    bool servo_active;         // 伺服是否激活
    bool target_reached;       // 是否达到目标精度
    bool detection_lost;       // 是否丢失检测
    float tracking_error;      // 当前跟踪误差
    float compensated_error_x; // 补偿后的X误差
    float compensated_error_y; // 补偿后的Y误差
};
```

#### Config
```cpp
struct Config {
    // PD控制器参数
    float kp_x, kp_y;          // 比例增益
    float kd_x, kd_y;          // 微分增益
    
    // 相机补偿偏移
    float camera_offset_x;      // 相机X轴偏移
    float camera_offset_y;      // 相机Y轴偏移
    float camera_offset_z;      // 相机Z轴偏移
    
    // 滤波参数
    float ema_alpha;           // 指数移动平均滤波系数
    
    // 速度和精度限制
    float max_velocity_ratio;  // 最大速度比例
    float tolerance;           // 到达容忍度
    
    // 检测验证参数
    float min_confidence;      // 最小置信度
    float max_target_distance; // 最大目标距离
    
    // 超时和丢失处理
    int detection_timeout_ms;   // 检测超时时间
    float search_velocity;      // 搜索时的移动速度
};
```

### 公共接口

#### 构造函数
```cpp
explicit VisualServoController(rclcpp::Node* node);
```
- **参数**：`node` - 指向ROS节点的指针
- **说明**：初始化控制器，加载配置文件

#### 核心控制接口
```cpp
OutputVelocity calculateVelocity(const InputData& input);
```
- **参数**：`input` - 视觉检测数据和当前状态
- **返回值**：计算得到的速度指令和状态信息
- **说明**：主要控制算法，处理视觉伺服和目标跟踪

#### 状态控制
```cpp
void reset();
```
- **说明**：重置所有内部状态，清除历史数据

#### 配置管理
```cpp
bool loadConfig();                          // 加载配置文件
const Config& getConfig() const;            // 获取当前配置
void setConfig(const Config& config);       // 设置新配置
```

### 控制算法

#### PD视觉伺服控制
- 比例项：减少当前位置误差
- 微分项：抑制超调和振荡
- 指数移动平均滤波：平滑检测噪声

#### 相机偏移补偿
- 自动补偿相机与无人机中心的物理偏移
- 支持三维偏移补偿（主要是X、Y轴）

#### 检测有效性验证
- 置信度阈值检查
- 目标距离合理性验证
- 数据时效性检查

#### 目标丢失处理
- 检测超时自动进入搜索模式
- 向最后有效位置缓慢移动
- 重新获得检测时自动恢复正常模式

### 配置文件

控制器支持YAML配置文件，位置：`config/visual_servo_config.yaml`

```yaml
visual_servo_controller:
  kp_x: 0.15
  kp_y: 0.15
  kd_x: 0.0
  kd_y: 0.0

camera_compensation:
  offset_x: 0.1075
  offset_y: 0.0
  offset_z: 0.0

control_parameters:
  ema_alpha: 0.3
  max_velocity_ratio: 0.3
  tolerance: 0.10

detection_parameters:
  min_confidence: 0.8
  max_target_distance: 0.5
  detection_timeout_ms: 1000
  search_velocity: 0.1
```

### 使用示例

```cpp
// 初始化
auto node = std::make_shared<rclcpp::Node>("visual_servo_node");
VisualServoController servo(node.get());

// 控制循环
while (rclcpp::ok()) {
    // 准备输入数据
    VisualServoController::InputData input;
    input.detection_x = getDetectionX();        // 从YOLO获取
    input.detection_y = getDetectionY();
    input.detection_valid = isDetectionValid();
    input.detection_confidence = getConfidence();
    input.current_x = getCurrentX();            // 当前位置
    input.current_y = getCurrentY();
    input.current_z = getCurrentZ();
    input.timestamp = std::chrono::steady_clock::now();
    
    // 计算控制指令
    auto output = servo.calculateVelocity(input);
    
    // 根据状态发送指令
    if (output.servo_active) {
        px4.setVelocitySetpoint(output.vx, output.vy, output.vz, 0.0f);
        
        if (output.target_reached) {
            RCLCPP_INFO(node->get_logger(), "目标对准完成！");
        }
        
        if (output.detection_lost) {
            RCLCPP_WARN(node->get_logger(), "目标丢失，搜索中...");
        }
    }
}
```

### 状态机说明

控制器内部维护以下状态：

1. **正常跟踪模式**：有效检测，正常PD控制
2. **搜索模式**：检测丢失超时，向最后位置移动
3. **目标到达**：误差小于容忍度，保持精确位置

### 性能特性

- **响应速度**：支持高频率控制（推荐20-50Hz）
- **精度**：可达到厘米级定位精度
- **鲁棒性**：自动处理检测丢失和噪声
- **可配置性**：所有关键参数都可通过配置文件调整

---

## 集成使用建议

### 典型使用流程

1. **初始化阶段**
   ```cpp
   PX4Communicator px4(node.get());
   VelocityAPFController waypoint_controller(node.get());
   VisualServoController visual_servo(node.get());
   ```

2. **粗略导航阶段**
   - 使用VelocityAPFController进行航点飞行
   - 启用避障功能确保安全

3. **精确定位阶段**
   - 在目标航点附近切换到VisualServoController
   - 使用视觉反馈进行精确对准

4. **安全监控**
   - 始终调用px4.update()进行安全检查
   - 监控通信状态，必要时执行紧急程序

### 注意事项

- 所有控制器都需要定期更新（建议50Hz）
- 配置文件应根据实际硬件参数调整
- 建议在仿真环境中充分测试后再进行实际飞行
- 务必确保急停和手动接管机制的可用性
