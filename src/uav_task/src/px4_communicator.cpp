#include "px4_communicator.hpp"
#include <limits>

using namespace std::chrono_literals;

PX4Communicator::PX4Communicator(rclcpp::Node* node) : node_(node) {
    // 创建发布者
    offboard_control_mode_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    // PX4 uXRCE-DDS output topics use the sensor-data/BEST_EFFORT QoS
    // profile.  SystemDefaultsQoS is commonly RELIABLE and is incompatible
    // with those publishers, which would leave the mission node permanently
    // without position or control-state feedback.
    auto qos_profile = rclcpp::SensorDataQoS();
    
    vehicle_local_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos_profile,
        std::bind(&PX4Communicator::vehicleLocalPositionCallback, this, std::placeholders::_1));
        
    vehicle_control_mode_sub_ = node_->create_subscription<px4_msgs::msg::VehicleControlMode>(
        "/fmu/out/vehicle_control_mode", qos_profile,
        std::bind(&PX4Communicator::vehicleControlModeCallback, this, std::placeholders::_1));

    // 初始化时间戳，防止一开始就超时
    last_setpoint_time_ = node_->get_clock()->now();
    last_offboard_request_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    startup_time_ = node_->get_clock()->now();
    
    RCLCPP_INFO(node_->get_logger(), "PX4Communicator已初始化");
}


void PX4Communicator::update() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // 状态机只应从 NORMAL -> EMERGENCY，不应反向
    if (comm_state_ != CommState::NORMAL) {
        // 如果已在紧急状态，处理紧急逻辑
        if (comm_state_ == CommState::EMERGENCY_HOLD || 
            comm_state_ == CommState::PX4_DATA_TIMEOUT_HOLD || 
            comm_state_ == CommState::MAIN_TIMEOUT_HOLD) { // 统一处理所有HOLD状态
            auto elapsed = node_->get_clock()->now() - emergency_start_time_;
            if (elapsed > EMERGENCY_HOLD_TIMEOUT_) {
                RCLCPP_ERROR(node_->get_logger(), "紧急保持超时，执行紧急降落!");
                comm_state_ = CommState::EMERGENCY_LAND;
                publishEmergencyLand(); // 立即发布降落命令
            } else {
                publishEmergencyHold();
            }
        }
        // 如果是 EMERGENCY_LAND，则不需要做任何事，飞控会处理降落
        return;
    }

    // --- 在 NORMAL 状态下，检查是否需要进入紧急状态 ---
    auto now = node_->get_clock()->now();

    // 给启动阶段一个宽限期，避免启动时误判超时
    if ((now - startup_time_) < rclcpp::Duration::from_seconds(5)) {
        return; // 启动后5秒内不进行超时检查
    }

    // 检查PX4数据超时
    // 只有当数据曾经有效(valid == true)，我们才检查它是否超时
    bool px4_data_timed_out = false;
    if (control_status_.valid && (now - control_status_.timestamp) > PX4_DATA_TIMEOUT_) {
        RCLCPP_ERROR(node_->get_logger(), "飞控状态数据(VehicleControlMode)超时!");
        px4_data_timed_out = true;
    }
    if (current_position_.valid && (now - current_position_.timestamp) > PX4_DATA_TIMEOUT_) {
        RCLCPP_ERROR(node_->get_logger(), "飞控位置数据(VehicleLocalPosition)超时!");
        px4_data_timed_out = true;
    }

    if (px4_data_timed_out) {
        RCLCPP_ERROR(node_->get_logger(), "PX4数据超时，进入紧急保持模式!");
        comm_state_ = CommState::PX4_DATA_TIMEOUT_HOLD;
        emergency_start_time_ = now;
        return; // 进入紧急状态后立即返回
    }
    
    // 仅在发送了一定数量的指令后才开始检查，避免启动时误判
    if (offboard_setpoint_counter_ > 10 && (now - last_setpoint_time_) > MAIN_PROGRAM_TIMEOUT_) {
        RCLCPP_WARN(node_->get_logger(), "主程序调用超时，进入紧急保持模式!");
        comm_state_ = CommState::MAIN_TIMEOUT_HOLD;
        emergency_start_time_ = now;
        // 注意：这里不需要 return，下一轮 update 循环会进入上面的紧急处理逻辑
    }
}

bool PX4Communicator::setPositionSetpoint(float x, float y, float z, float yaw) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (comm_state_ != CommState::NORMAL) {
        // 不使用 THROTTLE，因为在紧急模式下，上层逻辑应该停止发送
        RCLCPP_WARN(node_->get_logger(), "处于紧急状态，拒绝位置设定点命令");
        return false;
    }
    
    // 更新主程序活跃时间戳
    last_setpoint_time_ = node_->get_clock()->now();
    
    publishOffboardControlMode(true, false);  // 只控制位置
    
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    msg.position = {x, y, z};
    // 速度设为NaN，表示不控制速度
    msg.velocity = {std::numeric_limits<float>::quiet_NaN(), 
                    std::numeric_limits<float>::quiet_NaN(), 
                    std::numeric_limits<float>::quiet_NaN()};
    msg.yaw = yaw;
    trajectory_setpoint_pub_->publish(msg);
    
    offboard_setpoint_counter_++;
    return true;
}

bool PX4Communicator::setVelocitySetpoint(float vx, float vy, float vz, float yaw) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (comm_state_ != CommState::NORMAL) {
        RCLCPP_WARN(node_->get_logger(), "处于紧急状态，拒绝速度设定点命令");
        return false;
    }
    
    // 更新主程序活跃时间戳
    last_setpoint_time_ = node_->get_clock()->now();
    
    publishOffboardControlMode(false, true);  // 只控制速度
    
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    // 位置设为NaN，表示不控制位置
    msg.position = {std::numeric_limits<float>::quiet_NaN(), 
                    std::numeric_limits<float>::quiet_NaN(), 
                    std::numeric_limits<float>::quiet_NaN()};
    msg.velocity = {vx, vy, vz};
    msg.yaw = yaw;
    trajectory_setpoint_pub_->publish(msg);
    
    offboard_setpoint_counter_++;
    return true;
}

bool PX4Communicator::setPositionAndVelocitySetpoint(float x, float y, float z, float vx, float vy, float vz, float yaw) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (comm_state_ != CommState::NORMAL) {
        RCLCPP_WARN(node_->get_logger(), "处于紧急状态，拒绝位置和速度设定点命令");
        return false;
    }
    
    // 更新主程序活跃时间戳
    last_setpoint_time_ = node_->get_clock()->now();
    
    publishOffboardControlMode(true, true);  // 同时控制位置和速度
    
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    msg.position = {x, y, z};
    msg.velocity = {vx, vy, vz};
    msg.yaw = yaw;
    trajectory_setpoint_pub_->publish(msg);
    
    offboard_setpoint_counter_++;
    return true;
}

PX4Communicator::CommandResult PX4Communicator::arm() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (comm_state_ != CommState::NORMAL) {
        return {false, "处于紧急状态，无法解锁"};
    }
    if (control_status_.is_armed) {
        return {true, "已经解锁"};
    }
    
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    RCLCPP_INFO(node_->get_logger(), "发送解锁命令");
    return {true, ""};
}

PX4Communicator::CommandResult PX4Communicator::setOffboardMode() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (comm_state_ != CommState::NORMAL) {
        return {false, "处于紧急状态，无法切换到Offboard模式"};
    }
    if (!control_status_.is_armed) {
        return {false, "必须先解锁才能切换到Offboard模式"};
    }
    if (control_status_.is_offboard) {
        return {true, "已经在Offboard模式"};
    }

    const auto now = node_->get_clock()->now();
    if (last_offboard_request_time_.nanoseconds() != 0 &&
        (now - last_offboard_request_time_) < OFFBOARD_REQUEST_INTERVAL_) {
        return {true, "等待PX4处理上一次Offboard请求"};
    }

    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    last_offboard_request_time_ = now;
    RCLCPP_INFO(node_->get_logger(), "发送切换到Offboard模式命令");
    return {true, ""};
}

void PX4Communicator::disarm() {
    RCLCPP_INFO(node_->get_logger(), "发送上锁命令");
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

void PX4Communicator::land() {
    RCLCPP_INFO(node_->get_logger(), "发送降落命令");
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
}

PX4Communicator::Position PX4Communicator::getCurrentPosition() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_position_;
}

PX4Communicator::ControlStatus PX4Communicator::getControlStatus() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return control_status_;
}

PX4Communicator::CommState PX4Communicator::getCommunicationState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return comm_state_;
}

void PX4Communicator::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_position_.x = msg->x;
    current_position_.y = msg->y;
    current_position_.z = msg->z;
    current_position_.yaw = msg->heading;
    // 修复：使用ROS时间源，避免时间源不一致的错误
    current_position_.timestamp = node_->get_clock()->now();
    current_position_.valid = true;
}

void PX4Communicator::vehicleControlModeCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    control_status_.is_armed = msg->flag_armed;
    control_status_.is_offboard = msg->flag_control_offboard_enabled;
    // 修复：使用ROS时间源，避免时间源不一致的错误
    control_status_.timestamp = node_->get_clock()->now();
    control_status_.valid = true;
}

void PX4Communicator::publishOffboardControlMode(bool position_control, bool velocity_control) {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    msg.position = position_control;
    msg.velocity = velocity_control;
    offboard_control_mode_pub_->publish(msg);
}

void PX4Communicator::publishVehicleCommand(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
}

void PX4Communicator::publishEmergencyHold() {
    if (current_position_.valid) {
        publishOffboardControlMode(true, true);  // 紧急保持时同时控制位置和速度
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
        msg.position = {(float)current_position_.x, (float)current_position_.y, (float)current_position_.z};
        msg.velocity = {0.0f, 0.0f, 0.0f};
        msg.yaw = current_position_.yaw;
        trajectory_setpoint_pub_->publish(msg);
    }
}

void PX4Communicator::publishEmergencyLand() {
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
}
