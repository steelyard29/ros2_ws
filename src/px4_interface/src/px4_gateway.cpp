#include "px4_interface/px4_gateway.hpp"

#include <cstdint>
#include <exception>
#include <memory>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "px4_interface/msg_converters.hpp"
#include "px4_interface/process_manager.hpp"
#include "px4_interface/px4_msgs_cache.hpp"

namespace {
/// PX4 消息时间戳单位为微秒，需要乘以该常量转换为纳秒后才能构造 ROS Time。
constexpr uint64_t kMicrosToNanos = 1000ULL;
}  // namespace

PX4Gateway::PX4Gateway(const rclcpp::NodeOptions& options,
                       std::shared_ptr<Px4MsgsCache> cache)
    : Node("PX4_Gateway", options), px4_msgs_cache_(std::move(cache)) {
  if (px4_msgs_cache_ == nullptr) {
    throw std::invalid_argument("Px4MsgsCache pointer cannot be null");
  }

  if (!rclcpp::ok()) {
    throw std::runtime_error("ROS2 is not initialized");
  }

  init();
  RCLCPP_INFO(this->get_logger(), "PX4Gateway node initialized");
}

bool PX4Gateway::checkPx4Publishers() const {
  const auto topic = this->get_parameter("vehicle_status_topic").as_string();
  auto publishers = this->get_publishers_info_by_topic(topic);
  if (publishers.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "No publishers found for %s. Check PX4 connection.",
                topic.c_str());
    return false;
  }
  return true;
}

template <typename Data, typename PublisherPtr>
void publish(const Data& data, PublisherPtr publisher) {
  auto msg = MsgConverters::convert(data);
  publisher->publish(msg);
}

void PX4Gateway::publishCache() const {
  publish(px4_msgs_cache_->getVehicleStatus(), vehicle_status_publisher_);
  publish(px4_msgs_cache_->getVehiclePose(), vehicle_odometry_publisher_);
  publish(px4_msgs_cache_->getBatteryStatus(), battery_status_publisher_);
}

void PX4Gateway::chooseControlMethod(bool position_control,
                                     bool velocity_control) {
  // Tests:
  // Px4GatewayPublishCacheTest.OffboardControlModeReflectsChosenFlags
  // （验证标志位变更可通过 OffboardControlMode 心跳观察）。
  offboard_position_control_ = position_control;
  offboard_velocity_control_ = velocity_control;
}

// Tests:
// Px4GatewayPublishCacheTest.SetTargetPublishesConvertedTrajectorySetpoint
void PX4Gateway::setTarget(const px4GatewayTypes::setpoint& target) {
  px4_msgs::msg::TrajectorySetpoint
      traj_setpoint_msg;  // Todo: 优化，这里的转化过于机械
  traj_setpoint_msg.position[0] = target.position[0];
  traj_setpoint_msg.position[1] = target.position[1];
  traj_setpoint_msg.position[2] = target.position[2];
  traj_setpoint_msg.velocity[0] = target.velocity[0];
  traj_setpoint_msg.velocity[1] = target.velocity[1];
  traj_setpoint_msg.velocity[2] = target.velocity[2];
  traj_setpoint_msg.yaw = target.yaw;
  traj_setpoint_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(traj_setpoint_msg);
}

// TODO(TEST-LOW): setTarget(msg)
// 仅简单转发，后续可使用轻量发布捕获测试验证原样透传。
void PX4Gateway::setTarget(const px4_msgs::msg::TrajectorySetpoint& target) {
  trajectory_setpoint_publisher_->publish(target);
}

// Tests:
// Px4GatewayPublishCacheTest.PublishVehicleCommandFillsFieldsCorrectly
void PX4Gateway::publishVehicleCommand(const px4Enum::VehicleCommand command,
                                       const float param1, const float param2) {
  px4_msgs::msg::VehicleCommand vehicle_command_msg;
  vehicle_command_msg.command = static_cast<uint32_t>(command);
  vehicle_command_msg.param1 = param1;
  vehicle_command_msg.param2 = param2;
  vehicle_command_msg.target_system = 1;     // 通常为1
  vehicle_command_msg.target_component = 1;  // 通常为1
  vehicle_command_msg.source_system = 1;     // 通常为1
  vehicle_command_msg.source_component = 1;  // 通常为1
  vehicle_command_msg.from_external = true;  // 来自外部控制器
  vehicle_command_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher_->publish(vehicle_command_msg);
}

void PX4Gateway::setArmMode() {
  // TODO(TEST-MED): 补充单元测试覆盖 setArmMode/DisarmMode 等封装命令，验证调用
  // publishVehicleCommand 时参数取值正确。
  publishVehicleCommand(
      px4Enum::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);
  RCLCPP_INFO(this->get_logger(), "Sent ARM command to PX4");
}

void PX4Gateway::setDisarmMode() {
  // TODO(TEST-MED): 与 setArmMode 相同，验证解锁/锁定命令的参数分支。
  publishVehicleCommand(
      px4Enum::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f, 0.0f);
  RCLCPP_INFO(this->get_logger(), "Sent DISARM command to PX4");
}

void PX4Gateway::setOffboardMode() {
  // TODO(TEST-MED): 补充单元测试覆盖
  // setOffboardMode/setLandMode/triggerEmergencyStop 等封装命令，验证调用
  // publishVehicleCommand 时参数取值正确。
  publishVehicleCommand(px4Enum::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f,
                        6.0f);
  RCLCPP_INFO(this->get_logger(), "Sent OFFBOARD mode command to PX4");
}

void PX4Gateway::setLandMode() {
  publishVehicleCommand(px4Enum::VehicleCommand::VEHICLE_CMD_NAV_LAND);
  RCLCPP_INFO(this->get_logger(), "Sent LAND command to PX4");
}

void PX4Gateway::triggerEmergencyStop() {
  publishVehicleCommand(
      px4Enum::VehicleCommand::VEHICLE_CMD_DO_FLIGHTTERMINATION, 1.0f);
  RCLCPP_WARN(this->get_logger(), "Sent EMERGENCY STOP command to PX4!");
  RCLCPP_WARN(this->get_logger(), "Motors will stop immediately!");
}

PX4Gateway::Px4Timestamps PX4Gateway::processPx4Timestamp(
    uint64_t px4_timestamp_us, const char* topic_name,
    rclcpp::Duration tolerance) const {
  // 统一转换 PX4 原始时间戳，并记录本地接收时刻，便于缓存层判断数据是否过期。
  Px4Timestamps timestamps;
  timestamps.received_timestamp = this->now();
  const auto clock_type = this->get_clock()->get_clock_type();
  timestamps.msg_timestamp =
      rclcpp::Time(px4_timestamp_us * kMicrosToNanos, clock_type);

  const auto lower_bound = timestamps.received_timestamp - tolerance;
  const auto upper_bound = timestamps.received_timestamp + tolerance;

  if (timestamps.msg_timestamp < lower_bound) {
    RCLCPP_WARN(
        this->get_logger(), "Received old %s message (%.3f s late).",
        topic_name,
        (timestamps.received_timestamp - timestamps.msg_timestamp).seconds());
  } else if (timestamps.msg_timestamp > upper_bound) {
    RCLCPP_WARN(
        this->get_logger(),
        "Received %s message from the future (%.3f s ahead).", topic_name,
        (timestamps.msg_timestamp - timestamps.received_timestamp).seconds());
  }

  return timestamps;
}

void PX4Gateway::init() {
  // Tests:
  // Px4GatewayPublishCacheTest.InitSubscribesAndUpdatesCacheFromPx4Topics
  // （验证订阅回调将 PX4 消息写入 Px4MsgsCache）。
  // 提供给PX4的发布者
  offboard_control_mode_publisher_ =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>(
          "/fmu/in/offboard_control_mode", 10);
  trajectory_setpoint_publisher_ =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
          "/fmu/in/trajectory_setpoint", 10);
  vehicle_command_publisher_ =
      this->create_publisher<px4_msgs::msg::VehicleCommand>(
          "/fmu/in/vehicle_command", 10);  // 发布车辆控制命令

  // 提供给外部的发布者
  vehicle_status_publisher_ =
      this->create_publisher<px4_interface::msg::VehicleStatus>(
          "/cache/vehicle_status", 10);
  vehicle_odometry_publisher_ =
      this->create_publisher<px4_interface::msg::PoseNED>(
          "/cache/vehicle_odometry", 10);
  battery_status_publisher_ =
      this->create_publisher<px4_interface::msg::BatteryStatus>(
          "/cache/battery_status", 10);

  // PX4 v1.16 exposes versioned status topic names. Keep these configurable
  // so tests and multi-vehicle deployments can isolate their DDS endpoints.
  this->declare_parameter<std::string>(
      "vehicle_status_topic", "/fmu/out/vehicle_status_v1");
  this->declare_parameter<std::string>(
      "vehicle_odometry_topic", "/fmu/out/vehicle_odometry");
  this->declare_parameter<std::string>(
      "battery_status_topic", "/fmu/out/battery_status");
  const auto vehicle_status_topic =
      this->get_parameter("vehicle_status_topic").as_string();
  const auto vehicle_odometry_topic =
      this->get_parameter("vehicle_odometry_topic").as_string();
  const auto battery_status_topic =
      this->get_parameter("battery_status_topic").as_string();

  // 创建订阅者
  auto qos_profile = rclcpp::SystemDefaultsQoS();
  vehicle_status_subscriber_ =
      this->create_subscription<px4_msgs::msg::VehicleStatus>(
          vehicle_status_topic, qos_profile,
          [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
            px4Status::VehicleStatus status;
            const auto timestamps =
                processPx4Timestamp(msg->timestamp, "vehicle_status");
            status.valid = true;
            status.msg_timestamp = timestamps.msg_timestamp;
            status.latest_timestamp = timestamps.received_timestamp;
            status.arming_state = msg->arming_state;
            status.nav_state = msg->nav_state;
            status.failsafe = msg->failsafe;
            status.pre_flight_checks_pass = msg->pre_flight_checks_pass;
            px4_msgs_cache_->updateVehicleStatus(status);
          });

  vehicle_odometry_subscriber_ =
      this->create_subscription<px4_msgs::msg::VehicleOdometry>(
          vehicle_odometry_topic, qos_profile,
          [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
            px4Status::VehiclePose pose;
            const auto timestamps =
                processPx4Timestamp(msg->timestamp, "vehicle_odometry");
            pose.position = Eigen::Vector3d(msg->position[0], msg->position[1],
                                            msg->position[2]);
            pose.orientation =
                Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
            pose.velocity = Eigen::Vector3d(msg->velocity[0], msg->velocity[1],
                                            msg->velocity[2]);
            pose.valid = true;
            pose.msg_timestamp = timestamps.msg_timestamp;
            pose.latest_timestamp = timestamps.received_timestamp;

            px4_msgs_cache_->updateVehiclePose(pose);
          });

  battery_status_subscriber_ =
      this->create_subscription<px4_msgs::msg::BatteryStatus>(
          battery_status_topic, qos_profile,
          [this](const px4_msgs::msg::BatteryStatus::SharedPtr msg) {
            px4Status::BatteryStatus battery;
            battery.valid = true;
            const auto timestamps =
                processPx4Timestamp(msg->timestamp, "battery_status");
            battery.msg_timestamp = timestamps.msg_timestamp;
            battery.timestamp = timestamps.received_timestamp;
            battery.voltage_v = msg->voltage_v;
            battery.current_a = msg->current_a;
            battery.remaining = msg->remaining;
            battery.warning = msg->warning;
            px4_msgs_cache_->updateBatteryStatus(battery);
          });

  // 创建定时器，定期发布OffboardControlMode(50Hz)
  offboard_control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / offboard_control_rate_hz_), [this]() {
        publishOffboardControlMode(offboard_position_control_,
                                   offboard_velocity_control_);
      });
  // 创建定时器，定期发布缓存数据(10Hz)
  cache_publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                                 [this]() { publishCache(); });
}

void PX4Gateway::publishOffboardControlMode(const bool position_control,
                                            const bool velocity_control) {
  // Tests:
  //  Px4GatewayPublishCacheTest.OffboardControlModeReflectsChosenFlags
  px4_msgs::msg::OffboardControlMode offboard_msg;
  offboard_msg.position = position_control;  // 位置控制
  offboard_msg.velocity = velocity_control;  // 速度控制
  offboard_msg.acceleration = false;
  offboard_msg.attitude = false;
  offboard_msg.body_rate = false;
  offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(offboard_msg);
}
