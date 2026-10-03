#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <px4_interface/msg/battery_status.hpp>
#include <px4_interface/msg/pose_ned.hpp>
#include <px4_interface/msg/vehicle_status.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "px4_interface/px4_gateway.hpp"
#include "px4_interface/px4_msgs_cache.hpp"

using namespace std::chrono_literals;

namespace {

rclcpp::NodeOptions MakeGatewayTestOptions() {
  rclcpp::NodeOptions options;
  options.append_parameter_override("auto_start_dds_agent", false);
  options.append_parameter_override(
      "vehicle_status_topic", "/test/fmu/out/vehicle_status_v1");
  options.append_parameter_override(
      "vehicle_odometry_topic", "/test/fmu/out/vehicle_odometry");
  options.append_parameter_override(
      "battery_status_topic", "/test/fmu/out/battery_status");
  return options;
}

class CacheSubscriberNode : public rclcpp::Node {
 public:
  CacheSubscriberNode() : rclcpp::Node("px4_gateway_test_listener") {
    auto qos = rclcpp::QoS(10);
    vehicle_status_sub_ =
        this->create_subscription<px4_interface::msg::VehicleStatus>(
            "/cache/vehicle_status", qos,
            [this](const px4_interface::msg::VehicleStatus &msg) {
              std::lock_guard<std::mutex> lock(mutex_);
              vehicle_status_msg_ = msg;
            });

    position_sub_ = this->create_subscription<px4_interface::msg::PoseNED>(
        "/cache/vehicle_odometry", qos,
        [this](const px4_interface::msg::PoseNED &msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          position_msg_ = msg;
        });

    battery_sub_ = this->create_subscription<px4_interface::msg::BatteryStatus>(
        "/cache/battery_status", qos,
        [this](const px4_interface::msg::BatteryStatus &msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          battery_msg_ = msg;
        });
  }

  bool publishers_discovered() const {
    return vehicle_status_sub_->get_publisher_count() > 0 &&
           position_sub_->get_publisher_count() > 0 &&
           battery_sub_->get_publisher_count() > 0;
  }

  bool all_received() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return vehicle_status_msg_.has_value() && position_msg_.has_value() &&
           battery_msg_.has_value();
  }

  std::optional<px4_interface::msg::VehicleStatus> last_vehicle_status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return vehicle_status_msg_;
  }

  std::optional<px4_interface::msg::PoseNED> last_position() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return position_msg_;
  }

  std::optional<px4_interface::msg::BatteryStatus> last_battery() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return battery_msg_;
  }

 private:
  rclcpp::Subscription<px4_interface::msg::VehicleStatus>::SharedPtr
      vehicle_status_sub_;
  rclcpp::Subscription<px4_interface::msg::PoseNED>::SharedPtr position_sub_;
  rclcpp::Subscription<px4_interface::msg::BatteryStatus>::SharedPtr
      battery_sub_;

  mutable std::mutex mutex_;
  std::optional<px4_interface::msg::VehicleStatus> vehicle_status_msg_;
  std::optional<px4_interface::msg::PoseNED> position_msg_;
  std::optional<px4_interface::msg::BatteryStatus> battery_msg_;
};

class TrajectorySetpointListener : public rclcpp::Node {
 public:
  TrajectorySetpointListener()
      : rclcpp::Node("px4_gateway_test_traj_listener") {
    auto qos = rclcpp::QoS(10);
    subscription_ =
        this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos,
            [this](const px4_msgs::msg::TrajectorySetpoint &msg) {
              std::lock_guard<std::mutex> lock(mutex_);
              last_msg_ = msg;
              ++message_count_;
            });
  }

  bool publisher_discovered() const {
    return subscription_->get_publisher_count() > 0;
  }

  std::optional<px4_msgs::msg::TrajectorySetpoint> last_message() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_msg_;
  }

  size_t message_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return message_count_;
  }

 private:
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
      subscription_;
  mutable std::mutex mutex_;
  std::optional<px4_msgs::msg::TrajectorySetpoint> last_msg_;
  size_t message_count_ = 0;
};

class VehicleCommandListener : public rclcpp::Node {
 public:
  VehicleCommandListener()
      : rclcpp::Node("px4_gateway_test_vehicle_cmd_listener") {
    auto qos = rclcpp::QoS(10);
    subscription_ = this->create_subscription<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", qos,
        [this](const px4_msgs::msg::VehicleCommand &msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          last_msg_ = msg;
          ++message_count_;
        });
  }

  bool publisher_discovered() const {
    return subscription_->get_publisher_count() > 0;
  }

  std::optional<px4_msgs::msg::VehicleCommand> last_message() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_msg_;
  }

  size_t message_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return message_count_;
  }

 private:
  rclcpp::Subscription<px4_msgs::msg::VehicleCommand>::SharedPtr subscription_;
  mutable std::mutex mutex_;
  std::optional<px4_msgs::msg::VehicleCommand> last_msg_;
  size_t message_count_ = 0;
};

class OffboardControlModeListener : public rclcpp::Node {
 public:
  OffboardControlModeListener()
      : rclcpp::Node("px4_gateway_test_offboard_listener") {
    auto qos = rclcpp::QoS(10);
    subscription_ =
        this->create_subscription<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", qos,
            [this](const px4_msgs::msg::OffboardControlMode &msg) {
              std::lock_guard<std::mutex> lock(mutex_);
              last_msg_ = msg;
              ++message_count_;
            });
  }

  bool publisher_discovered() const {
    return subscription_->get_publisher_count() > 0;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    last_msg_.reset();
    message_count_ = 0;
  }

  std::optional<px4_msgs::msg::OffboardControlMode> last_message() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_msg_;
  }

  size_t message_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return message_count_;
  }

 private:
  rclcpp::Subscription<px4_msgs::msg::OffboardControlMode>::SharedPtr
      subscription_;
  mutable std::mutex mutex_;
  std::optional<px4_msgs::msg::OffboardControlMode> last_msg_;
  size_t message_count_ = 0;
};

class Px4GatewayPublishCacheTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(Px4GatewayPublishCacheTest, PublishesCachedMessagesToAllTopics) {
  // 经验总结：Gateway 场景的发布测试需要遵循“三步走”——(1) 先准备缓存数据，
  // (2) 让 Gateway 节点加入 executor 并等待发现订阅者，(3) 再触发发布并在
  // executor 内循环 spin 等待消息到达。任何一步省略都会导致测试间歇性失败。
  auto cache = std::make_shared<Px4MsgsCache>();

  rclcpp::Clock clock;
  auto base_time = clock.now();

  px4Status::VehicleStatus vehicle_status;
  vehicle_status.valid = true;
  vehicle_status.msg_timestamp = base_time;
  vehicle_status.latest_timestamp = base_time;
  vehicle_status.arming_state = 2;
  vehicle_status.nav_state = 4;
  vehicle_status.failsafe = false;
  vehicle_status.pre_flight_checks_pass = true;
  cache->updateVehicleStatus(vehicle_status);

  px4Status::BatteryStatus battery_status;
  battery_status.valid = true;
  battery_status.timestamp = base_time + rclcpp::Duration(0, 5000);
  battery_status.msg_timestamp = battery_status.timestamp;
  battery_status.voltage_v = 12.6f;
  battery_status.current_a = 3.4f;
  battery_status.remaining = 0.82f;
  battery_status.warning = 2;
  cache->updateBatteryStatus(battery_status);

  px4Status::VehiclePose pose;
  pose.valid = true;
  pose.position = Eigen::Vector3d(1.0, -2.0, 3.5);
  pose.orientation = Eigen::Quaterniond(0.9238795, 0.0, 0.3826834, 0.0);
  pose.velocity = Eigen::Vector3d::Zero();
  pose.msg_timestamp = base_time + rclcpp::Duration(0, 10000);
  pose.latest_timestamp = pose.msg_timestamp;
  cache->updateVehiclePose(pose);

  auto gateway =
      std::make_shared<PX4Gateway>(MakeGatewayTestOptions(), cache);
  auto listener = std::make_shared<CacheSubscriberNode>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(gateway);
  exec.add_node(listener);

  // ROS 2 的内存内测试同样需要等待发现完成，否则 publishCache 调用
  // 可能在订阅者尚未连接时触发，导致消息“丢失”。使用细粒度轮询比直接 sleep
  // 更可靠
  auto discovery_deadline = std::chrono::steady_clock::now() + 200ms;
  while (!listener->publishers_discovered() &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  gateway->publishCache();

  auto receive_deadline = std::chrono::steady_clock::now() + 500ms;
  while (!listener->all_received() &&
         std::chrono::steady_clock::now() < receive_deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  exec.remove_node(listener);
  exec.remove_node(gateway);

  auto received_vehicle = listener->last_vehicle_status();
  ASSERT_TRUE(received_vehicle.has_value());
  EXPECT_TRUE(received_vehicle->valid);
  EXPECT_EQ(received_vehicle->arming_state, vehicle_status.arming_state);
  EXPECT_EQ(received_vehicle->nav_state, vehicle_status.nav_state);
  EXPECT_EQ(received_vehicle->failsafe, vehicle_status.failsafe);
  EXPECT_EQ(received_vehicle->pre_flight_checks_pass,
            vehicle_status.pre_flight_checks_pass);
  EXPECT_EQ(rclcpp::Time(received_vehicle->timestamp).nanoseconds(),
            vehicle_status.msg_timestamp.nanoseconds());

  auto received_position = listener->last_position();
  ASSERT_TRUE(received_position.has_value());
  EXPECT_TRUE(received_position->valid);
  EXPECT_DOUBLE_EQ(received_position->translation[0], pose.position.x());
  EXPECT_DOUBLE_EQ(received_position->translation[1], pose.position.y());
  EXPECT_DOUBLE_EQ(received_position->translation[2], pose.position.z());
  EXPECT_NEAR(received_position->orientation[0], pose.orientation.w(), 1e-6);
  EXPECT_NEAR(received_position->orientation[1], pose.orientation.x(), 1e-6);
  EXPECT_NEAR(received_position->orientation[2], pose.orientation.y(), 1e-6);
  EXPECT_NEAR(received_position->orientation[3], pose.orientation.z(), 1e-6);
  EXPECT_EQ(rclcpp::Time(received_position->timestamp).nanoseconds(),
            pose.msg_timestamp.nanoseconds());

  auto received_battery = listener->last_battery();
  ASSERT_TRUE(received_battery.has_value());
  EXPECT_TRUE(received_battery->valid);
  EXPECT_FLOAT_EQ(received_battery->voltage_v, battery_status.voltage_v);
  EXPECT_FLOAT_EQ(received_battery->current_a, battery_status.current_a);
  EXPECT_FLOAT_EQ(received_battery->remaining, battery_status.remaining);
  EXPECT_EQ(received_battery->warning, battery_status.warning);
  EXPECT_EQ(rclcpp::Time(received_battery->timestamp).nanoseconds(),
            battery_status.timestamp.nanoseconds());
}

TEST_F(Px4GatewayPublishCacheTest,
       SetTargetPublishesConvertedTrajectorySetpoint) {
  auto cache = std::make_shared<Px4MsgsCache>();
  auto gateway =
      std::make_shared<PX4Gateway>(MakeGatewayTestOptions(), cache);
  auto listener = std::make_shared<TrajectorySetpointListener>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(gateway);
  exec.add_node(listener);

  auto discovery_deadline = std::chrono::steady_clock::now() + 200ms;
  while (!listener->publisher_discovered() &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  ASSERT_TRUE(listener->publisher_discovered())
      << "Gateway trajectory publisher not discovered";

  px4GatewayTypes::setpoint target{};
  target.position[0] = 1.2f;
  target.position[1] = -0.8f;
  target.position[2] = 2.5f;
  target.velocity[0] = 0.3f;
  target.velocity[1] = -0.4f;
  target.velocity[2] = 0.0f;
  target.yaw = 0.75f;

  auto before_publish = gateway->get_clock()->now();
  gateway->setTarget(target);

  auto receive_deadline = std::chrono::steady_clock::now() + 500ms;
  while (listener->message_count() == 0 &&
         std::chrono::steady_clock::now() < receive_deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  exec.remove_node(listener);
  exec.remove_node(gateway);

  auto received = listener->last_message();
  ASSERT_TRUE(received.has_value());
  EXPECT_FLOAT_EQ(received->position[0], target.position[0]);
  EXPECT_FLOAT_EQ(received->position[1], target.position[1]);
  EXPECT_FLOAT_EQ(received->position[2], target.position[2]);
  EXPECT_FLOAT_EQ(received->velocity[0], target.velocity[0]);
  EXPECT_FLOAT_EQ(received->velocity[1], target.velocity[1]);
  EXPECT_FLOAT_EQ(received->velocity[2], target.velocity[2]);
  EXPECT_FLOAT_EQ(received->yaw, target.yaw);

  auto after_publish = gateway->get_clock()->now();
  auto before_us = static_cast<uint64_t>(before_publish.nanoseconds() / 1000);
  auto after_us = static_cast<uint64_t>(after_publish.nanoseconds() / 1000);
  EXPECT_GE(received->timestamp, before_us);
  EXPECT_LE(received->timestamp, after_us + 1000);  // 容忍执行延迟
}

TEST_F(Px4GatewayPublishCacheTest, PublishVehicleCommandFillsFieldsCorrectly) {
  auto cache = std::make_shared<Px4MsgsCache>();
  auto gateway =
      std::make_shared<PX4Gateway>(MakeGatewayTestOptions(), cache);
  auto listener = std::make_shared<VehicleCommandListener>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(gateway);
  exec.add_node(listener);

  auto discovery_deadline = std::chrono::steady_clock::now() + 200ms;
  while (!listener->publisher_discovered() &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  ASSERT_TRUE(listener->publisher_discovered())
      << "Gateway vehicle command publisher not discovered";

  constexpr float kParam1 = 12.34f;
  constexpr float kParam2 = -56.78f;
  auto before_publish = gateway->get_clock()->now();

  gateway->publishVehicleCommand(
      px4Enum::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, kParam1, kParam2);

  auto receive_deadline = std::chrono::steady_clock::now() + 500ms;
  while (listener->message_count() == 0 &&
         std::chrono::steady_clock::now() < receive_deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  exec.remove_node(listener);
  exec.remove_node(gateway);

  auto received = listener->last_message();
  ASSERT_TRUE(received.has_value());
  EXPECT_EQ(
      received->command,
      static_cast<uint32_t>(px4Enum::VehicleCommand::VEHICLE_CMD_DO_SET_MODE));
  EXPECT_FLOAT_EQ(received->param1, kParam1);
  EXPECT_FLOAT_EQ(received->param2, kParam2);
  EXPECT_EQ(received->target_system, 1);
  EXPECT_EQ(received->target_component, 1);
  EXPECT_EQ(received->source_system, 1);
  EXPECT_EQ(received->source_component, 1);
  EXPECT_TRUE(received->from_external);

  auto after_publish = gateway->get_clock()->now();
  auto before_us = static_cast<uint64_t>(before_publish.nanoseconds() / 1000);
  auto after_us = static_cast<uint64_t>(after_publish.nanoseconds() / 1000);
  EXPECT_GE(received->timestamp, before_us);
  EXPECT_LE(received->timestamp, after_us + 1000);
}

TEST_F(Px4GatewayPublishCacheTest, InitSubscribesAndUpdatesCacheFromPx4Topics) {
  auto cache = std::make_shared<Px4MsgsCache>();
  auto gateway =
      std::make_shared<PX4Gateway>(MakeGatewayTestOptions(), cache);
  auto px4_source =
      std::make_shared<rclcpp::Node>("px4_gateway_test_px4_source");

  auto vehicle_status_pub =
      px4_source->create_publisher<px4_msgs::msg::VehicleStatus>(
          "/test/fmu/out/vehicle_status_v1", 10);
  auto vehicle_odometry_pub =
      px4_source->create_publisher<px4_msgs::msg::VehicleOdometry>(
          "/test/fmu/out/vehicle_odometry", 10);
  auto battery_status_pub =
      px4_source->create_publisher<px4_msgs::msg::BatteryStatus>(
          "/test/fmu/out/battery_status", 10);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(gateway);
  exec.add_node(px4_source);

  auto discovery_deadline = std::chrono::steady_clock::now() + 300ms;
  while ((vehicle_status_pub->get_subscription_count() == 0 ||
          vehicle_odometry_pub->get_subscription_count() == 0 ||
          battery_status_pub->get_subscription_count() == 0) &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  ASSERT_GT(vehicle_status_pub->get_subscription_count(), 0u);
  ASSERT_GT(vehicle_odometry_pub->get_subscription_count(), 0u);
  ASSERT_GT(battery_status_pub->get_subscription_count(), 0u);

  px4_msgs::msg::VehicleStatus vehicle_status_msg;
  vehicle_status_msg.arming_state = 2;
  vehicle_status_msg.nav_state = 14;
  vehicle_status_msg.failsafe = true;
  vehicle_status_msg.pre_flight_checks_pass = false;

  px4_msgs::msg::VehicleOdometry vehicle_odometry_msg;
  vehicle_odometry_msg.timestamp =
      static_cast<uint64_t>(rclcpp::Clock().now().nanoseconds() / 1000);
  vehicle_odometry_msg.position[0] = 4.5f;
  vehicle_odometry_msg.position[1] = -1.25f;
  vehicle_odometry_msg.position[2] = 0.8f;
  vehicle_odometry_msg.q[0] = 1.0f;
  vehicle_odometry_msg.q[1] = 0.0f;
  vehicle_odometry_msg.q[2] = 0.0f;
  vehicle_odometry_msg.q[3] = 0.0f;
  vehicle_odometry_msg.velocity[0] = 0.0f;
  vehicle_odometry_msg.velocity[1] = 0.0f;
  vehicle_odometry_msg.velocity[2] = 0.0f;

  px4_msgs::msg::BatteryStatus battery_msg;
  battery_msg.voltage_v = 15.2f;
  battery_msg.current_a = 6.7f;
  battery_msg.remaining = 0.42f;
  battery_msg.warning = 3;

  vehicle_status_pub->publish(vehicle_status_msg);
  vehicle_odometry_pub->publish(vehicle_odometry_msg);
  battery_status_pub->publish(battery_msg);

  auto receive_deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < receive_deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  exec.remove_node(px4_source);
  exec.remove_node(gateway);

  auto cached_status = cache->getVehicleStatus();
  EXPECT_TRUE(cached_status.valid);
  EXPECT_EQ(cached_status.arming_state, vehicle_status_msg.arming_state);
  EXPECT_EQ(cached_status.nav_state, vehicle_status_msg.nav_state);
  EXPECT_EQ(cached_status.failsafe, vehicle_status_msg.failsafe);
  EXPECT_EQ(cached_status.pre_flight_checks_pass,
            vehicle_status_msg.pre_flight_checks_pass);
  EXPECT_GT(cached_status.latest_timestamp.nanoseconds(), 0);

  auto cached_position = cache->getVehiclePose();
  EXPECT_TRUE(cached_position.valid);
  EXPECT_DOUBLE_EQ(cached_position.position.x(),
                   vehicle_odometry_msg.position[0]);
  EXPECT_DOUBLE_EQ(cached_position.position.y(),
                   vehicle_odometry_msg.position[1]);
  EXPECT_DOUBLE_EQ(cached_position.position.z(),
                   vehicle_odometry_msg.position[2]);
  EXPECT_DOUBLE_EQ(cached_position.orientation.w(), 1.0);
  EXPECT_DOUBLE_EQ(cached_position.orientation.x(), 0.0);
  EXPECT_DOUBLE_EQ(cached_position.orientation.y(), 0.0);
  EXPECT_DOUBLE_EQ(cached_position.orientation.z(), 0.0);
  EXPECT_GT(cached_position.msg_timestamp.nanoseconds(), 0);

  auto cached_battery = cache->getBatteryStatus();
  EXPECT_TRUE(cached_battery.valid);
  EXPECT_FLOAT_EQ(cached_battery.voltage_v, battery_msg.voltage_v);
  EXPECT_FLOAT_EQ(cached_battery.current_a, battery_msg.current_a);
  EXPECT_FLOAT_EQ(cached_battery.remaining, battery_msg.remaining);
  EXPECT_EQ(cached_battery.warning, battery_msg.warning);
  EXPECT_GT(cached_battery.timestamp.nanoseconds(), 0);
}

TEST_F(Px4GatewayPublishCacheTest, OffboardControlModeReflectsChosenFlags) {
  auto cache = std::make_shared<Px4MsgsCache>();
  auto gateway =
      std::make_shared<PX4Gateway>(MakeGatewayTestOptions(), cache);
  auto listener = std::make_shared<OffboardControlModeListener>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(gateway);
  exec.add_node(listener);

  auto discovery_deadline = std::chrono::steady_clock::now() + 200ms;
  while (!listener->publisher_discovered() &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  ASSERT_TRUE(listener->publisher_discovered())
      << "Gateway offboard control publisher not discovered";

  // 等待至少一个心跳，确保定时器正在发布
  auto initial_deadline = std::chrono::steady_clock::now() + 200ms;
  while (listener->message_count() == 0 &&
         std::chrono::steady_clock::now() < initial_deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  listener->clear();

  auto before_toggle = gateway->get_clock()->now();
  gateway->chooseControlMethod(false, true);

  auto receive_deadline = std::chrono::steady_clock::now() + 400ms;
  while (listener->message_count() == 0 &&
         std::chrono::steady_clock::now() < receive_deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  exec.remove_node(listener);
  exec.remove_node(gateway);

  auto received = listener->last_message();
  ASSERT_TRUE(received.has_value());
  EXPECT_FALSE(received->position);
  EXPECT_TRUE(received->velocity);
  EXPECT_FALSE(received->acceleration);
  EXPECT_FALSE(received->attitude);
  EXPECT_FALSE(received->body_rate);

  auto after_toggle = gateway->get_clock()->now();
  auto before_us = static_cast<uint64_t>(before_toggle.nanoseconds() / 1000);
  auto after_us = static_cast<uint64_t>(after_toggle.nanoseconds() / 1000);
  EXPECT_GE(received->timestamp, before_us);
  EXPECT_LE(received->timestamp, after_us + 2000);
}

}  // namespace
