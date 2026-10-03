#include "px4_communicator.hpp"
#include "velocity_apf_controller.hpp"
#include "visual_servo_controller.hpp"
#include <chrono>
#include <cstdlib>
#include <execinfo.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

class WaypointFlightTest : public rclcpp::Node {
public:
  // ... (配置参数部分保持不变)
  // 舵机相关参数
  static constexpr float SERVO_INIT_ANGLE = 180.0f;
  static constexpr float SERVO_STEP = 40.0f;
  static constexpr float SERVO_MIN_ANGLE = 0.0f;
  static constexpr int64_t INITIALIZATION_TIME_MS = 3000;
  static constexpr int64_t ARM_TIMEOUT_MS = 5000;
  static constexpr int64_t ARM_RETRY_INTERVAL_MS = 2000;
  static constexpr int64_t LOG_THROTTLE_MS = 2000;
  static constexpr int64_t STATUS_LOG_INTERVAL_MS = 5000;
  static constexpr int64_t SHUTDOWN_DELAY_MS = 5000;
  static constexpr int CONTROL_FREQUENCY_HZ = 50;
  static constexpr float TAKEOFF_TARGET_HEIGHT = -1.0f;
  static constexpr float TAKEOFF_HEIGHT_TOLERANCE = 0.2f;
  static constexpr float LANDING_HEIGHT_THRESHOLD = -0.1f;
  static constexpr int64_t VISUAL_SERVO_TIMEOUT_MS = 60000;
  static constexpr int64_t VISUAL_SERVO_HOVER_DURATION_MS = 5000;
  static constexpr const char *LASER_SCAN_TOPIC = "/scan";
  static constexpr const char *YOLO_NAME_TOPIC = "/yolo/scene/name";
  static constexpr const char *YOLO_CONFIDENCE_TOPIC = "/yolo/scene/confidence";
  static constexpr const char *YOLO_CENTRE_TOPIC = "/yolo/scene/centre";
  static constexpr const char *SEQUENCE_COMPLETE_TOPIC =
      "/waypoint_sequence_complete";

  struct WaypointConfig {
    float x, y, z;
    std::string description;
  };

  // 1. 新航点序列
  const std::vector<WaypointConfig> TEST_WAYPOINTS = {
      // 起飞点 (索引 0)
      {0.0f, 0.0f, TAKEOFF_TARGET_HEIGHT, "起飞到目标高度"},
      // 新路径的航点（从索引1开始排列）
      {0.8f, 0.0f, TAKEOFF_TARGET_HEIGHT, "航点1: (1, 0.0)"},
  };

  // 2. 航点与目标名称的映射
  const std::map<size_t, std::string> VISUAL_SERVO_TRIGGER_POINTS = {
      {1, "bridge"}, // 航点1 (索引2) 识别 bridge
  };

  WaypointFlightTest()
      : Node("waypoint_flight_test"), px4_comm_(this),
        waypoint_controller_(this), visual_servo_controller_(this),
        current_servo_angle_(SERVO_INIT_ANGLE) {
    RCLCPP_INFO(this->get_logger(),
                "=== 航点飞行测试程序启动 (YOLO通用目标检测) ===");
    current_state_ = TestState::INITIALIZING;
    test_start_time_ = this->get_clock()->now();
    last_arm_attempt_time_ = this->get_clock()->now();
    state_start_time_ = this->get_clock()->now();

    setupWaypoints();

    auto control_period =
        std::chrono::milliseconds(1000 / CONTROL_FREQUENCY_HZ);
    control_timer_ = this->create_wall_timer(
        control_period, std::bind(&WaypointFlightTest::controlLoop, this));

    sequence_complete_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        SEQUENCE_COMPLETE_TOPIC, 10,
        std::bind(&WaypointFlightTest::sequenceCompleteCallback, this,
                  std::placeholders::_1));

    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        LASER_SCAN_TOPIC, 10,
        std::bind(&WaypointFlightTest::laserScanCallback, this,
                  std::placeholders::_1));

    yolo_name_sub_ = this->create_subscription<std_msgs::msg::String>(
        YOLO_NAME_TOPIC, 10,
        std::bind(&WaypointFlightTest::yoloNameCallback, this,
                  std::placeholders::_1));

    yolo_confidence_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        YOLO_CONFIDENCE_TOPIC, 10,
        std::bind(&WaypointFlightTest::yoloConfidenceCallback, this,
                  std::placeholders::_1));

    yolo_centre_sub_ =
        this->create_subscription<geometry_msgs::msg::PointStamped>(
            YOLO_CENTRE_TOPIC, 10,
            std::bind(&WaypointFlightTest::yoloCentreCallback, this,
                      std::placeholders::_1));

    RCLCPP_INFO(
        this->get_logger(),
        "已启用避障测试和通用视觉伺服功能。订阅YOLO话题: '%s', '%s', '%s'",
        YOLO_NAME_TOPIC, YOLO_CONFIDENCE_TOPIC, YOLO_CENTRE_TOPIC);

    // 舵机角度发布器初始化
    servo_angle_pub_ =
        this->create_publisher<std_msgs::msg::Float32>("/servo/set_angle", 10);
    // 启动时发布初始角度
    publish_servo_angle(current_servo_angle_);
    RCLCPP_INFO(this->get_logger(), "舵机初始角度已设置为 %.1f 度",
                current_servo_angle_);
  }

private:
  // ... (枚举 TestState 和函数 setupWaypoints, controlLoop, 以及所有 handle*
  // 函数保持不变)
  enum class TestState {
    INITIALIZING,
    ARMING,
    TAKING_OFF,
    WAYPOINT_FLIGHT,
    VISUAL_SERVO,
    VISUAL_SERVO_ALIGNED_HOVER,
    WAITING_FOR_SERVO, // <--- 新增的状态
    LANDING,
    COMPLETED,
    EMERGENCY
  };

  void setupWaypoints() {
    std::vector<VelocityAPFController::Waypoint> waypoints;
    for (const auto &config : TEST_WAYPOINTS) {
      waypoints.push_back({config.x, config.y, config.z, config.description});
    }
    waypoint_controller_.setWaypointSequence(waypoints);
    RCLCPP_INFO(this->get_logger(),
                "设置航点序列完成，共 %zu 个航点:", waypoints.size());
    for (size_t i = 0; i < waypoints.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "  航点 %zu: %s (%.1f, %.1f, %.1f)",
                  i + 1, waypoints[i].description.c_str(), waypoints[i].x,
                  waypoints[i].y, waypoints[i].z);
    }
  }
  void controlLoop() {
    px4_comm_.update();
    auto comm_state = px4_comm_.getCommunicationState();
    if (comm_state != PX4Communicator::CommState::NORMAL) {
      if (current_state_ != TestState::EMERGENCY) {
        RCLCPP_ERROR(this->get_logger(), "检测到紧急状态，停止测试！");
        current_state_ = TestState::EMERGENCY;
      }
      return;
    }
    switch (current_state_) {
    case TestState::INITIALIZING:
      handleInitializing();
      break;
    case TestState::ARMING:
      handleArming();
      break;
    case TestState::TAKING_OFF:
      handleTakeOff();
      break;
    case TestState::WAYPOINT_FLIGHT:
      handleWaypointFlight();
      break;
    case TestState::VISUAL_SERVO:
      handleVisualServo();
      break;
    case TestState::VISUAL_SERVO_ALIGNED_HOVER:
      handleVisualServoAlignedHover();
      break;
    case TestState::WAITING_FOR_SERVO:
      handleWaitingForServo();
      break;
    case TestState::LANDING:
      handleLanding();
      break;
    case TestState::COMPLETED:
      handleCompleted();
      break;
    case TestState::EMERGENCY:
      handleEmergency();
      break;
    }
  }
  void handleInitializing() {
    auto elapsed = this->get_clock()->now() - test_start_time_;
    if (elapsed.nanoseconds() / 1000000 > INITIALIZATION_TIME_MS) {
      RCLCPP_INFO(this->get_logger(), "初始化完成，开始解锁...");
      current_state_ = TestState::ARMING;
      state_start_time_ = this->get_clock()->now();
    }
  }
  void handleArming() {
    auto control_status = px4_comm_.getControlStatus();
    if (!control_status.valid) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                           LOG_THROTTLE_MS, "等待PX4控制状态数据...");
      return;
    }
    auto elapsed = this->get_clock()->now() - state_start_time_;
    if (elapsed.nanoseconds() / 1000000 > ARM_TIMEOUT_MS) {
      RCLCPP_ERROR(this->get_logger(),
                   "解锁超时！%.1fs内无法解锁，进入紧急状态",
                   ARM_TIMEOUT_MS / 1000.0);
      RCLCPP_ERROR(
          this->get_logger(),
          "请检查：1) 遥控器开关位置 2) 安全检查 3) GPS信号 4) 传感器状态");
      current_state_ = TestState::EMERGENCY;
      return;
    }
    if (!control_status.is_armed) {
      auto now = this->get_clock()->now();
      auto since_last_attempt = now - last_arm_attempt_time_;
      if (since_last_attempt.nanoseconds() / 1000000 > ARM_RETRY_INTERVAL_MS) {
        auto result = px4_comm_.arm();
        last_arm_attempt_time_ = now;
        float remaining_time =
            (ARM_TIMEOUT_MS - elapsed.nanoseconds() / 1000000) / 1000.0;
        if (result.accepted) {
          RCLCPP_INFO(this->get_logger(), "发送解锁命令 (剩余时间: %.1fs)",
                      remaining_time);
        } else {
          RCLCPP_WARN(this->get_logger(), "解锁失败: %s (剩余时间: %.1fs)",
                      result.reason.c_str(), remaining_time);
        }
      } else {
        float remaining_time =
            (ARM_TIMEOUT_MS - elapsed.nanoseconds() / 1000000) / 1000.0;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                             STATUS_LOG_INTERVAL_MS,
                             "等待解锁... (剩余时间: %.1fs)", remaining_time);
      }
    } else {
      px4_comm_.setPositionSetpoint(0.0f, 0.0f, TAKEOFF_TARGET_HEIGHT, 0.0f);
      if (!control_status.is_offboard) {
        auto result = px4_comm_.setOffboardMode();
        if (result.accepted) {
          RCLCPP_INFO(this->get_logger(),
                      "切换到Offboard模式成功，开始起飞...");
          current_state_ = TestState::TAKING_OFF;
          state_start_time_ = this->get_clock()->now();
        } else {
          RCLCPP_WARN(this->get_logger(), "切换Offboard模式失败: %s",
                      result.reason.c_str());
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "已处于Offboard模式，开始起飞...");
        current_state_ = TestState::TAKING_OFF;
        state_start_time_ = this->get_clock()->now();
      }
    }
  }
  void handleTakeOff() {
    auto position = px4_comm_.getCurrentPosition();
    if (!position.valid) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                           LOG_THROTTLE_MS, "等待位置数据...");
      return;
    }
    if (position.z <= TAKEOFF_TARGET_HEIGHT + TAKEOFF_HEIGHT_TOLERANCE) {
      RCLCPP_INFO(this->get_logger(), "起飞完成，开始航点飞行！");
      current_state_ = TestState::WAYPOINT_FLIGHT;
      state_start_time_ = this->get_clock()->now();
    } else {
      px4_comm_.setPositionSetpoint(0.0f, 0.0f, TAKEOFF_TARGET_HEIGHT, 0.0f);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                           LOG_THROTTLE_MS,
                           "起飞中... 当前高度: %.2f米，目标: %.1f米",
                           -position.z, -TAKEOFF_TARGET_HEIGHT);
    }
  }
  void handleWaypointFlight() {
    auto position = px4_comm_.getCurrentPosition();
    if (!position.valid) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                           LOG_THROTTLE_MS, "航点飞行期间位置数据丢失");
      return;
    }
    VelocityAPFController::InputData input;
    input.current_x = position.x;
    input.current_y = position.y;
    input.current_z = position.z;
    input.current_yaw = position.yaw;
    input.scan_data = latest_scan_data_;
    input.avoidance_enabled = true;
    auto output = waypoint_controller_.calculateVelocity(input);
    // 检查当前航点是否是触发视觉伺服的航点
    if (VISUAL_SERVO_TRIGGER_POINTS.count(output.current_waypoint_index) > 0 &&
        output.waypoint_reached) {
      const auto &target_waypoint_config =
          TEST_WAYPOINTS[output.current_waypoint_index];
      const std::string &target_name =
          VISUAL_SERVO_TRIGGER_POINTS.at(output.current_waypoint_index);

      RCLCPP_INFO(this->get_logger(),
                  "到达目标航点%zu, 切换到视觉伺服模式！目标: %s",
                  output.current_waypoint_index + 1, target_name.c_str());

      auto current_pos = px4_comm_.getCurrentPosition();
      float fallback_yaw = 0.0f;

      // 设置视觉伺服控制器的回退点和目标名称
      VisualServoController::Waypoint fallback_waypoint(
          target_waypoint_config.x, target_waypoint_config.y,
          target_waypoint_config.z, fallback_yaw);
      visual_servo_controller_.setWaypoint(fallback_waypoint);
      visual_servo_controller_.setTargetLabel(
          target_name); // 新增: 设置视觉伺服的目标名称

      RCLCPP_INFO(this->get_logger(),
                  "已设置视觉伺服的回退点为航点%zu (%.2f, %.2f, %.2f)",
                  output.current_waypoint_index + 1, fallback_waypoint.x,
                  fallback_waypoint.y, -fallback_waypoint.z);

      current_state_ = TestState::VISUAL_SERVO;
      state_start_time_ = this->get_clock()->now();
      return;
    }
    if (output.sequence_completed) {
      RCLCPP_INFO(this->get_logger(), "所有航点飞行完成，开始降落！");
      current_state_ = TestState::LANDING;
      state_start_time_ = this->get_clock()->now();
    } else {
      if (output.is_pid_active) {
        px4_comm_.setVelocitySetpoint(output.vx, output.vy, output.vz, 0.0f);
        auto current_waypoint = waypoint_controller_.getCurrentWaypoint();
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), LOG_THROTTLE_MS,
            "航点 %zu/%zu: %s | 位置(%.2f,%.2f,%.2f) -> 目标(%.1f,%.1f,%.1f) | "
            "速度(%.2f,%.2f,%.2f)",
            output.current_waypoint_index + 1,
            waypoint_controller_.getWaypointCount(),
            current_waypoint.description.c_str(), position.x, position.y,
            -position.z, current_waypoint.x, current_waypoint.y,
            -current_waypoint.z, output.vx, output.vy, output.vz);
      }
    }
  }
  void handleVisualServo() {
    auto position = px4_comm_.getCurrentPosition();
    if (!position.valid) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                           LOG_THROTTLE_MS, "视觉伺服期间位置数据丢失");
      return;
    }
    VisualServoController::InputData input;
    input.current_x = position.x;
    input.current_y = position.y;
    input.current_z = position.z;
    input.current_yaw = position.yaw;

    // 只有检测到的目标名称与当前设定的目标名称匹配时才进行视觉伺服
    if (latest_detection_data_.valid &&
        latest_detection_data_.name ==
            visual_servo_controller_.getTargetLabel()) {
      input.detection_x = latest_detection_data_.x;
      input.detection_y = latest_detection_data_.y;
      input.detection_name = latest_detection_data_.name;
      input.detection_confidence = latest_detection_data_.confidence;
      input.detection_valid = true;
    } else {
      input.detection_valid = false;
      input.detection_confidence = 0.0f;
      input.detection_name = "";
      if (latest_detection_data_.valid) {
        RCLCPP_DEBUG_THROTTLE(
            this->get_logger(), *this->get_clock(), LOG_THROTTLE_MS,
            "检测到目标 '%s' 但不匹配视觉伺服目标 '%s'",
            latest_detection_data_.name.c_str(),
            visual_servo_controller_.getTargetLabel().c_str());
      }
    }
    auto output = visual_servo_controller_.calculateVelocity(input);
    if (output.servo_active) {
      px4_comm_.setVelocitySetpoint(output.vx, output.vy, output.vz, 0.0f);

      // =========================================================================
      // [逻辑修正] 只在精对准(VS)模式下判断是否到达
      // =========================================================================
      if (output.control_mode ==
              VisualServoController::ControlMode::VISUAL_SERVO &&
          output.target_reached) {
        RCLCPP_INFO(
            this->get_logger(),
            "视觉精对准完成！最终视觉误差: %.3f米。开始悬停观察 %.1f 秒...",
            output.tracking_error, VISUAL_SERVO_HOVER_DURATION_MS / 1000.0);

        current_state_ = TestState::VISUAL_SERVO_ALIGNED_HOVER;
        visual_servo_aligned_start_time_ = this->get_clock()->now();
      } else if (output.detection_lost) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), LOG_THROTTLE_MS,
            "目标检测丢失，搜索中... 误差X: %.3f, 误差Y: %.3f",
            output.compensated_error_x, output.compensated_error_y);
      } else {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), LOG_THROTTLE_MS,
            "视觉伺服中... 误差: %.3f米，速度(%.2f,%.2f,%.2f)",
            output.tracking_error, output.vx, output.vy, output.vz);
      }
    } else {
      px4_comm_.setVelocitySetpoint(0.0f, 0.0f, 0.0f, 0.0f);
      // --- 修改日志逻辑 ---
      // 根据控制器返回的实际模式显示不同信息
      const char *mode_str = "未知";
      switch (output.control_mode) {
      case VisualServoController::ControlMode::VISUAL_SERVO:
        mode_str = "视觉伺服";
        break;
      case VisualServoController::ControlMode::WAYPOINT:
        mode_str = "航点回退";
        break;
      case VisualServoController::ControlMode::SEARCH:
        mode_str = "搜索模式";
        break;
      }
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), LOG_THROTTLE_MS,
          "[%s] 跟踪误差: %.3f米 | 速度(vx:%.2f, vy:%.2f)", mode_str,
          output.tracking_error, output.vx, output.vy);
      // --- 修改日志逻辑结束 ---
    }
    auto elapsed = this->get_clock()->now() - state_start_time_;
    if (elapsed.nanoseconds() / 1000000 > VISUAL_SERVO_TIMEOUT_MS) {
      RCLCPP_WARN(this->get_logger(), "视觉伺服超时(%.1fs)，开始降落！",
                  VISUAL_SERVO_TIMEOUT_MS / 1000.0);
      current_state_ = TestState::LANDING;
      state_start_time_ = this->get_clock()->now();
    }
  }

  // --- 新增: 实现新的状态处理函数 ---
  void handleVisualServoAlignedHover() {
    // 命令无人机悬停 (发送0速度)
    px4_comm_.setVelocitySetpoint(0.0f, 0.0f, 0.0f, 0.0f);

    auto elapsed = this->get_clock()->now() - visual_servo_aligned_start_time_;
    float remaining_time =
        (VISUAL_SERVO_HOVER_DURATION_MS - elapsed.nanoseconds() / 1e6) / 1000.0;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "正在对准目标悬停... 剩余 %.1f 秒",
                         std::max(0.0f, remaining_time));

    // 检查是否已达到悬停时间
    if (elapsed.nanoseconds() / 1000000 > VISUAL_SERVO_HOVER_DURATION_MS) {
      RCLCPP_INFO(this->get_logger(), "悬停观察完成，触发舵机动作并等待...");

      // --- 舵机角度递减并发布 ---
      float next_angle = current_servo_angle_ - SERVO_STEP;
      if (next_angle < SERVO_MIN_ANGLE)
        next_angle = SERVO_MIN_ANGLE;
      if (next_angle != current_servo_angle_) {
        current_servo_angle_ = next_angle;
        publish_servo_angle(current_servo_angle_);
        RCLCPP_INFO(this->get_logger(), "舵机角度已调整为 %.1f 度",
                    current_servo_angle_);
      }

      // --- 修改状态转换 ---
      current_state_ = TestState::WAITING_FOR_SERVO; // 切换到等待状态
      servo_action_start_time_ = this->get_clock()->now(); // 记录开始时间
    }
  }

  void handleWaitingForServo() {
    // 持续命令无人机悬停
    px4_comm_.setVelocitySetpoint(0.0f, 0.0f, 0.0f, 0.0f);

    auto elapsed = this->get_clock()->now() - servo_action_start_time_;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "等待舵机动作完成...");

    // 检查等待时间是否足够
    if (elapsed.nanoseconds() / 1000000 > SERVO_ACTION_DURATION_MS) {
      RCLCPP_INFO(this->get_logger(), "舵机动作时间到，继续执行航点任务！");
      waypoint_controller_.advanceToNextWaypoint();
      current_state_ = TestState::WAYPOINT_FLIGHT;
      state_start_time_ = this->get_clock()->now();
    }
  }

  void handleLanding() {
    auto position = px4_comm_.getCurrentPosition();
    auto control_status = px4_comm_.getControlStatus();
    px4_comm_.land();
    if (position.valid && control_status.valid) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                           LOG_THROTTLE_MS, "降落中... 当前高度: %.2f米",
                           -position.z);
      if (position.z > LANDING_HEIGHT_THRESHOLD) {
        RCLCPP_INFO(this->get_logger(), "着陆完成！(检测方式: 高度阈值)");
        current_state_ = TestState::COMPLETED;
        test_end_time_ = this->get_clock()->now();
      }
    }
  }
  void handleCompleted() {
    px4_comm_.disarm();
    auto total_time = test_end_time_ - test_start_time_;
    float total_seconds = total_time.nanoseconds() / 1e9;
    RCLCPP_INFO(this->get_logger(), "=== 航点飞行测试完成 ===");
    RCLCPP_INFO(this->get_logger(), "总耗时: %.1f 秒", total_seconds);
    RCLCPP_INFO(this->get_logger(), "测试程序将在%.1f秒后退出...",
                SHUTDOWN_DELAY_MS / 1000.0);
    auto shutdown_delay = std::chrono::milliseconds(SHUTDOWN_DELAY_MS);
    this->create_wall_timer(shutdown_delay, [this]() {
      RCLCPP_INFO(this->get_logger(), "测试程序退出");
      rclcpp::shutdown();
    });
    current_state_ = TestState::EMERGENCY;
  }
  void handleEmergency() {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(),
                          STATUS_LOG_INTERVAL_MS,
                          "紧急状态激活，等待安全恢复或手动接管");
  }

  void sequenceCompleteCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      RCLCPP_INFO(this->get_logger(), "收到航点序列完成信号");
    }
  }

  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_data_ = msg;
  }

  // === 修改开始: 替换为三个独立的回调函数 ===
  void yoloNameCallback(const std_msgs::msg::String::SharedPtr msg) {
    // 只更新名称数据
    latest_name_ = msg->data;
  }

  void yoloConfidenceCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    // 只更新置信度数据
    latest_confidence_ = msg->data;
    has_received_confidence_ = true; // 标记已收到过置信度
  }

  void
  yoloCentreCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    // 以 centre 消息作为"触发器"来组合所有数据
    // 这样可以确保我们总是有最新的位置信息
    if (!has_received_confidence_ || latest_name_.empty()) {
      // 如果还没有收到过其他信息，则不处理，避免数据不完整
      RCLCPP_DEBUG(this->get_logger(),
                   "收到了中心点，但名称或置信度数据不完整，等待中...");
      return;
    }

    latest_detection_data_.valid = true;
    latest_detection_data_.name = latest_name_; // 使用最新收到的名称
    latest_detection_data_.confidence =
        latest_confidence_; // 使用最新收到的置信度
    latest_detection_data_.x = static_cast<float>(msg->point.x);
    latest_detection_data_.y = static_cast<float>(msg->point.y);
    // 删除时间戳处理，YOLO检测数据不需要时间戳也能正常工作

    RCLCPP_DEBUG(this->get_logger(),
                 "组合检测数据: 名称='%s', 置信度=%.2f, 位置=(%.3f, %.3f)",
                 latest_detection_data_.name.c_str(),
                 latest_detection_data_.confidence, latest_detection_data_.x,
                 latest_detection_data_.y);
  }
  // === 修改结束 ===

  // 成员变量
  PX4Communicator px4_comm_;
  VelocityAPFController waypoint_controller_;
  VisualServoController visual_servo_controller_;

  // 舵机控制相关
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servo_angle_pub_;
  float current_servo_angle_;

  // 状态变量
  TestState current_state_;
  rclcpp::Time test_start_time_;
  rclcpp::Time test_end_time_;
  rclcpp::Time state_start_time_;
  rclcpp::Time last_arm_attempt_time_;
  // --- 新增: 用于记录对准开始时间的变量 ---
  rclcpp::Time visual_servo_aligned_start_time_;
  // --- 新增: 用于记录舵机动作开始时间的变量 ---
  rclcpp::Time servo_action_start_time_;
  static constexpr int64_t SERVO_ACTION_DURATION_MS =
      1500; // 舵机释放后等待1.5秒

  // 传感器数据
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_data_;

  // 目标检测数据结构
  struct DetectionData {
    bool valid = false;
    std::string name;
    float x = 0.0f;
    float y = 0.0f;
    float confidence = 0.0f;
    // 删除时间戳字段，YOLO检测数据不需要时间戳
  } latest_detection_data_;

  // === 修改开始: 修改成员变量以适应手动同步 ===
  // 用于临时存储YOLO数据的成员变量
  std::string latest_name_;
  float latest_confidence_ = 0.0f;
  bool has_received_confidence_ = false; // 用于检查是否收到过初始数据

  // 订阅器和定时器
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sequence_complete_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;

  // YOLO相关的独立订阅器
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_name_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yolo_confidence_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      yolo_centre_sub_;
  // === 修改结束 ===

  // --- 舵机角度发布辅助函数 ---
  void publish_servo_angle(float angle) {
    if (servo_angle_pub_) {
      std_msgs::msg::Float32 msg;
      msg.data = angle;
      servo_angle_pub_->publish(msg);
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("main"), "启动航点飞行测试程序...");

  rclcpp::spin(std::make_shared<WaypointFlightTest>());

  rclcpp::shutdown();

  RCLCPP_INFO(rclcpp::get_logger("main"), "航点飞行测试程序结束");

  return 0;
}