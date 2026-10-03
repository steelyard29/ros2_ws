#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include "px4_communicator.hpp"
#include "velocity_apf_controller.hpp"

namespace {

template <typename T>
T clampValue(T value, T low, T high) {
  return std::max(low, std::min(high, value));
}

double distance2d(double ax, double ay, double bx, double by) {
  const double dx = ax - bx;
  const double dy = ay - by;
  return std::sqrt(dx * dx + dy * dy);
}

} // namespace

class ObstacleCourseMission : public rclcpp::Node {
public:
  ObstacleCourseMission()
      : Node("obstacle_course_mission"), px4_comm_(this),
        waypoint_controller_(this) {
    loadParameters();
    setupWaypoints();
    setupRosInterfaces();

    state_ = MissionState::INITIALIZING;
    mission_start_time_ = now();
    state_start_time_ = now();
    last_arm_attempt_time_ = now();

    control_timer_ = create_wall_timer(
        std::chrono::milliseconds(1000 / control_frequency_hz_),
        std::bind(&ObstacleCourseMission::controlLoop, this));

    RCLCPP_INFO(get_logger(), "Obstacle course mission started with %zu waypoints",
                mission_waypoints_.size());
  }

private:
  enum class MissionState {
    INITIALIZING,
    ARMING,
    TAKING_OFF,
    MISSION_FLIGHT,
    LAND_APPROACH,
    APRILTAG_ALIGN,
    CIRCLE_ALIGN_FALLBACK,
    PRECISION_DESCEND,
    FINAL_LAND,
    COMPLETED,
    EMERGENCY
  };

  struct LandingTarget {
    bool valid = false;
    double forward_m = 0.0;
    double right_m = 0.0;
    double distance_m = 0.0;
    rclcpp::Time stamp;
  };

  void loadParameters() {
    control_frequency_hz_ = declare_parameter<int>("control_frequency_hz", 50);
    initialization_time_s_ = declare_parameter<double>("initialization_time_s", 3.0);
    arm_timeout_s_ = declare_parameter<double>("arm_timeout_s", 8.0);
    arm_retry_interval_s_ = declare_parameter<double>("arm_retry_interval_s", 1.0);
    takeoff_height_m_ = declare_parameter<double>("takeoff_height_m", 1.2);
    takeoff_tolerance_m_ = declare_parameter<double>("takeoff_tolerance_m", 0.18);
    estimator_timeout_s_ = declare_parameter<double>("safety.estimator_timeout_s", 0.5);
    estimator_failure_grace_s_ =
        declare_parameter<double>("safety.estimator_failure_grace_s", 0.8);

    mission_waypoints_flat_ = declare_parameter<std::vector<double>>(
        "mission_waypoints_flat",
        {0.0, 0.0, -1.2, 1.5, 0.0, -1.2, 3.0, 0.5, -1.2,
         5.0, 0.5, -1.2, 7.0, 0.0, -1.2, 9.0, 0.5, -1.2,
         11.0, 0.0, -1.2});
    mission_waypoint_names_ = declare_parameter<std::vector<std::string>>(
        "mission_waypoint_names",
        {"起飞后稳定", "A区入口", "A区出口/B区入口", "B区出口/C区入口",
         "C区出口/D区入口", "D区穿越", "D区出口"});

    landing_x_ = declare_parameter<double>("landing.x", 12.0);
    landing_y_ = declare_parameter<double>("landing.y", 0.0);
    landing_z_ = declare_parameter<double>("landing.approach_z", -1.2);
    landing_xy_tolerance_m_ = declare_parameter<double>("landing.xy_tolerance_m", 0.25);
    landing_approach_timeout_s_ = declare_parameter<double>("landing.approach_timeout_s", 20.0);
    final_land_height_m_ = declare_parameter<double>("landing.final_land_height_m", 0.35);
    precision_descent_start_height_m_ =
        declare_parameter<double>("landing.precision_descent_start_height_m", 1.0);
    descent_speed_mps_ = declare_parameter<double>("landing.descent_speed_mps", 0.2);

    tag_topic_ = declare_parameter<std::string>("apriltag.pose_topic", "/landing_target/apriltag_pose");
    landing_tag_id_ = declare_parameter<int>("apriltag.landing_tag_id", 0);
    tag_timeout_s_ = declare_parameter<double>("apriltag.timeout_s", 1.0);
    tag_search_timeout_s_ = declare_parameter<double>("apriltag.search_timeout_s", 8.0);
    tag_forward_axis_ = declare_parameter<int>("apriltag.forward_axis", 0);
    tag_right_axis_ = declare_parameter<int>("apriltag.right_axis", 1);
    tag_forward_sign_ = declare_parameter<double>("apriltag.forward_sign", 1.0);
    tag_right_sign_ = declare_parameter<double>("apriltag.right_sign", 1.0);

    circle_center_topic_ =
        declare_parameter<std::string>("circle.center_topic", "/landing_target/circle_center");
    circle_distance_topic_ =
        declare_parameter<std::string>("circle.distance_topic", "/landing_target/distance_m");
    circle_valid_topic_ =
        declare_parameter<std::string>("circle.valid_topic", "/landing_target/valid");
    circle_timeout_s_ = declare_parameter<double>("circle.timeout_s", 1.0);
    circle_search_timeout_s_ = declare_parameter<double>("circle.search_timeout_s", 8.0);
    circle_image_width_ = declare_parameter<double>("circle.image_width", 1280.0);
    circle_image_height_ = declare_parameter<double>("circle.image_height", 720.0);
    circle_focal_px_ = declare_parameter<double>("circle.focal_px", 875.0);
    circle_forward_pixel_sign_ = declare_parameter<double>("circle.forward_pixel_sign", -1.0);
    circle_right_pixel_sign_ = declare_parameter<double>("circle.right_pixel_sign", 1.0);

    align_tolerance_m_ = declare_parameter<double>("landing.align_tolerance_m", 0.12);
    align_stable_time_s_ = declare_parameter<double>("landing.align_stable_time_s", 1.0);
    max_align_velocity_mps_ = declare_parameter<double>("landing.max_align_velocity_mps", 0.25);
    align_kp_ = declare_parameter<double>("landing.align_kp", 0.45);

    laser_topic_ = declare_parameter<std::string>("laser_topic", "/scan");
    yolo_name_topic_ = declare_parameter<std::string>("yolo.name_topic", "/yolo/scene/name");
    yolo_confidence_topic_ =
        declare_parameter<std::string>("yolo.confidence_topic", "/yolo/scene/confidence");
  }

  void setupWaypoints() {
    mission_waypoints_.clear();
    for (size_t i = 0; i + 2 < mission_waypoints_flat_.size(); i += 3) {
      const size_t index = i / 3;
      const std::string name =
          index < mission_waypoint_names_.size()
              ? mission_waypoint_names_[index]
              : "mission waypoint " + std::to_string(index + 1);
      mission_waypoints_.emplace_back(static_cast<float>(mission_waypoints_flat_[i]),
                                      static_cast<float>(mission_waypoints_flat_[i + 1]),
                                      static_cast<float>(mission_waypoints_flat_[i + 2]),
                                      name);
    }
    if (mission_waypoints_.empty()) {
      mission_waypoints_.emplace_back(0.0f, 0.0f, static_cast<float>(-takeoff_height_m_),
                                      "fallback takeoff hold");
    }
    waypoint_controller_.setWaypointSequence(mission_waypoints_);
  }

  void setupRosInterfaces() {
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        laser_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ObstacleCourseMission::laserCallback, this, std::placeholders::_1));
    tag_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        tag_topic_, 10,
        std::bind(&ObstacleCourseMission::tagCallback, this, std::placeholders::_1));
    circle_center_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        circle_center_topic_, 10,
        std::bind(&ObstacleCourseMission::circleCenterCallback, this, std::placeholders::_1));
    circle_distance_sub_ = create_subscription<std_msgs::msg::Float32>(
        circle_distance_topic_, 10,
        std::bind(&ObstacleCourseMission::circleDistanceCallback, this, std::placeholders::_1));
    circle_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
        circle_valid_topic_, 10,
        std::bind(&ObstacleCourseMission::circleValidCallback, this, std::placeholders::_1));
    yolo_name_sub_ = create_subscription<std_msgs::msg::String>(
        yolo_name_topic_, 10,
        [this](const std_msgs::msg::String::SharedPtr msg) { latest_yolo_name_ = msg->data; });
    yolo_confidence_sub_ = create_subscription<std_msgs::msg::Float32>(
        yolo_confidence_topic_, 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          latest_yolo_confidence_ = msg->data;
        });
    estimator_flags_sub_ =
        create_subscription<px4_msgs::msg::EstimatorStatusFlags>(
            "/fmu/out/estimator_status_flags", rclcpp::SensorDataQoS(),
            std::bind(&ObstacleCourseMission::estimatorFlagsCallback, this,
                      std::placeholders::_1));
  }

  void controlLoop() {
    px4_comm_.update();
    if (px4_comm_.getCommunicationState() != PX4Communicator::CommState::NORMAL) {
      state_ = MissionState::EMERGENCY;
      return;
    }
    const bool estimator_healthy = estimatorHealthy();
    if (!estimator_healthy) {
      if (estimator_unhealthy_since_.nanoseconds() == 0) {
        estimator_unhealthy_since_ = now();
      }
      if (state_ != MissionState::INITIALIZING &&
          state_ != MissionState::FINAL_LAND &&
          state_ != MissionState::COMPLETED &&
          state_ != MissionState::EMERGENCY &&
          elapsedSeconds(estimator_unhealthy_since_) >=
              estimator_failure_grace_s_) {
        transitionTo(MissionState::EMERGENCY,
                     "external vision/range estimator unhealthy");
      }
    } else {
      estimator_unhealthy_since_ =
          rclcpp::Time(0, 0, get_clock()->get_clock_type());
      if (estimator_healthy_since_.nanoseconds() == 0) {
        estimator_healthy_since_ = now();
      }
    }

    switch (state_) {
    case MissionState::INITIALIZING:
      handleInitializing();
      break;
    case MissionState::ARMING:
      handleArming();
      break;
    case MissionState::TAKING_OFF:
      handleTakeoff();
      break;
    case MissionState::MISSION_FLIGHT:
      handleMissionFlight();
      break;
    case MissionState::LAND_APPROACH:
      handleLandApproach();
      break;
    case MissionState::APRILTAG_ALIGN:
      handleAprilTagAlign();
      break;
    case MissionState::CIRCLE_ALIGN_FALLBACK:
      handleCircleAlign();
      break;
    case MissionState::PRECISION_DESCEND:
      handlePrecisionDescend();
      break;
    case MissionState::FINAL_LAND:
      handleFinalLand();
      break;
    case MissionState::COMPLETED:
      handleCompleted();
      break;
    case MissionState::EMERGENCY:
      handleEmergency();
      break;
    }
  }

  void transitionTo(MissionState next, const std::string &reason) {
    state_ = next;
    state_start_time_ = now();
    align_stable_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    RCLCPP_INFO(get_logger(), "Mission transition: %s", reason.c_str());
  }

  void handleInitializing() {
    if (!estimatorHealthy()) {
      estimator_healthy_since_ =
          rclcpp::Time(0, 0, get_clock()->get_clock_type());
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "waiting for EKF external-vision position and range height");
      return;
    }
    if (elapsedSeconds(estimator_healthy_since_) >= initialization_time_s_) {
      transitionTo(MissionState::ARMING, "initialization complete, arming");
    }
  }

  void handleArming() {
    const auto control = px4_comm_.getControlStatus();
    if (!control.valid) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for PX4 control status");
      return;
    }
    if (elapsedSeconds(state_start_time_) > arm_timeout_s_) {
      transitionTo(MissionState::EMERGENCY, "arm timeout");
      return;
    }

    // PX4 requires a continuous OffboardControlMode/setpoint stream before
    // accepting the mode switch.  Start streaming the takeoff hold while the
    // vehicle is still disarmed; this is safe (the command is only a setpoint)
    // and avoids the old arm-then-stream race.
    px4_comm_.setPositionSetpoint(0.0f, 0.0f,
                                  static_cast<float>(-takeoff_height_m_), 0.0f);

    if (!control.is_armed) {
      if (elapsedSeconds(last_arm_attempt_time_) >= arm_retry_interval_s_) {
        px4_comm_.arm();
        last_arm_attempt_time_ = now();
      }
      return;
    }

    if (!control.is_offboard) {
      px4_comm_.setOffboardMode();
      return;
    }
    transitionTo(MissionState::TAKING_OFF, "armed, taking off");
  }

  void handleTakeoff() {
    const auto position = px4_comm_.getCurrentPosition();
    if (!position.valid) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for position during takeoff");
      return;
    }
    px4_comm_.setPositionSetpoint(0.0f, 0.0f, static_cast<float>(-takeoff_height_m_), 0.0f);
    if (std::abs(position.z + takeoff_height_m_) <= takeoff_tolerance_m_) {
      transitionTo(MissionState::MISSION_FLIGHT, "takeoff complete, starting A/B/C/D mission");
    }
  }

  void handleMissionFlight() {
    const auto position = px4_comm_.getCurrentPosition();
    if (!position.valid) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for position during mission");
      return;
    }

    VelocityAPFController::InputData input;
    input.current_x = static_cast<float>(position.x);
    input.current_y = static_cast<float>(position.y);
    input.current_z = static_cast<float>(position.z);
    input.current_yaw = position.yaw;
    input.scan_data = latest_scan_;
    input.avoidance_enabled = true;

    const auto output = waypoint_controller_.calculateVelocity(input);
    if (output.sequence_completed) {
      transitionTo(MissionState::LAND_APPROACH, "A/B/C/D mission waypoints complete");
      return;
    }

    px4_comm_.setVelocitySetpoint(output.vx, output.vy, output.vz, 0.0f);
    const auto waypoint = waypoint_controller_.getCurrentWaypoint();
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "mission wp %zu/%zu %s pos=(%.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f) yolo=%s %.2f",
        output.current_waypoint_index + 1, waypoint_controller_.getWaypointCount(),
        waypoint.description.c_str(), position.x, position.y, -position.z,
        waypoint.x, waypoint.y, -waypoint.z, output.vx, output.vy, output.vz,
        latest_yolo_name_.c_str(), latest_yolo_confidence_);
  }

  void handleLandApproach() {
    const auto position = px4_comm_.getCurrentPosition();
    if (!position.valid) {
      return;
    }
    px4_comm_.setPositionSetpoint(static_cast<float>(landing_x_), static_cast<float>(landing_y_),
                                  static_cast<float>(landing_z_), 0.0f);
    if (distance2d(position.x, position.y, landing_x_, landing_y_) <= landing_xy_tolerance_m_) {
      if (isFreshTag()) {
        transitionTo(MissionState::APRILTAG_ALIGN, "landing approach complete, using AprilTag");
      } else {
        transitionTo(MissionState::CIRCLE_ALIGN_FALLBACK,
                     "landing approach complete, waiting for circle fallback");
      }
      return;
    }
    if (elapsedSeconds(state_start_time_) > landing_approach_timeout_s_) {
      transitionTo(MissionState::FINAL_LAND, "landing approach timeout, ordinary land");
    }
  }

  void handleAprilTagAlign() {
    if (!isFreshTag()) {
      if (elapsedSeconds(state_start_time_) >= tag_search_timeout_s_) {
        transitionTo(MissionState::CIRCLE_ALIGN_FALLBACK, "AprilTag unavailable, circle fallback");
      } else {
        holdPosition();
      }
      return;
    }
    runAlignController(latest_tag_, "apriltag");
  }

  void handleCircleAlign() {
    const auto circle = currentCircleTarget();
    if (!circle.valid) {
      if (elapsedSeconds(state_start_time_) >= circle_search_timeout_s_) {
        transitionTo(MissionState::FINAL_LAND, "circle unavailable, ordinary land");
      } else {
        holdPosition();
      }
      return;
    }
    runAlignController(circle, "circle");
  }

  void handlePrecisionDescend() {
    LandingTarget target;
    std::string source;
    if (isFreshTag()) {
      target = latest_tag_;
      source = "apriltag";
    } else {
      target = currentCircleTarget();
      source = "circle";
    }

    const auto position = px4_comm_.getCurrentPosition();
    if (!position.valid) {
      return;
    }
    const double height_m = -position.z;
    const double descent_speed =
        height_m > precision_descent_start_height_m_
            ? std::min(descent_speed_mps_, 0.15)
            : descent_speed_mps_;
    if (height_m <= final_land_height_m_) {
      transitionTo(MissionState::FINAL_LAND, "low enough for PX4 final land");
      return;
    }

    if (!target.valid) {
      if (elapsedSeconds(state_start_time_) > std::max(tag_search_timeout_s_, circle_search_timeout_s_)) {
        transitionTo(MissionState::FINAL_LAND, "precision target lost too long");
      } else {
        holdPosition();
      }
      return;
    }

    const auto [vx, vy] = bodyErrorToNedVelocity(target.forward_m, target.right_m, position.yaw);
    const float vz = static_cast<float>(descent_speed);
    px4_comm_.setVelocitySetpoint(vx, vy, vz, 0.0f);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "precision descend using %s height=%.2fm error=(%.2f,%.2f) vel=(%.2f,%.2f,%.2f)",
                         source.c_str(), height_m, target.forward_m, target.right_m, vx, vy, vz);
  }

  void handleFinalLand() {
    px4_comm_.land();
    const auto position = px4_comm_.getCurrentPosition();
    const auto control = px4_comm_.getControlStatus();
    if ((position.valid && position.z > -0.1) || (control.valid && !control.is_armed)) {
      transitionTo(MissionState::COMPLETED, "land complete or disarmed");
    }
  }

  void handleCompleted() {
    px4_comm_.disarm();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "mission completed");
  }

  void handleEmergency() {
    px4_comm_.land();
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                          "mission emergency state active; commanding PX4 land");
    const auto control = px4_comm_.getControlStatus();
    if (control.valid && !control.is_armed) {
      transitionTo(MissionState::COMPLETED, "emergency landing disarmed");
    }
  }

  void runAlignController(const LandingTarget &target, const std::string &source) {
    const auto position = px4_comm_.getCurrentPosition();
    if (!position.valid) {
      return;
    }
    const double error = std::hypot(target.forward_m, target.right_m);
    if (error <= align_tolerance_m_) {
      if (align_stable_since_.nanoseconds() == 0) {
        align_stable_since_ = now();
      }
      if (elapsedSeconds(align_stable_since_) >= align_stable_time_s_) {
        transitionTo(MissionState::PRECISION_DESCEND,
                     source + " aligned, starting precision descent");
        return;
      }
    } else {
      align_stable_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }

    const auto [vx, vy] = bodyErrorToNedVelocity(target.forward_m, target.right_m, position.yaw);
    px4_comm_.setVelocitySetpoint(vx, vy, 0.0f, 0.0f);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "%s align error=(%.2f,%.2f) norm=%.2f vel=(%.2f,%.2f)",
                         source.c_str(), target.forward_m, target.right_m, error, vx, vy);
  }

  std::pair<float, float> bodyErrorToNedVelocity(double forward_error_m, double right_error_m,
                                                 double yaw_rad) const {
    double body_forward = clampValue(align_kp_ * forward_error_m, -max_align_velocity_mps_,
                                    max_align_velocity_mps_);
    double body_right = clampValue(align_kp_ * right_error_m, -max_align_velocity_mps_,
                                  max_align_velocity_mps_);
    const double magnitude = std::hypot(body_forward, body_right);
    if (magnitude > max_align_velocity_mps_) {
      body_forward = body_forward / magnitude * max_align_velocity_mps_;
      body_right = body_right / magnitude * max_align_velocity_mps_;
    }

    const double cy = std::cos(yaw_rad);
    const double sy = std::sin(yaw_rad);
    const double ned_x = cy * body_forward - sy * body_right;
    const double ned_y = sy * body_forward + cy * body_right;
    return {static_cast<float>(ned_x), static_cast<float>(ned_y)};
  }

  void holdPosition() {
    const auto position = px4_comm_.getCurrentPosition();
    if (position.valid) {
      px4_comm_.setPositionSetpoint(static_cast<float>(position.x), static_cast<float>(position.y),
                                    static_cast<float>(position.z), position.yaw);
    }
  }

  bool isFreshTag() const {
    return latest_tag_.valid && elapsedSeconds(latest_tag_.stamp) <= tag_timeout_s_;
  }

  LandingTarget currentCircleTarget() const {
    LandingTarget target;
    if (!circle_valid_ || elapsedSeconds(circle_stamp_) > circle_timeout_s_ ||
        latest_circle_distance_m_ <= 0.0 || latest_circle_center_.point.z <= 0.0) {
      return target;
    }

    const double dx_px = latest_circle_center_.point.x - circle_image_width_ * 0.5;
    const double dy_px = latest_circle_center_.point.y - circle_image_height_ * 0.5;
    target.forward_m = circle_forward_pixel_sign_ * dy_px * latest_circle_distance_m_ / circle_focal_px_;
    target.right_m = circle_right_pixel_sign_ * dx_px * latest_circle_distance_m_ / circle_focal_px_;
    target.distance_m = latest_circle_distance_m_;
    target.stamp = circle_stamp_;
    target.valid = true;
    return target;
  }

  static double componentByAxis(const geometry_msgs::msg::Point &point, int axis) {
    switch (axis) {
    case 0:
      return point.x;
    case 1:
      return point.y;
    case 2:
      return point.z;
    default:
      return 0.0;
    }
  }

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { latest_scan_ = msg; }

  void tagCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    const auto &position = msg->pose.position;
    latest_tag_.forward_m = tag_forward_sign_ * componentByAxis(position, tag_forward_axis_);
    latest_tag_.right_m = tag_right_sign_ * componentByAxis(position, tag_right_axis_);
    latest_tag_.distance_m = std::abs(position.z);
    latest_tag_.stamp = now();
    latest_tag_.valid = true;
  }

  void circleCenterCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    latest_circle_center_ = *msg;
    circle_stamp_ = now();
  }

  void circleDistanceCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    latest_circle_distance_m_ = msg->data;
    circle_stamp_ = now();
  }

  void circleValidCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    circle_valid_ = msg->data;
    circle_stamp_ = now();
  }

  void estimatorFlagsCallback(
      const px4_msgs::msg::EstimatorStatusFlags::SharedPtr msg) {
    estimator_flags_ = *msg;
    estimator_flags_stamp_ = now();
    has_estimator_flags_ = true;
  }

  bool estimatorHealthy() const {
    if (!has_estimator_flags_ ||
        elapsedSeconds(estimator_flags_stamp_) > estimator_timeout_s_) {
      return false;
    }
    return estimator_flags_.cs_ev_pos && estimator_flags_.cs_rng_hgt &&
           !estimator_flags_.cs_inertial_dead_reckoning &&
           !estimator_flags_.reject_hor_pos &&
           !estimator_flags_.reject_hor_vel;
  }

  double elapsedSeconds(const rclcpp::Time &start) const {
    if (start.nanoseconds() == 0) {
      return std::numeric_limits<double>::infinity();
    }
    return (now() - start).seconds();
  }

  PX4Communicator px4_comm_;
  VelocityAPFController waypoint_controller_;

  MissionState state_ = MissionState::INITIALIZING;
  rclcpp::Time mission_start_time_;
  rclcpp::Time state_start_time_;
  rclcpp::Time last_arm_attempt_time_;
  rclcpp::Time align_stable_since_;

  int control_frequency_hz_ = 50;
  double initialization_time_s_ = 3.0;
  double arm_timeout_s_ = 8.0;
  double arm_retry_interval_s_ = 1.0;
  double takeoff_height_m_ = 1.2;
  double takeoff_tolerance_m_ = 0.18;
  double estimator_timeout_s_ = 0.5;
  double estimator_failure_grace_s_ = 0.8;

  std::vector<double> mission_waypoints_flat_;
  std::vector<std::string> mission_waypoint_names_;
  std::vector<VelocityAPFController::Waypoint> mission_waypoints_;

  double landing_x_ = 12.0;
  double landing_y_ = 0.0;
  double landing_z_ = -1.2;
  double landing_xy_tolerance_m_ = 0.25;
  double landing_approach_timeout_s_ = 20.0;
  double final_land_height_m_ = 0.35;
  double precision_descent_start_height_m_ = 1.0;
  double descent_speed_mps_ = 0.2;
  double align_tolerance_m_ = 0.12;
  double align_stable_time_s_ = 1.0;
  double max_align_velocity_mps_ = 0.25;
  double align_kp_ = 0.45;

  std::string laser_topic_;
  std::string tag_topic_;
  int landing_tag_id_ = 0;
  double tag_timeout_s_ = 1.0;
  double tag_search_timeout_s_ = 8.0;
  int tag_forward_axis_ = 0;
  int tag_right_axis_ = 1;
  double tag_forward_sign_ = 1.0;
  double tag_right_sign_ = 1.0;

  std::string circle_center_topic_;
  std::string circle_distance_topic_;
  std::string circle_valid_topic_;
  double circle_timeout_s_ = 1.0;
  double circle_search_timeout_s_ = 8.0;
  double circle_image_width_ = 1280.0;
  double circle_image_height_ = 720.0;
  double circle_focal_px_ = 875.0;
  double circle_forward_pixel_sign_ = -1.0;
  double circle_right_pixel_sign_ = 1.0;

  std::string yolo_name_topic_;
  std::string yolo_confidence_topic_;
  std::string latest_yolo_name_;
  float latest_yolo_confidence_ = 0.0f;

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  LandingTarget latest_tag_;
  geometry_msgs::msg::PointStamped latest_circle_center_;
  rclcpp::Time circle_stamp_;
  double latest_circle_distance_m_ = 0.0;
  bool circle_valid_ = false;
  px4_msgs::msg::EstimatorStatusFlags estimator_flags_;
  rclcpp::Time estimator_flags_stamp_;
  rclcpp::Time estimator_unhealthy_since_;
  rclcpp::Time estimator_healthy_since_;
  bool has_estimator_flags_ = false;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tag_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr circle_center_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr circle_distance_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr circle_valid_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_name_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yolo_confidence_sub_;
  rclcpp::Subscription<px4_msgs::msg::EstimatorStatusFlags>::SharedPtr
      estimator_flags_sub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleCourseMission>());
  rclcpp::shutdown();
  return 0;
}
