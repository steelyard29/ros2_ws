#include "px4_interface/vslam_odom_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "px4_interface/vslam_odom_math.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

double stampSeconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1e-9;
}

Eigen::Vector3d odomPosition(const nav_msgs::msg::Odometry& odom) {
  return {odom.pose.pose.position.x, odom.pose.pose.position.y,
          odom.pose.pose.position.z};
}

Eigen::Quaterniond odomOrientation(const nav_msgs::msg::Odometry& odom) {
  return px4_interface::vslam_odom_math::normalizedQuaternion(
      odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
}

double yawFromPx4Attitude(const px4_msgs::msg::VehicleAttitude& msg) {
  return px4_interface::vslam_odom_math::yawFromQuaternion(
      px4_interface::vslam_odom_math::normalizedQuaternion(
          msg.q[0], msg.q[1], msg.q[2], msg.q[3]));
}

}  // namespace

VslamOdomBridge::VslamOdomBridge(const rclcpp::NodeOptions& options)
    : Node("vslam_odom_bridge", options) {
  this->declare_parameter<std::string>("vslam_odom_topic",
                                       "/visual_slam/tracking/odometry");
  this->declare_parameter<std::string>("vehicle_status_topic",
                                       "/fmu/out/vehicle_status_v1");
  this->declare_parameter<std::string>("vehicle_attitude_topic",
                                       "/fmu/out/vehicle_attitude");
  this->declare_parameter<std::string>("px4_odom_topic",
                                       "/fmu/in/vehicle_visual_odometry");
  this->declare_parameter<int>("publish_rate_hz", 30);
  this->declare_parameter<double>("position_variance_xy", 0.01);
  this->declare_parameter<double>("position_variance_z", 0.04);
  this->declare_parameter<double>("orientation_variance", 0.01);
  this->declare_parameter<double>("orientation_variance_yaw", 0.06);
  this->declare_parameter<double>("velocity_variance", 0.04);
  this->declare_parameter<bool>("use_input_covariance", true);
  this->declare_parameter<bool>("use_range_vertical_velocity", true);
  // EKF2_EV_CTRL=15 fuses full EV position (XY+Z).  Bridge publishes
  // TFmini dist_bottom as the EV Z position, so EKF receives matching
  // height data from both the EV and rangefinder channels.
  // Set flatten_ev_vertical_position=true for horizontal-only EV (EV_CTRL=1/5).
  this->declare_parameter<bool>("flatten_ev_vertical_position", false);
  this->declare_parameter<double>("range_velocity_filter_alpha", 0.2);
  // Disabled by default: hand-lift 20260808 showed inflate→EKF XY worse (0.17→0.57 m).
  // Set >0 only after a gated climb test proves benefit for powered flight.
  this->declare_parameter<double>("climb_position_variance_xy", 0.0);
  this->declare_parameter<double>("climb_vertical_speed_mps", 0.08);
  this->declare_parameter<bool>("publish_local_ned_velocity", false);
  this->declare_parameter<bool>("rebase_to_initial_pose", true);
  this->declare_parameter<bool>("align_yaw_to_px4", true);
  // Indoor default FRD: horizontal EV position without EV yaw / mag yaw_align.
  this->declare_parameter<std::string>("pose_frame", "frd");
  this->declare_parameter<int>("stale_timeout_ms", 200);
  this->declare_parameter<int>("stabilization_samples", 10);
  this->declare_parameter<int>("min_publish_quality", 50);
  this->declare_parameter<double>("jump_threshold_m", 1.0);
  this->declare_parameter<double>("jump_threshold_deg", 30.0);
  this->declare_parameter<double>("max_sample_gap_s", 2.0);

  publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_int();
  position_variance_xy_ = static_cast<float>(
      this->get_parameter("position_variance_xy").as_double());
  position_variance_z_ = static_cast<float>(
      this->get_parameter("position_variance_z").as_double());
  orientation_variance_rp_ = static_cast<float>(
      this->get_parameter("orientation_variance").as_double());
  orientation_variance_yaw_ = static_cast<float>(
      this->get_parameter("orientation_variance_yaw").as_double());
  velocity_variance_ =
      static_cast<float>(this->get_parameter("velocity_variance").as_double());
  use_input_covariance_ =
      this->get_parameter("use_input_covariance").as_bool();
  use_range_vertical_velocity_ =
      this->get_parameter("use_range_vertical_velocity").as_bool();
  flatten_ev_vertical_position_ =
      this->get_parameter("flatten_ev_vertical_position").as_bool();
  range_velocity_filter_alpha_ =
      std::clamp(this->get_parameter("range_velocity_filter_alpha").as_double(),
                 0.01, 1.0);
  climb_position_variance_xy_ = static_cast<float>(
      this->get_parameter("climb_position_variance_xy").as_double());
  climb_vertical_speed_mps_ =
      this->get_parameter("climb_vertical_speed_mps").as_double();
  publish_local_ned_velocity_ =
      this->get_parameter("publish_local_ned_velocity").as_bool();
  rebase_to_initial_pose_ =
      this->get_parameter("rebase_to_initial_pose").as_bool();
  align_yaw_to_px4_ = this->get_parameter("align_yaw_to_px4").as_bool();
  {
    const std::string pose_frame =
        this->get_parameter("pose_frame").as_string();
    if (pose_frame == "ned" || pose_frame == "NED") {
      pose_frame_ = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    } else if (pose_frame == "frd" || pose_frame == "FRD") {
      pose_frame_ = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Unknown pose_frame='%s', using FRD (indoor EV_CTRL=1)",
                  pose_frame.c_str());
      pose_frame_ = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
    }
  }
  stale_timeout_ms_ = this->get_parameter("stale_timeout_ms").as_int();
  stabilization_samples_ =
      this->get_parameter("stabilization_samples").as_int();
  min_publish_quality_ = this->get_parameter("min_publish_quality").as_int();
  jump_threshold_m_ = this->get_parameter("jump_threshold_m").as_double();
  jump_threshold_rad_ =
      this->get_parameter("jump_threshold_deg").as_double() * kPi / 180.0;
  max_sample_gap_s_ = this->get_parameter("max_sample_gap_s").as_double();

  publish_rate_hz_ = std::max(1, publish_rate_hz_);
  stale_timeout_ms_ = std::max(1, stale_timeout_ms_);
  stabilization_samples_ = std::max(1, stabilization_samples_);
  min_publish_quality_ = std::clamp(min_publish_quality_, 0, 100);

  const std::string vslam_topic =
      this->get_parameter("vslam_odom_topic").as_string();
  const std::string vehicle_status_topic =
      this->get_parameter("vehicle_status_topic").as_string();
  const std::string vehicle_attitude_topic =
      this->get_parameter("vehicle_attitude_topic").as_string();
  const std::string px4_topic =
      this->get_parameter("px4_odom_topic").as_string();

  RCLCPP_INFO(this->get_logger(),
              "VslamOdomBridge: '%s' -> '%s' at %d Hz, "
              "stable_samples=%d, stale=%d ms, jump=%.2f m/%.1f deg, "
              "align_yaw_to_px4=%s, pose_frame=%s, yaw_var=%.3f, min_quality=%d",
              vslam_topic.c_str(), px4_topic.c_str(), publish_rate_hz_,
              stabilization_samples_, stale_timeout_ms_, jump_threshold_m_,
              jump_threshold_rad_ * 180.0 / kPi,
              align_yaw_to_px4_ ? "true" : "false",
              pose_frame_ == px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED
                  ? "NED"
                  : "FRD",
              orientation_variance_yaw_, min_publish_quality_);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      vslam_topic, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odomCallback(msg);
      });
  vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      vehicle_status_topic, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        vehicleStatusCallback(msg);
      });
  vehicle_attitude_sub_ =
      this->create_subscription<px4_msgs::msg::VehicleAttitude>(
          vehicle_attitude_topic, rclcpp::SensorDataQoS(),
          [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
            vehicleAttitudeCallback(msg);
          });

  timesync_sub_ = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
      "/fmu/out/timesync_status", rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::TimesyncStatus::SharedPtr msg) {
        timesyncCallback(msg);
      });

  local_position_sub_ =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
          [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
            localPositionCallback(msg);
          });

  px4_odom_pub_ =
      this->create_publisher<px4_msgs::msg::VehicleOdometry>(px4_topic, 10);
  publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / publish_rate_hz_),
      [this]() { publishPx4Odometry(); });
}

void VslamOdomBridge::vehicleStatusCallback(
    const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  const bool was_armed = vehicle_armed_;
  vehicle_armed_ =
      msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  if (was_armed && !vehicle_armed_ && reset_blocked_while_armed_) {
    reset_blocked_while_armed_ = false;
    resetTrackingLocked("vehicle disarmed after an in-flight VSLAM reset");
  }
}

void VslamOdomBridge::vehicleAttitudeCallback(
    const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  px4_yaw_ned_ = yawFromPx4Attitude(*msg);
  has_px4_attitude_ = true;
}

void VslamOdomBridge::timesyncCallback(
    const px4_msgs::msg::TimesyncStatus::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  // estimated_offset = PX4_time - ROS_time (both in μs)
  timesync_offset_us_ = msg->estimated_offset;
  timesync_valid_ = (msg->source_protocol ==
                     px4_msgs::msg::TimesyncStatus::SOURCE_PROTOCOL_DDS) ||
                    (msg->observed_offset != 0);
}

void VslamOdomBridge::localPositionCallback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  const auto now = std::chrono::steady_clock::now();
  dist_bottom_ = static_cast<double>(msg->dist_bottom);
  has_dist_bottom_ = msg->dist_bottom_valid || (msg->dist_bottom > 0.01f);
  if (has_dist_bottom_) {
    if (has_previous_dist_bottom_) {
      const double dt =
          std::chrono::duration<double>(now - previous_dist_bottom_at_).count();
      if (dt >= 0.005 && dt <= 0.5) {
        // FRD down velocity: increasing ground distance means climbing, hence
        // negative z velocity. Low-pass the differentiated range measurement.
        const double raw_velocity =
            -(dist_bottom_ - previous_dist_bottom_) / dt;
        range_vertical_velocity_frd_ =
            has_range_vertical_velocity_
                ? range_velocity_filter_alpha_ * raw_velocity +
                      (1.0 - range_velocity_filter_alpha_) *
                          range_vertical_velocity_frd_
                : raw_velocity;
        has_range_vertical_velocity_ = true;
      }
    }
    previous_dist_bottom_ = dist_bottom_;
    previous_dist_bottom_at_ = now;
    has_previous_dist_bottom_ = true;
    last_valid_dist_bottom_ = dist_bottom_;
    has_last_valid_dist_bottom_ = true;
  }
}

bool VslamOdomBridge::poseJumpDetectedLocked(
    const nav_msgs::msg::Odometry& odom, std::string& reason) const {
  if (!previous_raw_odom_msg_) {
    return false;
  }

  const double previous_stamp =
      stampSeconds(previous_raw_odom_msg_->header.stamp);
  const double current_stamp = stampSeconds(odom.header.stamp);
  const double dt = current_stamp - previous_stamp;
  if (dt < 0.0) {
    reason = "timestamp moved backwards";
    return true;
  }
  if (dt > max_sample_gap_s_) {
    reason = "odometry sample gap exceeded limit";
    return true;
  }

  const double position_step =
      (odomPosition(odom) - odomPosition(*previous_raw_odom_msg_)).norm();
  if (position_step > jump_threshold_m_) {
    std::ostringstream stream;
    stream << "position jumped " << position_step << " m";
    reason = stream.str();
    return true;
  }

  const double attitude_step =
      px4_interface::vslam_odom_math::quaternionAngularDistance(
          odomOrientation(odom), odomOrientation(*previous_raw_odom_msg_));
  if (attitude_step > jump_threshold_rad_) {
    std::ostringstream stream;
    stream << "attitude jumped " << attitude_step * 180.0 / kPi << " deg";
    reason = stream.str();
    return true;
  }
  return false;
}

int8_t VslamOdomBridge::trackingQualityLocked(
    const nav_msgs::msg::Odometry& odom) const {
  if (!previous_raw_odom_msg_) {
    return 100;
  }

  const double previous_stamp =
      stampSeconds(previous_raw_odom_msg_->header.stamp);
  const double current_stamp = stampSeconds(odom.header.stamp);
  const double dt = current_stamp - previous_stamp;
  if (dt <= 0.0 || dt > max_sample_gap_s_ * 0.75) {
    return static_cast<int8_t>(std::max(0, min_publish_quality_ - 1));
  }

  const double position_step =
      (odomPosition(odom) - odomPosition(*previous_raw_odom_msg_)).norm();
  const double attitude_step =
      px4_interface::vslam_odom_math::quaternionAngularDistance(
          odomOrientation(odom), odomOrientation(*previous_raw_odom_msg_));

  // Soft gate: degrade quality before hard jump reset so EKF2_EV_QMIN can reject.
  if (position_step > 0.5 * jump_threshold_m_ ||
      attitude_step > 0.5 * jump_threshold_rad_) {
    return static_cast<int8_t>(std::max(0, min_publish_quality_ - 1));
  }
  if (position_step > 0.25 * jump_threshold_m_ ||
      attitude_step > 0.25 * jump_threshold_rad_) {
    return 70;
  }
  return 100;
}

void VslamOdomBridge::resetTrackingLocked(const char* reason) {
  has_data_ = false;
  has_origin_ = false;
  stable_sample_count_ = 0;
  previous_raw_odom_msg_.reset();
  last_odom_msg_.reset();
  ++reset_counter_;
  RCLCPP_WARN(this->get_logger(),
              "VSLAM frame reset (%s); waiting for %d stable samples, "
              "reset_counter=%u",
              reason, stabilization_samples_, reset_counter_);
}

void VslamOdomBridge::initializeOriginLocked(
    const nav_msgs::msg::Odometry& odom) {
  origin_position_enu_ = odomPosition(odom);
  dist_bottom_at_origin_ = dist_bottom_;
  const Eigen::Quaterniond vslam_q = odomOrientation(odom);
  if (align_yaw_to_px4_ && has_px4_attitude_) {
    origin_yaw_inverse_ =
        px4_interface::vslam_odom_math::originYawAlignToPx4(vslam_q,
                                                            px4_yaw_ned_);
    RCLCPP_INFO(this->get_logger(),
                "VSLAM origin accepted: ENU x=%.3f, y=%.3f, z=%.3f; "
                "yaw aligned to PX4 NED yaw=%.1f deg",
                origin_position_enu_.x(), origin_position_enu_.y(),
                origin_position_enu_.z(), px4_yaw_ned_ * 180.0 / kPi);
  } else {
    origin_yaw_inverse_ =
        px4_interface::vslam_odom_math::inverseYawQuaternion(vslam_q);
    if (align_yaw_to_px4_ && !has_px4_attitude_) {
      RCLCPP_WARN(this->get_logger(),
                  "VSLAM origin accepted without PX4 attitude; "
                  "falling back to yaw-zero alignment "
                  "(ENU x=%.3f, y=%.3f, z=%.3f)",
                  origin_position_enu_.x(), origin_position_enu_.y(),
                  origin_position_enu_.z());
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "VSLAM origin accepted after stabilization: ENU "
                  "x=%.3f, y=%.3f, z=%.3f (yaw-zero alignment)",
                  origin_position_enu_.x(), origin_position_enu_.y(),
                  origin_position_enu_.z());
    }
  }
  has_origin_ = true;
}

void VslamOdomBridge::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  const auto now = std::chrono::steady_clock::now();
  last_odom_received_at_ = now;

  // RTAB-Map's relay publishes at a fixed rate while its source TF may update
  // more slowly. It can therefore repeat the exact same stamped sample.
  // Repeated samples are not a frame reset and must not clear stabilization;
  // wait for the next distinct source sample instead.
  if (previous_raw_odom_msg_ &&
      msg->header.stamp.sec == previous_raw_odom_msg_->header.stamp.sec &&
      msg->header.stamp.nanosec ==
          previous_raw_odom_msg_->header.stamp.nanosec) {
    return;
  }

  std::string jump_reason;
  if (poseJumpDetectedLocked(*msg, jump_reason)) {
    if (vehicle_armed_) {
      reset_blocked_while_armed_ = true;
      has_data_ = false;
      stable_sample_count_ = 0;
      RCLCPP_ERROR(this->get_logger(),
                   "VSLAM reset while armed (%s); external vision output "
                   "blocked until disarm",
                   jump_reason.c_str());
    } else {
      resetTrackingLocked(jump_reason.c_str());
    }
  }

  // Quality must be scored against the previous sample before overwriting it.
  last_tracking_quality_ = trackingQualityLocked(*msg);
  previous_raw_odom_msg_ = msg;
  if (reset_blocked_while_armed_) {
    has_data_ = false;
    stable_sample_count_ = 0;
    return;
  }

  stable_sample_count_ =
      std::min(stable_sample_count_ + 1, stabilization_samples_);
  if (stable_sample_count_ < stabilization_samples_) {
    has_data_ = false;
    return;
  }

  // Never lock origin without PX4 yaw when alignment is requested: an early
  // yaw-zero origin rotates FRD EV axes vs local NED and injects climb XY slip.
  if (rebase_to_initial_pose_ && !has_origin_) {
    if (align_yaw_to_px4_ && !has_px4_attitude_) {
      has_data_ = false;
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Waiting for PX4 attitude before accepting VSLAM origin "
          "(align_yaw_to_px4=true)");
      return;
    }
    initializeOriginLocked(*msg);
  }
  last_odom_msg_ = msg;
  has_data_ = true;
}

void VslamOdomBridge::publishPx4Odometry() {
  px4_msgs::msg::VehicleOdometry px4_msg;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (!has_data_ || !last_odom_msg_ || reset_blocked_while_armed_) {
      return;
    }
    if (now - last_odom_received_at_ >
        std::chrono::milliseconds(stale_timeout_ms_)) {
      has_data_ = false;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "VSLAM odometry stale; PX4 output paused");
      return;
    }
    if (last_tracking_quality_ < min_publish_quality_) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "VSLAM tracking quality=%d < min=%d; PX4 EV output paused",
          last_tracking_quality_, min_publish_quality_);
      return;
    }
    px4_msg = convertToPx4(*last_odom_msg_, last_tracking_quality_);
  }
  px4_odom_pub_->publish(px4_msg);
}

px4_msgs::msg::VehicleOdometry VslamOdomBridge::convertToPx4(
    const nav_msgs::msg::Odometry& odom, int8_t quality) const {
  px4_msgs::msg::VehicleOdometry px4_msg;

  const uint64_t stamp_ns =
      static_cast<uint64_t>(odom.header.stamp.sec) * 1000000000ULL +
      static_cast<uint64_t>(odom.header.stamp.nanosec);
  const uint64_t ros_time_us = stamp_ns / 1000ULL;

  // Convert ROS system time (Unix epoch) to PX4 boot-relative time.
  // timesync_offset_us_ = PX4_time - ROS_time (typically a large negative).
  uint64_t px4_time_us = ros_time_us;
  if (timesync_valid_) {
    const int64_t corrected =
        static_cast<int64_t>(ros_time_us) + timesync_offset_us_;
    if (corrected > 0) {
      px4_time_us = static_cast<uint64_t>(corrected);
    }
  }
  px4_msg.timestamp = px4_time_us;
  px4_msg.timestamp_sample = px4_time_us;

  Eigen::Vector3d pos_enu = odomPosition(odom);
  Eigen::Quaterniond q_enu = odomOrientation(odom);
  if (rebase_to_initial_pose_) {
    pos_enu = origin_yaw_inverse_ * (pos_enu - origin_position_enu_);
    q_enu = origin_yaw_inverse_ * q_enu;
    q_enu.normalize();
  }

  const Eigen::Vector3d pos_ned = px4Position::R_ned_from_enu * pos_enu;
  px4_msg.position[0] = static_cast<float>(pos_ned.x());
  px4_msg.position[1] = static_cast<float>(pos_ned.y());
  // Z: use TFmini laser dist_bottom (always valid), fallback to last valid
  // dist_bottom rather than VSLAM Z, which drifts during climb.
  if (flatten_ev_vertical_position_ &&
      pose_frame_ == px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD) {
    // Horizontal-only EV contract: do not let TFmini/VIO height enter the
    // FRD position rotation path. EKF2_HGT_REF/RNG_CTRL fuses TFmini itself.
    px4_msg.position[2] = 0.f;
  } else if (has_dist_bottom_ && has_origin_) {
    // FRD z positive down; dist_bottom increases with altitude →
    // negative FRD z = above origin
    px4_msg.position[2] =
        static_cast<float>(-(dist_bottom_ - dist_bottom_at_origin_));
  } else if (has_last_valid_dist_bottom_ && has_origin_) {
    // Hold last valid laser Z when sensor glitches (vibration dropout)
    px4_msg.position[2] =
        static_cast<float>(-(last_valid_dist_bottom_ - dist_bottom_at_origin_));
  } else {
    px4_msg.position[2] = static_cast<float>(pos_ned.z());
  }

  const Eigen::Vector3d velocity_flu(odom.twist.twist.linear.x,
                                     odom.twist.twist.linear.y,
                                     odom.twist.twist.linear.z);
  const Eigen::Vector3d velocity_frd =
      px4_interface::vslam_odom_math::fluToFrd(velocity_flu);
  // Prefer local NED velocity: with EV position already fused, BODY_FRD start
  // uses a much tighter gate and often never engages cs_ev_vel.
  if (publish_local_ned_velocity_) {
    const Eigen::Vector3d velocity_enu = q_enu * velocity_flu;
    const Eigen::Vector3d velocity_ned =
        px4Position::R_ned_from_enu * velocity_enu;
    px4_msg.velocity[0] = static_cast<float>(velocity_ned.x());
    px4_msg.velocity[1] = static_cast<float>(velocity_ned.y());
    px4_msg.velocity[2] =
        use_range_vertical_velocity_ && has_range_vertical_velocity_
            ? static_cast<float>(range_vertical_velocity_frd_)
            : static_cast<float>(velocity_ned.z());
    px4_msg.velocity_frame =
        px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
  } else {
    px4_msg.velocity[0] = static_cast<float>(velocity_frd.x());
    px4_msg.velocity[1] = static_cast<float>(velocity_frd.y());
    px4_msg.velocity[2] =
        use_range_vertical_velocity_ && has_range_vertical_velocity_
            ? static_cast<float>(range_vertical_velocity_frd_)
            : static_cast<float>(velocity_frd.z());
    px4_msg.velocity_frame =
        px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
  }

  const px4Position::PositionENU pos_enu_obj(pos_enu, q_enu,
                                             rclcpp::Time(stamp_ns));
  const px4Position::PositionNED pos_ned_obj =
      px4Position::enuToNed(pos_enu_obj);
  px4_msg.q[0] = static_cast<float>(pos_ned_obj.orientation.w());
  px4_msg.q[1] = static_cast<float>(pos_ned_obj.orientation.x());
  px4_msg.q[2] = static_cast<float>(pos_ned_obj.orientation.y());
  px4_msg.q[3] = static_cast<float>(pos_ned_obj.orientation.z());

  const Eigen::Vector3d angular_velocity_frd =
      px4_interface::vslam_odom_math::fluToFrd(
          {odom.twist.twist.angular.x, odom.twist.twist.angular.y,
           odom.twist.twist.angular.z});
  px4_msg.angular_velocity[0] =
      static_cast<float>(angular_velocity_frd.x());
  px4_msg.angular_velocity[1] =
      static_cast<float>(angular_velocity_frd.y());
  px4_msg.angular_velocity[2] =
      static_cast<float>(angular_velocity_frd.z());

  // FRD: EKF fuses EV XY without requiring yaw_align (EKF2_EV_CTRL=1).
  // NED: requires mag or EV yaw (EKF2_EV_CTRL=9) for cs_ev_pos.
  px4_msg.pose_frame = pose_frame_;
  // Z variance: 0.02 = 2cm when using live laser, 0.05 when held, higher for VSLAM fallback
  float pvz = has_dist_bottom_ ? 0.02f
              : (has_last_valid_dist_bottom_ ? 0.05f : position_variance_z_);
  const auto covarianceFloor = [](double input, float floor) {
    return std::isfinite(input) && input > 0.0
               ? std::max(static_cast<float>(input), floor)
               : floor;
  };
  float pxy = use_input_covariance_
                  ? std::max(covarianceFloor(odom.pose.covariance[0],
                                             position_variance_xy_),
                             covarianceFloor(odom.pose.covariance[7],
                                             position_variance_xy_))
                  : position_variance_xy_;
  const double climb_speed =
      has_range_vertical_velocity_ ? std::abs(range_vertical_velocity_frd_)
      : use_range_vertical_velocity_
          ? 0.0
          : std::abs(static_cast<double>(px4_msg.velocity[2]));
  if (climb_position_variance_xy_ > 0.0f &&
      climb_speed >= climb_vertical_speed_mps_) {
    pxy = std::max(pxy, climb_position_variance_xy_);
  }
  const float vxy = use_input_covariance_
                        ? std::max(covarianceFloor(odom.twist.covariance[0],
                                                   velocity_variance_),
                                   covarianceFloor(odom.twist.covariance[7],
                                                   velocity_variance_))
                        : velocity_variance_;
  const float vrp = use_input_covariance_
                        ? std::max(covarianceFloor(odom.pose.covariance[21],
                                                   orientation_variance_rp_),
                                   covarianceFloor(odom.pose.covariance[28],
                                                   orientation_variance_rp_))
                        : orientation_variance_rp_;
  const float vyaw =
      use_input_covariance_
          ? covarianceFloor(odom.pose.covariance[35],
                            orientation_variance_yaw_)
          : orientation_variance_yaw_;
  px4_msg.position_variance = {pxy, pxy, pvz};
  px4_msg.orientation_variance = {vrp, vrp, vyaw};
  px4_msg.velocity_variance = {
      vxy, vxy, vxy};
  px4_msg.reset_counter = reset_counter_;
  px4_msg.quality = quality;
  return px4_msg;
}
