#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "px4_interface/px4_comm_types.hpp"

class VslamOdomBridge : public rclcpp::Node {
 public:
  explicit VslamOdomBridge(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void vehicleAttitudeCallback(
      const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
  void timesyncCallback(
      const px4_msgs::msg::TimesyncStatus::SharedPtr msg);
  void localPositionCallback(
      const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void publishPx4Odometry();
  px4_msgs::msg::VehicleOdometry convertToPx4(
      const nav_msgs::msg::Odometry& odom, int8_t quality) const;
  void resetTrackingLocked(const char* reason);
  void initializeOriginLocked(const nav_msgs::msg::Odometry& odom);
  bool poseJumpDetectedLocked(const nav_msgs::msg::Odometry& odom,
                              std::string& reason) const;
  int8_t trackingQualityLocked(const nav_msgs::msg::Odometry& odom) const;

  // 订阅 VSLAM 里程计
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
      vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr
      vehicle_attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr
      timesync_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  // 发布 PX4 VehicleOdometry
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_pub_;
  // 定时发布
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // 缓存最新里程计数据
  nav_msgs::msg::Odometry::SharedPtr last_odom_msg_;
  nav_msgs::msg::Odometry::SharedPtr previous_raw_odom_msg_;
  std::mutex data_mutex_;
  bool has_data_{false};
  bool vehicle_armed_{false};
  bool reset_blocked_while_armed_{false};
  bool has_px4_attitude_{false};
  double px4_yaw_ned_{0.0};
  int64_t timesync_offset_us_{0};
  bool timesync_valid_{false};
  int stable_sample_count_{0};
  int8_t last_tracking_quality_{100};
  uint8_t reset_counter_{0};
  std::chrono::steady_clock::time_point last_odom_received_at_{};

  // 参数
  int publish_rate_hz_{30};
  float position_variance_xy_{0.01f};
  float position_variance_z_{0.04f};
  float orientation_variance_rp_{0.01f};
  float orientation_variance_yaw_{0.06f};
  float velocity_variance_{0.04f};
  bool use_input_covariance_{true};
  bool use_range_vertical_velocity_{true};
  bool flatten_ev_vertical_position_{true};
  double range_velocity_filter_alpha_{0.2};
  // Inflate EV XY variance while climbing/descending so EKF trusts IMU more
  // during the D435i/cuVSLAM vertical-lift slip observed in ground tests.
  float climb_position_variance_xy_{0.0f};
  double climb_vertical_speed_mps_{0.08};
  bool publish_local_ned_velocity_{true};
  bool rebase_to_initial_pose_{true};
  bool align_yaw_to_px4_{true};
  // POSE_FRAME_FRD: EKF can fuse EV XY without yaw_align (keep EKF2_EV_CTRL=1).
  // POSE_FRAME_NED: requires cs_yaw_align (mag or EKF2_EV_CTRL=9).
  uint8_t pose_frame_{px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD};
  int stale_timeout_ms_{200};
  int stabilization_samples_{10};
  int min_publish_quality_{50};
  double jump_threshold_m_{1.0};
  double jump_threshold_rad_{0.5235987756};
  double max_sample_gap_s_{0.5};

  // VSLAM odometry is relative, but its reported origin may not be near zero
  // when the node starts after tracking is already active. Rebase before PX4.
  bool has_origin_{false};
  Eigen::Vector3d origin_position_enu_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond origin_yaw_inverse_{Eigen::Quaterniond::Identity()};

  // Laser altimeter (TFmini via PX4 dist_bottom) — replaces VSLAM Z
  double dist_bottom_{0.0};
  double dist_bottom_at_origin_{0.0};
  bool has_dist_bottom_{false};
  // Last valid dist_bottom: held when sensor glitches, avoids VSLAM-Z fallback
  double last_valid_dist_bottom_{0.0};
  bool has_last_valid_dist_bottom_{false};
  double previous_dist_bottom_{0.0};
  double range_vertical_velocity_frd_{0.0};
  bool has_previous_dist_bottom_{false};
  bool has_range_vertical_velocity_{false};
  std::chrono::steady_clock::time_point previous_dist_bottom_at_{};
};
