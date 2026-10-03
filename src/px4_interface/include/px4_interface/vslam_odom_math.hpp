#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace px4_interface::vslam_odom_math {

inline Eigen::Quaterniond normalizedQuaternion(double w, double x, double y,
                                               double z) {
  Eigen::Quaterniond q(w, x, y, z);
  if (!std::isfinite(q.w()) || !std::isfinite(q.x()) || !std::isfinite(q.y()) ||
      !std::isfinite(q.z()) || q.norm() < 1e-6) {
    return Eigen::Quaterniond::Identity();
  }
  q.normalize();
  return q;
}

inline double yawFromQuaternion(const Eigen::Quaterniond& q) {
  const double sin_yaw = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cos_yaw = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(sin_yaw, cos_yaw);
}

inline Eigen::Quaterniond yawQuaternion(double yaw_rad) {
  return Eigen::Quaterniond(
      Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
}

inline Eigen::Quaterniond inverseYawQuaternion(const Eigen::Quaterniond& q) {
  return yawQuaternion(-yawFromQuaternion(q));
}

// Pure ENU yaw ψ maps to NED yaw -ψ under R_ned_from_enu. Align so that after
// rebase+ENU→NED the visual NED yaw matches px4_yaw_ned at origin lock.
inline Eigen::Quaterniond originYawAlignToPx4(
    const Eigen::Quaterniond& vslam_enu_q, double px4_yaw_ned) {
  return yawQuaternion(-px4_yaw_ned) * inverseYawQuaternion(vslam_enu_q);
}

inline double wrapAnglePi(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

inline double quaternionAngularDistance(const Eigen::Quaterniond& lhs,
                                        const Eigen::Quaterniond& rhs) {
  const double dot = std::clamp(std::abs(lhs.dot(rhs)), 0.0, 1.0);
  return 2.0 * std::acos(dot);
}

// Isaac VSLAM odometry reports twist in child_frame_id=base_link (FLU).
// PX4 BODY_FRD keeps X and reverses the lateral and vertical axes.
inline Eigen::Vector3d fluToFrd(const Eigen::Vector3d& vector_flu) {
  return {vector_flu.x(), -vector_flu.y(), -vector_flu.z()};
}

}  // namespace px4_interface::vslam_odom_math
