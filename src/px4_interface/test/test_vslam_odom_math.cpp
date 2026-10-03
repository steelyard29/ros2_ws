#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>

#include "px4_interface/vslam_odom_math.hpp"

namespace math = px4_interface::vslam_odom_math;

TEST(VslamOdomMath, InvalidQuaternionFallsBackToIdentity) {
  const Eigen::Quaterniond q = math::normalizedQuaternion(0.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(q.w(), 1.0, 1e-12);
  EXPECT_NEAR(q.vec().norm(), 0.0, 1e-12);
}

TEST(VslamOdomMath, InverseYawDoesNotTiltVerticalAxis) {
  const Eigen::Quaterniond roll_pitch_yaw =
      Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitX());
  const Eigen::Vector3d vertical =
      math::inverseYawQuaternion(roll_pitch_yaw) * Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(vertical.x(), 0.0, 1e-12);
  EXPECT_NEAR(vertical.y(), 0.0, 1e-12);
  EXPECT_NEAR(vertical.z(), 1.0, 1e-12);
}

TEST(VslamOdomMath, AngularDistanceIgnoresQuaternionSign) {
  const Eigen::Quaterniond q(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond negative(-q.w(), -q.x(), -q.y(), -q.z());
  EXPECT_NEAR(math::quaternionAngularDistance(q, negative), 0.0, 1e-12);
}

TEST(VslamOdomMath, AngularDistanceDetectsYawStep) {
  const double pi = std::acos(-1.0);
  const Eigen::Quaterniond first = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond second(
      Eigen::AngleAxisd(pi / 3.0, Eigen::Vector3d::UnitZ()));
  EXPECT_NEAR(math::quaternionAngularDistance(first, second), pi / 3.0, 1e-12);
}

TEST(VslamOdomMath, FluVelocityConvertsToBodyFrd) {
  const Eigen::Vector3d velocity_flu(1.0, 2.0, 3.0);
  const Eigen::Vector3d velocity_frd = math::fluToFrd(velocity_flu);
  EXPECT_DOUBLE_EQ(velocity_frd.x(), 1.0);
  EXPECT_DOUBLE_EQ(velocity_frd.y(), -2.0);
  EXPECT_DOUBLE_EQ(velocity_frd.z(), -3.0);
}

TEST(VslamOdomMath, OriginYawAlignMatchesPx4NedYaw) {
  const double pi = std::acos(-1.0);
  const double px4_yaw_ned = pi / 4.0;
  // Pure yaw: ENU ψ maps exactly to NED -ψ under R_ned_from_enu.
  const Eigen::Quaterniond vslam_enu(
      Eigen::AngleAxisd(0.9, Eigen::Vector3d::UnitZ()));

  const Eigen::Quaterniond origin_align =
      math::originYawAlignToPx4(vslam_enu, px4_yaw_ned);
  const Eigen::Quaterniond q_enu = origin_align * vslam_enu;

  const Eigen::Matrix3d R_ned_from_enu =
      (Eigen::Matrix3d() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished();
  const Eigen::Quaterniond q_ned(R_ned_from_enu * q_enu.toRotationMatrix() *
                                 R_ned_from_enu.transpose());

  EXPECT_NEAR(math::wrapAnglePi(math::yawFromQuaternion(q_ned) - px4_yaw_ned),
              0.0, 1e-9);
}

TEST(VslamOdomMath, OriginYawAlignWithTiltStaysNearPx4Yaw) {
  const double pi = std::acos(-1.0);
  const double px4_yaw_ned = -pi / 6.0;
  const Eigen::Quaterniond vslam_enu(
      Eigen::AngleAxisd(0.9, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(-0.05, Eigen::Vector3d::UnitX()));

  const Eigen::Quaterniond q_enu =
      math::originYawAlignToPx4(vslam_enu, px4_yaw_ned) * vslam_enu;
  const Eigen::Matrix3d R_ned_from_enu =
      (Eigen::Matrix3d() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished();
  const Eigen::Quaterniond q_ned(R_ned_from_enu * q_enu.toRotationMatrix() *
                                 R_ned_from_enu.transpose());

  // Yaw-only rebase leaves a small Euler coupling when roll/pitch ≠ 0.
  EXPECT_NEAR(math::wrapAnglePi(math::yawFromQuaternion(q_ned) - px4_yaw_ned),
              0.0, 0.02);
}

TEST(VslamOdomMath, OriginYawAlignZeroPx4MatchesYawZero) {
  const Eigen::Quaterniond vslam_enu(
      Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond aligned =
      math::originYawAlignToPx4(vslam_enu, 0.0) * vslam_enu;
  EXPECT_NEAR(math::yawFromQuaternion(aligned), 0.0, 1e-12);
}
