#pragma once
#include <builtin_interfaces/msg/time.hpp>

#include "px4_comm_types.hpp"
#include "px4_interface/msg/battery_status.hpp"
#include "px4_interface/msg/pose_ned.hpp"
#include "px4_interface/msg/vehicle_status.hpp"

namespace {
inline builtin_interfaces::msg::Time toRosTime(const rclcpp::Time &time) {
  builtin_interfaces::msg::Time ros_time;
  const auto nanoseconds = time.nanoseconds();
  ros_time.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  ros_time.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return ros_time;
}
}  // namespace

namespace MsgConverters {

// 将缓存的 VehicleStatus 转换为 ROS 消息
inline px4_interface::msg::VehicleStatus convert(
    const px4Status::VehicleStatus &status) {
  px4_interface::msg::VehicleStatus msg;
  msg.valid = status.valid;
  msg.timestamp = toRosTime(status.msg_timestamp);
  msg.arming_state = status.arming_state;
  msg.nav_state = status.nav_state;
  msg.failsafe = status.failsafe;
  msg.pre_flight_checks_pass = status.pre_flight_checks_pass;
  return msg;
}
// 将缓存的 VehiclePose 转换为 ROS 消息
inline px4_interface::msg::PoseNED convert(const px4Status::VehiclePose &pose) {
  px4_interface::msg::PoseNED msg;
  msg.valid = pose.valid;
  msg.translation = {pose.position.x(), pose.position.y(), pose.position.z()};
  msg.velocity = {pose.velocity.x(), pose.velocity.y(), pose.velocity.z()};
  msg.orientation = {pose.orientation.w(), pose.orientation.x(),
                     pose.orientation.y(), pose.orientation.z()};
  msg.timestamp = toRosTime(pose.msg_timestamp);
  return msg;
}
// 将缓存的 BatteryStatus 转换为 ROS 消息
inline px4_interface::msg::BatteryStatus convert(
    const px4Status::BatteryStatus &battery) {
  px4_interface::msg::BatteryStatus msg;
  msg.valid = battery.valid;
  msg.timestamp = toRosTime(battery.msg_timestamp);
  msg.voltage_v = battery.voltage_v;
  msg.current_a = battery.current_a;
  msg.remaining = battery.remaining;
  msg.warning = battery.warning;
  return msg;
}
}  // namespace MsgConverters