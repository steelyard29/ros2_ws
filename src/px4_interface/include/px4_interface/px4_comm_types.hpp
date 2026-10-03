// PX4_COMM_TYPES_HPP
#pragma once
#include <Eigen/Dense>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>

namespace px4Position {
// 坐标系定义
/**
 *@brief
 *NED坐标系是PX4数据的标准格式,室内飞行时以起飞时候的FRD建立NED系,室外以地理坐标建立NED系
 *@example PX4飞控的提供和需要的机身坐标数据
 */
struct FrameNED {};
/**
 *@brief ENU坐标系是ROS和大部分地理信息系统的标准格式,定义方式类似于NED系
 *@example SLAM系统计算的机身坐标
 */
struct FrameENU {};

/**
 *@brief 所有位置数据的一般格式
 */
template <typename Frame>
struct BasicPosition {
  bool valid = false;              // 位置信息是否有效
  Eigen::Vector3d translation;     // 位置 (m)
  Eigen::Quaterniond orientation;  // 姿态 (四元数)
  rclcpp::Time timestamp;          // 时间戳
  // 坐标系类型标签
  using FrameType = Frame;

  /**
   *@brief 默认构造函数
   * 调用的时候为invalid,时间戳为0
   */
  BasicPosition() = default;

  /**
   *@brief 全参数构造函数
   * @param t 位置 (m)
   * @param q 姿态 (四元数)
   * @param time 时间戳
   * 调用后为valid
   */
  explicit BasicPosition(const Eigen::Vector3d &t, const Eigen::Quaterniond &q,
                         const rclcpp::Time &time)
      : valid(true), translation(t), orientation(q), timestamp(time) {}

  /**
   * @brief 将四元数转换为欧拉角 (roll, pitch, yaw)
   * @warning 欧拉角存在万向锁问题,仅用于调试和显示,不要用于控制计算
   */
  Eigen::Vector3d toEuler() const {
    return orientation.toRotationMatrix().eulerAngles(0, 1, 2);
  }
  /**
   *@brief 生成字符串表示,用于调试
   */
  std::string toString() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "Position(valid=" << valid << ", t=[" << translation.x() << ", "
       << translation.y() << ", " << translation.z() << "]"
       << ", q(w,x,y,z)=[" << orientation.w() << ", " << orientation.x() << ", "
       << orientation.y() << ", " << orientation.z() << "]"
       << ", time=" << timestamp.seconds() << "." << std::setw(9)
       << std::setfill('0') << (timestamp.nanoseconds() % 1000000000ULL) << ")";
    return ss.str();
  }
};

// 具体坐标系类型
using PositionNED = BasicPosition<FrameNED>;
using PositionENU = BasicPosition<FrameENU>;

// ENU <-> NED 变换矩阵
const Eigen::Matrix3d R_ned_from_enu =
    (Eigen::Matrix3d() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished();

// 坐标系转换函数
/**
 * @brief 将ENU坐标系转换为NED坐标系
 */
[[nodiscard]] inline PositionNED enuToNed(const PositionENU &pos_enu) {
  if (!pos_enu.valid) {
    return PositionNED();  // 返回无效位置
  }
  Eigen::Vector3d t_ned = R_ned_from_enu * pos_enu.translation;
  Eigen::Quaterniond q_ned = Eigen::Quaterniond(
      R_ned_from_enu * pos_enu.orientation.toRotationMatrix() *
      R_ned_from_enu.transpose());
  return PositionNED(t_ned, q_ned, pos_enu.timestamp);
}

/**
 *@brief 将NED坐标系转换为ENU坐标系
 */
[[nodiscard]] inline PositionENU nedToEnu(const PositionNED &pos_ned) {
  if (!pos_ned.valid) {
    return PositionENU();  // 返回无效位置
  }
  Eigen::Vector3d t_enu = R_ned_from_enu.transpose() * pos_ned.translation;
  Eigen::Quaterniond q_enu = Eigen::Quaterniond(
      R_ned_from_enu.transpose() * pos_ned.orientation.toRotationMatrix() *
      R_ned_from_enu);
  return PositionENU(t_enu, q_enu, pos_ned.timestamp);
}
}  // namespace px4Position

namespace px4Status {
/**
 *@brief 需要持续更新的PX4飞控状态结构
 */
struct VehicleStatus {
  bool valid = false;                   // 数据是否有效
  rclcpp::Time latest_timestamp;        // 收到数据的时间戳
  rclcpp::Time msg_timestamp;           // 消息自带的时间戳
  uint8_t arming_state = 0;             // 解锁状态, 1: DISARMED, 2: ARMED
  uint8_t nav_state = 0;                // 导航状态, 见PX4文档
  bool failsafe = false;                // 是否处于故障保护状态
  bool pre_flight_checks_pass = false;  // 飞前检查是否通过
};

/**
 * @brief 实用的电池状态缓存，包含了做决策所需的核心信息
 */
struct BatteryStatus {
  bool valid = false;          // 数据是否有效
  rclcpp::Time timestamp;      // 受到数据的时间戳
  rclcpp::Time msg_timestamp;  // 消息自带的时间戳
  float voltage_v = 0.0f;      // [V] 电池电压
  float current_a = 0.0f;      // [A] 当前电流，可以判断负载情况
  float remaining = 0.0f;  // [0.0 to 1.0] 剩余电量百分比，重要指标
  uint8_t warning = 0;  // 警告状态 (来自PX4的WARNING_NONE, LOW, CRITICAL等)
};

/**
 *@brief 车辆位姿的简化表示
 */
struct VehiclePose {
  bool valid = false;             // 数据是否有效
  rclcpp::Time latest_timestamp;  // 收到数据的时间戳
  rclcpp::Time msg_timestamp;     // 消息自带的时间戳
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // 位置 (m)
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  // 速度 (m/s)
  Eigen::Quaterniond orientation =
      Eigen::Quaterniond::Identity();  // 姿态 (四元数)
};
}  // namespace px4Status

namespace px4GatewayTypes {
struct setpoint {
  float position[3];  // 位置 (x, y, z) 米
  float velocity[3];  // 速度 (vx, vy, vz) 米/秒
  float yaw;          // 偏航角 (弧度)
};
}  // namespace px4GatewayTypes

namespace px4Enum {
// PX4的导航状态枚举，参考px4_msgs/msg/vehicle_status.h
/**
 *@brief PX4飞控的导航状态常量
 */
enum class NavState : uint16_t {
  MANUAL = 0,        // Manual mode
  ALTCTL = 1,        // Altitude control mode
  POSCTL = 2,        // Position control mode
  AUTO_MISSION = 3,  // Auto mission mode
  AUTO_LOITER = 4,   // Auto loiter mode
  AUTO_RTL = 5,      // Auto return to launch mode
  POSITION_SLOW = 6,
  FREE5 = 7,
  FREE4 = 8,
  FREE3 = 9,
  ACRO = 10,  // Acro mode
  FREE2 = 11,
  DESCEND = 12,      // Descend mode (no position control)
  TERMINATION = 13,  // Termination mode
  OFFBOARD = 14,
  STAB = 15,  // Stabilized mode
  FREE1 = 16,
  AUTO_TAKEOFF = 17,        // Takeoff
  AUTO_LAND = 18,           // Land
  AUTO_FOLLOW_TARGET = 19,  // Auto Follow
  AUTO_PRECLAND = 20,       // Precision land with landing target
  ORBIT = 21,               // Orbit in a circle
  AUTO_VTOL_TAKEOFF = 22,   // Takeoff
  EXTERNAL1 = 23,
  EXTERNAL2 = 24,
  EXTERNAL3 = 25,
  EXTERNAL4 = 26,
  EXTERNAL5 = 27,
  EXTERNAL6 = 28,
  EXTERNAL7 = 29,
  EXTERNAL8 = 30,
  MAX = 31
};

// PX4 vehicle commands
enum class VehicleCommand : uint32_t {
  VEHICLE_CMD_NAV_WAYPOINT = 16,
  VEHICLE_CMD_NAV_LOITER_UNLIM = 17,
  VEHICLE_CMD_NAV_LOITER_TURNS = 18,
  VEHICLE_CMD_NAV_LOITER_TIME = 19,
  VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20,
  VEHICLE_CMD_NAV_LAND = 21,
  VEHICLE_CMD_NAV_TAKEOFF = 22,
  VEHICLE_CMD_NAV_PRECLAND = 23,
  VEHICLE_CMD_DO_ORBIT = 34,
  VEHICLE_CMD_DO_FIGUREEIGHT = 35,
  VEHICLE_CMD_NAV_ROI = 80,
  VEHICLE_CMD_NAV_PATHPLANNING = 81,
  VEHICLE_CMD_NAV_VTOL_TAKEOFF = 84,
  VEHICLE_CMD_NAV_VTOL_LAND = 85,
  VEHICLE_CMD_NAV_GUIDED_LIMITS = 90,
  VEHICLE_CMD_NAV_GUIDED_MASTER = 91,
  VEHICLE_CMD_NAV_DELAY = 93,
  VEHICLE_CMD_NAV_LAST = 95,
  VEHICLE_CMD_CONDITION_DELAY = 112,
  VEHICLE_CMD_CONDITION_CHANGE_ALT = 113,
  VEHICLE_CMD_CONDITION_DISTANCE = 114,
  VEHICLE_CMD_CONDITION_YAW = 115,
  VEHICLE_CMD_CONDITION_LAST = 159,
  VEHICLE_CMD_CONDITION_GATE = 4501,
  VEHICLE_CMD_DO_SET_MODE = 176,
  VEHICLE_CMD_DO_JUMP = 177,
  VEHICLE_CMD_DO_CHANGE_SPEED = 178,
  VEHICLE_CMD_DO_SET_HOME = 179,
  VEHICLE_CMD_DO_SET_PARAMETER = 180,
  VEHICLE_CMD_DO_SET_RELAY = 181,
  VEHICLE_CMD_DO_REPEAT_RELAY = 182,
  VEHICLE_CMD_DO_REPEAT_SERVO = 184,
  VEHICLE_CMD_DO_FLIGHTTERMINATION = 185,
  VEHICLE_CMD_DO_CHANGE_ALTITUDE = 186,
  VEHICLE_CMD_DO_SET_ACTUATOR = 187,
  VEHICLE_CMD_DO_LAND_START = 189,
  VEHICLE_CMD_DO_GO_AROUND = 191,
  VEHICLE_CMD_DO_REPOSITION = 192,
  VEHICLE_CMD_DO_PAUSE_CONTINUE = 193,
  VEHICLE_CMD_DO_SET_ROI_LOCATION = 195,
  VEHICLE_CMD_DO_SET_ROI_WPNEXT_OFFSET = 196,
  VEHICLE_CMD_DO_SET_ROI_NONE = 197,
  VEHICLE_CMD_DO_CONTROL_VIDEO = 200,
  VEHICLE_CMD_DO_SET_ROI = 201,
  VEHICLE_CMD_DO_DIGICAM_CONTROL = 203,
  VEHICLE_CMD_DO_MOUNT_CONFIGURE = 204,
  VEHICLE_CMD_DO_MOUNT_CONTROL = 205,
  VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST = 206,
  VEHICLE_CMD_DO_FENCE_ENABLE = 207,
  VEHICLE_CMD_DO_PARACHUTE = 208,
  VEHICLE_CMD_DO_MOTOR_TEST = 209,
  VEHICLE_CMD_DO_INVERTED_FLIGHT = 210,
  VEHICLE_CMD_DO_GRIPPER = 211,
  VEHICLE_CMD_DO_SET_CAM_TRIGG_INTERVAL = 214,
  VEHICLE_CMD_DO_MOUNT_CONTROL_QUAT = 220,
  VEHICLE_CMD_DO_GUIDED_MASTER = 221,
  VEHICLE_CMD_DO_GUIDED_LIMITS = 222,
  VEHICLE_CMD_DO_LAST = 240,
  VEHICLE_CMD_PREFLIGHT_CALIBRATION = 241,
  VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242,
  VEHICLE_CMD_PREFLIGHT_UAVCAN = 243,
  VEHICLE_CMD_PREFLIGHT_STORAGE = 245,
  VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246,
  VEHICLE_CMD_OBLIQUE_SURVEY = 260,
  VEHICLE_CMD_DO_SET_STANDARD_MODE = 262,
  VEHICLE_CMD_GIMBAL_DEVICE_INFORMATION = 283,
  VEHICLE_CMD_MISSION_START = 300,
  VEHICLE_CMD_ACTUATOR_TEST = 310,
  VEHICLE_CMD_CONFIGURE_ACTUATOR = 311,
  VEHICLE_CMD_COMPONENT_ARM_DISARM = 400,
  VEHICLE_CMD_RUN_PREARM_CHECKS = 401,
  VEHICLE_CMD_INJECT_FAILURE = 420,
  VEHICLE_CMD_START_RX_PAIR = 500,
  VEHICLE_CMD_REQUEST_MESSAGE = 512,
  VEHICLE_CMD_REQUEST_CAMERA_INFORMATION = 521,
  VEHICLE_CMD_SET_CAMERA_MODE = 530,
  VEHICLE_CMD_SET_CAMERA_ZOOM = 531,
  VEHICLE_CMD_SET_CAMERA_FOCUS = 532,
  VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000,
  VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE = 1001,
  VEHICLE_CMD_IMAGE_START_CAPTURE = 2000,
  VEHICLE_CMD_DO_TRIGGER_CONTROL = 2003,
  VEHICLE_CMD_VIDEO_START_CAPTURE = 2500,
  VEHICLE_CMD_VIDEO_STOP_CAPTURE = 2501,
  VEHICLE_CMD_LOGGING_START = 2510,
  VEHICLE_CMD_LOGGING_STOP = 2511,
  VEHICLE_CMD_CONTROL_HIGH_LATENCY = 2600,
  VEHICLE_CMD_DO_VTOL_TRANSITION = 3000,
  VEHICLE_CMD_ARM_AUTHORIZATION_REQUEST = 3001,
  VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY = 30001,
  VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY = 30002,
  VEHICLE_CMD_FIXED_MAG_CAL_YAW = 42006,
  VEHICLE_CMD_DO_WINCH = 42600,
  VEHICLE_CMD_EXTERNAL_POSITION_ESTIMATE = 43003,
  VEHICLE_CMD_EXTERNAL_WIND_ESTIMATE = 43004,
  VEHICLE_CMD_PX4_INTERNAL_START = 65537,
  VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN = 100000,
  VEHICLE_CMD_SET_NAV_STATE = 100001
};
}  // namespace px4Enum
