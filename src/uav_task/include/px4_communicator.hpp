#pragma once

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <mutex>
#include <string>

class PX4Communicator {
public:
  // --- 公共数据结构 ---

  // 无人机位置信息
  struct Position {
    bool valid = false;
    double x = 0.0, y = 0.0, z = 0.0;
    float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
    rclcpp::Time timestamp;
  };

  // 无人机控制状态
  struct ControlStatus {
    bool valid = false;
    bool is_armed = false;
    bool is_offboard = false;
    rclcpp::Time timestamp;
  };

  // 命令执行结果
  struct CommandResult {
    bool accepted = false;
    std::string reason;
  };

  // 通信和安全状态机
  enum class CommState {
    NORMAL,                // 一切正常
    MAIN_TIMEOUT_HOLD,     // 主程序调用超时，进入悬停
    PX4_DATA_TIMEOUT_HOLD, // PX4数据流超时，进入悬停
    EMERGENCY_HOLD,        // 紧急悬停状态
    EMERGENCY_LAND         // 紧急悬停超时，强制降落
  };

  // --- 公共接口 ---

  /**
   * @brief 构造函数.
   * @param node 指向拥有此对象的主ROS节点的裸指针.
   */
  explicit PX4Communicator(rclcpp::Node *node);

  /**
   * @brief 统一的更新函数，必须在主循环中以固定频率调用.
   *        负责处理所有内部状态机和安全检查.
   */
  void update();

  /**
   * @brief 设置位置设定点 (只控制位置).
   * @param x, y, z 目标位置 (m)
   * @param yaw 目标偏航角 (rad)
   * @return 如果命令被接受则返回true, 否则返回false (例如在紧急状态下).
   */
  bool setPositionSetpoint(float x, float y, float z, float yaw);

  /**
   * @brief 设置速度设定点 (只控制速度).
   * @param vx, vy, vz 目标速度 (m/s)
   * @param yaw 目标偏航角 (rad)
   * @return 如果命令被接受则返回true, 否则返回false (例如在紧急状态下).
   */
  bool setVelocitySetpoint(float vx, float vy, float vz, float yaw);

  /**
   * @brief 设置位置和速度设定点 (同时控制位置和速度).
   * @param x, y, z 目标位置 (m)
   * @param vx, vy, vz 目标速度 (m/s)
   * @param yaw 目标偏航角 (rad)
   * @return 如果命令被接受则返回true, 否则返回false (例在紧急状态下).
   */
  bool setPositionAndVelocitySetpoint(float x, float y, float z, float vx,
                                      float vy, float vz, float yaw);

  // --- 发送一次性命令 ---
  CommandResult arm();
  CommandResult setOffboardMode();
  void disarm(); // 安全操作，通常无条件执行
  void land();   // 安全操作，通常无条件执行

  // --- 获取状态 (线程安全) ---
  Position getCurrentPosition() const;
  ControlStatus getControlStatus() const;
  CommState getCommunicationState() const;

private:
  // --- 私有辅助函数 ---
  void publishOffboardControlMode(bool position_control = true,
                                  bool velocity_control = true);
  void publishVehicleCommand(uint16_t command, float param1 = 0.0,
                             float param2 = 0.0);
  void publishEmergencyHold();
  void publishEmergencyLand();

  // --- ROS 2 回调函数 ---
  void vehicleLocalPositionCallback(
      const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void vehicleControlModeCallback(
      const px4_msgs::msg::VehicleControlMode::SharedPtr msg);

  // --- 成员变量 ---
  rclcpp::Node *node_; // 不拥有所有权

  // 同步锁，保护所有下面的共享状态变量
  mutable std::mutex state_mutex_;

  // --- 受互斥锁保护的共享状态 ---
  Position current_position_;
  ControlStatus control_status_;
  CommState comm_state_ = CommState::NORMAL;
  rclcpp::Time last_setpoint_time_;
  rclcpp::Time last_offboard_request_time_;
  rclcpp::Time emergency_start_time_;
  rclcpp::Time startup_time_;
  uint64_t offboard_setpoint_counter_ = 0;

  // --- 配置参数 ---
  const rclcpp::Duration MAIN_PROGRAM_TIMEOUT_{std::chrono::milliseconds(1000)};
  const rclcpp::Duration PX4_DATA_TIMEOUT_{std::chrono::milliseconds(2000)};
  const rclcpp::Duration EMERGENCY_HOLD_TIMEOUT_{std::chrono::seconds(10)};
  const rclcpp::Duration OFFBOARD_REQUEST_INTERVAL_{std::chrono::seconds(1)};

  // --- ROS 2 通信接口 ---
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
      trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
      vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      vehicle_local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr
      vehicle_control_mode_sub_;
};
