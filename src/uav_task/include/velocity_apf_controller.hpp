#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>
#include <string>
#include <chrono>
#include <vector>

class VelocityAPFController {
public:
    // 航点结构体
    struct Waypoint {
        float x, y, z;
        std::string description;
        
        Waypoint(float x = 0.0f, float y = 0.0f, float z = 0.0f, const std::string& desc = "")
            : x(x), y(y), z(z), description(desc) {}
    };

    // PID控制器配置结构体
    struct PIDConfig {
        float kp_x, kp_y, kp_z;           // 比例增益
        float kd_x, kd_y, kd_z;           // 微分增益
        float ki_x, ki_y, ki_z;           // 积分增益
        float max_velocity_xy;            // 水平最大速度
        float max_velocity_z;             // 垂直最大速度
        float derivative_filter_alpha;    // 微分滤波系数
        float position_tolerance;         // 位置容差
    };

    // 避障配置结构体
    struct AvoidanceConfig {
        std::string laser_topic;
        float obstacle_distance_threshold;
        float avoidance_fov_degrees;
        float avoidance_strength_gain;
        float max_avoidance_velocity;
        int scan_timeout_ms;
        float min_valid_ranges_ratio;
    };

    // 输入数据结构体
    struct InputData {
        // 当前状态
        float current_x, current_y, current_z;
        float current_yaw;  // 虽然锁死，但算法可能需要知道朝向
        
        // 激光雷达数据
        sensor_msgs::msg::LaserScan::SharedPtr scan_data;
        
        // 控制标志
        bool avoidance_enabled;
    };
    
    // 输出速度结构体
    struct OutputVelocity {
        float vx, vy, vz;
        bool is_pid_active;        // PID控制是否激活
        bool is_avoidance_active;  // 避障是否激活
        bool waypoint_reached;     // 当前航点是否到达
        bool sequence_completed;   // 整个航点序列是否完成
        size_t current_waypoint_index; // 当前航点索引
        
        OutputVelocity(float vx = 0.0f, float vy = 0.0f, float vz = 0.0f, 
                      bool pid_active = false, bool avoidance_active = false, 
                      bool reached = false, bool completed = false, size_t index = 0)
            : vx(vx), vy(vy), vz(vz), is_pid_active(pid_active), 
              is_avoidance_active(avoidance_active), waypoint_reached(reached),
              sequence_completed(completed), current_waypoint_index(index) {}
    };

    // PID状态结构体
    struct PIDState {
        float prev_error_x = 0.0f, prev_error_y = 0.0f, prev_error_z = 0.0f;
        float integral_x = 0.0f, integral_y = 0.0f, integral_z = 0.0f;
        float filtered_derivative_x = 0.0f, filtered_derivative_y = 0.0f, filtered_derivative_z = 0.0f;
        rclcpp::Time last_time;
        bool initialized = false;
    };

    // 构造函数
    explicit VelocityAPFController(rclcpp::Node* node);
    
    // 设置航点序列
    void setWaypointSequence(const std::vector<Waypoint>& waypoints);
    
    // 核心接口：输入当前状态和环境，输出安全速度
    OutputVelocity calculateVelocity(const InputData& input);
    
    // 重置到第一个航点
    void resetSequence();
    
    // 获取当前目标航点
    const Waypoint& getCurrentWaypoint() const;
    
    // 获取当前配置（只读）
    const PIDConfig& getPIDConfig() const { return pid_config_; }
    const AvoidanceConfig& getAvoidanceConfig() const { return avoidance_config_; }
    
    // 获取航点序列信息
    size_t getWaypointCount() const { return waypoints_.size(); }
    size_t getCurrentWaypointIndex() const { return current_waypoint_index_; }
    bool isSequenceCompleted() const { return sequence_completed_; }
    
    // --- 新增: 公共方法来手动推进到下一个航点 ---
    void advanceToNextWaypoint();

private:
    rclcpp::Node* node_;
    PIDConfig pid_config_;
    AvoidanceConfig avoidance_config_;
    PIDState pid_state_;
    
    // 航点序列管理
    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_index_;
    bool sequence_completed_;
    rclcpp::Time waypoint_reached_time_;
    bool waypoint_reached_flag_;
    
    // 成功完成发布器
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sequence_complete_publisher_;
    
    // 加载配置
    bool loadConfig();
    
    // PID位置控制器
    std::tuple<float, float, float> calculatePIDVelocity(const Waypoint& target, const InputData& input);
    
    // 人工势场法计算避障速度
    std::pair<float, float> calculateRepulsiveVelocity(
        const sensor_msgs::msg::LaserScan::SharedPtr& scan_data);
    
    // 验证激光雷达数据有效性
    bool isValidScanData(const sensor_msgs::msg::LaserScan::SharedPtr& scan_data);
    
    // 限制速度幅值
    void limitVelocityMagnitude(float& vx, float& vy, float max_velocity);
    
    // 检查是否到达航点
    bool isWaypointReached(const Waypoint& target, const InputData& input);
    
    // 重置PID状态（切换航点时调用）
    void resetPIDState();
    
    // 发布序列完成信号
    void publishSequenceComplete();
};
