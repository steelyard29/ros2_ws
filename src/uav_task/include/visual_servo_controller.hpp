#ifndef VISUAL_SERVO_CONTROLLER_HPP
#define VISUAL_SERVO_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <chrono>
#include <tuple>

class VisualServoController {
public:
    // 航点数据结构
    struct Waypoint {
        float x, y, z;           // 航点坐标
        float yaw;               // 目标偏航角
        bool valid;              // 航点是否有效
        float tolerance;         // 航点到达容忍度
        
        Waypoint() : x(0.0f), y(0.0f), z(0.0f), yaw(0.0f), valid(false), tolerance(0.5f) {}
        Waypoint(float x, float y, float z, float yaw = 0.0f, float tol = 0.5f) 
            : x(x), y(y), z(z), yaw(yaw), valid(true), tolerance(tol) {}
    };
    
    // 控制模式枚举
    enum class ControlMode {
        VISUAL_SERVO,    // 视觉伺服模式
        WAYPOINT,        // 航点控制模式
        SEARCH           // 搜索模式
    };

    // 输入数据结构
    struct InputData {
        // YOLO检测数据 (已经是FRD坐标系)
        float detection_x;          // 目标在图像中的X位置误差
        float detection_y;          // 目标在图像中的Y位置误差
        bool detection_valid;       // 检测是否有效
        float detection_confidence; // 检测置信度
        std::string detection_name; // 检测到的目标名称
        // 当前无人机状态
        float current_x, current_y, current_z;  // 当前位置
        float current_yaw;          // 当前偏航角
        // 删除时间戳字段，YOLO检测数据不需要时间戳
        InputData() : detection_x(0.0f), detection_y(0.0f), detection_valid(false), 
                     detection_confidence(0.0f), current_x(0.0f), current_y(0.0f), 
                     current_z(0.0f), current_yaw(0.0f) {}
    };
    
    // 输出速度结构
    struct OutputVelocity {
        float vx, vy, vz;          // 输出速度指令
        bool servo_active;         // 伺服是否激活
        bool target_reached;       // 是否达到目标精度
        bool detection_lost;       // 是否丢失检测
        float tracking_error;      // 当前跟踪误差 (欧几里得距离)
        float compensated_error_x; // 补偿后的X误差
        float compensated_error_y; // 补偿后的Y误差
        ControlMode control_mode;  // 当前控制模式
        
        OutputVelocity(float x, float y, float z, bool active, bool reached, 
                      bool lost, float error, float comp_x, float comp_y, ControlMode mode)
            : vx(x), vy(y), vz(z), servo_active(active), target_reached(reached), 
              detection_lost(lost), tracking_error(error), 
              compensated_error_x(comp_x), compensated_error_y(comp_y), control_mode(mode) {}
    };
    
    // 配置结构
    struct Config {
        // PD控制器参数
        float kp_x, kp_y;          // 比例增益
        float kd_x, kd_y;          // 微分增益
        // 相机补偿偏移
        float camera_offset_x;      // 相机X轴偏移
        float camera_offset_y;      // 相机Y轴偏移
        float camera_offset_z;      // 相机Z轴偏移
        // 滤波参数
        float ema_alpha;           // 指数移动平均滤波系数
        // 速度和精度限制
        float max_velocity_ratio;  // 最大速度比例 (相对于MAX_VELOCITY)
        float tolerance;           // 到达容忍度
        // 检测验证参数
        float min_confidence;      // 最小置信度
        float max_target_distance; // 最大目标距离
        // 超时和丢失处理
        int detection_timeout_ms;   // 检测超时时间 (毫秒)
        float search_velocity;      // 搜索时的移动速度
        std::string target_label;   // 目标标签（从yaml加载）
        // 航点控制参数
        float waypoint_kp;         // 航点控制比例增益
        float waypoint_kd;         // 航点控制微分增益
        float waypoint_max_velocity; // 航点控制最大速度
        bool enable_waypoint_fallback; // 是否启用航点回退模式
    };

private:
    // 内部状态
    struct InternalState {
        bool initialized;
        float filtered_error_x;
        float filtered_error_y;
        float prev_error_x;
        float prev_error_y;
        // 删除时间相关字段，简化状态管理
        float last_valid_x;  // 最后有效检测的X位置
        float last_valid_y;  // 最后有效检测的Y位置
        bool search_mode;    // 是否处于搜索模式
        ControlMode current_mode; // 当前控制模式
        Waypoint current_waypoint; // 当前航点
        
        InternalState() : initialized(false), filtered_error_x(0.0f), filtered_error_y(0.0f),
                         prev_error_x(0.0f), prev_error_y(0.0f), last_valid_x(0.0f), 
                         last_valid_y(0.0f), search_mode(false), current_mode(ControlMode::WAYPOINT),
                         current_waypoint() {}
    };

    rclcpp::Node* node_;
    Config config_;
    InternalState state_;

public:
    explicit VisualServoController(rclcpp::Node* node);
    
    // 主要接口
    OutputVelocity calculateVelocity(const InputData& input);
    void reset();
    bool loadConfig();
    
    // 航点控制接口
    void setWaypoint(const Waypoint& waypoint);
    void clearWaypoint();
    const Waypoint& getCurrentWaypoint() const { return state_.current_waypoint; }
    ControlMode getCurrentMode() const { return state_.current_mode; }
    
    // 配置访问
    const Config& getConfig() const { return config_; }
    void setConfig(const Config& config) { config_ = config; }
    
    // --- 新增: 获取当前目标标签 ---
    const std::string& getTargetLabel() const { return config_.target_label; }
    
    // --- 新增: 设置目标标签 ---
    void setTargetLabel(const std::string& label) { config_.target_label = label; }

private:
    // 内部辅助函数
    bool isValidDetection(const InputData& input, const std::string& target_name = "");
    std::tuple<float, float> calculateCompensatedError(const InputData& input);
    std::tuple<float, float> calculatePDControl(float error_x, float error_y, float dt);
    void updateInternalState(const InputData& input, float compensated_x, float compensated_y);
    std::tuple<float, float> calculateSearchVelocity();
    std::tuple<float, float> calculateWaypointVelocity(const InputData& input);
    void limitVelocity(float& vx, float& vy, float max_velocity);
    bool isWaypointReached(const InputData& input, const Waypoint& waypoint);
};

#endif // VISUAL_SERVO_CONTROLLER_HPP
