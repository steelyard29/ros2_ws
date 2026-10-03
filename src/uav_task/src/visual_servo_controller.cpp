#include "visual_servo_controller.hpp"

#include <cmath>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

VisualServoController::VisualServoController(rclcpp::Node* node) 
    : node_(node) {
    
    if (!loadConfig()) {
        RCLCPP_ERROR(node_->get_logger(), "视觉伺服控制器配置加载失败，使用默认参数");
        
        // 设置默认配置
        config_.kp_x = 0.15f;
        config_.kp_y = 0.15f;
        config_.kd_x = 0.0f;
        config_.kd_y = 0.0f;
        
        config_.camera_offset_x = -0.1075f;
        config_.camera_offset_y = 0.0f;
        config_.camera_offset_z = 0.0f;
        
        config_.ema_alpha = 0.3f;
        config_.max_velocity_ratio = 0.3f;
        config_.tolerance = 0.10f;
        
        config_.min_confidence = 0.8f;
        config_.max_target_distance = 0.5f;
        
        config_.detection_timeout_ms = 1000;
        config_.search_velocity = 0.1f;
        config_.target_label = "bridge";
        
        config_.waypoint_kp = 0.8f;
        config_.waypoint_kd = 0.2f;
        config_.waypoint_max_velocity = 0.5f;
        config_.enable_waypoint_fallback = true;
    }
    
    RCLCPP_INFO(node_->get_logger(), "视觉伺服控制器已初始化");
    RCLCPP_INFO(node_->get_logger(), "PD控制器配置:");
    RCLCPP_INFO(node_->get_logger(), "  - P增益(x,y): (%.3f, %.3f)", config_.kp_x, config_.kp_y);
    RCLCPP_INFO(node_->get_logger(), "  - D增益(x,y): (%.3f, %.3f)", config_.kd_x, config_.kd_y);
    RCLCPP_INFO(node_->get_logger(), "  - 追踪目标: '%s'", config_.target_label.c_str());
    RCLCPP_INFO(node_->get_logger(), "  - 容忍度: %.3f", config_.tolerance);
    RCLCPP_INFO(node_->get_logger(), "  - 检测超时: %d ms", config_.detection_timeout_ms);
    RCLCPP_INFO(node_->get_logger(), "航点控制配置:");
    RCLCPP_INFO(node_->get_logger(), "  - P增益: %.3f", config_.waypoint_kp);
    RCLCPP_INFO(node_->get_logger(), "  - D增益: %.3f", config_.waypoint_kd);
    RCLCPP_INFO(node_->get_logger(), "  - 最大速度: %.3f m/s", config_.waypoint_max_velocity);
    RCLCPP_INFO(node_->get_logger(), "  - 航点回退: %s", config_.enable_waypoint_fallback ? "启用" : "禁用");
}

VisualServoController::OutputVelocity VisualServoController::calculateVelocity(const InputData& input) {
    // 详细的输入数据日志，保留为DEBUG级别
    RCLCPP_DEBUG(node_->get_logger(), "--- [新周期] ---");
    RCLCPP_DEBUG(node_->get_logger(), "输入: det_valid=%d, det_name='%s', det_conf=%.2f, det_pos(%.3f, %.3f), uav_pos(%.3f, %.3f, %.3f)",
                 input.detection_valid, input.detection_name.c_str(), input.detection_confidence, 
                 input.detection_x, input.detection_y, input.current_x, input.current_y, input.current_z);

    bool detection_currently_valid = isValidDetection(input, config_.target_label);
    auto [compensated_error_x, compensated_error_y] = calculateCompensatedError(input);

    // 状态机逻辑: 根据检测结果切换模式
    bool detection_lost = false;
    if (!detection_currently_valid) {
        if (state_.initialized) {
            detection_lost = true;
            if (state_.current_mode != ControlMode::WAYPOINT && state_.current_mode != ControlMode::SEARCH) {
                RCLCPP_WARN(node_->get_logger(), "目标 '%s' 检测丢失或无效!", config_.target_label.c_str());
                if (state_.current_waypoint.valid && config_.enable_waypoint_fallback) {
                    state_.current_mode = ControlMode::WAYPOINT;
                    RCLCPP_INFO(node_->get_logger(), "切换到 [航点控制模式]");
                } else {
                    state_.current_mode = ControlMode::SEARCH;
                    RCLCPP_INFO(node_->get_logger(), "切换到 [搜索模式]");
                }
            }
        }
    } else {
        if (state_.current_mode != ControlMode::VISUAL_SERVO) {
            RCLCPP_INFO(node_->get_logger(), "检测到目标 '%s'，切换到 [视觉伺服模式]", config_.target_label.c_str());
        }
        state_.current_mode = ControlMode::VISUAL_SERVO;
    }

    // 初始化变量
    float vx = 0.0f, vy = 0.0f, vz = 0.0f;
    bool servo_active = false;
    bool target_reached = false;
    float tracking_error = 0.0f;
    ControlMode current_control_mode = state_.current_mode;

    // 根据当前模式计算速度
    if (current_control_mode == ControlMode::VISUAL_SERVO) {
        if (!state_.initialized) {
            state_.initialized = true;
            state_.filtered_error_x = compensated_error_x;
            state_.filtered_error_y = compensated_error_y;
            vx = (config_.kp_x * compensated_error_x);
            vy = (config_.kp_y * compensated_error_y);
            servo_active = true;
        } else {
            updateInternalState(input, compensated_error_x, compensated_error_y);
            float dt = 0.02f; // 假设的控制周期
            auto [pd_vx, pd_vy] = calculatePDControl(state_.filtered_error_x, state_.filtered_error_y, dt);
            vx = pd_vx;
            vy = pd_vy;
            servo_active = true;
        }
        tracking_error = std::sqrt(state_.filtered_error_x * state_.filtered_error_x + state_.filtered_error_y * state_.filtered_error_y);
        target_reached = tracking_error < config_.tolerance;

    } else if (current_control_mode == ControlMode::WAYPOINT) {
        if (!state_.initialized) { state_.initialized = true; }

        auto [wp_vx, wp_vy] = calculateWaypointVelocity(input);
        vx = wp_vx;
        vy = wp_vy;
        servo_active = true;

        float dx = state_.current_waypoint.x - input.current_x;
        float dy = state_.current_waypoint.y - input.current_y;
        tracking_error = std::sqrt(dx * dx + dy * dy);
        target_reached = isWaypointReached(input, state_.current_waypoint);

    } else if (current_control_mode == ControlMode::SEARCH) {
        auto [search_vx, search_vy] = calculateSearchVelocity();
        vx = search_vx;
        vy = search_vy;
        servo_active = true;
        tracking_error = std::sqrt(state_.last_valid_x * state_.last_valid_x + state_.last_valid_y * state_.last_valid_y);
    }

    // 速度限制
    float max_velocity = (current_control_mode == ControlMode::WAYPOINT) ? config_.waypoint_max_velocity : (1.0f * config_.max_velocity_ratio);
    limitVelocity(vx, vy, max_velocity);

    // =========================================================================
    // [日志修改] 只在 VISUAL_SERVO 模式下打印详细日志，简化其他模式
    // =========================================================================
    char status_buf[200];
    if (current_control_mode == ControlMode::VISUAL_SERVO) {
        snprintf(status_buf, sizeof(status_buf),
            "[VS-精对准] 视觉误差: (%.3f, %.3f) | 速度: (%.2f, %.2f) | 容忍度: %.3f | 到达: %s",
            state_.filtered_error_x, state_.filtered_error_y, vx, vy, config_.tolerance, target_reached ? "是" : "否");
        RCLCPP_INFO(node_->get_logger(), "%s", status_buf);
    } else if (current_control_mode == ControlMode::WAYPOINT) {
        // 粗对准过程不再打印 [WP-Calc] 和 [WP] 日志，保持安静
        // 您可以根据需要打开下面的DEBUG日志
        RCLCPP_DEBUG(node_->get_logger(), "[WP-粗对准] 误差: %.2fm | 速度: (%.2f, %.2f) | 到达: %s",
                    tracking_error, vx, vy, target_reached ? "是" : "否");
    } else {
        // 其他模式保持不变
        snprintf(status_buf, sizeof(status_buf),
            "[Search] LastErr: (%.2f, %.2f) -> Vel: (%.2f, %.2f)",
            state_.last_valid_x, state_.last_valid_y, vx, vy);
        RCLCPP_INFO(node_->get_logger(), "%s", status_buf);
    }

    // 详细的最终输出，保留为DEBUG级别
    RCLCPP_DEBUG(node_->get_logger(), "最终输出: 模式=%d, 速度(%.3f, %.3f, %.3f), 激活=%d, 到达=%d, 丢失=%d",
                 static_cast<int>(current_control_mode), vx, vy, vz, servo_active, target_reached, detection_lost);

    return OutputVelocity(vx, vy, vz, servo_active, target_reached, detection_lost, 
                         tracking_error, compensated_error_x, compensated_error_y, current_control_mode);
}

void VisualServoController::reset() {
    state_ = InternalState();
    RCLCPP_INFO(node_->get_logger(), "视觉伺服控制器状态已重置");
}

bool VisualServoController::loadConfig() {
    try {
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("uav_task");
        std::string config_file = package_share_dir + "/config/visual_servo_config.yaml";
        
        YAML::Node config;
        try {
            config = YAML::LoadFile(config_file);
        } catch (const YAML::BadFile& e) {
             RCLCPP_WARN(node_->get_logger(), "无法从安装路径加载配置 '%s'，尝试源码路径...", config_file.c_str());
            std::string src_config_file = "/home/cfly/ros2_ws/src/uav_task/config/visual_servo_config.yaml";
            config = YAML::LoadFile(src_config_file);
            config_file = src_config_file; 
        }
        
        // PD控制器参数
        auto pd_config = config["visual_servo_controller"];
        config_.kp_x = pd_config["kp_x"].as<float>();
        config_.kp_y = pd_config["kp_y"].as<float>();
        config_.kd_x = pd_config["kd_x"].as<float>();
        config_.kd_y = pd_config["kd_y"].as<float>();

        // 相机补偿参数
        auto cam_config = config["camera_compensation"];
        config_.camera_offset_x = cam_config["offset_x"].as<float>();
        config_.camera_offset_y = cam_config["offset_y"].as<float>();
        config_.camera_offset_z = cam_config["offset_z"].as<float>();

        // 控制参数
        auto ctrl_config = config["control_parameters"];
        config_.ema_alpha = ctrl_config["ema_alpha"].as<float>();
        config_.max_velocity_ratio = ctrl_config["max_velocity_ratio"].as<float>();
        config_.tolerance = ctrl_config["tolerance"].as<float>();

        // 检测参数
        auto detection_config = config["detection_parameters"];
        config_.min_confidence = detection_config["min_confidence"].as<float>();
        config_.max_target_distance = detection_config["max_target_distance"].as<float>();
        config_.detection_timeout_ms = detection_config["detection_timeout_ms"].as<int>();
        config_.search_velocity = detection_config["search_velocity"].as<float>();
        config_.target_label = detection_config["target_label"].as<std::string>();

        // 航点控制参数
        auto waypoint_config = config["waypoint_control"];
        config_.waypoint_kp = waypoint_config["waypoint_kp"].as<float>();
        config_.waypoint_kd = waypoint_config["waypoint_kd"].as<float>();
        config_.waypoint_max_velocity = waypoint_config["waypoint_max_velocity"].as<float>();
        config_.enable_waypoint_fallback = waypoint_config["enable_waypoint_fallback"].as<bool>();

        // 显式初始化所有字段（防止未配置项）
        // 若有未读取到的字段，可在此补充默认值

        RCLCPP_INFO(node_->get_logger(), "从 %s 加载视觉伺服配置成功", config_file.c_str());
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "加载视觉伺服配置失败: %s", e.what());
        return false;
    }
}

bool VisualServoController::isValidDetection(const InputData& input, const std::string& target_name) {
    if (!input.detection_valid) {
        // [新增调试日志]
        RCLCPP_DEBUG(node_->get_logger(), "无效检测: 'detection_valid' 标志为 false");
        return false;
    }

    if (input.detection_name != target_name) {
        RCLCPP_DEBUG(node_->get_logger(), "无效检测: 检测到目标 '%s'，但期望目标是 '%s'，已忽略。", 
                     input.detection_name.c_str(), target_name.c_str());
        return false;
    }

    if (input.detection_confidence < config_.min_confidence) {
        // [新增调试日志]
        RCLCPP_DEBUG(node_->get_logger(), "无效检测: 置信度 %.2f 低于阈值 %.2f",
                     input.detection_confidence, config_.min_confidence);
        return false;
    }
    
    float compensated_x = input.detection_x + config_.camera_offset_x;
    float compensated_y = input.detection_y + config_.camera_offset_y;
    float dist_from_uav_center = std::sqrt(compensated_x * compensated_x + compensated_y * compensated_y);
    
    if (dist_from_uav_center > config_.max_target_distance) {
        // [新增调试日志]
        RCLCPP_DEBUG(node_->get_logger(), "无效检测: 目标距离 (%.3f) 大于最大距离阈值 (%.3f)",
                     dist_from_uav_center, config_.max_target_distance);
        return false;
    }

    // [新增调试日志]
    RCLCPP_DEBUG(node_->get_logger(), "有效检测: 目标 '%s' 通过所有检查", target_name.c_str());
    return true;
}

std::tuple<float, float> VisualServoController::calculateCompensatedError(const InputData& input) {
    float compensated_x = input.detection_x + config_.camera_offset_x;
    float compensated_y = input.detection_y + config_.camera_offset_y;
    return std::make_tuple(compensated_x, compensated_y);
}

std::tuple<float, float> VisualServoController::calculatePDControl(float error_x, float error_y, float dt) {
    float d_error_x = (error_x - state_.prev_error_x) / dt;
    float d_error_y = (error_y - state_.prev_error_y) / dt;
    
    // [新增调试日志] 分解P, D项
    float p_term_x = config_.kp_x * error_x;
    float d_term_x = config_.kd_x * d_error_x;
    float p_term_y = config_.kp_y * error_y;
    float d_term_y = config_.kd_y * d_error_y;
    
    float vx = (p_term_x + d_term_x);
    float vy = (p_term_y + d_term_y);
    
    RCLCPP_DEBUG(node_->get_logger(), "PD控制(X): err=%.3f, P=%.3f, D=%.3f -> vx=%.3f", error_x, -p_term_x, -d_term_x, vx);
    RCLCPP_DEBUG(node_->get_logger(), "PD控制(Y): err=%.3f, P=%.3f, D=%.3f -> vy=%.3f", error_y, -p_term_y, -d_term_y, vy);

    state_.prev_error_x = error_x;
    state_.prev_error_y = error_y;
    return std::make_tuple(vx, vy);
}

void VisualServoController::updateInternalState(const InputData& input, float compensated_x, float compensated_y) {
    float old_filtered_x = state_.filtered_error_x;
    float old_filtered_y = state_.filtered_error_y;
    state_.filtered_error_x = config_.ema_alpha * compensated_x + (1.0f - config_.ema_alpha) * state_.filtered_error_x;
    state_.filtered_error_y = config_.ema_alpha * compensated_y + (1.0f - config_.ema_alpha) * state_.filtered_error_y;
    
    // [新增调试日志]
    RCLCPP_DEBUG(node_->get_logger(), "EMA滤波(X): %.3f -> %.3f", old_filtered_x, state_.filtered_error_x);
    RCLCPP_DEBUG(node_->get_logger(), "EMA滤波(Y): %.3f -> %.3f", old_filtered_y, state_.filtered_error_y);

    state_.last_valid_x = compensated_x;
    state_.last_valid_y = compensated_y;
}

std::tuple<float, float> VisualServoController::calculateSearchVelocity() {
    float error_magnitude = std::sqrt(state_.last_valid_x * state_.last_valid_x + state_.last_valid_y * state_.last_valid_y);
    if (error_magnitude > 0.01f) {
        float vx = (state_.last_valid_x / error_magnitude) * config_.search_velocity;
        float vy = (state_.last_valid_y / error_magnitude) * config_.search_velocity;
        return std::make_tuple(vx, vy);
    }
    return std::make_tuple(0.0f, 0.0f);
}

void VisualServoController::limitVelocity(float& vx, float& vy, float max_velocity) {
    float magnitude = std::sqrt(vx * vx + vy * vy);
    if (magnitude > max_velocity) {
        // [新增调试日志] 只有在限速时才打印
        RCLCPP_DEBUG(node_->get_logger(), "速度限制: 原始速度大小 %.3f > 最大值 %.3f。进行缩放。", magnitude, max_velocity);
        vx = (vx / magnitude) * max_velocity;
        vy = (vy / magnitude) * max_velocity;
    }
}

// 新增的航点控制相关函数
void VisualServoController::setWaypoint(const Waypoint& waypoint) {
    state_.current_waypoint = waypoint;
    // [修改] 强制切换到航点模式
    state_.current_mode = ControlMode::WAYPOINT; 
    RCLCPP_INFO(node_->get_logger(), "设置新航点并切换到航点模式: (%.3f, %.3f, %.3f), Yaw: %.3f, Tol: %.3f", 
                waypoint.x, waypoint.y, waypoint.z, waypoint.yaw, waypoint.tolerance);
}

void VisualServoController::clearWaypoint() {
    state_.current_waypoint = Waypoint();
    RCLCPP_INFO(node_->get_logger(), "清除航点。如果目标不可见，将进入搜索模式。");
}


std::tuple<float, float> VisualServoController::calculateWaypointVelocity(const InputData& input) {
    if (!state_.current_waypoint.valid) {
        return std::make_tuple(0.0f, 0.0f);
    }
    
    // 1. 计算当前位置误差
    float error_x = state_.current_waypoint.x - input.current_x;
    float error_y = state_.current_waypoint.y - input.current_y;
    
    // (偏航角误差计算保持不变, 当前速度控制不直接使用)
    float yaw_error = state_.current_waypoint.yaw - input.current_yaw;
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
    
    float dt = 0.02f; // 假设的固定控制周期
    
    // 2. 计算P项 (比例项)
    float p_term_x = config_.waypoint_kp * error_x;
    float p_term_y = config_.waypoint_kp * error_y;
    
    // 3. 计算D项 (微分项)
    float d_term_x = 0.0f;
    float d_term_y = 0.0f;
    if (config_.waypoint_kd > 0.0f && state_.initialized) {
        // 计算误差变化率
        float d_error_x = (error_x - state_.prev_error_x) / dt;
        float d_error_y = (error_y - state_.prev_error_y) / dt;
        d_term_x = config_.waypoint_kd * d_error_x;
        d_term_y = config_.waypoint_kd * d_error_y;
    }
    
    // 4. 合成总速度 (限速前)
    float vx = p_term_x + d_term_x;
    float vy = p_term_y + d_term_y;

    // 5. 更新上一时刻的误差，为下一次D项计算做准备
    state_.prev_error_x = error_x;
    state_.prev_error_y = error_y;
    
    return std::make_tuple(vx, vy);
}

bool VisualServoController::isWaypointReached(const InputData& input, const Waypoint& waypoint) {
    if (!waypoint.valid) {
        return false;
    }
    
    float dx = waypoint.x - input.current_x;
    float dy = waypoint.y - input.current_y;
    float dz = waypoint.z - input.current_z;
    
    float position_error = std::sqrt(dx * dx + dy * dy + dz * dz);
    
    bool reached = position_error <= waypoint.tolerance;
    // [新增调试日志]
    if(reached){
        RCLCPP_DEBUG(node_->get_logger(), "航点到达判断: 位置误差 %.3f <= 容忍度 %.3f -> 已到达", position_error, waypoint.tolerance);
    }

    return reached;
}