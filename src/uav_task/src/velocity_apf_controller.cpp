#include "velocity_apf_controller.hpp"
#include <cmath>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

VelocityAPFController::VelocityAPFController(rclcpp::Node* node) 
    : node_(node), current_waypoint_index_(0), sequence_completed_(false), waypoint_reached_flag_(false) {
    
    // 创建序列完成发布器
    sequence_complete_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("/waypoint_sequence_complete", 10);
    
    if (!loadConfig()) {
        RCLCPP_ERROR(node_->get_logger(), "航点序列管理器配置加载失败，使用默认参数");
        
        // 设置默认PID配置
        pid_config_.kp_x = pid_config_.kp_y = 0.8f;
        pid_config_.kp_z = 0.6f;
        pid_config_.kd_x = pid_config_.kd_y = 0.1f;
        pid_config_.kd_z = 0.05f;
        pid_config_.ki_x = pid_config_.ki_y = pid_config_.ki_z = 0.0f;
        pid_config_.max_velocity_xy = 0.4f;
        pid_config_.max_velocity_z = 0.3f;
        pid_config_.derivative_filter_alpha = 0.8f;
        pid_config_.position_tolerance = 0.2f;
        
        // 设置默认避障配置
        avoidance_config_.laser_topic = "/scan";
        avoidance_config_.obstacle_distance_threshold = 1.0f;
        avoidance_config_.avoidance_fov_degrees = 360.0f;
        avoidance_config_.avoidance_strength_gain = 0.6f;
        avoidance_config_.max_avoidance_velocity = 0.3f;
        avoidance_config_.scan_timeout_ms = 200;
        avoidance_config_.min_valid_ranges_ratio = 0.5f;
    }
    
    RCLCPP_INFO(node_->get_logger(), "航点序列管理器 (Velocity APF Controller) 已初始化");
    RCLCPP_INFO(node_->get_logger(), "PID控制器配置:");
    RCLCPP_INFO(node_->get_logger(), "  - P增益(x,y,z): (%.2f, %.2f, %.2f)", 
               pid_config_.kp_x, pid_config_.kp_y, pid_config_.kp_z);
    RCLCPP_INFO(node_->get_logger(), "  - D增益(x,y,z): (%.2f, %.2f, %.2f)", 
               pid_config_.kd_x, pid_config_.kd_y, pid_config_.kd_z);
    RCLCPP_INFO(node_->get_logger(), "  - 最大速度(xy,z): (%.2f, %.2f) m/s", 
               pid_config_.max_velocity_xy, pid_config_.max_velocity_z);
    RCLCPP_INFO(node_->get_logger(), "避障模块配置:");
    RCLCPP_INFO(node_->get_logger(), "  - 障碍物距离阈值: %.2f m", avoidance_config_.obstacle_distance_threshold);
    RCLCPP_INFO(node_->get_logger(), "  - 避障视场角: %.1f 度", avoidance_config_.avoidance_fov_degrees);
    RCLCPP_INFO(node_->get_logger(), "  - 斥力增益: %.2f", avoidance_config_.avoidance_strength_gain);
    RCLCPP_INFO(node_->get_logger(), "  - 最大避障速度: %.2f m/s", avoidance_config_.max_avoidance_velocity);
}

void VelocityAPFController::setWaypointSequence(const std::vector<Waypoint>& waypoints) {
    waypoints_ = waypoints;
    current_waypoint_index_ = 0;
    sequence_completed_ = false;
    waypoint_reached_flag_ = false;
    resetPIDState();
    
    RCLCPP_INFO(node_->get_logger(), "设置航点序列，共 %zu 个航点:", waypoints_.size());
    for (size_t i = 0; i < waypoints_.size(); i++) {
        RCLCPP_INFO(node_->get_logger(), "  航点 %zu: %s (%.2f, %.2f, %.2f)",
                   i + 1, waypoints_[i].description.c_str(),
                   waypoints_[i].x, waypoints_[i].y, waypoints_[i].z);
    }
}

VelocityAPFController::OutputVelocity VelocityAPFController::calculateVelocity(const InputData& input) {
    // 检查航点序列是否已完成
    if (sequence_completed_ || waypoints_.empty()) {
        return OutputVelocity(0.0f, 0.0f, 0.0f, false, false, false, true, current_waypoint_index_);
    }
    
    // 检查当前航点索引是否有效
    if (current_waypoint_index_ >= waypoints_.size()) {
        sequence_completed_ = true;
        publishSequenceComplete();
        RCLCPP_INFO(node_->get_logger(), "航点序列已完成！");
        return OutputVelocity(0.0f, 0.0f, 0.0f, false, false, false, true, current_waypoint_index_);
    }
    
    const Waypoint& current_target = waypoints_[current_waypoint_index_];
    
    bool pid_active = false;
    bool avoidance_active = false;
    bool waypoint_reached = false;
    float final_vx = 0.0f, final_vy = 0.0f, final_vz = 0.0f;
    
    // 检查是否到达当前航点
    waypoint_reached = isWaypointReached(current_target, input);
    
    if (waypoint_reached && !waypoint_reached_flag_) {
        // 刚到达航点，记录时间
        waypoint_reached_time_ = node_->get_clock()->now();
        waypoint_reached_flag_ = true;
        RCLCPP_INFO(node_->get_logger(), "到达航点 %zu: %s", 
                   current_waypoint_index_ + 1, current_target.description.c_str());
    }

    if (waypoint_reached_flag_) {
        // 在航点处悬停一段时间
        auto elapsed = node_->get_clock()->now() - waypoint_reached_time_;
        float elapsed_seconds = elapsed.seconds();
        const float HOVER_TIME = 2.0f; // 悬停2秒

        if (elapsed_seconds > HOVER_TIME) {
            // 悬停时间结束，前进到下一个航点
            advanceToNextWaypoint();
            if (current_waypoint_index_ >= waypoints_.size()) {
                sequence_completed_ = true;
                publishSequenceComplete();
                RCLCPP_INFO(node_->get_logger(), "航点序列已完成！");
                return OutputVelocity(0.0f, 0.0f, 0.0f, false, false, true, true, current_waypoint_index_);
            }
        } else {
            // 继续悬停
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                                "航点悬停中... %.1f / %.1f 秒", elapsed_seconds, HOVER_TIME);
            return OutputVelocity(0.0f, 0.0f, 0.0f, true, false, true, false, current_waypoint_index_);
        }
    }
    
    // 当前目标可能已更新，重新获取
    const Waypoint& active_target = waypoints_[current_waypoint_index_];
    
    // 第一步：PID控制器计算目标速度
    auto pid_velocity = calculatePIDVelocity(active_target, input);
    final_vx = std::get<0>(pid_velocity);
    final_vy = std::get<1>(pid_velocity);
    final_vz = std::get<2>(pid_velocity);
    pid_active = true;
    
    // 第二步：避障算法修正速度
    if (input.avoidance_enabled && isValidScanData(input.scan_data)) {
        // The scan geometry is expressed in body FRD (after the FLU->FRD
        // sign change inside calculateRepulsiveVelocity), while PX4
        // trajectory velocity setpoints are NED.  Rotate the avoidance term
        // by the measured yaw before adding it to the NED waypoint velocity;
        // the old code only worked while yaw happened to be zero.
        const auto body_repulsive = calculateRepulsiveVelocity(input.scan_data);
        const float cy = std::cos(input.current_yaw);
        const float sy = std::sin(input.current_yaw);
        const std::pair<float, float> repulsive_velocity{
            cy * body_repulsive.first - sy * body_repulsive.second,
            sy * body_repulsive.first + cy * body_repulsive.second};
        
        if (std::abs(repulsive_velocity.first) > 1e-6f || std::abs(repulsive_velocity.second) > 1e-6f) {
            // 保存原PID输出用于日志
            float pid_vx = final_vx;
            float pid_vy = final_vy;
            
            // 避障速度叠加到PID输出上
            final_vx += repulsive_velocity.first;
            final_vy += repulsive_velocity.second;
            avoidance_active = true;
            
            // 重新限制速度，确保不超过PID的最大速度限制
            limitVelocityMagnitude(final_vx, final_vy, pid_config_.max_velocity_xy);
            
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                "避障激活! PID速度(%.2f, %.2f), 避障修正(%.2f, %.2f), 最终速度(%.2f, %.2f)",
                pid_vx, pid_vy, repulsive_velocity.first, repulsive_velocity.second, final_vx, final_vy);
        }
    } else if (input.avoidance_enabled && !isValidScanData(input.scan_data)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, 
                            "激光雷达数据无效，停止水平任务速度");
        // Continuing toward a waypoint without obstacle data is unsafe in the
        // structured/forest course. Zero velocity lets PX4 hold while the
        // mission waits for a fresh scan.
        final_vx = 0.0f;
        final_vy = 0.0f;
        final_vz = 0.0f;
        resetPIDState();
    }
    
    return OutputVelocity(final_vx, final_vy, final_vz, pid_active, avoidance_active, 
                         waypoint_reached, false, current_waypoint_index_);
}

void VelocityAPFController::resetSequence() {
    current_waypoint_index_ = 0;
    sequence_completed_ = false;
    waypoint_reached_flag_ = false;
    resetPIDState();
    RCLCPP_INFO(node_->get_logger(), "航点序列已重置到第一个航点");
}

const VelocityAPFController::Waypoint& VelocityAPFController::getCurrentWaypoint() const {
    static const Waypoint default_waypoint(0.0f, 0.0f, 0.0f, "无效航点");
    if (current_waypoint_index_ < waypoints_.size()) {
        return waypoints_[current_waypoint_index_];
    }
    return default_waypoint;
}

bool VelocityAPFController::loadConfig() {
    try {
        // 构建配置文件路径，优先使用测试配置
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("uav_task");
        // Competition and waypoint-test nodes intentionally use different
        // controller profiles.  The old implementation always preferred the
        // waypoint test file, so the competition mission silently ran with
        // test gains and obstacle thresholds.  An explicit parameter wins;
        // an empty value preserves the conservative waypoint-test default.
        const std::string configured_file =
            node_->declare_parameter<std::string>("controller_config_file", "");
        std::string test_config_file = configured_file.empty()
                                           ? package_share_dir + "/config/waypoint_test_config.yaml"
                                           : configured_file;
        std::string default_config_file = package_share_dir + "/config/obstacle_avoidance_config.yaml";
        
        // 尝试从测试配置文件加载
        try {
            YAML::Node config = YAML::LoadFile(test_config_file);
            
            // 加载PID配置
            auto pid_config = config["position_controller"];
            pid_config_.kp_x = pid_config["kp_x"].as<float>();
            pid_config_.kp_y = pid_config["kp_y"].as<float>();
            pid_config_.kp_z = pid_config["kp_z"].as<float>();
            pid_config_.kd_x = pid_config["kd_x"].as<float>();
            pid_config_.kd_y = pid_config["kd_y"].as<float>();
            pid_config_.kd_z = pid_config["kd_z"].as<float>();
            pid_config_.ki_x = pid_config["ki_x"].as<float>();
            pid_config_.ki_y = pid_config["ki_y"].as<float>();
            pid_config_.ki_z = pid_config["ki_z"].as<float>();
            pid_config_.max_velocity_xy = pid_config["max_velocity_xy"].as<float>();
            pid_config_.max_velocity_z = pid_config["max_velocity_z"].as<float>();
            pid_config_.derivative_filter_alpha = pid_config["derivative_filter_alpha"].as<float>();
            pid_config_.position_tolerance = pid_config["position_tolerance"].as<float>();
            
            // 加载避障配置
            auto avoidance_config = config["obstacle_avoidance"];
            avoidance_config_.laser_topic = avoidance_config["laser_topic"].as<std::string>();
            avoidance_config_.obstacle_distance_threshold = avoidance_config["obstacle_distance_threshold"].as<float>();
            avoidance_config_.avoidance_fov_degrees = avoidance_config["avoidance_fov_degrees"].as<float>();
            avoidance_config_.avoidance_strength_gain = avoidance_config["avoidance_strength_gain"].as<float>();
            avoidance_config_.max_avoidance_velocity = avoidance_config["max_avoidance_velocity"].as<float>();
            avoidance_config_.scan_timeout_ms = avoidance_config["scan_timeout_ms"].as<int>();
            avoidance_config_.min_valid_ranges_ratio = avoidance_config["min_valid_ranges_ratio"].as<float>();
            
            RCLCPP_INFO(node_->get_logger(), "从控制器配置文件 %s 加载配置成功", test_config_file.c_str());
            return true;
        } catch (const std::exception& e) {
            // 测试配置加载失败，尝试默认配置
            RCLCPP_WARN(node_->get_logger(), "测试配置加载失败，尝试默认配置: %s", e.what());
            
            try {
                YAML::Node config = YAML::LoadFile(default_config_file);
                
                // 加载PID配置
                auto pid_config = config["position_controller"];
                pid_config_.kp_x = pid_config["kp_x"].as<float>();
                pid_config_.kp_y = pid_config["kp_y"].as<float>();
                pid_config_.kp_z = pid_config["kp_z"].as<float>();
                pid_config_.kd_x = pid_config["kd_x"].as<float>();
                pid_config_.kd_y = pid_config["kd_y"].as<float>();
                pid_config_.kd_z = pid_config["kd_z"].as<float>();
                pid_config_.ki_x = pid_config["ki_x"].as<float>();
                pid_config_.ki_y = pid_config["ki_y"].as<float>();
                pid_config_.ki_z = pid_config["ki_z"].as<float>();
                pid_config_.max_velocity_xy = pid_config["max_velocity_xy"].as<float>();
                pid_config_.max_velocity_z = pid_config["max_velocity_z"].as<float>();
                pid_config_.derivative_filter_alpha = pid_config["derivative_filter_alpha"].as<float>();
                pid_config_.position_tolerance = pid_config["position_tolerance"].as<float>();
                
                // 加载避障配置
                auto avoidance_config = config["obstacle_avoidance"];
                avoidance_config_.laser_topic = avoidance_config["laser_topic"].as<std::string>();
                avoidance_config_.obstacle_distance_threshold = avoidance_config["obstacle_distance_threshold"].as<float>();
                avoidance_config_.avoidance_fov_degrees = avoidance_config["avoidance_fov_degrees"].as<float>();
                avoidance_config_.avoidance_strength_gain = avoidance_config["avoidance_strength_gain"].as<float>();
                avoidance_config_.max_avoidance_velocity = avoidance_config["max_avoidance_velocity"].as<float>();
                avoidance_config_.scan_timeout_ms = avoidance_config["scan_timeout_ms"].as<int>();
                avoidance_config_.min_valid_ranges_ratio = avoidance_config["min_valid_ranges_ratio"].as<float>();
                
                RCLCPP_INFO(node_->get_logger(), "从默认配置文件 %s 加载配置成功", default_config_file.c_str());
                return true;
            } catch (const std::exception& e2) {
                // 如果从安装路径加载失败，尝试从源码路径加载测试配置
                std::string src_test_config = "/home/wwm/ros2_ws/src/uav_task/config/waypoint_test_config.yaml";
                std::string src_default_config = "/home/wwm/ros2_ws/src/uav_task/config/obstacle_avoidance_config.yaml";
                
                try {
                    YAML::Node config = YAML::LoadFile(src_test_config);
                    
                    // 加载PID配置
                    auto pid_config = config["position_controller"];
                    pid_config_.kp_x = pid_config["kp_x"].as<float>();
                    pid_config_.kp_y = pid_config["kp_y"].as<float>();
                    pid_config_.kp_z = pid_config["kp_z"].as<float>();
                    pid_config_.kd_x = pid_config["kd_x"].as<float>();
                    pid_config_.kd_y = pid_config["kd_y"].as<float>();
                    pid_config_.kd_z = pid_config["kd_z"].as<float>();
                    pid_config_.ki_x = pid_config["ki_x"].as<float>();
                    pid_config_.ki_y = pid_config["ki_y"].as<float>();
                    pid_config_.ki_z = pid_config["ki_z"].as<float>();
                    pid_config_.max_velocity_xy = pid_config["max_velocity_xy"].as<float>();
                    pid_config_.max_velocity_z = pid_config["max_velocity_z"].as<float>();
                    pid_config_.derivative_filter_alpha = pid_config["derivative_filter_alpha"].as<float>();
                    pid_config_.position_tolerance = pid_config["position_tolerance"].as<float>();
                    
                    // 加载避障配置
                    auto avoidance_config = config["obstacle_avoidance"];
                    avoidance_config_.laser_topic = avoidance_config["laser_topic"].as<std::string>();
                    avoidance_config_.obstacle_distance_threshold = avoidance_config["obstacle_distance_threshold"].as<float>();
                    avoidance_config_.avoidance_fov_degrees = avoidance_config["avoidance_fov_degrees"].as<float>();
                    avoidance_config_.avoidance_strength_gain = avoidance_config["avoidance_strength_gain"].as<float>();
                    avoidance_config_.max_avoidance_velocity = avoidance_config["max_avoidance_velocity"].as<float>();
                    avoidance_config_.scan_timeout_ms = avoidance_config["scan_timeout_ms"].as<int>();
                    avoidance_config_.min_valid_ranges_ratio = avoidance_config["min_valid_ranges_ratio"].as<float>();
                    
                    RCLCPP_INFO(node_->get_logger(), "从源码测试配置 %s 加载配置成功", src_test_config.c_str());
                    return true;
                } catch (const std::exception& e3) {
                    // 最后尝试源码默认配置
                    YAML::Node config = YAML::LoadFile(src_default_config);
                    
                    // 加载PID配置
                    auto pid_config = config["position_controller"];
                    pid_config_.kp_x = pid_config["kp_x"].as<float>();
                    pid_config_.kp_y = pid_config["kp_y"].as<float>();
                    pid_config_.kp_z = pid_config["kp_z"].as<float>();
                    pid_config_.kd_x = pid_config["kd_x"].as<float>();
                    pid_config_.kd_y = pid_config["kd_y"].as<float>();
                    pid_config_.kd_z = pid_config["kd_z"].as<float>();
                    pid_config_.ki_x = pid_config["ki_x"].as<float>();
                    pid_config_.ki_y = pid_config["ki_y"].as<float>();
                    pid_config_.ki_z = pid_config["ki_z"].as<float>();
                    pid_config_.max_velocity_xy = pid_config["max_velocity_xy"].as<float>();
                    pid_config_.max_velocity_z = pid_config["max_velocity_z"].as<float>();
                    pid_config_.derivative_filter_alpha = pid_config["derivative_filter_alpha"].as<float>();
                    pid_config_.position_tolerance = pid_config["position_tolerance"].as<float>();
                    
                    // 加载避障配置
                    auto avoidance_config = config["obstacle_avoidance"];
                    avoidance_config_.laser_topic = avoidance_config["laser_topic"].as<std::string>();
                    avoidance_config_.obstacle_distance_threshold = avoidance_config["obstacle_distance_threshold"].as<float>();
                    avoidance_config_.avoidance_fov_degrees = avoidance_config["avoidance_fov_degrees"].as<float>();
                    avoidance_config_.avoidance_strength_gain = avoidance_config["avoidance_strength_gain"].as<float>();
                    avoidance_config_.max_avoidance_velocity = avoidance_config["max_avoidance_velocity"].as<float>();
                    avoidance_config_.scan_timeout_ms = avoidance_config["scan_timeout_ms"].as<int>();
                    avoidance_config_.min_valid_ranges_ratio = avoidance_config["min_valid_ranges_ratio"].as<float>();
                    
                    RCLCPP_INFO(node_->get_logger(), "从源码默认配置 %s 加载配置成功", src_default_config.c_str());
                    return true;
                }
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "加载配置失败: %s", e.what());
        return false;
    }
}

std::pair<float, float> VelocityAPFController::calculateRepulsiveVelocity(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan_data) {
    
    float repulsive_vx = 0.0f;
    float repulsive_vy = 0.0f;
    
    float fov_rad = avoidance_config_.avoidance_fov_degrees * M_PI / 180.0f;
    
    for (size_t i = 0; i < scan_data->ranges.size(); ++i) {
        float angle = scan_data->angle_min + i * scan_data->angle_increment;
        float range = scan_data->ranges[i];
        
        // 检查是否在视场角范围内
        if (std::abs(angle) > fov_rad / 2.0f) {
            continue;
        }
        
        // 检查距离是否在有效范围内且小于阈值
        if (std::isfinite(range) && 
            range > scan_data->range_min && 
            range < avoidance_config_.obstacle_distance_threshold) {
            
            // 计算斥力大小（距离越近，斥力越大）
            float magnitude = avoidance_config_.avoidance_strength_gain * 
                             (1.0f - (range / avoidance_config_.obstacle_distance_threshold));
            
            // 计算斥力方向（远离障碍物）
            // 坐标系说明：
            // - 激光雷达坐标系：FLU (Forward-Left-Up)，angle=0指向前方
            // - PX4飞控坐标系：FRD (Forward-Right-Down)
            // - 由于Yaw锁死，机体坐标系与世界坐标系X、Y轴对齐
            // - 因此直接在机体坐标系下计算避障速度
            repulsive_vx -= magnitude * std::cos(angle);  // X轴：前进方向，障碍物在前时产生后退力
            repulsive_vy += magnitude * std::sin(angle);  // Y轴：右侧方向，FLU转FRD需要反号，但这里是斥力所以保持正号
        }
    }
    
    // 限制避障速度幅值
    limitVelocityMagnitude(repulsive_vx, repulsive_vy, avoidance_config_.max_avoidance_velocity);
    
    return {repulsive_vx, repulsive_vy};
}

std::tuple<float, float, float> VelocityAPFController::calculatePIDVelocity(
    const Waypoint& target, const InputData& input) {
    
    auto current_time = node_->get_clock()->now();
    
    // 计算位置误差
    float error_x = target.x - input.current_x;
    float error_y = target.y - input.current_y;
    float error_z = target.z - input.current_z;
    
    // 初始化PID状态
    if (!pid_state_.initialized) {
        pid_state_.prev_error_x = error_x;
        pid_state_.prev_error_y = error_y;
        pid_state_.prev_error_z = error_z;
        pid_state_.integral_x = 0.0f;
        pid_state_.integral_y = 0.0f;
        pid_state_.integral_z = 0.0f;
        pid_state_.filtered_derivative_x = 0.0f;
        pid_state_.filtered_derivative_y = 0.0f;
        pid_state_.filtered_derivative_z = 0.0f;
        pid_state_.last_time = current_time;
        pid_state_.initialized = true;
        
        // 首次调用，只返回比例项
        float vx = pid_config_.kp_x * error_x;
        float vy = pid_config_.kp_y * error_y;
        float vz = pid_config_.kp_z * error_z;
        
        // 限制速度
        limitVelocityMagnitude(vx, vy, pid_config_.max_velocity_xy);
        vz = std::max(-pid_config_.max_velocity_z, std::min(pid_config_.max_velocity_z, vz));
        
        return std::make_tuple(vx, vy, vz);
    }
    
    // 计算时间间隔
    auto time_diff = current_time - pid_state_.last_time;
    float dt = time_diff.seconds();
    dt = std::max(dt, 0.001f);  // 防止除零
    
    // 计算微分项（带滤波）
    float derivative_x = (error_x - pid_state_.prev_error_x) / dt;
    float derivative_y = (error_y - pid_state_.prev_error_y) / dt;
    float derivative_z = (error_z - pid_state_.prev_error_z) / dt;
    
    // 低通滤波微分项
    pid_state_.filtered_derivative_x = pid_config_.derivative_filter_alpha * derivative_x + 
                                      (1.0f - pid_config_.derivative_filter_alpha) * pid_state_.filtered_derivative_x;
    pid_state_.filtered_derivative_y = pid_config_.derivative_filter_alpha * derivative_y + 
                                      (1.0f - pid_config_.derivative_filter_alpha) * pid_state_.filtered_derivative_y;
    pid_state_.filtered_derivative_z = pid_config_.derivative_filter_alpha * derivative_z + 
                                      (1.0f - pid_config_.derivative_filter_alpha) * pid_state_.filtered_derivative_z;
    
    // 计算积分项（暂时不使用，ki为0）
    pid_state_.integral_x += error_x * dt;
    pid_state_.integral_y += error_y * dt;
    pid_state_.integral_z += error_z * dt;
    
    // PID控制律
    float vx = pid_config_.kp_x * error_x + 
               pid_config_.kd_x * pid_state_.filtered_derivative_x + 
               pid_config_.ki_x * pid_state_.integral_x;
               
    float vy = pid_config_.kp_y * error_y + 
               pid_config_.kd_y * pid_state_.filtered_derivative_y + 
               pid_config_.ki_y * pid_state_.integral_y;
               
    float vz = pid_config_.kp_z * error_z + 
               pid_config_.kd_z * pid_state_.filtered_derivative_z + 
               pid_config_.ki_z * pid_state_.integral_z;
    
    // 限制速度
    limitVelocityMagnitude(vx, vy, pid_config_.max_velocity_xy);
    vz = std::max(-pid_config_.max_velocity_z, std::min(pid_config_.max_velocity_z, vz));
    
    // 更新状态
    pid_state_.prev_error_x = error_x;
    pid_state_.prev_error_y = error_y;
    pid_state_.prev_error_z = error_z;
    pid_state_.last_time = current_time;
    
    return std::make_tuple(vx, vy, vz);
}

void VelocityAPFController::resetPIDState() {
    pid_state_.initialized = false;
    pid_state_.integral_x = 0.0f;
    pid_state_.integral_y = 0.0f;
    pid_state_.integral_z = 0.0f;
    RCLCPP_DEBUG(node_->get_logger(), "PID状态已重置");
}

bool VelocityAPFController::isWaypointReached(const Waypoint& target, const InputData& input) {
    float dx = target.x - input.current_x;
    float dy = target.y - input.current_y;
    float dz = target.z - input.current_z;
    float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    return distance < pid_config_.position_tolerance;
}

bool VelocityAPFController::isValidScanData(const sensor_msgs::msg::LaserScan::SharedPtr& scan_data) {
    if (!scan_data) {
        return false;
    }
    
    // 检查数据新鲜度
    auto now = node_->get_clock()->now();
    auto scan_time = rclcpp::Time(scan_data->header.stamp);
    auto age_ms = (now - scan_time).nanoseconds() / 1000000;
    
    if (age_ms > avoidance_config_.scan_timeout_ms) {
        return false;
    }
    
    // 检查有效点比例
    if (scan_data->ranges.empty()) {
        return false;
    }
    
    size_t valid_points = 0;
    for (const auto& range : scan_data->ranges) {
        if (std::isfinite(range) && range >= scan_data->range_min && range <= scan_data->range_max) {
            valid_points++;
        }
    }
    
    float valid_ratio = static_cast<float>(valid_points) / scan_data->ranges.size();
    return valid_ratio >= avoidance_config_.min_valid_ranges_ratio;
}

void VelocityAPFController::limitVelocityMagnitude(float& vx, float& vy, float max_velocity) {
    float magnitude = std::sqrt(vx * vx + vy * vy);
    if (magnitude > max_velocity) {
        vx = (vx / magnitude) * max_velocity;
        vy = (vy / magnitude) * max_velocity;
    }
}

void VelocityAPFController::advanceToNextWaypoint() {
    current_waypoint_index_++;
    waypoint_reached_flag_ = false;
    resetPIDState();
    
    if (current_waypoint_index_ < waypoints_.size()) {
        RCLCPP_INFO(node_->get_logger(), "前进到航点 %zu: %s", 
                   current_waypoint_index_ + 1, waypoints_[current_waypoint_index_].description.c_str());
    }
}

void VelocityAPFController::publishSequenceComplete() {
    std_msgs::msg::Bool msg;
    msg.data = true;
    sequence_complete_publisher_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "发布航点序列完成信号");
}
