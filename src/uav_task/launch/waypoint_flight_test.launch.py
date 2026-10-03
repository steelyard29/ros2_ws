#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='日志级别 (debug, info, warn, error)'
        ),
        
        # 打印启动信息
        LogInfo(msg="=== 启动航点飞行测试程序 ==="),
        LogInfo(msg="测试路径: 起飞1m -> (1,0) -> (1,1) -> (2,1) -> (2,2) -> 降落"),
        LogInfo(msg="确保PX4 SITL或真实飞机已连接并运行"),
        
        # 启动航点飞行测试节点
        Node(
            package='uav_task',
            executable='waypoint_flight_test',
            name='waypoint_flight_test',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
    ])
