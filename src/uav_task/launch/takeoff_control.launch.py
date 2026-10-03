#!/usr/bin/env python3
"""
takeoff_control.launch.py — Layer 3 起飞/降落控制启动文件

启动:
  takeoff_control (Layer 3 交互式起飞/降落)

用法:
  ros2 launch uav_task takeoff_control.launch.py

前置条件:
  需要完整的 Layer 1 栈已在运行 (包括 state_bridge):
    ros2 launch px4_interface vslam_px4.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    require_rangefinder = DeclareLaunchArgument(
        'require_valid_rangefinder',
        default_value='true',
        description='Block automatic takeoff until PX4 dist_bottom is valid',
    )

    # Takeoff Control: 交互式起飞/降落 (Layer 3)
    takeoff_control = Node(
        package='uav_task',
        executable='takeoff_control.py',
        name='takeoff_control_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'require_valid_rangefinder': LaunchConfiguration(
                'require_valid_rangefinder'),
            'range_timeout_s': 0.5,
            'estimator_flags_timeout_s': 2.5,
            'min_dist_bottom_m': 0.05,
            # Ground-level raw TFmini readings can be below PX4's HAGL
            # validity floor.  Once the commanded climb crosses this height,
            # require PX4 to report a valid, fused range aid or land.
            'range_validity_gate_m': 0.45,
            'range_measurement_min_m': 0.35,
            'range_validity_timeout_s': 2.0,
            'center_above_sensor_m': 0.05,
            'stationary_check_s': 5.0,
            'max_ground_xy_drift_m': 0.15,
            'max_ground_xy_speed_m_s': 0.12,
            'no_liftoff_timeout_s': 3.0,
            'no_liftoff_delta_m': 0.15,
            'no_liftoff_max_height_m': 0.20,
        }],
    )

    return LaunchDescription([require_rangefinder, takeoff_control])
