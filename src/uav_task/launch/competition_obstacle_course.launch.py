#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    package_dir = Path(get_package_share_directory("uav_task"))
    default_config = str(package_dir / "config" / "competition_obstacle_course.yaml")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Competition obstacle course mission YAML config",
    )
    start_circle_arg = DeclareLaunchArgument(
        "start_circle_detector",
        default_value="false",
        description="Start the circular detector (run_slam_px4 already starts it with Microdia by default)",
    )
    start_qr_arg = DeclareLaunchArgument(
        "start_qr_detector",
        default_value="true",
        description="Start onboard QR landing/reconnaissance detector",
    )
    start_coop_arg = DeclareLaunchArgument(
        "start_air_ground_coop",
        default_value="true",
        description="Start air-ground cooperation report publisher",
    )
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/image_raw",
        description="Downward recognition camera image topic (Microdia by default)",
    )

    circle_node = Node(
        package="uav_task",
        executable="circle_landing_target_node.py",
        name="circle_landing_target",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {"image_topic": LaunchConfiguration("image_topic")},
        ],
        condition=IfCondition(LaunchConfiguration("start_circle_detector")),
    )
    qr_node = Node(
        package="uav_task",
        executable="qr_target_report_node.py",
        name="qr_target_report",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {"image_topic": LaunchConfiguration("image_topic")},
        ],
        condition=IfCondition(LaunchConfiguration("start_qr_detector")),
    )
    coop_node = Node(
        package="uav_task",
        executable="air_ground_coop_node.py",
        name="air_ground_coop",
        output="screen",
        parameters=[LaunchConfiguration("config")],
        condition=IfCondition(LaunchConfiguration("start_air_ground_coop")),
    )

    mission_node = Node(
        package="uav_task",
        executable="obstacle_course_mission",
        name="obstacle_course_mission",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("config"),
            {
                "controller_config_file": str(
                    package_dir / "config" / "obstacle_avoidance_config.yaml"
                )
            },
        ],
    )

    return LaunchDescription(
        [
            config_arg,
            start_circle_arg,
            start_qr_arg,
            start_coop_arg,
            image_topic_arg,
            LogInfo(msg="Starting obstacle course mission. Start PX4/VSLAM/RPLIDAR before arming."),
            circle_node,
            qr_node,
            coop_node,
            mission_node,
        ]
    )
