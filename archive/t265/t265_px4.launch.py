"""
T265 → /visual_slam/tracking/odometry（独立于 D435i / Isaac VSLAM）

不修改 slam_rtabmap.launch.py。默认外参占位（FLU / base_link）:
  camera_x = 0.15  (前 15cm)
  camera_y = 0.0
  camera_z = -0.04 (下 4cm)
  roll/pitch/yaw = 0  (朝向需按实机再标定)

用法:
  ros2 launch ~/ros2_ws/launch/t265_px4.launch.py
  ODOM_SOURCE=t265 ~/ros2_ws/scripts/run_slam_px4.sh
  ~/ros2_ws/scripts/run_slam_px4_t265.sh
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_odom(context, *args, **kwargs):
    script = os.path.expanduser('~/ros2_ws/scripts/t265_odom_publisher.py')

    def _v(name):
        return LaunchConfiguration(name).perform(context)

    cmd = [
        'bash', '-lc',
        'source /opt/ros/humble/setup.bash && '
        'source "$HOME/ros2_ws/install/setup.bash" && '
        f'exec python3 "{script}" --ros-args '
        f'-p output_odom_topic:={_v("output_odom_topic")} '
        f'-p input_odom_topic:={_v("input_odom_topic")} '
        f'-p input_pose_topic:={_v("input_pose_topic")} '
        f'-p camera_x:={_v("camera_x")} '
        f'-p camera_y:={_v("camera_y")} '
        f'-p camera_z:={_v("camera_z")} '
        f'-p camera_roll:={_v("camera_roll")} '
        f'-p camera_pitch:={_v("camera_pitch")} '
        f'-p camera_yaw:={_v("camera_yaw")} '
        f'-p try_pyrealsense:={_v("try_pyrealsense")}',
    ]
    return [ExecuteProcess(
        cmd=cmd,
        name='t265_odom_publisher',
        output='screen',
    )]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('camera_x', default_value='0.15'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='-0.04'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'output_odom_topic',
            default_value='/visual_slam/tracking/odometry',
        ),
        DeclareLaunchArgument('input_odom_topic', default_value='/t265/odom'),
        DeclareLaunchArgument(
            'input_pose_topic', default_value='/camera/pose/sample'),
        DeclareLaunchArgument('try_pyrealsense', default_value='true'),
    ]

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_t265_camera_pose',
        arguments=[
            '--x', LaunchConfiguration('camera_x'),
            '--y', LaunchConfiguration('camera_y'),
            '--z', LaunchConfiguration('camera_z'),
            '--roll', LaunchConfiguration('camera_roll'),
            '--pitch', LaunchConfiguration('camera_pitch'),
            '--yaw', LaunchConfiguration('camera_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_pose_frame',
        ],
        output='log',
    )

    return LaunchDescription(args + [
        LogInfo(msg=[
            'T265 odom path (D435i/Isaac untouched). '
            'Extrinsic placeholder: +0.15m forward, -0.04m down.'
        ]),
        static_tf,
        OpaqueFunction(function=_launch_odom),
    ])
