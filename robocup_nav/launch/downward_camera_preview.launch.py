"""Calibrated Microdia image/H preview; no assumed extrinsic TF or PX4 output."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from pathlib import Path
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    calibration=Path(__file__).resolve().parents[1]/'config/downward_camera_info.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('h_transport',default_value='compressed'),
        DeclareLaunchArgument('device', default_value=
            '/dev/v4l/by-id/usb-LRCP_H-720P_LRCP_H-720P_SN0001-video-index0'),
        Node(package='v4l2_camera', executable='v4l2_camera_node',
             name='robocup_downward_preview', parameters=[{
                 'video_device': LaunchConfiguration('device'),
                 'image_size': [1280, 720], 'pixel_format': 'YUYV',
                 'time_per_frame': [1, 10],
                 'camera_info_url': calibration.as_uri(),
                 'output_encoding': 'rgb8', 'camera_frame_id': 'downward_optical_unverified',
             }], remappings=[('image_raw', '/robocup/downward/image_raw'),
                            ('image_raw/compressed','/robocup/downward/image_raw/compressed'),
                            ('camera_info', '/robocup/downward/camera_info')],
             output='screen'),
        ExecuteProcess(cmd=['python3',str(Path(__file__).resolve().parents[1]/'scripts/h_target_preview.py'),
                            '--ros-args','-p',['transport:=',LaunchConfiguration('h_transport')]],
                       output='screen'),
    ])
