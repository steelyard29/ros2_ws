#!/usr/bin/env python3
"""Optional RPLIDAR + Microdia downward camera + obstacle_distance bridge."""

import os
import subprocess
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool(value: str) -> bool:
    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


def _find_microdia_device(prefer: str = 'auto') -> str:
    """Resolve Microdia UVC node (USB 0c45:6366 / LRCP). Prefer capture with size.

    Never returns a RealSense /dev/videoN just because it is video0.
    """
    prefer = (prefer or 'auto').strip()

    v4l = Path('/sys/class/video4linux')
    candidates = []
    if v4l.is_dir():
        for node in sorted(v4l.glob('video*')):
            try:
                name = (node / 'name').read_text(encoding='utf-8', errors='ignore').strip()
            except OSError:
                continue
            try:
                real = (node / 'device').resolve()
            except OSError:
                real = None
            idv = idp = ''
            d = real
            while d and d != Path('/'):
                vendor = d / 'idVendor'
                product = d / 'idProduct'
                if vendor.is_file() and product.is_file():
                    idv = vendor.read_text().strip()
                    idp = product.read_text().strip()
                    break
                d = d.parent
            usb_ok = (idv, idp) == ('0c45', '6366')
            name_ok = any(k.lower() in name.lower() for k in ('LRCP', 'Microdia', 'Vitade'))
            if not (usb_ok or name_ok):
                continue
            dev = f'/dev/{node.name}'
            if not os.path.exists(dev):
                continue
            has_size = False
            try:
                out = subprocess.check_output(
                    ['v4l2-ctl', '-d', dev, '--all'],
                    stderr=subprocess.DEVNULL,
                    text=True,
                    timeout=2,
                )
                has_size = 'Width/Height' in out
            except (OSError, subprocess.SubprocessError):
                has_size = False
            candidates.append(
                (0 if has_size else 1, int(node.name.replace('video', '') or 999), dev)
            )

    if prefer and prefer != 'auto' and os.path.exists(prefer):
        if any(c[2] == prefer for c in candidates):
            return prefer
        # Explicit path is not Microdia; fall through to auto if possible.

    if not candidates:
        if prefer and prefer != 'auto' and os.path.exists(prefer):
            return prefer
        return '/dev/video0'
    candidates.sort()
    return candidates[0][2]


def _launch_setup(context, *args, **kwargs):
    nodes = []

    start_rplidar = _bool(LaunchConfiguration('start_rplidar').perform(context))
    start_microdia = _bool(LaunchConfiguration('start_microdia').perform(context))
    start_od = _bool(LaunchConfiguration('start_obstacle_distance').perform(context))
    rplidar_port = LaunchConfiguration('rplidar_port').perform(context)
    microdia_device = _find_microdia_device(
        LaunchConfiguration('microdia_device').perform(context)
    )
    laser_x = LaunchConfiguration('laser_x').perform(context)
    laser_y = LaunchConfiguration('laser_y').perform(context)
    laser_z = LaunchConfiguration('laser_z').perform(context)
    laser_yaw = LaunchConfiguration('laser_yaw').perform(context)
    cam_x = LaunchConfiguration('cam_x').perform(context)
    cam_y = LaunchConfiguration('cam_y').perform(context)
    cam_z = LaunchConfiguration('cam_z').perform(context)
    calibration_file = LaunchConfiguration('calibration_file').perform(context)

    if start_rplidar:
        nodes.append(
            Node(
                package='rplidar_ros',
                executable='rplidar_node',
                name='rplidar_node',
                output='screen',
                # RPLidar A2M8 motor needs ~2s to spin up after DTR=1.
                # First attempt often times out (80008002); respawn with
                # a short delay gives the motor time to stabilize.
                respawn=True,
                respawn_delay=3.0,
                parameters=[{
                    'channel_type': 'serial',
                    'serial_port': rplidar_port,
                    'serial_baudrate': 115200,
                    'frame_id': 'laser',
                    'inverted': False,
                    'angle_compensate': True,
                }],
            )
        )
        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_laser',
                arguments=[
                    laser_x, laser_y, laser_z,
                    laser_yaw, '0', '0',
                    'base_link', 'laser',
                ],
            )
        )

    if start_od:
        nodes.append(
            Node(
                package='uav_task',
                executable='laser_scan_to_obstacle_distance.py',
                name='laser_scan_to_obstacle_distance',
                output='screen',
                parameters=[{
                    'scan_topic': '/scan',
                    'obstacle_topic': '/fmu/in/obstacle_distance',
                }],
            )
        )

    if start_microdia:
        # ros-humble-v4l2_camera cannot convert MJPG (crashes with empty encoding).
        # Use YUYV @ 640x480 for 30 FPS; MJPG 1280x720 is unsupported by this node.
        nodes.append(
            Node(
                package='v4l2_camera',
                executable='v4l2_camera_node',
                name='microdia_downward_camera',
                output='screen',
                parameters=[{
                    'video_device': microdia_device,
                    'image_size': [640, 480],
                    'pixel_format': 'YUYV',
                    'output_encoding': 'rgb8',
                    'camera_frame_id': 'downward_cam',
                }],
                remappings=[
                    ('image_raw', '/image_raw'),
                ],
            )
        )
        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_downward_cam',
                arguments=[
                    cam_x, cam_y, cam_z,
                    '0', '1.5707963', '0',
                    'base_link', 'downward_cam',
                ],
            )
        )
        nodes.append(
            Node(
                package='uav_task',
                executable='circle_landing_target_node.py',
                name='circle_landing_target',
                output='screen',
                parameters=[{
                    'image_topic': '/image_raw',
                    'calibration_file': calibration_file,
                    'real_diameter_cm': 60.0,
                }],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_rplidar', default_value='0'),
        DeclareLaunchArgument('start_microdia', default_value='0'),
        DeclareLaunchArgument('start_obstacle_distance', default_value='0'),
        DeclareLaunchArgument('rplidar_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument(
            'microdia_device',
            default_value='auto',
            description='UVC path or "auto" (USB 0c45:6366 / LRCP)',
        ),
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.1'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),
        DeclareLaunchArgument('cam_x', default_value='0.0'),
        DeclareLaunchArgument('cam_y', default_value='0.0'),
        DeclareLaunchArgument('cam_z', default_value='0.0'),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=(
                '/home/cfly/uav_circle_distance_orin/configs/'
                'drone_calibration_fov_stream1_corrected.json'
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
