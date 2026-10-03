"""Existing container: nvblox static TSDF/2D ESDF. Depth camera/VIO are separate."""
from pathlib import Path
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]


def nodes(context):
    center = float(LaunchConfiguration('center_z').perform(context))
    if not math.isfinite(center) or not -1.0 <= center <= 4.0:
        raise RuntimeError('center_z must be finite in [-1,4], in odom coordinates')
    below = float(LaunchConfiguration('band_below').perform(context))
    above = float(LaunchConfiguration('band_above').perform(context))
    if not all(math.isfinite(v) and 0 < v <= 1 for v in (below, above)):
        raise RuntimeError('positive finite collision band extents required')
    bounds_rate=float(LaunchConfiguration('bounds_rate_hz').perform(context))
    if not math.isfinite(bounds_rate) or not 0<=bounds_rate<=5:
        raise RuntimeError('bounds diagnostic rate must be within 0..5Hz')
    return [Node(package='nvblox_ros', executable='nvblox_node', name='nvblox_node',
        parameters=[str(ROOT/'config/nvblox.yaml'), {
            'static_mapper.esdf_slice_height': center,
            'static_mapper.esdf_slice_min_height': center-below,
            'static_mapper.esdf_slice_max_height': center+above,
            'publish_debug_vis_rate_hz': bounds_rate,
        }], remappings=[
            ('camera_0/depth/image', '/camera/camera/depth/image_rect_raw'),
            ('camera_0/depth/camera_info', '/camera/camera/depth/camera_info'),
        ], output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('center_z', default_value='0.0'),
                              DeclareLaunchArgument('band_below', default_value='0.4'),
                              DeclareLaunchArgument('band_above', default_value='0.4'),
                              DeclareLaunchArgument('bounds_rate_hz', default_value='0.0'),
                              OpaqueFunction(function=nodes)])
