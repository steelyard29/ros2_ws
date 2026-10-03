"""Host mapping using cuVSLAM external odometry, never stereo_odometry.

New run = new database. No --delete_db_on_start or overwrite option.
"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    database = Path(LaunchConfiguration('database').perform(context))
    if not database.is_absolute() or database.exists():
        raise RuntimeError('database must be a NEW absolute path; existing maps are never overwritten')
    database.parent.mkdir(parents=True, exist_ok=True)
    return [Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', parameters=[{
        'frame_id': 'base_link', 'map_frame_id': 'map', 'odom_frame_id': '',
        'publish_tf': True, 'subscribe_depth': True, 'subscribe_rgb': True,
        'subscribe_odom_info': False, 'approx_sync': True, 'approx_sync_max_interval': 0.05,
        'topic_queue_size': 10, 'sync_queue_size': 10,
        'qos_image': 2, 'qos_camera_info': 2, 'qos_odom': 2,
        'database_path': str(database),
        'Rtabmap/DetectionRate': '1.0', 'Reg/Strategy': '0',
        'Reg/Force3DoF': 'false', 'Optimizer/Slam2D': 'false',
        'Mem/IncrementalMemory': 'true', 'Grid/FromDepth': 'true',
        'Grid/3D': 'true', 'Grid/CellSize': '0.05',
        'Grid/RangeMax': '4.0', 'Grid/DepthDecimation': '4',
        'Grid/NormalsSegmentation': 'true', 'Grid/MaxGroundHeight': '0.05',
        'Grid/MaxObstacleHeight': '2.5', 'Grid/RayTracing': 'true',
        'RGBD/LinearUpdate': '0.10', 'RGBD/AngularUpdate': '0.10',
    }], remappings=[
        ('rgb/image', '/camera/camera/color/image_raw'),
        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
        ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ('odom', '/visual_slam/tracking/odometry'), ('grid_map', '/map'),
    ])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('database'), OpaqueFunction(function=nodes)])
