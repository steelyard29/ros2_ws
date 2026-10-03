"""Camera + cuVSLAM + nvblox + shadow Nav2; no flight-controller connection."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parent


def generate_launch_description():
    center = LaunchConfiguration('center_z')
    return LaunchDescription([
        DeclareLaunchArgument('center_z', default_value='0.0',
                              description='Fixed ESDF band center in odom; NOT a takeoff height command'),
        DeclareLaunchArgument('navigation', default_value='true'),
        DeclareLaunchArgument('trial_profile', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(ROOT/'perception.launch.py')),
            launch_arguments={
                'nvblox_depth': 'true',
                'publish_map_tf': 'true',
                'imu_fusion': 'false',
            }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(ROOT/'nvblox.launch.py')),
            launch_arguments={'center_z': center}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(ROOT/'nvblox_navigation.launch.py')),
            launch_arguments={'center_z': center,
                              'trial_profile': LaunchConfiguration('trial_profile')}.items(),
            condition=IfCondition(LaunchConfiguration('navigation'))),
    ])
