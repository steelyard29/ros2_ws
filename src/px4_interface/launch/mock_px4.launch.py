from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_interface',
            executable='mock_px4_cache_publisher.py',
            name='mock_px4_cache_publisher',
            output='screen'
        ),
    ])