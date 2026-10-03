from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():

    microxrce_agent_process = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent',
            'serial',
            '-D', '/dev/ttyTHS1',
            '-b', '921600'
        ],
        name='microxrce_agent',
        output='log'
    )

    px4_gateway_node = Node(
        package='px4_interface',
        executable='px4_gateway_node',
        name='px4_gateway_node',
        output='log'
    )

    uav_dashboard_process = ExecuteProcess(
        cmd=[
            'uav-dashboard',
            '--mode', 'ros2',
            '--ros-profile', 'px4_interface',
            '--poll-interval', '0.5',
            '--log-level', 'DEBUG'
        ],
        name='uav_dashboard',
        output='screen',
    )

    return LaunchDescription([
        microxrce_agent_process,
        

        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=microxrce_agent_process,
                on_start=[px4_gateway_node]
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=px4_gateway_node,
                on_start=[uav_dashboard_process]
            )
        )
    ])