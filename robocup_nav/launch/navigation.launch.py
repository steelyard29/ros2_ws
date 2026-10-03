"""Shadow navigation: outputs only /robocup/nav/cmd_vel_raw, never /fmu/in.

The included footprint is a synthetic fixture. Explicitly select demo geometry
for an offline test, or supply verified platform dimensions for a shadow run.
"""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]


def nodes(context):
    demo = LaunchConfiguration('demo_geometry').perform(context).lower() == 'true'
    platform = yaml.safe_load((ROOT / 'config/platform.yaml').read_text())
    config = yaml.safe_load((ROOT / 'config/navigation.yaml').read_text())
    if not demo:
        if not platform['geometry_verified']:
            raise RuntimeError('Measure platform.yaml geometry first; demo_geometry:=true is OFFLINE ONLY.')
        length, width = platform['length_m'], platform['width_m']
        if not all(isinstance(v, (float, int)) and 0 < v < 2 for v in (length, width)):
            raise RuntimeError('Invalid measured length/width')
        footprint = str([[length/2, width/2], [length/2, -width/2],
                         [-length/2, -width/2], [-length/2, width/2]])
        for key in ('global_costmap', 'local_costmap'):
            config[key][key]['ros__parameters']['footprint'] = footprint
    # launch_ros consumes a YAML file for nested costmap node parameters.
    from tempfile import NamedTemporaryFile
    with NamedTemporaryFile(mode='w', prefix='robocup-nav-', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(config, stream)
        params = stream.name
    from launch.actions import RegisterEventHandler, OpaqueFunction as Cleanup
    from launch.event_handlers import OnShutdown
    def cleanup(_context):
        Path(params).unlink(missing_ok=True)
        return []
    return [
        Node(package='nav2_planner', executable='planner_server', name='planner_server', parameters=[params]),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[params], remappings=[('cmd_vel', '/robocup/nav/cmd_vel_raw')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[params, {'default_nav_to_pose_bt_xml': str(ROOT / 'config/navigate.xml'),
                                  'default_nav_through_poses_bt_xml': str(ROOT / 'config/navigate.xml')}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='robocup_lifecycle',
             parameters=[{'autostart': True, 'node_names': ['planner_server', 'controller_server', 'bt_navigator']}]),
        RegisterEventHandler(OnShutdown(on_shutdown=[Cleanup(function=cleanup)])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('demo_geometry', default_value='false'),
        OpaqueFunction(function=nodes),
    ])
