"""A* + DWB + official nvblox layer; always shadow outputs, never flight inputs.

Both costmaps are in continuous odom to avoid projecting 3D map corrections
through the plugin's planar-transform approximation. No RTAB-Map/AMCL required.
"""
from pathlib import Path
from tempfile import NamedTemporaryFile
import yaml
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from navigation_trial_profile import apply_trial_profile


def nodes(context):
    p = yaml.safe_load((ROOT/'config/platform.yaml').read_text())
    cfg = yaml.safe_load((ROOT/'config/navigation.yaml').read_text())
    if LaunchConfiguration('trial_profile').perform(context).lower() == 'true':
        apply_trial_profile(cfg)
    if LaunchConfiguration('debug_trajectories').perform(context).lower() == 'true':
        cfg['controller_server']['ros__parameters']['FollowPath'].update(
            publish_evaluation=True, short_circuit_trajectory_evaluation=False)
        horizon=float(LaunchConfiguration('debug_sim_time').perform(context))
        if horizon:
            if not 2.0 <= horizon <= 6.0:
                raise RuntimeError('Diagnostic horizon must be between 2 and 6 seconds')
            cfg['controller_server']['ros__parameters']['FollowPath']['sim_time']=horizon
        generator=LaunchConfiguration('debug_generator').perform(context)
        if generator:
            if generator not in ('LimitedAccelGenerator','StandardTrajectoryGenerator'):
                raise RuntimeError('Unsupported diagnostic generator')
            cfg['controller_server']['ros__parameters']['FollowPath']['trajectory_generator_name']='dwb_plugins::'+generator
    length, width = p['length_m'], p['width_m']
    if LaunchConfiguration('trial_profile').perform(context).lower() == 'true':
        # 2026-09-29 operator: guarded maximum is <=600 mm; do not shrink
        # the trial envelope to the older 550 mm estimate.
        length, width = max(length, 0.60), max(width, 0.60)
    if not all(isinstance(v, (int, float)) and 0.1 <= v <= 1.0 for v in (length, width)):
        raise RuntimeError('Measured horizontal dimensions required')
    footprint = str([[length/2, width/2], [length/2, -width/2],
                     [-length/2, -width/2], [-length/2, width/2]])
    for key in ('global_costmap', 'local_costmap'):
        c = cfg[key][key]['ros__parameters']
        c['global_frame'] = 'odom'
        c['footprint'] = footprint
        c['plugins'] = ['nvblox_layer', 'inflation_layer']
        c.pop('static_layer', None)
        c.pop('obstacle_layer', None)
        c['always_send_full_costmap'] = True
        c['nvblox_layer'] = {
            'plugin': 'nvblox::nav2::NvbloxCostmapLayer',
            'enabled': True, 'nvblox_map_slice_topic': '/nvblox_node/static_map_slice',
            'nav2_costmap_global_frame': 'odom',
            'convert_to_binary_costmap': True,
        }
        if key == 'global_costmap':
            c.update({'rolling_window': False, 'width': 30, 'height': 30,
                      'origin_x': -15.0, 'origin_y': -15.0})
    cfg['bt_navigator']['ros__parameters']['global_frame'] = 'odom'
    with NamedTemporaryFile(mode='w', prefix='robocup-nvblox-', suffix='.yaml', delete=False) as f:
        yaml.safe_dump(cfg, f)
        params = f.name
    def cleanup(_):
        Path(params).unlink(missing_ok=True)
        return []
    center = float(LaunchConfiguration('center_z').perform(context))
    return [
        Node(package='nav2_planner', executable='planner_server', name='planner_server', parameters=[params]),
        Node(package='nav2_controller', executable='controller_server', name='controller_server', parameters=[params],
             remappings=[('cmd_vel', '/robocup/nvblox/cmd_vel_unchecked')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[params, {'default_nav_to_pose_bt_xml': str(ROOT/'config/navigate.xml'),
                                  'default_nav_through_poses_bt_xml': str(ROOT/'config/navigate.xml')}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='robocup_nvblox_lifecycle',
             parameters=[{'autostart': True, 'node_names': ['planner_server', 'controller_server', 'bt_navigator']}]),
        # Pure ROS helper, no px4_msgs requirement, no /fmu publishers.
        ExecuteProcess(
            cmd=['python3', str(ROOT/'scripts/nvblox_guard.py'), '--ros-args', '-p', f'center_z:={center}']),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('center_z', default_value='0.0'),
                              DeclareLaunchArgument('trial_profile', default_value='false'),
                              DeclareLaunchArgument('debug_trajectories', default_value='false'),
                              DeclareLaunchArgument('debug_sim_time', default_value='0'),
                              DeclareLaunchArgument('debug_generator', default_value=''),
                              OpaqueFunction(function=nodes)])
