"""Existing sensors -> A* -> tagged APF candidates. NEVER starts PX4 writers.

Requires independently started VIO, nvblox full-height slice and /scan with TF.
Do not launch beside another planner or another odom->base_footprint publisher.
Does not start DWB, BT navigation, drivers, a camera container, or DDS Agent.
"""
from pathlib import Path
from tempfile import NamedTemporaryFile
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,OpaqueFunction,RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
ROOT=Path(__file__).resolve().parents[1]


def planner_parameters():
    """Same planner config for deployment and isolated actual-plugin tests."""
    cfg=yaml.safe_load((ROOT/'config/navigation.yaml').read_text())
    cost=cfg['global_costmap']['global_costmap']['ros__parameters']
    cost.update(global_frame='odom',footprint='[[0.3,0.3],[0.3,-0.3],[-0.3,-0.3],[-0.3,0.3]]',
        plugins=['nvblox_layer','inflation_layer'],rolling_window=False,width=30,height=30,
        origin_x=-15.,origin_y=-15.,track_unknown_space=True,always_send_full_costmap=True)
    for key in ('static_layer','obstacle_layer'):cost.pop(key,None)
    cost['nvblox_layer']=dict(plugin='nvblox::nav2::NvbloxCostmapLayer',enabled=True,
        nvblox_map_slice_topic='/nvblox_node/static_map_slice',nav2_costmap_global_frame='odom',
        convert_to_binary_costmap=True)
    cfg['planner_server']['ros__parameters']['GridBased'].update(use_astar=True,allow_unknown=False,tolerance=.05)
    return {k:cfg[k] for k in ('planner_server','global_costmap')}


def setup(context):
    executable=Path(LaunchConfiguration('apf_executable').perform(context))
    if not executable.is_file():raise RuntimeError('APF binary must be built for this environment')
    selected=planner_parameters()
    with NamedTemporaryFile(mode='w',prefix='robocup-task-plan-',suffix='.yaml',delete=False) as f:
        yaml.safe_dump(selected,f);params=f.name
    def cleanup(_):Path(params).unlink(missing_ok=True);return []
    return [
        Node(package='nav2_planner',executable='planner_server',name='planner_server',parameters=[params]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='task_planner_lifecycle',
             parameters=[{'autostart':True,'node_names':['planner_server']}]),
        ExecuteProcess(cmd=['python3',str(ROOT/'scripts/navigation_goal_bridge.py')]),
        ExecuteProcess(cmd=['python3',str(ROOT/'scripts/task_scan_bridge.py')]),
        # Supplies existing yaw-only base_footprint TF. Its DWB output is unused.
        ExecuteProcess(cmd=['python3',str(ROOT/'scripts/nvblox_guard.py'),'--ros-args','-p',
            'center_z:='+LaunchConfiguration('center_z').perform(context),'-p','relay_tracking:=true']),
        ExecuteProcess(cmd=[str(executable),'--ros-args','-p','real_sensor_inputs:=true',
            '-p','isolated_task_test:='+LaunchConfiguration('isolated_task_test',default='false').perform(context),
            '-p','path_guided:=true','-p','arrival_tolerance:=0.05',
            '-p','controller_config_file:='+str(ROOT/'config/legacy_apf_shadow.yaml')]),
        # Inherit the caller's domain for ALL processes. Never send just the
        # APF process to domain 0 while its sensors/planner remain isolated.
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('center_z'),
        DeclareLaunchArgument('isolated_task_test',default_value='false'),
        DeclareLaunchArgument('apf_executable',default_value=str(ROOT/'host_task_build/apf/legacy_apf_shadow')),
        OpaqueFunction(function=setup)])
