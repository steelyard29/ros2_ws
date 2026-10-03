"""Explicit ground-start perception/map/APF candidate profile; no PX4 writer.

Starts sensors when an operator invokes it. Not run by offline tests.
initial_z MUST be the ground base_link height in the current VIO odom session.
Never reuses the old 0.6m/0.15m demonstration defaults for this flight profile.
"""
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from task_map_profile import mapper_profile


def setup(context):
    profile=mapper_profile(float(LaunchConfiguration('initial_z').perform(context)))
    def include(name,args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(ROOT/'launch'/name)),
            launch_arguments={k:str(v) for k,v in args.items()}.items())
    return [include('perception.launch.py',dict(nvblox_depth='true',publish_map_tf='true',imu_fusion='false')),
        include('nvblox.launch.py',profile),
        include('task_navigation.launch.py',dict(center_z=profile['center_z'],
            apf_executable=LaunchConfiguration('apf_executable').perform(context)))]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('initial_z'),
        DeclareLaunchArgument('apf_executable',default_value=str(ROOT/'host_task_build/apf/legacy_apf_shadow')),
        OpaqueFunction(function=setup)])
