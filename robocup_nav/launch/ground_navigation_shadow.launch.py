"""Ground-start perception and cruise-height SHADOW navigation, no PX4 input.

Uses provisional UNLOADED geometry only; never a launch authorization.
VIO initial base_link z must be confirmed near initial_z. Unknown cells remain
unknown: no invented clearing bubble under the camera or behind obstacles.
"""
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from cruise_band import make_band


def setup(context):
    band=make_band(initial_z=float(LaunchConfiguration('initial_z').perform(context)),
                   rise=float(LaunchConfiguration('cruise_rise').perform(context)))
    def include(name,args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(ROOT/'launch'/name)),
                                        launch_arguments=args.items())
    return [include('perception.launch.py',{'nvblox_depth':'true','publish_map_tf':'true','imu_fusion':'false'}),
            include('nvblox.launch.py',{'center_z':str(band.center),
                'band_below':str(band.center-band.lower),'band_above':str(band.upper-band.center)}),
            include('nvblox_navigation.launch.py',{'center_z':str(band.center),'trial_profile':'true'})]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('initial_z',default_value='0.0'),
        DeclareLaunchArgument('cruise_rise',default_value='0.6'),OpaqueFunction(function=setup)])
