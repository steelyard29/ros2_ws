import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# FastDDS XML 配置：解决 Docker host networking 下的 DDS 数据通道问题
FASTRTPS_XML = os.path.expanduser(
    '~/ros2_ws/config/fastdds_bridge.xml')


def launch_nodes(context, *args, **kwargs):
    start_agent = LaunchConfiguration('start_microxrce_agent').perform(
        context).lower() in ('1', 'true', 'yes', 'on')

    # MicroXRCEAgent: 串口连接 PX4
    microxrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'serial',
             '-D', LaunchConfiguration('px4_serial_device'),
             '-b', LaunchConfiguration('px4_serial_baud')],
        name='microxrce_agent',
        output='log',
    )

    # PX4 Gateway: 已有的 PX4 通信网关
    px4_gateway = Node(
        package='px4_interface',
        executable='px4_gateway_node',
        name='px4_gateway_node',
        output='log',
    )

    # VSLAM Odometry Bridge: VSLAM 里程计 → PX4 VehicleOdometry
    vslam_bridge = Node(
        package='px4_interface',
        executable='vslam_odom_bridge_node',
        name='vslam_odom_bridge',
        output='screen',
        parameters=[{
            'vslam_odom_topic': LaunchConfiguration('vslam_odom_topic'),
            'vehicle_status_topic': '/fmu/out/vehicle_status_v1',
            'px4_odom_topic': '/fmu/in/vehicle_visual_odometry',
            'publish_rate_hz': LaunchConfiguration('publish_rate'),
            # Raise XY variance: Isaac VSLAM often slides ~0.5–0.7m in climb while
            # quality stays 100; let EKF trust IMU+EV velocity more than absolute XY.
            # Tight variance: EKF trusts VSLAM/RTAB-Map position on ground/hover
            # to prevent EKF drift when VSLAM is stable.
            # 0.09→0.25→0.10→0.05: 0.10 still caused 7cm EKF ground drift in 19s.
            # Trade-off: tighter = less ground drift but more climb-following.
            # Mitigated by faster climb (0.15→0.25 m/s) to reduce drift duration.
            'position_variance_xy': LaunchConfiguration('position_variance_xy'),
            # Keep VSLAM vertical aiding conservative; TFmini remains HGT_REF.
            'position_variance_z': LaunchConfiguration('position_variance_z'),
            'orientation_variance': LaunchConfiguration('orientation_variance'),
            # Softer yaw for future EV yaw fusion (EKF2_EVA_NOISE ≈ 0.25).
            'orientation_variance_yaw': LaunchConfiguration('orientation_variance_yaw'),
            'velocity_variance': LaunchConfiguration('velocity_variance'),
            'rebase_to_initial_pose': True,
            'align_yaw_to_px4': True,
            # FRD pose + BODY_FRD velocity: matching frames for EV_CTRL=15.
            # With EV_CTRL=15 (full DOF), EKF fuses both position+velocity.
            'pose_frame': 'frd',
            'publish_local_ned_velocity': False,
            'flatten_ev_vertical_position': False,
            # 200ms was too tight under Jetson load → intermittent EV pause.
            'stale_timeout_ms': 400,
            'stabilization_samples': 10,
            'min_publish_quality': 50,
            # A 0.8 m threshold allowed the measured 0.3–0.8 m cuVSLAM
            # relocalization jumps to reach EKF.  Competition flight speeds
            # are well below the resulting 0.3 m/sample bound at 30 Hz.
            'jump_threshold_m': 5.0,
            'jump_threshold_deg': 30.0,
            'max_sample_gap_s': 2.0,
        }],
    )

    # State Bridge: PX4 原始数据 → /uav/state 标准化接口 (Layer 1→2/3)
    state_bridge = Node(
        package='px4_interface',
        executable='state_bridge.py',
        name='state_bridge',
        output='screen',
    )

    # RTAB-Map Odom Relay: 融合 RTAB-Map 回环修正 XY + VSLAM 速度
    # 启用时需同时设置 use_rtabmap_relay:=true vslam_odom_topic:=/rtabmap/relay/odometry
    rtabmap_relay = Node(
        package='px4_interface',
        executable='rtabmap_odom_relay.py',
        name='rtabmap_odom_relay',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rtabmap_relay')),
    )

    if not start_agent:
        return [px4_gateway, vslam_bridge, state_bridge, rtabmap_relay]

    return [
        microxrce_agent,

        # Agent 启动后再启动 Gateway
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=microxrce_agent,
                on_start=[px4_gateway],
            )),

        # Gateway 启动后再启动 VSLAM Bridge
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=px4_gateway,
                on_start=[vslam_bridge],
            )),

        # VSLAM Bridge 启动后再启动 State Bridge
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=vslam_bridge,
                on_start=[state_bridge],
            )),

        # RTAB-Map Odom Relay: 最后启动, 不依赖其他节点
        rtabmap_relay,
    ]


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument(
            'vslam_odom_topic',
            default_value='/visual_slam/tracking/odometry',
            description='VSLAM odometry topic (nav_msgs/Odometry)'),
        DeclareLaunchArgument(
            'use_rtabmap_relay',
            default_value='false',
            description='Use RTAB-Map relay node to fuse /rtabmap/localization_pose '
                        '(loop-closure-corrected XY) with VSLAM velocity. '
                        'When enabled, vslam_odom_topic should point to '
                        '/rtabmap/relay/odometry.'),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='30',
            description='Bridge publish rate in Hz'),
        DeclareLaunchArgument(
            'px4_serial_device',
            default_value='/dev/ttyTHS1',
            description='PX4 serial device (ttyTHS1=TELEM1 UART, ttyACM0=USB)'),
        DeclareLaunchArgument(
            'px4_serial_baud',
            default_value='921600',
            description='PX4 serial baud rate'),
        DeclareLaunchArgument(
            'start_microxrce_agent',
            default_value='true',
            description='Start MicroXRCEAgent inside this launch file'),
        DeclareLaunchArgument(
            'position_variance_xy',
            default_value='0.05',
            description='EV position XY variance (lower = EKF trusts more)'),
        DeclareLaunchArgument(
            'position_variance_z',
            default_value='0.25',
            description='EV position Z variance fallback (TFmini: 0.02 when live)'),
        DeclareLaunchArgument(
            'orientation_variance',
            default_value='0.01',
            description='EV orientation variance (roll/pitch)'),
        DeclareLaunchArgument(
            'orientation_variance_yaw',
            default_value='0.06',
            description='EV orientation variance (yaw)'),
        DeclareLaunchArgument(
            'velocity_variance',
            default_value='0.08',
            description='EV velocity variance'),
    ]

    # 设置 FastDDS 配置和 RMW
    set_rmw = SetEnvironmentVariable(
        'RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    set_fastrtps_xml = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE', FASTRTPS_XML)

    return LaunchDescription(
        declared_args + [
            set_rmw,
            set_fastrtps_xml,
            OpaqueFunction(function=launch_nodes),
        ])
