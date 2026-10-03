"""
方案B — RTAB-Map 视觉定位 + 回环（独立方案，不启动 cuVSLAM）

数据流:
  D435i 双目IR(+IMU) ─► rtabmap stereo_odometry ─► /visual_slam/tracking/odometry
                                                   └─ TF: odom → base_link
  D435i RGB-D ────────► RTAB-Map（建图/定位，读 maps/ 地图库）─► TF: map → odom

设计原则（2026-09-14 拆分，与 cuVSLAM 方案完全隔离）:
  - 里程计固定为 RTAB-Map 自带 stereo_odometry（CPU，ORB+GTSAM），不使用 cuVSLAM；
  - 不订阅 2D 激光（原 subscribe_scan 链路已随激光 SLAM 一并移除）；
  - 地图库路径/模式校验保持严格：localization 只读已存在的库，mapping 拒绝覆盖。

imu_source 显式选择（与方案A 同语义；none 时 subscribe_imu=false，不空等 IMU）:
  none（默认）| d435 = /camera/camera/imu | px4 = /camera/camera/imu_FC（宿主机 px4_imu_relay）

用法（容器内）:
  ros2 launch /workspaces/ros2_ws/launch/vslam_rtabmap.launch.py mode:=localization
  ros2 launch /workspaces/ros2_ws/launch/vslam_rtabmap.launch.py \
      mode:=mapping database_path:=/workspaces/ros2_ws/maps/drafts/rtabmap_test.db \
      rtabmap_args:=--delete_db_on_start
  ros2 launch /workspaces/ros2_ws/launch/vslam_rtabmap.launch.py mode:=odometry
      # 仅 stereo_odometry（无地图库），做 CPU 双目里程计基线

宿主机侧配套: scripts/run_slam_px4.sh SLAM_SCHEME=rtabmap RUN_MODE=production|mapping
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

_IMU_TOPICS = {'none': None, 'd435': '/camera/camera/imu', 'px4': '/camera/camera/imu_FC'}


def _rtabmap_include(context):
    """校验严格模式后生成 RTAB-Map include。"""
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    database_path = LaunchConfiguration('database_path').perform(context).strip()
    extra_args = LaunchConfiguration('rtabmap_args').perform(context).strip()

    if mode not in ('mapping', 'localization', 'odometry'):
        raise RuntimeError(
            "mode must be exactly 'mapping', 'localization' or 'odometry'")
    if mode == 'odometry':
        # 仅 CPU 双目里程计：不启动 RTAB-Map、不读/写地图库
        return []
    if not os.path.isabs(database_path):
        raise RuntimeError('database_path must be an absolute path')
    if mode == 'localization':
        if not os.path.isfile(database_path):
            raise RuntimeError(
                'localization database does not exist: {}'.format(database_path))
        if os.path.getsize(database_path) == 0:
            raise RuntimeError(
                'localization database is empty: {}'.format(database_path))
    elif os.path.exists(database_path) and '--delete_db_on_start' not in extra_args:
        raise RuntimeError(
            'mapping database path already exists (refusing to append): {}'.format(
                database_path))
    if '--delete_db_on_start' in extra_args and mode != 'mapping':
        raise RuntimeError(
            '--delete_db_on_start is only allowed in mapping mode')

    common = [
        '--Reg/Strategy', '2',
        '--Reg/Force3DoF', 'true',
        '--Optimizer/Slam2D', 'true',
        '--Grid/FromDepth', 'false',
        '--RGBD/ProximityBySpace', 'true',
        '--Icp/MaxCorrespondenceDistance', '0.20',
        '--Icp/CorrespondenceRatio', '0.20',
        '--Icp/Iterations', '20',
    ]
    if mode == 'mapping':
        common += [
            '--RGBD/NeighborLinkRefining', 'false',
            '--Mem/NotLinkedNodesKept', 'false',
            '--RGBD/LinearUpdate', '0.05',
            '--RGBD/AngularUpdate', '0.05',
            '--RGBD/ProximityPathMaxNeighbors', '10',
            '--Vis/MinInliers', '10',
            '--Vis/MinDepth', '0.3',
            '--Vis/MaxDepth', '4.0',
            '--Rtabmap/DetectionRate', '1.0',
        ]
    else:
        common += [
            '--Reg/Strategy', '1',
            '--RGBD/NeighborLinkRefining', 'true',
            '--RGBD/ProximityPathMaxNeighbors', '10',
            '--RGBD/ProximityMaxGraphDepth', '0',
            '--RGBD/ProximityAngle', '1.5708',
            '--RGBD/SavedLocalizationIgnored', 'true',
            '--Vis/MinInliers', '10',
            '--Icp/MaxCorrespondenceDistance', '0.40',
            '--Icp/CorrespondenceRatio', '0.05',
            '--Icp/MaxTranslation', '0.75',
            '--Icp/MaxRotation', '1.57',
            '--Icp/Iterations', '30',
            '--Rtabmap/DetectionRate', '2.0',
        ]
    if mode == 'localization':
        strict_mode = [
            '--Mem/IncrementalMemory', 'false',
            '--Mem/InitWMWithAllNodes', 'true',
            '--Mem/LocalizationReadOnly', 'true',
            '--Mem/LocalizationDataSaved', 'false',
        ]
    else:
        strict_mode = [
            '--Mem/IncrementalMemory', 'true',
            '--Mem/InitWMWithAllNodes', 'false',
            '--Mem/LocalizationReadOnly', 'false',
            '--Mem/LocalizationDataSaved', 'true',
        ]

    rtabmap_args = ' '.join(
        ([extra_args] if extra_args else []) + common + strict_mode)

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rtabmap_launch'),
                'launch',
                'rtabmap.launch.py',
            )
        ),
        launch_arguments={
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'odom_topic': '/visual_slam/tracking/odometry',
            'map_frame_id': 'map',
            'rgb_topic': '/camera/camera/color/image_raw',
            # 必须是与彩色对齐的深度：未对齐时特征多但几何内点=0（典型定位失败）
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'visual_odometry': 'false',
            'approx_sync': 'true',
            'rgbd_sync': 'true',
            'subscribe_scan': 'false',
            'subscribe_scan_cloud': 'false',
            'qos': '2',
            'qos_image': '2',
            'qos_camera_info': '2',
            'qos_odom': '2',
            'rtabmap_args': rtabmap_args,
            'localization': 'true' if mode == 'localization' else 'false',
            'database_path': database_path,
            'rviz': LaunchConfiguration('rviz'),
            'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
            'wait_for_transform': '0.5',
            'topic_queue_size': LaunchConfiguration('topic_queue_size'),
            'queue_size': LaunchConfiguration('sync_queue_size'),
            'approx_sync_max_interval': LaunchConfiguration('approx_sync_max_interval'),
        }.items(),
    )]


def _nodes(context):
    imu_source = LaunchConfiguration('imu_source').perform(context).strip().lower()
    if imu_source not in _IMU_TOPICS:
        raise RuntimeError("imu_source must be one of: none | d435 | px4")
    camera_imu = (imu_source == 'd435')
    subscribe_imu = (imu_source != 'none')
    imu_topic = _IMU_TOPICS[imu_source] or '/camera/camera/imu'

    _rsusb_lib = '/opt/realsense_rsusb/lib'
    _ld = os.environ.get('LD_LIBRARY_PATH', '')
    _env = {'LD_LIBRARY_PATH': _rsusb_lib + (':' + _ld if _ld else '')} \
        if os.path.isdir(_rsusb_lib) else {}

    realsense_node = Node(
        name='camera', namespace='camera',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        additional_env=_env,
        parameters=[{
            'serial_no': '_912112073953',
            'initial_reset': ParameterValue(
                LaunchConfiguration('d435_initial_reset'), value_type=bool),
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_color': True,
            'enable_depth': True,
            'depth_module.infra_profile': '640x480x30',
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'enable_sync': True,
            'align_depth.enable': True,
            'depth_module.emitter_enabled': ParameterValue(
                LaunchConfiguration('d435_emitter_enabled'), value_type=int),
            'depth_module.enable_auto_exposure': ParameterValue(
                LaunchConfiguration('d435_auto_exposure'), value_type=bool),
            'depth_module.exposure': ParameterValue(
                LaunchConfiguration('d435_exposure_us'), value_type=int),
            'depth_module.gain': ParameterValue(
                LaunchConfiguration('d435_gain'), value_type=int),
            'enable_gyro': camera_imu,
            'enable_accel': camera_imu,
            'gyro_fps': 200,
            'accel_fps': 250,
            'unite_imu_method': 2,
            'imu_frame_id': 'camera_imu_optical_frame',
        }],
    )

    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_link_to_camera_link',
        arguments=[
            '--x', LaunchConfiguration('camera_x'),
            '--y', LaunchConfiguration('camera_y'),
            '--z', LaunchConfiguration('camera_z'),
            '--roll', LaunchConfiguration('camera_roll'),
            '--pitch', LaunchConfiguration('camera_pitch'),
            '--yaw', LaunchConfiguration('camera_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
    )

    stereo_odom_node = Node(
        package='rtabmap_odom', executable='stereo_odometry',
        name='stereo_odometry', output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'approx_sync': True,
            'subscribe_imu': subscribe_imu,
        }],
        remappings=[
            ('left/image_rect', '/camera/camera/infra1/image_rect_raw'),
            ('right/image_rect', '/camera/camera/infra2/image_rect_raw'),
            ('left/camera_info', '/camera/camera/infra1/camera_info'),
            ('right/camera_info', '/camera/camera/infra2/camera_info'),
            ('imu', imu_topic),
            ('odom', '/visual_slam/tracking/odometry'),
        ],
    )

    return [realsense_node, static_tf, stereo_odom_node,
            OpaqueFunction(function=_rtabmap_include)]


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('rviz', default_value='true',
                              description='启动 RViz2'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              description='启动 rtabmap_viz（需 DISPLAY，无头环境保持 false）'),
        DeclareLaunchArgument('mode', default_value='localization',
                              description='严格模式：mapping 建图 / localization 只读定位 / '
                                          'odometry 仅 CPU 双目里程计（不启动 RTAB-Map）'),
        DeclareLaunchArgument(
            'database_path',
            default_value='/workspaces/ros2_ws/maps/production/rtabmap.db',
            description='RTAB-Map 地图库绝对路径'),
        DeclareLaunchArgument(
            'rtabmap_args', default_value='',
            description='额外 RTAB-Map 参数；--delete_db_on_start 仅 mapping 模式允许'),
        DeclareLaunchArgument(
            'imu_source', default_value='none',
            description='none=不用IMU(默认) | d435=D435i内置IMU | px4=飞控IMU(/camera/camera/imu_FC)'),
        DeclareLaunchArgument('topic_queue_size', default_value='30',
                              description='RTAB-Map 各话题订阅队列'),
        DeclareLaunchArgument('sync_queue_size', default_value='30',
                              description='RGB-D/里程计 approx-sync 队列'),
        DeclareLaunchArgument('approx_sync_max_interval', default_value='0.2',
                              description='跨传感器时间戳最大间隔(s)'),
        DeclareLaunchArgument('d435_initial_reset', default_value='false',
                              description='相机启动时复位；飞行中保持 false'),
        DeclareLaunchArgument('d435_emitter_enabled', default_value='0',
                              description='IR 投射器：1 增加室内纹理，0 关闭'),
        DeclareLaunchArgument('d435_auto_exposure', default_value='true',
                              description='IR 自动曝光；做振动 A/B 时置 0'),
        DeclareLaunchArgument('d435_exposure_us', default_value='8500',
                              description='关闭自动曝光时的 IR 曝光(µs)'),
        DeclareLaunchArgument('d435_gain', default_value='16',
                              description='关闭自动曝光时的 IR 增益'),
        DeclareLaunchArgument('camera_x', default_value='0.16',
                              description='D435i 相对 base_link 前向偏移(m)，2026-08-05 实测'),
        DeclareLaunchArgument('camera_y', default_value='0.0',
                              description='D435i 左向偏移(m)'),
        DeclareLaunchArgument('camera_z', default_value='-0.04',
                              description='D435i 上向偏移(m)'),
        DeclareLaunchArgument('camera_roll', default_value='-0.013094',
                              description='D435i roll(rad)，2026-08-05 标定'),
        DeclareLaunchArgument('camera_pitch', default_value='-0.052326',
                              description='D435i pitch(rad)，2026-08-05 标定'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0',
                              description='D435i yaw(rad)，未标定'),
    ]
    return LaunchDescription(declared_args + [OpaqueFunction(function=_nodes)])
