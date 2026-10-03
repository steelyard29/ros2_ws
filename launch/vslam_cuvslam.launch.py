"""
方案A — cuVSLAM 纯视觉惯性里程计（独立方案，不启动 RTAB-Map）

数据流:
  D435i 双IR(±IMU) ──► Isaac ROS cuVSLAM ──► /visual_slam/tracking/odometry
                                             └─ TF: odom → base_link(camera 挂载)

设计原则（2026-09-14 拆分，与 RTAB-Map 方案完全隔离）:
  - 不启动 RTAB-Map、不读地图库、不订阅 2D 激光；
  - IMU 来源用单一参数 `imu_source` 显式选择，杜绝历史上
    "enable_imu_fusion=true 但 IMU 流关闭 → 找不到 camera_imu_optical_frame
     → component_container abort(-6)" 的组合（2026-09-02 故障根因）。

  imu_source:=none   纯双目（默认；2026-09-02 已验证 28.7 Hz 稳定出流）
  imu_source:=d435   D435i 内置 IMU（2026-09-14 修复+标定；图像-IMU 硬件同步）
  imu_source:=px4    飞控 IMU，需宿主机 px4_imu_relay 转发到 /camera/camera/imu_FC

用法（容器内）:
  ros2 launch /workspaces/ros2_ws/launch/vslam_cuvslam.launch.py
  ros2 launch /workspaces/ros2_ws/launch/vslam_cuvslam.launch.py imu_source:=d435

宿主机侧配套: scripts/run_slam_px4.sh SLAM_SCHEME=cuvslam
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

# IMU 话题 / 是否需要相机自带 IMU 流
_IMU_TOPICS = {'none': None, 'd435': '/camera/camera/imu', 'px4': '/camera/camera/imu_FC'}


def _nodes(context):
    imu_source = LaunchConfiguration('imu_source').perform(context).strip().lower()
    if imu_source not in _IMU_TOPICS:
        raise RuntimeError("imu_source must be one of: none | d435 | px4")
    camera_imu = (imu_source == 'd435')
    imu_fusion = (imu_source != 'none')

    # ── 1. RealSense D435i（只开双目 IR；RGB-D 属于 RTAB-Map 方案）──
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
            'enable_color': False,
            'enable_depth': False,
            'depth_module.infra_profile': '640x480x30',
            'depth_module.emitter_enabled': ParameterValue(
                LaunchConfiguration('d435_emitter_enabled'), value_type=int),
            'depth_module.enable_auto_exposure': ParameterValue(
                LaunchConfiguration('d435_auto_exposure'), value_type=bool),
            'depth_module.exposure': ParameterValue(
                LaunchConfiguration('d435_exposure_us'), value_type=int),
            'depth_module.gain': ParameterValue(
                LaunchConfiguration('d435_gain'), value_type=int),
            # IMU 流只在 imu_source=d435 时打开（BMI055: accel 仅 63/250 Hz）
            'enable_gyro': camera_imu,
            'enable_accel': camera_imu,
            'gyro_fps': 200,
            'accel_fps': 250,
            'unite_imu_method': 2,
            'imu_frame_id': 'camera_imu_optical_frame',
        }],
    )

    # ── 2. 相机外参（2026-08-05 标定）──
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

    nodes = [realsense_node, static_tf]

    # px4 IMU: 相机不发 IMU TF，需补 base_link→camera_imu_optical_frame（飞控 IMU 在自稳位置）
    if imu_source == 'px4':
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_link_to_camera_imu_optical_frame',
            arguments=[
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_imu_optical_frame',
            ],
        ))

    # ── 3. Isaac ROS cuVSLAM ──
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_image_denoising': ParameterValue(
                LaunchConfiguration('vslam_enable_denoising'), value_type=bool),
            'rectified_images': True,
            'enable_imu_fusion': imu_fusion,
            'tracking_mode': ParameterValue(
                LaunchConfiguration('vslam_tracking_mode'), value_type=int),
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'imu_frame': 'camera_imu_optical_frame',
            'imu_buffer_size': 100,
            'image_jitter_threshold_ms': ParameterValue(
                LaunchConfiguration('vslam_image_jitter_ms'), value_type=float),
            'base_frame': 'base_link',
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'camera_optical_frames': [
                'camera_infra1_optical_frame',
                'camera_infra2_optical_frame',
            ],
        }],
        remappings=[
            ('visual_slam/image_0', '/camera/camera/infra1/image_rect_raw'),
            ('visual_slam/camera_info_0', '/camera/camera/infra1/camera_info'),
            ('visual_slam/image_1', '/camera/camera/infra2/image_rect_raw'),
            ('visual_slam/camera_info_1', '/camera/camera/infra2/camera_info'),
            # none 时该话题无发布者，节点不会等待（融合已关）
            ('visual_slam/imu',
             _IMU_TOPICS[imu_source] or '/camera/camera/imu'),
        ],
    )

    nodes.append(ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    ))
    return nodes


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument(
            'imu_source', default_value='none',
            description='none=纯双目(默认) | d435=D435i内置IMU | px4=飞控IMU(/camera/camera/imu_FC)',
        ),
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
        DeclareLaunchArgument('vslam_enable_denoising', default_value='false',
                              description='cuVSLAM 图像去噪（NVIDIA RealSense 默认关）'),
        DeclareLaunchArgument('vslam_tracking_mode', default_value='1',
                              description='cuVSLAM 0=纯双目, 1=双目+IMU VIO, 2=RGB-D'),
        DeclareLaunchArgument('vslam_image_jitter_ms', default_value='70.0',
                              description='允许的双目帧间隔上限(ms)；30 Hz + 一帧抖动'),
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
