"""
Isaac ROS Visual SLAM + RTAB-Map with RealSense D435i

Data flow (use_isaac_vslam=true, default):
  D435i → infra1/infra2 → Isaac Visual SLAM → odometry (TF + /visual_slam/tracking/odometry)
       → color/depth    → RTAB-Map ────→ 3D map (using Visual SLAM odometry)

Data flow (use_isaac_vslam=false, last year's approach):
  D435i → infra1/infra2 → rtabmap_odom stereo_odometry → /visual_slam/tracking/odometry
       → color/depth    → RTAB-Map ────→ 3D map (using stereo_odometry)

Usage (inside container):
  source /opt/ros/humble/setup.bash
  ros2 launch /workspaces/ros2_ws/launch/slam_rtabmap.launch.py

Options:
  mode:=localization       Strict RTAB-Map mode: mapping or localization
  database_path:=...       Explicit database path
  rviz:=false              Disable RViz2
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def _rtabmap_include(context):
    """Build RTAB-Map arguments after validating the strict operating mode."""
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    database_path = LaunchConfiguration('database_path').perform(context).strip()
    extra_args = LaunchConfiguration('rtabmap_args').perform(context).strip()
    start_rtabmap = LaunchConfiguration('start_rtabmap').perform(context).strip().lower()

    # Allow skipping RTAB-Map entirely (debug mode: VSLAM only)
    if start_rtabmap in ('false', '0', 'no', 'off'):
        return []

    if mode not in ('mapping', 'localization'):
        raise RuntimeError("mode must be exactly 'mapping' or 'localization'")
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
    if '--delete_db_on_start' in extra_args:
        # Allowed only in mapping mode (边飞边建图)
        if mode != 'mapping':
            raise RuntimeError(
                '--delete_db_on_start is only allowed in mapping mode')

    # Common registration for a level 2D lidar on a UAV. RTAB-Map still
    # consumes RGB-D features, while ICP constrains horizontal drift.
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
        # Hand-held mapping: keep odom neighbor links, drop rehearsed orphans,
        # and avoid ICP neighbor refining (tilted uncompensated scans often
        # reject otherwise-valid links and leave isolated nodes in the DB).
        common += [
            '--RGBD/NeighborLinkRefining', 'false',
            '--Mem/NotLinkedNodesKept', 'false',
            '--RGBD/LinearUpdate', '0.05',
            '--RGBD/AngularUpdate', '0.05',
            '--RGBD/ProximityPathMaxNeighbors', '10',
            '--Vis/MinInliers', '10',
            # Ignore near-field noisy depth (desk/hands) like common D435 demos.
            '--Vis/MinDepth', '0.3',
            '--Vis/MaxDepth', '4.0',
            '--Rtabmap/DetectionRate', '1.0',
        ]
    else:
        # Localization: after a coarse initialpose, laser proximity/ICP locks
        # map->odom (Mathieu / Yahboom RTAB-Map workflow). Visual loop closure
        # alone is unreliable in sparse indoor texture.
        common += [
            '--Reg/Strategy', '1',
            '--RGBD/NeighborLinkRefining', 'true',
            '--RGBD/ProximityPathMaxNeighbors', '10',
            '--RGBD/ProximityMaxGraphDepth', '0',
            '--RGBD/ProximityAngle', '1.5708',
            '--RGBD/ProximityGlobalScanMap', 'true',
            '--RGBD/SavedLocalizationIgnored', 'true',
            '--Vis/MinInliers', '10',
            '--Icp/MaxCorrespondenceDistance', '0.40',
            '--Icp/CorrespondenceRatio', '0.05',
            # A hand-entered initial pose is only coarse. The default 0.2 m
            # correction gate rejects otherwise usable startup scan matches.
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

    # Strict mode arguments are last so legacy tuning cannot weaken read-only
    # localization or accidentally switch mapping semantics.
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
            'odom_topic': '/visual_slam/tracking/odometry',  # same topic for both Isaac VSLAM and stereo_odometry
            'map_frame_id': 'map',
            'rgb_topic': '/camera/camera/color/image_raw',
            # Must be depth aligned to color; unaligned depth gives many descriptor
            # matches but 0 geometric inliers (classic RTAB-Map localization failure).
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'visual_odometry': 'false',
            'approx_sync': 'true',
            'rgbd_sync': 'true',
            'subscribe_scan': LaunchConfiguration('subscribe_scan'),
            'subscribe_scan_cloud': 'false',
            'scan_topic': LaunchConfiguration('scan_topic'),
            'qos_scan': LaunchConfiguration('qos_scan'),
            # Camera/depth are BEST_EFFORT from realsense2_camera.
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


def generate_launch_description():
    # ── Launch arguments ──
    declared_args = [
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz2 with RTAB-Map',
        ),
        DeclareLaunchArgument(
            'rtabmap_viz', default_value='false',
            description='Launch rtabmap_viz GUI (needs DISPLAY; keep false for headless)',
        ),
        DeclareLaunchArgument(
            'mode', default_value='localization',
            description='Strict RTAB-Map mode: mapping or localization',
        ),
        DeclareLaunchArgument(
            'database_path',
            default_value='/workspaces/ros2_ws/maps/production/rtabmap.db',
            description='Absolute RTAB-Map database path',
        ),
        DeclareLaunchArgument(
            'rtabmap_args', default_value='',
            description='Optional RTAB-Map tuning; --delete_db_on_start allowed in mapping mode only',
        ),
        DeclareLaunchArgument(
            'start_rtabmap', default_value='true',
            description='Start RTAB-Map node (true/false). Set false for VSLAM-only debug mode.',
        ),
        DeclareLaunchArgument(
            'use_isaac_vslam', default_value='true',
            description='Use Isaac ROS Visual SLAM (true). '
                        'Set false to use RTAB-Map stereo_odometry instead '
                        '(last year approach — ORB features + GTSAM, CPU-based).',
        ),
        DeclareLaunchArgument(
            'subscribe_scan', default_value='true',
            description=(
                'Subscribe to 2D LaserScan, e.g. RPLidar /scan '
                '(must be true/false, not 1/0)'
            ),
        ),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan_planar',
            description='LaserScan topic used when subscribe_scan:=true',
        ),
        DeclareLaunchArgument(
            'qos_scan', default_value='2',
            description='RTAB-Map LaserScan QoS when subscribe_scan:=true',
        ),
        DeclareLaunchArgument(
            'topic_queue_size', default_value='30',
            description='Per-topic subscriber queue for RTAB-Map sync',
        ),
        DeclareLaunchArgument(
            'sync_queue_size', default_value='30',
            description='Approx-sync queue between RGB-D, scan and odometry',
        ),
        DeclareLaunchArgument(
            'approx_sync_max_interval', default_value='0.2',
            description='Max timestamp spread (s) for approx sync across sensors',
        ),
        DeclareLaunchArgument(
            'camera_x', default_value='0.16',
            description='D435i X offset from base_link (forward, 15-17cm measured)',
        ),
        DeclareLaunchArgument(
            'camera_y', default_value='0.0',
            description='D435i Y offset from base_link (left, metres)',
        ),
        DeclareLaunchArgument(
            'camera_z', default_value='-0.04',
            description='D435i Z offset from base_link (up, metres; ~3-5cm below CG)',
        ),
        DeclareLaunchArgument(
            'camera_roll', default_value='-0.013094',
            description='D435i roll relative to base_link (radians). '
                        'Calibrated 2026-08-05: python3 ~/ros2_ws/scripts/calibrate_camera_extrinsic.py --samples 3000',
        ),
        DeclareLaunchArgument(
            'camera_pitch', default_value='-0.052326',
            description='D435i pitch relative to base_link (radians). '
                        'Calibrated 2026-08-05: python3 ~/ros2_ws/scripts/calibrate_camera_extrinsic.py --samples 3000',
        ),
        DeclareLaunchArgument(
            'camera_yaw', default_value='0.0',
            description='D435i yaw relative to base_link (radians). '
                        'Note: yaw is NOT calibrated (rotate check only via verify_ev_yaw_align.sh --rotate)',
        ),
    ]

    # ── 1. RealSense D435i ──
    # infra1/infra2 → Visual SLAM, color/depth → RTAB-Map
    # Prefer RSUSB backend on JetPack 6 (kernel HID IMU is unreliable).
    _rsusb_lib = '/opt/realsense_rsusb/lib'
    _ld = os.environ.get('LD_LIBRARY_PATH', '')
    _realsense_env = {}
    if os.path.isdir(_rsusb_lib):
        _realsense_env['LD_LIBRARY_PATH'] = (
            _rsusb_lib + (':' + _ld if _ld else '')
        )

    realsense_node = Node(
        name='camera',
        namespace='camera',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        additional_env=_realsense_env,
        parameters=[{
            # Bind explicitly; avoids picking the wrong UVC node when Microdia is present.
            'serial_no': '_912112073953',
            'initial_reset': True,
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_color': True,
            'enable_depth': True,
            'depth_module.emitter_enabled': 0,
            # Infra/Depth share the depth module: resolutions must match or the
            # camera reports "Depth stream start failure" (seen with 480 infra +
            # 360 depth). Color may differ; aligned_depth is resampled to color.
            'depth_module.infra_profile': '640x480x30',
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'enable_sync': True,
            # Required for RTAB-Map: RGB features need depth in the SAME optical
            # frame. Without align_depth, matches>>0 but geometric inliers stay 0.
            # See RealSense wiki + RabbitRobot D435 RTAB-Map launch.
            'align_depth.enable': True,
            # D435i Motion Module: JetPack 6 + FW 5.17 broke HID IMU (no IIO frames).
            # Restored with FW 5.13.0.55 (+ optional RSUSB librealsense). Expect ~200 Hz IMU.
            'enable_gyro': True,
            'enable_accel': True,
            'gyro_fps': 200,
            'accel_fps': 63,
            'unite_imu_method': 2,
            'imu_frame_id': 'camera_imu_optical_frame',
        }],
    )

    # ── 2. Static TF: base_link → camera_link ──
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
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

    # ── 3. Isaac ROS Visual SLAM (GPU-accelerated, default) ──
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_image_denoising': True,
            'rectified_images': True,
            'enable_imu_fusion': True,
            'tracking_mode': 1,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'imu_frame': 'camera_imu_optical_frame',
            'imu_buffer_size': 100,
            'image_jitter_threshold_ms': 50.0,
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
            ('visual_slam/imu', '/camera/camera/imu'),
        ],
    )

    visual_slam_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_isaac_vslam')),
    )

    # ── 3b. RTAB-Map stereo odometry (CPU-based, fallback when use_isaac_vslam=false) ──
    # Uses ORB features + GTSAM. Publishes /visual_slam/tracking/odometry (same topic as cuVSLAM)
    # so the bridge and RTAB-Map can consume it without changes.
    stereo_odom_node = Node(
        package='rtabmap_odom',
        executable='stereo_odometry',
        name='stereo_odometry',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_isaac_vslam')),
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'approx_sync': True,
            'subscribe_imu': True,
        }],
        remappings=[
            ('left/image_rect', '/camera/camera/infra1/image_rect_raw'),
            ('right/image_rect', '/camera/camera/infra2/image_rect_raw'),
            ('left/camera_info', '/camera/camera/infra1/camera_info'),
            ('right/camera_info', '/camera/camera/infra2/camera_info'),
            ('imu', '/camera/camera/imu'),
            ('odom', '/visual_slam/tracking/odometry'),
        ],
    )

    return LaunchDescription(
        declared_args + [
            realsense_node,
            static_tf,
            visual_slam_container,
            stereo_odom_node,
            OpaqueFunction(function=_rtabmap_include),
        ]
    )
