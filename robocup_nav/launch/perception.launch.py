"""Container: one D435i driver + cuVSLAM, no PX4 publishers and no EEPROM writes."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

ROOT = Path(__file__).resolve().parents[1]


def nodes(context):
    p = yaml.safe_load((ROOT / 'config/platform.yaml').read_text())
    mapping = LaunchConfiguration('rgbd').perform(context).lower() == 'true'
    depth_only = LaunchConfiguration('nvblox_depth').perform(context).lower() == 'true'
    publish_map_tf = LaunchConfiguration('publish_map_tf').perform(context).lower() == 'true'
    imu_fusion = LaunchConfiguration('imu_fusion').perform(context).lower() == 'true'
    imu_unite_method = int(LaunchConfiguration('imu_unite_method').perform(context))
    transport_profile = LaunchConfiguration('image_transport_profile').perform(context)
    if transport_profile not in ('unchanged', 'shm16m'):
        raise RuntimeError('image_transport_profile must be unchanged or shm16m')
    transport_env = None
    if transport_profile == 'shm16m':
        profile_path = ROOT / 'config/perception_dds_shm16m.xml'
        if not profile_path.is_file():
            raise RuntimeError('missing reviewed image transport candidate')
        # Process-local only. Does not change Agent/PX4 or unrelated ROS domains.
        transport_env = {'FASTRTPS_DEFAULT_PROFILES_FILE': str(profile_path),
                         'FASTDDS_DEFAULT_PROFILES_FILE': str(profile_path)}
    if imu_unite_method not in (1, 2):
        raise RuntimeError('imu_unite_method must be 1 (copy) or 2 (linear interpolation)')
    pose = p['camera_xyz_rpy']
    delayed = ['python3', str(ROOT / 'scripts/realsense_imu_then_ir.py'),
               '--enable-gyro', '--enable-accel']
    camera_params = {
                 'serial_no': '_912112073953', 'initial_reset': False,
                 # Hard constraint: IR first. IMU is enabled only after infra frames.
                 'enable_infra1': True, 'enable_infra2': True,
                 'depth_module.infra_profile': '640x480x30',
                 'enable_accel': False, 'enable_gyro': False,
                 'accel_fps': 250, 'gyro_fps': 200, 'unite_imu_method': imu_unite_method,
                 'enable_color': mapping, 'enable_depth': mapping or depth_only,
                 'rgb_camera.color_profile': '640x480x15',
                 'depth_module.depth_profile': '640x480x30',
                 'align_depth.enable': mapping, 'pointcloud.enable': False,
                 'depth_module.emitter_enabled': 0,
    }
    vision_on = camera_params['enable_infra1'] or camera_params['enable_infra2'] or camera_params['enable_depth'] or camera_params['enable_color']
    imu_on = camera_params['enable_accel'] or camera_params['enable_gyro']
    if vision_on and imu_on:
        raise RuntimeError(
            'D435i hard constraint violated: do not start IR/depth/color and IMU together. '
            'Start infra first; enable gyro/accel only after IR frames.')
    return [
        Node(package='realsense2_camera', executable='realsense2_camera_node',
             name='camera', namespace='camera', parameters=[camera_params], additional_env=transport_env),
        ExecuteProcess(cmd=delayed, output='screen', name='realsense_ir_then_imu',
                       additional_env={'PYTHONUNBUFFERED': '1'}),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='robocup_camera_extrinsics', arguments=[
                 '--x', str(pose[0]), '--y', str(pose[1]), '--z', str(pose[2]),
                 '--roll', str(pose[3]), '--pitch', str(pose[4]), '--yaw', str(pose[5]),
                 '--frame-id', 'base_link', '--child-frame-id', 'camera_link']),
        ComposableNodeContainer(name='robocup_vio_container', namespace='',
            package='rclcpp_components', executable='component_container',
            additional_env=transport_env,
            composable_node_descriptions=[ComposableNode(
                package='isaac_ros_visual_slam', plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam_node', parameters=[{
                    'rectified_images': True, 'enable_imu_fusion': imu_fusion,
                    'base_frame': 'base_link', 'odom_frame': 'odom',
                    # NVIDIA's RealSense reference launch uses the gyro optical
                    # frame for cuVSLAM, even when the merged IMU header uses
                    # camera_imu_optical_frame. Keep this even when IMU fusion is
                    # off so a later fused diagnostic does not silently revert.
                    'imu_frame': 'camera_gyro_optical_frame',
                    'camera_optical_frames': ['camera_infra1_optical_frame', 'camera_infra2_optical_frame'],
                    'publish_map_to_odom_tf': publish_map_tf, 'publish_odom_to_base_tf': True,
                    # PX4 needs continuous odometry. Keep cuVSLAM relocalization/
                    # mapping disabled so returning to a similar view cannot
                    # legitimately rewrite the odometry frame.
                    'enable_localization_n_mapping': False,
                    'enable_slam_visualization': False, 'enable_landmarks_view': False,
                    'enable_observations_view': False, 'enable_image_denoising': False,
                    'image_jitter_threshold_ms': 40.0,
                    'gyro_noise_density': 0.000244, 'gyro_random_walk': 0.000019393,
                    'accel_noise_density': 0.001862, 'accel_random_walk': 0.003,
                    'calibration_frequency': 200.0,
                }], remappings=[
                    ('visual_slam/image_0', '/camera/camera/infra1/image_rect_raw'),
                    ('visual_slam/image_1', '/camera/camera/infra2/image_rect_raw'),
                    ('visual_slam/camera_info_0', '/camera/camera/infra1/camera_info'),
                    ('visual_slam/camera_info_1', '/camera/camera/infra2/camera_info'),
                    ('visual_slam/imu', '/camera/camera/imu'),
                ])]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rgbd', default_value='false'),
        DeclareLaunchArgument('nvblox_depth', default_value='false'),
        DeclareLaunchArgument('publish_map_tf', default_value='false'),
        # Production candidate is D435i stereo visual-only plus PX4 IMU.
        # Fused D435i IMU failed rotational closure (1.2 m jump). Keep IMU
        # acquisition; do not feed it to cuVSLAM unless a fused diagnostic is
        # requested with imu_fusion:=true.
        DeclareLaunchArgument('imu_fusion', default_value='false'),
        DeclareLaunchArgument('imu_unite_method', default_value='2'),
        DeclareLaunchArgument('image_transport_profile', default_value='unchanged'),
        OpaqueFunction(function=nodes)])
