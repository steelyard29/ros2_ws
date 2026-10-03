 #!/bin/bash

# 启动脚本：同时运行Realsense相机和RTAB-Map Docker容器

# 设置环境变量
export DISPLAY=${DISPLAY}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 创建rtabmap工作目录（如果不存在）
mkdir -p ~/rtabmap_docker_ws

# 启动MicroXRCEAgent（后台运行）
echo "启动MicroXRCEAgent..."
MicroXRCEAgent serial -D /dev/ttyTHS1 -b 921600 &
MICROXRCE_PID=$!
echo "MicroXRCEAgent启动，进程ID: $MICROXRCE_PID"

# 等待MicroXRCEAgent启动
sleep 2

# 启动Realsense相机节点（后台运行）
echo "启动Realsense相机节点..."
source install/setup.bash
ros2 launch realsense2_camera imu_rtabmap.launch.py &

# 获取Realsense进程ID
REALSENSE_PID=$!
echo "Realsense相机节点启动，进程ID: $REALSENSE_PID"

# 等待相机节点完全启动
sleep 5

# 启动RTAB-Map Docker容器
echo "启动RTAB-Map Docker容器..."
xhost +

docker run -it --rm \
    --net=host \
    --runtime=nvidia \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v ~/ros2_ws:/root/my_ros2_ws \
    my_rtab:v1.1 \
    bash

# 当Docker容器退出后，清理Realsense进程
echo "清理Realsense相机节点进程..."
kill $REALSENSE_PID 2>/dev/null
echo "脚本执行完成"
ros2 launch rtabmap_launch rtabmap.launch.py \
    stereo:=true \
    rgbd_sync:=false \
    frame_id:='base_link' \
    left_image_topic:=/camera/d435i/infra1/image_rect_raw \
    right_image_topic:=/camera/d435i/infra2/image_rect_raw \
    left_camera_info_topic:=/camera/d435i/infra1/camera_info \
    right_camera_info_topic:=/camera/d435i/infra2/camera_info \
    approx_sync:=true \
    Vis/MaxFeatures:=3000 \
    OdomF2M/GuessMotion:=true\
    Vis/UseIMU:=true\
    rtabmap_viz:=false \
    subscribe_scan:=true \
    scan_topic:=/scan \
    imu_topic:=/imu/data \
    rviz:=false \
    subscribe_imu:=true \
    use_sim_time:=false \
    qos_image:=2 \
    qos_camera_info:=2 \
    qos_imu:=2 \
    qos_odom:=2 \
    qos_odom_info:=2 \
    qos_scan:=2 \
    Odom/Strategy:=2 \
    Odom/GuessMotion:=true \
    Vis/CorType:=1



ros2 launch rtabmap_launch rtabmap.launch.py \
    stereo:=false \
    rgb_topic:=/camera/d435i/color/image_raw \
    depth_topic:=/camera/d435i/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/d435i/color/camera_info \
    frame_id:='base_link' \
    approx_sync:=true \
    imu_topic:=/imu/data \
    subscribe_imu:=true \
    rtabmap_viz:=false \
    rviz:=false \
    use_sim_time:=false \
    qos:=2
    ros2 run rtabmap_odom stereo_odometry \
  --ros-args \
  -p frame_id:=base_link \
  -p odom_frame_id:=odom_visual \
  -p publish_tf:=false \
  -p approx_sync:=true \
  -r left/image_rect:=/camera/d435i/infra1/image_rect_raw \
  -r right/image_rect:=/camera/d435i/infra2/image_rect_raw \
  -r left/camera_info:=/camera/d435i/infra1/camera_info \
  -r right/camera_info:=/camera/d435i/infra2/camera_info \
  -r imu:=/camera/d435i/imu \
  -r odom:=/odom_visual

  ros2 launch rtabmap_launch rtabmap.launch.py \
    stereo:=false \
    rgbd_sync:=true \
    frame_id:='base_link' \
    rgb_topic:=/camera/d435i/color/image_raw \
    depth_topic:=/camera/d435i/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/d435i/aligned_depth_to_color/camera_info \
    approx_sync:=true \
    subscribe_imu:=true \
    imu_topic:=/imu/data \
    rtabmap_viz:=false \
    Odom/FeatureType:=6 \
    Vis/MaxFeatures:=1000 \
    OdomF2M/GuessMotion:=true \
    subscribe_scan:=true \
    scan_topic:=/scan \
    rviz:=false \
    use_sim_time:=false \
    qos_image:=1 \
    qos_depth:=1 \
    qos_scan:=2 \
    qos_odom:=2 \
    qos_imu:=1 

ros2 launch rtabmap_launch rtabmap.launch.py \
    depth:=false \
    subscribe_rgb:=false \
    subscribe_scan:=true \
    visual_odometry:=false \
    odom_topic:=/t265/pose/sample \
    scan_topic:=/scan \
    approx_sync:=true \
    frame_id:='base_link' \
    odom_frame_id:='odom_frame' \
    qos_scan:=2 \
    qos_odom:=2 \
    rtabmap_viz:=false \
    rviz:=false \
    odom_sensor_sync:=true \
    sync_queue_size:=30 \
    topic_queue_size:=30 \
    rtabmap_args:="--delete_db_on_start \
        --Mem/UseOdomGravity false \
        --Reg/Strategy 1 \
        --Grid/FromDepth false \
        --Rtabmap/DetectionRate 1 \
        --RGBD/ProximityBySpace true \
        --Grid/RangeMax 10.0 \
        --Registration/RangeMin 0.35 \
        --Grid/RangeMin 0.35"

