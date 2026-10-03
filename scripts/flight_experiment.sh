#!/usr/bin/env bash
# Record one synchronized localization experiment. Start the SLAM/PX4 stack first.
# ROS 2 setup files reference optional variables and are not nounset-safe.
set -eo pipefail

ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
CONTAINER_NAME="${CONTAINER_NAME:-isaac_ros_dev}"
RATE="${EXPERIMENT_RATE_HZ:-20}"
RECORD_CAMERA_IMAGES="${RECORD_CAMERA_IMAGES:-0}"
RECORD_VSLAM_STATUS="${RECORD_VSLAM_STATUS:-1}"
RECORD_CAMERA_TIMING="${RECORD_CAMERA_TIMING:-1}"
LABEL="${1:-ground}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SAFE_LABEL="$(printf '%s' "$LABEL" | tr -cs 'A-Za-z0-9_.-' '_')"
EXPERIMENT_DIR="${EXPERIMENT_DIR:-$ROS2_WS/logs/experiment_${STAMP}_${SAFE_LABEL}}"
POSE_PID=""
BAG_PID=""
VSLAM_STATUS_PID=""
CAMERA_TIMING_PID=""

mkdir -p "$EXPERIMENT_DIR"

cleanup() {
    local status=$?
    trap - INT TERM EXIT
    for pid in "$BAG_PID" "$POSE_PID" "$VSLAM_STATUS_PID" "$CAMERA_TIMING_PID"; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null || true
        fi
    done
    wait "$BAG_PID" 2>/dev/null || true
    wait "$POSE_PID" 2>/dev/null || true
    {
        echo "finished_at=$(date --iso-8601=seconds)"
        echo "exit_status=$status"
    } >> "$EXPERIMENT_DIR/manifest.env"
    echo "Experiment saved: $EXPERIMENT_DIR"
    exit "$status"
}
trap cleanup INT TERM EXIT

source /opt/ros/humble/setup.bash
source "$ROS2_WS/install/setup.bash"

{
    echo "label=$LABEL"
    echo "started_at=$(date --iso-8601=seconds)"
    echo "hostname=$(hostname)"
    echo "kernel=$(uname -r)"
    echo "ros_distro=${ROS_DISTRO:-}"
    echo "rmw_implementation=${RMW_IMPLEMENTATION:-}"
    echo "run_mode=${RUN_MODE:-production}"
    echo "slam_scheme=${SLAM_SCHEME:-cuvslam}"
    echo "imu_source=${IMU_SOURCE:-none}"
    echo "reset_realsense_usb=${RESET_REALSENSE_USB:-0}"
    echo "d435_initial_reset=${RESET_REALSENSE_USB:-0}"
    echo "d435_emitter_enabled=${D435_EMITTER_ENABLED:-1}"
    echo "vslam_enable_denoising=${VSLAM_ENABLE_DENOISING:-false}"
    echo "vslam_tracking_mode=${VSLAM_TRACKING_MODE:-1}"
    echo "vslam_image_jitter_ms=${VSLAM_IMAGE_JITTER_MS:-70.0}"
    echo "use_rtabmap_relay=${USE_RTABMAP_RELAY:-1}"
    echo "record_camera_images=$RECORD_CAMERA_IMAGES"
    echo "record_vslam_status=$RECORD_VSLAM_STATUS"
    echo "record_camera_timing=$RECORD_CAMERA_TIMING"
    echo "position_variance_xy=${POSITION_VARIANCE_XY:-0.05}"
    echo "velocity_variance=${VELOCITY_VARIANCE:-0.08}"
} > "$EXPERIMENT_DIR/manifest.env"

env | sort > "$EXPERIMENT_DIR/environment.txt"
ros2 node list > "$EXPERIMENT_DIR/nodes.txt" 2>&1 || true
ros2 topic list -t > "$EXPERIMENT_DIR/topics.txt" 2>&1 || true
ros2 param list > "$EXPERIMENT_DIR/ros_parameters.txt" 2>&1 || true
mkdir -p "$EXPERIMENT_DIR/parameter_dumps"
# `ros2 param list` only records parameter names. Persist actual values for
# every localization node that is discoverable in this ROS domain so a replay
# can distinguish a launch default from the configuration that really ran.
for node in /camera/camera /visual_slam_node /stereo_odometry /vslam_odom_bridge \
            /rtabmap /rtabmap_odom_relay /state_bridge; do
    safe_node="${node#/}"
    safe_node="${safe_node//\//_}"
    ros2 param dump "$node" > "$EXPERIMENT_DIR/parameter_dumps/${safe_node}.yaml" \
        2> "$EXPERIMENT_DIR/parameter_dumps/${safe_node}.stderr" || true
done
cp "$ROS2_WS/config/px4_sdcard/etc/config.txt" \
   "$EXPERIMENT_DIR/expected_px4_config.txt"

if git -C "$ROS2_WS" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    git -C "$ROS2_WS" rev-parse HEAD > "$EXPERIMENT_DIR/git_commit.txt"
    git -C "$ROS2_WS" status --short > "$EXPERIMENT_DIR/git_status.txt"
    git -C "$ROS2_WS" diff > "$EXPERIMENT_DIR/uncommitted.diff"
fi

python3 "$ROS2_WS/scripts/pose_logger.py" \
    --rate "$RATE" --dir "$EXPERIMENT_DIR" \
    > "$EXPERIMENT_DIR/pose_logger.log" 2>&1 &
POSE_PID=$!

TOPICS=(
    /visual_slam/tracking/odometry
    /rtabmap/relay/odometry
    /rtabmap/localization_pose
    /rtabmap/info
    /rtabmap/relay/status
    /scan
    /camera/camera/imu
    /camera/camera/infra1/camera_info
    /camera/camera/infra2/camera_info
    /tf
    /tf_static
    /fmu/in/vehicle_visual_odometry
    /fmu/in/trajectory_setpoint
    /fmu/out/vehicle_odometry
    /fmu/out/vehicle_local_position
    /fmu/out/vehicle_attitude
    /fmu/out/vehicle_status_v1
    /fmu/out/estimator_status_flags
    /fmu/out/distance_sensor
    /fmu/out/timesync_status
    /fmu/out/sensor_combined
    /fmu/out/actuator_motors
    /uav/state/pose
)

# Recording raw 640x480 images can starve the Jetson USB/ROS pipeline and
# create the very frame gaps this experiment is intended to diagnose.  Keep
# images opt-in; pose_logger and camera_info/IMU remain lightweight by default.
if [ "$RECORD_CAMERA_IMAGES" = "1" ]; then
    TOPICS+=(
        /camera/camera/infra1/image_rect_raw
        /camera/camera/infra2/image_rect_raw
    )
fi

# The Isaac container owns this interface package, so the host recorder cannot
# deserialize it unless the package is explicitly installed on the host.
if [ "$RECORD_VSLAM_STATUS" = "1" ] && \
   ros2 pkg prefix isaac_ros_visual_slam_interfaces >/dev/null 2>&1; then
    TOPICS+=(/visual_slam/status)
fi

# Isaac ROS is container-owned; the host cannot deserialize this interface.
# Capture its textual status inside the container instead of silently dropping
# the topic from rosbag.  The process is optional and cleaned up with the bag.
if [ "$RECORD_VSLAM_STATUS" = "1" ] && command -v docker >/dev/null 2>&1 \
   && docker ps --format '{{.Names}}' | grep -qx isaac_ros_dev; then
    docker exec "$CONTAINER_NAME" bash -lc \
        'source /opt/ros/humble/setup.bash; ros2 topic echo /visual_slam/status' \
        > "$EXPERIMENT_DIR/vslam_status.txt" 2>&1 &
    VSLAM_STATUS_PID=$!
fi

if [ "$RECORD_CAMERA_TIMING" = "1" ]; then
    python3 "$ROS2_WS/scripts/camera_timing_logger.py" \
        --output "$EXPERIMENT_DIR/camera_timing.csv" \
        > "$EXPERIMENT_DIR/camera_timing.log" 2>&1 &
    CAMERA_TIMING_PID=$!
fi

ros2 bag record --include-unpublished-topics \
    --qos-profile-overrides-path "$ROS2_WS/config/rosbag_sensor_qos.yaml" \
    --output "$EXPERIMENT_DIR/rosbag" "${TOPICS[@]}" \
    > "$EXPERIMENT_DIR/rosbag.log" 2>&1 &
BAG_PID=$!

echo "Recording experiment '$LABEL' in $EXPERIMENT_DIR"
echo "PX4 ULog is stored on the flight-controller SD card; copy the matching .ulg here."
echo "Press Ctrl+C after the test."

while kill -0 "$POSE_PID" 2>/dev/null && kill -0 "$BAG_PID" 2>/dev/null; do
    sleep 1
done
echo "A recorder exited unexpectedly." >&2
exit 1
