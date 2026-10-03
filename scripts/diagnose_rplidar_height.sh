#!/bin/bash
# Ground-only diagnostics for RPLidarA2 scan usage and PX4/VSLAM height sources.
#
# This script does not arm or take off. It only inspects devices, ROS nodes,
# topic endpoints, /scan health, and Z/height values from the current stack.

set -e

ROS2_SETUP="${ROS2_SETUP:-/opt/ros/humble/setup.bash}"
WS_SETUP="${WS_SETUP:-$HOME/ros2_ws/install/setup.bash}"
SPIN_TIME="${ROS_DISCOVERY_SPIN_TIME:-3}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
RPLIDAR_PORT="${RPLIDAR_PORT:-/dev/rplidar}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log() { echo -e "${CYAN}[$(date +'%H:%M:%S')]${NC} $1"; }
ok() { echo -e "${GREEN}[OK]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; }

usage() {
    cat <<EOF
Usage:
  $0
  $0 --start-slam
  $0 --start-rplidar-a2m8
  $0 --start-rplidar-a2m12
  $0 --watch-height
  $0 --watch-height-only

Environment:
  SCAN_TOPIC=/scan
  RPLIDAR_PORT=/dev/rplidar
  ROS_DISCOVERY_SPIN_TIME=3

Notes:
  - This is ground-only diagnostics. It never arms the vehicle.
  - If the RPLidar model is unknown, try A2M8 first, then A2M12.
  - --watch-height prints relative deltas after normal checks.
  - --watch-height-only skips slow ros2 CLI checks and starts the live delta watcher directly.
EOF
}

source_ros() {
    if [ -f "$ROS2_SETUP" ]; then
        # shellcheck disable=SC1090
        source "$ROS2_SETUP"
    else
        error "ROS2 setup not found: $ROS2_SETUP"
        exit 1
    fi

    if [ -f "$WS_SETUP" ]; then
        # shellcheck disable=SC1090
        source "$WS_SETUP"
    else
        error "Workspace setup not found: $WS_SETUP"
        exit 1
    fi
}

topic_count() {
    local topic="$1"
    local field="$2"
    ros2 topic info -v "$topic" --no-daemon --spin-time "$SPIN_TIME" 2>/dev/null |
        awk -F': ' -v key="$field" '$1 == key {print $2; exit}'
}

topic_exists() {
    local topic="$1"
    ros2 topic list --no-daemon --spin-time "$SPIN_TIME" 2>/dev/null |
        grep -qx "$topic"
}

print_topic_info() {
    local topic="$1"

    echo ""
    echo "Topic endpoints: $topic"
    if topic_exists "$topic"; then
        ros2 topic info -v "$topic" --no-daemon --spin-time "$SPIN_TIME" 2>/dev/null || true
    else
        echo "  status: missing"
    fi
}

start_slam_if_requested() {
    if [ "${START_SLAM:-false}" != "true" ]; then
        return 0
    fi

    log "Starting SLAM/PX4 bridge in background..."
    /home/cfly/ros2_ws/scripts/run_slam_px4.sh --bg
}

start_rplidar_if_requested() {
    case "${START_RPLIDAR_MODEL:-}" in
        a2m8)
            log "Starting RPLidar A2M8 on $RPLIDAR_PORT..."
            ros2 launch rplidar_ros rplidar_a2m8_launch.py serial_port:="$RPLIDAR_PORT" &
            RPLIDAR_PID=$!
            sleep 5
            ;;
        a2m12)
            log "Starting RPLidar A2M12 on $RPLIDAR_PORT..."
            ros2 launch rplidar_ros rplidar_a2m12_launch.py serial_port:="$RPLIDAR_PORT" &
            RPLIDAR_PID=$!
            sleep 5
            ;;
        "")
            ;;
        *)
            error "Unknown RPLidar model: $START_RPLIDAR_MODEL"
            exit 1
            ;;
    esac
}

check_devices() {
    log "Checking RPLidar USB device candidates..."

    local found=false
    for dev in /dev/rplidar /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0 /dev/ttyACM1; do
        if [ -e "$dev" ]; then
            ls -l "$dev"
            found=true
        fi
    done

    if [ -d /dev/serial/by-id ]; then
        find /dev/serial/by-id -maxdepth 1 -type l -print -exec ls -l {} \; 2>/dev/null || true
    fi

    if $found; then
        ok "RPLidar serial device candidate found"
    else
        warn "No /dev/rplidar, /dev/ttyUSB*, or /dev/ttyACM* candidate found"
        warn "Check RPLidar USB cable, motor power, udev rule, and whether the adapter enumerated"
    fi
}

check_ros_graph() {
    log "Checking ROS graph for RPLidar and scan usage..."

    local nodes topics
    nodes=$(ros2 node list --no-daemon --spin-time "$SPIN_TIME" 2>/dev/null || true)
    topics=$(ros2 topic list --no-daemon --spin-time "$SPIN_TIME" 2>/dev/null || true)

    if echo "$nodes" | grep -Eq '(^|/)rplidar_node$'; then
        ok "rplidar_node is present"
    else
        warn "rplidar_node is not present"
    fi

    if echo "$topics" | grep -qx "$SCAN_TOPIC"; then
        ok "Scan topic exists: $SCAN_TOPIC"
    else
        warn "Scan topic not found: $SCAN_TOPIC"
    fi

    local pub_count sub_count
    pub_count=$(topic_count "$SCAN_TOPIC" "Publisher count")
    sub_count=$(topic_count "$SCAN_TOPIC" "Subscription count")
    pub_count=${pub_count:-0}
    sub_count=${sub_count:-0}

    echo ""
    echo "Scan endpoints:"
    echo "  topic: $SCAN_TOPIC"
    echo "  publishers: $pub_count"
    echo "  subscribers: $sub_count"

    if [ "$pub_count" -ge 1 ]; then
        ok "$SCAN_TOPIC has publisher(s)"
    else
        warn "$SCAN_TOPIC has no publisher"
    fi

    if [ "$sub_count" -ge 1 ]; then
        ok "$SCAN_TOPIC has subscriber(s), so scan data is being consumed"
    else
        warn "$SCAN_TOPIC has no subscribers; RPLidar is not used by SLAM/tasks right now"
    fi

    if topic_exists "$SCAN_TOPIC"; then
        echo ""
        ros2 topic info -v "$SCAN_TOPIC" --no-daemon --spin-time "$SPIN_TIME" 2>/dev/null || true
    fi
}

check_scan_data() {
    log "Checking /scan data health..."

    if ! topic_exists "$SCAN_TOPIC"; then
        warn "Skipping scan data check because $SCAN_TOPIC is missing"
        return 0
    fi

    echo ""
    echo "Scan rate:"
    timeout 8 ros2 topic hz "$SCAN_TOPIC" --no-daemon --spin-time "$SPIN_TIME" 2>/dev/null || warn "Could not measure $SCAN_TOPIC rate"

    echo ""
    echo "One scan sample:"
    timeout 5 ros2 topic echo --once "$SCAN_TOPIC" --no-daemon --spin-time "$SPIN_TIME" 2>/dev/null |
        sed -n '1,80p' || warn "Could not read one $SCAN_TOPIC sample"
}

sample_field() {
    local label="$1"
    local topic="$2"
    local field="$3"

    echo ""
    echo "$label"
    echo "  topic: $topic"

    if ! topic_exists "$topic"; then
        echo "  status: missing"
        return 0
    fi

    local value
    value=$(timeout 5 ros2 topic echo --once "$topic" --field "$field" --no-daemon --spin-time "$SPIN_TIME" 2>/dev/null || true)
    if [ -n "$value" ]; then
        echo "$value" | sed 's/^/  /'
    else
        echo "  status: topic exists but no sample received"
    fi
}

sample_numeric_field() {
    local topic="$1"
    local field="$2"

    if ! topic_exists "$topic"; then
        echo "nan"
        return 0
    fi

    local value
    value=$(timeout 2 ros2 topic echo --once "$topic" --field "$field" --no-daemon --spin-time "$SPIN_TIME" 2>/dev/null |
        awk 'NF {print $1; exit}' || true)
    if [ -n "$value" ]; then
        echo "$value"
    else
        echo "nan"
    fi
}

sample_position_z() {
    local topic="$1"

    if ! topic_exists "$topic"; then
        echo "nan"
        return 0
    fi

    local value
    value=$(timeout 2 ros2 topic echo --once "$topic" --field "position" --no-daemon --spin-time "$SPIN_TIME" 2>/dev/null |
        awk '
            /\[/ {
                gsub(/\[/, "", $0)
                gsub(/\]/, "", $0)
                gsub(/,/, " ", $0)
                print $3
                exit
            }' || true)
    if [ -n "$value" ]; then
        echo "$value"
    else
        echo "nan"
    fi
}

check_height_sources() {
    log "Sampling height/Z sources..."

    sample_field "VSLAM odometry ENU pose.z" \
        "/visual_slam/tracking/odometry" \
        "pose.pose.position.z"

    sample_field "PX4 visual odometry input position[2] (NED Z when bridged to PX4)" \
        "/fmu/in/vehicle_visual_odometry" \
        "position"

    sample_field "PX4 EKF vehicle odometry output position[2] (NED Z)" \
        "/fmu/out/vehicle_odometry" \
        "position"

    sample_field "PX4 vehicle local position z (NED Z, optional/fallback)" \
        "/fmu/out/vehicle_local_position" \
        "z"

    sample_field "Standardized UAV pose ENU pose.z" \
        "/uav/state/pose" \
        "pose.position.z"

    print_topic_info "/uav/state/pose"

    echo ""
    echo "State bridge node:"
    ros2 node info /state_bridge --no-daemon --spin-time "$SPIN_TIME" 2>/dev/null ||
        echo "  status: /state_bridge not visible"

    echo ""
    echo "Interpretation:"
    echo "  - ENU Z increases upward."
    echo "  - PX4 NED Z is negative above the origin."
    echo "  - For a 1.5m hover, ENU delta should be about +1.5m and NED delta about -1.5m."
}

watch_height_sources() {
    log "Watching height deltas with one persistent ROS node."
    echo "Keep the vehicle disarmed; lift it by about 0.5m by hand. Press Ctrl+C to stop."
    echo ""

    python3 - <<'PY'
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry


def fmt_delta(base, current):
    if base is None or current is None:
        return "nan"
    return f"{current - base:+.3f}"


class HeightWatch(Node):
    def __init__(self):
        super().__init__("height_watch_node")
        self.values = {
            "vslam": None,
            "vio": None,
            "ekf": None,
            "local": None,
            "pose": None,
        }
        self.base = {key: None for key in self.values}
        self.started_at = time.monotonic()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            Odometry,
            "/visual_slam/tracking/odometry",
            lambda msg: self._set("vslam", float(msg.pose.pose.position.z)),
            qos,
        )
        self.create_subscription(
            VehicleOdometry,
            "/fmu/in/vehicle_visual_odometry",
            lambda msg: self._set("vio", float(msg.position[2])),
            qos,
        )
        self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            lambda msg: self._set("ekf", float(msg.position[2])),
            qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            lambda msg: self._set("local", float(msg.z)),
            qos,
        )
        self.create_subscription(
            PoseStamped,
            "/uav/state/pose",
            lambda msg: self._set("pose", float(msg.pose.position.z)),
            qos,
        )
        self.create_timer(1.0, self._print_row)
        print(
            f"{'t(s)':<9} {'dVSLAM_ENU':<12} {'dVIO_NED':<12} "
            f"{'dEKF_NED':<12} {'dLOCAL_NED':<12} {'dPOSE_ENU':<12}",
            flush=True,
        )

    def _set(self, key, value):
        if math.isfinite(value):
            self.values[key] = value
            if self.base[key] is None:
                self.base[key] = value

    def _print_row(self):
        elapsed = int(time.monotonic() - self.started_at)
        print(
            f"{elapsed:<9} "
            f"{fmt_delta(self.base['vslam'], self.values['vslam']):<12} "
            f"{fmt_delta(self.base['vio'], self.values['vio']):<12} "
            f"{fmt_delta(self.base['ekf'], self.values['ekf']):<12} "
            f"{fmt_delta(self.base['local'], self.values['local']):<12} "
            f"{fmt_delta(self.base['pose'], self.values['pose']):<12}",
            flush=True,
        )


rclpy.init()
node = HeightWatch()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY
}

cleanup() {
    if [ -n "${RPLIDAR_PID:-}" ] && kill -0 "$RPLIDAR_PID" 2>/dev/null; then
        log "Stopping temporary RPLidar launch PID $RPLIDAR_PID"
        kill "$RPLIDAR_PID" 2>/dev/null || true
    fi
}

trap cleanup EXIT

while [ $# -gt 0 ]; do
    case "$1" in
        --start-slam)
            START_SLAM=true
            ;;
        --start-rplidar-a2m8)
            START_RPLIDAR_MODEL=a2m8
            ;;
        --start-rplidar-a2m12)
            START_RPLIDAR_MODEL=a2m12
            ;;
        --watch-height)
            WATCH_HEIGHT=true
            ;;
        --watch-height-only)
            WATCH_HEIGHT_ONLY=true
            WATCH_HEIGHT=true
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            error "Unknown argument: $1"
            usage
            exit 1
            ;;
    esac
    shift
done

source_ros
if [ "${WATCH_HEIGHT_ONLY:-false}" = "true" ]; then
    watch_height_sources
    exit 0
fi
start_slam_if_requested
check_devices
start_rplidar_if_requested
check_ros_graph
check_scan_data
check_height_sources
if [ "${WATCH_HEIGHT:-false}" = "true" ]; then
    watch_height_sources
fi

log "Diagnostics complete"
