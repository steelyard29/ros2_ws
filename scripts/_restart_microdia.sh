#!/bin/bash
set -u
LOG=/tmp/slam_px4_logs/sensors_extra.log
mkdir -p /tmp/slam_px4_logs

# Kill only by executable paths
for pat in \
  'sensors_extra.launch.py' \
  '/opt/ros/humble/lib/v4l2_camera/v4l2_camera_node' \
  'circle_landing_target_node.py' \
  '/opt/ros/humble/lib/rplidar_ros/rplidar_node'
do
  pids=$(pgrep -f "$pat" || true)
  if [ -n "${pids:-}" ]; then
    echo "kill $pat: $pids"
    kill -9 $pids 2>/dev/null || true
  fi
done
sleep 2

DEV=$(python3 - <<'PY'
code = open("/home/cfly/ros2_ws/src/uav_task/launch/sensors_extra.launch.py").read().split("def _launch_setup")[0]
ns = {}
exec(code, ns)
print(ns["_find_microdia_device"]("auto"))
PY
)
echo "Microdia device: $DEV"

setsid bash -lc "
  source /opt/ros/humble/setup.bash
  source /home/cfly/ros2_ws/install/setup.bash
  exec ros2 launch uav_task sensors_extra.launch.py \
    start_rplidar:=1 start_microdia:=1 start_obstacle_distance:=0 \
    rplidar_port:=/dev/rplidar microdia_device:=${DEV}
" > "$LOG" 2>&1 < /dev/null &
echo "launched sensors_extra pid=$!"
sleep 10
echo '=== log ==='
tail -50 "$LOG"
echo '=== hz ==='
source /opt/ros/humble/setup.bash
timeout 5 ros2 topic hz /image_raw --window 15 2>&1 | head -10
echo '=== nodes ==='
ros2 node list 2>/dev/null | grep -iE 'micro|circle|rplidar|v4l' || true
