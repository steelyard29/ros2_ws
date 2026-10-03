#!/bin/bash
# Kill SLAM stack inside isaac_ros_dev without matching this script's argv.
set -u
kill_pat() {
  local pat="$1"
  local pids
  pids=$(pgrep -f "$pat" || true)
  if [ -n "${pids}" ]; then
    echo "kill ${pat}: ${pids}"
    # shellcheck disable=SC2086
    kill -9 ${pids} 2>/dev/null || true
  fi
}
kill_pat '/opt/ros/humble/lib/rtabmap_slam/rtabmap'
kill_pat '/opt/ros/humble/lib/rtabmap_odom/stereo_odometry'
kill_pat '/opt/ros/humble/lib/realsense2_camera/realsense2_camera_node'
kill_pat '/opt/ros/humble/lib/rclcpp_components/component_container'
# 两套方案各自的 launch（旧合并 launch slam_rtabmap.launch.py 已于 2026-09-14 退役）
kill_pat 'vslam_cuvslam.launch.py'
kill_pat 'vslam_rtabmap.launch.py'
sleep 1
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
echo "SHM_LEFT=$(ls /dev/shm/ 2>/dev/null | grep -ciE 'fast|rtps' || true)"
if pgrep -af '/opt/ros/humble/lib/(rtabmap_slam/rtabmap|rtabmap_odom/stereo_odometry|realsense2_camera|rclcpp_components)' >/tmp/_slam_left.txt 2>/dev/null; then
  echo "STILL_RUNNING:"
  cat /tmp/_slam_left.txt
else
  echo "procs_clean"
fi
