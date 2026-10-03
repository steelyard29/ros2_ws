#!/usr/bin/env bash
set -eo pipefail

root=/workspaces/ros2_ws/isaac_vio_debug
state_dir="$root/state"
current_file="$state_dir/CURRENT_RUN"
mkdir -p "$state_dir"

source "$root/scripts/container_env.sh"
set -u

node_exists() {
  local exact_name="$1"
  ros2 node list --no-daemon 2>/dev/null | grep -Fxq "$exact_name"
}

wait_for_node() {
  local exact_name="$1"
  local attempts="${2:-30}"
  local i
  for ((i = 0; i < attempts; ++i)); do
    if node_exists "$exact_name"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

pid_alive() {
  local pid_file="$1"
  [[ -s "$pid_file" ]] && kill -0 "$(<"$pid_file")" 2>/dev/null
}

stop_group() {
  local pid_file="$1"
  local label="$2"
  if ! pid_alive "$pid_file"; then
    echo "$label is not running"
    return 0
  fi

  local pid
  pid="$(<"$pid_file")"
  local actual_pgid
  actual_pgid="$(ps -o pgid= -p "$pid" | tr -d ' ')"
  if [[ "$actual_pgid" != "$pid" ]]; then
    echo "Refusing to signal $label: PID $pid is not its recorded process-group leader" >&2
    return 1
  fi

  kill -INT -- "-$pid" 2>/dev/null || true
  local i
  for ((i = 0; i < 10; ++i)); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 1
  done
  kill -TERM -- "-$pid" 2>/dev/null || true
  for ((i = 0; i < 5; ++i)); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 1
  done
  kill -KILL -- "-$pid" 2>/dev/null || true
}

current_run_dir() {
  [[ -s "$current_file" ]] || return 1
  local run_dir
  run_dir="$(<"$current_file")"
  [[ "$run_dir" == "$root/evidence/"* ]] || return 1
  printf '%s\n' "$run_dir"
}

start_stack() {
  if node_exists /camera/camera || node_exists /visual_slam_node; then
    echo "Refusing duplicate start: camera or VIO node already exists." >&2
    echo "Stop the known existing process first; this script never uses broad pkill." >&2
    return 2
  fi

  local run_id="${RUN_ID:-$(date +%Y%m%d_%H%M%S)_vio}"
  [[ "$run_id" =~ ^[A-Za-z0-9._-]+$ ]] || {
    echo "Invalid RUN_ID: $run_id" >&2
    return 2
  }
  local run_dir="$root/evidence/$run_id"
  if [[ -e "$run_dir" ]]; then
    echo "Refusing to overwrite existing run: $run_dir" >&2
    return 2
  fi
  mkdir -p "$run_dir/ros_logs_camera" "$run_dir/ros_logs_vio"
  printf '%s\n' "$run_dir" > "$current_file"

  ROS_LOG_DIR="$run_dir/ros_logs_camera" setsid \
    ros2 run realsense2_camera realsense2_camera_node --ros-args \
      -r __ns:=/camera \
      -r __node:=camera \
      --params-file "$root/config/d435i_ir_imu.yaml" \
      > "$run_dir/camera.log" 2>&1 < /dev/null &
  local camera_pid=$!
  printf '%s\n' "$camera_pid" > "$run_dir/camera.pid"

  if ! wait_for_node /camera/camera 30; then
    echo "Camera failed to appear; see $run_dir/camera.log" >&2
    stop_group "$run_dir/camera.pid" camera || true
    return 1
  fi

  ROS_LOG_DIR="$run_dir/ros_logs_vio" setsid \
    ros2 run isaac_ros_visual_slam isaac_ros_visual_slam --ros-args \
      -r __node:=visual_slam_node \
      --params-file "$root/config/isaac_vio.yaml" \
      -r visual_slam/image_0:=/camera/camera/infra1/image_rect_raw \
      -r visual_slam/camera_info_0:=/camera/camera/infra1/camera_info \
      -r visual_slam/image_1:=/camera/camera/infra2/image_rect_raw \
      -r visual_slam/camera_info_1:=/camera/camera/infra2/camera_info \
      -r visual_slam/imu:=/camera/camera/imu \
      > "$run_dir/vio.log" 2>&1 < /dev/null &
  local vio_pid=$!
  printf '%s\n' "$vio_pid" > "$run_dir/vio.pid"

  if ! wait_for_node /visual_slam_node 30; then
    echo "VIO failed to appear; see $run_dir/vio.log" >&2
    stop_group "$run_dir/vio.pid" VIO || true
    stop_group "$run_dir/camera.pid" camera || true
    return 1
  fi

  echo "Camera + VIO started"
  echo "Run directory: $run_dir"
  echo "No PX4 bridge was started."
}

stop_stack() {
  local run_dir
  if ! run_dir="$(current_run_dir)"; then
    echo "No managed current run. Nothing stopped." >&2
    return 1
  fi
  stop_group "$run_dir/vio.pid" VIO
  stop_group "$run_dir/camera.pid" camera
  echo "Stopped only the managed camera/VIO process groups from: $run_dir"
}

status_stack() {
  local run_dir=""
  run_dir="$(current_run_dir 2>/dev/null || true)"
  echo "Run directory: ${run_dir:-none}"
  if [[ -n "$run_dir" ]]; then
    for label in camera vio; do
      local pid_file="$run_dir/$label.pid"
      if pid_alive "$pid_file"; then
        echo "$label: running (PID $(<"$pid_file"))"
      else
        echo "$label: stopped"
      fi
    done
  fi
  echo "ROS nodes:"
  ros2 node list --no-daemon 2>/dev/null || true
}

record_bag() {
  local duration="${1:-15}"
  [[ "$duration" =~ ^[0-9]+$ ]] || {
    echo "Duration must be an integer number of seconds (0 means until Ctrl+C)." >&2
    return 2
  }
  local run_dir
  run_dir="$(current_run_dir)" || {
    echo "No managed current run." >&2
    return 1
  }
  pid_alive "$run_dir/camera.pid" && pid_alive "$run_dir/vio.pid" || {
    echo "Managed camera and VIO must both be running." >&2
    return 1
  }

  local bag_dir="$run_dir/bag_$(date +%Y%m%d_%H%M%S)"
  local topics=(
    /camera/camera/infra1/image_rect_raw
    /camera/camera/infra1/camera_info
    /camera/camera/infra2/image_rect_raw
    /camera/camera/infra2/camera_info
    /camera/camera/imu
    /camera/camera/accel/sample
    /camera/camera/gyro/sample
    /camera/camera/accel/imu_info
    /camera/camera/gyro/imu_info
    /camera/camera/extrinsics/depth_to_accel
    /camera/camera/extrinsics/depth_to_gyro
    /camera/camera/extrinsics/depth_to_infra1
    /camera/camera/extrinsics/depth_to_infra2
    /visual_slam/status
    /visual_slam/tracking/odometry
    /tf
    /tf_static
    /diagnostics
  )

  local rc=0
  if ((duration > 0)); then
    timeout --signal=INT --kill-after=10 "${duration}s" \
      ros2 bag record -o "$bag_dir" \
        --qos-profile-overrides-path "$root/config/rosbag_qos.yaml" \
        "${topics[@]}" || rc=$?
    [[ "$rc" == 0 || "$rc" == 124 ]] || return "$rc"
  else
    ros2 bag record -o "$bag_dir" \
      --qos-profile-overrides-path "$root/config/rosbag_qos.yaml" \
      "${topics[@]}"
  fi

  ros2 bag info "$bag_dir" | tee "$bag_dir/bag_info.txt"
  echo "Bag directory: $bag_dir"
}

validate_online() {
  local duration="${1:-30}"
  [[ "$duration" =~ ^[0-9]+$ ]] && ((duration > 0)) || {
    echo "Duration must be a positive integer number of seconds." >&2
    return 2
  }
  local run_dir
  run_dir="$(current_run_dir)" || {
    echo "No managed current run." >&2
    return 1
  }
  pid_alive "$run_dir/camera.pid" && pid_alive "$run_dir/vio.pid" || {
    echo "Managed camera and VIO must both be running." >&2
    return 1
  }
  local output="$run_dir/online_validation_$(date +%Y%m%d_%H%M%S).json"
  python3 "$root/scripts/validate_online.py" --duration "$duration" | tee "$output"
  echo "Validation JSON: $output"
}

case "${1:-status}" in
  start) start_stack ;;
  stop) stop_stack ;;
  status) status_stack ;;
  record) shift; record_bag "${1:-15}" ;;
  validate) shift; validate_online "${1:-30}" ;;
  *) echo "Usage: $0 {start|stop|status|record [seconds]|validate [seconds]}" >&2; exit 2 ;;
esac
