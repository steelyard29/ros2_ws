#!/usr/bin/env bash

# Source this inside isaac_ros_dev. It keeps the repaired RealSense and GXF
# runtime paths local to this reproducible camera/VIO workflow.
source /opt/ros/humble/setup.bash

gxf_root=/opt/ros/humble/share/isaac_ros_gxf/gxf/lib
if [[ ! -d "$gxf_root" ]]; then
  echo "Missing Isaac ROS GXF library root: $gxf_root" >&2
  return 1 2>/dev/null || exit 1
fi

gxf_library_path="$({
  find "$gxf_root" -type f -name '*.so*' -printf '%h\n'
} | sort -u | paste -sd:)"

export LD_LIBRARY_PATH="/opt/realsense_rsusb/lib:${gxf_library_path}:${LD_LIBRARY_PATH:-}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

