#!/bin/bash
# PX4 上锁指令
# 用法: ./disarm.sh

set -e

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/ros2_ws/config/fastdds_bridge.xml"
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null

echo "🔒 发送上锁指令..."

ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  "{timestamp: 0, param1: 0.0, param2: 0.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0, command: 400, target_system: 1, target_component: 1, source_system: 1, source_component: 1, confirmation: 0, from_external: true}"

sleep 1

# 检查上锁状态
ARM_STATE=$(ros2 topic echo /cache/vehicle_status --once 2>/dev/null | grep arming_state | awk '{print $2}')
if [ "$ARM_STATE" = "1" ]; then
    echo "✅ 上锁成功 (DISARMED)"
else
    echo "⚠️  上锁状态: arming_state=$ARM_STATE"
fi
