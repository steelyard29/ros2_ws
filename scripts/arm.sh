#!/bin/bash
# PX4 解锁指令
# 用法: ./arm.sh

set -e

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/ros2_ws/config/fastdds_bridge.xml"
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null

echo "🔓 发送解锁指令..."

ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  "{timestamp: 0, param1: 1.0, param2: 0.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0, command: 400, target_system: 1, target_component: 1, source_system: 1, source_component: 1, confirmation: 0, from_external: true}"

sleep 1

# 检查解锁状态
ARM_STATE=$(ros2 topic echo /cache/vehicle_status --once 2>/dev/null | grep arming_state | awk '{print $2}')
if [ "$ARM_STATE" = "2" ]; then
    echo "✅ 解锁成功 (ARMED)"
elif [ "$ARM_STATE" = "1" ]; then
    echo "❌ 解锁失败，仍为锁定状态 (DISARMED)"
    echo "   请检查:"
    echo "   1. 安全开关是否按下? (CBRK_IO_SAFETY=22027)"
    echo "   2. 飞前检查是否通过? (pre_flight_checks_pass)"
else
    echo "⚠️  无法获取解锁状态"
fi
