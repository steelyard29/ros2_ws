#!/bin/bash
# PX4 模式切换指令
# 用法: ./px4_mode.sh [offboard|land|status]

set -e

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/ros2_ws/config/fastdds_bridge.xml"
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null

MODE="${1:-status}"

case "$MODE" in
    offboard)
        echo "🔄 切换到 OFFBOARD 模式..."
        ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
          "{timestamp: 0, param1: 1.0, param2: 6.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0, command: 176, target_system: 1, target_component: 1, source_system: 1, source_component: 1, confirmation: 0, from_external: true}"
        echo "✅ Offboard 模式指令已发送"
        ;;
    land)
        echo "🛬 切换到 LAND 模式..."
        ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
          "{timestamp: 0, param1: 0.0, param2: 0.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0, command: 21, target_system: 1, target_component: 1, source_system: 1, source_component: 1, confirmation: 0, from_external: true}"
        echo "✅ Land 指令已发送"
        ;;
    status|*)
        echo "📊 PX4 状态:"
        ros2 topic echo /cache/vehicle_status --once 2>/dev/null
        ;;
esac
