#!/bin/bash
# ═══════════════════════════════════════════════════════════════════
# PX4 一键飞行命令封装 (Shell Wrapper)
#
# 自动 source ROS2 环境后调用 px4_fly.py
#
# 用法:
#   ./px4_fly.sh takeoff [高度]     # 一键起飞 (默认 2m)
#   ./px4_fly.sh land              # 一键降落
#   ./px4_fly.sh rtl               # 一键返航
#   ./px4_fly.sh arm               # 仅解锁
#   ./px4_fly.sh disarm            # 仅锁定
#   ./px4_fly.sh estop             # 紧急停机 ★危险★
#   ./px4_fly.sh status            # 查看飞控状态
#   ./px4_fly.sh hover [x] [y] [z] [yaw]  # Offboard 悬停
#
# 起飞前务确认:
#   ./run_slam_px4.sh --check-arm    # 飞控解锁条件检查
#   ./px4_fly.sh status              # 飞控实时状态
#
# 完整飞行流程:
#   ./run_slam_px4.sh --bg           # 1. 启动 VSLAM + PX4 Bridge
#   sleep 30                          # 2. 等待 VSLAM 初始化
#   ./run_slam_px4.sh --check-arm    # 3. 检查解锁条件
#   ./px4_fly.sh takeoff 2.5         # 4. 起飞到 2.5m
#   ./px4_fly.sh land                # 5. 降落
# ═══════════════════════════════════════════════════════════════════

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$HOME/ros2_ws/install/setup.bash"

# ── 颜色 ──
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log()   { echo -e "${CYAN}[$(date +'%H:%M:%S')]${NC} $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; }

# ── source 环境 ──
if [ -f "$ROS2_SETUP" ]; then
    source "$ROS2_SETUP"
else
    error "ROS2 Humble 未安装在 $ROS2_SETUP"
    exit 1
fi

if [ -f "$WS_SETUP" ]; then
    source "$WS_SETUP"
else
    error "ros2_ws 未编译: $WS_SETUP"
    exit 1
fi

# ── 调用 Python 脚本 ──
PY_SCRIPT="$SCRIPT_DIR/px4_fly.py"

if [ ! -f "$PY_SCRIPT" ]; then
    error "找不到 $PY_SCRIPT"
    exit 1
fi

if [ $# -eq 0 ]; then
    python3 "$PY_SCRIPT" --help
    exit 0
fi

CMD="${1:-}"

case "$CMD" in
    estop|emergency)
        # estop 不需要检查系统是否在运行
        shift
        python3 "$PY_SCRIPT" estop
        ;;
    status|arm|disarm|takeoff|land|rtl|hover)
        # 这些命令需要 PX4 Bridge 在运行
        # 快速检查 DDS 链路
        if ! ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_status_v1"; then
            warn "DDS 链路不通! 请先启动 PX4 Bridge:"
            warn "  $HOME/ros2_ws/scripts/run_slam_px4.sh --bg"
            warn ""
            warn "如果 Bridge 正在运行，请等待几秒后重试。"
        fi
        python3 "$PY_SCRIPT" "$@"
        ;;
    -h|--help|help)
        python3 "$PY_SCRIPT" --help
        ;;
    *)
        python3 "$PY_SCRIPT" "$@"
        ;;
esac
