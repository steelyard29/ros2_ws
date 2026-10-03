#!/bin/bash
# ═══════════════════════════════════════════════════════════════════
# 一键启动: 视觉定位 (容器内) + PX4 Bridge (宿主机)
#
# 两套互斥方案（2026-09-14 拆分；不再靠 USE_ISAAC_VSLAM/USE_RTABMAP_RELAY 交叉组合）:
#
#   SLAM_SCHEME=cuvslam (方案A, 默认)
#     D435i 双目IR(±IMU) → Isaac cuVSLAM (GPU VIO) → /visual_slam/tracking/odometry
#       → vslam_odom_bridge → /fmu/in/vehicle_visual_odometry → PX4
#     无 RTAB-Map、无地图库、无激光。launch: launch/vslam_cuvslam.launch.py
#
#   SLAM_SCHEME=rtabmap (方案B)
#     D435i 双目IR(±IMU) → rtabmap stereo_odometry (CPU) → /visual_slam/tracking/odometry
#     D435i RGB-D ──────► RTAB-Map 建图/定位（maps/ 地图库）→ TF: map→odom
#     可选回环修正 relay（USE_RTABMAP_RELAY=1，仅 production）。launch: launch/vslam_rtabmap.launch.py
#
#   避障(两方案通用): RPLidar → /scan → APF mission controller
#                              → /fmu/in/obstacle_distance (PX4 collision prevention)
#
#   注意: 2D RPLidar/ICP 不作为 PX4 定位源。飞行高度变化会改变扫描截面，
#         同一房间在不同高度不是同一张 2D scan，ICP/2D SLAM 无法可靠匹配。
#         激光 SLAM（slam_toolbox / laser_icp_odom / scan_planar）已于 2026-09-14 整体移除。
#
# 前置条件:
#   1. isaac_ros_dev 容器镜像已构建
#   2. /dev/ttyTHS1 已连接 PX4 飞控
#   3. PX4 EKF2/外部视觉 参数已配置 (室内激光定高 + 视觉全自由度):
#      EKF2_EV_CTRL=15          (视觉: HPOS+VPOS+VEL+YAW；Bridge 直传 TFmini Z，与 rangefinder 一致)
#      EKF2_HGT_REF=2           (高度参考使用 TELEM2 向下测距)
#      EKF2_BARO_CTRL=0         (室内关气压)
#      EKF2_MAG_TYPE=5          (室内关磁力计)
#      EKF2_RNG_CTRL=2          (始终融合测距; 激光主导定高)
#      EKF2_RNG_POS_Z=0.05      (TFmini 在 CG 下方 5cm)
#      EKF2_RNG_NOISE=0.15      (测距仪噪声)
#      EKF2_RNG_GATE=6.0        (测距仪 innovation gate)
#      EKF2_RNG_K_GATE=3.0      (测距仪 kalman gate)
#      EKF2_EV_DELAY=0          (外部视觉延迟, 不确定时设为0)
#
# 可选环境变量:
#   SLAM_SCHEME=cuvslam|rtabmap  定位方案（默认 cuvslam）
#   IMU_SOURCE=none|d435|px4     VIO 的 IMU 来源（默认 none=纯双目）
#                                  d435 = /camera/camera/imu（相机内置；图像-IMU 硬件同步）
#                                  px4  = /camera/camera/imu_FC（宿主机 px4_imu_relay 转发飞控 IMU）
#   RUN_MODE=production|mapping|debug  运行模式（默认 production）
#                                  cuvslam: production/debug 等价（无地图库）
#                                  rtabmap: production=只读定位 / mapping=边飞边建图 / debug=仅 CPU 双目里程计
#   POSITION_VARIANCE_XY=0.05    EV 水平位置方差（越小 EKF 越信任 VSLAM 位置）
#   VELOCITY_VARIANCE=0.08       EV 速度方差（越小 EKF 越信任 VSLAM 速度；调试漂移时可加大）
#   USE_RTABMAP_RELAY=1          仅 rtabmap 方案：回环修正里程计 relay（cuvslam 方案强制 0）
#   START_RPLIDAR=1             宿主机启动 RPLIDAR A2M8 → /scan 避障 (默认 1)
#   START_MICRODIA=1            宿主机启动下视 Microdia + 圆降落检测 (默认 1)
#   START_OBSTACLE_DISTANCE=1   LaserScan → /fmu/in/obstacle_distance (默认 1)
#   RESET_REALSENSE_USB=0       默认不复位 D435i；仅相机挂死时显式设为 1
#   USE_REALSENSE_RSUSB=1       优先用 /opt/realsense_rsusb (JP6 HID IMU 保险，默认 1)
#   CONFIGURE_PX4_TFMINI_HEIGHT=0  启动前经 MAVLink 写 TFmini 高度参 (默认 0；
#                                  已用 uXRCE 时写参会停 Agent 并易卡死 PX4 client，
#                                  参数已设好时务必保持 0)
#   RPLIDAR_PORT=/dev/rplidar   雷达串口
#   MICRODIA_DEVICE=auto        下视相机 (默认 auto：按 USB 0c45:6366 / LRCP 名自动找)
#
#   已移除的变量（2026-09-14 激光/T265/OpenVINS 清理）：ODOM_SOURCE、USE_ISAAC_VSLAM、
#   USE_LIDAR_SLAM、SUBSCRIBE_SCAN、SCAN_PLANAR、USE_PX4_IMU、D435_ENABLE_IMU、
#   VSLAM_ENABLE_IMU_FUSION、STEREO_SUBSCRIBE_IMU。若环境中仍设置，脚本会拒绝启动。

#
#   4. 飞控串口 uxrce_dds_client 已启用:
#      参数 UXRCE_DDS_CFG=TELEM1 (或其他对应 /dev/ttyTHS1 的串口)
#      参数 SER_TEL1_BAUD=921600
#
#   5. (可选) 如果 PX4 ≥ v1.14, EKF2_AID_MASK 已废弃,
#      使用 EKF2_EV_CTRL + EKF2_GPS_CTRL 替代
#
# 用法:
#   ./run_slam_px4.sh            # 前台运行(CTRL+C停止所有)
#   ./run_slam_px4.sh --bg       # 后台运行
#   ./run_slam_px4.sh stop       # 停止所有进程
#   ./run_slam_px4.sh reset-agent # 手动重置 MicroXRCEAgent
#   ./run_slam_px4.sh status     # 查看 topic 和节点状态
#   ./run_slam_px4.sh --check-arm # 检查飞控解锁条件是否满足 (★起飞前推荐运行!)
#   ./run_slam_px4.sh ground-only # 仅启动 D435i/cuSLAM，不连接 PX4、不启动 bridge
# ═══════════════════════════════════════════════════════════════════

set -e

CONTAINER_NAME="isaac_ros_dev"
ROS2_WS="$HOME/ros2_ws"
CONTAINER_WS="/workspaces/ros2_ws"
LOG_DIR="/tmp/slam_px4_logs"
PID_DIR="/tmp/run_slam_px4.pids"
ROS_DISCOVERY_SPIN_TIME="${ROS_DISCOVERY_SPIN_TIME:-3}"
PRODUCTION_DB="$ROS2_WS/maps/production/rtabmap.db"
CONTAINER_PRODUCTION_DB="$CONTAINER_WS/maps/production/rtabmap.db"

# PX4 串口设备 (可通过环境变量覆盖)
# ttyTHS1 = Jetson TELEM1 UART (默认), ttyACM0 = PX4 USB 口
PX4_SERIAL="${PX4_SERIAL_DEV:-/dev/ttyTHS1}"
PX4_BAUD="${PX4_SERIAL_BAUD:-921600}"
# 默认启用已接线的 RPLidar / Microdia；无设备时 start_extra_sensors 会自动跳过
START_RPLIDAR="${START_RPLIDAR:-1}"
START_MICRODIA="${START_MICRODIA:-1}"
START_OBSTACLE_DISTANCE="${START_OBSTACLE_DISTANCE:-1}"
# ── SLAM 方案二选一（2026-09-14 拆分：cuVSLAM 与 RTAB-Map 完全隔离）──
#   cuvslam = 方案A: Isaac cuVSLAM GPU VIO（无 RTAB-Map/地图库/激光）
#   rtabmap = 方案B: rtabmap stereo_odometry (CPU) + RTAB-Map 地图定位/闭环
SLAM_SCHEME="${SLAM_SCHEME:-cuvslam}"
# VIO 的 IMU 来源（两方案同语义）: none | d435 | px4
#   none 纯双目（默认，最保守；cuVSLAM 侧 2026-09-02 验证 28.7 Hz 稳定）
#   d435 D435i 内置 IMU（2026-09-14 修复+标定，图像-IMU 硬件同步，VIO 首选）
#   px4  飞控 IMU（宿主机 px4_imu_relay: /fmu/out/sensor_combined → /camera/camera/imu_FC）
IMU_SOURCE="${IMU_SOURCE:-none}"
# 默认不写参：uXRCE 占用 TELEM1 时停 Agent 会导致 PX4 client 卡死、DDS 全断
CONFIGURE_PX4_TFMINI_HEIGHT="${CONFIGURE_PX4_TFMINI_HEIGHT:-0}"
RPLIDAR_PORT="${RPLIDAR_PORT:-/dev/rplidar}"
# auto: 扫描 V4L，匹配 Microdia (0c45:6366) / LRCP；也可手动指定 /dev/videoN
MICRODIA_DEVICE="${MICRODIA_DEVICE:-auto}"
# 运行模式: 见头部说明（cuvslam: production/debug 等价；rtabmap: 三态）
RUN_MODE="${RUN_MODE:-production}"
# `ground-only` defaults to debug; override only with GROUND_RUN_MODE when a
# map-building ground session is intentionally needed.
GROUND_RUN_MODE="${GROUND_RUN_MODE:-debug}"
D435_EMITTER_ENABLED="${D435_EMITTER_ENABLED:-0}"
# IR exposure controls. Keep the proven auto-exposure baseline by default;
# set D435_AUTO_EXPOSURE=0 for a powered vibration A/B test.
D435_AUTO_EXPOSURE="${D435_AUTO_EXPOSURE:-true}"
D435_EXPOSURE_US="${D435_EXPOSURE_US:-8500}"
D435_GAIN="${D435_GAIN:-16}"
VSLAM_ENABLE_DENOISING="${VSLAM_ENABLE_DENOISING:-false}"
VSLAM_TRACKING_MODE="${VSLAM_TRACKING_MODE:-1}"
# D435i is 30 Hz (33.3 ms nominal); Jetson ground logs show occasional
# 66 ms one-frame scheduling gaps. Allow one gap, but keep >70 ms visible.
VSLAM_IMAGE_JITTER_MS="${VSLAM_IMAGE_JITTER_MS:-70.0}"
# 仅 rtabmap 方案：用生产地图回环修正里程计（debug/mapping 模式强制关闭；cuvslam 强制关闭）
USE_RTABMAP_RELAY="${USE_RTABMAP_RELAY:-1}"
# 启用同步实验日志（CSV + rosbag + 启动配置快照）
DRIFT_LOG="${DRIFT_LOG:-0}"
EXTRA_SENSORS_PID=""
CONFIGURE_PX4_SCRIPT="$ROS2_WS/scripts/configure_px4_dds.py"

# 以下由 resolve_slam_scheme() 归一化后填充
SLAM_LAUNCH_FILE=""
SLAM_LOG=""
USE_PX4_IMU_RELAY=0

mkdir -p "$LOG_DIR" "$PID_DIR"

# ── PID 追踪 ──
_track_pid() {
    local name="$1" pid="$2"
    echo "$pid" > "$PID_DIR/${name}.pid"
    log "  [$name] PID=$pid"
}

_read_pid() {
    local name="$1"
    [ -f "$PID_DIR/${name}.pid" ] && cat "$PID_DIR/${name}.pid" 2>/dev/null || true
}

_kill_tracked() {
    local name="$1" signal="${2:-TERM}"
    local pid
    pid=$(_read_pid "$name")
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        kill "-$signal" "$pid" 2>/dev/null || true
        return 0
    fi
    return 1
}

_cleanup_pids() {
    rm -rf "$PID_DIR"
}


# ── 颜色 ──
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log() { echo -e "${CYAN}[$(date +'%H:%M:%S')]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; }
ok() { echo -e "${GREEN}[OK]${NC} $1"; }

# ── 方案归一化 ──
# 旧开关已随激光/T265/OpenVINS 清理移除。宁可拒绝启动，也不要"设了却没生效"。
_LEGACY_VARS=(
    ODOM_SOURCE USE_ISAAC_VSLAM USE_LIDAR_SLAM SUBSCRIBE_SCAN SCAN_PLANAR
    USE_PX4_IMU D435_ENABLE_IMU VSLAM_ENABLE_IMU_FUSION STEREO_SUBSCRIBE_IMU
)

reject_legacy_vars() {
    local v found=()
    for v in "${_LEGACY_VARS[@]}"; do
        [ -n "${!v:-}" ] && found+=("$v=${!v}")
    done
    if [ "${#found[@]}" -gt 0 ]; then
        error "检测到已移除的旧变量: ${found[*]}"
        error "请改用 SLAM_SCHEME=cuvslam|rtabmap 选择方案，IMU_SOURCE=none|d435|px4 选择 IMU 来源"
        error "（旧变量已不生效；继续启动会按新默认值跑，故在此中止）"
        exit 1
    fi
}

resolve_slam_scheme() {
    case "${SLAM_SCHEME,,}" in
        cuvslam|isaac|cuvslam_only) SLAM_SCHEME="cuvslam" ;;
        rtabmap|rtab-map|rtab)      SLAM_SCHEME="rtabmap" ;;
        *)
            error "未知 SLAM_SCHEME='$SLAM_SCHEME'（可选: cuvslam | rtabmap）"
            exit 1
            ;;
    esac

    case "${IMU_SOURCE,,}" in
        none|"")            IMU_SOURCE="none" ;;
        d435|d435i|camera)  IMU_SOURCE="d435" ;;
        px4|fcu|imu_fc)     IMU_SOURCE="px4" ;;
        *)
            error "未知 IMU_SOURCE='$IMU_SOURCE'（可选: none | d435 | px4）"
            exit 1
            ;;
    esac

    USE_PX4_IMU_RELAY=0
    [ "$IMU_SOURCE" = "px4" ] && USE_PX4_IMU_RELAY=1

    if [ "$SLAM_SCHEME" = "cuvslam" ]; then
        # cuVSLAM 方案无 RTAB-Map relay，也无需地图库；relay 开关强制关
        USE_RTABMAP_RELAY=0
    fi

    SLAM_LAUNCH_FILE="$CONTAINER_WS/launch/vslam_${SLAM_SCHEME}.launch.py"
    SLAM_LOG="/tmp/vslam_${SLAM_SCHEME}.log"
}

# 方案与 IMU 来源的冲突检查（启动前调用，避免跑到一半才发现）
check_scheme_conflicts() {
    if [ "$IMU_SOURCE" = "px4" ] && [ "${GROUND_ONLY:-0}" = "1" ]; then
        warn "GROUND_ONLY=1 + IMU_SOURCE=px4: 飞控 IMU 需要 Agent/DDS，地面诊断可能一直无 IMU"
    fi
    if [ "$SLAM_SCHEME" = "cuvslam" ] && [ "${RUN_MODE,,}" = "mapping" ]; then
        warn "SLAM_SCHEME=cuvslam 不建图（无 RTAB-Map）；RUN_MODE=mapping 按 debug 处理"
        RUN_MODE="debug"
    fi
    if [ "$SLAM_SCHEME" = "rtabmap" ] && [ "${RUN_MODE,,}" = "production" ] \
            && [ "${USE_RTABMAP_RELAY,,}" = "1" ]; then
        log "rtabmap 方案: 生产地图回环 relay 已启用（odom 输入 /rtabmap/relay/odometry）"
    fi
}

_cleaning_up=false

stop_host_processes() {
    # First try PID-tracked cleanup (precise)
    local tracked_names=(
        microxrce_agent sensors_extra px4_bridge px4_imu_relay microdia
    )
    local name pid
    for name in "${tracked_names[@]}"; do
        if _kill_tracked "$name" TERM; then
            log "  [$name] SIGTERM sent (PID $(_read_pid "$name"))"
        fi
    done
    sleep 2
    for name in "${tracked_names[@]}"; do
        if _kill_tracked "$name" KILL; then
            log "  [$name] SIGKILL (残留)"
        fi
    done

    # Fallback: pkill known patterns (catches processes started without PID tracking)
    pkill -f "vslam_px4.launch.py" 2>/dev/null || true
    pkill -f "vslam_cuvslam.launch.py" 2>/dev/null || true
    pkill -f "vslam_rtabmap.launch.py" 2>/dev/null || true
    pkill -f "vslam_odom_bridge" 2>/dev/null || true
    pkill -f "px4_gateway_node" 2>/dev/null || true
    pkill -f "state_bridge.py" 2>/dev/null || true
    pkill -f "sensors_extra.launch.py" 2>/dev/null || true
    pkill -f "rplidar_node" 2>/dev/null || true
    pkill -f "v4l2_camera_node" 2>/dev/null || true
    pkill -f "circle_landing_target" 2>/dev/null || true
    pkill -f "laser_scan_to_obstacle_distance" 2>/dev/null || true
    pkill -f "static_transform_publisher.*base_link_to" 2>/dev/null || true
    pkill -f "rtabmap_odom_relay.py" 2>/dev/null || true
    pkill -f "px4_imu_relay.py" 2>/dev/null || true
    pkill -f "drift_logger.py" 2>/dev/null || true
    pkill -f "flight_experiment.sh" 2>/dev/null || true
    pkill -f "pose_logger.py.*experiment_" 2>/dev/null || true
    if [ -n "${EXTRA_SENSORS_PID:-}" ] && kill -0 "$EXTRA_SENSORS_PID" 2>/dev/null; then
        kill -TERM "$EXTRA_SENSORS_PID" 2>/dev/null || true
    fi
    if [ -n "${HOST_LAUNCH_PID:-}" ] && kill -0 "$HOST_LAUNCH_PID" 2>/dev/null; then
        kill -TERM "$HOST_LAUNCH_PID" 2>/dev/null || true
    fi
}

stop_microxrce_agent() {
    pkill -f "MicroXRCEAgent" 2>/dev/null || true
}

stop_container_slam() {
    docker exec "$CONTAINER_NAME" bash -c "
        for pattern in \
            '[r]os2 launch .*/vslam_(cuvslam|rtabmap).launch.py' \
            '[r]ealsense2_camera_node' \
            '[c]omponent_container' \
            '[v]isual_slam_launch_container' \
            '[v]isual_slam_node' \
            '[s]tereo_odometry' \
            '[s]tatic_transform_publisher' \
            '[r]tabmap_slam/rtabmap' \
            '[r]tabmap'
        do
            pkill -TERM -f \"\$pattern\" 2>/dev/null || true
        done
        sleep 1
        for pattern in \
            '[r]os2 launch .*/vslam_(cuvslam|rtabmap).launch.py' \
            '[r]ealsense2_camera_node' \
            '[c]omponent_container' \
            '[v]isual_slam_launch_container' \
            '[v]isual_slam_node' \
            '[s]tereo_odometry' \
            '[s]tatic_transform_publisher' \
            '[r]tabmap_slam/rtabmap' \
            '[r]tabmap'
        do
            pkill -KILL -f \"\$pattern\" 2>/dev/null || true
        done
    " 2>/dev/null || true
}

ensure_stopped_before_start() {
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    source "$ROS2_WS/install/setup.bash" 2>/dev/null || true

    local vslam_pub vio_pub
    vslam_pub=$(topic_count "/visual_slam/tracking/odometry" "Publisher count")
    vio_pub=$(topic_count "/fmu/in/vehicle_visual_odometry" "Publisher count")
    [ -n "$vslam_pub" ] || vslam_pub=0
    [ -n "$vio_pub" ] || vio_pub=0

    if [ "$vslam_pub" -gt 0 ] || [ "$vio_pub" -gt 0 ] \
            || pgrep -f "vslam_px4.launch.py|vslam_cuvslam.launch.py|vslam_rtabmap.launch.py|vslam_odom_bridge" >/dev/null 2>&1; then
        warn "检测到残留 SLAM/Bridge 进程 (vslam_pub=$vslam_pub, vio_pub=$vio_pub)，先清理再启动..."
        stop_host_processes
        stop_container_slam
        sleep 2
    fi
}

topic_count() {
    local topic="$1"
    local field="$2"
    ros2 topic info -v "$topic" --no-daemon --spin-time "$ROS_DISCOVERY_SPIN_TIME" 2>/dev/null | awk -F': ' -v key="$field" '$1 == key {print $2; exit}'
}

# PX4 uXRCE client 连上 Agent 并创建 subscriber 通常需 15–30s（尤其容器/飞控刚启动）
wait_for_px4_dds() {
    local timeout="${1:-30}"
    local elapsed=0
    log "等待 PX4 uXRCE 订阅 vehicle_visual_odometry (最多 ${timeout}s)..."
    while [ "$elapsed" -lt "$timeout" ]; do
        local vio_sub status_pub
        vio_sub=$(topic_count "/fmu/in/vehicle_visual_odometry" "Subscription count")
        status_pub=$(topic_count "/fmu/out/vehicle_status_v1" "Publisher count")
        [ -n "$vio_sub" ] || vio_sub=0
        [ -n "$status_pub" ] || status_pub=0
        if [ "$vio_sub" -ge 1 ] && [ "$status_pub" -ge 1 ]; then
            ok "PX4 DDS 已就绪 (${elapsed}s): vio_sub=$vio_sub, status_pub=$status_pub"
            return 0
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done
    warn "PX4 DDS 在 ${timeout}s 内未完全就绪 (可能仍会继续连接，稍后用 $0 status 复查)"
    return 1
}

cleanup() {
    # 防止重复进入（反复按 CTRL+C 时 trap 可重入）
    if $_cleaning_up; then
        echo -e "${RED}[$(date +'%H:%M:%S')]${NC} 正在清理中，请稍候..."
        return 0
    fi
    _cleaning_up=true

    log "正在停止所有进程..."

    stop_host_processes
    stop_container_slam

    # ★ 再杀 launch 进程（如果还在）
    if [ -n "$HOST_LAUNCH_PID" ] && kill -0 "$HOST_LAUNCH_PID" 2>/dev/null; then
        kill -TERM "$HOST_LAUNCH_PID" 2>/dev/null || true
        # 最多等 2 秒，不卡住
        sleep 2
        # 如果还没死，强制 kill
        if kill -0 "$HOST_LAUNCH_PID" 2>/dev/null; then
            kill -KILL "$HOST_LAUNCH_PID" 2>/dev/null || true
        fi
    fi

    log "已停止"
    exit 0
}

trap cleanup SIGINT SIGTERM

# ── 检查前置条件 ──
check_prereqs() {
    log "检查前置条件..."

    if [ "$SLAM_SCHEME" = "cuvslam" ]; then
        log "SLAM_SCHEME=cuvslam - 不使用 RTAB-Map 地图库，跳过生产地图检查"
    else
    case "${RUN_MODE,,}" in
        debug|mapping)
            log "RUN_MODE=$RUN_MODE - 跳过生产地图检查（不需要预建地图）"
            ;;
        *)
            # RTAB-Map may create an empty database when the path is absent. Production
            # startup must fail closed before any container process can do that.
            if [ ! -f "$PRODUCTION_DB" ]; then
                error "生产定位地图不存在: $PRODUCTION_DB"
                error "拒绝启动，且不会创建空库。请先用 build_rtabmap_map.sh 建图并 promote。"
                error "或使用 RUN_MODE=debug 跳过此检查（纯 VSLAM，不建图）"
                error "或使用 RUN_MODE=mapping 边飞边建图"
                exit 1
            fi
            if [ ! -s "$PRODUCTION_DB" ]; then
                error "生产定位地图为空: $PRODUCTION_DB"
                exit 1
            fi
            if ! python3 - "$PRODUCTION_DB" <<'PY'
import sqlite3
import sys

path = sys.argv[1]
try:
    con = sqlite3.connect("file:{}?mode=ro".format(path), uri=True)
    con.execute("PRAGMA query_only=ON")
    result = con.execute("PRAGMA quick_check").fetchone()
    tables = {row[0] for row in con.execute(
        "SELECT name FROM sqlite_master WHERE type='table'")}
    if not result or result[0] != "ok" or "Node" not in tables:
        raise RuntimeError("quick_check={} tables={}".format(result, sorted(tables)))
finally:
    try:
        con.close()
    except NameError:
        pass
PY
            then
                error "生产定位地图不是有效的 RTAB-Map SQLite 数据库: $PRODUCTION_DB"
                exit 1
            fi
            chmod a-w "$PRODUCTION_DB" ||
                { error "无法移除生产数据库写权限: $PRODUCTION_DB"; exit 1; }
            if stat -c '%A' "$PRODUCTION_DB" | grep -q 'w'; then
                error "生产数据库仍可写，拒绝启动: $PRODUCTION_DB"
                exit 1
            fi
            ok "生产定位地图只读检查通过: $PRODUCTION_DB"
            ;;
    esac
    fi

    if [ "${GROUND_ONLY:-0}" != "1" ]; then
        # 检查串口设备
        if [ ! -c "$PX4_SERIAL" ]; then
            error "PX4 串口 $PX4_SERIAL 不存在！请检查 PX4 飞控连接。"
            exit 1
        fi
        ok "PX4 串口 $PX4_SERIAL 存在"

        # 检查 MicroXRCEAgent
        if ! which MicroXRCEAgent >/dev/null 2>&1; then
            error "MicroXRCEAgent 未安装! 请安装: sudo apt install micro-xrce-dds-agent"
            exit 1
        fi
        ok "MicroXRCEAgent 已安装"
    else
        log "GROUND_ONLY=1 - 跳过 PX4 串口与 MicroXRCEAgent 检查"
    fi

    # 检查 ROS2
    if [ ! -f /opt/ros/humble/setup.bash ]; then
        error "ROS2 Humble 未安装在宿主机!"
        exit 1
    fi
    ok "ROS2 Humble 已安装"

    # 检查 ros2_ws
    if [ ! -f "$ROS2_WS/install/setup.bash" ]; then
        error "ros2_ws 未编译! 请先运行: cd $ROS2_WS && colcon build"
        exit 1
    fi
    ok "ros2_ws 已编译"

    # 检查 Docker 容器
    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        warn "容器 $CONTAINER_NAME 未运行，正在启动..."
        docker start "$CONTAINER_NAME" >/dev/null
        sleep 2
        if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            error "无法启动容器 $CONTAINER_NAME!"
            exit 1
        fi
    fi
    ok "容器 $CONTAINER_NAME 运行中"

    ok "所有前置条件满足"
}

# ── 启动前写入 TFmini 高度融合参数 ──
# TELEM1 已是 uXRCE 时，停 Agent 也不能变回 MAVLink；优先试 USB。
ensure_px4_tfmini_height() {
    case "${CONFIGURE_PX4_TFMINI_HEIGHT,,}" in
        0|false|no|off)
            log "跳过 PX4 TFmini 高度参数写入 (CONFIGURE_PX4_TFMINI_HEIGHT=0)"
            return 0
            ;;
    esac

    if [ ! -f "$CONFIGURE_PX4_SCRIPT" ]; then
        warn "未找到 $CONFIGURE_PX4_SCRIPT，跳过 TFmini 高度参数配置"
        return 0
    fi

    log "配置 PX4 TFmini 高度融合 (EKF2_HGT_REF=2, EKF2_RNG_CTRL=2, BARO/MAG off)..."
    local cfg_log="$LOG_DIR/configure_px4_tfmini.log"
    local cfg_env=(
        PX4_ENABLE_TFMINI_HEIGHT=1
        PX4_HEIGHT_ONLY=1
        PX4_SKIP_REBOOT=1
        PX4_SKIP_SAVE=0
        "PX4_SERIAL_CANDIDATES=/dev/ttyUSB0,/dev/ttyACM0,$PX4_SERIAL"
    )

    # 1) 先不中断 DDS：尝试 USB MAVLink
    if env "${cfg_env[@]}" python3 "$CONFIGURE_PX4_SCRIPT" >"$cfg_log" 2>&1; then
        ok "PX4 TFmini 高度融合参数已写入 (日志: $cfg_log)"
        return 0
    fi

    # 2) 再短暂停 Agent，尝试 TELEM1（仅当飞控仍跑 MAVLink 时有效）
    warn "USB/当前串口 MAVLink 写参失败，短暂停止 MicroXRCEAgent 后重试 TELEM1..."
    local agent_was_running=0
    if [ -n "$(microxrce_agent_pid)" ]; then
        agent_was_running=1
        stop_microxrce_agent
        sleep 1
    fi

    if env "${cfg_env[@]}" \
        "PX4_SERIAL_CANDIDATES=$PX4_SERIAL,/dev/ttyUSB0,/dev/ttyACM0" \
        python3 "$CONFIGURE_PX4_SCRIPT" >>"$cfg_log" 2>&1; then
        ok "PX4 TFmini 高度融合参数已写入 (TELEM1, 日志: $cfg_log)"
    else
        warn "无法经 MAVLink 写入 TFmini 融合参数 (uXRCE 占用 TELEM1 时属正常)"
        warn "请用 QGC/USB 一次性设置: EKF2_HGT_REF=2, EKF2_RNG_CTRL=2, EKF2_BARO_CTRL=0, EKF2_MAG_TYPE=5, SENS_TFMINI_CFG=102"
        warn "或临时打开 USB MAVLink 后执行:"
        warn "  PX4_HEIGHT_ONLY=1 PX4_SKIP_REBOOT=1 PX4_ENABLE_TFMINI_HEIGHT=1 \\"
        warn "    python3 $CONFIGURE_PX4_SCRIPT"
        warn "详情: $cfg_log"
    fi

    if [ "$agent_was_running" -eq 1 ] || [ -z "$(microxrce_agent_pid)" ]; then
        start_microxrce_agent
    fi
}

# 查找 D435i sysfs 路径（仅匹配 8086:0b3a，禁止写死旧路径以免误复位）
find_realsense_sysfs() {
    local d vendor product
    for d in /sys/bus/usb/devices/*; do
        [ -f "$d/idVendor" ] && [ -f "$d/idProduct" ] || continue
        vendor=$(cat "$d/idVendor" 2>/dev/null || true)
        product=$(cat "$d/idProduct" 2>/dev/null || true)
        if [ "$vendor" = "8086" ] && [ "$product" = "0b3a" ]; then
            echo "$d"
            return 0
        fi
    done
    return 1
}

# 自动查找 Microdia 下视 UVC（USB 0c45:6366 / 名称含 LRCP|Microdia）
# 优先带 Video Capture 且能报 Width/Height 的节点（避开 metadata）
# 即使 /dev/video0 存在（常被 D435i 占用），也不会误选。
find_microdia_device() {
    local prefer="${1:-auto}"
    local v name real d idv idp
    local -a ranked=()

    _microdia_match() {
        local dev="$1" name="$2" idv="$3" idp="$4"
        case "${idv}:${idp}" in
            0c45:6366) return 0 ;;
        esac
        echo "$name" | grep -qiE 'LRCP|Microdia|Vitade'
    }

    for v in /sys/class/video4linux/video*; do
        [ -e "$v" ] || continue
        name=$(cat "$v/name" 2>/dev/null || true)
        real=$(readlink -f "$v/device" 2>/dev/null || true)
        idv=""; idp=""
        d="$real"
        while [ -n "$d" ] && [ "$d" != "/" ]; do
            if [ -f "$d/idVendor" ]; then
                idv=$(cat "$d/idVendor" 2>/dev/null || true)
                idp=$(cat "$d/idProduct" 2>/dev/null || true)
                break
            fi
            d=$(dirname "$d")
        done
        local dev="/dev/$(basename "$v")"
        [ -e "$dev" ] || continue
        _microdia_match "$dev" "$name" "$idv" "$idp" || continue

        local rank=1
        if command -v v4l2-ctl >/dev/null 2>&1; then
            if v4l2-ctl -d "$dev" --all 2>/dev/null | grep -q "Width/Height"; then
                rank=0
            fi
        fi
        ranked+=("${rank}:${dev}")
    done

    if [ "${#ranked[@]}" -eq 0 ]; then
        return 1
    fi

    # Prefer capture (rank 0), then lowest video index
    local best=""
    IFS=$'\n' sorted=($(printf '%s\n' "${ranked[@]}" | sort -t: -k1,1n -k2,2))
    unset IFS
    best="${sorted[0]#*:}"

    if [ -n "$prefer" ] && [ "$prefer" != "auto" ]; then
        if [ -e "$prefer" ]; then
            for item in "${ranked[@]}"; do
                if [ "${item#*:}" = "$prefer" ]; then
                    echo "$prefer"
                    return 0
                fi
            done
            # Explicit path exists but is not Microdia - fall back to auto
            echo "$best"
            return 0
        fi
    fi

    echo "$best"
    return 0
}

# ── RealSense USB 软复位 ──
# Jetson + librealsense V4L2 后端在 Failed-to-resolve / VIDIOC_QBUF 后，
# /dev/video* 会消失；对 D435i 做 authorized 0→1 通常可恢复。
reset_realsense_usb() {
    case "${RESET_REALSENSE_USB:-0}" in
        0|false|no|off)
            log "跳过 RealSense USB 复位 (RESET_REALSENSE_USB=0)"
            return 0
            ;;
    esac

    local rs_sysfs=""
    rs_sysfs=$(find_realsense_sysfs || true)

    if [ -z "$rs_sysfs" ]; then
        error "未检测到 D435i (USB 8086:0b3a)！当前 lsusb 无 Intel RealSense。"
        warn "请拔插 D435i（或重启 USB 口/整机）后再运行。"
        warn "可用: lsusb | grep -i RealSense"
        return 1
    fi

    log "复位 RealSense USB ($rs_sysfs)..."
    if docker exec -u root "$CONTAINER_NAME" bash --noprofile --norc -c "
            echo 0 > '$rs_sysfs/authorized' && sleep 2 && echo 1 > '$rs_sysfs/authorized'
        " 2>/dev/null; then
        sleep 5
        if find_realsense_sysfs >/dev/null && ls /dev/video* >/dev/null 2>&1; then
            ok "RealSense USB 已复位，video 节点: $(ls /dev/video* 2>/dev/null | tr '\n' ' ')"
            return 0
        fi
        warn "USB 复位后 D435i/video 未就绪，请拔插相机"
        return 1
    fi
    warn "无法经容器 root 复位 USB；可手动拔插 D435i"
    return 1
}

# 快速判断 topic 是否有 publisher（比 topic echo 轻量，避免"假死"感）
topic_has_publisher() {
    local topic="$1"
    local n
    n=$(topic_count "$topic" "Publisher count")
    [ -n "$n" ] && [ "$n" -gt 0 ]
}

# ── 启动视觉定位 (容器内) ──
# 方案A vslam_cuvslam.launch.py / 方案B vslam_rtabmap.launch.py
# 两套 launch 的参数语义统一（imu_source / d435_*），此函数只负责按方案拼参数
start_container_slam() {
    if ! reset_realsense_usb; then
        error "D435i 未就绪，放弃启动容器 SLAM"
        return 1
    fi

    # 清掉残留 SLAM 进程 + FastDDS SHM，避免"有 topic 无数据 / SHM lock"
    docker exec -u root "$CONTAINER_NAME" \
        bash /workspaces/ros2_ws/scripts/_cleanup_slam.sh >/dev/null 2>&1 || true

    # JetPack 6: 可选 RSUSB librealsense；D435i FW 建议 5.13.0.55（5.17 上 HID IMU 常无帧）
    local use_rsusb="${USE_REALSENSE_RSUSB:-1}"
    local rsusb_export=""
    case "${use_rsusb,,}" in
        1|true|yes|on)
            rsusb_export='if [ -d /opt/realsense_rsusb/lib ]; then export LD_LIBRARY_PATH=/opt/realsense_rsusb/lib:${LD_LIBRARY_PATH}; fi'
            ;;
    esac

    # 两方案通用参数
    local -a launch_args=(
        "rviz:=false"
        "imu_source:=$IMU_SOURCE"
        "d435_emitter_enabled:=$D435_EMITTER_ENABLED"
        "d435_auto_exposure:=$D435_AUTO_EXPOSURE"
        "d435_exposure_us:=$D435_EXPOSURE_US"
        "d435_gain:=$D435_GAIN"
        "d435_initial_reset:=${RESET_REALSENSE_USB:-0}"
    )

    if [ "$SLAM_SCHEME" = "rtabmap" ]; then
        # ── 方案B: stereo_odometry + RTAB-Map（RUN_MODE 决定建图/定位/仅里程计）──
        local slam_mode="" slam_db="" extra_rtabmap_args=""
        case "${RUN_MODE,,}" in
            debug)
                slam_mode="odometry"
                log "RUN_MODE=debug - 仅 rtabmap stereo_odometry（不启动 RTAB-Map）"
                ;;
            mapping)
                slam_mode="mapping"
                local ts
                ts=$(date +%Y%m%d_%H%M%S)
                slam_db="$CONTAINER_WS/maps/drafts/rtabmap_${ts}.db"
                extra_rtabmap_args="--delete_db_on_start"
                mkdir -p "$ROS2_WS/maps/drafts"
                log "RUN_MODE=mapping - RTAB-Map 边飞边建图 → $ROS2_WS/maps/drafts/rtabmap_${ts}.db"
                ;;
            *)
                slam_mode="localization"
                slam_db="$CONTAINER_PRODUCTION_DB"
                log "RUN_MODE=production - RTAB-Map 只读定位: $CONTAINER_PRODUCTION_DB"
                ;;
        esac
        [ -n "$slam_db" ] && launch_args+=("database_path:='$slam_db'")
        [ -n "$extra_rtabmap_args" ] && launch_args+=("rtabmap_args:='$extra_rtabmap_args'")
        launch_args+=("mode:=$slam_mode" "rtabmap_viz:=false")
    else
        # ── 方案A: 纯 cuVSLAM（无 RTAB-Map/地图库）──
        log "SLAM_SCHEME=cuvslam - Isaac cuVSLAM GPU VIO（imu_source=$IMU_SOURCE）"
        launch_args+=(
            "vslam_enable_denoising:=$VSLAM_ENABLE_DENOISING"
            "vslam_tracking_mode:=$VSLAM_TRACKING_MODE"
            "vslam_image_jitter_ms:=$VSLAM_IMAGE_JITTER_MS"
        )
    fi

    log "容器内启动: $SLAM_LAUNCH_FILE ${launch_args[*]}"
    docker exec -d "$CONTAINER_NAME" \
        bash -c "
            source /opt/ros/humble/setup.bash && \
            $rsusb_export && \
            ros2 launch \"$SLAM_LAUNCH_FILE\" \
                ${launch_args[*]} \
                > \"$SLAM_LOG\" 2>&1
        "

    # 确保 ROS2 环境已加载 (ros2 topic list 依赖此环境)
    source /opt/ros/humble/setup.bash 2>/dev/null || true

    # 等待相机图像 + VSLAM 里程计真正有数据（仅 topic 存在不够）
    log "等待相机/VSLAM 数据流 (最多 60s，每 5s 汇报进度)..."
    local timeout=60
    local elapsed=0
    local have_img=0 have_odom=0
    local img_pub=0 odom_pub=0
    while [ $elapsed -lt $timeout ]; do
        # 相机若已报 disconnected / Failed to resolve，尽早退出，避免空等
        if docker exec "$CONTAINER_NAME" bash --noprofile --norc -c \
            "grep -qE 'device has been disconnected|Failed to resolve the request|Error starting device' $SLAM_LOG 2>/dev/null"; then
            error "RealSense 启动失败（见容器日志），停止空等"
            docker exec "$CONTAINER_NAME" bash --noprofile --norc -c \
                "grep -E 'ERROR|Failed|disconnected|QBUF' $SLAM_LOG | tail -20" 2>/dev/null || true
            return 1
        fi

        if [ "$have_img" -eq 0 ]; then
            img_pub=$(topic_count "/camera/camera/infra1/image_rect_raw" "Publisher count")
            [ -n "$img_pub" ] || img_pub=0
            # Image is BEST_EFFORT; default echo QoS + short timeout often false-negatives.
            # Publisher-count discovery can lag for composable/Nitros image
            # endpoints even while BEST_EFFORT samples are flowing. The
            # successful QoS-matched sample is the authoritative readiness
            # check, so do not gate it on img_pub.
            if timeout 8 ros2 topic echo \
                    /camera/camera/infra1/image_rect_raw --once \
                    --qos-reliability best_effort --qos-durability volatile \
                    >/dev/null 2>&1; then
                have_img=1
                ok "D435i infra1 有图像 (${elapsed}s)"
            fi
        fi
        if [ "$have_odom" -eq 0 ]; then
            odom_pub=$(topic_count "/visual_slam/tracking/odometry" "Publisher count")
            [ -n "$odom_pub" ] || odom_pub=0
            if [ "$odom_pub" -gt 0 ] && timeout 8 ros2 topic echo \
                    /visual_slam/tracking/odometry --once \
                    --qos-reliability best_effort --qos-durability volatile \
                    >/dev/null 2>&1; then
                have_odom=1
                ok "VSLAM 里程计有数据 (${elapsed}s)"
            fi
        fi
        if [ "$have_img" -eq 1 ] && [ "$have_odom" -eq 1 ]; then
            return 0
        fi

        if [ $((elapsed % 5)) -eq 0 ]; then
            log "仍在等待... ${elapsed}s/${timeout}s (infra_pub=${img_pub:-0} odom_pub=${odom_pub:-0} img_ok=$have_img odom_ok=$have_odom)"
        fi
        sleep 5
        elapsed=$((elapsed + 5))
    done
    if [ "$have_img" -eq 0 ]; then
        warn "D435i 图像在 ${timeout}s 内无数据 - 检查: docker exec $CONTAINER_NAME grep -E 'ERROR|QBUF|Failed' $SLAM_LOG | tail -30"
    fi
    if [ "$have_odom" -eq 0 ]; then
        warn "VSLAM 里程计在 ${timeout}s 内无数据"
        warn "常见原因: IMU 失败导致 VIO 无输出；或相机对着无纹理表面未完成初始化"
        warn "检查: docker exec $CONTAINER_NAME grep -E 'Motion Module|imu|ERROR' $SLAM_LOG | tail -30"
        warn "并确认: ros2 topic hz /camera/camera/imu   # 若一直无数据，需关 IMU 融合或修 Motion Module"
    fi
    return 1
}

microxrce_agent_pid() {
    pgrep -f "MicroXRCEAgent serial -D $PX4_SERIAL -b $PX4_BAUD" 2>/dev/null | head -1 || true
}

start_microxrce_agent() {
    local agent_pid
    agent_pid=$(microxrce_agent_pid)

    if [ -n "$agent_pid" ]; then
        ok "MicroXRCEAgent 已运行 (PID: $agent_pid)，复用现有 PX4 XRCE 会话"
        return 0
    fi

    if pgrep -f "MicroXRCEAgent" >/dev/null 2>&1; then
        warn "检测到其他 MicroXRCEAgent 进程，但不是 $PX4_SERIAL@$PX4_BAUD"
        warn "如需强制清理: $0 reset-agent"
    fi

    log "启动常驻 MicroXRCEAgent ($PX4_SERIAL @ $PX4_BAUD)..."
    setsid MicroXRCEAgent serial -D "$PX4_SERIAL" -b "$PX4_BAUD" \
        > "$LOG_DIR/microxrce_agent.log" 2>&1 < /dev/null &

    sleep 2
    agent_pid=$(microxrce_agent_pid)
    if [ -n "$agent_pid" ]; then
        ok "MicroXRCEAgent 已启动 (PID: $agent_pid)"
        log "Agent 日志: $LOG_DIR/microxrce_agent.log"
    else
        error "MicroXRCEAgent 启动失败，请查看: $LOG_DIR/microxrce_agent.log"
        exit 1
    fi
}

# ── 启动 PX4 Bridge (宿主机) ──
start_px4_bridge() {
    log "启动宿主机 PX4 Bridge..."
    local use_rtabmap=0
    # 仅 rtabmap 方案的 production 模式用回环 relay；debug/mapping 无生产地图可定位
    case "${RUN_MODE,,}" in
        debug|mapping|odometry) USE_RTABMAP_RELAY=0 ;;
    esac
    case "${USE_RTABMAP_RELAY,,}" in 1|true|yes|on) use_rtabmap=1 ;; esac

    local relay_args=()
    local odom_topic='/visual_slam/tracking/odometry'

    if [ "$use_rtabmap" = "1" ]; then
        odom_topic='/rtabmap/relay/odometry'
        relay_args=('use_rtabmap_relay:=true')
        log "  RTAB-Map 回环修正融合已启用"
        log "  odom 输入: $odom_topic"
    fi

    # 根据 odom 源选择合适的方差（可通过环境变量覆盖）
    local xy_var="${POSITION_VARIANCE_XY:-0.05}"
    local vel_var="${VELOCITY_VARIANCE:-0.08}"
    log "  position_variance_xy=$xy_var, velocity_variance=$vel_var"

    # PX4 IMU relay 由 _parallel_phase2 统一启动（IMU_SOURCE=px4），此处不再重复拉起
    if [ "$USE_PX4_IMU_RELAY" = "1" ]; then
        log "  ★ IMU_SOURCE=px4: /fmu/out/sensor_combined → /camera/camera/imu_FC (Phase 2 已启动 relay)"
    fi

    setsid bash -lc "
        source /opt/ros/humble/setup.bash
        source '$ROS2_WS/install/setup.bash'
        exec ros2 launch px4_interface vslam_px4.launch.py \
            vslam_odom_topic:='$odom_topic' \
            px4_serial_device:='$PX4_SERIAL' \
            px4_serial_baud:='$PX4_BAUD' \
            start_microxrce_agent:=false \
            position_variance_xy:=$xy_var \
            velocity_variance:=$vel_var \
            ${relay_args[@]}
    " > "$LOG_DIR/px4_bridge.log" 2>&1 < /dev/null &
    HOST_LAUNCH_PID=$!

    log "PX4 Bridge 已启动 (PID: $HOST_LAUNCH_PID)"
    log "日志: $LOG_DIR/px4_bridge.log"

    # 可选: 同步漂移实验日志（CSV + rosbag + 配置快照）
    case "${DRIFT_LOG,,}" in
        1|true|yes|on)
            log "启动同步漂移实验日志..."
            setsid bash -lc "
                source /opt/ros/humble/setup.bash
                source '$ROS2_WS/install/setup.bash'
                exec '$ROS2_WS/scripts/flight_experiment.sh' auto
            " > "$LOG_DIR/flight_experiment.log" 2>&1 < /dev/null &
            local recorder_pid=$!
            sleep 2
            if kill -0 "$recorder_pid" 2>/dev/null; then
                log "Experiment Recorder PID: $recorder_pid → $ROS2_WS/logs/experiment_*"
            else
                error "实验记录器启动失败: $LOG_DIR/flight_experiment.log"
                return 1
            fi
            ;;
    esac
}

# ── RPLidar 电机预启动 (DTR 上电) ──
# RPLidar A2M8 电机需要 DTR=1 后 ~2s 才能稳定旋转。
# rplidar_ros SDK v2.1 的启动超时太短，需要在节点启动前预先给电机上电。
prime_rplidar_motor() {
    local port="${RPLIDAR_PORT:-/dev/rplidar}"
    case "${START_RPLIDAR,,}" in 1|true|yes|on) ;; *) return 0 ;; esac
    if [ ! -e "$port" ]; then
        warn "RPLIDAR $port 不存在，跳过电机预启动"
        return 0
    fi
    # Check if port is already occupied
    if fuser "$port" 2>/dev/null; then
        return 0  # port already in use, motor is probably running
    fi
    python3 -c "
import serial, time
try:
    s = serial.Serial('$port', 115200, timeout=1)
    s.dtr = True
    time.sleep(2.5)
    s.close()
except Exception as e:
    print(f'RPLidar motor prime warning: {e}')
" 2>/dev/null || true
}

# ── 可选: RPLIDAR / Microdia / obstacle_distance ──
start_extra_sensors() {
    local want_rplidar=0 want_microdia=0 want_od=0
    case "${START_RPLIDAR,,}" in 1|true|yes|on) want_rplidar=1 ;; esac
    case "${START_MICRODIA,,}" in 1|true|yes|on) want_microdia=1 ;; esac
    case "${START_OBSTACLE_DISTANCE,,}" in 1|true|yes|on) want_od=1 ;; esac

    if [ "$want_rplidar" -eq 0 ] && [ "$want_microdia" -eq 0 ] && [ "$want_od" -eq 0 ]; then
        log "额外传感器未启用 (START_RPLIDAR / START_MICRODIA / START_OBSTACLE_DISTANCE)"
        return 0
    fi

    if [ "$want_od" -eq 1 ] && [ "$want_rplidar" -eq 0 ]; then
        warn "START_OBSTACLE_DISTANCE=1 需要雷达数据，自动启用 START_RPLIDAR"
        want_rplidar=1
    fi

    if [ "$want_rplidar" -eq 1 ] && [ ! -e "$RPLIDAR_PORT" ]; then
        warn "RPLIDAR 设备不存在: $RPLIDAR_PORT，跳过雷达启动"
        want_rplidar=0
        want_od=0
    fi

    if [ "$want_microdia" -eq 1 ]; then
        local resolved=""
        if resolved=$(find_microdia_device "$MICRODIA_DEVICE"); then
            MICRODIA_DEVICE="$resolved"
            # 方便人工排查；失败则忽略（无需 root）
            ln -sfn "$MICRODIA_DEVICE" /dev/microdia 2>/dev/null || true
            ok "Microdia 下视相机: $MICRODIA_DEVICE"
        else
            warn "未找到 Microdia (USB 0c45:6366 / LRCP)；当前 MICRODIA_DEVICE=$MICRODIA_DEVICE，跳过下视相机"
            want_microdia=0
        fi
    fi

    if [ "$want_rplidar" -eq 0 ] && [ "$want_microdia" -eq 0 ] && [ "$want_od" -eq 0 ]; then
        return 0
    fi

    log "启动额外传感器 (rplidar=$want_rplidar microdia=$want_microdia obstacle_distance=$want_od)..."
    if [ "$want_microdia" -eq 1 ]; then
        log "  microdia_device=$MICRODIA_DEVICE"
    fi
    setsid bash -lc "
        source /opt/ros/humble/setup.bash
        source '$ROS2_WS/install/setup.bash'
        exec ros2 launch uav_task sensors_extra.launch.py \
            start_rplidar:=$want_rplidar \
            start_microdia:=$want_microdia \
            start_obstacle_distance:=$want_od \
            rplidar_port:='$RPLIDAR_PORT' \
            microdia_device:='$MICRODIA_DEVICE'
    " > "$LOG_DIR/sensors_extra.log" 2>&1 < /dev/null &
    EXTRA_SENSORS_PID=$!
    log "额外传感器已启动 (PID: $EXTRA_SENSORS_PID)，日志: $LOG_DIR/sensors_extra.log"
}

# ── 状态检查 ──
check_status() {
    log ""
    log "══════════════════════════════════════"
    log "  系统状态检查"
    log "══════════════════════════════════════"

    # 检查 topic
    local topic_list node_list
    topic_list=$(ros2 topic list --no-daemon --spin-time "$ROS_DISCOVERY_SPIN_TIME" 2>/dev/null || true)
    node_list=$(ros2 node list --no-daemon --spin-time "$ROS_DISCOVERY_SPIN_TIME" 2>/dev/null || true)

    local topics=(
        "/visual_slam/tracking/odometry"
        "/fmu/in/vehicle_visual_odometry"
        "/fmu/in/obstacle_distance"
        "/scan"
    )

    for topic in "${topics[@]}"; do
        if echo "$topic_list" | grep -q "$topic"; then
            ok "Topic: $topic"
        else
            warn "Topic: $topic (未检测到)"
        fi
    done

    # 检查关键节点（按 SLAM_SCHEME 区分）
    local nodes=(
        "vslam_odom_bridge"
        "px4_gateway_node"
        "rplidar_node"
    )
    case "$SLAM_SCHEME" in
        cuvslam)
            nodes=("visual_slam_node" "${nodes[@]}")
            ;;
        rtabmap)
            nodes=("stereo_odometry" "rtabmap" "${nodes[@]}")
            ;;
    esac
    for node in "${nodes[@]}"; do
        if echo "$node_list" | grep -q "$node"; then
            ok "Node: $node"
        else
            warn "Node: $node (未检测到)"
        fi
    done

    log ""
    log "── 关键 DDS endpoint 检查 ──"
    wait_for_px4_dds 30 || true

    local vslam_pub vslam_sub vio_pub vio_sub status_pub
    vslam_pub=$(topic_count "/visual_slam/tracking/odometry" "Publisher count")
    vslam_sub=$(topic_count "/visual_slam/tracking/odometry" "Subscription count")
    vio_pub=$(topic_count "/fmu/in/vehicle_visual_odometry" "Publisher count")
    vio_sub=$(topic_count "/fmu/in/vehicle_visual_odometry" "Subscription count")
    status_pub=$(topic_count "/fmu/out/vehicle_status_v1" "Publisher count")

    [ -n "$vslam_pub" ] || vslam_pub=0
    [ -n "$vslam_sub" ] || vslam_sub=0
    [ -n "$vio_pub" ] || vio_pub=0
    [ -n "$vio_sub" ] || vio_sub=0
    [ -n "$status_pub" ] || status_pub=0

    if [ "$vslam_pub" -eq 1 ]; then
        ok "/visual_slam/tracking/odometry publisher=$vslam_pub subscriber=$vslam_sub"
    else
        warn "/visual_slam/tracking/odometry publisher=$vslam_pub (期望=1，若>1说明有残留 SLAM 进程)"
    fi

    if [ "$vio_pub" -eq 1 ]; then
        ok "/fmu/in/vehicle_visual_odometry publisher=$vio_pub"
    elif [ "$vio_pub" -gt 1 ]; then
        warn "/fmu/in/vehicle_visual_odometry publisher=$vio_pub (期望=1，重复 Bridge 会导致 EKF 拒绝视觉融合)"
        warn "请先执行: $0 stop && $0 --bg"
    else
        warn "/fmu/in/vehicle_visual_odometry publisher=$vio_pub (桥接节点未发布)"
    fi

    if [ "$vio_sub" -ge 1 ] && [ "$status_pub" -ge 1 ]; then
        ok "PX4 DDS 已建立: vehicle_visual_odometry subscriber=$vio_sub, vehicle_status publisher=$status_pub"
    else
        warn "PX4 DDS 未建立: vehicle_visual_odometry subscriber=$vio_sub, vehicle_status publisher=$status_pub"
        warn "请优先检查 PX4 参数/串口: UXRCE_DDS_CFG=TELEM1, SER_TEL1_BAUD=921600, /dev/ttyTHS1, TX/RX/GND"
        warn "MicroXRCEAgent 日志应出现 client session；若只有 running/fd 信息，说明 PX4 client 没连上 Agent"
    fi

    log ""
    log "── PX4 向下测距检查 ──"
    local local_position estimator_flags dist_bottom cs_rng_hgt cs_ev_hgt
    local i
    # PX4 刚建立 uXRCE session 时，新 ROS CLI 订阅者的 DDS 发现常超过 4 秒。
    # 使用已经运行的 ROS daemon，并给首个样本留足时间，避免把超时误报为无测距。
    for i in 1 2 3; do
        local_position=$(timeout 8 ros2 topic echo \
            /fmu/out/vehicle_local_position --once 2>/dev/null || true)
        estimator_flags=$(timeout 8 ros2 topic echo \
            /fmu/out/estimator_status_flags --once 2>/dev/null || true)
        dist_bottom=$(echo "$local_position" | awk '/^dist_bottom:/{print $2; exit}')
        if [ -n "$local_position" ] && [ -n "${dist_bottom}" ]; then
            break
        fi
        [ "$i" -lt 5 ] && sleep 2
    done
    cs_rng_hgt=$(echo "$estimator_flags" | awk '/^cs_rng_hgt:/{print $2; exit}')
    cs_ev_hgt=$(echo "$estimator_flags" | awk '/^cs_ev_hgt:/{print $2; exit}')

    if echo "$local_position" | grep -q "dist_bottom_valid: true"; then
        ok "TFmini 测距正常 dist_bottom_valid=true (离地: ${dist_bottom:-unknown}m)"
    elif [ "${cs_rng_hgt}" = "true" ] && [ -n "${dist_bottom}" ]; then
        ok "TFmini 已融合 EKF: dist_bottom=${dist_bottom}m, cs_rng_hgt=true (dist_bottom_valid 可为 false)"
    elif [ -n "${dist_bottom}" ] && awk -v d="$dist_bottom" 'BEGIN{exit !(d+0 > 0.05)}'; then
        warn "TFmini 有读数 dist_bottom=${dist_bottom}m 但未融合 (cs_rng_hgt≠true)"
        warn "需要 EKF2_HGT_REF=2、EKF2_RNG_CTRL=2、EKF2_BARO_CTRL=0、SENS_TFMINI_CFG=102"
        warn "若启动时 MAVLink 写参失败，用 QGC/USB 设参后重启飞控；或 CONFIGURE_PX4_TFMINI_HEIGHT=1 重跑"
    else
        warn "TFmini 无有效 dist_bottom；检查 SENS_TFMINI_CFG=102、TX→TELEM2 RX 和供电"
    fi
    local cs_ev_pos cs_ev_vel cs_baro_hgt cs_ev_yaw
    cs_ev_pos=$(echo "$estimator_flags" | awk '/^cs_ev_pos:/{print $2; exit}')
    cs_ev_vel=$(echo "$estimator_flags" | awk '/^cs_ev_vel:/{print $2; exit}')
    cs_baro_hgt=$(echo "$estimator_flags" | awk '/^cs_baro_hgt:/{print $2; exit}')
    cs_ev_yaw=$(echo "$estimator_flags" | awk '/^cs_ev_yaw:/{print $2; exit}')
    if [ "${cs_baro_hgt}" = "true" ]; then
        warn "气压高度仍在融合 (cs_baro_hgt=true)；室内应 EKF2_BARO_CTRL=0"
    fi
    if [ "${cs_ev_pos}" = "true" ] && [ "${cs_ev_vel}" = "true" ]; then
        if [ "${cs_ev_hgt}" = "true" ]; then
            ok "视觉 EV 已融合: pos+vel+yaw=true, ev_hgt=true (含视觉高度) [SLAM_SCHEME=$SLAM_SCHEME IMU_SOURCE=$IMU_SOURCE]"
        elif [ "${cs_ev_yaw}" = "true" ]; then
            ok "视觉 EV 已融合: pos+vel+yaw=true, ev_hgt=false [SLAM_SCHEME=$SLAM_SCHEME IMU_SOURCE=$IMU_SOURCE]"
        else
            ok "视觉 EV 已融合: pos+vel=true, yaw=false, ev_hgt=false [SLAM_SCHEME=$SLAM_SCHEME IMU_SOURCE=$IMU_SOURCE]"
        fi
    elif [ "${cs_ev_pos}" = "true" ]; then
        warn "视觉 EV 仅融合位置 (cs_ev_pos=true, cs_ev_vel=false); Bridge 20 帧后复查 [SLAM_SCHEME=$SLAM_SCHEME IMU_SOURCE=$IMU_SOURCE]"
    else
        warn "视觉 EV 未融合 (cs_ev_pos=false, cs_ev_vel=false) [SLAM_SCHEME=$SLAM_SCHEME IMU_SOURCE=$IMU_SOURCE]"
        warn "Bridge 可能正在初始化或持续 reset；检查: tail -f $LOG_DIR/px4_bridge.log"
    fi

    log ""
    log "── Microdia 下视相机检查 ──"
    local md_dev md_pub
    md_dev=$(find_microdia_device "${MICRODIA_DEVICE:-auto}" 2>/dev/null || true)
    if [ -n "$md_dev" ]; then
        ok "Microdia 设备: $md_dev"
    else
        warn "未枚举到 Microdia UVC (USB 0c45:6366)；检查接线/供电"
    fi
    md_pub=$(topic_count "/image_raw" "Publisher count")
    [ -n "$md_pub" ] || md_pub=0
    if [ "$md_pub" -ge 1 ]; then
        ok "Topic: /image_raw publisher=$md_pub (下视)"
    else
        warn "Topic: /image_raw 无 publisher（设备: ${md_dev:-未找到}）"
        warn "若节点曾崩溃，查看: tail -40 /tmp/slam_px4_logs/sensors_extra.log"
        warn "常见原因: v4l2_camera 不支持 MJPG（已改为 YUYV 640x480）"
    fi
    if echo "$node_list" | grep -q "microdia_downward_camera"; then
        ok "Node: microdia_downward_camera"
    elif echo "$node_list" | grep -q "v4l2_camera"; then
        ok "Node: v4l2_camera (下视相机)"
    else
        warn "Node: microdia_downward_camera (未检测到；进程可能已崩溃)"
    fi
    if echo "$node_list" | grep -q "circle_landing_target"; then
        ok "Node: circle_landing_target (视觉降落)"
    else
        warn "Node: circle_landing_target (未检测到；Microdia 未启动时属正常)"
    fi

    log ""
    local slam_tf_ok=false relay_pub=0 relay_rate=0
    if [ "$SLAM_SCHEME" = "cuvslam" ]; then
        # ── 方案A: cuVSLAM 无地图/无回环，只看 vo_state 与 odom→base_link TF ──
        log "── 方案A cuVSLAM 定位检查 ──"
        local vo_state
        vo_state=$(timeout 5 ros2 topic echo /visual_slam/status --once --no-daemon \
            --spin-time "$ROS_DISCOVERY_SPIN_TIME" 2>/dev/null |
            awk '/^vo_state:/{print $2; exit}')
        if [ "${vo_state:-x}" = "1" ]; then
            ok "cuVSLAM vo_state=1 (TRACKING)"
        else
            warn "cuVSLAM vo_state=${vo_state:-unknown}（期望 1；非 1=丢失跟踪: 纹理/光照/IMU/振动）"
        fi
        if timeout 5 ros2 run tf2_ros tf2_echo odom base_link \
            2>/dev/null | grep -q "Translation"; then
            ok "TF: odom→base_link 存在"
        else
            warn "TF: odom→base_link 无数据（cuVSLAM 未初始化）"
        fi
        warn "方案A 无地图闭环：漂移不会被修正，长距离 A→B 前请先过 isaac_vio_debug/vio_stack.sh validate 门限"
        if [ "$vslam_pub" -ge 1 ]; then
            ok "定位融合: OK (无闭环校正)"
        else
            warn "定位融合: DEGRADED - 无 /visual_slam/tracking/odometry"
        fi
        log ""
    else
    # ── 方案B: RTAB-Map 地图定位 + 闭环 ──
    log "── 方案B RTAB-Map 定位检查 ──"

    if timeout 5 ros2 run tf2_ros tf2_echo map base_link \
        2>/dev/null | grep -q "Translation"; then
        slam_tf_ok=true
        ok "TF: map→base_link 存在"
    else
        slam_tf_ok=false
        warn "TF: map→base_link 无数据（生产地图尚未定位成功）"
    fi
    # Require a live geometric match (proximity/loop). After initialpose,
    # localization_pose can report low covariance without map lock - that
    # false trust followed VIO climb slip in flight_pose_20260804_225335.
    # Relay only publishes odometry while a recent proximity/loop is held.
    local loc_ids
    loc_ids=$(timeout 5 ros2 topic echo /rtabmap/info --once --no-daemon \
        --spin-time "$ROS_DISCOVERY_SPIN_TIME" 2>/dev/null |
        awk '
          $1=="loop_closure_id:" {lc=$2}
          $1=="proximity_detection_id:" {pr=$2}
          END {print lc+0, pr+0}
        ')
    local lc_id pr_id
    lc_id=$(echo "$loc_ids" | awk '{print $1}')
    pr_id=$(echo "$loc_ids" | awk '{print $2}')
    local cov_line cov_xx cov_yy cov_yaw cov_ok=0
    cov_line=$(timeout 5 ros2 topic echo /rtabmap/localization_pose --once \
        --no-daemon --spin-time "$ROS_DISCOVERY_SPIN_TIME" 2>/dev/null |
        awk '
          BEGIN {n=0}
          $1=="covariance:" {c=1; next}
          c && $1 ~ /^-/ {
            n++
            if (n==1) xx=$2
            if (n==8) yy=$2
            if (n==36) yaw=$2
          }
          END {
            if (xx == "" || yy == "") print "9999 9999 9999"
            else print xx+0, yy+0, ((yaw=="") ? 9999 : yaw+0)
          }
        ')
    cov_xx=$(echo "$cov_line" | awk '{print $1}')
    cov_yy=$(echo "$cov_line" | awk '{print $2}')
    cov_yaw=$(echo "$cov_line" | awk '{print $3}')
    if awk -v xx="${cov_xx:-9999}" -v yy="${cov_yy:-9999}" -v yaw="${cov_yaw:-9999}" \
        'BEGIN { exit !((xx+0) < 10.0 && (yy+0) < 10.0 && (yaw+0) < 10.0) }'
    then
        cov_ok=1
    fi
    relay_pub=$(topic_count "/rtabmap/relay/odometry" "Publisher count")
    relay_rate=$(ros2 topic hz /rtabmap/relay/odometry --no-daemon \
        --spin-time "$ROS_DISCOVERY_SPIN_TIME" 2>/dev/null |
        grep -oP 'average rate: \K[\d.]+' 2>/dev/null || echo "0")
    local relay_match_ok=0
    if awk -v r="${relay_rate:-0}" 'BEGIN { exit !(r+0 > 0.5) }'; then
        relay_match_ok=1
    fi
    if [ "${lc_id:-0}" -gt 0 ] || [ "${pr_id:-0}" -gt 0 ] || \
            [ "$relay_match_ok" -eq 1 ]; then
        ok "RTAB-Map 几何匹配有效 (info loop=$lc_id proximity=$pr_id; relay_rate=${relay_rate}Hz)"
    elif [ "$cov_ok" -eq 1 ]; then
        slam_tf_ok=false
        warn "RTAB-Map localization_pose 协方差看似可信，但尚无 proximity/loop - 禁止当作已锁图"
        warn "当前状态会像 flight_pose_20260804_225335：中心空洞 + VIO 爬升滑移被当成绝对 XY"
        warn "先给初始位姿并移到有节点的墙边等到匹配: python3 $ROS2_WS/scripts/set_rtabmap_initialpose.py"
        warn "或重建覆盖起飞点(中心)的生产地图后再 promote"
    else
        slam_tf_ok=false
        warn "RTAB-Map 尚未匹配生产地图 (loop=0 proximity=0, localization cov 不可信)"
        warn "先给初始位姿再慢转: python3 $ROS2_WS/scripts/set_rtabmap_initialpose.py"
        warn "若起飞点在地图轨迹空洞区(中心无节点)，需补走中心重建图或先在边缘锁上再移到中心"
    fi

    [ -n "$relay_pub" ] || relay_pub=0
    if [ -z "${relay_rate+x}" ]; then
        relay_rate=0
    fi
    if [ "$relay_pub" -ge 1 ] && [ "$relay_rate" != "0" ]; then
        ok "定位 relay publisher=$relay_pub rate=${relay_rate}Hz"
    elif [ "$relay_pub" -ge 1 ]; then
        ok "定位 relay publisher=$relay_pub (rate pending)"
    else
        warn "定位 relay publisher=$relay_pub (未启动或等待数据)"
    fi

    if $slam_tf_ok && [ "$relay_pub" -ge 1 ]; then
        ok "定位融合: OK"
    else
        warn "定位融合: DEGRADED - 检查生产地图和容器日志"
    fi
    fi

    log ""
    log "── RPLidarA2 /scan 检查（避障用，不参与定位）──"

    local scan_pub scan_sub
    scan_pub=$(topic_count "/scan" "Publisher count")
    scan_sub=$(topic_count "/scan" "Subscription count")
    [ -n "$scan_pub" ] || scan_pub=0
    [ -n "$scan_sub" ] || scan_sub=0

    if [ -e /dev/rplidar ]; then
        ok "RPLidar 设备符号链接存在: /dev/rplidar"
    elif ls /dev/ttyUSB* /dev/ttyACM* >/dev/null 2>&1; then
        warn "未发现 /dev/rplidar，但存在 USB 串口设备；请确认 udev 规则或使用实际串口启动 rplidar_ros"
    else
        warn "未发现 /dev/rplidar、/dev/ttyUSB* 或 /dev/ttyACM*，RPLidar 当前可能未插入/未上电/未枚举"
    fi

    if [ "$scan_pub" -ge 1 ]; then
        ok "/scan publisher=$scan_pub subscriber=$scan_sub"
    else
        warn "/scan publisher=$scan_pub (RPLidar 节点未发布 LaserScan)"
    fi

    local obstacle_pub obstacle_sub
    obstacle_pub=$(topic_count "/fmu/in/obstacle_distance" "Publisher count")
    obstacle_sub=$(topic_count "/fmu/in/obstacle_distance" "Subscription count")
    [ -n "$obstacle_pub" ] || obstacle_pub=0
    [ -n "$obstacle_sub" ] || obstacle_sub=0
    if [ "$obstacle_pub" -ge 1 ]; then
        ok "/fmu/in/obstacle_distance publisher=$obstacle_pub subscriber=$obstacle_sub"
    else
        warn "/fmu/in/obstacle_distance 无 publisher；PX4 仅靠任务节点 APF 避障"
        warn "默认应由 laser_scan_to_obstacle_distance 发布，检查: tail -40 $LOG_DIR/sensors_extra.log"
    fi

    if [ "$scan_sub" -ge 1 ]; then
        ok "/scan 有订阅者 (任务节点 APF / obstacle_distance 桥接已订阅雷达)"
    else
        warn "/scan subscriber=$scan_sub；雷达已发布但无人订阅"
        warn "检查: tail -40 $LOG_DIR/sensors_extra.log（laser_scan_to_obstacle_distance 是否起来）"
        warn "注意: 两套定位方案都不订阅 /scan，雷达只做避障"
    fi

    if [ "$SLAM_SCHEME" = "rtabmap" ]; then
        if echo "$node_list" | grep -qiE 'rtabmap'; then
            ok "Node: rtabmap 在线"
        else
            case "${RUN_MODE,,}" in
                debug) log "RUN_MODE=debug - 未启动 RTAB-Map（仅 stereo_odometry），符合预期" ;;
                *) warn "Node: rtabmap 未检测到；生产地图定位未运行" ;;
            esac
        fi
    fi

    log "══════════════════════════════════════"
    log ""
    log "如果以上 topic 都有数据，说明 ROS 端链路正常。"
    log ""
    log "── PX4 飞控端需要确认的参数 ──"
    log "  [EKF2 室内激光定高 + 视觉全自由度]"
    log "    EKF2_EV_CTRL = 15         (HPOS+VPOS+VEL+YAW；全自由度视觉融合)"
    log "    EKF2_EV_QMIN = 50         (拒绝失效视觉数据)"
    log "    EKF2_EV_POS_X/Y/Z = 0     (bridge 输出 base_link 位姿，勿重复补偿相机杠杆臂)"
    log "    EKF2_EVA_NOISE = 0.25     (姿态噪声)"
    log "    EKF2_EVP_NOISE = 0.20     (位置噪声)"
    log "    EKF2_EVV_NOISE = 0.15     (速度噪声)"
    log "    EKF2_HGT_REF = 2          (高度参考使用 TELEM2 测距)"
    log "    EKF2_BARO_CTRL = 0        (室内关气压)"
    log "    EKF2_MAG_TYPE = 5         (室内关磁力计)"
    log "    EKF2_GPS_CTRL = 0         (室内无 GPS)"
    log "    EKF2_OF_CTRL = 0          (室内无光流)"
    log "    EKF2_RNG_CTRL = 2         (始终融合测距; 激光主导定高)"
    log "    EKF2_RNG_A_VMAX = 2.0     (测距垂直加速度门限)"
    log "    EKF2_RNG_GATE = 6.0       (测距 innovation 门限)"
    log "    EKF2_RNG_K_GATE = 3.0     (测距 kalman gate)"
    log "    EKF2_RNG_NOISE = 0.15     (测距噪声标准差)"
    log "    EKF2_RNG_POS_Z = 0.05     (TFmini 位于 PX4/CG 下方 5cm)"
    log "    EKF2_EV_DELAY = 0         (外部视觉延迟)"
    log "    SENS_TFMINI_CFG = 102     (TFmini 使用 TELEM2)"
    log "  [不连QGC/不插USB也能解锁 - 关键!]"
    log "    COM_POWER_COUNT = 1       ★ 默认=2(需检测到USB+电池),"
    log "                               不插USB只有电池→飞前检查失败→红灯锁定!"
    log "    CBRK_IO_SAFETY = 22027    (旁路安全开关)"
    log "    COM_RC_IN_MODE = 2        (遥控器模式切换 + MAVLink兼容)"
    log "    COM_ARM_WO_GPS = 1        (允许无GPS解锁)"
    log "  [串口]"
    log "    UXRCE_DDS_CFG = TELEM1   (micro XRCE-DDS 串口映射)"
    log "    SER_TEL1_BAUD = 921600"
    log ""
    log "  确认 pre_flight_checks_pass = True 后即可解锁起飞"
    log "══════════════════════════════════════"
}

# ── 解锁条件检查 ──
# 通过 ros2 topic echo 读取飞控状态，判断是否满足解锁条件
check_arm_readiness() {
    source /opt/ros/humble/setup.bash 2>/dev/null
    source "$ROS2_WS/install/setup.bash" 2>/dev/null

    echo ""
    echo "╔══════════════════════════════════════════════╗"
    echo "║         PX4 解锁条件检查 (Arm Readiness)       ║"
    echo "╚══════════════════════════════════════════════╝"
    echo ""

    # ── 1. 检查 ROS 端连接 ──
    local ros_ok=true

    echo "── ROS 端链路 ──"

    if ros2 topic list --no-daemon --spin-time "$ROS_DISCOVERY_SPIN_TIME" 2>/dev/null | grep -q "/fmu/out/vehicle_status_v1"; then
        ok "飞控状态 topic /fmu/out/vehicle_status_v1 在线 (DDS 链路正常)"
    else
        error "飞控状态 topic 不在线! 请先启动 PX4 Bridge: $0 --bg"
        ros_ok=false
    fi

    if ros2 topic list --no-daemon --spin-time "$ROS_DISCOVERY_SPIN_TIME" 2>/dev/null | grep -q "/fmu/in/vehicle_visual_odometry"; then
        ok "视觉里程计输入 /fmu/in/vehicle_visual_odometry 已建立"
    else
        warn "视觉里程计 topic 不在线 (VSLAM 桥未启动?)"
        ros_ok=false
    fi

    echo ""

    if ! $ros_ok; then
        error "ROS 链路不完整，无法继续检查。请先启动系统。"
        return 1
    fi

    # ── 2. 抓取飞控状态 ──
    echo "── 正在读取飞控状态 (等待 2 秒)... ──"

    local vs_status
    vs_status=$(python3 -c "
import subprocess, sys, json, time
# 通过 ros2 topic echo --once 抓一帧 vehicle_status
result = subprocess.run(
    ['ros2', 'topic', 'echo', '--once', '/fmu/out/vehicle_status_v1', '--no-daemon', '--spin-time', '$ROS_DISCOVERY_SPIN_TIME'],
    capture_output=True, text=True, timeout=5
)
if result.returncode != 0:
    print('ERROR: ros2 topic echo 失败')
    print(result.stderr, file=sys.stderr)
    sys.exit(1)

# 解析 YAML 风格输出为 key=value
raw = result.stdout
lines = raw.strip().split('\n')
parsed = {}
for line in lines:
    line = line.strip()
    if ':' in line:
        k, v = line.split(':', 1)
        k = k.strip()
        v = v.strip()
        parsed[k] = v

# 输出 JSON 供 bash 解析
out = {
    'arming_state': int(parsed.get('arming_state', 0)),
    'nav_state': int(parsed.get('nav_state', 0)),
    'failsafe': parsed.get('failsafe', 'false').lower() == 'true',
    'pre_flight_checks_pass': parsed.get('pre_flight_checks_pass', 'false').lower() == 'true',
}
print(json.dumps(out))
" 2>&1)

    if [ $? -ne 0 ] || echo "$vs_status" | grep -q "ERROR"; then
        error "无法读取飞控状态:"
        echo "$vs_status"
        return 1
    fi

    # 解析 JSON
    local arming_state nav_state failsafe preflight_pass
    arming_state=$(echo "$vs_status" | python3 -c "import sys,json; print(json.load(sys.stdin)['arming_state'])")
    nav_state=$(echo "$vs_status" | python3 -c "import sys,json; print(json.load(sys.stdin)['nav_state'])")
    failsafe=$(echo "$vs_status" | python3 -c "import sys,json; print(json.load(sys.stdin)['failsafe'])")
    preflight_pass=$(echo "$vs_status" | python3 -c "import sys,json; print(json.load(sys.stdin)['pre_flight_checks_pass'])")

    # ── NAV_STATE 中文映射 ──
    local nav_name
    case $nav_state in
        0)  nav_name="MANUAL (手动)";;
        1)  nav_name="ALTCTL (定高)";;
        2)  nav_name="POSCTL (定点)";;
        3)  nav_name="AUTO_MISSION (任务)";;
        4)  nav_name="AUTO_LOITER (悬停)";;
        5)  nav_name="AUTO_RTL (返航)";;
        14) nav_name="OFFBOARD";;
        15) nav_name="STAB (自稳)";;
        17) nav_name="AUTO_TAKEOFF (自动起飞)";;
        *)  nav_name="未知($nav_state)";;
    esac

    # ── 3. 输出结果 ──
    echo ""
    echo "┌──────────────────────────────────────────────┐"
    echo "│            飞控实时状态                        │"
    echo "├──────────────────────────────────────────────┤"
    printf "│ %-30s %8s │\n" "arming_state (解锁状态)" "$([ "$arming_state" -eq 2 ] && echo "ARMED ✓" || echo "DISARMED")"
    printf "│ %-30s %8s │\n" "nav_state (飞行模式)" "$nav_name"
    printf "│ %-30s %8s │\n" "failsafe (故障保护)" "$([ "$failsafe" = "True" ] && echo "触发!!" || echo "正常")"
    printf "│ %-30s %8s │\n" "pre_flight_checks_pass" "$([ "$preflight_pass" = "True" ] && echo "通过 ✓" || echo "未通过 ✗")"
    echo "└──────────────────────────────────────────────┘"
    echo ""

    # ── 4. 综合判断 ──
    local all_ok=true

    if [ "$preflight_pass" != "True" ]; then
        error "飞前检查 未通过! 飞控无法解锁。"
        echo ""
        echo "  请用 QGC 查看飞控具体报错信息，常见原因:"
        echo "    ① COM_POWER_COUNT ≠ 1   (不插USB时电源检查失败 → 红灯)"
        echo "    ② CBRK_IO_SAFETY ≠ 22027 (安全开关未旁路)"
        echo "    ③ 传感器未校准 (加速度计/陀螺仪/罗盘)"
        echo "    ④ 电池电压过低"
        echo "    ⑤ GPS 丢失 (如果没设 COM_ARM_WO_GPS=1)"
        all_ok=false
    else
        ok "飞前检查 通过!"
    fi

    if [ "$failsafe" = "True" ]; then
        error "飞控处于 故障保护 状态! 请检查飞控 LED 或 QGC 查看具体原因。"
        all_ok=false
    fi

    # 检查 VSLAM 里程计是否有数据
    local odom_hz
    # `ros2 topic hz` does not support --no-daemon/--spin-time on Humble.
    # Use the environment switch and wait long enough for an actual average;
    # otherwise a healthy BEST_EFFORT cuSLAM stream is falsely reported dead.
    odom_hz=$(ROS2_DISABLE_DAEMON=1 timeout 6 ros2 topic hz \
        /visual_slam/tracking/odometry 2>/dev/null | \
        grep -oP 'average rate: \K[\d.]+' | tail -1 || echo "0")
    if [ "$odom_hz" = "0" ] || [ -z "$odom_hz" ]; then
        # topic hz 如果超时则尝试用 echo 测试
        local odom_msg
        odom_msg=$(ROS2_DISABLE_DAEMON=1 timeout 5 ros2 topic echo --once \
            /visual_slam/tracking/odometry \
            --qos-reliability best_effort --qos-durability volatile \
            2>/dev/null | head -1)
        if [ -z "$odom_msg" ]; then
            warn "VSLAM 里程计 无数据! 相机可能未初始化或朝向纯色表面。"
            echo "  → 确保相机朝向有纹理的地面/墙面 (不要对着纯色光滑表面)"
            all_ok=false
        else
            ok "VSLAM 里程计 有数据输出"
        fi
    else
        ok "VSLAM 里程计 频率: ${odom_hz} Hz"
    fi

    echo ""

    # ── 5. 最终结论 ──
    if $all_ok; then
        echo "╔══════════════════════════════════════════════╗"
        echo "║  ✓  所有解锁条件满足，可以尝试解锁起飞!       ║"
        echo "║                                              ║"
        echo "║  操作步骤:                                    ║"
        echo "║  1. 遥控器切 STABILIZED (自稳)               ║"
        echo "║  2. 解锁 (ARM switch)                        ║"
        echo "║  3. 推油门起飞                                 ║"
        echo "║  4. 悬停稳定后切 POSCTL 测试定点              ║"
        echo "╚══════════════════════════════════════════════╝"
    else
        echo "╔══════════════════════════════════════════════╗"
        echo "║  ✗  解锁条件不满足，请先解决上述红色问题。     ║"
        echo "╚══════════════════════════════════════════════╝"
    fi
    echo ""
}

# ── 停止 ──
stop_all() {
    log "停止所有进程..."
    release_start_lock
    pkill -f "[r]un_slam_px4.sh --bg" 2>/dev/null || true
    stop_host_processes
    stop_container_slam
    _cleanup_pids
    sleep 1
    # 验证无残留
    local residual
    residual=$(pgrep -a -f "vslam_px4.launch.py|vslam_cuvslam.launch.py|vslam_rtabmap.launch.py|vslam_odom_bridge|rplidar_node" 2>/dev/null || true)
    if [ -n "$residual" ]; then
        warn "仍有残留进程:"
        echo "$residual"
        warn "请手动 kill 或等待后重试 stop"
    else
        ok "所有 ROS/SLAM 进程已停止。MicroXRCEAgent 保持运行以避免 PX4 uXRCE client 卡死"
    fi
    log "如需重置 Agent: $0 reset-agent"
}

START_LOCK="/tmp/run_slam_px4.start.lock"

acquire_start_lock() {
    if [ -f "$START_LOCK" ]; then
        local old_pid
        old_pid=$(cat "$START_LOCK" 2>/dev/null || true)
        if [ -n "$old_pid" ] && kill -0 "$old_pid" 2>/dev/null; then
            error "另一个 run_slam_px4 仍在运行 (PID $old_pid)"
            error "请先运行: $0 stop"
            exit 1
        fi
        # Stale lock — remove it
        rm -f "$START_LOCK"
    fi
    echo $$ > "$START_LOCK"
}

release_start_lock() {
    rm -f "$START_LOCK"
}

reset_agent() {
    log "重置 MicroXRCEAgent..."
    stop_microxrce_agent
    sleep 1
    start_microxrce_agent
}

# ═══════════════════════════════════════════════════════════
#  并行启动核心流程
# ═══════════════════════════════════════════════════════════

# 并行启动阶段 1：无依赖组件同时启动
_parallel_phase1() {
    log "并行启动 Phase 1: MicroXRCEAgent + RPLidar + 容器 SLAM ..."

    # 1a. MicroXRCEAgent
    if [ "${GROUND_ONLY:-0}" != "1" ]; then
        start_microxrce_agent &
        _track_pid "microxrce_agent" $!
    fi

    # 1b. RPLidar (prime motor first, then start node)
    if [ "${START_RPLIDAR:-1}" = "1" ] || [ "${START_RPLIDAR,,}" = "true" ]; then
        prime_rplidar_motor
        START_MICRODIA=0 start_extra_sensors &
        _track_pid "sensors_extra" $!
    fi

    # 1c. Container VSLAM (D435i + cuVSLAM / rtabmap stereo_odometry)
    #     This is the slowest component (up to 60s for D435i init + VSLAM boot)
    start_container_slam &
    _track_pid "container_slam" $!
}

# 并行启动阶段 2：依赖 PX4 DDS
_parallel_phase2() {
    log "并行启动 Phase 2: PX4 IMU relay ..."

    # PX4 IMU relay (needs PX4 DDS up via MicroXRCEAgent)
    if [ "$USE_PX4_IMU_RELAY" = "1" ]; then
        (source /opt/ros/humble/setup.bash 2>/dev/null
         source "$ROS2_WS/install/setup.bash" 2>/dev/null
         exec python3 "$ROS2_WS/src/px4_interface/scripts/px4_imu_relay.py" \
            > "$LOG_DIR/px4_imu_relay.log" 2>&1 < /dev/null) &
        _track_pid "px4_imu_relay" $!
    fi
}

# 并行启动阶段 3：PX4 Bridge（依赖 VSLAM odom）
_parallel_phase3() {
    log "Phase 3: PX4 Bridge ..."
    start_px4_bridge
    _track_pid "px4_bridge" "${HOST_LAUNCH_PID:-$!}"
}

# 并行启动阶段 4：Microdia 下视相机（延迟启动，避免与 D435i 抢 UVC）
_parallel_phase4() {
    if [ "${START_MICRODIA:-1}" = "1" ] || [ "${START_MICRODIA,,}" = "true" ]; then
        log "Phase 4: Microdia 下视相机 (延迟启动)..."
        START_RPLIDAR=0 START_OBSTACLE_DISTANCE=0 start_extra_sensors &
        _track_pid "microdia" $!
    fi
}

# ── 完整并行启动流程 ──
do_parallel_startup() {
    # Phase 1: launch all independent components in parallel
    _parallel_phase1

    # Wait for Phase 1 minimum readiness
    sleep 3

    # Phase 2: components that need /scan + PX4 DDS
    _parallel_phase2

    # Wait for VSLAM odometry (the slowest step, blocks until ready)
    log "等待容器 VSLAM 就绪 (最多 60s)..."
    local slam_pid
    slam_pid=$(_read_pid "container_slam")
    if [ -n "$slam_pid" ] && kill -0 "$slam_pid" 2>/dev/null; then
        wait "$slam_pid" || {
            error "容器 VSLAM 启动失败 (exit=$?)，检查: docker exec $CONTAINER_NAME tail -100 $SLAM_LOG"
            return 1
        }
    else
        warn "container_slam 后台任务不在运行，可能已提前退出"
    fi

    # Phase 3: PX4 Bridge
    _parallel_phase3

    sleep 5

    # Phase 4: Microdia (delayed to avoid D435i UVC conflict)
    _parallel_phase4

    sleep 10
    return 0
}

# ═══════════════════════════════════════════
#  主流程
# ═══════════════════════════════════════════

# 方案/IMU 来源在进入任何子命令前归一化（stop/status 也用同一套名字）
# `stop` 是应急停机，必须在任何配置错误下都能执行，故跳过参数校验
if [ "${1:-}" != "stop" ]; then
    reject_legacy_vars
    resolve_slam_scheme
fi

case "${1:-}" in
    stop)
        stop_all
        exit 0
        ;;
    reset-agent)
        reset_agent
        exit 0
        ;;
    status)
        source /opt/ros/humble/setup.bash 2>/dev/null
        source "$ROS2_WS/install/setup.bash" 2>/dev/null
        check_status
        exit 0
        ;;
    --check-arm|--arm-check|armcheck)
        check_arm_readiness
        exit 0
        ;;
    --print-scheme|print-scheme)
        check_scheme_conflicts
        echo "SLAM_SCHEME=$SLAM_SCHEME"
        echo "IMU_SOURCE=$IMU_SOURCE"
        echo "RUN_MODE=$RUN_MODE"
        echo "USE_RTABMAP_RELAY=$USE_RTABMAP_RELAY"
        echo "USE_PX4_IMU_RELAY=$USE_PX4_IMU_RELAY"
        echo "LAUNCH=$SLAM_LAUNCH_FILE"
        echo "CONTAINER_LOG=$SLAM_LOG"
        exit 0
        ;;
    ground-only|--ground-only)
        acquire_start_lock
        GROUND_ONLY=1
        RUN_MODE="$GROUND_RUN_MODE"
        check_scheme_conflicts
        log "仅启动地面定位诊断 (SLAM_SCHEME=$SLAM_SCHEME IMU_SOURCE=$IMU_SOURCE RUN_MODE=$RUN_MODE)"
        check_prereqs
        ensure_stopped_before_start
        START_RPLIDAR=0
        START_MICRODIA=0
        START_OBSTACLE_DISTANCE=0
        if ! start_container_slam; then
            error "D435i/VSLAM 未就绪，地面诊断启动失败。"
            exit 1
        fi
        log "地面定位诊断已启动；查看: $0 status"
        log "停止: $0 stop"
        log "容器日志: docker exec $CONTAINER_NAME tail -f $SLAM_LOG"
        exit 0
        ;;
    --bg|-b)
        acquire_start_lock
        log "以后台模式启动 (并行)..."
        check_scheme_conflicts
        check_prereqs
        ensure_stopped_before_start
        [ "${GROUND_ONLY:-0}" != "1" ] && ensure_px4_tfmini_height
        if ! do_parallel_startup; then
            error "并行启动失败"
            exit 1
        fi
        check_status
        log ""
        log "后台运行中。查看状态: $0 status"
        log "停止: $0 stop"
        log "宿主机日志: tail -f $LOG_DIR/px4_bridge.log"
        log "额外传感器: tail -f $LOG_DIR/sensors_extra.log"
        log "容器日志 ($SLAM_SCHEME): docker exec $CONTAINER_NAME tail -f $SLAM_LOG"
        release_start_lock
        # disown all tracked PIDs so they survive shell exit
        for pid_file in "$PID_DIR"/*.pid; do
            [ -f "$pid_file" ] && disown "$(cat "$pid_file")" 2>/dev/null || true
        done
        ok "启动完成"
        exit 0
        ;;
    *)
        acquire_start_lock
        log "以前台模式启动 (并行, CTRL+C 停止)..."
        check_scheme_conflicts
        check_prereqs
        ensure_stopped_before_start
        [ "${GROUND_ONLY:-0}" != "1" ] && ensure_px4_tfmini_height
        if ! do_parallel_startup; then
            error "并行启动失败。将执行 cleanup..."
            cleanup
        fi
        check_status
        log ""
        log "系统运行中，按 CTRL+C 停止..."
        release_start_lock
        # 不阻塞 wait（用循环 + sleep 代替，CTRL+C 触发 trap 清理）
        local watchdog_pid
        watchdog_pid=$(_read_pid "px4_bridge")
        if [ -n "$watchdog_pid" ] && kill -0 "$watchdog_pid" 2>/dev/null; then
            while kill -0 "$watchdog_pid" 2>/dev/null; do
                sleep 1
            done
        else
            sleep 5
        fi
        log "进程已退出"
        ;;
esac
