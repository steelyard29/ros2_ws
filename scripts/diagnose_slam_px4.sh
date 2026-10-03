#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════
# SLAM → PX4 全链路诊断工具
#
# 逐层检查数据链路:
#   Layer 0: UART 物理接线 (Jetson ↔ PX4 TELEM1)
#   Layer 1: D435i 相机 + Docker + Visual SLAM
#   Layer 2: DDS 跨容器发现 (Docker → 宿主机)
#   Layer 3: vslam_odom_bridge (ENU→NED 转换)
#   Layer 4: MicroXRCEAgent (串口桥接到 PX4)
#   Layer 5: PX4 uXRCE-DDS 参数
#   Layer 6: PX4 EKF2 外部视觉融合
#   Layer 7: PX4 飞前检查 + 解锁条件
#
# 用法:
#   ./diagnose_slam_px4.sh              # 基础检查 (硬件+环境, 无需系统运行)
#   ./diagnose_slam_px4.sh --serial     # 仅串口/接线检查 (含 loopback 指导)
#   ./diagnose_slam_px4.sh --runtime    # 运行时检查 (需要系统已启动)
#   ./diagnose_slam_px4.sh --all        # 完整诊断 (基础 + 运行时)
# ═══════════════════════════════════════════════════════════════════════

set -e

# ── 路径配置 ──
ROS2_WS="$HOME/ros2_ws"
ISAAC_WS="$ROS2_WS/scripts"
CONTAINER_NAME="isaac_ros_dev"
LOG_DIR="/tmp/slam_px4_logs"
PX4_SERIAL_DEV="/dev/ttyTHS1"
PX4_SERIAL_BAUD="921600"

# ── 颜色 ──
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m'

# ── 计数器 ──
PASS_COUNT=0
FAIL_COUNT=0
WARN_COUNT=0
SKIP_COUNT=0

# ── 辅助函数 ──
section() {
    echo ""
    echo -e "${BOLD}${BLUE}═══════════════════════════════════════════════════════${NC}"
    echo -e "${BOLD}${BLUE}  $1${NC}"
    echo -e "${BOLD}${BLUE}═══════════════════════════════════════════════════════${NC}"
    echo ""
}

pass() {
    echo -e "  ${GREEN}[✓ PASS]${NC} $1"
    PASS_COUNT=$((PASS_COUNT + 1))
}

fail() {
    echo -e "  ${RED}[✗ FAIL]${NC} $1"
    FAIL_COUNT=$((FAIL_COUNT + 1))
}

warn() {
    echo -e "  ${YELLOW}[! WARN]${NC} $1"
    WARN_COUNT=$((WARN_COUNT + 1))
}

info() {
    echo -e "  ${CYAN}[i]${NC} $1"
}

skip() {
    echo -e "  ${CYAN}[- SKIP]${NC} $1"
    SKIP_COUNT=$((SKIP_COUNT + 1))
}

check_cmd() {
    if ! command -v "$1" &>/dev/null; then
        fail "$2: 未安装 ($1)"
        return 1
    fi
    pass "$2: 已安装"
    return 0
}

check_device() {
    if [ -c "$1" ]; then
        pass "$1 设备存在"
        return 0
    else
        fail "$1 设备不存在"
        return 1
    fi
}

PX4_PARAM_SNAPSHOT="${PX4_PARAM_FILE:-$ROS2_WS/config/px4/recovered_params.txt}"

param_get_snapshot() {
    local key="$1"
    if [ ! -f "$PX4_PARAM_SNAPSHOT" ]; then
        return 1
    fi
    awk -F',' -v key="$key" '$1 == key {print $2; exit}' "$PX4_PARAM_SNAPSHOT"
}

compare_px4_param() {
    local key="$1"
    local expected="$2"
    local actual
    actual="$(param_get_snapshot "$key" || true)"
    if [ -z "$actual" ]; then
        warn "$key 未在 $PX4_PARAM_SNAPSHOT 中找到 (请用 QGC/param show 核对飞控实值)"
        return 1
    fi
    if [ "$actual" = "$expected" ]; then
        pass "$key=$actual (符合预期)"
        return 0
    fi
    fail "$key=$actual (期望 $expected) — 可能导致融合后水平漂移"
    return 1
}

topic_subscriber_count() {
    local topic="$1"
    # Humble prints "Subscription count: N" on the same line.
    timeout 3 ros2 topic info "$topic" -v 2>/dev/null \
        | awk '/Subscription count:/ {print $3; exit}'
}

# Source ROS2 environment safely
source_ros2() {
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    if [ -f "$ROS2_WS/install/setup.bash" ]; then
        source "$ROS2_WS/install/setup.bash" 2>/dev/null || true
    fi
}

# ═══════════════════════════════════════════════════════════════════════
# 接线图
# ═══════════════════════════════════════════════════════════════════════
print_wiring_diagram() {
    echo ""
    echo -e "${BOLD}╔══════════════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BOLD}║              UART 接线图: Jetson Orin Nano ↔ PX4 TELEM1                  ║${NC}"
    echo -e "${BOLD}╠══════════════════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${BOLD}║${NC}                                                                          ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}  Jetson 40-pin 排针                        PX4 TELEM1 (JST GH 4-pin)   ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}  ──────────────────                        ────────────────────────    ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}                                                                          ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}  Pin 6  (GND)     ─────── 黑线 ──────   Pin 4 (GND)                  ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}  Pin 8  (TXD)     ─────── 白线 ──────   Pin 2 (RX)   ${RED}★ TX→RX ★${NC}       ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}  Pin 10 (RXD)     ─────── 绿线 ──────   Pin 3 (TX)   ${RED}★ RX←TX ★${NC}       ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}                                                                          ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}  ${RED}★ 关键: UART 需要交叉连接!${NC}                                              ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}    Jetson TXD (Pin 8)  →  PX4 RX  (Pin 2)    ${GREEN}正确 ✓${NC}                  ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}    Jetson RXD (Pin 10) →  PX4 TX  (Pin 3)    ${GREEN}正确 ✓${NC}                  ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}                                                                          ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}  ${RED}常见接线错误:${NC}                                                            ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}    ❌ TX→TX, RX→RX (直连)    → 完全无通信                               ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}    ❌ 只接 TX/RX, 没接 GND   → 电平不匹配, 数据乱码                     ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}    ❌ 接到 Pin 3/5 (I2C1)    → 端口错误, 不是 UART                      ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}    ❌ 接到 Pin 11/12 (I2C2)  → 端口错误, 不是 UART                      ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}                                                                          ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}  软件配置: /dev/ttyTHS1 @ 921600 baud, 8N1                              ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}  PX4 参数: UXRCE_DDS_CFG=TELEM1, SER_TEL1_BAUD=921600                  ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}                                                                          ${BOLD}║${NC}"
    echo -e "${BOLD}╚══════════════════════════════════════════════════════════════════════════╝${NC}"
}

# ═══════════════════════════════════════════════════════════════════════
# Layer 0: UART 物理接线
# ═══════════════════════════════════════════════════════════════════════
layer0_uart_wiring() {
    section "Layer 0: UART 物理接线"

    # ── 0.1 串口设备存在性 ──
    echo "  ── 串口设备检查 ──"
    check_device "$PX4_SERIAL_DEV"

    # 权限检查
    if [ -c "$PX4_SERIAL_DEV" ]; then
        PERMS=$(stat -c "%a" "$PX4_SERIAL_DEV" 2>/dev/null || echo "???")
        if [ "$PERMS" = "666" ] || [ "$PERMS" = "rw-rw-rw-" ]; then
            pass "串口权限: $PERMS (读写正常)"
        else
            warn "串口权限: $PERMS — 可能需要: sudo chmod 666 $PX4_SERIAL_DEV"
        fi
    fi

    # ── 0.2 其他串口设备一览 ──
    echo ""
    echo "  ── 系统中所有串口 ──"
    for dev in /dev/ttyTHS* /dev/ttyUSB* /dev/ttyACM*; do
        if [ -c "$dev" ]; then
            OWNER=$(stat -c "%U:%G" "$dev" 2>/dev/null || echo "unknown")
            echo "    $dev  (owner: $OWNER)"
        fi
    done 2>/dev/null || true

    # ── 0.3 串口进程占用 ──
    echo ""
    echo "  ── 串口占用情况 ──"
    if fuser "$PX4_SERIAL_DEV" 2>/dev/null; then
        info "串口正被以下进程占用:"
        lsof "$PX4_SERIAL_DEV" 2>/dev/null | head -5 || true
    else
        pass "串口未被占用 (可正常使用)"
    fi

    # ── 0.4 stty 检查 ──
    echo ""
    if stty -F "$PX4_SERIAL_DEV" 921600 2>/dev/null; then
        pass "串口波特率 921600 可设置"
    else
        fail "串口波特率 921600 设置失败 — 检查串口是否已被占用"
    fi

    # ── 0.5 Loopback 指导 ──
    echo ""
    echo -e "  ${BOLD}── 串口 Loopback 测试 (验证硬件收发) ──${NC}"
    echo ""
    echo "    此测试验证 Jetson 端串口硬件是否正常。"
    echo "    需要: 断开 PX4 连接，用跳线短接 Pin 8(TXD) 和 Pin 10(RXD)"
    echo ""
    echo "    操作步骤:"
    echo "    ┌─────────────────────────────────────────────────────────┐"
    echo "    │ 1. 断开 PX4 TELEM1 的连接线                              │"
    echo "    │ 2. 用跳线/杜邦线短接 Jetson Pin 8 和 Pin 10             │"
    echo "    │ 3. 终端1: cat $PX4_SERIAL_DEV                            │"
    echo "    │ 4. 终端2: echo 'TEST123' > $PX4_SERIAL_DEV               │"
    echo "    │ 5. 如果终端1 显示 TEST123 → 串口硬件正常 ✓              │"
    echo "    │    如果终端1 无输出      → 串口硬件故障 ✗              │"
    echo "    │ 6. 测试完成后拆除跳线, 恢复 PX4 接线                     │"
    echo "    └─────────────────────────────────────────────────────────┘"
    echo ""

    # 尝试自动 loopback (仅在明确请求且 PX4 未连接的情况下)
    if [ "${1:-}" = "--loopback" ]; then
        echo "  ── 自动 Loopback 测试 ──"
        info "正在尝试自动 loopback..."

        # 先检查串口是否被占用
        if fuser "$PX4_SERIAL_DEV" 2>/dev/null; then
            fail "串口被占用，无法进行 loopback 测试。请先停止占用进程:"
            echo "    pkill -f MicroXRCEAgent"
            echo "    pkill -f vslam_odom_bridge"
            echo "    pkill -f px4_gateway_node"
            return 1
        fi

        LOOPBACK_RESULT=$(python3 -c "
import serial, time, sys
try:
    ser = serial.Serial('$PX4_SERIAL_DEV', $PX4_SERIAL_BAUD, timeout=3)
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    test_data = b'PX4DIAG_' + str(int(time.time() * 1000))[-8:].encode()
    ser.write(test_data)
    ser.flush()
    time.sleep(0.5)
    received = ser.read(len(test_data) + 16)
    ser.close()
    if received == test_data:
        print('LOOPBACK_OK')
    elif len(received) > 0:
        sys.stdout.buffer.write(b'GOT_DATA:')
        sys.stdout.buffer.write(received[:64])
    else:
        print('NO_ECHO')
except serial.SerialException as e:
    print(f'SERIAL_ERROR:{e}')
except Exception as e:
    print(f'ERROR:{e}')
" 2>&1)

        case "$LOOPBACK_RESULT" in
            LOOPBACK_OK)
                pass "UART Loopback 测试通过! 串口硬件收发正常"
                ;;
            GOT_DATA:*)
                local rx_data="${LOOPBACK_RESULT#GOT_DATA:}"
                warn "UART Loopback: 收到其他数据 (可能是 PX4 仍连接?) — 接收: $rx_data"
                info "  断开 PX4 并用跳线短接 Pin8→Pin10 后重试"
                ;;
            NO_ECHO)
                fail "UART Loopback 失败: 无数据回传"
                echo "    → Pin 8(TXD) 和 Pin 10(RXD) 是否已用跳线短接?"
                echo "    → UART 是否已在设备树启用? (sudo /opt/nvidia/jetson-io/jetson-io.py)"
                ;;
            SERIAL_ERROR:*)
                fail "串口错误: ${LOOPBACK_RESULT#SERIAL_ERROR:}"
                ;;
            ERROR:*)
                fail "未知错误: ${LOOPBACK_RESULT#ERROR:}"
                ;;
        esac
    else
        info "跳过自动 loopback 测试 (使用 --serial --loopback 强制执行)"
    fi
}

# ═══════════════════════════════════════════════════════════════════════
# Layer 1: D435i 相机 + Docker + Visual SLAM
# ═══════════════════════════════════════════════════════════════════════
layer1_camera_slam() {
    section "Layer 1: D435i 相机 + Docker + Visual SLAM"

    # ── 1.1 Docker 容器 ──
    echo "  ── Docker 容器状态 ──"
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -q "^${CONTAINER_NAME}$"; then
        pass "容器 '$CONTAINER_NAME' 运行中"
        CONTAINER_RUNNING=true
    else
        fail "容器 '$CONTAINER_NAME' 未运行"
        echo "    启动: cd $ISAAC_WS && ./run_isaac_dev.sh"
        echo "    或: docker start $CONTAINER_NAME"
        CONTAINER_RUNNING=false
    fi

    # ── 1.2 Docker 网络模式 ──
    if $CONTAINER_RUNNING; then
        NETMODE=$(docker inspect "$CONTAINER_NAME" --format '{{.HostConfig.NetworkMode}}' 2>/dev/null || echo "unknown")
        if [ "$NETMODE" = "host" ]; then
            pass "Docker 网络模式: host (DDS 发现正确)"
        else
            fail "Docker 网络模式: $NETMODE (应为 host! DDS 跨容器发现可能失败)"
            echo "    检查: docker inspect $CONTAINER_NAME | grep NetworkMode"
        fi
    fi

    # ── 1.3 D435i USB 检测 ──
    echo ""
    echo "  ── 相机检测 ──"
    HOST_USB=$(lsusb 2>/dev/null | grep -i "8086\|Intel\|RealSense" || true)
    if [ -n "$HOST_USB" ]; then
        pass "宿主机检测到 RealSense 相机:"
        echo "    $HOST_USB"
        CAMERA_DETECTED=true
    else
        fail "宿主机未检测到 RealSense 相机!"
        echo "    → 检查 USB 连接线 (需要数据线, 不能是充电线)"
        echo "    → 尝试其他 USB 端口 (推荐 USB 3.0)"
        echo "    → 运行: lsusb | grep -i intel"
        CAMERA_DETECTED=false
    fi

    # USB 端口速度检查
    if $CAMERA_DETECTED; then
        for dev in /sys/bus/usb/devices/*/speed; do
            SPEED=$(cat "$dev" 2>/dev/null || echo "0")
            if [ "$SPEED" = "5000" ] || [ "$SPEED" = "3" ]; then
                pass "相机 USB 速度: SuperSpeed (5Gbps) — 正常"
                break
            elif [ "$SPEED" = "480" ] || [ "$SPEED" = "2" ]; then
                warn "相机 USB 速度: HighSpeed (480Mbps) — USB 2.x, IMU 带宽受限"
                echo "    → 推荐使用 USB 3.0 端口以获得完整 IMU 数据"
                echo "    → 电机振动下 VIO yaw 噪声会明显增大；确认相机刚性安装/减振"
                break
            fi
        done
        info "VIO yaw 可靠化: USB3 + enable_imu_fusion + 刚性/减振安装 + 充足纹理"
    fi

    # ── 1.4 ROS2 topic 检查 (需要宿主机 ROS2) ──
    source_ros2

    echo ""
    echo "  ── SLAM topic 检查 (宿主机视角, 需要 DDS 跨容器) ──"

    HOST_TOPICS=$(timeout 5 ros2 topic list 2>/dev/null || echo "")

    # 相机 topics
    if echo "$HOST_TOPICS" | grep -q "/camera/infra1/image_rect_raw"; then
        pass "相机 infra1 图像 topic 可见"
    else
        warn "相机 infra1 图像 topic 不可见 (可能 SLAM 未启动或 DDS 发现失败)"
    fi

    if echo "$HOST_TOPICS" | grep -q "/camera/infra2/image_rect_raw"; then
        pass "相机 infra2 图像 topic 可见"
    else
        warn "相机 infra2 图像 topic 不可见"
    fi

    # IMU topic + VSLAM 是否真正订阅
    local imu_topic="/camera/camera/imu"
    if echo "$HOST_TOPICS" | grep -q "$imu_topic"; then
        pass "IMU topic 可见: $imu_topic"
        local imu_subs
        imu_subs="$(topic_subscriber_count "$imu_topic" || echo "0")"
        if [ -n "$imu_subs" ] && [ "$imu_subs" -gt 0 ] 2>/dev/null; then
            pass "IMU 已被 VSLAM 订阅 (subscriber=$imu_subs)"
        else
            fail "IMU 无订阅者 — VSLAM 实际未融合 IMU (常见原因: enable_imu_fusion=false)"
            echo "    → 修复: launch/vslam_cuvslam.launch.py 用 imu_source:=d435|px4（或 vslam_rtabmap.launch.py 同参数）"
            echo "    → 重启 SLAM 后应看到 /visual_slam_node 订阅 $imu_topic"
        fi
        if ros2 param list /visual_slam_node 2>/dev/null | grep -q enable_imu_fusion; then
            local imu_fusion
            imu_fusion=$(timeout 3 ros2 param get /visual_slam_node enable_imu_fusion 2>/dev/null \
                | grep -oP '(?<=Boolean value is: ).*' || echo "")
            if [ "$imu_fusion" = "true" ]; then
                pass "visual_slam enable_imu_fusion=true"
            elif [ "$imu_fusion" = "false" ]; then
                fail "visual_slam enable_imu_fusion=false — 纯视觉模式，融合后易漂移"
            fi
        fi
    else
        warn "IMU topic 不可见 (VSLAM 无法做 VIO, 融合后易漂移)"
    fi

    # ── 1.5 VSLAM 里程计 ──
    echo ""
    echo "  ── VSLAM 里程计 /visual_slam/tracking/odometry ──"

    if echo "$HOST_TOPICS" | grep -q "/visual_slam/tracking/odometry"; then
        pass "VSLAM 里程计 topic 已发布"
        VSLAM_TOPIC_UP=true
    else
        fail "VSLAM 里程计 topic 未发布!"
        echo "    → 检查 VSLAM 是否已启动: docker exec $CONTAINER_NAME ros2 node list"
        echo "    → 检查 SLAM 日志: docker exec $CONTAINER_NAME tail -100 /tmp/slam_rtabmap.log"
        echo "    → 可能需要运行: cd $ISAAC_WS && ./run_slam_px4.sh --bg"
        VSLAM_TOPIC_UP=false
    fi

    # ── 1.6 里程计数据频率 ──
    if $VSLAM_TOPIC_UP; then
        echo ""
        local odom_hz
        odom_hz=$(timeout 5 ros2 topic hz /visual_slam/tracking/odometry 2>/dev/null \
            | grep "average rate" | grep -oP '[\d.]+' | head -1 || echo "0")

        if [ -n "$odom_hz" ] && [ "$odom_hz" != "0" ] && [ "${odom_hz%.*}" -gt 0 ] 2>/dev/null; then
            pass "VSLAM 里程计频率: ${odom_hz} Hz (期望 ~30Hz)"
        else
            fail "VSLAM 里程计频率: ${odom_hz:-0} Hz — 无数据!"
            echo "    → 可能原因:"
            echo "      ① VSLAM 未初始化 (对着纯色/无纹理表面)"
            echo "      ② 相机遮挡或朝向不良"
            echo "      ③ 光线太暗"
            echo "      ④ 相机未校准"
            echo "    → 解决: 将相机对着有丰富纹理的表面, 缓慢移动"
        fi

        # ── 1.7 里程计数据内容 ──
        echo ""
        local odom_data
        odom_data=$(timeout 3 ros2 topic echo /visual_slam/tracking/odometry --once 2>/dev/null | head -20 || echo "")
        if [ -n "$odom_data" ]; then
            local pos_x=$(echo "$odom_data" | grep "x:" | head -1 | grep -oP '[-]?\d+\.\d+' | head -1 || echo "0")
            local pos_y=$(echo "$odom_data" | grep "y:" | head -1 | grep -oP '[-]?\d+\.\d+' | head -1 || echo "0")
            local pos_z=$(echo "$odom_data" | grep "z:" | head -1 | grep -oP '[-]?\d+\.\d+' | head -1 || echo "0")

            if [ "$pos_x" != "0.0" ] || [ "$pos_y" != "0.0" ]; then
                pass "里程计数据有效: pos=($pos_x, $pos_y, $pos_z)"
            elif [ "$pos_x" = "0.0" ] && [ "$pos_y" = "0.0" ] && [ "$pos_z" = "0.0" ]; then
                warn "里程计位置全零 — VSLAM 未完成初始化"
                echo "    → 将相机对着有纹理的表面缓慢移动, 直到位置变为非零"
            else
                pass "里程计有数据输出"
            fi
        fi
    fi
}

# ═══════════════════════════════════════════════════════════════════════
# Layer 2: DDS 跨容器发现
# ═══════════════════════════════════════════════════════════════════════
layer2_dds_discovery() {
    section "Layer 2: DDS 跨容器发现 (Docker → 宿主机)"

    echo "  VSLAM 运行在 Docker 容器内, 宿主机必须能通过 DDS 发现其 topics。"
    echo "  Docker 使用 --network host 模式, DDS 基于 UDP multicast 通信。"
    echo ""

    # ── 2.1 FastDDS 配置 ──
    local fastdds_xml="$ROS2_WS/config/fastdds_bridge.xml"
    if [ -f "$fastdds_xml" ]; then
        pass "FastDDS 配置文件存在: $fastdds_xml"
        if grep -q "UDPv4" "$fastdds_xml" 2>/dev/null; then
            pass "FastDDS 使用 UDPv4 传输"
        fi
    else
        warn "FastDDS 配置文件不存在: $fastdds_xml"
        info "  (launch 文件会自动设置 FASTRTPS_DEFAULT_PROFILES_FILE)"
    fi

    # ── 2.2 环境变量 ──
    if [ -n "${RMW_IMPLEMENTATION:-}" ]; then
        if [ "$RMW_IMPLEMENTATION" = "rmw_fastrtps_cpp" ]; then
            pass "RMW_IMPLEMENTATION = rmw_fastrtps_cpp"
        else
            warn "RMW_IMPLEMENTATION = $RMW_IMPLEMENTATION (推荐: rmw_fastrtps_cpp)"
        fi
    else
        info "RMW_IMPLEMENTATION 未设置 (launch 脚本会自动设置)"
    fi

    if [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
        pass "FASTRTPS_DEFAULT_PROFILES_FILE 已设置"
    else
        info "FASTRTPS_DEFAULT_PROFILES_FILE 未设置 (launch 脚本会自动设置)"
    fi

    # ── 2.3 宿主机是否能看到容器 topics ──
    HOST_TOPICS=$(timeout 5 ros2 topic list 2>/dev/null || echo "")
    if echo "$HOST_TOPICS" | grep -q "/visual_slam/tracking/odometry"; then
        pass "宿主机 ROS2 可以发现容器内的 VSLAM topic — DDS 发现正常"
    else
        fail "宿主机 ROS2 无法发现容器内的 VSLAM topic!"
        echo ""
        echo "  ┌── DDS 发现故障排查 ─────────────────────────────────────┐"
        echo "  │ 1. 确认容器网络模式:                                      │"
        echo "  │    docker inspect $CONTAINER_NAME | grep NetworkMode     │"
        echo "  │                                                           │"
        echo "  │ 2. 确认容器内 ROS2 环境已加载:                            │"
        echo "  │    docker exec $CONTAINER_NAME bash -c \\                 │"
        echo "  │      'source /opt/ros/humble/setup.bash && ros2 topic list' │"
        echo "  │                                                           │"
        echo "  │ 3. 临时尝试:                                              │"
        echo "  │    export ROS_DOMAIN_ID=0 (宿主机和容器内都设)            │"
        echo "  │                                                           │"
        echo "  │ 4. 检查是否有防火墙阻止 UDP 7400-7500:                    │"
        echo "  │    sudo iptables -L -n | grep 7400                        │"
        echo "  │                                                           │"
        echo "  │ 5. 检查 DDS 发现流量:                                     │"
        echo "  │    sudo tcpdump -i lo udp port 7400 -c 10                 │"
        echo "  └───────────────────────────────────────────────────────────┘"
    fi

    # ── 2.4 ROS_DOMAIN_ID ──
    if [ -n "${ROS_DOMAIN_ID:-}" ]; then
        info "ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
    else
        info "ROS_DOMAIN_ID 未设置 (使用默认值 0)"
    fi
}

# ═══════════════════════════════════════════════════════════════════════
# Layer 3: vslam_odom_bridge
# ═══════════════════════════════════════════════════════════════════════
layer3_odom_bridge() {
    section "Layer 3: vslam_odom_bridge (ENU→NED 转换)"

    # ── 3.1 节点检查 ──
    NODE_LIST=$(timeout 5 ros2 node list 2>/dev/null || echo "")

    if echo "$NODE_LIST" | grep -q "vslam_odom_bridge"; then
        pass "vslam_odom_bridge 节点运行中"
        BRIDGE_RUNNING=true
    else
        fail "vslam_odom_bridge 节点未运行"
        echo "    → 启动: ros2 launch px4_interface vslam_px4.launch.py"
        echo "    → 或运行: cd $ISAAC_WS && ./run_slam_px4.sh --bg"
        BRIDGE_RUNNING=false
    fi

    if echo "$NODE_LIST" | grep -q "px4_gateway_node"; then
        pass "px4_gateway_node 节点运行中"
    else
        fail "px4_gateway_node 节点未运行"
    fi

    # ── 3.2 Bridge 输出 topic ──
    HOST_TOPICS=$(timeout 5 ros2 topic list 2>/dev/null || echo "")

    echo ""
    echo "  ── Bridge 数据流 ──"

    if echo "$HOST_TOPICS" | grep -q "/fmu/in/vehicle_visual_odometry"; then
        pass "Bridge 输出 topic /fmu/in/vehicle_visual_odometry 已创建"

        # 检查频率
        local bridge_hz
        bridge_hz=$(timeout 5 ros2 topic hz /fmu/in/vehicle_visual_odometry 2>/dev/null \
            | grep "average rate" | grep -oP '[\d.]+' | head -1 || echo "0")

        if [ -n "$bridge_hz" ] && [ "$bridge_hz" != "0" ] && [ "${bridge_hz%.*}" -gt 0 ] 2>/dev/null; then
            pass "Bridge 输出频率: ${bridge_hz} Hz (应和 VSLAM 同频 ~30Hz)"
        else
            fail "Bridge 输出频率: ${bridge_hz:-0} Hz — 无数据流过 bridge!"
            echo "    → VSLAM 里程计有数据吗? (检查 Layer 1)"
            echo "    → 检查 bridge 订阅: ros2 topic info /visual_slam/tracking/odometry"
            echo "    → 检查 bridge 日志: tail -100 $LOG_DIR/px4_bridge.log | grep -i vslam"
        fi

        # ── 3.3 坐标转换验证 ──
        echo ""
        echo "  ── 坐标转换验证 (ENU → NED) ──"
        local px4_odom
        px4_odom=$(timeout 3 ros2 topic echo /fmu/in/vehicle_visual_odometry --once 2>/dev/null | head -30 || echo "")

        if [ -n "$px4_odom" ]; then
            # 读取 VSLAM 数据做对比
            local slam_odom
            slam_odom=$(timeout 3 ros2 topic echo /visual_slam/tracking/odometry --once 2>/dev/null | head -20 || echo "")

            local px4_pos_x=$(echo "$px4_odom" | grep "position:" -A5 | grep "x:" | grep -oP '[-]?\d+\.?\d*' | head -1 || echo "0")
            local px4_quality=$(echo "$px4_odom" | grep "quality:" | grep -oP '\d+' | head -1 || echo "0")
            local px4_pose_frame=$(echo "$px4_odom" | grep "pose_frame:" | grep -oP '\d+' | head -1 || echo "?")
            local px4_vel_frame=$(echo "$px4_odom" | grep "velocity_frame:" | grep -oP '\d+' | head -1 || echo "?")

            if [ -n "$px4_pos_x" ] && [ "$px4_pos_x" != "0" ]; then
                pass "PX4 VehicleOdometry position 有数据: x=$px4_pos_x"
            else
                warn "PX4 VehicleOdometry position 全零 — 可能 VSLAM 未初始化"
            fi

            if [ "$px4_quality" = "100" ]; then
                pass "quality = 100 (bridge 设置)"
            else
                warn "quality = $px4_quality (期望 100)"
            fi

            # px4_msgs VehicleOdometry: POSE_FRAME_NED = 1
            if [ "$px4_pose_frame" = "1" ]; then
                pass "pose_frame = NED (=1) — 正确"
            else
                warn "pose_frame = $px4_pose_frame (期望 1 = POSE_FRAME_NED)"
            fi

            # px4_msgs VehicleOdometry: VELOCITY_FRAME_BODY_FRD = 3
            if [ "$px4_vel_frame" = "3" ]; then
                pass "velocity_frame = BODY_FRD (=3) — 正确"
            else
                warn "velocity_frame = $px4_vel_frame (期望 3 = VELOCITY_FRAME_BODY_FRD)"
            fi
        else
            warn "无法读取 /fmu/in/vehicle_visual_odometry 数据 (请等待系统启动完成)"
        fi
    else
        fail "Bridge 输出 topic /fmu/in/vehicle_visual_odometry 未创建"
        echo "    → vslam_odom_bridge 节点可能启动失败"
        echo "    → 检查日志: tail -100 $LOG_DIR/px4_bridge.log"
    fi
}

# ═══════════════════════════════════════════════════════════════════════
# Layer 4: MicroXRCEAgent (串口通信)
# ═══════════════════════════════════════════════════════════════════════
layer4_microxrce_agent() {
    section "Layer 4: MicroXRCEAgent (串口 → PX4 uXRCE-DDS)"

    # ── 4.1 Agent 进程 ──
    AGENT_PID=$(pgrep -f "MicroXRCEAgent" 2>/dev/null || true)
    if [ -n "$AGENT_PID" ]; then
        PASS_AGENT=true
        AGENT_CMD=$(ps -p "$AGENT_PID" -o cmd= 2>/dev/null | head -1 || echo "unknown")
        pass "MicroXRCEAgent 运行中 (PID: $AGENT_PID)"
        echo "    命令: $AGENT_CMD"

        # 验证串口参数
        if echo "$AGENT_CMD" | grep -q "/dev/ttyTHS1"; then
            pass "使用正确串口: /dev/ttyTHS1"
        else
            fail "Agent 未使用 /dev/ttyTHS1! 命令: $AGENT_CMD"
        fi

        if echo "$AGENT_CMD" | grep -q "921600"; then
            pass "使用正确波特率: 921600"
        else
            fail "Agent 未使用 921600 波特率"
        fi
    else
        fail "MicroXRCEAgent 未运行"
        echo "    → 启动: MicroXRCEAgent serial -D $PX4_SERIAL_DEV -b $PX4_SERIAL_BAUD"
        echo "    → 或运行: cd $ISAAC_WS && ./run_slam_px4.sh --bg"
        PASS_AGENT=false
    fi

    # ── 4.2 PX4 上行 topics (最关键检查!) ──
    echo ""
    echo "  ── PX4 → Jetson 数据 (uXRCE-DDS 双向验证) ──"
    echo "  ★ 这是最关键的检查! 如果能看到 /fmu/out/* topics, 说明串口双向通信正常!"
    echo ""

    HOST_TOPICS=$(timeout 5 ros2 topic list 2>/dev/null || echo "")

    # /fmu/out/vehicle_status_v1 — PX4 状态
    if echo "$HOST_TOPICS" | grep -q "/fmu/out/vehicle_status_v1"; then
        pass "/fmu/out/vehicle_status_v1 在线 — 串口双向通信正常! TELEM1 接线正确!"
        PX4_COMM_OK=true
    else
        fail "/fmu/out/vehicle_status_v1 不在线 — PX4 无数据返回!"
        echo ""
        echo "  ┌── PX4 串口通信故障排查 ─────────────────────────────────┐"
        echo "  │                                                          │"
        echo "  │ ① TX/RX 接线是否交叉?                                    │"
        echo "  │   Jetson Pin 8(TXD) → PX4 TELEM1 RX (Pin 2)             │"
        echo "  │   Jetson Pin 10(RXD) → PX4 TELEM1 TX (Pin 3)            │"
        echo "  │   如果 Pin8→Pin3, Pin10→Pin2 = ${RED}接线错误!${NC}               │"
        echo "  │                                                          │"
        echo "  │ ② PX4 参数是否正确?                                      │"
        echo "  │   UXRCE_DDS_CFG = TELEM1                                │"
        echo "  │   SER_TEL1_BAUD = 921600                                │"
        echo "  │                                                          │"
        echo "  │ ③ PX4 固件是否支持 uXRCE-DDS?                            │"
        echo "  │   PX4 v1.13+ 默认包含 uXRCE-DDS 客户端                  │"
        echo "  │   老旧固件可能需要重新编译: make px4_fmu-v5_rtps          │"
        echo "  │                                                          │"
        echo "  │ ④ PX4 是否已上电? LED 指示正常?                          │"
        echo "  │                                                          │"
        echo "  │ ⑤ Linux 串口驱动: 检查 /dev/ttyTHS1 是否正常             │"
        echo "  │   dmesg | grep ttyTHS1                                  │"
        echo "  └──────────────────────────────────────────────────────────┘"
        PX4_COMM_OK=false
    fi

    # 其他 PX4 topics
    if echo "$HOST_TOPICS" | grep -q "/fmu/out/vehicle_odometry"; then
        pass "/fmu/out/vehicle_odometry 在线 (PX4 EKF 里程计输出)"
    else
        warn "/fmu/out/vehicle_odometry 不在线"
    fi

    if echo "$HOST_TOPICS" | grep -q "/fmu/out/vehicle_local_position"; then
        pass "/fmu/out/vehicle_local_position 在线 (本地位置)"
    else
        warn "/fmu/out/vehicle_local_position 不在线"
    fi

    if echo "$HOST_TOPICS" | grep -q "/fmu/out/battery_status"; then
        pass "/fmu/out/battery_status 在线 (电池状态)"
    else
        warn "/fmu/out/battery_status 不在线"
    fi

    # ── 4.3 PX4 状态数据内容 ──
    if $PX4_COMM_OK; then
        echo ""
        echo "  ── PX4 实时状态 ──"
        local vs_status
        vs_status=$(timeout 3 ros2 topic echo /fmu/out/vehicle_status_v1 --once 2>/dev/null | head -20 || echo "")

        if [ -n "$vs_status" ]; then
            local arming=$(echo "$vs_status" | grep "arming_state:" | grep -oP '\d+' | head -1 || echo "?")
            local nav=$(echo "$vs_status" | grep "nav_state:" | grep -oP '\d+' | head -1 || echo "?")
            local preflight=$(echo "$vs_status" | grep "pre_flight_checks_pass:" | grep -oP 'true|false' | head -1 || echo "?")
            local failsafe=$(echo "$vs_status" | grep "failsafe:" | grep -oP 'true|false' | head -1 || echo "?")

            echo ""
            echo "  ┌──────────────────────────────────────────────────┐"
            echo "  │           PX4 飞控实时状态                        │"
            echo "  ├──────────────────────────────────────────────────┤"

            local arm_str="DISARMED"
            [ "$arming" = "2" ] && arm_str="${GREEN}ARMED${NC}"
            [ "$arming" = "1" ] && arm_str="DISARMED"
            printf "  │  arming_state            = %-18s │\n" "$arm_str ($arming)"

            printf "  │  nav_state               = %-18s │\n" "$nav"

            local pf_str="${RED}✗ 未通过${NC}"
            [ "$preflight" = "true" ] && pf_str="${GREEN}✓ 通过${NC}"
            printf "  │  pre_flight_checks_pass  = %-18s │\n" "$pf_str ($preflight)"

            local fs_str="${GREEN}正常${NC}"
            [ "$failsafe" = "true" ] && fs_str="${RED}触发!!${NC}"
            printf "  │  failsafe                = %-18s │\n" "$fs_str ($failsafe)"

            echo "  └──────────────────────────────────────────────────┘"
            echo ""
        fi
    fi
}

# ═══════════════════════════════════════════════════════════════════════
# Layer 5: PX4 参数检查
# ═══════════════════════════════════════════════════════════════════════
layer5_px4_params() {
    section "Layer 5: PX4 参数检查 (uXRCE-DDS + EKF2 外部视觉)"

    if ! timeout 3 ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_status_v1"; then
        warn "无法读取 PX4 参数 — uXRCE-DDS 通信未建立 (先修复 Layer 4)"
        echo ""
    fi

    echo "  ┌── PX4 必需参数列表 (通过 QGC 或 PX4 串口终端验证) ───────────┐"
    echo "  │                                                              │"
    echo "  │  ${CYAN}参数                    值           说明${NC}                    │"
    echo "  │  ─────────────────────────────────────────────────────────   │"
    echo "  │  UXRCE_DDS_CFG          TELEM1        uXRCE-DDS 串口映射    │"
    echo "  │  SER_TEL1_BAUD          921600        串口波特率            │"
    echo "  │  EKF2_EV_CTRL           1             仅水平位置 (验收后可 9) │"
    echo "  │  EKF2_EV_QMIN           50            拒绝失效视觉数据       │"
    echo "  │  EKF2_HGT_REF           2             高度参考使用 TFmini   │"
    echo "  │  EKF2_BARO_CTRL         0             室内关气压            │"
    echo "  │  EKF2_MAG_TYPE          5             室内关磁力计          │"
    echo "  │  EKF2_GPS_CTRL          0             室内无 GPS             │"
    echo "  │  EKF2_RNG_CTRL          2             始终融合向下测距       │"
    echo "  │  EKF2_RNG_POS_Z         0.05          TFmini 位于CG下方5cm  │"
    echo "  │  EKF2_EVA_NOISE         0.25          姿态噪声 (预留开 yaw) │"
    echo "  │  SENS_TFMINI_CFG        102           TFmini 使用 TELEM2    │"
    echo "  │  EKF2_EV_DELAY          0             视觉延迟 (0=自动)     │"
    echo "  │  ${RED}COM_POWER_COUNT         1             ★ 关键! USB不接时=1${NC}    │"
    echo "  │  CBRK_IO_SAFETY         22027         旁路安全开关          │"
    echo "  │  COM_RC_IN_MODE         2             遥控器 + MAVLink兼容   │"
    echo "  │  COM_ARM_WO_GPS         1             允许无GPS解锁         │"
    echo "  │                                                              │"
    echo "  │  ${BOLD}验证方法:${NC}                                                │"
    echo "  │    QGC → Parameters → 搜索上述参数名                         │"
    echo "  │    或 PX4 串口终端: param show <参数名>                      │"
    echo "  │                                                              │"
    echo "  └──────────────────────────────────────────────────────────────┘"
    echo ""

    # ── 与 repo 预期对比 (recovered_params 快照，需定期从 QGC 导出更新) ──
    if [ -f "$PX4_PARAM_SNAPSHOT" ]; then
        echo "  ── PX4 参数快照对比 ($PX4_PARAM_SNAPSHOT) ──"
        warn "这是磁盘快照，不是飞控实时参数；请以 QGC / param show 为准"
        echo ""
        compare_px4_param "EKF2_EV_CTRL" "1"
        compare_px4_param "EKF2_EV_POS_X" "0"
        compare_px4_param "EKF2_EV_POS_Y" "0"
        compare_px4_param "EKF2_EV_POS_Z" "0"
        compare_px4_param "EKF2_EV_DELAY" "0"
        compare_px4_param "EKF2_EV_QMIN" "50"
        compare_px4_param "EKF2_HGT_REF" "2"
        compare_px4_param "EKF2_RNG_CTRL" "2"
        compare_px4_param "EKF2_BARO_CTRL" "0"
        compare_px4_param "EKF2_MAG_TYPE" "5"
        compare_px4_param "EKF2_RNG_POS_Z" "0.05"
        compare_px4_param "EKF2_OF_CTRL" "0"
        echo ""
        info "若飞控实时值仍 FAIL: 写入 SD 卡 config.txt 并重启，或 USB MAVLink 运行 configure_px4_dds.py"
        echo ""
    else
        warn "未找到 PX4 参数快照: $PX4_PARAM_SNAPSHOT"
        info "请从 QGC 导出参数到该文件，便于诊断 drift 相关配置"
        echo ""
    fi

    # ── 尝试通过 uXRCE-DDS 参数服务读取 (如果可用) ──
    if $PX4_COMM_OK; then
        local param_count
        param_count=$(timeout 5 ros2 param list /px4_gateway_node 2>/dev/null | wc -l || echo "0")
        if [ "$param_count" -gt 0 ]; then
            pass "可通过 ROS2 param 服务访问 PX4 参数 (发现 $param_count 个参数)"
        else
            info "无法通过 ROS2 param 服务读取 PX4 参数, 请使用 QGC 手动验证"
        fi
    fi
}

# ═══════════════════════════════════════════════════════════════════════
# Layer 6: PX4 EKF2 外部视觉融合
# ═══════════════════════════════════════════════════════════════════════
layer6_ekf2_fusion() {
    section "Layer 6: PX4 EKF2 外部视觉融合"

    if ! timeout 3 ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_status_v1"; then
        warn "uXRCE-DDS 通信未建立, 无法检查 EKF2 融合状态"
        return
    fi

    # ── 6.1 输入输出频率对比 ──
    echo "  ── EKF2 数据流 ──"
    echo ""

    local in_rate out_rate
    in_rate=$(timeout 5 ros2 topic hz /fmu/in/vehicle_visual_odometry 2>/dev/null \
        | grep "average rate" | grep -oP '[\d.]+' | head -1 || echo "0")
    out_rate=$(timeout 5 ros2 topic hz /fmu/out/vehicle_odometry 2>/dev/null \
        | grep "average rate" | grep -oP '[\d.]+' | head -1 || echo "0")

    echo "  视觉里程计输入 (/fmu/in):  ${in_rate:-0} Hz"
    echo "  EKF 里程计输出 (/fmu/out): ${out_rate:-0} Hz"

    if [ "${in_rate%.*}" -gt 0 ] 2>/dev/null; then
        pass "视觉里程计正在发送到 PX4"
    else
        fail "视觉里程计未发送到 PX4 (检查 Layer 1-3)"
    fi

    if [ "${out_rate%.*}" -gt 0 ] 2>/dev/null; then
        pass "PX4 EKF 正在输出里程计 (EKF2 运行中)"
    else
        warn "PX4 EKF 未输出里程计 (可能 EKF 未初始化或视觉数据被拒绝)"
    fi

    # ── 6.2 检查 estimator_status (是否有 ev 标志) ──
    echo ""
    if timeout 3 ros2 topic list 2>/dev/null | grep -q "/fmu/out/estimator_status"; then
        pass "/fmu/out/estimator_status 可用"

        local est_status
        est_status=$(timeout 3 ros2 topic echo /fmu/out/estimator_status --once 2>/dev/null | head -30 || echo "")

        local ctrl_flags
        ctrl_flags=$(echo "$est_status" | grep "control_mode_flags:" | grep -oP '\d+' | head -1 || echo "0")

        echo "  estimator_status.control_mode_flags = $ctrl_flags (raw)"
        echo ""
        if [ "$ctrl_flags" != "0" ] 2>/dev/null; then
            # Bit check: bit2=EV horiz, bit3=EV vert, bit4=EV yaw
            local ev_horiz=$(( (ctrl_flags >> 2) & 1 ))
            local ev_vert=$(( (ctrl_flags >> 3) & 1 ))
            local ev_yaw=$(( (ctrl_flags >> 4) & 1 ))

            echo "  EKF2 视觉融合状态:"
            [ "$ev_horiz" = "1" ] && pass "  EV 水平位置融合: 已启用" || warn "  EV 水平位置融合: 未启用"
            [ "$ev_vert" = "1" ] && warn "  EV 垂直位置融合: 已启用 (室内激光定高应关闭 bit1)" || pass "  EV 垂直位置融合: 未启用 (激光定高)"
            [ "$ev_yaw" = "1" ] && pass "  EV 偏航角融合: 已启用 (需先通过 verify_ev_yaw_align.sh)" \
                || info "  EV 偏航角融合: 未启用 (默认 EV_CTRL=1；验收后可开 9)"

            if [ "$ev_horiz" = "0" ] && [ "$ev_vert" = "0" ] && [ "$ev_yaw" = "0" ]; then
                fail "EKF2 未融合任何外部视觉数据!"
                echo "    → 检查 PX4 参数: EKF2_EV_CTRL = 1 (仅水平位置) 或验收后 9"
                echo "    → 视觉里程计数据可能被EKF拒绝 (检查时间戳/协方差/quality)"
                echo "    → 用 QGC 查看: MAVLink Inspector → VEHICLE_VISUAL_ODOMETRY"
            fi
        else
            warn "control_mode_flags = 0 — EKF2 可能未启用外部视觉"
            echo "    → 确认 PX4 参数 EKF2_EV_CTRL = 1 (或验收后 9)"
            echo "    → 该参数修改后需${RED}重启飞控${NC}生效!"
        fi
    else
        warn "/fmu/out/estimator_status 不可用"
        echo "    需要在 PX4 固件中启用 EKF2 日志输出"
    fi
}

# ═══════════════════════════════════════════════════════════════════════
# Layer 7: 飞前检查 + 解锁条件
# ═══════════════════════════════════════════════════════════════════════
layer7_arm_readiness() {
    section "Layer 7: PX4 飞前检查 + 解锁条件"

    if ! timeout 3 ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_status_v1"; then
        fail "无法读取飞前检查状态 — uXRCE-DDS 通信未建立"
        return
    fi

    local vs_status
    vs_status=$(timeout 3 ros2 topic echo /fmu/out/vehicle_status_v1 --once 2>/dev/null || echo "")

    local arming=$(echo "$vs_status" | grep "arming_state:" | grep -oP '\d+' | head -1 || echo "?")
    local preflight=$(echo "$vs_status" | grep "pre_flight_checks_pass:" | grep -oP 'true|false' | head -1 || echo "?")
    local failsafe=$(echo "$vs_status" | grep "failsafe:" | grep -oP 'true|false' | head -1 || echo "?")

    ALL_ARM_OK=true

    if [ "$preflight" = "true" ]; then
        pass "pre_flight_checks_pass = TRUE — 飞前检查通过!"
    else
        fail "pre_flight_checks_pass = FALSE — 飞控${RED}无法解锁!${NC}"
        ALL_ARM_OK=false
    fi

    if [ "$failsafe" = "true" ]; then
        fail "failsafe = TRUE — 飞控处于故障保护状态!"
        ALL_ARM_OK=false
    else
        pass "failsafe = FALSE — 无故障保护"
    fi

    if [ "$arming" = "2" ]; then
        pass "飞控已解锁 (ARMED)"
    elif [ "$arming" = "1" ]; then
        info "飞控已锁定 (DISARMED) — 满足条件后可解锁"
    else
        warn "arming_state = $arming"
    fi

    echo ""

    if ! $ALL_ARM_OK; then
        echo "  ┌── 飞前检查失败常见原因 ──────────────────────────────────┐"
        echo "  │                                                          │"
        echo "  │  ① ${RED}COM_POWER_COUNT ≠ 1${NC} — 没接USB时PX4认为电源异常       │"
        echo "  │     解决: 设 COM_POWER_COUNT=1                           │"
        echo "  │                                                          │"
        echo "  │  ② 传感器未校准 (加速度计/陀螺仪/罗盘)                    │"
        echo "  │     解决: QGC → Sensors → Calibrate                      │"
        echo "  │                                                          │"
        echo "  │  ③ 安全开关未旁路                                        │"
        echo "  │     解决: 设 CBRK_IO_SAFETY=22027                        │"
        echo "  │                                                          │"
        echo "  │  ④ 电池电压过低                                          │"
        echo "  │     解决: 充电或更换电池                                  │"
        echo "  │                                                          │"
        echo "  │  ⑤ GPS 丢失 (且 COM_ARM_WO_GPS≠1)                        │"
        echo "  │     解决: 设 COM_ARM_WO_GPS=1                            │"
        echo "  │                                                          │"
        echo "  │  ${BOLD}用 QGC 查看详细报错:${NC}                                  │"
        echo "  │    QGC → Vehicle Setup → Summary → 查看具体失败项        │"
        echo "  └──────────────────────────────────────────────────────────┘"

        echo ""
        echo "  ┌── 飞前检查通过的解锁条件 ───────────────────────────────┐"
        echo "  │                                                          │"
        echo "  │  COM_POWER_COUNT = 1      (不插USB能解锁, 红灯变绿)     │"
        echo "  │  CBRK_IO_SAFETY = 22027   (旁路安全开关)                │"
        echo "  │  COM_ARM_WO_GPS = 1       (允许无GPS解锁)               │"
        echo "  │  COM_RC_IN_MODE = 2       (遥控器 + MAVLink兼容)         │"
        echo "  │  EKF2_EV_CTRL = 1         (仅视觉水平位置; 验收后可 9) │"
        echo "  │  UXRCE_DDS_CFG = TELEM1   (串口通信)                    │"
        echo "  │  SER_TEL1_BAUD = 921600   (波特率匹配)                  │"
        echo "  └──────────────────────────────────────────────────────────┘"
    fi
}

# ═══════════════════════════════════════════════════════════════════════
# 汇总
# ═══════════════════════════════════════════════════════════════════════
print_summary() {
    local total=$((PASS_COUNT + FAIL_COUNT + WARN_COUNT + SKIP_COUNT))

    echo ""
    echo -e "${BOLD}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BOLD}║                    诊断汇总                                 ║${NC}"
    echo -e "${BOLD}╠══════════════════════════════════════════════════════════════╣${NC}"

    if [ $total -gt 0 ]; then
        printf "${BOLD}║${NC}  ${GREEN}✓ PASS${NC}  %-3d  ${RED}✗ FAIL${NC}  %-3d  ${YELLOW}! WARN${NC}  %-3d  ${CYAN}- SKIP${NC}  %-3d  ${BOLD}║${NC}\n" \
            $PASS_COUNT $FAIL_COUNT $WARN_COUNT $SKIP_COUNT
    fi

    echo -e "${BOLD}╠══════════════════════════════════════════════════════════════╣${NC}"

    if [ $PASS_COUNT -eq $total ] 2>/dev/null; then
        echo -e "${BOLD}║${NC}  ${GREEN}✓ 所有检查通过! 系统应可正常工作。                       ${BOLD}║${NC}"
        echo -e "${BOLD}║${NC}                                                          ${BOLD}║${NC}"
        echo -e "${BOLD}║${NC}  尝试解锁:                                                ${BOLD}║${NC}"
        echo -e "${BOLD}║${NC}    ~/ros2_ws/scripts/run_slam_px4.sh                    ${BOLD}║${NC}"
    else
        echo -e "${BOLD}║${NC}  ${RED}✗ 存在未通过的检查项, 按以下顺序修复:                      ${BOLD}║${NC}"
        echo -e "${BOLD}║${NC}    1. Layer 0 (物理接线) — 检查 TX/RX 是否交叉           ${BOLD}║${NC}"
        echo -e "${BOLD}║${NC}    2. Layer 1 (相机/VSLAM) — 检查相机和SLAM初始化         ${BOLD}║${NC}"
        echo -e "${BOLD}║${NC}    3. Layer 4 (串口通信) — 检查 /fmu/out 是否有数据       ${BOLD}║${NC}"
        echo -e "${BOLD}║${NC}    4. Layer 6 (EKF2 融合) — 检查 EKF2_EV_CTRL 参数       ${BOLD}║${NC}"
        echo -e "${BOLD}║${NC}    5. Layer 7 (飞前检查) — 用 QGC 查看具体报错             ${BOLD}║${NC}"
    fi

    echo -e "${BOLD}║${NC}                                                          ${BOLD}║${NC}"
    echo -e "${BOLD}║${NC}  诊断结论:                                                ${BOLD}║${NC}"

    # 自动判断最可能的断点
    if [ "${PX4_COMM_OK:-false}" = "true" ]; then
        echo -e "${BOLD}║${NC}  ${GREEN}✓ 串口通信正常 — PX4 TELEM1 接线正确, 双向数据流通      ${BOLD}║${NC}"
    else
        echo -e "${BOLD}║${NC}  ${RED}→ 数据断在 Layer 4 (串口通信层):${NC}                           ${BOLD}║${NC}"
        echo -e "${BOLD}║${NC}    最可能原因: TX/RX 未交叉 或 PX4参数未设置              ${BOLD}║${NC}"
    fi

    echo -e "${BOLD}║${NC}                                                          ${BOLD}║${NC}"

    if [ "${preflight:-false}" = "false" ] && [ "${PX4_COMM_OK:-false}" = "true" ]; then
        echo -e "${BOLD}║${NC}  ${RED}→ PX4 飞前检查失败, 无法解锁:${NC}                            ${BOLD}║${NC}"
        echo -e "${BOLD}║${NC}    最可能原因: COM_POWER_COUNT≠1 或 传感器未校准          ${BOLD}║${NC}"
    fi

    echo -e "${BOLD}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

# ═══════════════════════════════════════════════════════════════════════
# 主入口
# ═══════════════════════════════════════════════════════════════════════

usage() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  (无参数)            基础检查: 接线图 + Layer 0 (硬件+串口) + Layer 5 (参数表)"
    echo "  --serial            串口专项检查: 基础检查 + 自动 loopback 测试"
    echo "  --runtime           运行时检查: Layer 1-7 (需要系统已启动)"
    echo "  --all               完整诊断: 基础 + 运行时 (自动检测)"
    echo "  --quick             快速检查: 只看串口通信状态 (Layer 4)"
    echo "  --help, -h          显示帮助"
    echo ""
    echo "示例:"
    echo "  $0                        # 先检查硬件和环境"
    echo "  $0 --serial               # 检查串口接线 (含 loopback)"
    echo "  $0 --runtime              # 系统启动后检查数据链路"
    echo "  $0 --all                  # 一键完整诊断"
}

main() {
    echo -e "${BOLD}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BOLD}║  SLAM → PX4 全链路诊断工具  v1.0                            ║${NC}"
    echo -e "${BOLD}║  $(date '+%Y-%m-%d %H:%M:%S')                                             ║${NC}"
    echo -e "${BOLD}╚══════════════════════════════════════════════════════════════╝${NC}"

    case "${1:-}" in
        --help|-h)
            print_wiring_diagram
            usage
            exit 0
            ;;
        --serial)
            print_wiring_diagram
            layer0_uart_wiring "--loopback"
            print_summary
            exit 0
            ;;
        --quick)
            source_ros2
            layer4_microxrce_agent
            print_summary
            exit 0
            ;;
        --runtime)
            # 需要系统运行中
            source_ros2
            echo ""
            echo -e "${YELLOW}注意: 此模式需要 SLAM+PX4 系统已启动${NC}"
            echo -e "${YELLOW}      如果未启动, 请先运行: cd $ISAAC_WS && ./run_slam_px4.sh --bg${NC}"
            echo ""
            layer1_camera_slam
            layer2_dds_discovery
            layer3_odom_bridge
            layer4_microxrce_agent
            layer5_px4_params
            layer6_ekf2_fusion
            layer7_arm_readiness
            print_summary
            exit 0
            ;;
        --all)
            print_wiring_diagram
            layer0_uart_wiring  # 基础串口检查，不做 loopback
            source_ros2
            echo ""
            echo -e "${YELLOW}── 以下检查需要系统运行中 (run_slam_px4.sh --bg) ──${NC}"
            layer1_camera_slam
            layer2_dds_discovery
            layer3_odom_bridge
            layer4_microxrce_agent
            layer5_px4_params
            layer6_ekf2_fusion
            layer7_arm_readiness
            print_summary
            exit 0
            ;;
        "")
            # 默认: 基础检查
            print_wiring_diagram
            layer0_uart_wiring
            echo ""
            echo -e "${YELLOW}── 基础检查完成。如需检查运行时数据链路, 请: ──${NC}"
            echo -e "${YELLOW}   1. 启动系统: cd $ISAAC_WS && ./run_slam_px4.sh --bg${NC}"
            echo -e "${YELLOW}   2. 运行诊断: $0 --runtime${NC}"
            echo ""
            print_summary
            exit 0
            ;;
        *)
            echo -e "${RED}未知选项: $1${NC}"
            usage
            exit 1
            ;;
    esac
}

main "$@"
