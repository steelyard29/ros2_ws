#!/bin/bash
# PX4 Quad X 电机映射/斜起飞软件侧诊断
#
# 安全边界:
#   - 只读检查参数和 ROS/PX4 topic
#   - 不 Arm
#   - 不发送 actuator test
#   - 不修改 PX4 参数

set -u

ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
PARAM_FILE="${PX4_PARAM_FILE:-$ROS2_WS/config/px4/recovered_params.txt}"
SPIN_TIME="${ROS_DISCOVERY_SPIN_TIME:-3}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0
SKIP_COUNT=0

pass() { echo -e "  ${GREEN}[PASS]${NC} $1"; PASS_COUNT=$((PASS_COUNT + 1)); }
warn() { echo -e "  ${YELLOW}[WARN]${NC} $1"; WARN_COUNT=$((WARN_COUNT + 1)); }
fail() { echo -e "  ${RED}[FAIL]${NC} $1"; FAIL_COUNT=$((FAIL_COUNT + 1)); }
skip() { echo -e "  ${CYAN}[SKIP]${NC} $1"; SKIP_COUNT=$((SKIP_COUNT + 1)); }
info() { echo -e "  ${CYAN}[INFO]${NC} $1"; }

section() {
    echo ""
    echo -e "${BOLD}═══════════════════════════════════════════════════════${NC}"
    echo -e "${BOLD}  $1${NC}"
    echo -e "${BOLD}═══════════════════════════════════════════════════════${NC}"
}

param_get() {
    local key="$1"
    if [ ! -f "$PARAM_FILE" ]; then
        return 1
    fi
    awk -F',' -v key="$key" '$1 == key {print $2; exit}' "$PARAM_FILE"
}

topic_exists() {
    local topic="$1"
    echo "$TOPICS" | grep -qx "$topic"
}

ros_echo_once() {
    local topic="$1"
    timeout 5 ros2 topic echo --once --qos-reliability best_effort "$topic" 2>/dev/null
}

source_ros() {
    set +u
    if [ -f /opt/ros/humble/setup.bash ]; then
        # shellcheck disable=SC1091
        source /opt/ros/humble/setup.bash
    fi
    if [ -f "$ROS2_WS/install/setup.bash" ]; then
        # shellcheck disable=SC1090
        source "$ROS2_WS/install/setup.bash"
    fi
    set -u
    export ROS2_DISABLE_DAEMON=1
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
    if [ -f "$ROS2_WS/config/fastdds_bridge.xml" ]; then
        export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$ROS2_WS/config/fastdds_bridge.xml}"
    fi
}

print_qgc_motor_checklist() {
    section "0. 必做安全检查: QGC Motor Test"
    echo "  测试前必须拆桨。此脚本不会发电机测试命令。"
    echo ""
    echo "  PX4 Generic Quad X (SYS_AUTOSTART=4001) 预期编号:"
    echo "    Motor 1: 前右"
    echo "    Motor 2: 后左"
    echo "    Motor 3: 前左"
    echo "    Motor 4: 后右"
    echo ""
    echo "  请在 QGC > Actuators 逐个点 Motor 1~4，记录:"
    echo "    1) 实际转动的物理角"
    echo "    2) 旋向"
    echo "    3) 是否和 PX4 编号一致"
}

check_param_file() {
    section "1. PX4 参数备份检查"

    if [ ! -f "$PARAM_FILE" ]; then
        fail "找不到参数备份: $PARAM_FILE"
        echo "    可通过 QGC 导出参数，或设置 PX4_PARAM_FILE=/path/to/params"
        return
    fi
    pass "参数备份存在: $PARAM_FILE"

    local autostart rotor_count board_rot
    autostart="$(param_get SYS_AUTOSTART || true)"
    rotor_count="$(param_get CA_ROTOR_COUNT || true)"
    board_rot="$(param_get SENS_BOARD_ROT || true)"

    if [ "$autostart" = "4001" ]; then
        pass "SYS_AUTOSTART=4001 (Generic Quad X)"
    else
        warn "SYS_AUTOSTART=$autostart，不是 4001；请按当前机架的 PX4 电机图核对"
    fi

    if [ "$rotor_count" = "4" ]; then
        pass "CA_ROTOR_COUNT=4"
    else
        fail "CA_ROTOR_COUNT=$rotor_count，应为四旋翼的 4"
    fi

    if [ "$board_rot" = "0" ]; then
        pass "SENS_BOARD_ROT=0 (飞控箭头应朝机头)"
    else
        warn "SENS_BOARD_ROT=$board_rot；若飞控未按箭头朝前安装，需确认该旋转设置正确"
    fi
}

check_control_allocation() {
    section "2. Control Allocation 几何检查"

    local expected_keys=(
        "CA_ROTOR0_PX=1.0" "CA_ROTOR0_PY=1.0" "CA_ROTOR0_KM=0.05000000074505806"
        "CA_ROTOR1_PX=-1.0" "CA_ROTOR1_PY=-1.0" "CA_ROTOR1_KM=0.05000000074505806"
        "CA_ROTOR2_PX=1.0" "CA_ROTOR2_PY=-1.0" "CA_ROTOR2_KM=-0.05000000074505806"
        "CA_ROTOR3_PX=-1.0" "CA_ROTOR3_PY=1.0" "CA_ROTOR3_KM=-0.05000000074505806"
    )

    local mismatch=0
    for item in "${expected_keys[@]}"; do
        local key="${item%%=*}"
        local expected="${item#*=}"
        local actual
        actual="$(param_get "$key" || true)"
        if [ "$actual" = "$expected" ]; then
            pass "$key=$actual"
        else
            warn "$key=$actual (4001 默认期望 $expected)"
            mismatch=1
        fi
    done

    if [ "$mismatch" -eq 0 ]; then
        pass "CA_ROTOR0~3 几何与 4001 Quad X 参数一致"
    else
        warn "Control Allocation 几何与 4001 默认不完全一致；优先用 QGC Actuators 确认机架图"
    fi
}

check_pwm_mapping() {
    section "3. PWM 输出映射检查"

    local main_funcs=()
    local motor_ports=()
    local motor_functions=()
    local i func

    for i in 1 2 3 4 5 6 7 8; do
        func="$(param_get "PWM_MAIN_FUNC$i" || echo "")"
        main_funcs+=("$func")
        printf "  MAIN%-2d -> function %s\n" "$i" "${func:-unset}"
        case "$func" in
            101|102|103|104)
                motor_ports+=("MAIN$i")
                motor_functions+=("$func")
                ;;
        esac
    done

    echo ""
    if [ "${main_funcs[0]}" = "101" ] && [ "${main_funcs[1]}" = "102" ] \
        && [ "${main_funcs[2]}" = "103" ] && [ "${main_funcs[3]}" = "104" ]; then
        pass "Motor1~4 连续映射到 MAIN1~MAIN4"
    elif [ "${main_funcs[0]}" = "101" ] && [ "${main_funcs[2]}" = "102" ] \
        && [ "${main_funcs[4]}" = "103" ] && [ "${main_funcs[6]}" = "104" ]; then
        warn "Motor1~4 当前映射到 MAIN1/3/5/7"
        echo "    如果 ESC 实际插在 MAIN1/2/3/4，这是高概率软件/参数原因。"
        echo "    若 ESC 确实插在 MAIN1/3/5/7，则该映射可以保留。"
    else
        warn "Motor1~4 映射不是常见连续 1/2/3/4，也不是当前备份中的 1/3/5/7"
    fi

    for required in 101 102 103 104; do
        local count
        count=0
        for func in "${main_funcs[@]}"; do
            [ "$func" = "$required" ] && count=$((count + 1))
        done
        if [ "$count" -eq 1 ]; then
            pass "function $required 仅映射一次"
        elif [ "$count" -eq 0 ]; then
            fail "function $required 未映射到 MAIN 输出"
        else
            fail "function $required 重复映射 $count 次"
        fi
    done

    echo ""
    echo "  若 ESC 插在 MAIN1/2/3/4，建议目标映射为:"
    echo "    PWM_MAIN_FUNC1=101"
    echo "    PWM_MAIN_FUNC2=102"
    echo "    PWM_MAIN_FUNC3=103"
    echo "    PWM_MAIN_FUNC4=104"
    echo "    PWM_MAIN_FUNC5=0"
    echo "    PWM_MAIN_FUNC7=0"
}

check_estimator_params() {
    section "4. EKF/VSLAM 相关参数提示"

    local ev_ctrl of_ctrl ev_delay hgt_ref
    ev_ctrl="$(param_get EKF2_EV_CTRL || true)"
    of_ctrl="$(param_get EKF2_OF_CTRL || true)"
    ev_delay="$(param_get EKF2_EV_DELAY || true)"
    hgt_ref="$(param_get EKF2_HGT_REF || true)"

    case "$ev_ctrl" in
        1)
            pass "EKF2_EV_CTRL=$ev_ctrl (仅水平位置；升降验收基线)"
            ;;
        5)
            warn "EKF2_EV_CTRL=$ev_ctrl (含 EV 速度；仅在速度验收后启用)"
            ;;
        9)
            pass "EKF2_EV_CTRL=$ev_ctrl (水平位置+yaw；无 EV 速度/高度)"
            ;;
        13)
            warn "EKF2_EV_CTRL=$ev_ctrl (含 EV 速度+yaw；需完整验收)"
            ;;
        15)
            warn "EKF2_EV_CTRL=$ev_ctrl 含 EV 垂直；室内激光定高应改为 1 或 9"
            ;;
        *)
            warn "EKF2_EV_CTRL=$ev_ctrl；室内激光定高建议 1 (或验收后 9)"
            ;;
    esac

    if [ "$hgt_ref" = "2" ]; then
        pass "EKF2_HGT_REF=2 (高度参考使用向下测距)"
    else
        warn "EKF2_HGT_REF=$hgt_ref；TFmini 定高方案建议设为 2"
    fi

    if [ "$of_ctrl" = "0" ]; then
        pass "EKF2_OF_CTRL=0 (未启用 optical flow)"
    else
        warn "EKF2_OF_CTRL=$of_ctrl；若没有光流传感器，建议确认是否误启用"
    fi

    info "EKF2_EV_DELAY=$ev_delay, EKF2_HGT_REF=$hgt_ref"
}

check_runtime_topics() {
    section "5. 运行时只读遥测检查"

    source_ros
    TOPICS="$(ros2 topic list --no-daemon --spin-time "$SPIN_TIME" 2>/dev/null || true)"

    if [ -z "$TOPICS" ] || ! echo "$TOPICS" | grep -q "/fmu/out/vehicle_status_v1"; then
        warn "当前没有发现 PX4 DDS topic"
        echo "    启动后再运行: $ROS2_WS/scripts/run_slam_px4.sh --bg"
        echo "    然后重跑:   $ROS2_WS/scripts/diagnose_motor_mapping.sh"
        return
    fi

    pass "PX4 DDS topic 在线"
    echo "$TOPICS" | sed 's/^/    /'

    echo ""
    for topic in \
        /fmu/out/vehicle_status_v1 \
        /fmu/out/vehicle_local_position \
        /fmu/out/vehicle_odometry \
        /fmu/out/vehicle_attitude \
        /fmu/out/actuator_outputs \
        /fmu/out/actuator_motors \
        /fmu/out/esc_status
    do
        if topic_exists "$topic"; then
            pass "$topic 存在"
        else
            skip "$topic 不存在或未桥接"
        fi
    done

    echo ""
    if topic_exists "/fmu/out/vehicle_status_v1"; then
        echo "  ── vehicle_status_v1 ──"
        ros_echo_once /fmu/out/vehicle_status_v1 | sed 's/^/    /'
    fi

    if topic_exists "/fmu/out/vehicle_odometry"; then
        echo ""
        echo "  ── vehicle_odometry 姿态/高度快照 ──"
        local odom
        odom="$(ros_echo_once /fmu/out/vehicle_odometry || true)"
        echo "$odom" | sed 's/^/    /'
        echo "$odom" | python3 -c '
import sys, math, re
text = sys.stdin.read()
nums = re.findall(r"q:\n(?:- ([^\n]+)\n){4}", text)
if not nums:
    q = []
    grab = False
    for line in text.splitlines():
        if line.strip() == "q:":
            grab = True
            continue
        if grab and line.strip().startswith("- "):
            q.append(float(line.strip()[2:]))
            if len(q) == 4:
                break
    if len(q) == 4:
        w, x, y, z = q
        roll = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
        pitch = math.degrees(math.asin(max(-1, min(1, 2*(w*y - z*x)))))
        yaw = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
        print(f"    姿态估算: roll={roll:.1f} deg, pitch={pitch:.1f} deg, yaw={yaw:.1f} deg")
        if abs(roll) > 5 or abs(pitch) > 5:
            print("    [WARN] 静止水平放置时 roll/pitch 不应超过约 5 deg，请检查飞控安装方向/姿态估计")
' || true
    fi

    if topic_exists "/fmu/out/actuator_outputs"; then
        echo ""
        echo "  ── actuator_outputs 快照 ──"
        local outputs
        outputs="$(ros_echo_once /fmu/out/actuator_outputs || true)"
        echo "$outputs" | sed 's/^/    /'
        echo "$outputs" | python3 -c '
import sys, re
text = sys.stdin.read()
values = []
grab = False
for line in text.splitlines():
    s = line.strip()
    if s == "output:":
        grab = True
        continue
    if grab:
        if s.startswith("- "):
            try:
                values.append(float(s[2:]))
            except ValueError:
                pass
        elif values:
            break
if len(values) >= 4:
    first = values[:4]
    avg = sum(first) / 4.0
    print("    前四路输出:", ", ".join(f"{v:.1f}" for v in first))
    if avg > 1:
        for idx, value in enumerate(first, 1):
            diff = value - avg
            print(f"    OUT{idx}: diff_from_avg={diff:.1f}")
        if min(first) < avg - 80:
            print("    [WARN] 前四路中存在明显偏低输出；若无桨低油门测试时出现，优先查映射/姿态/控制分配")
' || true
    fi
}

print_summary() {
    section "结论与下一步"

    echo "  PASS=$PASS_COUNT WARN=$WARN_COUNT FAIL=$FAIL_COUNT SKIP=$SKIP_COUNT"
    echo ""
    echo "  优先级建议:"
    echo "    1. 拆桨，用 QGC Actuators 验证 Motor1~4 对应物理角。"
    echo "    2. 若 ESC 插在 MAIN1/2/3/4，先修正 PWM_MAIN_FUNC 连续映射。"
    echo "    3. 若映射正确，再看静止姿态 roll/pitch 与 actuator 输出是否异常。"
    echo "    4. 软件输出均衡但 3 号实际转速低，再转硬件侧排查 ESC/电机/桨/供电。"
}

main() {
    echo -e "${BOLD}PX4 Quad X 电机映射/斜起飞软件侧诊断${NC}"
    echo "参数文件: $PARAM_FILE"
    echo "安全: 只读，不 Arm，不发电机测试，不改参数"

    print_qgc_motor_checklist
    check_param_file
    check_control_allocation
    check_pwm_mapping
    check_estimator_params
    check_runtime_topics
    print_summary
}

main "$@"
