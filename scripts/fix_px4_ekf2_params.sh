#!/bin/bash
# ═══════════════════════════════════════════════════════════════════
# PX4 EKF2 参数修复 — 将漂移的参数恢复为正确的室内激光定高配置
#
# 背景:
#   PX4 飞控当前运行的 EKF2 参数可能与 SD 卡标准不一致:
#     EKF2_HGT_REF=3   (应为 2, 主高度源是 VSLAM 而非激光)
#     EKF2_EV_QMIN=0   (应为 50, 未过滤低质量视觉数据)
#     EKF2_RNG_NOISE 等参数漂移
#   修正为与 SD 卡 /etc/config.txt 完全一致的标准参数。
#
# 用法:
#   方法A (USB NSH, 推荐):  ./fix_px4_ekf2_params.sh
#   方法B (MAVLink USB):    ./fix_px4_ekf2_params.sh --mavlink
# ═══════════════════════════════════════════════════════════════════

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

echo ""
echo -e "${BOLD}═══════════════════════════════════════════════════════${NC}"
echo -e "${BOLD}  PX4 EKF2 参数修复工具${NC}"
echo -e "${BOLD}═══════════════════════════════════════════════════════${NC}"
echo ""

MODE="${1:-nsh}"

# ── 需要修正的参数列表 ──
declare -A PARAMS
PARAMS=(
    # EKF2 视觉融合: 全自由度 (HPOS+VPOS+VEL+YAW)
    ["EKF2_EV_CTRL"]="15"
    # 主高度参考: TFmini 激光测距
    ["EKF2_HGT_REF"]="2"
    # 视觉里程计最低质量阈值
    ["EKF2_EV_QMIN"]="50"
    # 视觉延迟补偿
    ["EKF2_EV_DELAY"]="0"
    # 相机安装偏移清零 (bridge 已发布 base_link 位姿)
    ["EKF2_EV_POS_X"]="0"
    ["EKF2_EV_POS_Y"]="0"
    ["EKF2_EV_POS_Z"]="0"
    # TFmini 在 CG 下方 5cm
    ["EKF2_RNG_POS_Z"]="0.05"
    # 测距仪始终融合
    ["EKF2_RNG_CTRL"]="2"
    # 关气压计(室内)
    ["EKF2_BARO_CTRL"]="0"
    # 关磁力计(室内)
    ["EKF2_MAG_TYPE"]="5"
    # 关GPS
    ["EKF2_GPS_CTRL"]="0"
    # 关光流
    ["EKF2_OF_CTRL"]="0"
    # MPC 高度模式: 测距仪
    ["MPC_ALT_MODE"]="2"
    # 视觉噪声参数
    ["EKF2_EVA_NOISE"]="0.25"
    ["EKF2_EVP_NOISE"]="0.20"
    ["EKF2_EVV_NOISE"]="0.15"
    # 测距仪噪声和门限参数
    ["EKF2_RNG_NOISE"]="0.15"
    ["EKF2_RNG_GATE"]="6.0"
    ["EKF2_RNG_K_GATE"]="3.0"
)

# ── 方法A: USB NSH Shell ──
method_nsh() {
    echo -e "${CYAN}方法A: 通过 USB NSH Shell 修改参数${NC}"
    echo ""
    echo -e "步骤:"
    echo -e "  1. 用 USB 线连接 PX4 飞控到 Jetson"
    echo -e "  2. 确认 USB 设备出现:"
    echo -e "     ${BOLD}ls /dev/ttyACM*${NC}"
    echo ""
    echo -e "  3. 连接 NSH shell:"
    echo -e "     ${BOLD}sudo screen /dev/ttyACM0 57600${NC}"
    echo -e "     (按 Enter 出现 nsh> 提示符)"
    echo ""
    echo -e "  4. 在 ${BOLD}nsh>${NC} 提示符下，逐行粘贴以下命令:"
    echo ""

    for param in "${!PARAMS[@]}"; do
        printf "     ${GREEN}param set %-20s %s${NC}\n" "$param" "${PARAMS[$param]}"
    done

    echo ""
    echo -e "     ${BOLD}param save${NC}"
    echo -e "     ${BOLD}reboot${NC}"
    echo ""

    # 也输出可直接复制粘贴的版本
    echo -e "${YELLOW}━━━━━━━━ 一键复制粘贴版 (NSH) ━━━━━━━━${NC}"
    echo ""
    for param in "${!PARAMS[@]}"; do
        echo "param set $param ${PARAMS[$param]}"
    done
    echo "param save"
    echo "reboot"
}

# ── 方法B: MAVLink (需先临时开 USB MAVLink) ──
method_mavlink() {
    echo -e "${CYAN}方法B: 通过 MAVLink USB 修改参数${NC}"
    echo ""
    echo -e "  前置: 需要先通过 NSH 临时开启 USB MAVLink:"
    echo -e "    ${BOLD}nsh> SYS_USB_AUTO set 1${NC}"
    echo -e "    ${BOLD}nsh> reboot${NC}"
    echo ""
    echo -e "  重启后运行:"
    echo -e "    ${BOLD}cd ~/ros2_ws/scripts${NC}"
    echo -e "    ${BOLD}python3 configure_px4_dds.py${NC}"
    echo ""
    echo -e "  configure_px4_dds.py 已包含所有正确参数值，会自动:"
    echo -e "    1. 搜索串口 (/dev/ttyACM0, /dev/ttyUSB0 等)"
    echo -e "    2. 逐一设置并验证每个参数"
    echo -e "    3. 保存到 flash"
    echo -e "    4. 重启飞控"
    echo ""
    echo -e "  完成后建议恢复:"
    echo -e "    ${BOLD}nsh> SYS_USB_AUTO set 0${NC}"
    echo -e "    ${BOLD}nsh> param save && reboot${NC}"
}

# ── 验证参数的方法 ──
show_verify() {
    echo ""
    echo -e "${YELLOW}━━━━━━━━ 验证参数是否生效 ━━━━━━━━${NC}"
    echo ""
    echo -e "  重启后在 NSH 中检查:"
    echo ""
    for param in "${!PARAMS[@]}"; do
        printf "     ${BOLD}param show %-20s${NC}  ${GREEN}# 应为 %s${NC}\n" "$param" "${PARAMS[$param]}"
    done
    echo ""
    echo -e "  PX4 重启后在 Jetson 上验证话题:"
    echo -e "     ${BOLD}cd ~/ros2_ws/scripts${NC}"
    echo -e "     ${BOLD}./run_slam_px4.sh --check-arm${NC}"
}

case "$MODE" in
    --mavlink|mavlink|-m)
        method_mavlink
        ;;
    --verify|verify|-v)
        show_verify
        ;;
    --all|all|-a)
        method_nsh
        echo ""
        echo -e "${YELLOW}───────────────────────────────────────────────────${NC}"
        echo ""
        method_mavlink
        ;;
    *)
        method_nsh
        ;;
esac

show_verify

echo ""
echo -e "${GREEN}═══════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  修复后建议: 低空(1m)不带桨测试 → 带桨低空悬停 → 正常飞行${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════════${NC}"
