#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════
# DDS 版 EV yaw 对齐验收（替代博客 MAV_ODOM_LP）
#
# 对比:
#   /visual_slam/tracking/odometry          (VSLAM ENU)
#   /fmu/in/vehicle_visual_odometry         (Bridge → PX4 NED)
#   /fmu/out/vehicle_attitude               (PX4 NED)
#
# 通过前务必保持 EKF2_EV_CTRL=1（不开 EV yaw/EV velocity）。
# 验收通过后:
#   PX4_ENABLE_EV_YAW=1 python3 ~/ros2_ws/scripts/configure_px4_dds.py
#   或手动: EKF2_EV_CTRL=9（水平位置+yaw，无 EV 速度/高度）
#
# 用法:
#   ./verify_ev_yaw_align.sh              # 静置采样
#   ./verify_ev_yaw_align.sh --rotate     # 交互: 绕竖轴 ±90°
#   ./verify_ev_yaw_align.sh --displace   # 交互: 前/右/上位移符号
# ═══════════════════════════════════════════════════════════════════

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
SAMPLE_TIMEOUT_S="${SAMPLE_TIMEOUT_S:-6}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

pass() { echo -e "${GREEN}✓${NC} $*"; }
fail() { echo -e "${RED}✗${NC} $*"; FAILS=$((FAILS + 1)); }
warn() { echo -e "${YELLOW}!${NC} $*"; }
info() { echo -e "${CYAN}·${NC} $*"; }

FAILS=0
MODE="static"

for arg in "$@"; do
  case "$arg" in
    --rotate) MODE="rotate" ;;
    --displace) MODE="displace" ;;
    --static) MODE="static" ;;
    -h|--help)
      sed -n '2,25p' "$0"
      exit 0
      ;;
  esac
done

source_ros() {
  # ROS setup.bash references unset vars (e.g. AMENT_TRACE_SETUP_FILES).
  set +u
  if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  fi
  if [ -f "$ROS2_WS/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$ROS2_WS/install/setup.bash"
  fi
  set -u
}

# One-shot sample via rclpy (Best Effort / SensorData QoS).
# Prints KEY=VALUE lines; exit 0 on success.
sample_python() {
  SAMPLE_TIMEOUT_S="$SAMPLE_TIMEOUT_S" python3 - <<'PY'
import math
import os
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude, VehicleOdometry

TIMEOUT = float(os.environ.get("SAMPLE_TIMEOUT_S", "6"))


def yaw_wxyz(w, x, y, z):
    return math.degrees(math.atan2(2.0 * (w * z + x * y),
                                   1.0 - 2.0 * (y * y + z * z)))


def rpy_wxyz(w, x, y, z):
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def wrap_err(a, b):
    d = math.atan2(math.sin(math.radians(a - b)),
                   math.cos(math.radians(a - b)))
    return math.degrees(d)


rclpy.init()
node = rclpy.create_node("verify_ev_yaw_align_sample")
qos = qos_profile_sensor_data
got = {"vslam": None, "bridge": None, "att": None}


def on_vslam(msg):
    got["vslam"] = msg


def on_bridge(msg):
    got["bridge"] = msg


def on_att(msg):
    got["att"] = msg


node.create_subscription(Odometry, "/visual_slam/tracking/odometry", on_vslam, qos)
node.create_subscription(VehicleOdometry, "/fmu/in/vehicle_visual_odometry",
                         on_bridge, qos)
node.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude",
                         on_att, qos)

deadline = time.monotonic() + TIMEOUT
missing = []
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
    if all(got.values()):
        break
else:
    missing = [k for k, v in got.items() if v is None]

node.destroy_node()
rclpy.shutdown()

if missing:
    print(f"MISSING={','.join(missing)}", flush=True)
    sys.exit(2)

v = got["vslam"]
b = got["bridge"]
a = got["att"]
oq = v.pose.pose.orientation
vslam_yaw = yaw_wxyz(oq.w, oq.x, oq.y, oq.z)
br, bp, by = rpy_wxyz(b.q[0], b.q[1], b.q[2], b.q[3])
ar, ap, ay = rpy_wxyz(a.q[0], a.q[1], a.q[2], a.q[3])
yaw_err = wrap_err(by, ay)

print(f"VSLAM_YAW={vslam_yaw:.2f}")
print(f"BRIDGE_RPY={br:.2f},{bp:.2f},{by:.2f}")
print(f"PX4_RPY={ar:.2f},{ap:.2f},{ay:.2f}")
print(f"YAW_ERR={yaw_err:.2f}")
print(f"POS_NED={b.position[0]:.4f},{b.position[1]:.4f},{b.position[2]:.4f}")
sys.exit(0)
PY
}

sample_once() {
  local label="$1"
  echo ""
  info "采样: $label (最多 ${SAMPLE_TIMEOUT_S}s, SensorData QoS)…"

  local out rc=0
  out="$(sample_python)" || rc=$?

  if [ "$rc" -ne 0 ]; then
    local miss
    miss="$(echo "$out" | awk -F= '/^MISSING=/{print $2; exit}')"
    fail "采样超时/失败 (missing=${miss:-unknown}) — 确认 run_slam_px4 在跑且 topic 有数据"
    echo "$out" | sed 's/^/  /'
    return 1
  fi

  local vslam_yaw bridge_rpy px4_rpy yaw_err pos
  vslam_yaw="$(echo "$out" | awk -F= '/^VSLAM_YAW=/{print $2; exit}')"
  bridge_rpy="$(echo "$out" | awk -F= '/^BRIDGE_RPY=/{print $2; exit}')"
  px4_rpy="$(echo "$out" | awk -F= '/^PX4_RPY=/{print $2; exit}')"
  yaw_err="$(echo "$out" | awk -F= '/^YAW_ERR=/{print $2; exit}')"
  pos="$(echo "$out" | awk -F= '/^POS_NED=/{print $2; exit}')"

  local br bp by ar ap ay
  IFS=',' read -r br bp by <<<"$bridge_rpy"
  IFS=',' read -r ar ap ay <<<"$px4_rpy"
  IFS=',' read -r LAST_POS_N LAST_POS_E LAST_POS_D <<<"$pos"

  echo "  VSLAM ENU yaw:     ${vslam_yaw} deg"
  echo "  Bridge NED r/p/y:  ${br} / ${bp} / ${by} deg"
  echo "  PX4   NED r/p/y:   ${ar} / ${ap} / ${ay} deg"
  echo "  Bridge−PX4 yaw:    ${yaw_err} deg"
  echo "  Bridge NED pos:    N=${LAST_POS_N} E=${LAST_POS_E} D=${LAST_POS_D}"

  LAST_BRIDGE_YAW="$by"
  LAST_PX4_YAW="$ay"
  LAST_BRIDGE_ROLL="$br"
  LAST_BRIDGE_PITCH="$bp"
  LAST_YAW_ERR="$yaw_err"
}

wrap_err_deg() {
  python3 -c "import math; a=float('$1'); b=float('$2'); d=math.atan2(math.sin(math.radians(a-b)), math.cos(math.radians(a-b))); print(f'{math.degrees(d):.2f}')"
}

echo "════════════════════════════════════════"
echo " EV Yaw 对齐验收 (DDS)"
echo " mode=$MODE"
echo "════════════════════════════════════════"
info "通过前保持 EKF2_EV_CTRL=1；勿开 EV yaw/EV velocity"
echo ""

source_ros

need_topics=(
  /visual_slam/tracking/odometry
  /fmu/in/vehicle_visual_odometry
  /fmu/out/vehicle_attitude
)

info "发现 topic (最多 8s)…"
topic_list="$(timeout -k 2 8 ros2 topic list 2>/dev/null || true)"
for t in "${need_topics[@]}"; do
  if echo "$topic_list" | grep -qx "$t"; then
    pass "topic 可见: $t"
  else
    fail "缺少 topic: $t — 请先启动 run_slam_px4.sh"
  fi
done
[ "$FAILS" -eq 0 ] || exit 1

case "$MODE" in
  static)
    info "请保持机体静置…"
    sample_once "静置" || true
    if [ -z "${LAST_YAW_ERR:-}" ]; then
      fail "无有效静置样本"
    else
      abs_err="$(python3 -c "print(abs(float('${LAST_YAW_ERR}')))")"
      if python3 -c "import sys; sys.exit(0 if float('$abs_err') < 15.0 else 1)"; then
        pass "静置 Bridge↔PX4 yaw 误差 ${LAST_YAW_ERR}° < 15°"
      else
        fail "静置 Bridge↔PX4 yaw 误差 ${LAST_YAW_ERR}° ≥ 15° — 检查 align_yaw_to_px4 / 原点锁定时机"
      fi
      if python3 -c "import sys; sys.exit(0 if abs(float('${LAST_BRIDGE_ROLL:-99}')) < 20 and abs(float('${LAST_BRIDGE_PITCH:-99}')) < 20 else 1)"; then
        pass "Bridge roll/pitch 接近水平 (|r|,|p|<20°)"
      else
        warn "Bridge roll/pitch 偏大: r=${LAST_BRIDGE_ROLL} p=${LAST_BRIDGE_PITCH}"
      fi
    fi
    ;;
  rotate)
    echo ""
    info "步骤: 记录基准 → 仅绕竖轴缓慢转约 +90° → 再采样"
    sample_once "旋转前基准" || exit 1
    yaw0="$LAST_BRIDGE_YAW"
    px40="$LAST_PX4_YAW"
    read -r -p "转完 +90° 后按 Enter…" _
    sample_once "旋转后" || exit 1
    d_bridge="$(wrap_err_deg "$LAST_BRIDGE_YAW" "$yaw0")"
    d_px4="$(wrap_err_deg "$LAST_PX4_YAW" "$px40")"
    echo "  Δ Bridge yaw: ${d_bridge}°"
    echo "  Δ PX4 yaw:    ${d_px4}°"
    if python3 -c "import sys; b=float('$d_bridge'); p=float('$d_px4'); sys.exit(0 if b*p>0 and abs(b)>40 and abs(p)>40 and abs(b-p)<35 else 1)"; then
      pass "绕竖轴旋转: Bridge 与 PX4 同向同步"
    else
      fail "绕竖轴旋转不同步或幅度不足 — 检查 ENU→NED / FLU→FRD 与 yaw 对齐"
    fi
    if python3 -c "import sys; sys.exit(0 if abs(float('${LAST_BRIDGE_ROLL}'))<25 and abs(float('${LAST_BRIDGE_PITCH}'))<25 else 1)"; then
      pass "旋转后 roll/pitch 仍≈0（无倾倒耦合）"
    else
      warn "旋转后 roll/pitch 偏大: r=${LAST_BRIDGE_ROLL} p=${LAST_BRIDGE_PITCH}"
    fi
    ;;
  displace)
    echo ""
    info "步骤: 记录基准 → 依次前移 / 右移 / 上抬（手持机体）"
    sample_once "位移前基准" || exit 1
    n0="$LAST_POS_N"; e0="$LAST_POS_E"; d0="$LAST_POS_D"
    read -r -p "向前移动约 0.5m 后按 Enter…" _
    sample_once "前移后" || exit 1
    dn="$(python3 -c "print(float('${LAST_POS_N}')-float('$n0'))")"
    info "ΔN=${dn} (前移应对应 NED North 增加，取决于机头朝向；看相对符号一致性)"
    read -r -p "向右移动约 0.5m 后按 Enter…" _
    sample_once "右移后" || exit 1
    de="$(python3 -c "print(float('${LAST_POS_E}')-float('$e0'))")"
    info "ΔE=${de}"
    read -r -p "向上抬约 0.3m 后按 Enter…" _
    sample_once "上抬后" || exit 1
    dd="$(python3 -c "print(float('${LAST_POS_D}')-float('$d0'))")"
    if python3 -c "import sys; sys.exit(0 if float('$dd') < -0.1 else 1)"; then
      pass "上抬 → Bridge NED D 减小 (ΔD=${dd})"
    else
      fail "上抬后 D 未减小 (ΔD=${dd}) — 检查垂直轴符号"
    fi
    warn "前/右位移请目视确认 N/E 符号与机头 NED 一致 (ΔN=${dn}, ΔE=${de})"
    ;;
esac

echo ""
echo "════════════════════════════════════════"
if [ "$FAILS" -eq 0 ]; then
  pass "验收通过。启用 EV yaw:"
  echo "  PX4_ENABLE_EV_YAW=1 PX4_SKIP_REBOOT=0 python3 $SCRIPT_DIR/configure_px4_dds.py"
  echo "  或 QGC: EKF2_EV_CTRL=9 （水平位置+yaw；仍保持 BARO_CTRL=0, MAG_TYPE=5, HGT_REF=2, RNG_CTRL=2）"
  echo "  然后悬停 15s 确认无明显旋转与水平漂移。"
  exit 0
else
  fail "验收未通过 ($FAILS 项)。保持 EKF2_EV_CTRL=1，勿开 EV yaw/EV velocity。"
  exit 1
fi
