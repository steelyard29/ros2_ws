#!/usr/bin/env bash
# Quantify a recorded experiment against localization flight gates.
set -eo pipefail

ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
CSV="${1:-}"
if [ -z "$CSV" ] || [ ! -f "$CSV" ]; then
  echo "Usage: $0 /path/to/flight_pose_*.csv" >&2
  exit 2
fi

# The evaluator intentionally returns non-zero for a failed gate.  Do not let
# `set -e` hide the diagnostic report and the explicit safety decision below.
set +e
python3 "$ROS2_WS/scripts/eval_vio_replay.py" "$CSV" --label "$(basename "$(dirname "$CSV")")"
STATUS=$?
set -e

echo ""
echo "Flight gates (localization):"
echo "  static XY <= 5 cm / 60 s"
echo "  hand-lift / hover XY span <= 10 cm above 0.8 m AGL"
echo "  powered record required (active motors + Offboard setpoints)"
echo "  powered PX4 local XY span <= 10 cm above 0.8 m AGL"
echo "  no sustained inertial dead reckoning"
if [ "$STATUS" -eq 0 ]; then
  echo "RESULT: PASS — may request next altitude gate"
else
  echo "RESULT: FAIL — do not increase altitude; fix VIO/fusion first"
fi
exit "$STATUS"
