#!/usr/bin/env python3
"""Offline VIO candidate scoring from a flight_experiment CSV.

Compares absolute XY drift of the recorded source against the plan gates:
  - static: XY max in any 60 s window <= 0.05 m
  - vertical lift: high-altitude XY span <= 0.10 m

This does not run ORB-SLAM3/VINS/OpenVINS; those packages are not installed.
Use it to accept/reject cuVSLAM vs RTAB stereo (or a future adapter that
publishes the same /visual_slam/tracking/odometry columns via pose_logger).
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("--label", default="candidate")
    parser.add_argument(
        "--static-gate-m", type=float, default=0.05,
        help="Max XY drift in any 60 s window")
    parser.add_argument(
        "--lift-gate-m", type=float, default=0.10,
        help="Max high-altitude XY span after 0.8 m AGL")
    args = parser.parse_args()

    analyzer = Path(__file__).resolve().parent / "analyze_flight_experiment.py"
    raw = subprocess.check_output(
        [sys.executable, str(analyzer), str(args.csv), "--json"],
        text=True)
    result = json.loads(raw)
    metrics = result["metrics"]

    static_xy = metrics.get("vio_xy_max_60s_m")
    lift_xy = metrics.get("high_altitude_vio_xy_span_m")
    fused_lift_xy = metrics.get("high_altitude_ekf_xy_span_m")
    powered = bool(metrics.get("powered_flight_evidence", False))
    static_pass = (
        isinstance(static_xy, (int, float))
        and static_xy == static_xy
        and static_xy <= args.static_gate_m)
    lift_pass = (
        lift_xy is None
        or (isinstance(lift_xy, (int, float)) and lift_xy != lift_xy)
        or (isinstance(lift_xy, (int, float)) and lift_xy <= args.lift_gate_m))
    fused_lift_pass = (
        isinstance(fused_lift_xy, (int, float))
        and fused_lift_xy == fused_lift_xy
        and fused_lift_xy <= args.lift_gate_m)

    report = {
        "label": args.label,
        "file": str(args.csv),
        "static_xy_max_60s_m": static_xy,
        "lift_xy_span_m": lift_xy,
        "fused_lift_xy_span_m": fused_lift_xy,
        "powered_flight_evidence": powered,
        "static_pass": static_pass,
        "lift_pass": lift_pass,
        # A hand-held lift can score raw VIO, but cannot certify EKF/control.
        "fused_lift_pass": fused_lift_pass,
        "accepted": bool(static_pass and lift_pass and powered and fused_lift_pass),
        "classification": result.get("classification", []),
        "next_step": (
            "keep current VIO and proceed to a guarded powered hover"
            if static_pass and lift_pass and powered and fused_lift_pass
            else (
                "repeat with motors/setpoints active; this hand-held record "
                "cannot certify Offboard hover"
                if static_pass and lift_pass and not powered
                else "inspect PX4 EV aid-source innovations, frame alignment, "
                     "delay and control before changing VIO"
            )
        ),
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
