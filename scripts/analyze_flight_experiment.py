#!/usr/bin/env python3
"""Classify indoor-flight drift from a pose_logger CSV without third-party libs."""

import argparse
import csv
import json
import math
from pathlib import Path


def number(row, key):
    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return math.nan


def finite(values):
    return [value for value in values if math.isfinite(value)]


def span(values):
    values = finite(values)
    return max(values) - min(values) if values else math.nan


def distance(row, x_key, y_key, origin):
    x, y = number(row, x_key), number(row, y_key)
    if not (math.isfinite(x) and math.isfinite(y)):
        return math.nan
    return math.hypot(x - origin[0], y - origin[1])


def first_xy(rows, x_key, y_key):
    for row in rows:
        x, y = number(row, x_key), number(row, y_key)
        if math.isfinite(x) and math.isfinite(y):
            return x, y
    return math.nan, math.nan


def first_crossing(rows, height):
    for index, row in enumerate(rows):
        z = number(row, "z")
        dist_bottom = number(row, "dist_bottom")
        altitude = dist_bottom if math.isfinite(dist_bottom) else z
        if math.isfinite(altitude) and altitude >= height:
            return index
    return None


def max_window_displacement(rows, x_key, y_key, window_s=60.0):
    samples = []
    for row in rows:
        stamp = number(row, "timestamp")
        x, y = number(row, x_key), number(row, y_key)
        if math.isfinite(stamp) and math.isfinite(x) and math.isfinite(y):
            samples.append((stamp, x, y))
    if len(samples) < 2:
        return math.nan
    start = 0
    maximum = 0.0
    for end, (stamp, x, y) in enumerate(samples):
        while start < end and stamp - samples[start][0] > window_s:
            start += 1
        # Compare all samples in this short diagnostic window. At 20 Hz this
        # is small enough and catches non-monotonic drift better than endpoints.
        for _, old_x, old_y in samples[start:end]:
            maximum = max(maximum, math.hypot(x - old_x, y - old_y))
    return maximum


def bool_ratio(rows, key, expected="1"):
    values = [row.get(key, "") for row in rows if row.get(key, "") != ""]
    return (
        sum(value == expected for value in values) / len(values)
        if values else math.nan
    )


def finite_count(rows, key):
    return sum(math.isfinite(number(row, key)) for row in rows)


def max_difference(rows, ax, ay, bx, by):
    differences = []
    for row in rows:
        a_x, a_y = number(row, ax), number(row, ay)
        b_x, b_y = number(row, bx), number(row, by)
        if all(math.isfinite(value) for value in (a_x, a_y, b_x, b_y)):
            differences.append(math.hypot(a_x - b_x, a_y - b_y))
    return max(differences, default=math.nan)


def summarize(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("CSV contains no samples")

    required = {"timestamp", "x", "y", "z", "vslam_x", "vslam_y", "ev_n", "ev_e"}
    missing = sorted(required - set(rows[0]))
    if missing:
        raise ValueError(f"missing columns: {', '.join(missing)}")

    ekf_origin = first_xy(rows, "x", "y")
    vio_origin = first_xy(rows, "vslam_x", "vslam_y")
    ev_origin = first_xy(rows, "ev_n", "ev_e")
    rtab_origin = first_xy(rows, "rtabmap_x", "rtabmap_y")
    laser_origin = first_xy(rows, "laser_x", "laser_y")

    ekf_drift = [distance(r, "x", "y", ekf_origin) for r in rows]
    vio_drift = [distance(r, "vslam_x", "vslam_y", vio_origin) for r in rows]
    ev_drift = [distance(r, "ev_n", "ev_e", ev_origin) for r in rows]
    rtab_drift = [distance(r, "rtabmap_x", "rtabmap_y", rtab_origin) for r in rows]
    laser_drift = [distance(r, "laser_x", "laser_y", laser_origin) for r in rows]

    crossings = {
        str(height): first_crossing(rows, height)
        for height in (0.3, 0.6, 0.8, 0.9, 1.0, 1.2)
    }
    high_start = crossings["0.8"]
    high_rows = rows[high_start:] if high_start is not None else []
    motor_samples = finite_count(rows, "motor_m1")
    max_motor = max(
        (number(row, key) for row in rows for key in
         ("motor_m1", "motor_m2", "motor_m3", "motor_m4")
         if math.isfinite(number(row, key))),
        default=math.nan,
    )
    setpoint_samples = finite_count(rows, "setpoint_n")
    armed_samples = sum(number(row, "arming_state") == 2 for row in rows)
    offboard_samples = sum(number(row, "nav_state") == 14 for row in rows)
    powered = (
        (math.isfinite(max_motor) and max_motor > 0.05)
        or (armed_samples > 0 and (setpoint_samples > 0 or offboard_samples > 0))
    )

    metrics = {
        "samples": len(rows),
        "duration_s": number(rows[-1], "timestamp") - number(rows[0], "timestamp"),
        "ekf_xy_max_m": max(finite(ekf_drift), default=math.nan),
        "vio_xy_max_m": max(finite(vio_drift), default=math.nan),
        "ev_xy_max_m": max(finite(ev_drift), default=math.nan),
        "rtab_xy_max_m": max(finite(rtab_drift), default=math.nan),
        "laser_xy_max_m": max(finite(laser_drift), default=math.nan),
        "ekf_xy_max_60s_m": max_window_displacement(rows, "x", "y"),
        "vio_xy_max_60s_m": max_window_displacement(
            rows, "vslam_x", "vslam_y"),
        "ev_xy_max_60s_m": max_window_displacement(rows, "ev_n", "ev_e"),
        "laser_xy_max_60s_m": max_window_displacement(
            rows, "laser_x", "laser_y"),
        "high_altitude_ekf_xy_span_m": max(
            span(number(r, key) for r in high_rows) for key in ("x", "y")
        ) if high_rows else math.nan,
        "high_altitude_vio_xy_span_m": max(
            span(number(r, key) for r in high_rows)
            for key in ("vslam_x", "vslam_y")
        ) if high_rows else math.nan,
        "ev_fusion_ratio": bool_ratio(rows, "cs_ev_pos"),
        "dead_reckoning_ratio": bool_ratio(rows, "cs_inertial_dead_reckoning"),
        "range_height_ratio": bool_ratio(rows, "cs_rng_hgt"),
        "ev_height_ratio": bool_ratio(rows, "cs_ev_hgt"),
        "setpoint_xy_error_max_m": max(
            finite(number(r, "setpoint_xy_error_m") for r in rows),
            default=math.nan,
        ),
        "timesync_offset_span_us": span(
            number(r, "timesync_offset_us") for r in rows
        ),
        "local_vs_ev_xy_max_m": max_difference(
            rows, "local_n", "local_e", "ev_n", "ev_e"),
        "high_altitude_local_vs_ev_xy_max_m": max_difference(
            high_rows, "local_n", "local_e", "ev_n", "ev_e"),
        "motor_samples": motor_samples,
        "max_motor_output": max_motor,
        "setpoint_samples": setpoint_samples,
        "armed_samples": armed_samples,
        "offboard_samples": offboard_samples,
        "powered_flight_evidence": powered,
        "crossing_sample_indices": crossings,
    }

    vio_high = metrics["high_altitude_vio_xy_span_m"]
    ekf_high = metrics["high_altitude_ekf_xy_span_m"]
    ev_ratio = metrics["ev_fusion_ratio"]
    control_error = metrics["setpoint_xy_error_max_m"]
    labels = []
    if math.isfinite(vio_high) and vio_high > 0.10:
        labels.append("A: raw visual odometry moves during the vertical lift")
    if math.isfinite(ev_ratio) and ev_ratio < 0.90:
        labels.append("B: PX4 external-vision fusion is intermittent")
    if (
        math.isfinite(metrics["vio_xy_max_m"])
        and math.isfinite(metrics["ev_xy_max_m"])
        and abs(metrics["vio_xy_max_m"] - metrics["ev_xy_max_m"]) > 0.20
    ):
        labels.append("C: VIO-to-EV frame/origin conversion requires inspection")
    if metrics["ev_height_ratio"] > 0.05 and metrics["range_height_ratio"] > 0.50:
        labels.append("D: EV and range height are fused concurrently")
    if math.isfinite(control_error) and control_error > 0.20:
        labels.append("E: setpoint tracking error exceeds 20 cm")
    local_ev_high = metrics["high_altitude_local_vs_ev_xy_max_m"]
    raw_stable = (math.isfinite(vio_high) and vio_high <= 0.10)
    if (math.isfinite(local_ev_high) and local_ev_high > 0.10 and raw_stable):
        if powered:
            labels.append(
                "B/C: PX4 local position diverges from stable EV during "
                "powered flight; inspect EV aid-source innovations, frame "
                "alignment and delay")
        else:
            labels.append(
                "INCONCLUSIVE: PX4 local position diverges from stable EV, "
                "but this recording has no powered-flight evidence")
    if not powered:
        labels.append(
            "NOTE: no active motor output; this recording cannot validate "
            "Offboard hover/control")
    if (math.isfinite(metrics["timesync_offset_span_us"])
            and metrics["timesync_offset_span_us"] > 5000.0):
        labels.append(
            "TIMESTAMP RISK: timesync offset changed by more than 5 ms; "
            "verify PX4 ULog/EV_DELAY before tuning estimator noise")
    if not labels and math.isfinite(ekf_high) and ekf_high <= 0.10:
        labels.append("PASS: no >10 cm high-altitude XY drift in this recording")
    if not labels:
        labels.append("INCONCLUSIVE: inspect rosbag and PX4 ULog")

    return {"file": str(path), "metrics": metrics, "classification": labels}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    result = summarize(args.csv)
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=True))
        return
    print(f"File: {result['file']}")
    for key, value in result["metrics"].items():
        print(f"{key}: {value}")
    print("Classification:")
    for label in result["classification"]:
        print(f"- {label}")


if __name__ == "__main__":
    main()
