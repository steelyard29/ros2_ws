#!/usr/bin/env python3
"""Read-only relative attitude check for an alignment-preview JSONL capture."""
from __future__ import annotations

import argparse
import bisect
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


AXES = ("roll", "pitch", "yaw")


def rpy_wxyz(quaternion):
    q = np.asarray(quaternion, dtype=float)
    return Rotation.from_quat(q[[1, 2, 3, 0]]).as_euler("xyz", degrees=False)


def unwrap_degrees(values):
    return np.degrees(np.unwrap(np.asarray(values), axis=0))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("samples")
    parser.add_argument("--required-axis", choices=AXES, default="yaw")
    parser.add_argument("--max-pair-ms", type=float, default=100.0)
    args = parser.parse_args()

    samples = Path(args.samples)
    attitudes = []
    previews = []
    with samples.open() as stream:
        for line in stream:
            item = json.loads(line)
            topic = item.get("topic")
            msg = item.get("message", {})
            if topic == "/fmu/out/vehicle_attitude":
                attitudes.append((int(msg["timestamp_sample"]), msg["q"]))
            elif topic == "/robocup/alignment/visual_odometry_preview":
                previews.append((int(msg["timestamp_sample"]), msg["q"]))
    if len(attitudes) < 20 or len(previews) < 20:
        raise RuntimeError("insufficient PX4 attitude or VIO preview samples")

    attitude_stamps = [row[0] for row in attitudes]
    pairs = []
    for stamp, preview_q in previews:
        index = bisect.bisect_left(attitude_stamps, stamp)
        candidates = [i for i in (index - 1, index) if 0 <= i < len(attitudes)]
        nearest = min(candidates, key=lambda i: abs(attitude_stamps[i] - stamp))
        px4_stamp, px4_q = attitudes[nearest]
        dt_ms = abs(px4_stamp - stamp) / 1000.0
        if dt_ms <= args.max_pair_ms:
            pairs.append((stamp, dt_ms, rpy_wxyz(preview_q), rpy_wxyz(px4_q)))
    if len(pairs) < 20:
        raise RuntimeError("insufficient timestamp-paired attitude samples")

    preview = unwrap_degrees([row[2] for row in pairs])
    px4 = unwrap_degrees([row[3] for row in pairs])
    # A constant initial yaw offset is expected. Compare motion after removing
    # each stream's median over the first second of paired data.
    stamps = np.asarray([row[0] for row in pairs])
    initial = stamps <= stamps[0] + 1_000_000
    if initial.sum() < 3:
        initial = np.arange(len(stamps)) < min(20, len(stamps))
    preview_delta = preview - np.median(preview[initial], axis=0)
    px4_delta = px4 - np.median(px4[initial], axis=0)

    metrics = {}
    for index, name in enumerate(AXES):
        a = preview_delta[:, index]
        b = px4_delta[:, index]
        preview_span = float(np.ptp(a))
        px4_span = float(np.ptp(b))
        excited = min(preview_span, px4_span) >= 10.0
        correlation = None
        slope = None
        offset = None
        rmse = None
        if excited and np.std(a) > 1e-6 and np.std(b) > 1e-6:
            correlation = float(np.corrcoef(a, b)[0, 1])
            slope, offset = (float(v) for v in np.polyfit(a, b, 1))
            rmse = float(math.sqrt(np.mean((b - (slope * a + offset)) ** 2)))
        metrics[name] = {
            "preview_span_deg": preview_span,
            "px4_span_deg": px4_span,
            "excited": excited,
            "correlation": correlation,
            "slope_px4_per_preview": slope,
            "fit_offset_deg": offset,
            "fit_rmse_deg": rmse,
        }

    selected = metrics[args.required_axis]
    required_axis_pass = bool(
        selected["excited"]
        and selected["correlation"] is not None
        and selected["correlation"] >= 0.95
        and 0.85 <= selected["slope_px4_per_preview"] <= 1.15
        and selected["fit_rmse_deg"] <= 3.0
    )
    report = {
        "test": "alignment_relative_attitude_motion",
        "flight_validation": False,
        "input_samples": str(samples),
        "paired_samples": len(pairs),
        "pair_dt_ms_p50_p95_max": np.percentile(
            [row[1] for row in pairs], [50, 95, 100]
        ).tolist(),
        "required_axis": args.required_axis,
        "metrics": metrics,
        "required_axis_pass": required_axis_pass,
        "alignment_accepted": False,
        "note": (
            "Relative-motion preview check only; no EV injection, covariance "
            "calibration, parameter write, or flight approval."
        ),
    }
    output = samples.parent / "motion_analysis.json"
    if output.exists():
        raise FileExistsError(output)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if required_axis_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
