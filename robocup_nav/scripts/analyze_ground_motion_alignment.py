#!/usr/bin/env python3
"""Read-only direction/scale check from one resident EV session's pairs.

Input is the service report.json written by resident_ev_service.py while the
operator slides the DISARMED aircraft on the ground: forward, then left, then
a yaw turn. Pairs exist only while disarmed, landed, xy/z valid and on one
reset generation, so this never covers airborne motion.

PX4 fuses this same EV, so VIO/PX4 agreement alone only shows consistent
acceptance. Direction is judged against the operator-declared motion in the
takeoff FRD frame (forward = +x, left = -y). Output never sets review flags.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from alignment_math import LocalFRD

MIN_EXCURSION_M = .30
MAX_CROSS_RATIO = .35
MAX_SCALE_ERROR = .10
MIN_YAW_DEG = 20.
MAX_YAW_ERROR_DEG = 5.


def wrap(angle):
    return (angle+math.pi) % (2*math.pi)-math.pi


def analyze(pairs):
    if len(pairs) < 50:
        raise ValueError('need at least 50 same-session pairs')
    resets = {tuple(p['reset_counters']) for p in pairs}
    if len(resets) != 1:
        raise ValueError('PX4 reset generation changed during capture')
    first = pairs[0]
    frame = LocalFRD(first['vio_position'], first['vio_quaternion'])
    px4_origin = np.asarray(first['px4_position'], dtype=float)
    heading0 = float(first['px4_heading'])
    vio, px4, yaw_vio, yaw_px4 = [], [], [], []
    for p in pairs:
        position, q_wxyz = frame.convert(p['vio_position'], p['vio_quaternion'])
        vio.append(position)
        px4.append(np.asarray(p['px4_position'], dtype=float)-px4_origin)
        w, x, y, z = q_wxyz
        yaw_vio.append(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
        yaw_px4.append(wrap(float(p['px4_heading'])-heading0))
    vio, px4 = np.asarray(vio), np.asarray(px4)
    yaw_vio = np.degrees(np.unwrap(yaw_vio))
    yaw_px4 = np.degrees(np.unwrap(yaw_px4))

    checks = {}
    i_fwd = int(np.argmax(px4[:, 0]))
    forward, cross_fwd = px4[i_fwd, 0], abs(px4[i_fwd, 1])
    checks['forward_is_plus_x'] = bool(forward >= MIN_EXCURSION_M
                                       and cross_fwd <= MAX_CROSS_RATIO*forward)
    i_left = int(np.argmin(px4[:, 1]))
    left, cross_left = -px4[i_left, 1], abs(px4[i_left, 0]-px4[i_fwd, 0]) if i_left > i_fwd else None
    checks['left_is_minus_y'] = bool(left >= MIN_EXCURSION_M and i_left > i_fwd
                                     and cross_left <= MAX_CROSS_RATIO*left)
    moved = np.linalg.norm(vio[:, :2], axis=1) >= MIN_EXCURSION_M
    if moved.sum() < 10:
        scale = None
        checks['scale_within_10pct'] = False
    else:
        a, b = vio[moved, :2].ravel(), px4[moved, :2].ravel()
        scale = float(a @ b / (a @ a))
        checks['scale_within_10pct'] = abs(scale-1) <= MAX_SCALE_ERROR
    yaw_span = float(np.ptp(yaw_px4))
    yaw_error = float(np.max(np.abs(yaw_px4-yaw_vio)))
    checks['yaw_excited'] = yaw_span >= MIN_YAW_DEG
    checks['yaw_agrees'] = yaw_error <= MAX_YAW_ERROR_DEG
    return dict(
        pairs=len(pairs), reset_counters=list(next(iter(resets))),
        forward_m=float(forward), forward_cross_m=float(cross_fwd),
        left_m=float(left), left_cross_m=None if cross_left is None else float(cross_left),
        horizontal_scale_px4_over_vio=scale,
        yaw_span_deg=yaw_span, max_yaw_error_deg=yaw_error,
        max_pair_difference_ms=float(max(p['difference_ms'] for p in pairs)),
        checks=checks, passed=all(checks.values()),
        scope='ground-sliding direction/scale/yaw only; no vertical, airborne or landing accuracy',
        sets_review_flags=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('service_report', type=Path)
    args = parser.parse_args()
    report = json.loads(args.service_report.read_text())
    result = analyze(report['observation']['pairs'])
    result['service_report'] = str(args.service_report)
    result['session'] = report.get('session')
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
