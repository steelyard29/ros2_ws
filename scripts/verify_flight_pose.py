#!/usr/bin/env python3
"""Offline acceptance checks for flight_pose / drift CSVs against the indoor plan.

Example:
  python3 scripts/verify_flight_pose.py \\
    /home/cfly/ros2_ws/logs/flight_pose_20260804_225335.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple


def _f(row: Dict[str, str], *keys: str, default: float = float('nan')) -> float:
    for key in keys:
        raw = row.get(key, '')
        if raw is None or raw == '':
            continue
        try:
            return float(raw)
        except ValueError:
            continue
    return default


def _load(path: Path) -> List[Dict[str, str]]:
    with path.open(newline='') as handle:
        return list(csv.DictReader(handle))


def _xy(row: Dict[str, str]) -> Tuple[float, float]:
    return _f(row, 'x', 'px4_x'), _f(row, 'y', 'px4_y')


def _analyze(rows: List[Dict[str, str]]) -> Dict[str, object]:
    if not rows:
        return {'error': 'empty csv'}

    t0 = _f(rows[0], 'timestamp', 'elapsed_s', default=0.0)
    use_elapsed = 'elapsed_s' in rows[0] and rows[0].get('elapsed_s') not in ('', None)

    samples = []
    for row in rows:
        if use_elapsed:
            t = _f(row, 'elapsed_s', default=0.0)
        else:
            t = _f(row, 'timestamp', default=0.0) - t0
        x, y = _xy(row)
        z = _f(row, 'z', 'px4_z')
        d = math.hypot(x, y) if math.isfinite(x) and math.isfinite(y) else float('nan')
        samples.append({
            't': t,
            'x': x, 'y': y, 'z': z, 'd': d,
            'vslam_x': _f(row, 'vslam_x'),
            'vslam_y': _f(row, 'vslam_y'),
            'rtab_x': _f(row, 'rtabmap_x'),
            'rtab_y': _f(row, 'rtabmap_y'),
            'loc_x': _f(row, 'localization_x'),
            'loc_y': _f(row, 'localization_y'),
            'loc_cov': max(
                _f(row, 'localization_cov_x', default=0.0),
                _f(row, 'localization_cov_y', default=0.0)),
            'cs_rng': _f(row, 'cs_rng_hgt'),
            'cs_ev_pos': _f(row, 'cs_ev_pos'),
            'ev_quality': _f(row, 'ev_quality'),
        })

    # Static window: first 15 s or until |z| climb > 0.05 from start.
    z0 = samples[0]['z']
    static = []
    for sample in samples:
        if sample['t'] > 60.0:
            break
        if math.isfinite(z0) and math.isfinite(sample['z']) and abs(sample['z'] - z0) > 0.08:
            break
        if sample['t'] <= 15.0 or abs(sample['z'] - z0) <= 0.08:
            static.append(sample)
        if sample['t'] > 15.0 and abs(sample['z'] - z0) <= 0.08:
            continue

    # Prefer explicit first 15 s for static if climb starts later.
    static15 = [s for s in samples if s['t'] <= 15.0]
    if static15:
        static = static15

    def span(vals: List[float]) -> float:
        finite = [v for v in vals if math.isfinite(v)]
        if not finite:
            return float('nan')
        return max(finite) - min(finite)

    static_xy = span([s['d'] for s in static]) if static else float('nan')
    # Also use axis ranges (plan: PX4 XY change <5 cm / 60 s).
    static_x_span = span([s['x'] for s in static])
    static_y_span = span([s['y'] for s in static])
    static_xy_span = math.hypot(
        static_x_span if math.isfinite(static_x_span) else 0.0,
        static_y_span if math.isfinite(static_y_span) else 0.0)

    climb = [
        s for s in samples
        if math.isfinite(s['z']) and math.isfinite(z0) and (s['z'] - z0) > 0.15
    ]
    if not climb:
        # NED logs may go more negative while climbing.
        climb = [
            s for s in samples
            if math.isfinite(s['z']) and math.isfinite(z0) and (z0 - s['z']) > 0.15
        ]

    climb_max_d = max((s['d'] for s in climb), default=float('nan'))
    climb_t_at_max = float('nan')
    if climb:
        worst = max(climb, key=lambda s: s['d'] if math.isfinite(s['d']) else -1.0)
        climb_t_at_max = worst['t']

    # Detect "false map lock": localization cov stays low while map XY slides with VIO.
    false_lock = False
    slide_m = 0.0
    if climb:
        c0 = climb[0]
        c1 = max(climb, key=lambda s: s['t'])
        v_slide = math.hypot(
            c1['vslam_x'] - c0['vslam_x'], c1['vslam_y'] - c0['vslam_y'])
        r_slide = math.hypot(
            c1['rtab_x'] - c0['rtab_x'], c1['rtab_y'] - c0['rtab_y'])
        loc_slide = math.hypot(
            c1['loc_x'] - c0['loc_x'], c1['loc_y'] - c0['loc_y'])
        covs = [s['loc_cov'] for s in climb if math.isfinite(s['loc_cov'])]
        median_cov = sorted(covs)[len(covs) // 2] if covs else float('nan')
        slide_m = max(v_slide, r_slide, loc_slide)
        if (
                math.isfinite(median_cov) and median_cov < 0.05
                and slide_m > 0.35
                and r_slide > 0.25 and v_slide > 0.25):
            false_lock = True

    rng_false = sum(
        1 for s in samples
        if math.isfinite(s['cs_rng']) and s['cs_rng'] < 0.5)
    total_flags = sum(1 for s in samples if math.isfinite(s['cs_rng']))
    rng_false_frac = (rng_false / total_flags) if total_flags else float('nan')

    checks = {
        'static_xy_span_m': static_xy_span,
        'static_pass': bool(math.isfinite(static_xy_span) and static_xy_span < 0.05),
        'climb_max_horiz_m': climb_max_d,
        'climb_pass': bool(math.isfinite(climb_max_d) and climb_max_d < 0.20),
        'false_map_lock': false_lock,
        'slide_during_climb_m': slide_m,
        'cs_rng_false_frac': rng_false_frac,
        'climb_t_at_max_d_s': climb_t_at_max,
        'n_samples': len(samples),
        'duration_s': samples[-1]['t'] if samples else 0.0,
    }
    checks['overall_pass'] = (
        checks['static_pass']
        and checks['climb_pass']
        and not false_lock
    )
    return checks


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_path', type=Path)
    args = parser.parse_args(argv)
    if not args.csv_path.is_file():
        print(f'ERROR: missing file {args.csv_path}', file=sys.stderr)
        return 2

    result = _analyze(_load(args.csv_path))
    if 'error' in result:
        print(f'ERROR: {result["error"]}', file=sys.stderr)
        return 2

    print(f'file: {args.csv_path}')
    print(f'samples: {result["n_samples"]}  duration: {result["duration_s"]:.1f}s')
    print(
        f'static XY span: {result["static_xy_span_m"]:.3f} m '
        f'({"PASS" if result["static_pass"] else "FAIL"} <0.05)')
    print(
        f'climb max horiz: {result["climb_max_horiz_m"]:.3f} m '
        f'@ t={result["climb_t_at_max_d_s"]:.1f}s '
        f'({"PASS" if result["climb_pass"] else "FAIL"} <0.20)')
    print(
        f'false map lock (low cov + VIO/RTAB slide): '
        f'{"YES" if result["false_map_lock"] else "no"} '
        f'(slide={result["slide_during_climb_m"]:.3f} m)')
    print(f'cs_rng_hgt false fraction: {result["cs_rng_false_frac"]:.3f}')
    print(f'overall: {"PASS" if result["overall_pass"] else "FAIL"}')
    if result['false_map_lock']:
        print(
            'NOTE: RTAB-Map was online but not geometrically locking; '
            'rebuild a production map that covers the takeoff point, then '
            'require proximity/loop before arming.')
    return 0 if result['overall_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
