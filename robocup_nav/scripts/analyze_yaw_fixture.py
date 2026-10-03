#!/usr/bin/env python3
"""Analyze a fixture yaw bag: peak yaw vs XY/Z coupling. Read-only, no publish."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_deg(delta):
    return (delta + 180.0) % 360.0 - 180.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument('--target-yaw-deg', type=float, default=90.0)
    parser.add_argument('--yaw-tol-deg', type=float, default=5.0)
    parser.add_argument('--xy-tol-m', type=float, default=0.05)
    parser.add_argument('--z-tol-m', type=float, default=0.03)
    args = parser.parse_args()
    bag = Path(args.bag)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {x.name: get_message(x.type) for x in reader.get_all_topics_and_types()}
    poses = []
    while reader.has_next():
        topic, data, _received = reader.read_next()
        if topic != '/visual_slam/tracking/odometry':
            continue
        msg = deserialize_message(data, types[topic])
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        poses.append([t, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    if len(poses) < 10:
        raise RuntimeError('not enough VIO poses')
    p = np.asarray(poses)
    q = p[:, 4:8]
    q = q / np.linalg.norm(q, axis=1)[:, None]
    yaw0 = yaw_from_quat(*q[0])
    yaw = np.array([wrap_deg(math.degrees(yaw_from_quat(*row) - yaw0)) for row in q])
    xy = np.hypot(p[:, 1] - p[0, 1], p[:, 2] - p[0, 2])
    z = p[:, 3] - p[0, 3]
    peak_idx = int(np.argmax(np.abs(yaw)))
    peak_yaw = float(yaw[peak_idx])
    geodesic = np.degrees(2 * np.arccos(np.clip(np.abs(q @ q[0]), 0, 1)))
    yaw_error = abs(abs(peak_yaw) - args.target_yaw_deg)
    z_ptp = float(np.ptp(p[:, 3]))
    report = {
        'flight_validation': False,
        'test': 'fixture_yaw_isolation',
        'input_bag': str(bag),
        'span_s': float(p[-1, 0] - p[0, 0]),
        'vio_samples': len(p),
        'peak_yaw_deg': peak_yaw,
        'peak_yaw_abs_deg': abs(peak_yaw),
        'peak_yaw_error_deg': yaw_error,
        'endpoint_yaw_deg': float(yaw[-1]),
        'max_geodesic_orientation_deg': float(geodesic.max()),
        'max_xy_from_start_m': float(xy.max()),
        'endpoint_xy_m': float(xy[-1]),
        'z_peak_to_peak_m': z_ptp,
        'endpoint_z_m': float(z[-1]),
        'position_axis_peak_to_peak_m': np.ptp(p[:, 1:4], axis=0).tolist(),
        'max_displacement_m': float(np.linalg.norm(p[:, 1:4] - p[0, 1:4], axis=1).max()),
        'endpoint_displacement_m': float(np.linalg.norm(p[-1, 1:4] - p[0, 1:4])),
        'gates': {
            'yaw_within_tol': yaw_error <= args.yaw_tol_deg,
            'xy_isolated': float(xy.max()) <= args.xy_tol_m,
            'z_isolated': z_ptp <= args.z_tol_m,
            'endpoint_xy_ok': float(xy[-1]) <= args.xy_tol_m,
            'endpoint_yaw_ok': abs(float(yaw[-1])) <= args.yaw_tol_deg,
        },
    }
    g = report['gates']
    report['angle_scale_pass'] = g['yaw_within_tol']
    report['pure_rotation_isolated'] = g['xy_isolated'] and g['z_isolated']
    report['return_pass'] = g['endpoint_xy_ok'] and g['endpoint_yaw_ok']
    report['passed'] = (
        report['angle_scale_pass']
        and report['pure_rotation_isolated']
        and report['return_pass']
    )
    output = bag.parent / 'yaw_fixture_analysis.json'
    if output.exists():
        raise FileExistsError(output)
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
