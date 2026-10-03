#!/usr/bin/env python3
"""Read recorded raw VIO/IMU; summarize a stationary interval, never publish."""
import argparse
import json
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    args = parser.parse_args()
    bag = Path(args.bag)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {x.name: get_message(x.type) for x in reader.get_all_topics_and_types()}
    poses, imu = [], []
    while reader.has_next():
        topic, data, received = reader.read_next()
        if topic not in ('/visual_slam/tracking/odometry', '/camera/camera/imu'):
            continue
        msg = deserialize_message(data, types[topic])
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if topic.endswith('odometry'):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            poses.append([t, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        else:
            a, g = msg.linear_acceleration, msg.angular_velocity
            imu.append([t, a.x, a.y, a.z, g.x, g.y, g.z])
    report = {'flight_validation': False, 'input_bag': str(bag)}
    if not poses or not imu:
        raise RuntimeError('VIO or IMU is absent')
    p, m = np.asarray(poses), np.asarray(imu)
    start = max(p[0, 0], p[-1, 0]-60.)
    p, m = p[p[:, 0] >= start], m[m[:, 0] >= start]
    if not np.isfinite(p).all() or not np.isfinite(m).all() or not len(m):
        raise RuntimeError('Invalid samples')
    q = p[:, 4:8]
    q = q / np.linalg.norm(q, axis=1)[:, None]
    report.update({
        'evaluated_span_s': float(p[-1, 0]-p[0, 0]),
        'vio_samples': len(p), 'imu_samples': len(m),
        'max_displacement_m': float(np.linalg.norm(p[:, 1:4]-p[0, 1:4], axis=1).max()),
        'endpoint_displacement_m': float(np.linalg.norm(p[-1, 1:4]-p[0, 1:4])),
        'position_axis_peak_to_peak_m': np.ptp(p[:, 1:4], axis=0).tolist(),
        'max_orientation_change_deg': float(np.degrees(2*np.arccos(np.clip(np.abs(q @ q[0]), 0, 1))).max()),
        'acceleration_norm_mean_mps2': float(np.linalg.norm(m[:, 1:4], axis=1).mean()),
        'acceleration_norm_std_mps2': float(np.linalg.norm(m[:, 1:4], axis=1).std()),
        'gyro_mean_radps': m[:, 4:7].mean(axis=0).tolist(),
        'gyro_std_radps': m[:, 4:7].std(axis=0).tolist(),
    })
    output = bag.parent/'static_analysis.json'
    if output.exists():
        raise FileExistsError(output)
    output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
