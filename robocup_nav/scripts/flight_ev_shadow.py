#!/usr/bin/env python3
"""Independent candidate EV publisher. Output is hardcoded to shadow only.

No live switch is offered while flight supervision/physical acceptance is
incomplete. Normal arming does not stop this candidate after ground warmup.
It can run beside the bounded bench EV publisher to compare decisions.
"""
import argparse
import json
import os
from pathlib import Path
import time

from flight_ev_adapter import FlightEvAdapter
from ev_odometry import apply_to_vehicle_odometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=int, default=60)
    args = parser.parse_args()
    if not 10 <= args.duration <= 180:
        parser.error('duration must be 10..180 s')
    os.environ['ROS_DOMAIN_ID'] = '0'
    os.environ['ROS_LOCALHOST_ONLY'] = '0'
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = '/home/cfly/ros2_ws/config/fastdds_bridge.xml'
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data as qos
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import VehicleStatus, EstimatorStatusFlags, VehicleOdometry
    from std_msgs.msg import String
    rclpy.init()
    node = Node('robocup_flight_ev_shadow', enable_rosout=False, start_parameter_services=False)
    pub = node.create_publisher(VehicleOdometry, '/robocup/flight_shadow/vehicle_visual_odometry', qos)
    # Begin the guard at first VIO sample so a bounded camera startup can happen
    # alongside this observer. No output at all before that sample.
    adapter = None
    latest_status = latest_flags = latest_tracking = None
    actual_published = 0
    def status(m):
        nonlocal latest_status
        latest_status = (int(m.arming_state), time.monotonic())
        if adapter:
            adapter.on_vehicle(*latest_status)
    def flags(m):
        nonlocal latest_flags
        latest_flags = (bool(m.cs_ev_hgt), bool(m.cs_ev_vel), time.monotonic())
        if adapter:
            adapter.on_flags(*latest_flags)
    def tracking(m):
        nonlocal latest_tracking
        try:
            d = json.loads(m.data)
            latest_tracking = (int(d['vo_state']), int(d['stamp_ns']), time.monotonic())
            if adapter:
                adapter.on_tracking(*latest_tracking)
        except (ValueError, TypeError, KeyError):
            if adapter:
                adapter.trip('Invalid tracking message')
    def pose(m):
        nonlocal adapter, actual_published
        now = time.monotonic()
        if adapter is None:
            adapter = FlightEvAdapter(now)
            if latest_status:
                adapter.on_vehicle(*latest_status)
            if latest_flags:
                adapter.on_flags(*latest_flags)
            if latest_tracking:
                adapter.on_tracking(*latest_tracking)
        stamp = int(m.header.stamp.sec)*10**9 + int(m.header.stamp.nanosec)
        now_ns = node.get_clock().now().nanoseconds
        p, q = m.pose.pose.position, m.pose.pose.orientation
        result = adapter.on_pose(stamp, (now_ns-stamp)/1e6, now,
                                 (p.x, p.y, p.z), (q.x, q.y, q.z, q.w),
                                 frame_ok=m.header.frame_id == 'odom' and m.child_frame_id == 'base_link',
                                 now_ns=now_ns)
        if result:
            message = VehicleOdometry()
            apply_to_vehicle_odometry(message, result)
            pub.publish(message)
            actual_published += 1
    node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', status, qos)
    node.create_subscription(EstimatorStatusFlags, '/fmu/out/estimator_status_flags', flags, qos)
    node.create_subscription(String, '/robocup/alignment/tracking', tracking, 10)
    node.create_subscription(Odometry, '/visual_slam/tracking/odometry', pose, qos)
    started = time.monotonic()
    try:
        while time.monotonic()-started < args.duration:
            rclpy.spin_once(node, timeout_sec=.05)
            if adapter and not adapter.watchdog(time.monotonic()):
                break
    finally:
        report = {'flight_validation': False, 'shadow_only': True,
                  'real_ev_publish_count': 0, 'shadow_ev_publish_count': actual_published,
                  'fault': adapter.fault if adapter else 'No VIO received',
                  'quality_counts': adapter.quality_counts if adapter else {},
                  'armed_observed': adapter.armed if adapter else False}
        out = Path(__file__).resolve().parents[1]/'evidence'/time.strftime('flight_ev_shadow_%Y%m%d_%H%M%S')
        out.mkdir(parents=True, exist_ok=False)
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({'evidence': str(out), **report}, indent=2), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if report['fault'] is None and actual_published else 1


if __name__ == '__main__':
    raise SystemExit(main())
