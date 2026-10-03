#!/usr/bin/env python3
"""Prop-off production-candidate EV publisher.

Publishes only `/fmu/in/vehicle_visual_odometry` using the bench contract in
`ev_odometry.py`. Requires `--prop-off`, refuses `live_flight_enabled`, never
writes PX4 parameters, never arms, and never publishes commands or setpoints.

Camera/VIO must already be running (visual-only cuVSLAM). On the host this
expects `/robocup/alignment/tracking` from `alignment_camera.py`. This node
does not start RealSense or the old `vslam_odom_bridge`. Keep MicroXRCEAgent
running.
"""
from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import yaml
from ev_adapter import EvAdapter

ROOT = Path(__file__).resolve().parents[1]
os.environ['ROS_DOMAIN_ID'] = '0'
os.environ['ROS_LOCALHOST_ONLY'] = '0'
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = '/home/cfly/ros2_ws/config/fastdds_bridge.xml'

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import EstimatorStatusFlags, VehicleLocalPosition, VehicleOdometry, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String

from ev_odometry import (
    EV_TOPIC,
    apply_to_vehicle_odometry,
    extra_flight_inputs,
)


def _load_platform():
    return yaml.safe_load((ROOT / 'config' / 'platform.yaml').read_text())


class EvBridge(Node):
    def __init__(self, duration_s: int, allow_ev_hgt: bool = False):
        super().__init__('robocup_ev_bridge', enable_rosout=False, start_parameter_services=False)
        self.duration_s = duration_s
        self.allow_ev_hgt = allow_ev_hgt
        self.end = time.monotonic() + duration_s
        self.adapter = EvAdapter(time.monotonic(), allow_ev_hgt=bool(allow_ev_hgt))
        self.last_audit = 0.0
        self.done = False
        self.flags = {}
        self.flag_samples = {
            'cs_ev_pos': 0, 'cs_ev_yaw': 0, 'cs_ev_hgt': 0, 'cs_ev_vel': 0,
            'xy_valid': 0, 'heading_good_for_control': 0,
        }
        self.xy_valid = None
        self.heading_good_for_control = None
        ev_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE,
                            history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(VehicleOdometry, EV_TOPIC, ev_qos)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry',
                                 self.on_vio, qos_profile_sensor_data)
        self.create_subscription(String, '/robocup/alignment/tracking',
                                 self.on_tracking, 10)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.on_vehicle, qos_profile_sensor_data)
        self.create_subscription(EstimatorStatusFlags, '/fmu/out/estimator_status_flags',
                                 self.on_flags, qos_profile_sensor_data)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                 self.on_local, qos_profile_sensor_data)
        self.create_timer(0.05, self.watchdog)

    @property
    def fault(self):
        return self.adapter.fault

    @property
    def inhibit(self):
        return self.adapter.inhibit

    @property
    def preview_count(self):
        return self.adapter.preview_count

    @property
    def publish_count(self):
        return self.adapter.publish_count

    @property
    def quality_counts(self):
        return self.adapter.quality_counts

    def _stop(self, reason: str, complete: bool = False):
        self.adapter.trip('duration_complete' if complete else reason)
        self.done = True
        if self.pub is not None:
            self.destroy_publisher(self.pub)
            self.pub = None

    def _audit(self):
        counts = {}
        for topic, _ in self.get_topic_names_and_types():
            if topic.startswith('/fmu/in/'):
                counts[topic] = self.count_publishers(topic)
        own = 1 if self.pub is not None else 0
        extras = extra_flight_inputs(counts, own_ev_publishers=own)
        if not self.adapter.extra_inputs(extras):
            self._stop(self.adapter.fault)

    def on_tracking(self, msg):
        try:
            data = json.loads(msg.data)
            self.adapter.on_tracking(int(data['vo_state']), int(data['stamp_ns']), time.monotonic())
        except (ValueError, TypeError, KeyError):
            self._stop('Invalid tracking status')
            return
        if self.adapter.inhibit:
            self._stop(self.adapter.fault)

    def on_vehicle(self, msg):
        self.adapter.on_vehicle(int(msg.arming_state), time.monotonic())
        if self.adapter.inhibit:
            self._stop(self.adapter.fault)

    def on_flags(self, msg):
        self.flags = {
            'cs_ev_pos': bool(msg.cs_ev_pos),
            'cs_ev_yaw': bool(msg.cs_ev_yaw),
            'cs_ev_hgt': bool(msg.cs_ev_hgt),
            'cs_ev_vel': bool(msg.cs_ev_vel),
        }
        for key in ('cs_ev_pos', 'cs_ev_yaw', 'cs_ev_hgt', 'cs_ev_vel'):
            if self.flags[key]:
                self.flag_samples[key] += 1
        self.adapter.on_flags(bool(msg.cs_ev_hgt), bool(msg.cs_ev_vel), time.monotonic())
        if self.adapter.inhibit:
            self._stop(self.adapter.fault)

    def on_local(self, msg):
        self.xy_valid = bool(msg.xy_valid)
        self.heading_good_for_control = bool(msg.heading_good_for_control)
        if self.xy_valid:
            self.flag_samples['xy_valid'] += 1
        if self.heading_good_for_control:
            self.flag_samples['heading_good_for_control'] += 1

    def on_vio(self, msg):
        if self.adapter.inhibit or self.pub is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        age_ms = (now_ns - stamp) / 1e6
        pos, q = msg.pose.pose.position, msg.pose.pose.orientation
        frame_ok = msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_link'
        ev = self.adapter.on_pose(stamp, age_ms, time.monotonic(),
                                  [pos.x, pos.y, pos.z], [q.x, q.y, q.z, q.w],
                                  frame_ok=frame_ok, now_ns=now_ns)
        if self.adapter.inhibit:
            self._stop(self.adapter.fault)
            return
        if ev is None:
            return
        ev_msg = VehicleOdometry()
        apply_to_vehicle_odometry(ev_msg, ev)
        self.pub.publish(ev_msg)

    def watchdog(self):
        if self.done:
            return
        if not self.adapter.watchdog(time.monotonic()):
            self._stop(self.adapter.fault)
            return
        if time.monotonic() >= self.end:
            self._stop('duration_complete', complete=True)
            return
        if time.monotonic() - self.last_audit >= 0.5:
            self._audit()
            self.last_audit = time.monotonic()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--prop-off', action='store_true',
                        help='Required: confirm propellers are removed and the vehicle stays disarmed')
    parser.add_argument('--allow-ev-hgt', action='store_true',
                        help='Do not stop when cs_ev_hgt is true (EV height fusion expected)')
    parser.add_argument('--duration', type=int, default=60)
    args = parser.parse_args()
    if not args.prop_off:
        parser.error('refusing to publish EV without --prop-off')
    if not 10 <= args.duration <= 900:
        parser.error('duration outside 10..900')
    platform = _load_platform()
    if platform.get('live_flight_enabled'):
        raise RuntimeError('live_flight_enabled is true; ev_bridge remains prohibited')
    rclpy.init()
    node = EvBridge(args.duration, allow_ev_hgt=bool(args.allow_ev_hgt))
    try:
        node._audit()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.fault and node.fault != 'duration_complete':
                break
    except KeyboardInterrupt:
        node._stop('operator_interrupt')
    finally:
        report = {
            'flight_validation': False,
            'ev_publish_count': node.publish_count,
            'preview_count': node.preview_count,
            'fault': node.fault,
            'estimator_flags': node.flags,
            'ev_flag_samples': node.flag_samples,
            'xy_valid': node.xy_valid,
            'heading_good_for_control': node.heading_good_for_control,
            'quality_counts': node.quality_counts,
            'note': 'Prop-off candidate EV publisher; quality from tracking. Not flight approval.',
        }
        print(json.dumps(report, indent=2))
        if node.pub is not None:
            node.destroy_publisher(node.pub)
        node.destroy_node()
        rclpy.try_shutdown()
    ok = node.fault in (None, 'duration_complete') and node.publish_count > 0
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
