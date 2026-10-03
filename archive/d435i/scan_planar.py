#!/usr/bin/env python3
"""
scan_planar — IMU-flatten 2D LaserScan to the horizontal plane.

Problem: RPLidar A2M8 scan plane is fixed to the drone body. When the drone
pitches or rolls, the scan plane tilts, causing geometric distortion in
scan matching (walls appear curved, distances are skewed).

Solution: Use PX4's fused attitude (/fmu/out/vehicle_attitude) to de-rotate
each scan point back to the horizontal plane. Yaw is preserved.

When no PX4 attitude is available, scans are dropped by default. Ground
mapping may explicitly set allow_uncompensated:=true while keeping the
lidar level.

Subscribes:
  /scan                          (sensor_msgs/LaserScan)
  /fmu/out/vehicle_attitude      (px4_msgs/VehicleAttitude) [BEST_EFFORT]

Publishes:
  /scan_planar                   (sensor_msgs/LaserScan) — flattened scan

Usage:
  ros2 run px4_interface scan_planar.py
"""

import math
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import VehicleAttitude
from std_msgs.msg import String


class ScanPlanar(Node):
    def __init__(self):
        super().__init__('scan_planar')

        self.declare_parameter('max_attitude_age_ms', 150.0)
        self.declare_parameter('max_tilt_deg', 20.0)
        self.declare_parameter('min_tilt_deg', 1.0)
        self.declare_parameter('attitude_buffer_size', 100)
        self.declare_parameter('allow_uncompensated', False)

        self._max_attitude_age_s = (
            float(self.get_parameter('max_attitude_age_ms').value) * 1e-3)
        self._max_tilt_rad = math.radians(
            float(self.get_parameter('max_tilt_deg').value))
        self._min_tilt_rad = math.radians(
            float(self.get_parameter('min_tilt_deg').value))
        buffer_size = max(
            2, int(self.get_parameter('attitude_buffer_size').value))
        self._allow_uncompensated = bool(
            self.get_parameter('allow_uncompensated').value)

        # PX4 uXRCE-DDS uses BEST_EFFORT QoS
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        self._scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self._att_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self._att_cb, px4_qos)

        self._planar_pub = self.create_publisher(
            LaserScan, '/scan_planar', qos_profile_sensor_data)
        self._status_pub = self.create_publisher(
            String, '/scan_planar/status', 10)

        # PX4 timestamps are boot-relative while LaserScan stamps are normally
        # Unix/ROS time. Buffer by ROS receive time so both streams share one
        # clock domain and choose the closest attitude for each scan callback.
        self._attitudes = deque(maxlen=buffer_size)
        self._last_status = None

        self.get_logger().info(
            'scan_planar started: /scan + PX4 attitude -> /scan_planar '
            f'(allow_uncompensated={self._allow_uncompensated})')

    def _att_cb(self, msg: VehicleAttitude):
        """PX4 NED attitude: convert quaternion to roll, pitch, yaw (radians)."""
        # VehicleAttitude.q is float[4]: [w, x, y, z]
        q = msg.q
        r = Rotation.from_quat([q[1], q[2], q[3], q[0]])  # scipy: [x, y, z, w]
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        self._attitudes.append((
            self.get_clock().now(),
            float(roll),
            float(pitch),
            float(yaw),
        ))

    def _publish_status(self, state, detail):
        text = f'state={state} {detail}'
        if text != self._last_status:
            msg = String()
            msg.data = text
            self._status_pub.publish(msg)
            self._last_status = text

    def _nearest_attitude(self, now):
        if not self._attitudes:
            return None, float('inf')
        sample = min(
            self._attitudes,
            key=lambda item: abs((now - item[0]).nanoseconds))
        age_s = abs((now - sample[0]).nanoseconds) * 1e-9
        return sample[1:], age_s

    def _scan_cb(self, msg: LaserScan):
        now = self.get_clock().now()
        attitude, age_s = self._nearest_attitude(now)
        if attitude is None or age_s > self._max_attitude_age_s:
            if self._allow_uncompensated:
                self._planar_pub.publish(msg)
                self._publish_status(
                    'degraded',
                    f'mode=uncompensated reason=attitude_stale '
                    f'age_ms={age_s * 1e3:.0f}')
                return
            self._publish_status(
                'invalid', f'reason=attitude_stale age_ms={age_s * 1e3:.0f}')
            self.get_logger().warn(
                'Dropping scan: PX4 attitude is unavailable or stale',
                throttle_duration_sec=2.0)
            return

        roll, pitch, yaw = attitude
        tilt = math.hypot(roll, pitch)
        if tilt > self._max_tilt_rad:
            self._publish_status(
                'invalid',
                f'reason=tilt_exceeded tilt_deg={math.degrees(tilt):.1f}')
            self.get_logger().warn(
                f'Dropping scan: tilt {math.degrees(tilt):.1f} deg exceeds '
                f'{math.degrees(self._max_tilt_rad):.1f} deg',
                throttle_duration_sec=2.0)
            return

        # Only flatten when drone is tilted (> 1 deg). At near-level attitude,
        # rotation introduces numerical noise that breaks scan matching.
        if tilt < self._min_tilt_rad:
            self._planar_pub.publish(msg)
            self._publish_status(
                'healthy',
                f'mode=passthrough age_ms={age_s * 1e3:.0f} '
                f'tilt_deg={math.degrees(tilt):.1f}')
            return

        # Rotation: body→world→yaw-only (vectorized, all points at once)
        R_b2w = Rotation.from_euler('ZYX', [yaw, pitch, roll]).as_matrix()
        R_yaw_only = Rotation.from_euler('z', yaw).as_matrix()
        # Combined: remove pitch/roll, keep yaw
        R_planar = R_yaw_only.T @ R_b2w  # (3,3)

        n = len(msg.ranges)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        ranges = np.array(msg.ranges, dtype=np.float32)

        # Valid mask (skip inf/NaN/out-of-range before computation)
        valid = (ranges >= msg.range_min) & (ranges <= msg.range_max) & np.isfinite(ranges)

        # Fill invalid with a safe placeholder (max range) for matmul, then mask afterward
        safe_ranges = np.where(valid, ranges, msg.range_max)

        # Build body-frame points: [3, n], float32 for speed on ARM
        p_body = np.zeros((3, n), dtype=np.float32)
        p_body[0, :] = safe_ranges * np.cos(angles).astype(np.float32)
        p_body[1, :] = safe_ranges * np.sin(angles).astype(np.float32)

        # Apply rotation (float32)
        R_planar_f = R_planar.astype(np.float32)
        p_planar = R_planar_f @ p_body  # (3, n)
        projected_ranges = np.hypot(
            p_planar[0], p_planar[1]).astype(np.float32)
        projected_angles = np.arctan2(
            p_planar[1], p_planar[0]).astype(np.float32)

        # A 3-D tilt correction changes both range and bearing. Re-bin each
        # projected point onto the original LaserScan angular grid, keeping the
        # nearest obstacle in bins that receive multiple points.
        new_ranges = np.full(n, np.inf, dtype=np.float32)
        projected_bins = np.rint(
            (projected_angles - msg.angle_min) / msg.angle_increment
        ).astype(np.int64)
        projected_valid = (
            valid
            & np.isfinite(projected_ranges)
            & (projected_ranges >= msg.range_min)
            & (projected_ranges <= msg.range_max)
            & (projected_bins >= 0)
            & (projected_bins < n)
        )
        np.minimum.at(
            new_ranges,
            projected_bins[projected_valid],
            projected_ranges[projected_valid])

        planar = LaserScan()
        planar.header = msg.header
        planar.header.frame_id = msg.header.frame_id
        planar.angle_min = msg.angle_min
        planar.angle_max = msg.angle_max
        planar.angle_increment = msg.angle_increment
        planar.time_increment = msg.time_increment
        planar.scan_time = msg.scan_time
        planar.range_min = msg.range_min
        planar.range_max = msg.range_max
        planar.ranges = new_ranges.tolist()

        self._planar_pub.publish(planar)
        self._publish_status(
            'healthy',
            f'mode=projected age_ms={age_s * 1e3:.0f} '
            f'tilt_deg={math.degrees(tilt):.1f}')


def main(args=None):
    rclpy.init(args=args)
    node = ScanPlanar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
