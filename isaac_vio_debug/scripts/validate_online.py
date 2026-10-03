#!/usr/bin/env python3
"""Measure D435i/VIO header timestamps without storing image payloads."""

import argparse
import json
import math
import time
from collections import Counter

import rclpy
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu


def stamp_ns(message):
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class StreamStats:
    def __init__(self):
        self.count = 0
        self.first_ns = None
        self.last_ns = None
        self.max_gap_ns = 0
        self.non_monotonic = 0
        self.frames = set()
        self.dimensions = set()
        self.stamps = []

    def add(self, message, keep_stamps=False):
        current_ns = stamp_ns(message)
        if self.first_ns is None:
            self.first_ns = current_ns
        if self.last_ns is not None:
            delta = current_ns - self.last_ns
            if delta <= 0:
                self.non_monotonic += 1
            else:
                self.max_gap_ns = max(self.max_gap_ns, delta)
        self.last_ns = current_ns
        self.count += 1
        self.frames.add(message.header.frame_id)
        if hasattr(message, "width") and hasattr(message, "height"):
            self.dimensions.add((int(message.width), int(message.height)))
        if keep_stamps:
            self.stamps.append(current_ns)

    def result(self):
        span_s = 0.0
        rate_hz = 0.0
        if self.first_ns is not None and self.last_ns is not None:
            span_s = max(0.0, (self.last_ns - self.first_ns) / 1e9)
            if span_s > 0.0 and self.count > 1:
                rate_hz = (self.count - 1) / span_s
        return {
            "count": self.count,
            "header_span_s": round(span_s, 6),
            "header_rate_hz": round(rate_hz, 3),
            "max_header_gap_ms": round(self.max_gap_ns / 1e6, 3),
            "non_monotonic": self.non_monotonic,
            "frame_ids": sorted(self.frames),
            "dimensions": [list(item) for item in sorted(self.dimensions)],
        }


def stereo_pairing(left, right):
    left_stamps = sorted(left.stamps)
    right_stamps = sorted(right.stamps)
    exact = len(set(left_stamps).intersection(right_stamps))
    deltas = []
    j = 0
    for value in left_stamps:
        while j + 1 < len(right_stamps) and abs(right_stamps[j + 1] - value) <= abs(
            right_stamps[j] - value
        ):
            j += 1
        if right_stamps:
            deltas.append(abs(right_stamps[j] - value) / 1e6)
    return {
        "left_count": len(left_stamps),
        "right_count": len(right_stamps),
        "exact_stamp_matches": exact,
        "max_nearest_delta_ms": round(max(deltas), 6) if deltas else None,
        "mean_nearest_delta_ms": round(sum(deltas) / len(deltas), 6) if deltas else None,
    }


class Validator(Node):
    def __init__(self, duration):
        super().__init__("d435i_vio_online_validator")
        self.started = time.monotonic()
        self.duration = duration
        self.streams = {
            "infra1": StreamStats(),
            "infra2": StreamStats(),
            "camera_info1": StreamStats(),
            "camera_info2": StreamStats(),
            "imu": StreamStats(),
            "accel_raw": StreamStats(),
            "gyro_raw": StreamStats(),
            "odometry": StreamStats(),
            "status": StreamStats(),
        }
        self.vo_states = Counter()
        self.odom_child_frames = set()
        self.odom_initial_position = None
        self.odom_final_position = None
        self.odom_max_displacement_m = 0.0

        self.create_subscription(
            Image,
            "/camera/camera/infra1/image_rect_raw",
            lambda msg: self.streams["infra1"].add(msg, keep_stamps=True),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/camera/camera/infra2/image_rect_raw",
            lambda msg: self.streams["infra2"].add(msg, keep_stamps=True),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/camera/camera/infra1/camera_info",
            lambda msg: self.streams["camera_info1"].add(msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/camera/camera/infra2/camera_info",
            lambda msg: self.streams["camera_info2"].add(msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/camera/camera/imu",
            lambda msg: self.streams["imu"].add(msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/camera/camera/accel/sample",
            lambda msg: self.streams["accel_raw"].add(msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/camera/camera/gyro/sample",
            lambda msg: self.streams["gyro_raw"].add(msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/visual_slam/tracking/odometry",
            self.on_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VisualSlamStatus,
            "/visual_slam/status",
            self.on_status,
            qos_profile_sensor_data,
        )

    def on_odometry(self, message):
        self.streams["odometry"].add(message)
        self.odom_child_frames.add(message.child_frame_id)
        position = message.pose.pose.position
        current = (float(position.x), float(position.y), float(position.z))
        if self.odom_initial_position is None:
            self.odom_initial_position = current
        self.odom_final_position = current
        displacement = math.sqrt(
            sum(
                (value - initial) ** 2
                for value, initial in zip(current, self.odom_initial_position)
            )
        )
        self.odom_max_displacement_m = max(
            self.odom_max_displacement_m, displacement
        )

    def on_status(self, message):
        self.streams["status"].add(message)
        self.vo_states[str(message.vo_state)] += 1

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        raise SystemExit("--duration must be a positive finite number")

    rclpy.init()
    node = Validator(args.duration)
    try:
        deadline = node.started + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            # Check the wall-clock deadline between callbacks. A ROS timer can
            # starve behind high-rate image/IMU subscriptions in a
            # single-threaded executor.
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        elapsed = time.monotonic() - node.started
        odometry_motion = {
            "initial_position_m": node.odom_initial_position,
            "final_position_m": node.odom_final_position,
            "delta_position_m": None,
            "final_displacement_m": None,
            "max_displacement_m": round(node.odom_max_displacement_m, 6),
        }
        if (
            node.odom_initial_position is not None
            and node.odom_final_position is not None
        ):
            delta = tuple(
                final - initial
                for final, initial in zip(
                    node.odom_final_position, node.odom_initial_position
                )
            )
            odometry_motion["delta_position_m"] = delta
            odometry_motion["final_displacement_m"] = round(
                math.sqrt(sum(value * value for value in delta)), 6
            )
        output = {
            "requested_duration_s": args.duration,
            "wall_duration_s": round(elapsed, 3),
            "streams": {name: value.result() for name, value in node.streams.items()},
            "stereo_pairing": stereo_pairing(
                node.streams["infra1"], node.streams["infra2"]
            ),
            "vo_states": dict(sorted(node.vo_states.items())),
            "odometry_child_frames": sorted(node.odom_child_frames),
            "odometry_motion": odometry_motion,
        }
        print(json.dumps(output, indent=2, sort_keys=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
