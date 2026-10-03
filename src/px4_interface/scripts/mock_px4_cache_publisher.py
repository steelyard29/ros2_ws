#!/usr/bin/env python3
"""Publish mock PX4 cache topics for driving the UAV TUI dashboard."""

from __future__ import annotations

import math
import random
import time
from typing import Tuple

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from px4_interface.msg import BatteryStatus, PoseNED, VehicleStatus


def _now_time(node: Node) -> Time:
    stamp = node.get_clock().now().to_msg()
    return stamp


class MockPx4CachePublisher(Node):
    """Simple ROS2 node that publishes simulated PX4 cache messages."""

    def __init__(self) -> None:
        super().__init__("mock_px4_cache_publisher")
        self._pose_pub = self.create_publisher(PoseNED, "/cache/vehicle_odometry", 10)
        self._battery_pub = self.create_publisher(BatteryStatus, "/cache/battery_status", 10)
        self._vehicle_pub = self.create_publisher(VehicleStatus, "/cache/vehicle_status", 10)

        self._start_time = time.time()
        self._heading = 0.0
        self._battery = 1.0
        self._random = random.Random(42)
        self._monitored_topics: Tuple[str, ...] = (
            "/cache/vehicle_odometry",
            "/cache/battery_status",
            "/cache/vehicle_status",
        )
        self._publishing_enabled = True
        self._conflict_reported = False

        self.create_timer(0.2, self._publish_samples)  # 5 Hz
        self.create_timer(1.0, self._monitor_conflicting_publishers)
        self.get_logger().info("Mock PX4 cache publisher started (5 Hz)")

    def _publish_samples(self) -> None:
        if not self._publishing_enabled:
            if not self._conflict_reported:
                # 避免重复打印，冲突检测计时器负责恢复日志状态。
                self._conflict_reported = True
            return

        t = time.time() - self._start_time
        radius = 5.0
        omega = 0.2  # rad/s
        self._heading += omega * 0.2

        x_north = radius * math.cos(omega * t)
        y_east = radius * math.sin(omega * t)
        z_down = -1.0 + 0.2 * math.sin(omega * t * 0.5)

        pose_msg = PoseNED()
        pose_msg.valid = True
        pose_msg.translation = [float(x_north), float(y_east), float(z_down)]
        pose_msg.velocity = [float(-radius * omega * math.sin(omega * t)), float(radius * omega * math.cos(omega * t)), 0.0]
        qw = math.cos(self._heading / 2.0)
        qz = math.sin(self._heading / 2.0)
        pose_msg.orientation = [float(qw), 0.0, 0.0, float(qz)]
        pose_msg.timestamp = _now_time(self)
        self._pose_pub.publish(pose_msg)

        self._battery = max(0.2, self._battery - 0.001)
        battery_msg = BatteryStatus()
        battery_msg.valid = True
        battery_msg.timestamp = _now_time(self)
        battery_msg.voltage_v = 15.2 - 3.0 * (1.0 - self._battery)
        battery_msg.current_a = 12.0 + self._random.uniform(-1.0, 1.0)
        battery_msg.remaining = float(self._battery)
        battery_msg.warning = 0 if self._battery > 0.3 else 1
        self._battery_pub.publish(battery_msg)

        vehicle_msg = VehicleStatus()
        vehicle_msg.valid = True
        vehicle_msg.timestamp = _now_time(self)
        vehicle_msg.arming_state = 2  # ARMING_STATE_ARMED
        vehicle_msg.nav_state = 4  # NAVIGATION_STATE_MISSION
        vehicle_msg.failsafe = self._battery < 0.25
        vehicle_msg.pre_flight_checks_pass = True
        self._vehicle_pub.publish(vehicle_msg)

    def _monitor_conflicting_publishers(self) -> None:
        conflicts: list[str] = []
        for topic in self._monitored_topics:
            publisher_count = len(self.get_publishers_info_by_topic(topic))
            extra_publishers = max(0, publisher_count - 1)
            if extra_publishers > 0:
                conflicts.append(f"{topic} (+{extra_publishers})")

        if conflicts:
            if self._publishing_enabled:
                self._publishing_enabled = False
                self._conflict_reported = False
                details = ", ".join(conflicts)
                self.get_logger().error(
                    "Detected additional publishers on PX4 cache topics [%s]; "
                    "suspending mock outputs to avoid conflicting telemetry.",
                    details,
                )
        else:
            if not self._publishing_enabled:
                self._publishing_enabled = True
                self._conflict_reported = False
                self.get_logger().info("No conflicting publishers detected; resuming mock outputs.")

    def destroy_node(self) -> None:
        self.get_logger().info("Mock PX4 cache publisher stopping")
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MockPx4CachePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
