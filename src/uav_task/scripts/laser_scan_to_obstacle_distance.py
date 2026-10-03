#!/usr/bin/env python3
"""Convert sensor_msgs/LaserScan (/scan) to px4_msgs/ObstacleDistance for XRCE."""

from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import ObstacleDistance
from sensor_msgs.msg import LaserScan


class LaserScanToObstacleDistance(Node):
    NUM_BINS = 72

    def __init__(self) -> None:
        super().__init__('laser_scan_to_obstacle_distance')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('obstacle_topic', '/fmu/in/obstacle_distance')
        self.declare_parameter('min_distance_m', 0.2)
        self.declare_parameter('max_distance_m', 8.0)
        self.declare_parameter('angle_offset_deg', 0.0)

        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        obstacle_topic = self.get_parameter('obstacle_topic').get_parameter_value().string_value
        self._min_m = float(self.get_parameter('min_distance_m').value)
        self._max_m = float(self.get_parameter('max_distance_m').value)
        self._angle_offset_deg = float(self.get_parameter('angle_offset_deg').value)

        self._pub = self.create_publisher(ObstacleDistance, obstacle_topic, qos_out)
        self._sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_cb, qos_profile_sensor_data)
        self.get_logger().info(
            f'Bridging {scan_topic} → {obstacle_topic} '
            f'(bins={self.NUM_BINS}, max={self._max_m}m)'
        )

    def _scan_cb(self, scan: LaserScan) -> None:
        msg = ObstacleDistance()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.frame = ObstacleDistance.MAV_FRAME_BODY_FRD
        msg.sensor_type = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
        msg.min_distance = int(max(0, round(self._min_m * 100.0)))
        msg.max_distance = int(max(msg.min_distance + 1, round(self._max_m * 100.0)))
        msg.increment = 360.0 / float(self.NUM_BINS)
        msg.angle_offset = self._angle_offset_deg

        unknown = 65535
        no_obstacle = msg.max_distance + 1
        distances = [unknown] * self.NUM_BINS

        if not scan.ranges:
            msg.distances = distances
            self._pub.publish(msg)
            return

        angle = float(scan.angle_min)
        for r in scan.ranges:
            if math.isfinite(r) and self._min_m <= r <= self._max_m:
                # LaserScan: 0 = forward, CCW positive (ROS). PX4 body FRD:
                # angle_offset=0 is forward, positive clockwise.
                yaw_cw_deg = -math.degrees(angle) + self._angle_offset_deg
                idx = int(round(yaw_cw_deg / msg.increment)) % self.NUM_BINS
                cm = int(round(r * 100.0))
                if distances[idx] == unknown or cm < distances[idx]:
                    distances[idx] = cm
            angle += float(scan.angle_increment)

        for i, d in enumerate(distances):
            if d == unknown:
                distances[i] = no_obstacle

        msg.distances = distances
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = LaserScanToObstacleDistance()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
