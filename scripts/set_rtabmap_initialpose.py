#!/usr/bin/env python3
"""Publish a coarse /rtabmap/initialpose so laser proximity can lock localization.

RTAB-Map in localization mode cannot solve the kidnapped-robot problem from
visual matches alone when depth/RGB are poorly associated or the scene is
texture-poor. Mathieu's recommended workflow (and Yahboom RTAB-Map nav demos)
is: give a 2D pose guess near the true place, then let scan proximity/ICP
refine (yellow "Local match" in rtabmapviz).

Examples:
  # Assume the vehicle is at the map origin, yaw=0
  python3 scripts/set_rtabmap_initialpose.py

  # Explicit map-frame pose (metres, radians)
  python3 scripts/set_rtabmap_initialpose.py --x 1.2 --y -0.4 --yaw 1.57

  # Keep republishing for a few seconds while RTAB-Map starts
  python3 scripts/set_rtabmap_initialpose.py --repeat 10 --period 0.5
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class InitialPosePublisher(Node):
    def __init__(self, x, y, z, yaw, frame_id, topic):
        super().__init__('set_rtabmap_initialpose')
        self._pub = self.create_publisher(PoseWithCovarianceStamped, topic, 10)
        self._msg = PoseWithCovarianceStamped()
        self._msg.header.frame_id = frame_id
        self._msg.pose.pose.position.x = float(x)
        self._msg.pose.pose.position.y = float(y)
        self._msg.pose.pose.position.z = float(z)
        qx, qy, qz, qw = yaw_to_quat(float(yaw))
        self._msg.pose.pose.orientation.x = qx
        self._msg.pose.pose.orientation.y = qy
        self._msg.pose.pose.orientation.z = qz
        self._msg.pose.pose.orientation.w = qw
        # Loose prior: RTAB-Map will refine with proximity/ICP.
        self._msg.pose.covariance[0] = 0.25
        self._msg.pose.covariance[7] = 0.25
        self._msg.pose.covariance[35] = 0.17

    def publish_once(self):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._msg)
        self.get_logger().info(
            f'published initialpose '
            f'x={self._msg.pose.pose.position.x:.3f} '
            f'y={self._msg.pose.pose.position.y:.3f} '
            f'yaw={2.0 * math.atan2(self._msg.pose.pose.orientation.z, self._msg.pose.pose.orientation.w):.3f}'
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--z', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0, help='yaw in radians')
    parser.add_argument('--frame', default='map')
    parser.add_argument('--topic', default='/rtabmap/initialpose')
    parser.add_argument('--repeat', type=int, default=3)
    parser.add_argument('--period', type=float, default=0.5)
    args = parser.parse_args()

    rclpy.init()
    node = InitialPosePublisher(
        args.x, args.y, args.z, args.yaw, args.frame, args.topic)
    try:
        # Give DDS discovery a moment before the first publish.
        time.sleep(0.5)
        for _ in range(max(1, args.repeat)):
            node.publish_once()
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(max(0.05, args.period))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
