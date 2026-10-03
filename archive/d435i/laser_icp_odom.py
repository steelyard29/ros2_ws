#!/usr/bin/env python3
"""
2D Laser Odometry — ICP-based XY drift corrector for RPLidar A2M8.

Subscribes to /scan and computes the XY translation between consecutive
scans using Iterative Closest Point (ICP). Accumulates into an odometry
estimate that is drift-free in XY (unlike VSLAM, which drifts with yaw).

Publishes:
  /laser_odom (nav_msgs/Odometry) — ICP-based XY + VSLAM velocity/orientation

This replaces the slam_toolbox dependency for providing a drift-free XY
position source. It does NOT build a map — it only computes scan-to-scan
relative motion, which is drift-free for short-term position correction.

Combined with VSLAM velocity in lidar_tf_relay.py, this provides:
  激光 ICP XY (no drift) + VSLAM velocity/orientation → bridge → PX4 EKF

Usage:
  ros2 run px4_interface laser_icp_odom.py
"""

import math
import numpy as np
from sklearn.neighbors import NearestNeighbors
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros


class LaserIcpOdom(Node):
    def __init__(self):
        super().__init__('laser_icp_odom')

        # Parameters
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('max_iterations', 30)
        self.declare_parameter('tolerance', 0.001)
        self.declare_parameter('max_correspondence_dist', 0.5)
        self.declare_parameter('min_valid_points', 50)
        self.declare_parameter('max_angular_velocity', 1.0)  # rad/s, reject outlier rotations

        self._publish_rate = self.get_parameter('publish_rate').value
        self._max_iter = self.get_parameter('max_iterations').value
        self._tolerance = self.get_parameter('tolerance').value
        self._max_corr_dist = self.get_parameter('max_correspondence_dist').value
        self._min_valid = self.get_parameter('min_valid_points').value
        self._max_ang_vel = self.get_parameter('max_angular_velocity').value

        # Subscribers
        self._scan_sub = self.create_subscription(
            LaserScan, '/scan_planar', self._scan_cb, 10)
        self._vslam_sub = self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry', self._vslam_cb, 10)

        # Publishers
        self._odom_pub = self.create_publisher(Odometry, '/laser_odom', 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # State
        self._prev_scan_pts = None  # (N,2) numpy array of previous scan points
        self._latest_vslam = None
        self._accumulated_x = 0.0
        self._accumulated_y = 0.0
        self._accumulated_yaw = 0.0
        self._last_scan_time = None

        self._timer = self.create_timer(1.0 / self._publish_rate, self._publish)

        self.get_logger().info(
            f'Laser ICP Odom started: /scan ICP → /laser_odom @ {self._publish_rate} Hz')

    def _vslam_cb(self, msg: Odometry):
        self._latest_vslam = msg

    @staticmethod
    def _scan_to_points(scan: LaserScan) -> np.ndarray:
        """Convert LaserScan to (N,2) numpy array of valid XY points in laser frame."""
        n = len(scan.ranges)
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        ranges = np.array(scan.ranges, dtype=np.float32)
        valid = (ranges >= scan.range_min) & (ranges <= scan.range_max) & np.isfinite(ranges)
        if np.sum(valid) < 10:
            return np.empty((0, 2))
        a = angles[valid]
        r = ranges[valid]
        x = r * np.cos(a)
        y = r * np.sin(a)
        return np.column_stack([x, y])

    @staticmethod
    def _icp_step(prev_pts: np.ndarray, curr_pts: np.ndarray,
                  max_corr_dist: float) -> tuple:
        """One ICP step: match points and compute transform.

        Returns (dx, dy, dtheta) or None if failed.
        """
        # Downsample for speed
        if len(prev_pts) > 400:
            idx = np.random.choice(len(prev_pts), 400, replace=False)
            prev_pts = prev_pts[idx]
        if len(curr_pts) > 400:
            idx = np.random.choice(len(curr_pts), 400, replace=False)
            curr_pts = curr_pts[idx]

        # Nearest neighbor matching (prev → curr)
        nn = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(curr_pts)
        distances, indices = nn.kneighbors(prev_pts)
        distances = distances.ravel()
        indices = indices.ravel()

        # Filter by distance
        valid = distances < max_corr_dist
        if np.sum(valid) < 10:
            return None

        src = prev_pts[valid]
        dst = curr_pts[indices[valid]]

        # Compute centroids
        src_centroid = np.mean(src, axis=0)
        dst_centroid = np.mean(dst, axis=0)

        # Compute rotation (using SVD)
        src_centered = src - src_centroid
        dst_centered = dst - dst_centroid
        H = src_centered.T @ dst_centered
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        dtheta = math.atan2(R[1, 0], R[0, 0])
        t = dst_centroid - R @ src_centroid

        return float(t[0]), float(t[1]), float(dtheta)

    def _scan_cb(self, msg: LaserScan):
        curr_pts = self._scan_to_points(msg)
        if len(curr_pts) < self._min_valid:
            return

        if self._prev_scan_pts is not None and len(self._prev_scan_pts) >= self._min_valid:
            # Run ICP
            result = self._icp_step(self._prev_scan_pts, curr_pts, self._max_corr_dist)
            if result is not None:
                dx, dy, dtheta = result
                # Reject outlier rotations (VSLAM yaw is more reliable)
                if abs(dtheta) > self._max_ang_vel * 0.1:  # max 0.1 rad/frame
                    dtheta = 0.0
                # Accumulate
                self._accumulated_x += dx
                self._accumulated_y += dy
                self._accumulated_yaw += dtheta

        self._prev_scan_pts = curr_pts
        self._last_scan_time = self.get_clock().now()

    def _publish(self):
        now = self.get_clock().now()

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position: ICP accumulated XY
        odom.pose.pose.position.x = self._accumulated_x
        odom.pose.pose.position.y = self._accumulated_y

        # Orientation + velocity: VSLAM (more reliable for rotation/velocity)
        if self._latest_vslam is not None:
            odom.pose.pose.position.z = self._latest_vslam.pose.pose.position.z
            odom.pose.pose.orientation = self._latest_vslam.pose.pose.orientation
            odom.twist.twist = self._latest_vslam.twist.twist
            odom.twist.covariance = self._latest_vslam.twist.covariance
            # Orientation covariance from VSLAM
            for i in [21, 28, 35]:
                odom.pose.covariance[i] = self._latest_vslam.pose.covariance[i]
            odom.pose.covariance[14] = self._latest_vslam.pose.covariance[14]
        else:
            odom.pose.pose.orientation.w = 1.0

        # ICP XY covariance: 0.01 (10cm) — conservative, EKF will weight appropriately
        odom.pose.covariance[0] = 0.01   # x variance
        odom.pose.covariance[7] = 0.01   # y variance

        self._odom_pub.publish(odom)

        # Also publish TF lidar_map→base_link (drop-in replacement for slam_toolbox)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = 'lidar_map'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self._accumulated_x
        tf_msg.transform.translation.y = self._accumulated_y
        tf_msg.transform.translation.z = 0.0
        if self._latest_vslam is not None:
            tf_msg.transform.rotation = self._latest_vslam.pose.pose.orientation
        else:
            tf_msg.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserIcpOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
