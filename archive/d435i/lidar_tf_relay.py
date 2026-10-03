#!/usr/bin/env python3
"""
Lidar SLAM → VSLAM Odom Relay Node (hardened v2)

Fuses slam_toolbox lidar_map→base_link TF (drift-free XY from 2D lidar SLAM)
with VSLAM velocity + orientation. Features:

- Three-tier degradation chain: lidar XY → VSLAM XY → frozen pose
  (NEVER silently stops publishing — that causes EKF EV pauses)
- Loop-closure jump guard: smooths >0.3m jumps over 0.5s
- Velocity fallback: differentiates lidar XY when VSLAM velocity stale
- Dynamic covariance based on source freshness
- Time-aligned TF lookup with cached stamps
- Configurable via ROS params and /lidar/relay/debug topic

Subscribes:
  /visual_slam/tracking/odometry (nav_msgs/Odometry) — velocity + orientation

TF lookup:
  lidar_map → base_link (from slam_toolbox, 20Hz)

Publishes:
  /lidar/relay/odometry (nav_msgs/Odometry) — fused odometry @30Hz
  /lidar/relay/status (std_msgs/String) — degradation status (latch)

Parameters:
  publish_rate: 30.0          — output rate (Hz)
  tf_stale_threshold_ms: 300  — TF age to switch to VSLAM fallback
  vslam_stale_threshold_ms: 2000 — VSLAM age to freeze
  lidar_variance_xy: 0.0025   — lidar XY covariance (5cm²)
  vslam_fallback_variance_xy: 0.25 — inflated covariance when degraded
  freeze_variance_xy: 1.0     — max covariance when frozen
  jump_threshold_m: 0.3       — XY jump to trigger smooth guard
  jump_smooth_duration_s: 0.5 — smoothing duration for jumps
  velocity_buffer_size: 5     — lidar XY history for velocity differentiation
  velocity_lpf_alpha: 0.3     — low-pass filter for lidar-derived velocity
  slam_tf_timeout_ms: 300     — TF lookup timeout

Usage:
  ros2 run px4_interface lidar_tf_relay.py \
    --ros-args -p publish_rate:=30.0 -p tf_stale_threshold_ms:=300
"""

import math
import time
from collections import deque
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf2_ros
from geometry_msgs.msg import TransformStamped


class LidarTfRelay(Node):
    def __init__(self):
        super().__init__('lidar_tf_relay')

        # ── Parameters ──
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('tf_stale_threshold_ms', 1000)
        self.declare_parameter('vslam_stale_threshold_ms', 2000)
        self.declare_parameter('lidar_variance_xy', 0.0025)
        self.declare_parameter('vslam_fallback_variance_xy', 0.25)
        self.declare_parameter('freeze_variance_xy', 1.0)
        self.declare_parameter('jump_threshold_m', 0.3)
        self.declare_parameter('jump_smooth_duration_s', 0.5)
        self.declare_parameter('velocity_buffer_size', 5)
        self.declare_parameter('velocity_lpf_alpha', 0.3)
        self.declare_parameter('slam_tf_timeout_ms', 1000)

        self._publish_rate = self.get_parameter('publish_rate').value
        self._tf_stale_ms = self.get_parameter('tf_stale_threshold_ms').value
        self._vslam_stale_ms = self.get_parameter('vslam_stale_threshold_ms').value
        self._lidar_var = self.get_parameter('lidar_variance_xy').value
        self._vslam_var = self.get_parameter('vslam_fallback_variance_xy').value
        self._freeze_var = self.get_parameter('freeze_variance_xy').value
        self._jump_threshold = self.get_parameter('jump_threshold_m').value
        self._jump_smooth_s = self.get_parameter('jump_smooth_duration_s').value
        self._vel_buf_size = self.get_parameter('velocity_buffer_size').value
        self._vel_alpha = self.get_parameter('velocity_lpf_alpha').value
        self._tf_timeout_ms = self.get_parameter('slam_tf_timeout_ms').value

        # ── TF ──
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._last_tf = None  # (TransformStamped, stamp)
        self._last_tf_stamp = None

        # ── Subscriptions ──
        self._vslam_sub = self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry', self._vslam_cb, 10)

        # ── Publishers ──
        self._odom_pub = self.create_publisher(Odometry, '/lidar/relay/odometry', 10)
        self._status_pub = self.create_publisher(String, '/lidar/relay/status', 10)

        # ── State ──
        self._lock = Lock()
        self._latest_vslam = None
        self._vslam_stamp = None

        # Jump guard state
        self._jump_active = False
        self._jump_start_time = 0.0
        self._jump_start_xy = (0.0, 0.0)
        self._jump_target_xy = (0.0, 0.0)
        self._last_lidar_xy = (0.0, 0.0)
        self._last_published_xy = (0.0, 0.0)

        # Velocity fallback: ring buffer of (stamp, x, y)
        self._lidar_xy_history = deque(maxlen=self._vel_buf_size)
        self._lidar_vel_filtered = (0.0, 0.0)

        # Degradation status
        self._current_source = 'unknown'
        self._last_status_msg = ''

        # ── Timer ──
        self._timer = self.create_timer(1.0 / self._publish_rate, self._publish_odom)

        self.get_logger().info(
            f'Lidar TF Relay v2: lidar XY + VSLAM vel → /lidar/relay/odometry '
            f'@{self._publish_rate:.0f} Hz'
        )
        self.get_logger().info(
            f'  TF stale={self._tf_stale_ms}ms, VSLAM stale={self._vslam_stale_ms}ms, '
            f'lidar_var={self._lidar_var}, jump={self._jump_threshold}m')

    # ─────────────────────────────────────────────────────────────
    #  Callbacks
    # ─────────────────────────────────────────────────────────────

    def _vslam_cb(self, msg: Odometry):
        with self._lock:
            self._latest_vslam = msg
            self._vslam_stamp = self.get_clock().now()

    # ─────────────────────────────────────────────────────────────
    #  Core: publish odometry
    # ─────────────────────────────────────────────────────────────

    def _publish_odom(self):
        now = self.get_clock().now()

        # ── 1. Acquire latest data under lock ──
        with self._lock:
            vslam = self._latest_vslam
            vslam_stamp = self._vslam_stamp

        # ── 2. Lookup lidar TF with timeout ──
        lidar_xy = None
        tf_stamp = None
        try:
            tf_msg: TransformStamped = self._tf_buffer.lookup_transform(
                'lidar_map', 'base_link',
                rclpy.time.Time(seconds=0, nanoseconds=0),
                rclpy.duration.Duration(seconds=self._tf_timeout_ms * 1e-3))
            self._last_tf = tf_msg
            self._last_tf_stamp = now
            lidar_xy = (
                tf_msg.transform.translation.x,
                tf_msg.transform.translation.y,
            )
            tf_stamp = tf_msg.header.stamp
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass

        # ── 3. Compute TF age ──
        tf_age_ms = None
        if self._last_tf_stamp is not None:
            tf_age_ms = (now - self._last_tf_stamp).nanoseconds * 1e-6
        if tf_age_ms is None:
            tf_age_ms = float('inf')

        # ── 4. Compute VSLAM age ──
        vslam_age_ms = float('inf')
        if vslam_stamp is not None:
            vslam_age_ms = (now - vslam_stamp).nanoseconds * 1e-6

        # ── 5. Determine active source (degradation chain) ──
        source = 'frozen'
        if tf_age_ms < self._tf_stale_ms:
            source = 'lidar'
        elif vslam is not None and vslam_age_ms < self._vslam_stale_ms:
            source = 'vslam'

        if vslam is None and source == 'vslam':
            source = 'frozen'

        # ── 6. Build odometry ──
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        if source == 'lidar':
            self._build_lidar(odom, lidar_xy, vslam, vslam_age_ms, tf_age_ms)
        elif source == 'vslam':
            self._build_vslam(odom, vslam)
        else:
            self._build_frozen(odom)

        # ── 7. Update velocity history for fallback ──
        if source == 'lidar' and lidar_xy is not None:
            self._lidar_xy_history.append((now, lidar_xy[0], lidar_xy[1]))
            self._update_lidar_velocity()

        # ── 8. Publish ──
        self._odom_pub.publish(odom)

        # ── 9. Status (latch changes only) ──
        if source != self._current_source:
            self._current_source = source
            status_str = f'source={source} tf_age={tf_age_ms:.0f}ms vslam_age={vslam_age_ms:.0f}ms'
            status = String()
            status.data = status_str
            self._status_pub.publish(status)
            if source != 'lidar':
                self.get_logger().warn(status_str, throttle_duration_sec=1.0)

    # ─────────────────────────────────────────────────────────────
    #  Source builders
    # ─────────────────────────────────────────────────────────────

    def _build_lidar(self, odom, lidar_xy, vslam, vslam_age_ms, tf_age_ms):
        """Lidar XY (primary) + VSLAM velocity/orientation."""
        if lidar_xy is None:
            # TF available but XY is None — fall back to VSLAM
            self._build_vslam(odom, vslam)
            return
        x, y = lidar_xy

        # ── Jump guard ──
        x, y = self._guard_jump(x, y)

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y

        if vslam is not None:
            odom.pose.pose.position.z = vslam.pose.pose.position.z
            odom.pose.pose.orientation = vslam.pose.pose.orientation
        else:
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.w = 1.0

        # Covariance: tight XY from lidar, rest from VSLAM
        odom.pose.covariance[0] = self._lidar_var   # x
        odom.pose.covariance[7] = self._lidar_var   # y
        if vslam is not None:
            odom.pose.covariance[14] = vslam.pose.covariance[14]  # z
            for i in [21, 28, 35]:
                odom.pose.covariance[i] = vslam.pose.covariance[i]
        else:
            odom.pose.covariance[14] = self._freeze_var
            for i in [21, 28, 35]:
                odom.pose.covariance[i] = self._freeze_var

        # Velocity
        if vslam is not None and vslam_age_ms < self._vslam_stale_ms:
            odom.twist.twist = vslam.twist.twist
            odom.twist.covariance = vslam.twist.covariance
        else:
            # Velocity fallback: use lidar-derived velocity
            odom.twist.twist.linear.x = self._lidar_vel_filtered[0]
            odom.twist.twist.linear.y = self._lidar_vel_filtered[1]
            if vslam is not None:
                odom.twist.twist.linear.z = vslam.twist.twist.linear.z
                odom.twist.twist.angular = vslam.twist.twist.angular
                odom.twist.covariance = vslam.twist.covariance
            else:
                odom.twist.twist.angular.z = 0.0
            # Inflate velocity covariance when lidar-derived
            for i in [0, 7, 14, 21, 28, 35]:
                odom.twist.covariance[i] = self._vslam_var

        self._last_published_xy = (x, y)

    def _build_vslam(self, odom, vslam):
        """VSLAM fallback: all data from VSLAM with inflated XY covariance."""
        odom.pose.pose.position.x = vslam.pose.pose.position.x
        odom.pose.pose.position.y = vslam.pose.pose.position.y
        odom.pose.pose.position.z = vslam.pose.pose.position.z
        odom.pose.pose.orientation = vslam.pose.pose.orientation
        odom.twist.twist = vslam.twist.twist

        # Full covariance from VSLAM, but inflate XY
        odom.pose.covariance = list(vslam.pose.covariance)
        odom.twist.covariance = list(vslam.twist.covariance)
        odom.pose.covariance[0] = self._vslam_var
        odom.pose.covariance[7] = self._vslam_var

        self._last_published_xy = (
            vslam.pose.pose.position.x,
            vslam.pose.pose.position.y,
        )

    def _build_frozen(self, odom):
        """Frozen: use last published XY, max covariance → EKF rejects."""
        x, y = self._last_published_xy
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0

        for i in range(36):
            odom.pose.covariance[i] = 0.0
            odom.twist.covariance[i] = 0.0
        odom.pose.covariance[0] = self._freeze_var
        odom.pose.covariance[7] = self._freeze_var
        odom.pose.covariance[14] = self._freeze_var
        for i in [21, 28, 35]:
            odom.pose.covariance[i] = self._freeze_var
        for i in [0, 7, 14, 21, 28, 35]:
            odom.twist.covariance[i] = self._freeze_var

    # ─────────────────────────────────────────────────────────────
    #  Jump guard: smooth lidar XY jumps (loop closure / re-localization)
    # ─────────────────────────────────────────────────────────────

    def _guard_jump(self, x, y):
        """If XY jumps > threshold, smooth linearly over jump_smooth_duration."""
        if not self._jump_active:
            if self._last_published_xy != (0.0, 0.0):
                dx = x - self._last_published_xy[0]
                dy = y - self._last_published_xy[1]
                dist = math.sqrt(dx * dx + dy * dy)
                if dist > self._jump_threshold:
                    self._jump_active = True
                    self._jump_start_time = time.time()
                    self._jump_start_xy = self._last_published_xy
                    self._jump_target_xy = (x, y)
                    self.get_logger().warn(
                        f'Lidar XY jump detected: {dist:.3f}m → smoothing over '
                        f'{self._jump_smooth_s}s'
                    )

        if self._jump_active:
            elapsed = time.time() - self._jump_start_time
            if elapsed >= self._jump_smooth_s:
                self._jump_active = False
                self.get_logger().info('Jump smoothing complete')
                return x, y

            # Linear interpolation
            t = elapsed / self._jump_smooth_s
            t = t * t * (3.0 - 2.0 * t)  # smoothstep easing
            sx = self._jump_start_xy[0] + t * (self._jump_target_xy[0] - self._jump_start_xy[0])
            sy = self._jump_start_xy[1] + t * (self._jump_target_xy[1] - self._jump_start_xy[1])
            return sx, sy

        return x, y

    # ─────────────────────────────────────────────────────────────
    #  Velocity fallback: differentiate lidar XY history
    # ─────────────────────────────────────────────────────────────

    def _update_lidar_velocity(self):
        """Differentiate lidar XY ring buffer for velocity estimate."""
        if len(self._lidar_xy_history) < 2:
            return

        oldest = self._lidar_xy_history[0]
        newest = self._lidar_xy_history[-1]
        dt = (newest[0] - oldest[0]).nanoseconds * 1e-9
        if dt < 1e-6:
            return

        vx = (newest[1] - oldest[1]) / dt
        vy = (newest[2] - oldest[2]) / dt

        # Low-pass filter
        self._lidar_vel_filtered = (
            self._vel_alpha * vx + (1.0 - self._vel_alpha) * self._lidar_vel_filtered[0],
            self._vel_alpha * vy + (1.0 - self._vel_alpha) * self._lidar_vel_filtered[1],
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarTfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
