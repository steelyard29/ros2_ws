#!/usr/bin/env python3
"""Gate a globally consistent RTAB-Map pose with a fresh VIO body twist.

The output pose is the complete ``map -> base_link`` TF transform.  Its
timestamp is the TF source timestamp (never the relay's wall-clock publish
time).  VIO contributes only the body-frame twist; its absolute pose is never
used as a fallback.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import threading
import time
from typing import Optional, Sequence, Tuple

try:
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.time import Time
    from rtabmap_msgs.msg import Info as RtabmapInfo
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener
except ModuleNotFoundError:  # Allow ROS-independent unit tests of pure logic.
    rclpy = None
    Node = object
    RtabmapInfo = None


@dataclass(frozen=True)
class RelayHealth:
    source: str
    healthy: bool
    age: float
    reason: str


def sample_age(now_sec: float, stamp_sec: float) -> Optional[float]:
    """Return source age, or None for an absent/zero source timestamp."""
    if stamp_sec <= 0.0 or not math.isfinite(stamp_sec):
        return None
    return now_sec - stamp_sec


def quaternion_angle(
        first: Sequence[float], second: Sequence[float]) -> float:
    """Shortest angular distance between two (x, y, z, w) quaternions."""
    first_norm = math.sqrt(sum(value * value for value in first))
    second_norm = math.sqrt(sum(value * value for value in second))
    if first_norm < 1e-9 or second_norm < 1e-9:
        return math.inf
    dot = abs(sum(a * b for a, b in zip(first, second)))
    dot = min(1.0, max(0.0, dot / (first_norm * second_norm)))
    return 2.0 * math.acos(dot)


def pose_jump(
        previous_position: Optional[Sequence[float]],
        previous_orientation: Optional[Sequence[float]],
        position: Sequence[float],
        orientation: Sequence[float]) -> Tuple[float, float]:
    """Return translational and angular changes from the preceding TF sample."""
    if previous_position is None or previous_orientation is None:
        return 0.0, 0.0
    distance = math.sqrt(sum(
        (current - previous) ** 2
        for current, previous in zip(position, previous_position)))
    return distance, quaternion_angle(previous_orientation, orientation)


def assess_health(
        tf_age: Optional[float],
        vio_age: Optional[float],
        position_jump_m: float,
        orientation_jump_rad: float,
        tf_timeout_sec: float,
        vio_timeout_sec: float,
        max_position_jump_m: float,
        max_orientation_jump_rad: float,
        future_tolerance_sec: float = 0.05,
        data_valid: bool = True,
        frame_valid: bool = True) -> RelayHealth:
    """Pure health/status decision used by the ROS node and unit tests."""
    source = 'rtabmap_tf+vio_body_twist'
    ages = [age for age in (tf_age, vio_age) if age is not None]
    age = max(ages) if ages else -1.0
    if tf_age is None:
        return RelayHealth(source, False, age, 'missing_rtabmap_tf')
    if tf_age < -future_tolerance_sec:
        return RelayHealth(source, False, age, 'rtabmap_tf_from_future')
    if tf_age > tf_timeout_sec:
        return RelayHealth(source, False, age, 'rtabmap_tf_stale')
    if vio_age is None:
        return RelayHealth(source, False, age, 'missing_vio')
    if vio_age < -future_tolerance_sec:
        return RelayHealth(source, False, age, 'vio_from_future')
    if vio_age > vio_timeout_sec:
        return RelayHealth(source, False, age, 'vio_stale')
    if not frame_valid:
        return RelayHealth(source, False, age, 'vio_child_frame_mismatch')
    if not data_valid:
        return RelayHealth(source, False, age, 'non_finite_or_invalid_data')
    if position_jump_m > max_position_jump_m:
        return RelayHealth(source, False, age, 'position_jump')
    if orientation_jump_rad > max_orientation_jump_rad:
        return RelayHealth(source, False, age, 'orientation_jump')
    return RelayHealth(source, True, age, 'ok')


def assess_geometric_match(
        require_match: bool,
        match_age_sec: Optional[float],
        match_hold_timeout_sec: float) -> Optional[str]:
    """Return a rejection reason when map lock is missing/stale, else None.

    initialpose alone can publish a low-covariance localization_pose while the
    vehicle is still in a map hole. Absolute XY must wait for proximity/loop.
    """
    if not require_match:
        return None
    if match_age_sec is None:
        return 'waiting_geometric_match'
    if match_age_sec > match_hold_timeout_sec:
        return 'geometric_match_stale'
    return None


class RtabmapOdomRelay(Node):
    def __init__(self):
        super().__init__('rtabmap_odom_relay')

        self.declare_parameter(
            'vio_topic', '/visual_slam/tracking/odometry')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('tf_timeout_sec', 0.25)
        self.declare_parameter('vio_timeout_sec', 0.15)
        self.declare_parameter('tf_lookup_timeout_sec', 0.02)
        self.declare_parameter('future_tolerance_sec', 0.05)
        self.declare_parameter(
            'localization_pose_topic', '/rtabmap/localization_pose')
        self.declare_parameter('localization_timeout_sec', 1.0)
        self.declare_parameter('max_localization_variance', 10.0)
        self.declare_parameter('max_position_jump_m', 0.75)
        self.declare_parameter('max_orientation_jump_rad', 0.70)
        self.declare_parameter('info_topic', '/rtabmap/info')
        # initialpose + low cov is not a map lock; require proximity/loop.
        self.declare_parameter('require_geometric_match', True)
        # DetectionRate is typically 2 Hz; keep a short hold after last match.
        self.declare_parameter('match_hold_timeout_sec', 3.0)
        self.declare_parameter(
            'pose_covariance_diagonal',
            [0.25, 0.25, 0.50, 0.12, 0.12, 0.20])
        self.declare_parameter(
            'twist_covariance_diagonal',
            [0.10, 0.10, 0.20, 0.08, 0.08, 0.12])

        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        vio_topic = self.get_parameter('vio_topic').value
        self._tf_timeout = float(self.get_parameter('tf_timeout_sec').value)
        self._vio_timeout = float(self.get_parameter('vio_timeout_sec').value)
        self._lookup_timeout = float(
            self.get_parameter('tf_lookup_timeout_sec').value)
        self._future_tolerance = float(
            self.get_parameter('future_tolerance_sec').value)
        self._localization_timeout = float(
            self.get_parameter('localization_timeout_sec').value)
        self._max_localization_variance = float(
            self.get_parameter('max_localization_variance').value)
        self._max_position_jump = float(
            self.get_parameter('max_position_jump_m').value)
        self._max_orientation_jump = float(
            self.get_parameter('max_orientation_jump_rad').value)
        self._require_geometric_match = bool(
            self.get_parameter('require_geometric_match').value)
        self._match_hold_timeout = float(
            self.get_parameter('match_hold_timeout_sec').value)
        self._pose_covariance = self._diagonal_covariance(
            self.get_parameter('pose_covariance_diagonal').value)
        self._twist_covariance = self._diagonal_covariance(
            self.get_parameter('twist_covariance_diagonal').value)

        self._vslam_sub = self.create_subscription(
            Odometry, vio_topic, self._vslam_cb, 10)
        self._localization_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('localization_pose_topic').value,
            self._localization_cb,
            10)
        if RtabmapInfo is None:
            raise RuntimeError(
                'rtabmap_msgs is required for geometric match gating')
        self._info_sub = self.create_subscription(
            RtabmapInfo,
            self.get_parameter('info_topic').value,
            self._info_cb,
            10)
        self._odom_pub = self.create_publisher(
            Odometry, '/rtabmap/relay/odometry', 10)
        self._status_pub = self.create_publisher(
            String, '/rtabmap/relay/status', 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._lock = threading.Lock()
        self._latest_vslam = None
        self._latest_localization = None
        self._last_match_monotonic = None
        self._last_loop_id = 0
        self._last_proximity_id = 0
        self._previous_tf_position = None
        self._previous_tf_orientation = None
        self._previous_tf_stamp = None
        self._jump_latched = False

        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self._timer = self.create_timer(1.0 / rate, self._publish_odom)

        self.get_logger().info(
            f'RTAB-Map relay: TF {self._map_frame}->{self._base_frame} '
            f'+ body twist {vio_topic} -> /rtabmap/relay/odometry; '
            f'geometric_match='
            f'{"required" if self._require_geometric_match else "optional"} '
            f'(hold {self._match_hold_timeout:.1f}s); '
            'unhealthy input stops odometry (no absolute-pose fallback)')

    def _vslam_cb(self, msg: Odometry):
        with self._lock:
            self._latest_vslam = msg

    def _localization_cb(self, msg: PoseWithCovarianceStamped):
        with self._lock:
            self._latest_localization = msg

    def _info_cb(self, msg: RtabmapInfo):
        loop_id = int(msg.loop_closure_id)
        proximity_id = int(msg.proximity_detection_id)
        if loop_id <= 0 and proximity_id <= 0:
            return
        with self._lock:
            self._last_match_monotonic = time.monotonic()
            self._last_loop_id = loop_id
            self._last_proximity_id = proximity_id

    @staticmethod
    def _diagonal_covariance(diagonal: Sequence[float]):
        if len(diagonal) != 6 or any(
                value < 0.0 or not math.isfinite(value)
                for value in diagonal):
            raise ValueError('covariance diagonal must contain 6 finite values')
        covariance = [0.0] * 36
        for index, value in enumerate(diagonal):
            covariance[index * 6 + index] = float(value)
        return covariance

    @staticmethod
    def _covariance_with_floor(
            source: Sequence[float], floor: Sequence[float]):
        """Preserve VIO covariance while enforcing configured diagonal floors."""
        covariance = list(source) if len(source) == 36 else [0.0] * 36
        covariance = [
            value if math.isfinite(value) else 0.0 for value in covariance]
        for index in range(6):
            diagonal_index = index * 6 + index
            covariance[diagonal_index] = max(
                covariance[diagonal_index], floor[diagonal_index])
        return covariance

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _transform_values(transform: TransformStamped):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w))

    @staticmethod
    def _finite_message_data(position, orientation, vslam) -> bool:
        twist = vslam.twist.twist
        values = tuple(position) + tuple(orientation) + (
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z)
        quaternion_norm = math.sqrt(sum(value * value for value in orientation))
        return all(math.isfinite(value) for value in values) and (
            quaternion_norm > 1e-9)

    def _publish_status(
            self, health: RelayHealth,
            tf_age: Optional[float], vio_age: Optional[float],
            match_age: Optional[float],
            loop_id: int, proximity_id: int):
        message = String()
        message.data = json.dumps({
            'source': health.source,
            'healthy': health.healthy,
            'age': round(health.age, 6),
            'reason': health.reason,
            'tf_age': None if tf_age is None else round(tf_age, 6),
            'vio_age': None if vio_age is None else round(vio_age, 6),
            'match_age': None if match_age is None else round(match_age, 6),
            'loop_closure_id': int(loop_id),
            'proximity_detection_id': int(proximity_id),
            'frame_id': self._map_frame,
            'child_frame_id': self._base_frame,
        }, separators=(',', ':'))
        self._status_pub.publish(message)

    def _publish_odom(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        now_mono = time.monotonic()
        with self._lock:
            vslam = self._latest_vslam
            localization = self._latest_localization
            last_match = self._last_match_monotonic
            loop_id = self._last_loop_id
            proximity_id = self._last_proximity_id

        match_age = None if last_match is None else now_mono - last_match

        transform = None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=self._lookup_timeout))
        except TransformException:
            pass

        tf_age = None
        position = orientation = None
        position_delta = orientation_delta = 0.0
        if transform is not None:
            tf_stamp_sec = self._stamp_seconds(transform.header.stamp)
            tf_age = sample_age(now_sec, tf_stamp_sec)
            position, orientation = self._transform_values(transform)
            with self._lock:
                stamp_key = (
                    transform.header.stamp.sec,
                    transform.header.stamp.nanosec)
                if stamp_key != self._previous_tf_stamp:
                    position_delta, orientation_delta = pose_jump(
                        self._previous_tf_position,
                        self._previous_tf_orientation,
                        position,
                        orientation)
                    self._jump_latched = (
                        position_delta > self._max_position_jump or
                        orientation_delta > self._max_orientation_jump)
                    # Updating after a rejected jump permits recovery only
                    # after a subsequent distinct, stable TF source sample.
                    self._previous_tf_position = position
                    self._previous_tf_orientation = orientation
                    self._previous_tf_stamp = stamp_key
                elif self._jump_latched:
                    position_delta = math.inf

        vio_age = None
        frame_valid = False
        data_valid = False
        if vslam is not None:
            vio_age = sample_age(
                now_sec, self._stamp_seconds(vslam.header.stamp))
            frame_valid = vslam.child_frame_id == self._base_frame
            if position is not None:
                data_valid = self._finite_message_data(
                    position, orientation, vslam)

        health = assess_health(
            tf_age, vio_age, position_delta, orientation_delta,
            self._tf_timeout, self._vio_timeout,
            self._max_position_jump, self._max_orientation_jump,
            self._future_tolerance, data_valid, frame_valid)
        if health.healthy:
            localization_age = None
            localization_variance = math.inf
            if localization is not None:
                localization_age = sample_age(
                    now_sec,
                    self._stamp_seconds(localization.header.stamp))
                covariance = localization.pose.covariance
                localization_variance = max(
                    covariance[0], covariance[7], covariance[35])
            if localization_age is None:
                health = RelayHealth(
                    health.source, False, health.age,
                    'missing_localization_pose')
            elif localization_age > self._localization_timeout:
                health = RelayHealth(
                    health.source, False,
                    max(health.age, localization_age),
                    'localization_pose_stale')
            elif (
                    not math.isfinite(localization_variance) or
                    localization_variance >
                    self._max_localization_variance):
                health = RelayHealth(
                    health.source, False, health.age,
                    'localization_covariance_untrusted')
            else:
                match_reason = assess_geometric_match(
                    self._require_geometric_match,
                    match_age,
                    self._match_hold_timeout)
                if match_reason is not None:
                    # Degraded mode: publish with inflated covariance instead of
                    # blocking.  The TF pose (initial + odometry) is still usable
                    # for short-term positioning; laser ICP constrains drift.
                    health = RelayHealth(
                        health.source, True,   # still publish
                        max(health.age, match_age or health.age),
                        f'degraded_{match_reason}')
                    self._degraded = True
                    # Use coarse localization variance as floor
                    cov_floor = self._max_localization_variance
                    if math.isfinite(localization_variance):
                        localization_variance = max(localization_variance, cov_floor)
                    else:
                        localization_variance = cov_floor
                else:
                    self._degraded = False
        self._publish_status(
            health, tf_age, vio_age, match_age, loop_id, proximity_id)
        if not health.healthy:
            return

        odom = Odometry()
        # When degraded (no geometric match), use current time so bridge sees
        # fresh samples and doesn't trigger 'sample gap exceeded' reset.
        if getattr(self, '_degraded', False):
            odom.header.stamp = self.get_clock().now().to_msg()
        else:
            odom.header.stamp = transform.header.stamp
        odom.header.frame_id = self._map_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = position[0]
        odom.pose.pose.position.y = position[1]
        odom.pose.pose.position.z = position[2]
        odom.pose.pose.orientation.x = orientation[0]
        odom.pose.pose.orientation.y = orientation[1]
        odom.pose.pose.orientation.z = orientation[2]
        odom.pose.pose.orientation.w = orientation[3]
        odom.pose.covariance = self._pose_covariance
        odom.twist.twist = vslam.twist.twist
        odom.twist.covariance = self._covariance_with_floor(
            vslam.twist.covariance, self._twist_covariance)
        self._odom_pub.publish(odom)


def main(args=None):
    if rclpy is None:
        raise RuntimeError('ROS 2 Python packages are not available')
    rclpy.init(args=args)
    node = RtabmapOdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
