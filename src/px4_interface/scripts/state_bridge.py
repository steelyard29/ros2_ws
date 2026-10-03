#!/usr/bin/env python3
"""
state_bridge.py — Layer 1 标准化状态输出节点

将 PX4 原始数据 (NED 坐标系, /fmu/out/*) 转换为
Layer 2/3 可用的标准化接口 (ENU 坐标系, /uav/state/*)。

发布:
  /uav/state/pose   — geometry_msgs/PoseStamped (ENU 坐标系)
  /uav/state/status — px4_interface/VehicleStatus (标准化飞控状态)

订阅:
  /fmu/out/vehicle_odometry       — px4_msgs/VehicleOdometry
  /fmu/out/vehicle_local_position — px4_msgs/VehicleLocalPosition (fallback)
  /fmu/out/vehicle_status_v1      — px4_msgs/VehicleStatus
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from px4_msgs.msg import (
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleStatus as PX4VehicleStatus,
)
from px4_interface.msg import VehicleStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class StateBridge(Node):
    """PX4 原始数据 → /uav/state 标准化接口"""

    def __init__(self):
        super().__init__('state_bridge')

        # QoS: 与 PX4 一致 (BEST_EFFORT for sensor data)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── 订阅 PX4 原始数据 ──
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._local_pos_cb,
            qos,
        )
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self._odom_cb,
            qos,
        )
        self.status_sub = self.create_subscription(
            PX4VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self._status_cb,
            qos,
        )

        # ── 发布标准化接口 ──
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/uav/state/pose',
            10,
        )
        self.status_pub = self.create_publisher(
            VehicleStatus,
            '/uav/state/status',
            10,
        )

        # ── 内部状态 ──
        self._local_pos = VehicleLocalPosition()
        self._odom = VehicleOdometry()
        self._px4_status = PX4VehicleStatus()
        self._has_pos = False
        self._has_odom = False
        self._local_pos_received_at = 0.0
        self._odom_received_at = 0.0
        self._pose_timeout_s = 0.5

        # 10Hz 定时发布
        self._timer = self.create_timer(0.1, self._publish_state)

        self.get_logger().info(
            'state_bridge 已启动 — 发布 /uav/state/pose, /uav/state/status; '
            'pose 优先使用 /fmu/out/vehicle_odometry，local_position 作为回退')

    # ── 回调 ────────────────────────────────────────────────

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self._local_pos = msg
        self._has_pos = True
        self._local_pos_received_at = time.monotonic()

    def _odom_cb(self, msg: VehicleOdometry):
        self._odom = msg
        self._has_odom = True
        self._odom_received_at = time.monotonic()

    def _status_cb(self, msg: PX4VehicleStatus):
        self._px4_status = msg

    # ── 定时发布 ────────────────────────────────────────────

    def _publish_state(self):
        now = self.get_clock().now().to_msg()

        # 发布 ENU 位姿 (NED → ENU 转换)。只发布新鲜数据，避免上层控制使用旧高度。
        source = self._select_pose_source()
        if source is not None:
            pose = PoseStamped()
            pose.header = Header(stamp=now, frame_id='map')
            if source == 'odometry':
                ned_x = self._odom.position[0]
                ned_y = self._odom.position[1]
                ned_z = self._odom.position[2]
            else:
                ned_x = self._local_pos.x
                ned_y = self._local_pos.y
                ned_z = self._local_pos.z
            pose.pose.position.x = float(ned_y)
            pose.pose.position.y = float(ned_x)
            pose.pose.position.z = float(-ned_z)
            # 姿态保持单位四元数 (由 PX4 EKF 内部处理)
            pose.pose.orientation.w = 1.0
            self.pose_pub.publish(pose)

        # 发布标准化状态
        status = VehicleStatus()
        status.valid = True
        status.timestamp = now
        status.arming_state = self._px4_status.arming_state
        status.nav_state = self._px4_status.nav_state
        status.failsafe = self._px4_status.failsafe
        status.pre_flight_checks_pass = self._px4_status.pre_flight_checks_pass
        self.status_pub.publish(status)

    def _select_pose_source(self):
        now = time.monotonic()
        odom_fresh = (
            self._has_odom and
            now - self._odom_received_at <= self._pose_timeout_s
        )
        local_pos_fresh = (
            self._has_pos and
            now - self._local_pos_received_at <= self._pose_timeout_s
        )

        if odom_fresh:
            return 'odometry'
        if local_pos_fresh:
            return 'local_position'
        return None


def main():
    rclpy.init()
    node = StateBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
