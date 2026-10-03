#!/usr/bin/env python3
"""
T265 → /visual_slam/tracking/odometry 适配器（不改 D435i / Isaac VSLAM 链路）

优先顺序:
  1) 订阅已有 T265 里程计/位姿话题（外部旧版 realsense-ros / 自建驱动）
  2) 若安装了带 TM2 的 pyrealsense2，则直接读 pose 流

输出与 Isaac VSLAM 对齐，供现有 vslam_odom_bridge 使用:
  - topic: /visual_slam/tracking/odometry
  - frame_id: odom
  - child_frame_id: base_link
  - 位姿已补偿 base_link→camera 外参（默认前 0.15m、下 0.04m）

用法（一般由 t265_px4.launch.py 拉起）:
  python3 t265_odom_publisher.py
"""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_ros import TransformBroadcaster


def _quat_multiply(q1, q2):
    """Hamilton product, quats as (x,y,z,w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _quat_rotate(q, v):
    """Rotate vector v by quat q (x,y,z,w)."""
    x, y, z, w = q
    qv = (v[0], v[1], v[2], 0.0)
    q_conj = (-x, -y, -z, w)
    out = _quat_multiply(_quat_multiply(q, qv), q_conj)
    return np.array([out[0], out[1], out[2]], dtype=float)


def _rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class T265OdomPublisher(Node):
    def __init__(self):
        super().__init__('t265_odom_publisher')

        self.declare_parameter('output_odom_topic', '/visual_slam/tracking/odometry')
        self.declare_parameter('input_odom_topic', '/t265/odom')
        self.declare_parameter('input_pose_topic', '/camera/pose/sample')
        self.declare_parameter('camera_x', 0.15)
        self.declare_parameter('camera_y', 0.0)
        self.declare_parameter('camera_z', -0.04)
        self.declare_parameter('camera_roll', 0.0)
        self.declare_parameter('camera_pitch', 0.0)
        self.declare_parameter('camera_yaw', 0.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_pose_frame')
        self.declare_parameter('try_pyrealsense', True)
        self.declare_parameter('publish_rate_hz', 50.0)

        self._out_topic = self.get_parameter('output_odom_topic').value
        self._in_odom = self.get_parameter('input_odom_topic').value
        self._in_pose = self.get_parameter('input_pose_topic').value
        self._t_bc = np.array([
            float(self.get_parameter('camera_x').value),
            float(self.get_parameter('camera_y').value),
            float(self.get_parameter('camera_z').value),
        ], dtype=float)
        self._q_bc = _rpy_to_quat(
            float(self.get_parameter('camera_roll').value),
            float(self.get_parameter('camera_pitch').value),
            float(self.get_parameter('camera_yaw').value),
        )
        # camera→base = inverse of base→camera (translation in cam frame)
        self._q_cb = (-self._q_bc[0], -self._q_bc[1], -self._q_bc[2], self._q_bc[3])
        self._t_cb = -_quat_rotate(self._q_cb, self._t_bc)

        self._world = self.get_parameter('world_frame').value
        self._base = self.get_parameter('base_frame').value
        self._cam = self.get_parameter('camera_frame').value
        self._publish_tf = bool(self.get_parameter('publish_tf').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(Odometry, self._out_topic, qos)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        self._got_ros_input = False
        self._rs_pipeline = None
        self._rs = None

        self.create_subscription(Odometry, self._in_odom, self._on_odom, qos)
        self.create_subscription(PoseStamped, self._in_pose, self._on_pose, qos)

        if bool(self.get_parameter('try_pyrealsense').value):
            self._try_start_pyrealsense()

        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self._on_timer)
        self.create_timer(5.0, self._warn_if_idle)
        self._last_pub_mono = 0.0

        self.get_logger().info(
            f'T265 odom adapter → {self._out_topic}; '
            f'extrinsic base←cam xyz=({self._t_bc[0]:.3f},{self._t_bc[1]:.3f},{self._t_bc[2]:.3f}); '
            f'listen {self._in_odom} / {self._in_pose}'
        )

    def _try_start_pyrealsense(self):
        try:
            import pyrealsense2 as rs  # type: ignore
        except Exception as exc:
            self.get_logger().warn(
                f'pyrealsense2 不可用 ({exc}); 仅订阅外部 T265 话题。'
            )
            return
        try:
            pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.pose)
            pipe.start(cfg)
            self._rs = rs
            self._rs_pipeline = pipe
            self.get_logger().info('已用 pyrealsense2 打开 T265 pose 流')
        except Exception as exc:
            self.get_logger().warn(
                f'无法用当前 librealsense 打开 T265 pose ({exc}). '
                f'请改用带 TM2 的旧版驱动，并发布到 {self._in_odom} 或 {self._in_pose}。'
            )
            self._rs_pipeline = None

    def _cam_pose_to_base(
        self,
        p_cam: np.ndarray,
        q_cam,
        v_cam: Optional[np.ndarray] = None,
        w_cam: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, tuple, np.ndarray, np.ndarray]:
        # T_wb = T_wc * T_cb
        p_base = p_cam + _quat_rotate(q_cam, self._t_cb)
        q_base = _quat_multiply(q_cam, self._q_cb)
        if v_cam is None:
            v_base = np.zeros(3)
        else:
            # ignore omega×r for placeholder; T265 body rates usually small at hover
            v_base = _quat_rotate(self._q_cb, v_cam)
        if w_cam is None:
            w_base = np.zeros(3)
        else:
            w_base = _quat_rotate(self._q_cb, w_cam)
        return p_base, q_base, v_base, w_base

    def _publish_base_odom(self, stamp, p_base, q_base, v_base, w_base):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._world
        msg.child_frame_id = self._base
        msg.pose.pose.position.x = float(p_base[0])
        msg.pose.pose.position.y = float(p_base[1])
        msg.pose.pose.position.z = float(p_base[2])
        msg.pose.pose.orientation.x = float(q_base[0])
        msg.pose.pose.orientation.y = float(q_base[1])
        msg.pose.pose.orientation.z = float(q_base[2])
        msg.pose.pose.orientation.w = float(q_base[3])
        msg.twist.twist.linear.x = float(v_base[0])
        msg.twist.twist.linear.y = float(v_base[1])
        msg.twist.twist.linear.z = float(v_base[2])
        msg.twist.twist.angular.x = float(w_base[0])
        msg.twist.twist.angular.y = float(w_base[1])
        msg.twist.twist.angular.z = float(w_base[2])
        # Diagonal covariances; bridge overwrites PX4 variances anyway
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[14] = 0.04
        msg.pose.covariance[21] = 0.05
        msg.pose.covariance[28] = 0.05
        msg.pose.covariance[35] = 0.08
        self._pub.publish(msg)
        self._last_pub_mono = time.monotonic()

        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._world
            tf.child_frame_id = self._base
            tf.transform.translation.x = float(p_base[0])
            tf.transform.translation.y = float(p_base[1])
            tf.transform.translation.z = float(p_base[2])
            tf.transform.rotation.x = float(q_base[0])
            tf.transform.rotation.y = float(q_base[1])
            tf.transform.rotation.z = float(q_base[2])
            tf.transform.rotation.w = float(q_base[3])
            self._tf_broadcaster.sendTransform(tf)

    def _on_odom(self, msg: Odometry):
        self._got_ros_input = True
        p = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ], dtype=float)
        q = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        v = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ], dtype=float)
        w = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ], dtype=float)
        # If upstream already claims base_link, still apply extrinsic only when
        # child is camera frame; otherwise pass through.
        child = (msg.child_frame_id or '').strip()
        if child in ('', self._cam, 'camera_pose_frame', 'camera_link'):
            p, q, v, w = self._cam_pose_to_base(p, q, v, w)
        self._publish_base_odom(msg.header.stamp, p, q, v, w)

    def _on_pose(self, msg: PoseStamped):
        self._got_ros_input = True
        p = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=float)
        q = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        p, q, v, w = self._cam_pose_to_base(p, q)
        self._publish_base_odom(msg.header.stamp, p, q, v, w)

    def _on_timer(self):
        if self._rs_pipeline is None or self._got_ros_input:
            return
        try:
            frames = self._rs_pipeline.wait_for_frames(timeout_ms=50)
            pose = frames.get_pose_frame()
            if not pose:
                return
            data = pose.get_pose_data()
            # librealsense T265: translation meters, rotation as x,y,z,w
            p = np.array([
                data.translation.x,
                data.translation.y,
                data.translation.z,
            ], dtype=float)
            q = (
                data.rotation.x,
                data.rotation.y,
                data.rotation.z,
                data.rotation.w,
            )
            v = np.array([
                data.velocity.x,
                data.velocity.y,
                data.velocity.z,
            ], dtype=float)
            w = np.array([
                data.angular_velocity.x,
                data.angular_velocity.y,
                data.angular_velocity.z,
            ], dtype=float)
            p, q, v, w = self._cam_pose_to_base(p, q, v, w)
            stamp = self.get_clock().now().to_msg()
            self._publish_base_odom(stamp, p, q, v, w)
        except Exception:
            return

    def _warn_if_idle(self):
        if time.monotonic() - self._last_pub_mono > 3.0:
            self.get_logger().warn(
                '仍无 T265 里程计输出。请确认: '
                '(1) USB 已接 T265 (8086:0b37); '
                '(2) 旧版 TM2 librealsense/realsense-ros 在发布 '
                f'{self._in_odom} 或 {self._in_pose}; '
                '(3) 当前仓库 librealsense 2.58 默认不含 T265。'
            )

    def destroy_node(self):
        if self._rs_pipeline is not None:
            try:
                self._rs_pipeline.stop()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = T265OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
