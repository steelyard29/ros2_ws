#!/usr/bin/env python3
"""Detect a QR code below the UAV and publish landing/reconnaissance outputs."""

from __future__ import annotations

import json
import math
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from px4_msgs.msg import VehicleLocalPosition


class QrTargetReport(Node):
    def __init__(self):
        super().__init__("qr_target_report")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("pose_topic", "/landing_target/qr_pose")
        self.declare_parameter("valid_topic", "/landing_target/qr_valid")
        self.declare_parameter("report_topic", "/uav/cooperation/target_report")
        self.declare_parameter("fx", 875.0)
        self.declare_parameter("fy", 875.0)
        self.declare_parameter("forward_pixel_sign", -1.0)
        self.declare_parameter("right_pixel_sign", 1.0)
        self.declare_parameter("process_period_s", 0.15)
        self.declare_parameter("stale_timeout_s", 0.5)

        self._fx = float(self.get_parameter("fx").value)
        self._fy = float(self.get_parameter("fy").value)
        self._forward_sign = float(
            self.get_parameter("forward_pixel_sign").value)
        self._right_sign = float(
            self.get_parameter("right_pixel_sign").value)
        self._period = float(self.get_parameter("process_period_s").value)
        self._stale_timeout = float(
            self.get_parameter("stale_timeout_s").value)

        self._bridge = CvBridge()
        self._detector = cv2.QRCodeDetector()
        self._last_process = 0.0
        self._height = math.nan
        self._height_at = 0.0

        self._pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter("pose_topic").value, 10)
        self._valid_pub = self.create_publisher(
            Bool, self.get_parameter("valid_topic").value, 10)
        self._report_pub = self.create_publisher(
            String, self.get_parameter("report_topic").value, 10)
        self._image_sub = self.create_subscription(
            Image, self.get_parameter("image_topic").value,
            self._image_cb, qos_profile_sensor_data)
        self._local_sub = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position",
            self._local_cb, qos_profile_sensor_data)

    def _local_cb(self, msg):
        if msg.dist_bottom_valid and math.isfinite(msg.dist_bottom):
            self._height = float(msg.dist_bottom)
            self._height_at = time.monotonic()

    def _image_cb(self, msg):
        now = time.monotonic()
        if now - self._last_process < self._period:
            return
        self._last_process = now
        if (
            not math.isfinite(self._height)
            or now - self._height_at > self._stale_timeout
        ):
            self._valid_pub.publish(Bool(data=False))
            return

        image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        text, points, _ = self._detector.detectAndDecode(image)
        if points is None or not text:
            self._valid_pub.publish(Bool(data=False))
            return

        corners = points.reshape(-1, 2)
        center_x = float(corners[:, 0].mean())
        center_y = float(corners[:, 1].mean())
        image_height, image_width = image.shape[:2]
        right_m = (
            self._right_sign * (center_x - image_width * 0.5)
            * self._height / self._fx
        )
        forward_m = (
            self._forward_sign * (center_y - image_height * 0.5)
            * self._height / self._fy
        )

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "base_link"
        pose.pose.position.x = forward_m
        pose.pose.position.y = right_m
        pose.pose.position.z = self._height
        pose.pose.orientation.w = 1.0
        self._pose_pub.publish(pose)
        self._valid_pub.publish(Bool(data=True))

        report = {
            "schema": "uav_target_report/v1",
            "stamp_ns": int(self.get_clock().now().nanoseconds),
            "target_type": "qr",
            "target_id": text,
            "frame_id": "base_link",
            "forward_m": forward_m,
            "right_m": right_m,
            "height_m": self._height,
        }
        self._report_pub.publish(
            String(data=json.dumps(report, ensure_ascii=False)))


def main():
    rclpy.init()
    node = QrTargetReport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
