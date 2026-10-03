#!/usr/bin/env python3
"""Publish onboard air-ground cooperation reports for a UGV consumer.

Inputs (optional, best-effort):
  /uav/cooperation/target_report   String JSON from qr_target_report_node
  /yolo/scene/name                 String
  /yolo/scene/confidence           Float32
  /uav/state/pose                  PoseStamped ENU

Output:
  /uav/cooperation/ugv_assist      String JSON with shared-frame pose + target
"""

from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32, String


class AirGroundCoop(Node):
    def __init__(self):
        super().__init__("air_ground_coop")
        self.declare_parameter("report_hz", 2.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter(
            "output_topic", "/uav/cooperation/ugv_assist")
        self.declare_parameter(
            "qr_topic", "/uav/cooperation/target_report")
        self.declare_parameter("yolo_name_topic", "/yolo/scene/name")
        self.declare_parameter(
            "yolo_confidence_topic", "/yolo/scene/confidence")
        self.declare_parameter("pose_topic", "/uav/state/pose")

        self._pose = None
        self._pose_at = 0.0
        self._qr = None
        self._qr_at = 0.0
        self._yolo_name = ""
        self._yolo_conf = 0.0
        self._yolo_at = 0.0
        self._frame = self.get_parameter("frame_id").value

        self._pub = self.create_publisher(
            String, self.get_parameter("output_topic").value, 10)
        self.create_subscription(
            PoseStamped, self.get_parameter("pose_topic").value,
            self._pose_cb, 10)
        self.create_subscription(
            String, self.get_parameter("qr_topic").value,
            self._qr_cb, 10)
        self.create_subscription(
            String, self.get_parameter("yolo_name_topic").value,
            self._yolo_name_cb, 10)
        self.create_subscription(
            Float32, self.get_parameter("yolo_confidence_topic").value,
            self._yolo_conf_cb, 10)

        period = 1.0 / max(
            0.2, float(self.get_parameter("report_hz").value))
        self.create_timer(period, self._tick)
        self.get_logger().info(
            "air_ground_coop publishing "
            f"{self.get_parameter('output_topic').value}")

    def _pose_cb(self, msg: PoseStamped):
        self._pose = msg
        self._pose_at = time.monotonic()

    def _qr_cb(self, msg: String):
        try:
            self._qr = json.loads(msg.data)
            self._qr_at = time.monotonic()
        except json.JSONDecodeError:
            self.get_logger().warn("invalid QR report JSON")

    def _yolo_name_cb(self, msg: String):
        self._yolo_name = msg.data
        self._yolo_at = time.monotonic()

    def _yolo_conf_cb(self, msg: Float32):
        self._yolo_conf = float(msg.data)
        self._yolo_at = time.monotonic()

    def _tick(self):
        now = time.monotonic()
        report = {
            "schema": "uav_ugv_assist/v1",
            "stamp_ns": int(self.get_clock().now().nanoseconds),
            "frame_id": self._frame,
            "uav_pose_valid": False,
            "targets": [],
        }
        if self._pose is not None and now - self._pose_at <= 1.0:
            p = self._pose.pose.position
            report["uav_pose_valid"] = True
            report["uav_x"] = float(p.x)
            report["uav_y"] = float(p.y)
            report["uav_z"] = float(p.z)
        if self._qr is not None and now - self._qr_at <= 1.0:
            report["targets"].append({
                "type": "qr",
                "id": self._qr.get("target_id"),
                "forward_m": self._qr.get("forward_m"),
                "right_m": self._qr.get("right_m"),
                "height_m": self._qr.get("height_m"),
                "confidence": 1.0,
            })
        if self._yolo_name and now - self._yolo_at <= 2.0:
            report["targets"].append({
                "type": "yolo_scene",
                "id": self._yolo_name,
                "confidence": self._yolo_conf,
            })
        if not report["uav_pose_valid"] and not report["targets"]:
            return
        self._pub.publish(String(data=json.dumps(report, ensure_ascii=False)))


def main():
    rclpy.init()
    node = AirGroundCoop()
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
