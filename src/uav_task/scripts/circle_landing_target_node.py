#!/usr/bin/env python3
"""ROS 2 detector for a known-size circular landing target."""

from __future__ import annotations

import contextlib
import io
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

_import_stderr = io.StringIO()
try:
    with contextlib.redirect_stderr(_import_stderr):
        from cv_bridge import CvBridge
        import cv2
        import numpy as np
except Exception as exc:  # pragma: no cover - depends on deployed Python env
    CvBridge = None
    cv2 = None
    np = None
    CV_IMPORT_ERROR = exc
else:
    CV_IMPORT_ERROR = None


@dataclass(frozen=True)
class Detection:
    center_x: float
    center_y: float
    major_axis_px: float
    minor_axis_px: float
    angle_deg: float
    contour_area_px: float


@dataclass(frozen=True)
class Calibration:
    fx: float | None
    fy: float | None
    image_width: float | None = None
    image_height: float | None = None
    distance_scale: float = 1.0


def focal_from_fov(image_size_px: float, fov_deg: float) -> float:
    return image_size_px / (2.0 * math.tan(math.radians(fov_deg) / 2.0))


def load_calibration(path: Path) -> Calibration:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)

    image_width = data.get("image_width") or data.get("calibration_image_width")
    image_height = data.get("image_height") or data.get("calibration_image_height")
    image_width = float(image_width) if image_width is not None else None
    image_height = float(image_height) if image_height is not None else None
    distance_scale = float(data.get("distance_scale", 1.0))

    if "fx" in data or "fy" in data:
        return Calibration(
            fx=float(data["fx"]) if "fx" in data else None,
            fy=float(data["fy"]) if "fy" in data else None,
            image_width=image_width,
            image_height=image_height,
            distance_scale=distance_scale,
        )

    matrix = data.get("camera_matrix") or data.get("K")
    if matrix is None:
        focal_length_mm = data.get("focal_length_mm")
        sensor_width_mm = data.get("sensor_width_mm")
        sensor_height_mm = data.get("sensor_height_mm")
        if focal_length_mm and sensor_width_mm and sensor_height_mm and image_width and image_height:
            return Calibration(
                fx=float(focal_length_mm) / float(sensor_width_mm) * image_width,
                fy=float(focal_length_mm) / float(sensor_height_mm) * image_height,
                image_width=image_width,
                image_height=image_height,
                distance_scale=distance_scale,
            )
        raise ValueError("calibration must contain fx/fy, camera_matrix/K, or physical camera fields")

    arr = np.asarray(matrix, dtype=float)
    if arr.shape != (3, 3):
        raise ValueError("camera_matrix/K must be 3x3")
    return Calibration(
        fx=float(arr[0, 0]),
        fy=float(arr[1, 1]),
        image_width=image_width,
        image_height=image_height,
        distance_scale=distance_scale,
    )


def scaled_focal_candidates(calibration: Calibration, image_width: int, image_height: int) -> list[float]:
    if not calibration.image_width or not calibration.image_height:
        return [value * calibration.distance_scale for value in (calibration.fx, calibration.fy) if value]

    direct_error = abs((image_width / image_height) - (calibration.image_width / calibration.image_height))
    rotated_error = abs((image_width / image_height) - (calibration.image_height / calibration.image_width))
    if rotated_error < direct_error:
        scale_x = image_width / calibration.image_height
        scale_y = image_height / calibration.image_width
    else:
        scale_x = image_width / calibration.image_width
        scale_y = image_height / calibration.image_height

    candidates: list[float] = []
    if calibration.fx:
        candidates.append(calibration.fx * scale_x * calibration.distance_scale)
    if calibration.fy:
        candidates.append(calibration.fy * scale_y * calibration.distance_scale)
    return candidates


def build_blue_mask(image_bgr: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    blue_mask = cv2.inRange(hsv, (85, 25, 25), (130, 255, 255))
    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel_close, iterations=1)
    return cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel_open, iterations=1)


def build_target_mask(image_bgr: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    blue_mask = cv2.inRange(hsv, (85, 35, 35), (125, 255, 255))
    dark_mask = cv2.inRange(gray, 0, 95)
    mask = cv2.bitwise_or(blue_mask, dark_mask)
    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (35, 35))
    kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=2)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open, iterations=1)


def contour_score(contour: np.ndarray, image_shape: tuple[int, int, int], min_area_ratio: float):
    if len(contour) < 5:
        return None
    height, width = image_shape[:2]
    image_area = width * height
    area = float(cv2.contourArea(contour))
    if area < image_area * min_area_ratio:
        return None

    (cx, cy), (axis_a, axis_b), angle = cv2.fitEllipse(contour)
    major = float(max(axis_a, axis_b))
    minor = float(min(axis_a, axis_b))
    if major <= 0 or minor <= 0 or minor / major < 0.45:
        return None

    ellipse_area = math.pi * (major / 2.0) * (minor / 2.0)
    fill_ratio = area / ellipse_area if ellipse_area else 0.0
    if not 0.35 <= fill_ratio <= 1.4:
        return None

    score = area * (minor / major) * (1.0 + 0.25 * (cy / height))
    return score, Detection(float(cx), float(cy), major, minor, float(angle), area), contour


def detect_target(image_bgr: np.ndarray, min_area_ratio: float) -> Detection:
    height, width = image_bgr.shape[:2]
    image_area = width * height
    blue_mask = build_blue_mask(image_bgr)
    contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    scored = []
    for contour in contours:
        result = contour_score(contour, image_bgr.shape, max(min_area_ratio * 0.35, 0.001))
        if result is None:
            continue
        score, detection, candidate = result
        if detection.center_y < height * 0.35:
            score *= 0.35
        scored.append((score, detection, candidate))

    if not scored:
        raise ValueError("no circular landing target detected")

    scored.sort(key=lambda item: item[0], reverse=True)
    _, blue_detection, blue_contour = scored[0]
    x, y, w, h = cv2.boundingRect(blue_contour)
    fallback_detection = Detection(
        blue_detection.center_x,
        blue_detection.center_y,
        float(max(blue_detection.major_axis_px, w, h)),
        float(min(max(blue_detection.minor_axis_px, min(w, h)), max(blue_detection.major_axis_px, w, h))),
        blue_detection.angle_deg,
        blue_detection.contour_area_px,
    )

    pad = int(max(w, h) * 0.18)
    roi_x = max(0, x - pad)
    roi_y = max(0, y - pad)
    roi_w = min(width - roi_x, w + 2 * pad)
    roi_h = min(height - roi_y, h + 2 * pad)
    roi = image_bgr[roi_y : roi_y + roi_h, roi_x : roi_x + roi_w]
    edges = cv2.Canny(build_target_mask(roi), 50, 150)
    ys, xs = np.where(edges > 0)
    if len(xs) < 5:
        return fallback_detection

    global_xs = xs.astype(np.float32) + roi_x
    global_ys = ys.astype(np.float32) + roi_y
    distances = np.hypot(global_xs - blue_detection.center_x, global_ys - blue_detection.center_y)
    low = np.percentile(distances, 60)
    high = np.percentile(distances, 92)
    points = np.column_stack([global_xs[(distances >= low) & (distances <= high)],
                              global_ys[(distances >= low) & (distances <= high)]]).astype(np.float32)
    if len(points) < 5:
        return fallback_detection

    (cx, cy), (axis_a, axis_b), angle = cv2.fitEllipse(points.reshape(-1, 1, 2))
    major = float(max(axis_a, axis_b))
    minor = float(min(axis_a, axis_b))
    if major > 0 and minor > 0 and minor / major >= 0.55 and major * minor > image_area * 0.02:
        return Detection(float(cx), float(cy), major, minor, float(angle), blue_detection.contour_area_px)
    return fallback_detection


class CircleLandingTargetNode(Node):
    def __init__(self) -> None:
        super().__init__("circle_landing_target")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter(
            "calibration_file",
            "/home/cfly/uav_circle_distance_orin/configs/drone_calibration_fov_stream1_corrected.json",
        )
        self.declare_parameter("real_diameter_cm", 60.0)
        self.declare_parameter("min_area_ratio", 0.01)
        self.declare_parameter("hfov_deg", 0.0)
        self.declare_parameter("vfov_deg", 0.0)
        self.declare_parameter("process_period_s", 0.2)

        self.bridge = CvBridge() if CvBridge is not None else None
        self.latest_image: Image | None = None
        self.latest_stamp = None
        self.processing = False
        self.cv_available = self.bridge is not None and cv2 is not None and np is not None

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.center_pub = self.create_publisher(PointStamped, "/landing_target/circle_center", 10)
        self.distance_pub = self.create_publisher(Float32, "/landing_target/distance_m", 10)
        # Alias used by uav_circle_distance_orin deploy notes
        self.target_distance_pub = self.create_publisher(Float32, "/target_distance_m", 10)
        self.valid_pub = self.create_publisher(Bool, "/landing_target/valid", 10)

        period = self.get_parameter("process_period_s").get_parameter_value().double_value
        self.timer = self.create_timer(period, self.process_latest)
        self.get_logger().info(f"Circle landing target detector subscribed to {image_topic}")
        if not self.cv_available:
            import_detail = _import_stderr.getvalue().strip().splitlines()
            detail = import_detail[-1] if import_detail else str(CV_IMPORT_ERROR)
            self.get_logger().error(
                "OpenCV/NumPy import failed; circle detector will publish valid=false. "
                f"python={sys.executable}, error={CV_IMPORT_ERROR}, detail={detail}"
            )

    def image_callback(self, msg: Image) -> None:
        self.latest_image = msg
        self.latest_stamp = msg.header.stamp

    def resolve_focal_px(self, image_shape: tuple[int, int, int]) -> float:
        height, width = image_shape[:2]
        candidates: list[float] = []
        calibration_file = self.get_parameter("calibration_file").get_parameter_value().string_value
        if calibration_file:
            candidates.extend(scaled_focal_candidates(load_calibration(Path(calibration_file)), width, height))
        hfov = self.get_parameter("hfov_deg").get_parameter_value().double_value
        vfov = self.get_parameter("vfov_deg").get_parameter_value().double_value
        if hfov > 0.0:
            candidates.append(focal_from_fov(width, hfov))
        if vfov > 0.0:
            candidates.append(focal_from_fov(height, vfov))
        if not candidates:
            raise ValueError("camera calibration, hfov_deg, or vfov_deg is required")
        return float(sum(candidates) / len(candidates))

    def publish_valid(self, value: bool) -> None:
        msg = Bool()
        msg.data = value
        self.valid_pub.publish(msg)

    def process_latest(self) -> None:
        if self.processing or self.latest_image is None:
            return
        if not self.cv_available:
            self.publish_valid(False)
            return
        self.processing = True
        try:
            image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
            if image.dtype != np.uint8:
                image = image.astype(np.uint8)
            focal_px = self.resolve_focal_px(image.shape)
            detection = detect_target(
                image,
                self.get_parameter("min_area_ratio").get_parameter_value().double_value,
            )
            real_diameter_cm = self.get_parameter("real_diameter_cm").get_parameter_value().double_value
            distance_m = (focal_px * real_diameter_cm / detection.major_axis_px) / 100.0

            center = PointStamped()
            center.header.stamp = self.latest_stamp
            center.header.frame_id = self.latest_image.header.frame_id or "camera"
            center.point.x = detection.center_x
            center.point.y = detection.center_y
            center.point.z = detection.major_axis_px
            self.center_pub.publish(center)

            distance = Float32()
            distance.data = float(distance_m)
            self.distance_pub.publish(distance)
            self.target_distance_pub.publish(distance)
            self.publish_valid(True)
            self.get_logger().info(
                f"circle target center=({detection.center_x:.1f},{detection.center_y:.1f}) "
                f"axis={detection.major_axis_px:.1f}px distance={distance_m:.3f}m",
                throttle_duration_sec=1.0,
            )
        except Exception as exc:
            self.publish_valid(False)
            self.get_logger().warn(f"circle target unavailable: {exc}", throttle_duration_sec=2.0)
        finally:
            self.processing = False


def main() -> None:
    rclpy.init()
    node = CircleLandingTargetNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
