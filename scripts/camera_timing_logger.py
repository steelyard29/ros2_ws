#!/usr/bin/env python3
"""Record only camera/IMU header timing; never copy image payloads.

This is intentionally separate from rosbag recording.  Writing raw D435
frames can perturb the USB/ROS pipeline and create artificial frame drops.
"""

import argparse
import csv
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu


class CameraTimingLogger(Node):
    def __init__(self, output: str):
        super().__init__('camera_timing_logger')
        self._file = open(output, 'w', newline='')
        self._closed = False
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            'recv_time_ns', 'topic', 'header_stamp_ns', 'frame_id',
            'width', 'height',
        ])
        self._subs = [
            self.create_subscription(
                Image, '/camera/camera/infra1/image_rect_raw',
                lambda msg: self._image_cb('/camera/camera/infra1/image_rect_raw', msg),
                qos_profile_sensor_data),
            self.create_subscription(
                Image, '/camera/camera/infra2/image_rect_raw',
                lambda msg: self._image_cb('/camera/camera/infra2/image_rect_raw', msg),
                qos_profile_sensor_data),
            self.create_subscription(
                Imu, '/camera/camera/imu',
                lambda msg: self._imu_cb('/camera/camera/imu', msg),
                qos_profile_sensor_data),
        ]

    @staticmethod
    def _stamp(msg):
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _image_cb(self, topic, msg):
        self._writer.writerow([
            time.time_ns(), topic, self._stamp(msg), msg.header.frame_id,
            msg.width, msg.height,
        ])
        self._file.flush()

    def _imu_cb(self, topic, msg):
        self._writer.writerow([
            time.time_ns(), topic, self._stamp(msg), msg.header.frame_id, '', '',
        ])
        # Flush less frequently for the 200 Hz IMU stream.
        if int(msg.header.stamp.nanosec) % 50_000_000 < 5_000_000:
            self._file.flush()

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._file.flush()
        self._file.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = CameraTimingLogger(args.output)

    def stop(_signum, _frame):
        node.close()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.close()


if __name__ == '__main__':
    main()
