#!/usr/bin/env python3
"""Subscription-only left/right IR snapshot in isolated camera domain174.

Does not start cameras, publish, or access flight controllers. Saves raw
mono8 pixels for human scene inspection; not a feature-quality certificate.
"""
import os
os.environ['ROS_DOMAIN_ID'] = '174'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import json
from pathlib import Path
import time
import uuid
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    rclpy.init(args=[])
    node = Node('camera_view_snapshot', enable_rosout=False, start_parameter_services=False)
    frames = {}
    metadata = {}

    def receive(key, msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age = (node.get_clock().now().nanoseconds-stamp)/1e9
        if (key in frames or msg.encoding != 'mono8' or not -.05 <= age <= .3
                or msg.width <= 0 or msg.height <= 0 or msg.step < msg.width
                or len(msg.data) < msg.step*msg.height):
            return
        frames[key] = np.ndarray((msg.height, msg.width), dtype=np.uint8,
                                buffer=msg.data, strides=(msg.step, 1)).copy()
        metadata[key] = {'timestamp_ns': stamp, 'age_s': age, 'frame_id': msg.header.frame_id}

    for key, index in (('left', 1), ('right', 2)):
        node.create_subscription(Image, f'/camera/camera/infra{index}/image_rect_raw',
                                 lambda m, k=key: receive(k, m), qos_profile_sensor_data)
    try:
        end = time.monotonic()+15
        while len(frames) < 2 and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        if len(frames) != 2:
            raise RuntimeError('fresh stereo snapshot unavailable; camera not started by this tool')
        out = Path(__file__).resolve().parents[1]/'evidence'/(
            time.strftime('camera_view_%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:6])
        out.mkdir(parents=True, exist_ok=False)
        for key, pixels in frames.items():
            if not cv2.imwrite(str(out/(key+'.png')), pixels):
                raise RuntimeError('snapshot write failed')
        (out/'report.json').write_text(json.dumps({'subscription_only': True,
            'domain_id': 174, 'flight_approved': False, 'frames': metadata}, indent=2)+'\n')
        print(str(out), flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
