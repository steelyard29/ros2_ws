"""Bounded image-only subscriber; no flight output or camera startup."""
import json
import time
import numpy as np
import yaml
from pathlib import Path
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node('downward_snapshot')
    frames = []
    info = []
    detections = []
    ages=[]
    def receive(m):
        frames.append((time.monotonic(), m))
        ages.append((node.get_clock().now().nanoseconds-
                     (m.header.stamp.sec*10**9+m.header.stamp.nanosec))/1e9)
    node.create_subscription(Image, '/robocup/downward/image_raw',
                             receive, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/robocup/downward/camera_info',
                             lambda m: info.append(m), qos_profile_sensor_data)
    node.create_subscription(String, '/robocup/landing/h_candidate',
                             lambda m: detections.append(json.loads(m.data)), 10)
    end = time.monotonic() + 10
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
    try:
        if not frames:
            raise RuntimeError('No downward images received')
        out = Path(__file__).resolve().parents[1] / 'evidence' / time.strftime('downward_%Y%m%d_%H%M%S')
        out.mkdir(exist_ok=False)
        image = CvBridge().imgmsg_to_cv2(frames[-1][1], desired_encoding='bgr8')
        if not cv2.imwrite(str(out / 'image.png'), image):
            raise RuntimeError('Image write failed')
        span = frames[-1][0] - frames[0][0]
        calibration=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/downward_camera_info.yaml').read_text())
        last=info[-1] if info else None
        matching=bool(last and last.width==frames[-1][1].width==calibration['image_width']
            and last.height==frames[-1][1].height==calibration['image_height']
            and np.allclose(last.k,calibration['camera_matrix']['data'])
            and len(last.d)==len(calibration['distortion_coefficients']['data'])
            and np.allclose(last.d,calibration['distortion_coefficients']['data'])
            and last.distortion_model==calibration['distortion_model'])
        report = {'frames': len(frames), 'hz': (len(frames)-1)/span if span else 0,
                  'size': [frames[-1][1].width, frames[-1][1].height],
                  'calibrated_intrinsics_present': bool(info and info[-1].k[0] > 0),
                  'camera_info_matches_selected_calibration':matching,
                  'camera_info_k':list(last.k) if last else None,
                  'camera_info_d':list(last.d) if last else None,
                  'image_age_seconds_min_p50_p95_max':np.quantile(ages,[0,.5,.95,1]).tolist(),
                  'maximum_receive_gap_seconds':max((b[0]-a[0] for a,b in zip(frames,frames[1:])),default=0),
                  'h_status_messages': len(detections),
                  'h_stable_messages': sum(bool(m.get('candidate_stable')) for m in detections),
                  'last_h_status': detections[-1] if detections else None,
                  'flight_validated': False}
        (out / 'report.json').write_text(json.dumps(report, indent=2))
        print(out, report)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
