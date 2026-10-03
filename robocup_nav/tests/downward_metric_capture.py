"""ONE explicitly authorized <=60s downward-only capture; no hardware retry.

Saves original compressed frames for continuous corner tracking. Does not
calibrate/write a camera, publish PX4 input, or start other sensors.
"""
import os
os.environ['ROS_DOMAIN_ID'] = '179'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import json
import argparse
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, CameraInfo

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=int, default=60)
    args = parser.parse_args()
    if not 10 <= args.duration <= 60:
        parser.error('one authorized capture must be bounded to 10..60 seconds')
    duration = args.duration
    out = ROOT/'evidence'/time.strftime('downward_metric_%Y%m%d_%H%M%S')
    out.mkdir(exist_ok=False)
    (out/'frames').mkdir()
    rclpy.init()
    node = rclpy.create_node('downward_metric_recorder')
    rows = []
    info = []
    proc = None
    start = time.monotonic()
    report = dict(passed=False, duration_limit_s=duration, retry=False, flight_authorized=False,
                  only_downward_camera=True, commanded_motion=False)

    def image(msg):
        at = time.monotonic()
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        filename = 'frames/%06d.jpg' % len(rows)
        (out/filename).write_bytes(bytes(msg.data))
        rows.append(dict(at=at, relative_s=at-start, stamp=stamp,
            age_s=node.get_clock().now().nanoseconds/1e9-stamp, file=filename, format=msg.format))
        if len(rows) == 50:
            print(json.dumps(dict(event='BASELINE_READY', frames=len(rows),
                baseline=str(out/filename), remaining_s=max(0., duration-(at-start)))), flush=True)

    def calibration(msg):
        if not info:
            info.append(dict(width=msg.width, height=msg.height, k=list(msg.k), d=list(msg.d),
                             distortion_model=msg.distortion_model))

    node.create_subscription(CompressedImage, '/robocup/downward/image_raw/compressed', image, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/robocup/downward/camera_info', calibration, qos_profile_sensor_data)
    command = ['timeout', '--signal=INT', '--kill-after=5s', str(duration)+'s', 'ros2', 'run',
        'v4l2_camera', 'v4l2_camera_node', '--ros-args', '-r', '__node:=robocup_downward_metric',
        '-p', 'video_device:=/dev/v4l/by-id/usb-LRCP_H-720P_LRCP_H-720P_SN0001-video-index0',
        '-p', 'image_size:=[1280,720]', '-p', 'pixel_format:=YUYV', '-p', 'time_per_frame:=[1,10]',
        '-p', 'output_encoding:=rgb8', '-p', 'camera_frame_id:=downward_optical_unverified',
        '-p', 'camera_info_url:='+(ROOT/'config/downward_camera_info.yaml').as_uri(),
        '-r', 'image_raw:=/robocup/downward/image_raw',
        '-r', 'image_raw/compressed:=/robocup/downward/image_raw/compressed',
        '-r', 'camera_info:=/robocup/downward/camera_info']
    print(json.dumps(dict(event='STARTING_ONE_AUTHORIZED_CAPTURE', evidence=str(out))), flush=True)
    try:
        with (out/'camera.log').open('w') as log:
            start = time.monotonic()
            proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        next_audit = start
        while time.monotonic()-start < duration:
            rclpy.spin_once(node, timeout_sec=.02)
            if time.monotonic() >= next_audit:
                graph = node.get_topic_names_and_types()
                forbidden = [n for n, _ in graph if n.startswith(('/fmu/', '/servo/', '/visual_slam/', '/camera/camera/', '/scan'))]
                if forbidden:
                    raise RuntimeError('unexpected topics in camera-only domain: '+str(forbidden))
                next_audit = time.monotonic()+1.
            if proc.poll() is not None and time.monotonic()-start < duration-1:
                raise RuntimeError('camera driver exited before capture completed; no retry')
        report['passed'] = len(rows) >= 50 and bool(info)
        report['pass_scope'] = 'capture availability only; displacement analysis separate'
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=5)
        report.update(frames=len(rows), camera_info=info, process_exit=proc.poll() if proc else None)
        (out/'frames.json').write_text(json.dumps(rows))
        (out/'report.json').write_text(json.dumps(report, indent=2))
        node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps(dict(event='CAPTURE_STOPPED', evidence=str(out), report=report)), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
