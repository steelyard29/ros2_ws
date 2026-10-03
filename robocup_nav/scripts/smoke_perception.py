#!/usr/bin/env python3
"""Bounded independent camera/VIO/RGB-D/RTAB-Map smoke test inside container.

No PX4 or actuator nodes. A separate ROS domain, foreground process groups,
bounded wall-clock collection and new evidence directory on every run.
"""
import os
os.environ['ROS_DOMAIN_ID'] = '174'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
from collections import Counter
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image, Imu
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus

ROOT = Path(__file__).resolve().parents[1]


class Monitor(Node):
    def __init__(self):
        super().__init__('robocup_perception_smoke')
        self.counts = Counter({key: 0 for key in ('left', 'right', 'rgb', 'depth', 'imu', 'vio', 'status', 'map')})
        self.first = {}
        self.last = {}
        self.gaps = Counter()
        self.nonmonotonic = Counter()
        self.vo_states = Counter()
        self.map_cells = 0
        self.frames = {}
        self.position0 = None
        self.last_position = None
        self.max_displacement = 0.
        self.initial_quaternion = None
        self.max_orientation_change_deg = 0.
        for name, topic, kind in [
            ('left', '/camera/camera/infra1/image_rect_raw', Image),
            ('right', '/camera/camera/infra2/image_rect_raw', Image),
            ('rgb', '/camera/camera/color/image_raw', Image),
            ('depth', '/camera/camera/aligned_depth_to_color/image_raw', Image),
            ('imu', '/camera/camera/imu', Imu),
            ('vio', '/visual_slam/tracking/odometry', Odometry),
            ('status', '/visual_slam/status', VisualSlamStatus),
        ]:
            self.create_subscription(kind, topic, lambda msg, key=name: self.record(key, msg), qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def record(self, name, msg):
        t = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if name in self.last:
            delta = t-self.last[name]
            self.gaps[name] = max(self.gaps[name], delta)
            self.nonmonotonic[name] += int(delta <= 0)
        self.first.setdefault(name, t)
        self.last[name] = t
        self.counts[name] += 1
        self.frames[name] = msg.header.frame_id
        if name == 'status':
            self.vo_states[int(msg.vo_state)] += 1
        if name == 'vio':
            q = msg.pose.pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            norm = math.sqrt(sum(v*v for v in quat))
            if norm > 0 and all(math.isfinite(v) for v in quat):
                quat = [v/norm for v in quat]
                if self.initial_quaternion is None:
                    self.initial_quaternion = quat
                dot = abs(sum(a*b for a, b in zip(quat, self.initial_quaternion)))
                self.max_orientation_change_deg = max(self.max_orientation_change_deg,
                    math.degrees(2*math.acos(min(1., dot))))
            p = msg.pose.pose.position
            self.last_position = [p.x, p.y, p.z]
            if self.position0 is None:
                self.position0 = self.last_position
            self.max_displacement = max(self.max_displacement,
                sum((a-b)**2 for a, b in zip(self.position0, self.last_position))**0.5)

    def on_map(self, msg):
        self.counts['map'] += 1
        self.map_cells = len(msg.data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stereo-only', action='store_true')
    parser.add_argument('--duration', type=float, default=45., help='Total collection including startup, 10..180 seconds')
    parser.add_argument('--record-state', action='store_true', help='Record raw VIO, status, IMU and TF, no images')
    parser.add_argument('--rgbd-no-map', action='store_true',
                        help='Enable RGB-D with stereo+IMU+VIO; do not start RTAB-Map')
    args = parser.parse_args()
    if not 10 <= args.duration <= 180:
        parser.error('duration must be 10..180 seconds')
    if args.stereo_only and args.rgbd_no_map:
        parser.error('use only one of --stereo-only or --rgbd-no-map')
    if args.stereo_only:
        mode = 'stereo_imu'
        rgbd = False
        start_mapping = False
        required = ('left', 'right', 'imu', 'vio', 'status')
    elif args.rgbd_no_map:
        mode = 'rgbd_stereo_imu'
        rgbd = True
        start_mapping = False
        required = ('left', 'right', 'rgb', 'depth', 'imu', 'vio', 'status')
    else:
        mode = 'rgbd_stereo_imu_mapping'
        rgbd = True
        start_mapping = True
        required = ('left', 'right', 'rgb', 'depth', 'imu', 'vio', 'status', 'map')
    out = ROOT/'evidence'/time.strftime('perception_%Y%m%d_%H%M%S')
    out.mkdir(parents=True, exist_ok=False)
    processes, logs = [], []
    report = {'flight_validation': False, 'eeprom_write_performed': False,
              'mode': mode}
    rclpy.init()
    monitor = Monitor()
    try:
        commands = [('perception', ['ros2', 'launch', str(ROOT/'launch/perception.launch.py'),
                     'rgbd:=true' if rgbd else 'rgbd:=false', 'imu_fusion:=false'])]
        if args.record_state:
            commands.insert(0, ('record', ['ros2', 'bag', 'record', '-o', str(out/'state_bag'),
                '/visual_slam/tracking/odometry', '/visual_slam/status', '/camera/camera/imu', '/tf', '/tf_static']))
        if start_mapping:
            commands.append(('mapping', ['ros2', 'launch', str(ROOT/'launch/mapping.launch.py'), f'database:={out}/map.db']))
        for name, command in commands:
            log = open(out/f'{name}.log', 'w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic()+args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(monitor, timeout_sec=0.01)
        report['streams'] = {}
        for name, count in monitor.counts.items():
            span = (monitor.last.get(name, 0)-monitor.first.get(name, 0))/1e9
            report['streams'][name] = {'count': count, 'hz': (count-1)/span if span > 0 else None,
                'span_s': span,
                'max_gap_ms': monitor.gaps[name]/1e6, 'nonmonotonic': monitor.nonmonotonic[name]}
        report['vo_states'] = dict(monitor.vo_states)
        report['frame_ids'] = monitor.frames
        report['map_cells'] = monitor.map_cells
        report['initial_position'] = monitor.position0
        report['final_position'] = monitor.last_position
        report['max_observed_displacement_m'] = monitor.max_displacement
        report['max_orientation_change_deg'] = monitor.max_orientation_change_deg
        report['collection_duration_s'] = args.duration
        report['all_streams_received'] = all(monitor.counts[k] > 1 for k in required)
        report['flight_input_topics'] = [name for name, _ in monitor.get_topic_names_and_types() if name.startswith('/fmu/in/')]
    finally:
        for p in processes:
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in processes:
            try:
                p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGTERM)
                p.wait(timeout=5)
        for log in logs:
            log.close()
        monitor.destroy_node()
        rclpy.try_shutdown()
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2))
        print('Evidence:', out)
    return 0 if report.get('all_streams_received') and not report.get('flight_input_topics') else 1


if __name__ == '__main__':
    raise SystemExit(main())
