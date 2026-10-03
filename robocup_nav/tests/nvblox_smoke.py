#!/usr/bin/env python3
"""Real nvblox GPU execution with synthetic depth or explicitly selected camera.

Synthetic: tests native depth+TF -> nonempty ESDF. Live: adds real cuVSLAM.
Bounded collection, isolated ROS domain, no navigation goals or PX4 input.
"""
import argparse
import os
os.environ['ROS_DOMAIN_ID'] = '175'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import json
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from lifecycle_msgs.srv import GetState
from tf2_ros import StaticTransformBroadcaster
from nvblox_msgs.msg import DistanceMapSlice
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus

ROOT = Path(__file__).resolve().parents[1]


class Probe(Node):
    def __init__(self, synthetic):
        super().__init__('robocup_nvblox_probe')
        self.counts = {k: 0 for k in ('depth', 'imu', 'vio', 'slice')}
        self.first, self.last, self.gaps, self.bad_stamps = {}, {}, {}, {}
        self.valid_stamp_counts, self.zero_stamps = {}, {}
        self.vo_states = {}
        self.observed = self.obstacles = 0
        self.max_cells = 0
        self.last_slice = None
        self.costmaps = {'global': 0, 'local': 0}
        self.readiness = {}
        for key in self.costmaps:
            self.create_subscription(OccupancyGrid, f'/{key}_costmap/costmap',
                                     lambda msg, k=key: self.costmap(k, msg), 10)
        self.create_subscription(String, '/robocup/nvblox/readiness', self.guard_status, 10)
        for key, topic, kind in [
            ('depth', '/camera/camera/depth/image_rect_raw', Image),
            ('imu', '/camera/camera/imu', Imu),
            ('vio', '/visual_slam/tracking/odometry', Odometry),
            ('slice', '/nvblox_node/static_map_slice', DistanceMapSlice),
        ]:
            self.create_subscription(kind, topic, lambda msg, k=key: self.receive(k, msg),
                10 if key == 'slice' else qos_profile_sensor_data)
        self.create_subscription(VisualSlamStatus, '/visual_slam/status', self.status, qos_profile_sensor_data)
        if synthetic:
            self.depth_pub = self.create_publisher(Image, '/camera/camera/depth/image_rect_raw', 10)
            self.info_pub = self.create_publisher(CameraInfo, '/camera/camera/depth/camera_info', 10)
            self.tf = StaticTransformBroadcaster(self)
            transforms = []
            for parent, child, quat in [('odom', 'base_link', (0., 0., 0., 1.)),
                                         ('base_link', 'camera_depth_optical_frame', (-.5, .5, -.5, .5))]:
                tf = TransformStamped()
                tf.header.stamp = self.get_clock().now().to_msg()
                tf.header.frame_id, tf.child_frame_id = parent, child
                tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = quat
                transforms.append(tf)
            self.tf.sendTransform(transforms)
            self.create_timer(.1, self.publish)

    def status(self, msg):
        self.vo_states[int(msg.vo_state)] = self.vo_states.get(int(msg.vo_state), 0)+1

    def costmap(self, key, msg):
        if len(msg.data) == msg.info.width*msg.info.height and any(v >= 0 for v in msg.data):
            self.costmaps[key] += 1

    def guard_status(self, msg):
        self.readiness[msg.data] = self.readiness.get(msg.data, 0)+1

    def receive(self, key, msg):
        t = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if t > 0 and key in self.last:
            dt = t-self.last[key]
            self.gaps[key] = max(self.gaps.get(key, 0), dt)
            self.bad_stamps[key] = self.bad_stamps.get(key, 0)+int(dt <= 0)
        if t > 0:
            self.first.setdefault(key, t)
            self.last[key] = t
            self.valid_stamp_counts[key] = self.valid_stamp_counts.get(key, 0)+1
        else:
            self.zero_stamps[key] = self.zero_stamps.get(key, 0)+1
        self.counts[key] += 1
        if key == 'slice':
            a = np.asarray(msg.data)
            known = np.isfinite(a) & (a != msg.unknown_value)
            self.observed = max(self.observed, int(known.sum()))
            self.obstacles = max(self.obstacles, int((known & (a <= 0)).sum()))
            self.max_cells = max(self.max_cells, len(a))
            self.last_slice = msg

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        image = Image()
        image.header.stamp, image.header.frame_id = stamp, 'camera_depth_optical_frame'
        image.width, image.height, image.encoding, image.step = 160, 120, '16UC1', 320
        image.data = np.full((120, 160), 2000, dtype='<u2').tobytes()
        info = CameraInfo()
        info.header = image.header
        info.width, info.height = image.width, image.height
        info.k = [90., 0., 80., 0., 90., 60., 0., 0., 1.]
        info.p = [90., 0., 80., 0., 0., 90., 60., 0., 0., 0., 1., 0.]
        self.info_pub.publish(info)
        self.depth_pub.publish(image)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--live', action='store_true')
    parser.add_argument('--navigation', action='store_true', help='Also run shadow Nav2, without sending goals')
    parser.add_argument('--ground-start', action='store_true', help='Test cruise-height shadow map from ground')
    parser.add_argument('--duration', type=float, default=30.)
    args = parser.parse_args()
    if args.navigation and not args.live:
        parser.error('--navigation requires --live')
    if args.ground_start and not (args.navigation and args.live):
        parser.error('--ground-start requires --live --navigation')
    if not 10 <= args.duration <= 90:
        parser.error('duration must be 10..90 seconds')
    out = ROOT/'evidence'/time.strftime(('nvblox_live_' if args.live else 'nvblox_gpu_')+'%Y%m%d_%H%M%S')
    out.mkdir(parents=True, exist_ok=False)
    report = {'flight_validation': False, 'mode': 'live' if args.live else 'synthetic', 'passed': False}
    processes, logs = [], []
    rclpy.init()
    probe = Probe(not args.live)
    try:
        commands = [('nvblox', ['ros2', 'launch', str(ROOT/'launch/nvblox.launch.py')])]
        if args.live:
            commands.insert(0, ('perception', ['ros2', 'launch', str(ROOT/'launch/perception.launch.py'),
                                              'nvblox_depth:=true', 'publish_map_tf:=true',
                                              'imu_fusion:=false']))
        if args.navigation:
            commands = [('stack', ['ros2', 'launch', str(ROOT/'launch/nvblox_stack.launch.py')])]
        if args.ground_start:
            commands = [('stack', ['ros2', 'launch', str(ROOT/'launch/ground_navigation_shadow.launch.py')])]
            report['ground_start'] = True
            report['cruise_rise_m'] = .6
        for name, command in commands:
            log = open(out/f'{name}.log', 'w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic()+args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=.02)
        report['streams'] = {}
        for key, count in probe.counts.items():
            span = (probe.last.get(key, 0)-probe.first.get(key, 0))/1e9
            report['streams'][key] = {'count': count, 'hz': (probe.valid_stamp_counts.get(key, 0)-1)/span if span > 0 else 0,
                                     'max_gap_ms': probe.gaps.get(key, 0)/1e6,
                                     'nonmonotonic': probe.bad_stamps.get(key, 0),
                                     'zero_stamp_messages': probe.zero_stamps.get(key, 0)}
        report.update({'known_esdf_cells': probe.observed, 'obstacle_cells': probe.obstacles,
                       'max_slice_cells': probe.max_cells, 'vo_states': probe.vo_states})
        forbidden = [name for name, _ in probe.get_topic_names_and_types() if name.startswith('/fmu/in/')]
        report['flight_input_topics'] = forbidden
        required = ('depth', 'imu', 'vio', 'slice') if args.live else ('depth', 'slice')
        report['passed'] = not forbidden and probe.observed > 20 and all(probe.counts[k] > 5 for k in required)
        if args.live:
            report['passed'] &= bool(probe.vo_states) and set(probe.vo_states) == {1}
        if args.navigation:
            states = {}
            for name in ('planner_server', 'controller_server', 'bt_navigator'):
                client = probe.create_client(GetState, f'/{name}/get_state')
                if client.wait_for_service(timeout_sec=2.):
                    future = client.call_async(GetState.Request())
                    rclpy.spin_until_future_complete(probe, future, timeout_sec=2.)
                    if future.done() and future.result() is not None:
                        states[name] = future.result().current_state.id
            report['lifecycle_states'] = states
            report['costmap_messages'] = probe.costmaps
            report['guard_readiness'] = probe.readiness
            report['passed'] &= len(states) == 3 and all(v == 3 for v in states.values())
            report['passed'] &= all(v > 5 for v in probe.costmaps.values())
        if probe.last_slice is not None:
            msg = probe.last_slice
            np.savez_compressed(out/'esdf_slice.npz', data=np.asarray(msg.data), width=msg.width,
                                height=msg.height, resolution=msg.resolution, unknown=msg.unknown_value,
                                origin=[msg.origin.x, msg.origin.y, msg.origin.z])
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
        probe.destroy_node()
        rclpy.try_shutdown()
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2))
        print('Evidence:', out)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
