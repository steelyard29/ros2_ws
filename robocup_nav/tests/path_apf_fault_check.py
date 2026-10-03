"""Domain-176 DDS tests of the actual C++ adapter, no sensors or PX4 routes.

Positive controls surround every rejection. Zero candidate output proves only
adapter inhibition, NOT actuator stopping. Run without other domain-176 tests.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '176'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan

ROOT = Path(__file__).resolve().parents[1]


class Fixture(Node):
    def __init__(self):
        super().__init__('path_apf_fault_fixture')
        self.case = 'missing_scan'
        self.commands = []
        self.scan = self.create_publisher(LaserScan, '/robocup/legacy_apf/scan', 10)
        self.odom = self.create_publisher(Odometry, '/visual_slam/tracking/odometry', 10)
        self.path = self.create_publisher(NavPath, '/robocup/legacy_apf/path', 10)
        self.create_subscription(Twist, '/robocup/legacy_apf/cmd_vel_odom', self.received, 10)
        self.create_timer(.05, self.publish_inputs)

    def received(self, msg):
        self.commands.append((time.monotonic(), [msg.linear.x, msg.linear.y, msg.linear.z,
                                               msg.angular.x, msg.angular.y, msg.angular.z]))

    def stamp(self, delta=0.):
        ns = self.get_clock().now().nanoseconds + int(delta * 1e9)
        msg = self.get_clock().now().to_msg()
        msg.sec, msg.nanosec = divmod(ns, 10**9)
        return msg

    def publish_inputs(self):
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.header.stamp = self.stamp(-1. if self.case == 'stale_odom' else 0.)
        odom.pose.pose.orientation.w = 0. if self.case == 'invalid_orientation' else 1.
        self.odom.publish(odom)
        path = NavPath()
        path.header.frame_id = 'odom'
        path.header.stamp = self.stamp()
        if self.case != 'empty_path':
            for x in (0., .5, 1., 2.):
                p = PoseStamped()
                p.header = path.header
                p.pose.position.x = x
                p.pose.orientation.w = 1.
                path.poses.append(p)
        self.path.publish(path)
        if self.case in ('missing_scan', 'scan_silence'):
            return
        scan = LaserScan()
        scan.header.frame_id = 'laser' if self.case == 'wrong_scan_frame' else 'base_link'
        scan.header.stamp = self.stamp({'stale_scan': -1., 'future_scan': 1.}.get(self.case, 0.))
        scan.angle_min = -math.pi
        scan.angle_increment = 2 * math.pi / 720
        scan.angle_max = scan.angle_min + 719 * scan.angle_increment
        scan.range_min, scan.range_max = .1, 12.
        scan.ranges = [5.] * 720
        if self.case == 'nan_scan':
            scan.ranges = [float('nan')] * 720
        elif self.case == 'inf_scan':
            scan.ranges = [float('inf')] * 720
        elif self.case == 'low_valid_ratio':
            scan.ranges = [5.] * 424 + [float('nan')] * 296
        elif self.case == 'partial_scan':
            scan.ranges = [5.] * 360
            scan.angle_max = scan.angle_min + 359 * scan.angle_increment
        elif self.case == 'inconsistent_angles':
            scan.angle_increment *= 2
        elif self.case == 'invalid_range_limits':
            scan.range_max = scan.range_min
        self.scan.publish(scan)

    def check(self, case, expect_motion):
        self.case = case
        start = time.monotonic()
        while time.monotonic() - start < 1.2:
            rclpy.spin_once(self, timeout_sec=.03)
        # Ignore the transition/transport interval; freshness is 250ms.
        rows = [(at, v) for at, v in self.commands if at >= start + .7]
        finite = all(all(math.isfinite(x) for x in v) for _, v in rows)
        motion = [math.hypot(v[0], v[1]) for _, v in rows]
        passed = len(rows) >= 5 and finite and all(
            (.1 < s <= .150000000001) if expect_motion else s == 0. for s in motion)
        passed &= all(all(x == 0. for x in v[2:]) for _, v in rows)
        return dict(case=case, passed=bool(passed), expected_motion=expect_motion,
                    sample_count=len(rows), max_speed=max(motion, default=None))


def main():
    out = ROOT / 'evidence' / time.strftime('path_apf_fault_%Y%m%d_%H%M%S')
    out.mkdir(parents=True)
    report = dict(passed=False, physical_stop_verified=False, px4_sitl=False, cases=[])
    rclpy.init()
    fixture = Fixture()
    proc = None
    try:
        # Do not contaminate an existing offline navigation run.
        for _ in range(20):
            rclpy.spin_once(fixture, timeout_sec=.05)
        others = [n for n in fixture.get_node_names() if n != fixture.get_name()]
        if others:
            raise RuntimeError('domain 176 already occupied: ' + ', '.join(others))
        env = dict(os.environ)
        env['AMENT_PREFIX_PATH'] = str(ROOT.parent / 'build/uav_task/ament_cmake_index') + ':' + env.get('AMENT_PREFIX_PATH', '')
        with (out / 'adapter.log').open('w') as log:
            proc = subprocess.Popen([str(ROOT / 'build/legacy_apf_shadow/legacy_apf_shadow'),
                '--ros-args', '-p', 'controller_config_file:=' + str(ROOT / 'config/legacy_apf_shadow.yaml'),
                '-p', 'path_guided:=true', '--log-level', 'warn'],
                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 10
        while fixture.scan.get_subscription_count() != 1:
            if time.monotonic() > deadline or proc.poll() is not None:
                raise RuntimeError('isolated adapter did not start')
            rclpy.spin_once(fixture, timeout_sec=.05)
        report['cases'].append(fixture.check('missing_scan', False))
        for case in ('nan_scan', 'inf_scan', 'low_valid_ratio', 'partial_scan',
                     'inconsistent_angles', 'invalid_range_limits', 'wrong_scan_frame',
                     'stale_scan', 'future_scan', 'scan_silence', 'stale_odom',
                     'invalid_orientation', 'empty_path'):
            report['cases'].append(fixture.check('normal_before_' + case, True))
            report['cases'].append(fixture.check(case, False))
        report['cases'].append(fixture.check('final_normal', True))
        report['no_flight_inputs'] = not any(n.startswith('/fmu/in/') for n, _ in fixture.get_topic_names_and_types())
        report['passed'] = report['no_flight_inputs'] and all(c['passed'] for c in report['cases'])
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
        (out / 'report.json').write_text(json.dumps(report, indent=2))
        (out / 'commands.json').write_text(json.dumps(fixture.commands))
        fixture.destroy_node()
        rclpy.try_shutdown()
    print(out)
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
