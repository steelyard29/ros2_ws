#!/usr/bin/env python3
"""Isolated real Nav2 plugins on a synthetic map; no camera or flight inputs.

Checks lifecycle, obstacle detour, DWB command production, stale preview stop,
blocked global path, and absence of PX4 command publishers. Not flight dynamics.
"""
import os
os.environ['ROS_DOMAIN_ID'] = '173'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import json
import math
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from nav2_msgs.action import ComputePathToPose, FollowPath
from lifecycle_msgs.srv import GetState
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

ROOT = Path(__file__).resolve().parents[1]


class Fixture(Node):
    def __init__(self, enable_px4_preview=True):
        super().__init__('robocup_test_fixture')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.odom_pub = self.create_publisher(Odometry, '/robocup/odom', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/robocup/obstacles', 10)
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = 'map', 'odom'
        tf.transform.rotation.w = 1.
        self.static.sendTransform(tf)
        self.nonzero = 0
        self.previews = 0
        self.odom_enabled = True
        self.create_subscription(Twist, '/robocup/nav/cmd_vel_raw', self.command, 10)
        if enable_px4_preview:
            from px4_msgs.msg import TrajectorySetpoint
            self.create_subscription(TrajectorySetpoint, '/robocup/preview/trajectory_setpoint', self.preview, 10)
        self.create_timer(0.05, self.tick)
        self.plan = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.follow = ActionClient(self, FollowPath, '/follow_path')

    def command(self, msg):
        if abs(msg.linear.x)+abs(msg.angular.z) > 0.001:
            self.nonzero += 1

    def preview(self, msg):
        assert all(math.isfinite(v) for v in msg.velocity)
        assert math.isnan(msg.position[0]) and math.isnan(msg.position[1])
        self.previews += 1

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'odom', 'base_footprint'
        tf.transform.translation.x = 1.
        tf.transform.translation.y = 2.5
        tf.transform.rotation.w = 1.
        self.tf.sendTransform(tf)
        if self.odom_enabled:
            odom = Odometry()
            odom.header = tf.header
            odom.child_frame_id = 'base_footprint'
            odom.pose.pose.position.x, odom.pose.pose.position.y = 1., 2.5
            odom.pose.pose.orientation.w = 1.
            self.odom_pub.publish(odom)
        # Obstacle observation far off the route; static map contains the wall.
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(
            Header(stamp=stamp, frame_id='base_footprint'), [[0., -2., 0.5]]))

    def map(self, blocked=False):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution, grid.info.width, grid.info.height = 0.05, 120, 120
        grid.info.origin.orientation.w = 1.
        cells = [0]*14400
        for y in range(120):
            for x in range(120):
                if x in (0, 119) or y in (0, 119) or (57 <= x <= 62 and (blocked or 28 <= y <= 72)):
                    cells[y*120+x] = 100
        grid.data = cells
        self.map_pub.publish(grid)

    def wait(self, predicate, timeout=20):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise TimeoutError('test predicate timed out')

    def spin_for(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def result(self, future, timeout=20):
        self.wait(future.done, timeout)
        return future.result()

    def planned_path(self):
        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x, goal.goal.pose.position.y = 5., 2.5
        goal.goal.pose.orientation.w = 1.
        goal.planner_id = 'GridBased'
        handle = self.result(self.plan.send_goal_async(goal))
        assert handle.accepted
        return self.result(handle.get_result_async())


def main():
    evidence = ROOT/'evidence'
    evidence.mkdir(exist_ok=True)
    report = {'test': 'real_nav2_synthetic_scene', 'flight_validation': False}
    processes = []
    logs = []
    rclpy.init()
    fixture = Fixture()
    try:
        for name, args in [
            ('navigation', ['ros2', 'launch', str(ROOT/'launch/navigation.launch.py'), 'demo_geometry:=true']),
            ('preview', ['python3', str(ROOT/'scripts/shadow_bridge.py')]),
        ]:
            log = open(evidence/f'{name}_test.log', 'w')
            logs.append(log)
            processes.append(subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        fixture.map()
        for name in ('planner_server', 'controller_server', 'bt_navigator'):
            client = fixture.create_client(GetState, f'/{name}/get_state')
            fixture.wait(lambda: client.service_is_ready(), 35)
            deadline = time.monotonic()+35
            while True:
                state = fixture.result(client.call_async(GetState.Request())).current_state
                if state.id == 3:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError(f'{name} lifecycle not active: {state.label}')
                fixture.spin_for(0.5)
        report['lifecycle_active'] = True
        fixture.wait(lambda: fixture.plan.server_is_ready())
        result = fixture.planned_path()
        assert result.status == 4 and len(result.result.path.poses) > 2, result
        path = result.result.path
        assert any(abs(p.pose.position.y-2.5) > 1.0 for p in path.poses), 'Path did not detour around wall'
        report['astar_obstacle_detour'] = True
        fixture.wait(lambda: fixture.follow.server_is_ready())
        goal = FollowPath.Goal()
        goal.path, goal.controller_id, goal.goal_checker_id = path, 'FollowPath', 'goal_checker'
        handle = fixture.result(fixture.follow.send_goal_async(goal))
        assert handle.accepted
        fixture.wait(lambda: fixture.nonzero >= 3 and fixture.previews >= 3, 12)
        report['dwb_nonzero_commands'] = fixture.nonzero
        report['preview_conversion'] = True
        fixture.odom_enabled = False
        fixture.spin_for(0.7)
        count = fixture.previews
        fixture.spin_for(0.5)
        assert fixture.previews == count, 'Preview continued on stale odometry'
        report['stale_odometry_inhibits_preview'] = True
        fixture.result(handle.cancel_goal_async())
        fixture.map(blocked=True)
        fixture.spin_for(2.)
        blocked = fixture.planned_path()
        assert blocked.status != 4 or not blocked.result.path.poses, 'Planner crossed blocked wall'
        report['blocked_path_rejected'] = True
        forbidden = [t for t, _ in fixture.get_topic_names_and_types() if t.startswith('/fmu/in/')]
        assert not forbidden, forbidden
        report['no_flight_input_topics'] = True
        report['passed'] = True
    except Exception as exc:
        report['passed'] = False
        report['error'] = repr(exc)
        raise
    finally:
        for proc in processes:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in processes:
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=5)
        for log in logs:
            log.close()
        fixture.destroy_node()
        rclpy.try_shutdown()
        (evidence/'integration.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
