#!/usr/bin/env python3
"""Actual Nav2 nvblox plugin + A*/DWB + stale-source guards, synthetic scene.

Uses generated ESDF fixture, not the GPU mapper (which has a separate test).
"""
import os
import argparse
from integration import Fixture
os.environ['ROS_DOMAIN_ID'] = '176'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import json
from pathlib import Path
import signal
import subprocess
import time
import sys
import math
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from nvblox_msgs.msg import DistanceMapSlice
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from nav2_msgs.action import FollowPath
from lifecycle_msgs.srv import GetState

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from cruise_band import make_band
from navigation_landing_shadow import Scene,MissionShadow
from navigation_frame_contract import SessionAlignment
from navigation_shadow_pipeline import ShadowPipeline
from stopping_space import stopping_space_clear


class NvbloxFixture(Fixture):
    def __init__(self,trial=False):
        super().__init__(enable_px4_preview=False)
        self.slice_pub = self.create_publisher(DistanceMapSlice, '/nvblox_node/static_map_slice', 10)
        self.raw_odom_pub = self.create_publisher(Odometry, '/visual_slam/tracking/odometry', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/camera/depth/image_rect_raw', 10)
        self.vo_pub = self.create_publisher(VisualSlamStatus, '/visual_slam/status', 10)
        self.depth_enabled = self.slice_enabled = True
        self.map_msg = None
        self.checked_nonzero = 0
        self.last_checked = None
        self.offset = 0.
        self.pipeline_messages=0
        self.pipeline_active_outputs=0
        self.pipeline_fault=None
        self.pipeline=None
        if trial:
            def clear(p,v,c):
                if self.map_msg is None:return False
                free=np.asarray(self.map_msg.data).reshape(120,120)>0
                return stopping_space_clear(free,.05,(0,0),p,v,c,body_radius=.43,
                    uncertainty=.05,latency=.3,braking=.2)
            # Synthetic airborne origin/frame pairing, not a hardware alignment.
            mission=MissionShadow(make_band(initial_z=-.6),(.5,5.5,.5,5.5),clear)
            alignment=SessionAlignment([1,2.5,0],0,[4,5,6],0,'synthetic',(0,)*5,verified=True)
            self.pipeline=ShadowPipeline(mission,alignment,0.)
        self.create_subscription(Twist, '/robocup/nav/cmd_vel_raw', self.checked, 10)

    def checked(self, msg):
        self.last_checked = msg
        self.checked_nonzero += int(abs(msg.linear.x)+abs(msg.angular.z) > .001)
        if self.pipeline is not None:
            now=time.monotonic()
            s=Scene(at=now if self.odom_enabled else now-1,position=(1.,2.5,0.),
                tilt_rad=0.,localization_ok=self.odom_enabled,airborne=True,
                map_at=now if self.slice_enabled and self.depth_enabled else now-1,
                footprint_known_free=True,navigation_at=now,
                navigation_velocity_odom=(msg.linear.x,msg.linear.y))
            r=self.pipeline.step(now,s,timestamp_us=self.get_clock().now().nanoseconds//1000,
                vio_session='synthetic',reset_counters=(0,)*5,exclusive_writer_verified=True,
                yaw_rate_odom=msg.angular.z)
            if r.get('fault'):self.pipeline_fault=r['fault']
            if r['messages']:
                sp=r['messages']['trajectory_setpoint']
                assert math.isnan(sp.position[0]) and math.isnan(sp.position[1])
                assert abs(sp.position[2]-6.)<1e-5
                assert math.hypot(*sp.velocity[:2])<=.15001
                assert 'vehicle_command' not in r['messages']
                self.pipeline_messages+=1
                if not r['intention']['reason'] and abs(msg.linear.x)+abs(msg.angular.z)>.001:
                    assert abs(sp.velocity[0]-msg.linear.x)<1e-6
                    assert abs(sp.velocity[1]+msg.linear.y)<1e-6
                    self.pipeline_active_outputs+=1

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        if self.odom_enabled:
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = 1.+self.offset, 2.5
            odom.pose.pose.orientation.w = 1.
            self.raw_odom_pub.publish(odom)
        status = VisualSlamStatus()
        status.header.stamp, status.header.frame_id = stamp, 'map'
        status.vo_state = 1
        self.vo_pub.publish(status)
        if self.depth_enabled:
            depth = Image()
            depth.header.stamp, depth.header.frame_id = stamp, 'camera_depth_optical_frame'
            depth.width, depth.height, depth.step, depth.encoding = 1, 1, 2, '16UC1'
            depth.data = b'\xd0\x07'
            self.depth_pub.publish(depth)
        if self.slice_enabled and self.map_msg is not None:
            self.map_msg.header.stamp = stamp
            self.slice_pub.publish(self.map_msg)

    def map(self, blocked=False):
        msg = DistanceMapSlice()
        msg.header.frame_id = 'odom'
        msg.resolution, msg.width, msg.height, msg.unknown_value = .05, 120, 120, -1000.
        cells = [2.]*14400
        for y in range(120):
            for x in range(120):
                if x in (0, 119) or y in (0, 119) or (57 <= x <= 62 and (blocked or 28 <= y <= 72)):
                    cells[y*120+x] = -.01
        msg.data = cells
        self.map_msg = msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trial-profile', action='store_true')
    args = parser.parse_args()
    out = ROOT/'evidence'/time.strftime('nvblox_navigation_%Y%m%d_%H%M%S')
    out.mkdir(parents=True, exist_ok=False)
    report = {'passed': False, 'flight_validation': False}
    proc = None
    rclpy.init()
    fixture = NvbloxFixture(trial=args.trial_profile)
    log = open(out/'navigation.log', 'w')
    try:
        proc = subprocess.Popen(['ros2', 'launch', str(ROOT/'launch/nvblox_navigation.launch.py'),
                                 'trial_profile:=' + str(args.trial_profile).lower()],
                                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        fixture.map()
        for name in ('planner_server', 'controller_server', 'bt_navigator'):
            client = fixture.create_client(GetState, f'/{name}/get_state')
            fixture.wait(client.service_is_ready, 35)
            deadline = time.monotonic()+35
            while fixture.result(client.call_async(GetState.Request())).current_state.id != 3:
                if time.monotonic() > deadline:
                    raise RuntimeError(name+' not active')
                fixture.spin_for(.5)
        report['lifecycle_active'] = True
        fixture.wait(fixture.plan.server_is_ready)
        result = fixture.planned_path()
        assert result.status == 4 and len(result.result.path.poses) > 2
        assert any(abs(p.pose.position.y-2.5) > 1. for p in result.result.path.poses)
        report['astar_nvblox_detour'] = True
        goal = FollowPath.Goal()
        goal.path, goal.controller_id, goal.goal_checker_id = result.result.path, 'FollowPath', 'goal_checker'
        fixture.wait(fixture.follow.server_is_ready)
        handle = fixture.result(fixture.follow.send_goal_async(goal))
        assert handle.accepted
        fixture.wait(lambda: fixture.checked_nonzero > 3, 12)
        report['dwb_guarded_output'] = True
        if args.trial_profile:
            assert fixture.pipeline_fault is None,fixture.pipeline_fault
            assert fixture.pipeline_messages>=4
            assert fixture.pipeline_active_outputs>=4,'No active DWB commands crossed the candidate pipeline'
            report['actual_dwb_to_shadow_px4_message_objects']=True
            report['shadow_message_count_before_fault_tests']=fixture.pipeline_messages
            report['active_dwb_commands_to_message_objects']=fixture.pipeline_active_outputs
        for field in ('depth_enabled', 'slice_enabled', 'odom_enabled'):
            setattr(fixture, field, False)
            fixture.spin_for(.9)
            count = fixture.checked_nonzero
            fixture.spin_for(.4)
            assert fixture.checked_nonzero == count, field+' did not inhibit velocity'
            assert fixture.last_checked is not None and fixture.last_checked.linear.x == 0.
            report[field+'_stale_inhibits'] = True
            setattr(fixture, field, True)
            fixture.wait(lambda: fixture.checked_nonzero > count, 3)
        fixture.result(handle.cancel_goal_async())
        fixture.map(blocked=True)
        fixture.spin_for(2.)
        blocked = fixture.planned_path()
        assert blocked.status != 4 or not blocked.result.path.poses
        report['blocked_path_rejected'] = True
        forbidden = [name for name, _ in fixture.get_topic_names_and_types() if name.startswith('/fmu/in/')]
        assert not forbidden
        report['no_flight_inputs'] = True
        report['passed'] = True
    finally:
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=5)
        log.close()
        fixture.destroy_node()
        rclpy.try_shutdown()
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2))
        print('Evidence:', out)


if __name__ == '__main__':
    main()
