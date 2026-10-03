"""Actual generated ROS message callbacks. No nodes/drivers/flight publishers."""
from collections import deque
from dataclasses import replace
from types import SimpleNamespace as NS
from pathlib import Path
import json
import math
import sys
import unittest
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from task_scene_input import TaskSceneInput
from task_map_profile import MapBandEvidence
from task_scan_bridge import transform_scan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Transform
from nvblox_msgs.msg import DistanceMapSlice
from std_msgs.msg import String
from task_flight_controller import TaskFlightSupervisor
from test_flight_supervisor import healthy


class SceneTests(unittest.TestCase):
    def prepare_ground(self,reviewed=True,vio=True):
        a=self.a
        a.core=NS(controller=TaskFlightSupervisor(),sample=healthy(10.),
                  source_stamps={'vehicle_local_position':10_000_000},ev=NS(inhibit=False))
        a.core.local_history=[(10_000_000_000,a.core.sample)]
        a.mission=a.alignment=None;a.session='ground-test';a.last_status=10.
        a.cfg.update(alignment_reviewed=True,bounds_odom=[-1,4,-2,2],
                     takeoff_column_reviewed=reviewed)
        a.owns=lambda:True
        a.topics={'map':'map','bounds':'bounds'}
        a.node=NS(get_publishers_info_by_topic=lambda _:[])
        if vio:a.pose(self.odom())
        a.prepare(10.)
        return a

    def test_no_map_or_h_does_not_block_reviewed_vertical_takeoff(self):
        a=self.prepare_ground()
        self.assertIsNotNone(a.mission)
        self.assertEqual(a.blockers,[])
        self.assertIn('stale/missing map',a.navigation_blockers)
        self.assertFalse(a.core.controller.scene.footprint_known_free)
        self.assertFalse(a.core.controller.scene.h_metric_verified)
        self.assertTrue(a.core.controller.ready(a.core.sample,10.))

    def test_missing_column_or_vio_still_blocks_takeoff(self):
        a=self.prepare_ground(reviewed=False)
        self.assertFalse(a.core.controller.ready(a.core.sample,10.))
        self.setUp();a=self.prepare_ground(vio=False)
        self.assertIsNone(a.mission)
        self.assertFalse(a.core.controller.ready(a.core.sample,10.))

    def test_initial_alignment_uses_timestamp_matched_position_not_latest(self):
        a=self.prepare_ground(vio=False)
        old=replace(a.core.sample,local_at=9.9,x=1.)
        a.core.sample=replace(a.core.sample,x=2.)
        a.core.local_history=[(9_900_000_000,old),(10_000_000_000,a.core.sample)]
        msg=self.odom();msg.header.stamp.sec=9;msg.header.stamp.nanosec=900_000_000
        a.pose(msg);a.prepare(10.)
        self.assertIsNotNone(a.alignment)
        self.assertEqual(a.alignment.local_origin[0],1.)
        self.assertEqual(a.pairing_evidence['difference_ms'],0.)

    def setUp(self):
        self.a=TaskSceneInput.__new__(TaskSceneInput)
        a=self.a;a.seen={};a.stamps={};a.poses=deque(maxlen=40)
        a.clock=lambda:10.;a.now_ns=lambda:10_000_000_000
        a.pose_data=a.grid=a.h_target=a.command=None;a.fault=None
        a.band_evidence=MapBandEvidence()
        a.mapper_identity=None
        a.mission=NS(goal_id='s:OUTBOUND',band=NS(floor=-.14))
        a.core=NS(controller=NS(context_fault=None))
        a.cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/roundtrip_task.yaml').read_text())

    def odom(self):
        m=Odometry();m.header.stamp.sec=10;m.header.frame_id='odom';m.child_frame_id='base_link'
        m.pose.pose.orientation.w=1.;return m

    def test_pose_twist_rotation(self):
        m=self.odom();m.pose.pose.orientation.z=math.sin(.5);m.pose.pose.orientation.w=math.cos(.5)
        m.twist.twist.linear.x=.1;self.a.pose(m)
        np.testing.assert_allclose(self.a.pose_data[3],(.1*math.cos(1),.1*math.sin(1),0))

    def test_mapper_identity_must_match_and_restart_invalidates_session(self):
        a=self.a;a.topics=dict(map='map',bounds='bounds')
        pubs={'map':[NS(node_name='mapper',node_namespace='/',endpoint_gid=[1,2])],
              'bounds':[NS(node_name='mapper',node_namespace='/',endpoint_gid=[3,4])]}
        a.node=NS(get_publishers_info_by_topic=lambda t:pubs[t])
        self.assertTrue(a.mapper_owned())
        pubs['bounds'][0].node_name='wrong_mapper';self.assertFalse(a.mapper_owned())
        pubs['bounds'][0].node_name='mapper';pubs['map'][0].endpoint_gid=[5,6]
        self.assertFalse(a.mapper_owned());self.assertIn('publisher changed',a.fault)

    def test_pose_reset_latches(self):
        m=self.odom();self.a.pose(m);self.a.pose(m)
        self.assertIn('reset',self.a.fault)

    def test_unknown_cells_not_cleared(self):
        m=DistanceMapSlice();m.header.stamp.sec=10;m.header.frame_id='odom'
        m.width=2;m.height=2;m.resolution=.1;m.unknown_value=-1000.
        m.data=[-1000.,0.,-.1,1.];self.a.map(m)
        np.testing.assert_equal(self.a.grid[0],[[False,False],[False,True]])

    def test_candidate_goal_source_age_and_speed(self):
        d=dict(schema=1,frame_id='odom',goal_id='s:OUTBOUND',stamp_ns=10_000_000_000,
               path_stamp_ns=9_900_000_000,velocity_xy=[.1,0.])
        self.a.navigation(String(data=json.dumps(dict(d,goal_id='s:RETURN'))))
        self.assertIsNone(self.a.command)
        self.a.navigation(String(data=json.dumps(d)));self.assertEqual(self.a.command,('s:OUTBOUND',(.1,0.)))
        self.a.navigation(String(data=json.dumps(d)));self.assertIsNone(self.a.command)

    def test_raw_pixel_cannot_claim_metric_with_unreviewed_nominal_axes(self):
        self.a.pose(self.odom())
        d=dict(candidate_stable=True,source_stamp_ns=10_000_000_000,
            frame_id=self.a.cfg['camera']['frame_id'],center_px=[605.,483.],image_size=[1280,720])
        self.a.h(String(data=json.dumps(d)));self.assertIsNone(self.a.h_target)

    def test_projection_uses_authorized_intrinsics_with_explicit_fixture_axes(self):
        self.a.pose(self.odom());camera=self.a.cfg['camera']
        camera.update(axes_reviewed=True,body_from_optical=[[0,-1,0],[-1,0,0],[0,0,-1]])
        d=dict(candidate_stable=True,source_stamp_ns=10_000_000_000,frame_id=camera['frame_id'],
            center_px=[camera['k'][0][2],camera['k'][1][2]],image_size=[1280,720])
        self.a.h(String(data=json.dumps(d)))
        np.testing.assert_allclose(self.a.h_target,[.12,0,-.14],atol=1e-8)

    def test_projection_uses_configured_offset_not_hidden_constant(self):
        self.a.pose(self.odom());camera=self.a.cfg['camera']
        camera.update(axes_reviewed=True,offset_flu_m=[.08,.02,-.04])
        d=dict(candidate_stable=True,source_stamp_ns=10_000_000_000,frame_id=camera['frame_id'],
            center_px=[camera['k'][0][2],camera['k'][1][2]],image_size=[1280,720])
        self.a.h(String(data=json.dumps(d)))
        np.testing.assert_allclose(self.a.h_target,[.08,.02,-.14],atol=1e-8)


class ScanTests(unittest.TestCase):
    def scan(self):
        m=LaserScan();m.header.frame_id='laser';m.header.stamp.sec=10
        m.angle_min=-math.pi;m.angle_increment=2*math.pi/360;m.angle_max=m.angle_min+359*m.angle_increment
        m.range_min=.1;m.range_max=12.;m.ranges=[5.]*360;m.ranges[180]=.8
        return m

    def test_inverted_mount_preserves_forward_and_source_stamp(self):
        tf=Transform();tf.rotation.x=1.;tf.rotation.w=0.;tf.translation.z=.1
        m=transform_scan(self.scan(),tf)
        self.assertAlmostEqual(m.ranges[180],.8,places=6)
        self.assertEqual(m.header.stamp.sec,10);self.assertEqual(m.header.frame_id,'base_link')

    def test_unknown_not_invented_and_offset_rejected(self):
        tf=Transform();tf.rotation.w=1.;m=self.scan();m.ranges[180]=math.nan
        self.assertTrue(math.isnan(transform_scan(m,tf).ranges[180]))
        tf.translation.x=.1
        with self.assertRaises(ValueError):transform_scan(m,tf)
