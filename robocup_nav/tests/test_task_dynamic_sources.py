import copy
import json
from pathlib import Path
import unittest
import subprocess
import sys
import yaml
import numpy as np
from task_dynamic_sources import checked_pose,h_observation,ideal_navigation,messages
from landing_projection import target_on_ground


class DynamicSourceTests(unittest.TestCase):
    def test_full_entry_default_and_invalid_worker_do_not_start(self):
        script=Path(__file__).resolve().parent/'task_full_pipe_check.py'
        result=subprocess.run([sys.executable,str(script)],capture_output=True,text=True,timeout=5)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn('Describe only',result.stdout)
        result=subprocess.run([sys.executable,str(script),'--source'],capture_output=True,text=True,timeout=5)
        self.assertEqual(result.returncode,2)
        self.assertIn('bounded private source',result.stderr)

    def test_long_reader_cannot_extend_real_or_readonly_sessions(self):
        from task_pipe_authority import reader_budget
        self.assertEqual(reader_budget(),25.)
        self.assertEqual(reader_budget(isolated=True,navigation_writes=True),25.)
        self.assertEqual(reader_budget(live=True),200.)
        self.assertEqual(reader_budget(isolated=True,navigation_writes=True,isolated_sortie=True),150.)
        for args in ({},{'isolated':True},{'navigation_writes':True},
                     {'isolated':True,'navigation_writes':True,'live':True}):
            with self.assertRaises(ValueError):reader_budget(**args,isolated_sortie=True)
        reader=Path(__file__).resolve().parents[1]/'scripts/task_perception_reader.py'
        rejected=subprocess.run([sys.executable,str(reader),'--isolated-sortie'],
                                capture_output=True,text=True,timeout=5)
        self.assertEqual(rejected.returncode,2)
        self.assertIn('isolated navigation only',rejected.stderr)

    def setUp(self):
        self.cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/roundtrip_task.yaml').read_text())
        self.record=dict(session='synthetic',at=1.,stamp_ns=10_000_000_000,
                         position=[0.,0.,.86],velocity=[.1,0.,0.])

    def test_snapshot_session_age_and_geometry(self):
        checked_pose(self.record,'synthetic',1.05)
        for changes in (dict(session='old'),dict(at=.8),dict(at=1.2),
                        dict(position=[0.,float('nan'),0.]),dict(velocity=[0.,0.])):
            with self.assertRaises(ValueError):checked_pose(dict(self.record,**changes),'synthetic',1.05)

    def test_h_disappears_outside_view_and_reprojects_home(self):
        camera=self.cfg['camera']
        self.assertFalse(h_observation([2.5,0.,.86],camera,1)['candidate_stable'])
        for p in ([0.,0.,.86],[.04,-.03,.86]):
            h=h_observation(p,camera,1);self.assertTrue(h['candidate_stable'])
            recovered=target_on_ground(h['center_px'],camera['k'],camera['distortion'],
                h['image_size'],camera['image_size'],p,np.eye(3),camera['body_from_optical'],
                camera['offset_flu_m'],-.14,calibration_verified=True,axes_verified=True)
            np.testing.assert_allclose(recovered,[0.,0.,-.14],atol=1e-5)
        self.assertFalse(camera['axes_reviewed'])  # Never edits production approval.

    def test_goal_leg_and_direction_change(self):
        outbound=dict(frame_id='odom',goal_id='s:OUTBOUND',position=[2.5,0.,.86])
        returning=dict(frame_id='odom',goal_id='s:RETURN',position=[0.,0.,.86])
        a=ideal_navigation([1.,0.,.86],outbound,10)
        b=ideal_navigation([1.,0.,.86],returning,11)
        self.assertEqual(a['velocity_xy'],[.15,0.]);self.assertEqual(b['velocity_xy'],[-.15,0.])
        self.assertEqual(b['goal_id'],'s:RETURN');self.assertTrue(b['synthetic_ideal_navigation'])

    def test_generated_inputs_share_snapshot_stamp_and_move(self):
        cfg_before=copy.deepcopy(self.cfg)
        result=messages(self.record,'synthetic',1.05,self.cfg['camera'],None)
        self.assertEqual(result['vio'].pose.pose.position.z,.86)
        self.assertAlmostEqual(result['vio'].twist.twist.linear.x,.1)
        self.assertEqual(result['map'].origin.x,-1.8)
        self.assertEqual({m.header.stamp.sec for m in result['bounds']},{10})
        self.assertEqual(json.loads(result['h'].data)['source_stamp_ns'],10_000_000_000)
        self.assertEqual(cfg_before,self.cfg)
        moved=dict(self.record,position=[2.5,0.,.86])
        after=messages(moved,'synthetic',1.05,self.cfg['camera'],None)
        self.assertEqual(after['vio'].pose.pose.position.x,2.5)
        self.assertFalse(json.loads(after['h'].data)['candidate_stable'])
