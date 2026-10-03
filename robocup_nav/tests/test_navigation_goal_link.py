from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_goal_link import GoalLink


class LinkTests(unittest.TestCase):
    def setUp(self):
        self.l=GoalLink();self.goal=dict(goal_id='s:OUTBOUND',frame_id='odom',position=[2.5,0,1],yaw=0.)
        self.l.select(self.goal);self.ticket=self.l.request(1_000_000_000)
        self.assertTrue(self.l.accept_path(self.ticket,[(0,0),(2.5,0)],1_010_000_000))
        self.d=dict(schema=1,valid=True,frame_id='odom',stamp_ns=1_100_000_000,
            path_stamp_ns=1_000_000_000,odom_stamp_ns=1_080_000_000,
            scan_stamp_ns=1_070_000_000,velocity_xy=[.1,.02])

    def test_atomic_command_and_replay(self):
        self.assertEqual(self.l.command(self.d,1_110_000_000).goal_id,'s:OUTBOUND')
        self.assertIsNone(self.l.command(self.d,1_120_000_000))

    def test_goal_switch_rejects_old_path_and_candidate(self):
        self.l.select(dict(self.goal,goal_id='s:RETURN',position=[0,0,1]))
        self.assertFalse(self.l.accept_path(self.ticket,[(2.5,0)],1_010_000_000))
        self.assertIsNone(self.l.command(self.d,1_110_000_000))

    def test_same_goal_cannot_mutate(self):
        with self.assertRaises(ValueError):self.l.select(dict(self.goal,position=[1,0,1]))

    def test_old_request_and_wrong_endpoint(self):
        ticket=self.l.request(1_020_000_000)
        self.assertFalse(self.l.accept_path(self.ticket,[(2.5,0)],1_030_000_000))
        self.assertFalse(self.l.accept_path(ticket,[(2,0)],1_030_000_000))

    def test_source_age_frame_overspeed_and_invalid(self):
        for patch in ({'scan_stamp_ns':1},{'stamp_ns':2_000_000_000},
                      {'frame_id':'base_link'},{'velocity_xy':[.16,0]},
                      {'velocity_xy':[float('nan'),0]},{'valid':False},
                      {'path_stamp_ns':1_010_000_000}):
            with self.subTest(patch=patch):self.assertIsNone(self.l.command(dict(self.d,**patch),1_110_000_000))

    def test_stale_path_rejected(self):
        self.assertIsNone(self.l.command(self.d,1_600_000_000))
