"""Existing ground/ACK/PX4-message/H-descent chain with the new two-leg task.

Ideal plant, perfect map and synthetic H observations. No ROS publishers.
"""
from pathlib import Path
import sys
import unittest
import math
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from test_sortie_shadow import Rig
from cruise_band import make_band
from roundtrip_flight_test import RoundTripMission
from navigation_shadow_pipeline import ShadowPipeline
from landing_sequence_shadow import LandingSequenceShadow
from sortie_shadow import SortieShadow


class RoundTripRig(Rig):
    def __init__(self):
        super().__init__()
        self.mission = RoundTripMission(make_band(rise=.86,ground_clearance=.14),(-1,4,-2,2),
            lambda p,v,c:True,ground_origin=(0,0,0),yaw_odom=0,session_id='synthetic-roundtrip')
        self.core = SortieShadow(LandingSequenceShadow(
            ShadowPipeline(self.mission,self.alignment,0.),.05),height=.86,hover=1.)
        self.max_forward = 0.
        self.max_px4_height = .14
        self.legs = set()

    def tick(self,scene_kw=None,**kwargs):
        goal = self.mission.planner_goal()
        delta = np.array(goal['position'][:2])-self.p[:2]
        command = .6*delta
        norm = float(np.linalg.norm(command))
        if norm > .15:command *= .15/norm
        scene = dict(navigation_goal_id=goal['goal_id'],navigation_velocity_odom=tuple(command),
                     goal_reached=False,h_error_odom=(-self.p[0],-self.p[1]))
        scene.update(scene_kw or {})
        result = super().tick(scene_kw=scene,**kwargs)
        self.max_forward = max(self.max_forward,float(self.p[0]))
        self.max_px4_height = max(self.max_px4_height,float(self.p[2])+.14)
        self.legs.add(self.mission.leg)
        return result

    def run(self):
        for _ in range(2400):
            out = self.tick()
            if self.core.state in self.core.TERMINAL:return out
        raise AssertionError('bounded 120s simulated mission unfinished')


class RoundTripSortieTests(unittest.TestCase):
    def test_full_ground_outbound_return_h_touchdown(self):
        rig = RoundTripRig();out = rig.run()
        self.assertEqual(out['state'],'DONE',out)
        self.assertEqual(rig.legs,{'OUTBOUND','RETURN'})
        self.assertGreater(rig.max_forward,2.4)
        self.assertGreater(rig.max_px4_height,.99)
        self.assertLessEqual(rig.max_px4_height,1.00001)
        self.assertLess(rig.max_forward,2.6)
        self.assertLess(math.hypot(*rig.p[:2]),.05)
        self.assertLess(rig.p[2],.003)
        self.assertFalse(rig.armed)
        self.assertTrue({'TAKEOFF','HOVER','NAVIGATE','ALIGN_H','DESCEND','TOUCHDOWN','DONE'}
                        <= {r['state'] for r in rig.trace})
        self.assertFalse(any(r['flight_authorized'] for r in rig.trace))

    def test_return_goal_swap_cannot_reuse_outbound_command(self):
        rig = RoundTripRig()
        old = rig.mission.goal_id
        while rig.mission.leg=='OUTBOUND':
            out=rig.tick()
            self.assertNotIn(out['state'],rig.core.TERMINAL,out)
        out=rig.tick(scene_kw=dict(navigation_goal_id=old,navigation_velocity_odom=(.15,0.)))
        sp=out['messages']['trajectory_setpoint']
        np.testing.assert_allclose(sp.velocity[:2],[0,0],atol=1e-8)
        self.assertNotIn('vehicle_command',out['messages'])

    def test_original_takeoff_departure_protection_still_active(self):
        rig=RoundTripRig()
        while rig.core.state!='TAKEOFF':rig.tick()
        rig.p[0]=.31
        out=rig.tick()
        self.assertEqual(out['state'],'ABORT')
        self.assertIn('horizontal departure',out['reason'])


if __name__=='__main__':unittest.main()
