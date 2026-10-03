"""Integrated candidate chain with synthetic verified geometry, NOT flight proof."""
import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from cruise_band import make_band
from navigation_landing_shadow import MissionShadow,Scene
from navigation_shadow_pipeline import ShadowPipeline
from navigation_frame_contract import SessionAlignment
from landing_sequence_shadow import LandingSequenceShadow,LandingEvidence


class SequenceTests(unittest.TestCase):
    def make(self):
        mission=MissionShadow(make_band(),(-2,2,-2,2),lambda p,v,c:True,landing_error_limit=.04)
        nav=ShadowPipeline(mission,SessionAlignment([0,0,0],0,[0,0,0],0,'s',(0,)*5,verified=True),0.)
        return LandingSequenceShadow(nav,.04)

    def tick(self,m,t,z=.6,scene_kw=None,evidence_kw=None,frame_kw=None):
        scene=Scene(at=t,position=(0.,0.,z),tilt_rad=0.,velocity_xy=(0.,0.),
            localization_ok=True,airborne=True,map_at=t,footprint_known_free=True,
            navigation_at=t,goal_reached=True,h_error_odom=(.01,0.),h_at=t,
            h_metric_verified=True,landing_boundary_verified=True)
        # These exact dimensions/flags belong ONLY to this synthetic fixture.
        evidence=LandingEvidence(t,-.14,.14,geometry_verified=True,column_clear=True,bounds_verified=True)
        scene=replace(scene,**(scene_kw or {}));evidence=replace(evidence,**(evidence_kw or {}))
        frame=dict(timestamp_us=int((t+1)*1e6),vio_session='s',reset_counters=(0,)*5,
                   exclusive_writer_verified=True,yaw_rate_odom=0.)
        frame.update(frame_kw or {})
        return m.step(t,scene,evidence,**frame)

    def ready(self,m):
        for i in range(23):r=self.tick(m,i*.1)
        self.assertEqual(r['state'],'DESCEND',r)

    def test_navigation_alignment_descent_touchdown_single_message_path(self):
        m=self.make();z=.6;states=set()
        for i in range(600):
            t=i*.05;landed=z<=.002
            if landed:z=0.
            r=self.tick(m,t,z,scene_kw=dict(h_metric_verified=z>.20,landed=landed,airborne=not landed),
                        evidence_kw=dict(armed=not landed))
            states.add(r['state']);self.assertFalse(r['flight_authorized'])
            self.assertNotIn('vehicle_command',r['messages'])
            if r['messages']:
                sp=r['messages']['trajectory_setpoint']
                self.assertTrue(math.isnan(sp.position[0]) and math.isnan(sp.position[1]))
                self.assertTrue(math.isnan(sp.velocity[2]))
                target=-sp.position[2]
                self.assertGreaterEqual(target,0.)
                z+=max(-.1,min(.1,(target-z)/.2))*.05
            if r['state']=='DONE':break
            self.assertNotIn(r['state'],('ABORT','HANDOVER'),r)
        self.assertEqual(r['state'],'DONE',r)
        self.assertTrue({'ALIGN_H','DESCEND','TERMINAL_DESCEND','TOUCHDOWN','DONE'}<=states,states)

    def test_unverified_landing_geometry_never_descends(self):
        m=self.make()
        for i in range(30):r=self.tick(m,i*.1,evidence_kw=dict(geometry_verified=False))
        self.assertEqual(r['state'],'LAND_REQUEST')
        self.assertAlmostEqual(r['messages']['trajectory_setpoint'].position[2],-.6,places=5)
        self.assertIsNone(m.anchor)

    def test_h_loss_above_terminal_holds_vertical_reference(self):
        m=self.make();self.ready(m)
        r=self.tick(m,2.3,scene_kw=dict(h_metric_verified=False))
        self.assertEqual(r['state'],'DESCEND',r)
        self.assertAlmostEqual(r['height_target_odom'],.6)

    def test_reset_geometry_clearance_and_ownership_faults_latch(self):
        for kwargs in (dict(frame_kw=dict(vio_session='new')),
                       dict(frame_kw=dict(exclusive_writer_verified=False)),
                       dict(evidence_kw=dict(column_clear=False)),
                       dict(evidence_kw=dict(ground_z=-.13)),
                       dict(scene_kw=dict(map_at=0)),
                       dict(scene_kw=dict(localization_ok=False)),
                       dict(scene_kw=dict(h_error_odom=(.2,0.)))):
            m=self.make();self.ready(m);r=self.tick(m,2.3,**kwargs)
            self.assertEqual(r['state'],'ABORT',r);self.assertFalse(r['messages'])
            self.assertFalse(self.tick(m,2.4)['messages'])

    def test_manual_takeover_and_touchdown_loss(self):
        m=self.make();self.ready(m)
        self.assertEqual(self.tick(m,2.3,scene_kw=dict(manual_override=True))['state'],'HANDOVER')
        m=self.make();self.ready(m)
        r=self.tick(m,2.3,scene_kw=dict(landed=True),evidence_kw=dict(armed=True))
        self.assertEqual(r['state'],'TOUCHDOWN')
        self.assertEqual(self.tick(m,2.4)['state'],'ABORT')


if __name__=='__main__':unittest.main()
