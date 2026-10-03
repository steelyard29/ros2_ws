import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from cruise_band import make_band
from navigation_landing_shadow import Scene,MissionShadow
from stopping_space import stopping_space_clear
import numpy as np


class MissionTests(unittest.TestCase):
    # Mock geometry only for transition tests, never flight evidence.
    def new(self):return MissionShadow(make_band(),(-3,3,-2,2),lambda p,v,c: True)
    def scene(self,t,**kw):
        fields=dict(at=t,position=(0,0,.6),tilt_rad=0.,localization_ok=True,airborne=True,map_at=t,
                    footprint_known_free=True,navigation_at=t,navigation_velocity_odom=(.1,0))
        fields.update(kw);return Scene(**fields)

    def test_ground_and_unknown_start_never_moves(self):
        m=self.new()
        self.assertEqual(m.step(0,self.scene(0,airborne=False,position=(0,0,0)))['velocity_odom_xy'],[0,0])
        self.assertEqual(m.step(.1,self.scene(.1,footprint_known_free=False))['velocity_odom_xy'],[0,0])

    def test_rehearsal_navigation_align_land_intention(self):
        m=self.new();self.assertEqual(m.step(0,self.scene(0))['state'],'NAVIGATE')
        for i in range(1,26):
            t=i*.1
            out=m.step(t,self.scene(t,goal_reached=True,h_metric_verified=True,h_at=t,landing_boundary_verified=True))
        self.assertEqual(out['state'],'LAND_REQUEST');self.assertFalse(out['flight_authorized'])
        self.assertEqual(m.step(2.6,self.scene(2.6,landed=True,airborne=False))['state'],'DONE')

    def test_h_loss_and_override_latch(self):
        m=self.new()
        for i in range(35):
            t=i*.1;out=m.step(t,self.scene(t,goal_reached=True))
        self.assertEqual(out['state'],'ABORT')
        self.assertEqual(m.step(3.5,self.scene(3.5,h_metric_verified=True,h_at=3.5))['state'],'ABORT')
        m=self.new();self.assertEqual(m.step(0,self.scene(0,manual_override=True))['state'],'HANDOVER')

    def test_boundary_required(self):
        m=self.new()
        for i in range(30):
            t=i*.1;out=m.step(t,self.scene(t,goal_reached=True,h_metric_verified=True,h_at=t))
        self.assertEqual(out['state'],'ALIGN_H')

    def test_tilt_must_be_known_and_inside_envelope(self):
        for tilt in (float('nan'),float('inf'),.2,-.2):
            m=self.new()
            out=m.step(0,self.scene(0,tilt_rad=tilt))
            self.assertEqual(out['state'],'WAIT_CRUISE')
            self.assertEqual(out['velocity_odom_xy'],[0,0])

    def test_no_stopping_evidence_inhibits_navigation_and_alignment(self):
        for align in (False,True):
            m=MissionShadow(make_band(),(-3,3,-2,2))
            for i in range(30):
                t=i*.1
                out=m.step(t,self.scene(t,goal_reached=align,h_metric_verified=True,
                           h_at=t,landing_boundary_verified=True))
                self.assertEqual(out['velocity_odom_xy'],[0,0])
                self.assertNotEqual(out['state'],'LAND_REQUEST')

    def test_map_backed_stopping_check(self):
        free=np.ones((80,80),dtype=bool)
        def check(p,v,c):
            return stopping_space_clear(free,.05,(-2,-2),p,v,c,
                body_radius=.43,uncertainty=.05,latency=.3,braking=.2)
        m=MissionShadow(make_band(),(-2,2,-2,2),check)
        self.assertEqual(m.step(0,self.scene(0))['velocity_odom_xy'],[.1,0])
        # Map changes while pose and command remain fresh.
        free[:,49]=False
        self.assertEqual(m.step(.1,self.scene(.1))['velocity_odom_xy'],[0,0])
        free[:]=True
        self.assertEqual(m.step(.2,self.scene(.2))['velocity_odom_xy'],[.1,0])

    def test_pending_landing_intention_is_revoked(self):
        for fault in (dict(h_at=0),dict(h_metric_verified=False),
                      dict(landing_boundary_verified=False),dict(map_at=0),
                      dict(footprint_known_free=False),dict(h_error_odom=(.2,0)),
                      dict(velocity_xy=(.1,0)),dict(tilt_rad=.3)):
            m=self.new()
            for i in range(25):
                t=i*.1
                out=m.step(t,self.scene(t,goal_reached=True,h_metric_verified=True,
                           h_at=t,landing_boundary_verified=True))
            self.assertEqual(out['state'],'LAND_REQUEST')
            args=dict(goal_reached=True,h_metric_verified=True,h_at=2.5,landing_boundary_verified=True)
            args.update(fault)
            out=m.step(2.5,self.scene(2.5,**args))
            self.assertEqual(out['state'],'ALIGN_H')
            self.assertEqual(out['velocity_odom_xy'],[0,0])
            out=m.step(2.6,self.scene(2.6,goal_reached=True,h_metric_verified=True,
                       h_at=2.6,landing_boundary_verified=True))
            self.assertNotEqual(out['state'],'LAND_REQUEST')


if __name__=='__main__':unittest.main()
