import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from cruise_band import make_band
from navigation_landing_shadow import Scene
from roundtrip_flight_test import RoundTripMission


class RoundTripTests(unittest.TestCase):
    def new(self, yaw=0., check=lambda p,v,c:True):
        return RoundTripMission(make_band(rise=.86,ground_clearance=.14),(-4,4,-4,4),check,
            ground_origin=(0.,0.,0.),yaw_odom=yaw,session_id='unit')

    def scene(self,m,t,p=None,**kw):
        data=dict(at=t,position=(*m.home,m.band.center) if p is None else p,
            velocity_xy=(0.,0.),localization_ok=True,airborne=True,
            map_at=t,footprint_known_free=True,navigation_at=t,
            navigation_velocity_odom=(.1,0.),navigation_goal_id=m.goal_id,tilt_rad=0.,
            h_at=t,h_metric_verified=True,h_error_odom=(0.,0.),landing_boundary_verified=True)
        data.update(kw)
        return Scene(**data)

    def outbound_complete(self,m):
        for i in range(24):
            t=i*.1
            out=m.step(t,self.scene(m,t,(*m.endpoint,m.band.center)))
            if m.leg=='RETURN':return t,out
        self.fail('outbound did not settle')

    def test_heading_and_exact_distance(self):
        for yaw in (0.,.7,math.pi/2,math.pi,-math.pi/2):
            m=self.new(yaw)
            self.assertAlmostEqual(math.dist(m.home,m.endpoint),2.5)
            self.assertAlmostEqual(m.endpoint[0],2.5*math.cos(yaw))
            self.assertAlmostEqual(m.endpoint[1],2.5*math.sin(yaw))

    def test_reject_height_or_bounds_mismatch(self):
        with self.assertRaises(ValueError):
            RoundTripMission(make_band(),(-4,4,-4,4),None,
                ground_origin=(0,0,0),yaw_odom=0,session_id='a')
        with self.assertRaises(ValueError):
            RoundTripMission(make_band(rise=.86,ground_clearance=.14),(-1,1,-1,1),None,
                ground_origin=(0,0,0),yaw_odom=0,session_id='a')

    def test_absolute_height_uses_ground_plane_and_arbitrary_odom_origin(self):
        for z in (-3.,0.,8.1):
            band=make_band(initial_z=z,rise=.86,ground_clearance=.14)
            m=RoundTripMission(band,(-4,4,-4,4),None,
                ground_origin=(0,0,z),yaw_odom=0,session_id='height')
            self.assertAlmostEqual(m.band.center-m.band.floor,1.)
            self.assertAlmostEqual(m.band.center-z,.86)
        for rise,clearance in ((1.,.14),(.86,.15)):
            with self.assertRaises(ValueError):
                RoundTripMission(make_band(rise=rise,ground_clearance=clearance),(-4,4,-4,4),None,
                    ground_origin=(0,0,0),yaw_odom=0,session_id='wrong-height')

    def test_outbound_never_lands_and_old_goal_cannot_drive_return(self):
        m=self.new();old=m.goal_id
        t,out=self.outbound_complete(m)
        self.assertEqual(m.leg,'RETURN')
        self.assertEqual(out['velocity_odom_xy'],[0,0])
        self.assertEqual(m.state,'NAVIGATE')
        out=m.step(t+.1,self.scene(m,t+.1,(2.4,0,m.band.center),navigation_goal_id=old,goal_reached=True))
        self.assertEqual(out['velocity_odom_xy'],[0,0])
        self.assertIn('goal ID',out['reason'])

    def test_goal_reached_flag_cannot_skip_route(self):
        m=self.new()
        for i in range(30):
            t=i*.1
            out=m.step(t,self.scene(m,t,goal_reached=True))
        self.assertEqual(m.leg,'OUTBOUND')
        self.assertEqual(m.state,'NAVIGATE')

    def test_both_arrivals_require_continuous_dwell_then_h(self):
        m=self.new();t,_=self.outbound_complete(m)
        for i in range(1,25):
            at=t+i*.1
            out=m.step(at,self.scene(m,at))
        self.assertEqual(m.leg,'RETURN')
        self.assertIn(m.state,('ALIGN_H','LAND_REQUEST'))
        for i in range(25,50):
            at=t+i*.1
            out=m.step(at,self.scene(m,at))
        self.assertEqual(m.state,'LAND_REQUEST')
        self.assertFalse(out['flight_authorized'])

    def test_unknown_map_or_blocked_stopping_space_holds(self):
        for kw,check in ((dict(footprint_known_free=False),lambda p,v,c:True),
                         ({},lambda p,v,c:False),
                         (dict(map_at=-1),lambda p,v,c:True)):
            m=self.new(check=check)
            out=m.step(0,self.scene(m,0,**kw))
            self.assertEqual(out['velocity_odom_xy'],[0,0])
            self.assertEqual(m.leg,'OUTBOUND')

    def test_missing_h_never_descends_at_home(self):
        m=self.new();t,_=self.outbound_complete(m)
        for i in range(1,75):
            at=t+i*.1
            out=m.step(at,self.scene(m,at,h_metric_verified=False))
        self.assertEqual(m.state,'ABORT')
        self.assertEqual(out['velocity_odom_xy'],[0,0])

    def test_takeover_and_stale_localization_latch(self):
        for kw,want in ((dict(manual_override=True),'HANDOVER'),
                        (dict(localization_ok=False),'ABORT')):
            m=self.new();out=m.step(0,self.scene(m,0,**kw))
            self.assertEqual(m.state,want)
            out=m.step(.1,self.scene(m,.1))
            self.assertEqual(out['velocity_odom_xy'],[0,0])
            self.assertEqual(m.state,want)


if __name__=='__main__':unittest.main()
