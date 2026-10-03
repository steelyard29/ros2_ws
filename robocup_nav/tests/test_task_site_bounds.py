"""Synthetic geometry only: no site inspection, ROS, or flight output."""
import copy
import math
from pathlib import Path
import sys
import unittest
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from task_site_bounds import resolve_bounds,inside_bounds,TakeoffRectangle
from roundtrip_flight_test import RoundTripMission
from cruise_band import make_band


class SiteBoundsTests(unittest.TestCase):
    def setUp(self):
        self.cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/roundtrip_task.yaml').read_text())

    def test_confirmed_site_is_inset_once_and_sortie_still_2p5(self):
        for yaw in (0.,math.pi/4,math.pi/2,-2.1):
            origin=(10.,-7.)
            bounds=resolve_bounds(self.cfg,origin,yaw)
            self.assertIs(type(bounds),TakeoffRectangle)
            for value,want in zip(bounds.limits,(-1.32,5.72,-2.02,2.02)):
                self.assertAlmostEqual(value,want)
            mission=RoundTripMission(make_band(rise=.86,ground_clearance=.14),bounds,lambda *a:True,
                                     ground_origin=(*origin,0.),yaw_odom=yaw,session_id='synthetic')
            self.assertAlmostEqual(math.dist(mission.home,mission.endpoint),2.5)
            self.assertTrue(inside_bounds(bounds,mission.endpoint))

    def test_rotated_aabb_corner_outside_site_is_not_accepted(self):
        bounds=resolve_bounds(self.cfg,(0.,0.),math.pi/4)
        point=(-2.2/math.sqrt(2),2.2/math.sqrt(2))
        # This point is inside the rotated rectangle's outer AABB, but outside
        # the actual y-left center boundary (2.02m).
        self.assertTrue(-3.34/math.sqrt(2)<point[0]<7.74/math.sqrt(2))
        self.assertFalse(inside_bounds(bounds,point))

    def test_stopping_disk_must_fit_and_touching_is_rejected(self):
        bounds=resolve_bounds(self.cfg,(0.,0.),0.)
        self.assertTrue(inside_bounds(bounds,(5.7,0.)))
        self.assertFalse(inside_bounds(bounds,(5.7,0.),.1))
        self.assertFalse(inside_bounds(bounds,(bounds.limits[1],0.)))
        self.assertFalse(inside_bounds(bounds,(float('nan'),0.)))

    def test_missing_measurement_and_impossible_inset_rejected(self):
        for value in (None,0.,-1.,float('nan'),True,.1):
            cfg=copy.deepcopy(self.cfg)
            cfg['site_measurement']['distances_to_physical_boundary_m']['forward']=value
            if value==.1 and type(value) is float:
                cfg['site_measurement']['distances_to_physical_boundary_m']['backward']=.1
            with self.assertRaises(ValueError):resolve_bounds(cfg,(0.,0.),0.)

    def test_explicit_legacy_odom_bounds_are_not_rotated_again(self):
        self.cfg['bounds_odom']=[-1,4,-2,2]
        self.assertEqual(resolve_bounds(self.cfg,(10,20),math.pi/2),(-1,4,-2,2))

    def test_configured_full_sortie_uses_same_rotated_site_and_h_geometry(self):
        from task_configured_sortie_replay import replay
        for yaw in (0.,math.pi/4,math.pi/2):
            with self.subTest(yaw=yaw):
                result=replay(yaw)
                self.assertTrue(result['passed'],result)
                self.assertFalse(result['production_axes_reviewed'])
                self.assertGreater(result['out_of_image_h_samples'],0)


if __name__=='__main__':unittest.main()
