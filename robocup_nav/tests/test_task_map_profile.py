import importlib.util
import math
from pathlib import Path
import sys
import unittest
from builtin_interfaces.msg import Time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from task_map_profile import task_band,mapper_profile,MapBandEvidence,BOOTSTRAP_MARGIN
from task_map_fixture import markers


class MapProfileTests(unittest.TestCase):
    def test_navigation_processes_inherit_one_domain(self):
        from launch import LaunchContext
        from launch.actions import ExecuteProcess
        from tempfile import TemporaryDirectory,NamedTemporaryFile
        from unittest.mock import patch
        path=Path(__file__).resolve().parents[1]/'launch/task_navigation.launch.py'
        spec=importlib.util.spec_from_file_location('task_nav_domain_test',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        c=LaunchContext();c.environment['ROS_DOMAIN_ID']='176'
        c.launch_configurations.update(center_z='.86',apf_executable='/usr/bin/true')
        with TemporaryDirectory() as tmp,patch.object(module,'NamedTemporaryFile',
                side_effect=lambda **kwargs:NamedTemporaryFile(dir=tmp,**kwargs)):
            actions=module.setup(c)
            processes=[a for a in actions if isinstance(a,ExecuteProcess)]
            self.assertEqual(len(processes),6)
            for p in processes:
                self.assertIsNone(p.env)
                self.assertIsNone(p.additional_env)

    def evidence(self,initial_z=0.):
        e=MapBandEvidence();e.slice(initial_z+.86,10.)
        for m in markers(Time(sec=10),initial_z):self.assertTrue(e.marker(m,10.))
        return e

    def test_actual_height_and_ground_reference(self):
        b=task_band(3.);self.assertAlmostEqual(b.center,3.86)
        self.assertAlmostEqual(b.floor,2.86);self.assertAlmostEqual(b.center-b.floor,1.)
        self.assertTrue(self.evidence(3.).check(b,10.1))

    def test_wrong_old_rise_rejected_despite_fresh_known_free_slice(self):
        e=self.evidence();e.slice(.6,10.)
        self.assertFalse(e.check(task_band(0.),10.1))
        self.assertIn('does not cover',e.reason)

    def test_narrow_bound_rejected(self):
        e=self.evidence();m=next(markers(Time(sec=10)))
        for p in m.points:p.z=.8
        e.marker(m,10.);self.assertFalse(e.check(task_band(0.),10.1))

    def test_stale_missing_mismatched_pairs(self):
        e=self.evidence();self.assertFalse(e.check(task_band(0.),10.8))
        e=self.evidence();e.bounds.pop('top_height_limit');self.assertFalse(e.check(task_band(0.),10.1))
        e=MapBandEvidence();e.slice(.86,10.)
        e.marker(list(markers(Time(sec=10)))[0],10.)
        e.marker(list(markers(Time(sec=10,nanosec=1)))[1],10.)
        self.assertFalse(e.check(task_band(0.),10.1))

    def test_interleaved_next_pair_does_not_falsely_drop_valid_current_pair(self):
        e=self.evidence();pair=list(markers(Time(sec=10,nanosec=200_000_000)))
        e.marker(pair[0],10.2);self.assertTrue(e.check(task_band(0.),10.2))
        self.assertEqual(e.bounds['top_height_limit'][0],10_000_000_000)
        e.marker(pair[1],10.2);self.assertTrue(e.check(task_band(0.),10.2))
        self.assertEqual(e.bounds['top_height_limit'][0],10_200_000_000)

    def test_half_voxel_bootstrap_margin_requires_full_containment(self):
        e=self.evidence();self.assertTrue(e.check(task_band(.02),10.1))
        self.assertFalse(e.check(task_band(BOOTSTRAP_MARGIN+.001),10.1))

    def test_invalid_frame_action_nonflat_nan_center(self):
        for kind in ('frame','delete','nonflat'):
            e=self.evidence();m=next(markers(Time(sec=10)))
            if kind=='frame':m.header.frame_id='base_link'
            if kind=='delete':m.action=2
            if kind=='nonflat':m.points[0].z+=.1
            self.assertFalse(e.marker(m,10.));self.assertFalse(e.check(task_band(0.),10.1))
        e=self.evidence();e.slice(math.nan,10.);self.assertFalse(e.check(task_band(0.),10.1))

    def test_ground_task_launch_forwards_identical_band_no_px4(self):
        from launch import LaunchContext
        from launch.utilities import perform_substitutions
        path=Path(__file__).resolve().parents[1]/'launch/ground_task_navigation.launch.py'
        spec=importlib.util.spec_from_file_location('ground_task_launch_test',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        declarations=module.generate_launch_description().entities
        self.assertIsNone(declarations[0].default_value)  # No silent odom-z assumption.
        c=LaunchContext();c.launch_configurations.update(initial_z='2.',apf_executable='/fixture/apf')
        includes=module.setup(c);self.assertEqual(len(includes),3)
        actual={k:(v if isinstance(v,str) else perform_substitutions(c,v))
                for k,v in includes[1].launch_arguments}
        for k,v in mapper_profile(2.).items():self.assertAlmostEqual(float(actual[k]),v)
        self.assertFalse(any('px4' in str(x) for x in includes))
