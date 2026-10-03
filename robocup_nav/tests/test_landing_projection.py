import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from landing_projection import target_on_ground,AlignmentPreview,nominal_landing_clearance


class ProjectionTests(unittest.TestCase):
    def test_confirmed_ring_and_motor_circle_not_body_envelope(self):
        g=dict(ring_inner_diameter_m=.6,motor_diagonal_m=.4,motor_circle_center_matches_px4=True)
        self.assertAlmostEqual(nominal_landing_clearance(g),.1)
        for change in (dict(motor_circle_center_matches_px4=False),dict(motor_diagonal_m=.6),
                       dict(ring_inner_diameter_m=float('nan')),dict(motor_diagonal_m=True)):
            with self.assertRaises(ValueError):nominal_landing_clearance(dict(g,**change))

    def test_nominal_config_axes_have_correct_signed_directions_but_do_not_release(self):
        import yaml
        cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/roundtrip_task.yaml').read_text())
        camera=cfg['camera']
        self.assertFalse(camera['axes_reviewed'])
        with self.assertRaises(ValueError):
            self.call(body_from_optical=camera['body_from_optical'],axes_verified=camera['axes_reviewed'])
        center=self.call(body_from_optical=camera['body_from_optical'])
        forward=self.call(pixel=[320,140],body_from_optical=camera['body_from_optical'])
        right=self.call(pixel=[420,240],body_from_optical=camera['body_from_optical'])
        self.assertGreater(forward[0],center[0]);self.assertLess(right[1],center[1])

    def call(self,**kw):
        args=dict(pixel=[320,240],k=[[500,0,320],[0,500,240],[0,0,1]],distortion=[0]*5,
                  image_size=[640,480],calibration_size=[640,480],body_position=[0,0,1],
                  odom_from_body=np.eye(3),body_from_optical=[[0,-1,0],[-1,0,0],[0,0,-1]],
                  camera_offset=[.12,0,-.055],ground_z=0,calibration_verified=True,axes_verified=True)
        args.update(kw);return target_on_ground(**args)

    def test_camera_center_not_body_center(self):
        np.testing.assert_allclose(self.call(),[.12,0,0],atol=1e-6)

    def test_image_right_is_body_right_only_for_verified_fixture(self):
        self.assertLess(self.call(pixel=[420,240])[1],0)

    def test_refuse_unknown_axes_or_resolution(self):
        for kw in [dict(axes_verified=False),dict(calibration_verified=False),dict(calibration_size=[1280,720]),dict(body_from_optical=np.eye(3)),dict(ground_z=1)]:
            with self.assertRaises(ValueError):self.call(**kw)

    def test_stale_resets_settle(self):
        a=AlignmentPreview()
        self.assertFalse(a.step(1,1,1,[0,0],[0,0])['aligned'])
        for t in np.arange(1.1,3.2,.1):out=a.step(float(t),float(t),float(t),[0,0],[0,0])
        self.assertTrue(out['aligned'])
        out=a.step(4,3,4,[0,0],[0,0]);self.assertFalse(out['aligned']);self.assertEqual(out['velocity_xy'],[0,0])
        self.assertFalse(a.step(4.1,4.1,4.1,[0,0],[0,0])['aligned'])
        self.assertFalse(out['descent_allowed'])

    def test_malformed_vectors_never_align_and_clear_history(self):
        for bad in ([],[0],[0,0,0],[[0,0]],None,['bad',0],[float('nan'),0]):
            for invalid_error in (True,False):
                a=AlignmentPreview()
                for i in range(25):
                    t=i*.1
                    a.step(t,t,t,[0,0],[0,0])
                out=a.step(2.5,2.5,2.5,bad if invalid_error else [0,0],
                           [0,0] if invalid_error else bad)
                self.assertFalse(out['aligned'])
                self.assertEqual(out['velocity_xy'],[0,0])
                self.assertFalse(a.step(2.6,2.6,2.6,[0,0],[0,0])['aligned'])

    def test_invalid_clock_does_not_poison_recovery(self):
        for bad in (None,'invalid',float('nan'),[1,2]):
            a=AlignmentPreview()
            self.assertFalse(a.step(bad,0,0,[0,0],[0,0])['aligned'])
            self.assertFalse(a.step(1,1,1,[0,0],[0,0])['aligned'])


if __name__=='__main__':unittest.main()
