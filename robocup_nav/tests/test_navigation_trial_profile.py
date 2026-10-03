import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from navigation_trial_profile import projected_width, required_clearance, apply_trial_profile


class TrialTests(unittest.TestCase):
    def test_gate_rotation(self):
        self.assertAlmostEqual(projected_width(.55, .55, 0), .55)
        self.assertGreater(projected_width(.55, .55, math.pi/4) + 2*.03, .8)

    def test_stopping_distance(self):
        self.assertAlmostEqual(required_clearance(.15, .3, .2, .1), .20125)
        with self.assertRaises(ValueError):
            required_clearance(.15, .3, 0, .1)

    def test_trial_limits(self):
        c = {'controller_server': {'ros__parameters': {'FollowPath': {}}}}
        p = apply_trial_profile(c)['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(p['max_vel_y'], 0)
        self.assertEqual(p['max_vel_x'], .15)


if __name__ == '__main__':
    unittest.main()
