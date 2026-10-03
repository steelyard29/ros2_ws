"""No ROS/hardware. Synthetic positive and rejection controls."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_ground_motion_alignment import analyze


def pairs_for(path_frd, yaws_deg, *, scale=1., flip_y=False, resets=(1, 1, 1, 1, 1)):
    rows = []
    for i, ((x, y), yaw) in enumerate(zip(path_frd, yaws_deg)):
        # FRD forward/left(-y) -> ROS FLU odom: x, -y, -z.
        vio_p = [x/scale, (y if flip_y else -y)/scale, 0.]
        q = Rotation.from_euler('z', math.radians(-yaw)).as_quat()
        rows.append(dict(vio_position=vio_p, vio_quaternion=list(q),
                         px4_position=[x, y, 0.], px4_heading=math.radians(yaw),
                         reset_counters=list(resets), difference_ms=5.))
    return rows


def ground_motion():
    path, yaws = [], []
    for s in np.linspace(0, .5, 40):
        path.append((s, 0.)); yaws.append(0.)
    for s in np.linspace(0, .5, 40):
        path.append((.5, -s)); yaws.append(0.)
    for a in np.linspace(0, 30, 30):
        path.append((.5, -.5)); yaws.append(a)
    return path, yaws


class Tests(unittest.TestCase):
    def test_consistent_ground_motion_passes(self):
        result = analyze(pairs_for(*ground_motion()))
        self.assertTrue(result['passed'], result['checks'])
        self.assertAlmostEqual(result['horizontal_scale_px4_over_vio'], 1., places=6)

    def test_scale_error_rejected(self):
        result = analyze(pairs_for(*ground_motion(), scale=1.3))
        self.assertFalse(result['checks']['scale_within_10pct'])

    def test_mirrored_left_rejected(self):
        path, yaws = ground_motion()
        mirrored = [(x, -y) for x, y in path]
        result = analyze(pairs_for(mirrored, yaws))
        self.assertFalse(result['checks']['left_is_minus_y'])

    def test_no_motion_rejected(self):
        result = analyze(pairs_for([(0., 0.)]*80, [0.]*80))
        self.assertFalse(result['passed'])

    def test_reset_change_rejected(self):
        rows = pairs_for(*ground_motion())
        rows[-1]['reset_counters'] = [9, 9, 9, 9, 9]
        with self.assertRaises(ValueError):
            analyze(rows)


if __name__ == '__main__':
    unittest.main()
