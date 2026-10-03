import sys
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from alignment_math import LocalFRD, rpy_degrees


class AlignmentTests(unittest.TestCase):
    def test_forward_left_up(self):
        frame = LocalFRD([4, 5, 6], [0, 0, 0, 1])
        p, q = frame.convert([5, 6, 7], [0, 0, 0, 1])
        np.testing.assert_allclose(p, [1, -1, -1])
        np.testing.assert_allclose(q, [1, 0, 0, 0])

    def test_fixed_frame_with_initial_heading(self):
        initial = Rotation.from_euler('z', 60, degrees=True)
        frame = LocalFRD([0, 0, 0], initial.as_quat())
        moved = initial.apply([1, 0, 0])
        q = Rotation.from_euler('z', 150, degrees=True).as_quat()
        p, out = frame.convert(moved, q)
        np.testing.assert_allclose(p, [1, 0, 0], atol=1e-10)
        self.assertAlmostEqual(rpy_degrees(out)[2], -90)

    def test_tilt_preserved_and_pitch_sign(self):
        q = Rotation.from_euler('xyz', [10, 5, 0], degrees=True).as_quat()
        frame = LocalFRD([0, 0, 0], q)
        _, out = frame.convert([0, 0, 0], q)
        np.testing.assert_allclose(rpy_degrees(out), [10, -5, 0], atol=1e-10)

    def test_invalid(self):
        for p, q in [([float('nan'), 0, 0], [0, 0, 0, 1]), ([0, 0, 0], [0, 0, 0, 0])]:
            with self.assertRaises(ValueError):
                LocalFRD(p, q)


if __name__ == '__main__':
    unittest.main()
