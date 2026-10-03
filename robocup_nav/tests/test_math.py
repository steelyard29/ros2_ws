import math
import unittest
import numpy as np
from nav_math import rotation, preview_ned, depth_points


class GeometryTest(unittest.TestCase):
    def test_body_to_world_to_ned(self):
        v, rate, z = preview_ned(0.2, 0., 0.1, math.pi/2, 1.2)
        np.testing.assert_allclose(v, [0.2, 0., 0.], atol=1e-8)
        self.assertEqual(rate, -0.1)
        self.assertEqual(z, -1.2)
        np.testing.assert_allclose(preview_ned(0.2, 0., 0., 0., 1.2)[0], [0., 0.2, 0.])

    def test_reject_nonfinite(self):
        with self.assertRaises(ValueError):
            preview_ned(float('nan'), 0., 0., 0., 1.2)
        with self.assertRaises(ValueError):
            rotation([0., 0., 0., 0.])

    def test_missing_depth_not_free_space(self):
        points = depth_points(np.array([[0., 1.], [np.nan, 9.]]), 1., 1., 0., 0., stride=1)
        np.testing.assert_allclose(points, [[1., 0., 1.]])

    def test_full_attitude_rotation(self):
        r = rotation([math.sin(math.pi/4), 0., 0., math.cos(math.pi/4)])
        np.testing.assert_allclose(r @ [0., 1., 0.], [0., 0., 1.], atol=1e-8)


if __name__ == '__main__':
    unittest.main()
