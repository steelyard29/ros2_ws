import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_limits import horizontal_velocity


class LimitsTests(unittest.TestCase):
    def test_roundoff_and_rotated_endpoint(self):
        for angle in range(360):
            a=math.radians(angle)
            result=horizontal_velocity((.15000000000000002*math.cos(a),
                                        .15000000000000002*math.sin(a)))
            self.assertLessEqual(math.hypot(*result),.15)

    def test_real_overlimit_and_invalid_rejected(self):
        for value in ((.150001,0),(-.150001,0),(float('nan'),0),(0,float('inf')),(),(0,)):
            with self.assertRaises(ValueError):horizontal_velocity(value)

    def test_in_limit_unchanged(self):
        for value in ((0,0),(.1,-.02),(-.15,0)):
            self.assertEqual(horizontal_velocity(value),value)
