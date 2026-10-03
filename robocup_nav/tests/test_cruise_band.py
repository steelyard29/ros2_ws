import unittest
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from cruise_band import make_band


class BandTests(unittest.TestCase):
    def test_ground_inhibited_cruise_allowed(self):
        b=make_band()
        self.assertFalse(b.contains_vehicle(0,0))
        self.assertTrue(b.contains_vehicle(.6,0))
        self.assertFalse(b.contains_vehicle(.4,0))
        self.assertGreater(b.lower,b.floor+.05)
        self.assertLess(b.lower,.6-.15)
        self.assertGreater(b.upper,.6+.115)

    def test_offset_and_loaded_body(self):
        b=make_band(initial_z=2)
        self.assertAlmostEqual(b.center,2.6)
        with self.assertRaises(ValueError):make_band(below=.9)
        with self.assertRaises(ValueError):make_band(rise=float('nan'))


if __name__=='__main__':unittest.main()
