import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from flight_readiness import review


class ReadinessTests(unittest.TestCase):
    def test_missing_data_does_not_pass(self):
        result = review({}, {})
        self.assertFalse(result['configuration_consistent'])
        self.assertFalse(result['flight_approved'])

    def test_shared_channel_and_offboard_exception_rejected(self):
        result = review({'RC_MAP_FLTMODE': 5., 'RC_MAP_KILL_SW': 5.,
                         'COM_RCL_EXCEPT': 4., 'COM_RC_OVERRIDE': 1.}, {})
        checks = {c['check']: c['passed'] for c in result['checks']}
        self.assertFalse(checks['Dedicated mode and kill channels'])
        self.assertFalse(checks['RC loss not exempt in Offboard'])
        self.assertFalse(checks['Offboard stick override'])

    def test_strings_do_not_verify_geometry(self):
        result = review({}, {'geometry_verified': 'true', 'above_center_m': float('nan')})
        checks = {c['check']: c['passed'] for c in result['checks']}
        self.assertFalse(checks['geometry_verified'])
        self.assertFalse(checks['above_center_m'])

    def test_mode_channel_not_reused_for_dedicated_offboard(self):
        for offboard, expected in ((6., False), (0., True), (5., False)):
            result = review({'RC_MAP_FLTMODE': 6., 'RC_MAP_KILL_SW': 5.,
                             'RC_MAP_OFFB_SW': offboard}, {})
            checks = {c['check']: c['passed'] for c in result['checks']}
            self.assertEqual(checks['No duplicate Offboard switch mapping'], expected)

    def test_candidate_slots_and_land_speed_are_checked(self):
        result = review({'RC_MAP_FLTMODE': 6, 'RC_MAP_KILL_SW': 5, 'RC_MAP_OFFB_SW': 0,
                         'COM_RCL_EXCEPT': 0, 'COM_FLTMODE6': 7,
                         'MPC_LAND_SPEED': 0.600000024, 'COM_LOW_BAT_ACT': 0}, {})
        checks = {c['check']: c['passed'] for c in result['checks']}
        self.assertTrue(checks['COM_FLTMODE6'])
        self.assertTrue(checks['MPC_LAND_SPEED'])
        self.assertFalse(checks['COM_FLTMODE1'])  # Missing is not assumed Position.
        self.assertNotIn('COM_LOW_BAT_ACT', checks)
        result = review({'COM_FLTMODE6': 5., 'MPC_LAND_SPEED': .7}, {})
        checks = {c['check']: c['passed'] for c in result['checks']}
        self.assertFalse(checks['COM_FLTMODE6'])
        self.assertFalse(checks['MPC_LAND_SPEED'])


if __name__ == '__main__':
    unittest.main()
