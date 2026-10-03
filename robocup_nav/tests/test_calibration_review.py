from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from calibration_review import compare


class CalibrationReviewTests(unittest.TestCase):
    def test_unchanged_is_not_calibration_certificate(self):
        r = compare({'CAL_ACC0_XOFF': .1}, {'CAL_ACC0_XOFF': .1})
        self.assertEqual(r['changes'], [])
        self.assertFalse(r['calibration_performed_verified'])
        self.assertFalse(r['flight_approved'])

    def test_calibration_changes_are_only_candidates(self):
        r = compare({'CAL_ACC0_XOFF': .1, 'SENS_BOARD_X_OFF': 1.},
                    {'CAL_ACC0_XOFF': .2, 'SENS_BOARD_X_OFF': 2.})
        self.assertEqual(len(r['changes']), 2)
        self.assertEqual(r['other_changes_requiring_review'], [])
        self.assertFalse(r['calibration_performed_verified'])

    def test_safety_and_orientation_changes_require_review(self):
        for name in ('COM_LOW_BAT_ACT', 'RC_MAP_KILL_SW', 'EKF2_EV_CTRL', 'SENS_BOARD_ROT', 'CAL_ACC0_ID'):
            self.assertEqual(len(compare({name: 0}, {name: 1})['other_changes_requiring_review']), 1)

    def test_added_or_removed_calibration_fields_require_review(self):
        self.assertTrue(compare({}, {'CAL_ACC0_XOFF': .1})['other_changes_requiring_review'])
        self.assertTrue(compare({'CAL_ACC0_XOFF': .1}, {})['other_changes_requiring_review'])
