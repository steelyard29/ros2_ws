import json
import unittest
from pathlib import Path
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]


class CameraConfigTests(unittest.TestCase):
    def test_matches_operator_confirmed_original(self):
        candidate=yaml.safe_load((ROOT/'config/landing_candidate.yaml').read_text())
        original=json.loads(Path(candidate['candidate_calibration_file']).read_text())
        ros=yaml.safe_load((ROOT/'config/downward_camera_info.yaml').read_text())
        self.assertEqual(candidate['current_preview_size'],[ros['image_width'],ros['image_height']])
        self.assertEqual(candidate['current_preview_size'],[original['image_width'],original['image_height']])
        np.testing.assert_array_equal(np.array(ros['camera_matrix']['data']).reshape(3,3),original['camera_matrix'])
        np.testing.assert_array_equal(ros['distortion_coefficients']['data'],original['distortion_coefficients'])
        self.assertTrue(candidate['calibration_use_authorized'])
        self.assertFalse(candidate['descent_enabled'])
        if candidate['runtime_calibration_load_verified']:
            report=json.loads((ROOT/candidate['runtime_calibration_evidence']).read_text())
            self.assertTrue(report['camera_info_matches_selected_calibration'])
            self.assertEqual(report['size'],candidate['current_preview_size'])


if __name__=='__main__':unittest.main()
