import unittest
from pathlib import Path
import yaml


class RulesTests(unittest.TestCase):
    def setUp(self):
        self.r=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/competition_rules.yaml').read_text())

    def test_confirmed_field_not_test_site(self):
        self.assertEqual(self.r['field']['size_m'],[10,8,4])
        self.assertEqual(self.r['physical_test_site']['size_m'],[8,5,3])
        self.assertEqual(self.r['obstacles']['height_range_m'],[1,3])
        self.assertEqual(self.r['corridor']['altitude_limit_m'],1.5)
        self.assertFalse(self.r['field']['registered_to_odom'])

    def test_original_delivery_preserved(self):
        self.assertEqual(self.r['delivery']['weights']['red_cross'],10)
        self.assertEqual(self.r['delivery']['red_cross_zone_size_m'],[.35,.35])
        self.assertFalse(self.r['delivery']['dynamic_tank_task_adopted'])
        self.assertIsNone(self.r['delivery']['tank_weight'])

    def test_marker_not_ring_diameter_or_flight_enable(self):
        self.assertEqual(self.r['landing']['marker_size_m'],[.8,.8])
        self.assertEqual(self.r['landing']['ring_inner_diameter_m'],.6)
        self.assertIsNone(self.r['landing']['ring_outer_diameter_m'])
        self.assertFalse(self.r['runtime_enabled'])

    def test_development_order(self):
        self.assertEqual(self.r['development_order'],
            ['obstacle_and_h_landing','gates','parcel_delivery','full_competition'])

    def test_test_height_not_scoring_or_site_approval(self):
        rule=self.r['takeoff_scoring']
        trial=rule['commissioning_profile']; candidate=rule['competition_candidate']
        self.assertAlmostEqual(trial['px4_center_agl_m']-trial['ground_px4_center_agl_m'],
            trial['relative_climb_m'])
        self.assertFalse(trial['qualifies_as_competition_takeoff_evidence'])
        self.assertGreater(candidate['px4_center_agl_m']-candidate['unloaded_lower_envelope_m'],
            rule['ground_height_must_exceed_m'])
        self.assertGreater(candidate['stable_duration_s'],rule['stable_duration_rule_s'])
        for key in ('runtime_enabled','site_clearance_verified','loaded_geometry_verified'):
            self.assertFalse(candidate[key])

    def test_no_fixed_target_positions_or_overflight_score(self):
        self.assertFalse(self.r['delivery']['target_positions_known_in_advance'])
        self.assertTrue(self.r['obstacles']['online_horizontal_avoidance_required'])
        self.assertTrue(self.r['obstacles']['collision_free_required_for_avoidance_score'])
        self.assertFalse(self.r['obstacles']['overflight_qualifies_for_avoidance_score'])
        self.assertEqual(self.r['operation']['max_sortie_s'],600)
        self.assertEqual(self.r['operation']['referee_takeoff_deadline_s'],30)
        self.assertFalse(self.r['operation']['resume_after_takeover_allowed'])

    def test_parcel_geometry_not_invented(self):
        self.assertEqual(self.r['delivery']['parcel_mass_kg'],.1)
        self.assertFalse(self.r['delivery']['actual_parcel_and_attachment_geometry_verified'])
        self.assertFalse(self.r['delivery']['first_hit_without_retention_weighted'])


if __name__=='__main__':unittest.main()
