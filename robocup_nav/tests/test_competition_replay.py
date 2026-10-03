import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from competition_replay import demo_rows, replay


class ReplayTests(unittest.TestCase):
    def test_six_synthetic_scenarios(self):
        expected = {'nominal': 'DONE', 'vision-loss': 'STOPPED',
                    'link-loss': 'STOPPED', 'manual-takeover': 'HANDOVER',
                    'release-timeout': 'DONE', 'liftoff-timeout': 'STOPPED'}
        for scenario, state in expected.items():
            with self.subTest(scenario=scenario):
                result = replay(demo_rows(scenario))
                self.assertEqual(result['final_state'], state)
                self.assertEqual(result['real_commands_sent'], 0)
                self.assertFalse(result['flight_validation'])
                if scenario == 'nominal':
                    self.assertEqual(len(result['confirmed_releases']), 3)
                elif scenario == 'release-timeout':
                    self.assertEqual(result['uncertain_slots'], [0])
                    self.assertEqual(result['confirmed_releases'], [])

    def test_string_booleans_rejected(self):
        row = next(demo_rows('nominal'))
        row['evidence']['preflight_ok'] = 'false'
        with self.assertRaises(ValueError):
            replay([row])

    def test_truncated_replay_not_complete(self):
        result = replay([next(demo_rows('nominal'))])
        self.assertFalse(result['complete'])


if __name__ == '__main__':
    unittest.main()
