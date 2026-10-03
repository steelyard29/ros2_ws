import sys
import unittest
from pathlib import Path
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from flight_switch_contract import add_switch_publication, canonical_message


class SwitchContractTests(unittest.TestCase):
    def test_only_adds_readonly_publication(self):
        original = ('publications:\n  - topic: /fmu/out/manual_control_setpoint\n'
                    '    type: px4_msgs::msg::ManualControlSetpoint\n'
                    'subscriptions: []\n')
        before, after = yaml.safe_load(original), yaml.safe_load(add_switch_publication(original))
        self.assertEqual(before['subscriptions'], after['subscriptions'])
        self.assertEqual(after['publications'][:-1], before['publications'])
        self.assertEqual(after['publications'][-1]['topic'], '/fmu/out/manual_control_switches')
        with self.assertRaises(ValueError):
            add_switch_publication(add_switch_publication(original))

    def test_unknown_layout_rejected(self):
        with self.assertRaises(ValueError):
            add_switch_publication('publications: []')

    def test_message_comparison_preserves_fields_and_constants(self):
        self.assertEqual(canonical_message('uint64 timestamp # clock\n\n'), ['uint64 timestamp'])
        self.assertNotEqual(canonical_message('uint8 ON=1'), canonical_message('uint8 ON=3'))
