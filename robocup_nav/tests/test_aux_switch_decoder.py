import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from aux_switch_decoder import AuxSwitchDecoder, validate_contract, px4_slot


def params():
    return {'RC_MAP_AUX1': 6, 'RC_MAP_AUX2': 5, 'RC_MAP_FLTMODE': 6,
            'RC_MAP_KILL_SW': 5, 'RC_MAP_OFFB_SW': 0, 'RC_MAP_FLTM_BTN': 0,
            'RC_KILLSWITCH_TH': .75, 'COM_RC_IN_MODE': 0,
            **{f'COM_FLTMODE{i}': 2 if i < 6 else 7 for i in range(1, 7)}}


def feed(d, us=1_000_000, **changes):
    fields = dict(stamp=us, sample=us-1_000, now=us/1e6, age=.01,
                  valid=True, source=1, aux1=-1., aux2=-1.)
    fields.update(changes)
    return d.receive(**fields)


class DecoderTests(unittest.TestCase):
    def test_missing_unassigned_or_mismatched_contract_rejected(self):
        for k in params():
            p = params()
            p.pop(k)
            with self.assertRaises(ValueError):
                validate_contract(p)
        p = params()
        p['RC_MAP_AUX1'] = 0
        with self.assertRaises(ValueError):
            AuxSwitchDecoder(p)

    def test_position_offboard_and_explicit_derived_origin(self):
        d = AuxSwitchDecoder(params())
        self.assertIsNone(feed(d))
        s = feed(d, 1_120_000)
        self.assertEqual((s.mode_slot, s.kill_switch), (1, 3))
        self.assertEqual(s.origin, 'derived_aux_mirror_not_native')
        self.assertIsNone(feed(d, 1_140_000, aux1=1.))
        s = feed(d, 1_260_000, aux1=1.)
        self.assertEqual((s.mode_slot, s.kill_switch), (6, 3))

    def test_kill_assertion_not_debounced_release_is(self):
        d = AuxSwitchDecoder(params())
        self.assertEqual(feed(d, aux2=1.).kill_switch, 1)
        self.assertIsNone(feed(d, 1_020_000))
        self.assertEqual(feed(d, 1_140_000).kill_switch, 3)

    def test_duplicate_cannot_complete_settle(self):
        d = AuxSwitchDecoder(params())
        feed(d)
        self.assertIsNone(feed(d, 1_120_000, sample=999_000))
        self.assertEqual(d.last_receipt, 1.)
        self.assertIsNone(d.last_good)

    def test_stale_reset_source_and_nan_latch(self):
        for changes in ({'source': 2}, {'valid': False}, {'aux1': float('nan')},
                        {'aux2': 1.1}, {'age': .6}, {'sample': 0}, {'sample': 800_000},
                        {'now': .1}):
            d = AuxSwitchDecoder(params())
            feed(d)
            feed(d, 1_120_000, **changes)
            self.assertIsNotNone(d.fault, changes)
            self.assertIsNone(feed(d, 1_240_000))

    def test_gap_requires_new_session(self):
        d = AuxSwitchDecoder(params())
        feed(d)
        self.assertIsNone(feed(d, 2_000_000))
        self.assertIn('gap', d.fault)

    def test_threshold_and_mode_boundary_fail_closed(self):
        mode_boundary = ((2-1/6-1)*2.1-1/6)/6-1.05
        for changes in ({'aux2': .5}, {'aux1': mode_boundary}):
            d = AuxSwitchDecoder(params())
            self.assertIsNone(feed(d, **changes))
            self.assertIn('ambiguous', d.fault)
        self.assertEqual(px4_slot(-1), 1)
        self.assertEqual(px4_slot(1), 6)
