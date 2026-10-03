from pathlib import Path
import sys
import unittest
import json
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from preflight_source_guard import SourceGuard, observer_passed, telemetry_observation


class ObserverGuardTests(unittest.TestCase):
    def test_fresh_source(self):
        g = SourceGuard()
        self.assertTrue(g.accept('rc', 1_000_000, 1., .01, 999_000))
        self.assertTrue(g.accept('rc', 1_020_000, 1.02, .01, 1_019_000))

    def test_republication_of_frozen_sample_cannot_renew(self):
        g = SourceGuard()
        g.accept('rc', 1_000_000, 1., 0, 999_000)
        self.assertFalse(g.accept('rc', 1_020_000, 1.02, 0, 999_000))
        self.assertEqual(g.receipts['rc'], 1.)
        self.assertEqual(g.duplicates, 1)

    def test_old_future_and_nonfinite_sources_latch(self):
        for age in (.6, -.1, float('nan')):
            g = SourceGuard()
            self.assertFalse(g.accept('status', 100, 1., age))
            self.assertFalse(g.accept('status', 200, 2., 0))

    def test_sample_age_and_order(self):
        for stamp, sample, age in ((1_000_000, 1_000_001, 0),
                                  (1_000_000, 600_000, .2), (1_000_000, 0, 0)):
            self.assertFalse(SourceGuard().accept('rc', stamp, 1., age, sample))

    def test_source_or_receipt_reset_latches(self):
        for stamp, now in ((99, 2.), (101, .9)):
            g = SourceGuard()
            g.accept('status', 100, 1., 0)
            self.assertFalse(g.accept('status', stamp, now, 0))

    def test_ev_may_lead_publish_time_by_the_existing_vio_allowance(self):
        g = SourceGuard()
        self.assertTrue(g.accept('ev', 1_000_000, 1., 0., 1_050_000, sample_lead_us=60_000))
        self.assertFalse(SourceGuard().accept('ev', 1_000_000, 1., 0., 1_080_000, sample_lead_us=60_000))
        self.assertFalse(SourceGuard().accept('rc', 1_000_000, 1., 0., 1_050_000))

    def test_sample_lead_cannot_be_widened_arbitrarily(self):
        self.assertFalse(SourceGuard().accept('ev', 1_000_000, 1., 0., 1_010_000, sample_lead_us=60_001))
        self.assertFalse(SourceGuard().accept('ev', 1_000_000, 1., 0., 1_010_000, sample_lead_us=-1))

    def test_prior_good_period_does_not_override_bad_final_state(self):
        self.assertTrue(observer_passed(10, True, False, None))
        self.assertFalse(observer_passed(10, False, False, None))
        self.assertFalse(observer_passed(10, True, True, None))
        self.assertFalse(observer_passed(10, True, False, 'reset'))
        self.assertFalse(observer_passed(4, True, False, None))

    def test_first_fault_clocks_are_preserved_after_raw_recovery(self):
        g = SourceGuard()
        self.assertTrue(g.accept('flags', 1_000_000, 1., .01, observed={'cs_ev_pos': True}))
        self.assertFalse(g.accept('flags', 2_000_000, 2., .51, observed={'cs_ev_pos': False}))
        first = dict(g.first_fault)
        self.assertEqual(first['source_age_s'], .51)
        self.assertEqual(first['previous_accepted_stamp_us'], 1_000_000)
        self.assertEqual(first['previous_accepted_receipt_monotonic_s'], 1.)
        self.assertEqual(first['source_age_limits_s'], [-.05, .5])
        self.assertFalse(g.accept('flags', 3_000_000, 3., .01, observed={'cs_ev_pos': True}))
        self.assertEqual(g.first_fault, first)
        self.assertEqual(g.receipts['flags'], 1.)
        d = g.diagnostics(4.)
        self.assertTrue(d['telemetry_frozen'])
        self.assertEqual(d['last_accepted_receipt_age_s']['flags'], 3.)
        raw = d['last_received_unvalidated']['flags']
        self.assertEqual(raw['observed'], {'cs_ev_pos': True})
        self.assertEqual(raw['decision'], 'rejected_latched')
        self.assertEqual(d['received_counts']['flags'], 3)
        self.assertFalse(observer_passed(10, True, False, g.fault))

    def test_other_topics_freeze_and_raw_receipts_continue(self):
        g = SourceGuard()
        g.accept('local', 100, 1., 0.)
        g.accept('flags', 200, 2., -.1)
        self.assertFalse(g.accept('local', 300, 3., 0.))
        self.assertEqual(g.stamps['local'], 100)
        self.assertEqual(g.last_received['local']['stamp_us'], 300)

    def test_duplicate_is_not_a_fault_or_a_new_receipt(self):
        g = SourceGuard()
        g.accept('flags', 100, 1., 0.)
        self.assertFalse(g.accept('flags', 100, 1.2, .2))
        d = g.diagnostics(1.3)
        self.assertFalse(d['telemetry_frozen'])
        self.assertIsNone(d['first_fault'])
        self.assertEqual(d['last_received_unvalidated']['flags']['decision'], 'duplicate_not_renewed')
        self.assertEqual(d['last_accepted_receipt_monotonic_s']['flags'], 1.)

    def test_fault_diagnostics_are_strict_json_and_snapshot_not_alias(self):
        g = SourceGuard()
        g.accept('flags', 100, 1., float('nan'))
        d = g.diagnostics(2.)
        json.dumps(d, allow_nan=False)
        self.assertEqual(d['first_fault']['source_age_s'], 'nan')
        d['first_fault']['topic'] = 'changed'
        self.assertEqual(g.first_fault['topic'], 'flags')

    def test_sample_age_and_external_fault_details(self):
        g = SourceGuard()
        g.accept('ev', 1_000_000, 1., .2, 600_000)
        self.assertAlmostEqual(g.first_fault['sample_age_s'], .6)
        self.assertEqual(g.first_fault['reason'], 'stale sample: ev')
        g = SourceGuard()
        g.accept('status', 100, 1., 0., observed={'arming_state': 2})
        g.latch_fault('status is not explicitly disarmed', 'status')
        self.assertEqual(g.first_fault['observed']['arming_state'], 2)
        self.assertTrue(g.diagnostics(2.)['telemetry_frozen'])

    def test_raw_state_whitelist_and_no_gate_effect(self):
        m = SimpleNamespace(cs_ev_pos=True, cs_ev_yaw=False, unrelated='not retained')
        self.assertEqual(telemetry_observation('flags', m), {'cs_ev_pos': True, 'cs_ev_yaw': False})
        self.assertEqual(telemetry_observation('local', SimpleNamespace(x=float('inf'))), {'x': 'inf'})
        g = SourceGuard()
        self.assertFalse(g.accept('flags', 100, 1., .6, observed=telemetry_observation('flags', m)))
        self.assertEqual(g.stamps, {})
