"""Analytic positive and negative controls; no ROS, devices or network."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from disarmed_ev_fusion_evidence import summarize


def sample(t,**kw):
    row=dict(at=float(t),stamp_us=int((100+t)*1e6),source_age_s=.01,
             ev_position=True,ev_yaw=True,ev_height=True)
    row.update(kw);return row


class FusionEvidenceTests(unittest.TestCase):
    def test_regular_true_samples_measure_endpoints_only(self):
        r=summarize([sample(t) for t in range(10,31)])
        self.assertEqual(r['longest_sampled_span_s'],20.)
        self.assertEqual(r['fully_fused_samples'],21)
        self.assertEqual(r['runs'][0]['end_at'],30.)
        self.assertFalse(r['flight_ready']);self.assertEqual(r['issues'],[])

    def test_single_true_sample_does_not_establish_duration(self):
        r=summarize([sample(10)])
        self.assertEqual(r['longest_sampled_span_s'],0.)

    def test_legacy_missing_evidence_is_unknown_not_pass(self):
        r=summarize(None)
        self.assertFalse(r['available']);self.assertFalse(r['flight_ready'])

    def test_any_required_flag_false_breaks_span(self):
        for key in ('ev_position','ev_yaw','ev_height'):
            with self.subTest(key=key):
                r=summarize([sample(1),sample(2),sample(3,**{key:False}),sample(4),sample(5)])
                self.assertEqual(r['longest_sampled_span_s'],1.)
                self.assertEqual(len(r['runs']),2)

    def test_true_after_long_gap_does_not_bridge_missing_data(self):
        r=summarize([sample(1),sample(2),sample(6),sample(7)])
        self.assertEqual(r['longest_sampled_span_s'],1.)
        self.assertEqual(r['issues'][0]['reason'],'receipt gap')

    def test_invalid_duplicate_or_reversed_input_breaks_span(self):
        for row in (sample(2),sample(1.5),sample(3,stamp_us=102000000),
                    sample(3,at=float('nan')),sample(3,source_age_s=.51),
                    sample(3,source_age_s=-.051),sample(3,ev_yaw=1),{}):
            with self.subTest(row=row):
                r=summarize([sample(1),sample(2),row,sample(4)])
                self.assertEqual(r['longest_sampled_span_s'],1.)
                self.assertTrue(r['issues'])

    def test_receipt_limit_boundary_and_truncation_disclosed(self):
        r=summarize([sample(1),sample(3)],truncated=True)
        self.assertEqual(r['longest_sampled_span_s'],2.)
        self.assertTrue(r['truncated'])

    def test_old_real_transition_log_cannot_substitute_for_samples(self):
        # Same shape as the previous real failed EV test; no invented sample stamps.
        r=summarize([dict(at=11012.022419854,ev_position=True,ev_yaw=True,ev_height=True)])
        self.assertEqual(r['fully_fused_samples'],0);self.assertTrue(r['issues'])


if __name__=='__main__':unittest.main()
