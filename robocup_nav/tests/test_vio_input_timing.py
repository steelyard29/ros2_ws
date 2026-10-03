from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from vio_input_timing import stereo_gap_evidence


class StereoTimingTests(unittest.TestCase):
    def test_input_frames_exist_inside_output_gap(self):
        p = [dict(kind=k, stamp_ns=t) for k in ('infra1', 'infra2')
             for t in (0, 50_000_000, 100_000_000, 150_000_000, 200_000_000)]
        p += [dict(kind='pose', stamp_ns=t) for t in (0, 200_000_000)]
        d = stereo_gap_evidence(p)
        self.assertEqual(d['pose_stamps_observed_in_both_inputs'], 2)
        self.assertEqual(d['last_gap_examples'][0]['exact_stereo_pairs_inside_gap'], 3)

    def test_missing_input_does_not_invent_camera_failure(self):
        d = stereo_gap_evidence([dict(kind='pose', stamp_ns=t) for t in (1, 300_000_002)])
        self.assertEqual(d['input_sample_counts'], dict(infra1=0, infra2=0))
        self.assertEqual(d['last_gap_examples'][0]['exact_stereo_pairs_inside_gap'], 0)
        self.assertNotIn('camera_failed', d)
        self.assertIn('cannot alone prove camera failure', d['note'])

    def test_fast_stream_and_reversed_stamp_not_a_positive_gap(self):
        p = [dict(kind='pose', stamp_ns=t) for t in (100_000_000, 200_000_000, 0)]
        self.assertEqual(stereo_gap_evidence(p)['pose_gaps_over_100ms'], 0)

    def test_exact_pair_requires_both_and_examples_bounded(self):
        p = [dict(kind='pose', stamp_ns=t*200_000_000) for t in range(40)]
        p += [dict(kind='infra1', stamp_ns=50_000_000), dict(kind='infra2', stamp_ns=50_000_001)]
        d = stereo_gap_evidence(p)
        self.assertEqual(d['pose_gaps_over_100ms'], 39)
        self.assertEqual(len(d['last_gap_examples']), 32)
        d = stereo_gap_evidence(p[:2]+p[-2:])
        self.assertEqual(d['last_gap_examples'][0]['exact_stereo_pairs_inside_gap'], 0)
