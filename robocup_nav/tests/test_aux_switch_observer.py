import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from aux_switch_observer import AuxEvidence


class AuxEvidenceTests(unittest.TestCase):
    def test_rc_only_and_no_inferred_mapping(self):
        e = AuxEvidence()
        self.assertTrue(e.receive(1_000_000, 980_000, 0., 1., True, 1, 0., 0.))
        self.assertNotIn('kill_switch', e.latest)
        self.assertFalse(e.receive(1_100_000, 1_080_000, 0., 1.1, True, 2, -1., -1.))
        self.assertFalse(e.receive(1_200_000, 1_180_000, 0., 1.2, False, 1, -1., -1.))

    def test_frozen_sample_does_not_renew(self):
        e = AuxEvidence()
        self.assertTrue(e.receive(1_000_000, 980_000, 0., 1., True, 1, -1., -1.))
        self.assertFalse(e.receive(1_100_000, 980_000, 0., 1.1, True, 1, -1., -1.))
        self.assertEqual(e.counts['fresh_rc_samples'], 1)

    def test_stale_future_and_invalid_aux_rejected(self):
        for sample, age, aux in ((0, 0., -1.), (1_000_001, 0., -1.),
                                 (500_000, .1, -1.), (980_000, -.1, -1.),
                                 (980_000, 0., float('nan')), (980_000, 0., 1.1)):
            e = AuxEvidence()
            self.assertFalse(e.receive(1_000_000, sample, age, 1., True, 1, aux, -1.))

    def test_source_reset_latches(self):
        e = AuxEvidence()
        e.receive(1_000_000, 980_000, 0., 1., True, 1, -1., -1.)
        self.assertFalse(e.receive(900_000, 880_000, 0., 2., True, 1, -1., -1.))
        self.assertEqual(e.fault, 'source clock reset')
