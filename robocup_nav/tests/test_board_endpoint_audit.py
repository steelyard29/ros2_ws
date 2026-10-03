"""Offline diagnostic controls; these do not certify camera geometry."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from board_endpoint_audit import consistent_estimate


def match(shift, score=.97, alternative=.7):
    return dict(pixel_translation=shift, score=score, alternative_score=alternative,
                nominal_forward_left_m=[.036, -.011])


class EndpointAuditTests(unittest.TestCase):
    def test_consistent_distinctive_matches_positive_control(self):
        np.testing.assert_allclose(consistent_estimate([
            match([110, -375]), match([112, -376])]), [.036, -.011])

    def test_wrong_repeated_label_never_averaged(self):
        self.assertIsNone(consistent_estimate([
            match([107, -113]), match([114, -376])]))

    def test_ambiguous_weak_or_single_match_not_consensus(self):
        for matches in ([match([110, -375])],
                        [match([110, -375], score=.8), match([112, -376])],
                        [match([110, -375], alternative=.92), match([112, -376])]):
            with self.subTest(matches=matches):
                self.assertIsNone(consistent_estimate(matches))


if __name__ == '__main__':
    unittest.main()
