import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from task_joint_timing_check import compare,shared_gaps


class TimingTests(unittest.TestCase):
    def test_only_exact_source_stamp_matches(self):
        r=compare([dict(stamp_us=100,at=2.,age_s=.6),dict(stamp_us=200,at=3.,age_s=.1)],
                  [dict(stamp_us=100,at=1.5,age_s=.1),dict(stamp_us=201,at=3.,age_s=.1)])
        self.assertEqual(len(r),1)
        self.assertAlmostEqual(r[0]['task_minus_independent_receipt_s'],.5)
        self.assertEqual(r[0]['task_age_s'],.6)
        self.assertEqual(compare([],[]),[])

    def test_common_gap_requires_same_two_source_endpoints(self):
        a=[dict(stamp_us=1000000,at=1.),dict(stamp_us=3010000,at=3.01)]
        b=[dict(stamp_us=1000000,at=1.001),dict(stamp_us=3010000,at=3.012)]
        self.assertEqual(len(shared_gaps(a,b)),1)
        self.assertAlmostEqual(shared_gaps(a,b)[0]['source_gap_s'],2.01)
        self.assertEqual(shared_gaps(a,[dict(stamp_us=1000001,at=1.),b[1]]),[])
        self.assertEqual(shared_gaps(a,[a[0],dict(stamp_us=2000000,at=2.),a[1]]),[])
