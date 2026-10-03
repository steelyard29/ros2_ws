import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from flight_runtime_core import FlightRuntimeCore


class RejectionTests(unittest.TestCase):
    def test_rejected_age_preserved_without_freshness_or_threshold_change(self):
        c=FlightRuntimeCore(0.)
        self.assertTrue(c.receive('estimator_status_flags',1000000,1.,.01))
        self.assertFalse(c.receive('estimator_status_flags',2000000,2.6,.6))
        r=c.report()['source_rejection']
        self.assertEqual(r['source_age_s'],.6)
        self.assertEqual(r['previous_accepted_stamp_us'],1000000)
        self.assertEqual(c.source_stamps['estimator_status_flags'],1000000)
        self.assertFalse(c.receive('vehicle_status_v1',1,3.,-.1))
        self.assertEqual(c.report()['source_rejection'],r)
