import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from reader_delay_analysis import delay_evidence,required_streams_present


def rows():
    return [dict(kind='pose',seq=i,reader_receipt_monotonic_s=t,stamp_ns=stamp,
                 reader_clock=dict(ros_ns=clock)) for i,t,stamp,clock in
            [(1,1.,1_000_000_000,1_020_000_000),
             (2,1.36,1_033_000_000,1_393_000_000)]]


class DelayTests(unittest.TestCase):
    def test_slow_graph_overlap_preserved_without_causal_claim(self):
        slow=dict(stage='graph_pose',start=1.01,end=1.35,elapsed_s=.34)
        r=delay_evidence(rows(),dict(slow_calls=[slow]))
        self.assertEqual(r['events'][0]['overlapping_measured_calls'],[slow])
        self.assertAlmostEqual(r['events'][0]['reader_source_age_s'],.36)
        self.assertIn('not proof',r['scope'])
    def test_no_timing_is_unknown_not_normal(self):
        r=delay_evidence(rows(),None)
        self.assertFalse(r['timing_available'])
        self.assertEqual(len(r['events']),1)
    def test_nonoverlapping_event_not_attributed(self):
        r=delay_evidence(rows(),dict(slow_calls=[dict(stage='audit',start=2.,end=3.)]))
        self.assertEqual(r['events'][0]['overlapping_measured_calls'],[])
    def test_no_packets_cannot_pass_observation(self):
        self.assertFalse(required_streams_present({}))
        self.assertFalse(required_streams_present({'pose':{'count':1}}))
        both={'pose':{'count':1},'tracking':{'count':1}}
        self.assertTrue(required_streams_present(both))
        self.assertFalse(required_streams_present(both,True))
    def test_fresh_continuous_input_has_no_delay_event(self):
        p=rows();p[1]['reader_receipt_monotonic_s']=1.033
        p[1]['reader_clock']['ros_ns']=1_050_000_000
        self.assertEqual(delay_evidence(p,dict(slow_calls=[]))['events'],[])


if __name__=='__main__':unittest.main()
