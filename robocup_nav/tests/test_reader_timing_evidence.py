import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from reader_timing_evidence import ReaderTiming


class TimingTests(unittest.TestCase):
    def test_records_long_audit_and_preserves_result(self):
        stamps=iter([1.,1.36])
        t=ReaderTiming(clock=lambda:next(stamps))
        self.assertEqual(t.call('audit',lambda x:x+1,4),5)
        self.assertAlmostEqual(t.report['stages']['audit']['max_s'],.36)
        self.assertEqual(t.report['slow_calls'][0]['stage'],'audit')
    def test_exception_propagates_with_evidence(self):
        stamps=iter([1.,1.4]);t=ReaderTiming(clock=lambda:next(stamps))
        def fail():raise ValueError('read failure')
        with self.assertRaisesRegex(ValueError,'read failure'):t.call('stop_file',fail)
        self.assertEqual(t.report['slow_calls_total'],1)
    def test_events_bounded_and_fast_calls_not_logged(self):
        stamps=iter([0.,.001,1.,1.1,2.,2.2,3.,3.3])
        t=ReaderTiming(clock=lambda:next(stamps),capacity=2)
        for _ in range(4):t.call('spin_once',lambda:None)
        self.assertEqual(t.report['stages']['spin_once']['count'],4)
        self.assertEqual(t.report['slow_calls_total'],3)
        self.assertEqual(len(t.report['slow_calls']),2)
        self.assertEqual(t.report['slow_calls'][0]['start'],2.)


if __name__=='__main__':unittest.main()
