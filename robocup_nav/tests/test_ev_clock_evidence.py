import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from ev_clock_evidence import snapshot


class ClockTests(unittest.TestCase):
    def test_records_disagreement_without_correction(self):
        ticks=iter([100,120])
        s=snapshot(lambda:2000,wall=lambda:4000,monotonic=lambda:next(ticks))
        self.assertEqual(s,dict(ros_ns=2000,system_ns=4000,
            monotonic_before_ns=100,monotonic_after_ns=120,read_span_ns=20))
    def test_zero_ros_is_preserved_not_replaced(self):
        s=snapshot(lambda:0,wall=lambda:100,monotonic=lambda:20)
        self.assertEqual(s['ros_ns'],0)
        self.assertEqual(s['read_span_ns'],0)


if __name__=='__main__':unittest.main()
