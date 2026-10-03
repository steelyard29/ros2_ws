from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from camera_stream_gate import StreamWindow, parameter_result_ok


class GateTests(unittest.TestCase):
    def feed(self, gate):
        for i in range(42):
            gate.observe(1_000_000_000+i*50_000_000, i*.05, 0.)

    def test_complete_stable_window(self):
        gate = StreamWindow(15, 1., .2)
        self.feed(gate)
        self.assertTrue(gate.ready(gate.last))
        self.assertFalse(gate.ready(3.))

    def test_frame_count_alone_not_enough(self):
        gate = StreamWindow(15, 1., .2)
        for i in range(20):
            gate.observe(1_000_000_000+i*1_000_000, i*.001, 0.)
        self.assertFalse(gate.ready(.019))

    def test_long_gap_restarts_window(self):
        gate = StreamWindow(15, 1., .2)
        self.feed(gate)
        gate.observe(4_000_000_000, 3., 0.)
        self.assertEqual(gate.count, 1)
        self.assertFalse(gate.ready(3.))

    def test_bad_or_duplicate_sample_breaks_window(self):
        for stamp, now, age, finite in ((3_050_000_000, 2.1, 0, True),
                (4_000_000_000, 2.1, 1., True), (4_000_000_000, 2.1, 0, False),
                (4_000_000_000, float('nan'), 0, True)):
            gate = StreamWindow(15, 1., .2)
            self.feed(gate)
            self.assertFalse(gate.observe(stamp, now, age, finite))
            self.assertFalse(gate.ready(2.1))

    def test_empty_or_partial_parameter_response_fails(self):
        ok = SimpleNamespace(successful=True)
        for result in (None, SimpleNamespace(results=[]), SimpleNamespace(results=[ok]),
                       SimpleNamespace(results=[ok, SimpleNamespace(successful=False)])):
            self.assertFalse(parameter_result_ok(result, 2))
        self.assertTrue(parameter_result_ok(SimpleNamespace(results=[ok, ok]), 2))

    def test_backlog_burst_does_not_meet_receipt_span(self):
        gate = StreamWindow(15, 1., .2)
        for i in range(42):
            gate.observe(1_000_000_000+i*50_000_000, i*.001, 0.)
        self.assertFalse(gate.ready(.041))

    def test_imu_requires_two_seconds_not_just_200_frames(self):
        gate = StreamWindow(200, 2., .1)
        for i in range(201):
            gate.observe(1_000_000_000+i*5_000_000, i*.005, 0.)
        self.assertFalse(gate.ready(gate.last))
        for i in range(201, 402):
            gate.observe(1_000_000_000+i*5_000_000, i*.005, 0.)
        self.assertTrue(gate.ready(gate.last))

    def test_invalid_limits_rejected(self):
        for values in ((1, 1., .1), (15, float('nan'), .1), (15, 1., 0.)):
            with self.assertRaises(ValueError):
                StreamWindow(*values)

    def test_source_future_and_clock_reversal_rejected(self):
        gate = StreamWindow(15, 1., .2)
        self.feed(gate)
        self.assertFalse(gate.observe(3_100_000_000, 1., 0.))
        self.assertFalse(gate.observe(3_100_000_000, 2.1, -.1))
