import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from ev_session_guard import EvSessionGuard


class SessionTests(unittest.TestCase):
    def healthy(self, gate, count=30):
        for i in range(count):
            now = i * 0.033
            stamp = int((now + 1) * 1e9)
            gate.tracking(1, stamp, now)
            accepted = gate.pose(stamp, 20, now)
        return accepted

    def test_warmup(self):
        self.assertFalse(self.healthy(EvSessionGuard(0), 29))
        self.assertTrue(self.healthy(EvSessionGuard(0)))

    def test_no_automatic_recovery(self):
        gate = EvSessionGuard(0)
        self.healthy(gate)
        gate.tracking(2, 2_000_000_000, 1)
        self.assertFalse(gate.tracking(1, 2_030_000_000, 1.03))
        self.assertFalse(gate.pose(2_030_000_000, 10, 1.03))

    def test_stale_without_callbacks(self):
        gate = EvSessionGuard(0)
        self.healthy(gate)
        self.assertFalse(gate.watchdog(1.3))
        self.assertFalse(gate.tracking(1, 3_000_000_000, 1.31))

    def test_source_continuous_pose_survives_host_gap(self):
        gate = EvSessionGuard(0)
        self.assertTrue(self.healthy(gate))
        last = gate.pose_stamp
        now = gate.pose_at + 0.35
        stamp = last + 40_000_000
        gate.tracking(1, stamp, now)
        self.assertTrue(gate.pose(stamp, 20, now))
        self.assertIsNone(gate.fault)

    def test_source_gap_still_requires_new_session(self):
        gate = EvSessionGuard(0)
        self.healthy(gate)
        last = gate.pose_stamp
        stamp = last + 350_000_000
        gate.tracking(1, stamp, gate.pose_at + 0.05)
        self.assertFalse(gate.pose(stamp, 20, gate.pose_at + 0.05))
        self.assertEqual(gate.fault, 'VIO stale; new session required')

    def test_reset_and_invalid_samples(self):
        for stamp, age, frame in [(1, 20, True), (3_000_000_000, 500, True),
                                  (3_000_000_000, 20, False), (3_000_000_000, float('nan'), True)]:
            gate = EvSessionGuard(0)
            self.healthy(gate)
            self.assertFalse(gate.pose(stamp, age, 0.99, frame))
            self.assertIsNotNone(gate.fault)

    def test_startup_timeout(self):
        gate = EvSessionGuard(0)
        self.assertFalse(gate.watchdog(11))


if __name__ == '__main__':
    unittest.main()
