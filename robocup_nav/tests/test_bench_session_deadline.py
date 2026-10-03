from pathlib import Path
import math
import signal
import sys
import time
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from bench_session_deadline import SessionBudget, DeadlineAlarm


class BudgetTests(unittest.TestCase):
    def test_legacy_duration_unchanged(self):
        b = SessionBudget(None, lambda: 100.)
        self.assertEqual(b.stage_end(45), 145)
        self.assertEqual(b.cleanup_timeout(20), 20)

    def test_shared_deadline_reserves_final_and_cleanup(self):
        now = [100.]
        b = SessionBudget(155., lambda: now[0])
        self.assertEqual(b.active_end, 138)
        self.assertEqual(b.stage_end(90), 135)
        self.assertEqual(b.stage_end(90, final=True), 138)
        now[0] = 136
        with self.assertRaises(TimeoutError):
            b.require_active()
        now[0] = 152
        self.assertEqual(b.cleanup_timeout(20), 2)
        now[0] = 160
        self.assertEqual(b.cleanup_timeout(20), 0)

    def test_invalid_budget_rejected(self):
        for value in (math.nan, math.inf, 100, 124, 161):
            with self.assertRaises(ValueError):
                SessionBudget(value, lambda: 100.)

    def test_alarm_interrupts_and_restores_without_hardware(self):
        previous = signal.getsignal(signal.SIGALRM)
        alarm = DeadlineAlarm(time.monotonic()+.04)
        try:
            alarm.arm()
            with self.assertRaises(TimeoutError):
                time.sleep(.15)
        finally:
            alarm.cancel()
        self.assertEqual(signal.getsignal(signal.SIGALRM), previous)
        self.assertEqual(signal.getitimer(signal.ITIMER_REAL)[0], 0)

    def test_expired_alarm_does_not_start(self):
        with self.assertRaises(TimeoutError):
            DeadlineAlarm(time.monotonic()-1).arm()


if __name__ == '__main__':
    unittest.main()
