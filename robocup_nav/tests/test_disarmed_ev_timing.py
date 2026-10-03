"""Deterministic mixed-rate replay; no ROS, devices, wall-clock sleeps or PX4."""
import unittest
from test_disarmed_ev_session import Rig
from disarmed_ev_contract import RECEIPT_LIMITS


class MixedRig(Rig):
    # 10 ms clock grid; different phases and bounded alternating jitter.
    periods = dict(status=51, flags=101, land=100, rc=3)
    def __init__(self):
        super().__init__()
        self.tick = 0
        self.due = dict(status=1, flags=7, land=13, rc=2)
        self.counts = dict.fromkeys(self.periods, 0)

    def advance(self, stopped=(), duplicates=()):
        self.tick += 1
        self.now = self.tick / 100
        n, s, c = self.now, self.s, self.s.core
        c.ownership(n, True)
        def rc():
            c.rc(n, True)
            c.switches(n, 1, 3)
        callbacks = dict(
            status=lambda: c.status(1, n, nav_state=2, preflight_ok=True, failsafe=False),
            flags=lambda: c.flags(n, ev_position=True, ev_yaw=True, ev_height=True,
                                 ev_velocity=False, baro_height=True, range_height=False),
            land=lambda: c.land(n, True), rc=rc)
        for name, period in self.periods.items():
            if self.tick < self.due[name]:
                continue
            self.counts[name] += 1
            self.due[name] = self.tick + period + self.counts[name] % 2
            if name in stopped:
                continue
            stamp = (c.source_stamps[name] if name in duplicates
                     else int((100+n)*1e6))
            # Even a duplicate claiming zero transport age must not renew receipt.
            s.telemetry(name, stamp, 0, n, callbacks[name])
        if self.tick % 5 == 0:
            ns = int((100+n)*1e9)
            s.packet(self.packet('tracking', state=1), n, ns)
            ev = s.packet(self.packet('pose', frame='odom', child='base_link',
                          position=[0, 0, 0], quaternion=[0, 0, 0, 1]), n, ns)
            if ev is not None:
                s.dispatched()
                self.outputs.append(ev)
        s.watchdog(n)

    def run_to(self, end, **kwargs):
        while self.now < end:
            self.advance(**kwargs)


class TimingTests(unittest.TestCase):
    def test_mixed_rates_jitter_and_phases_sustain_output(self):
        r = MixedRig(); r.run_to(15)
        self.assertIsNone(r.s.fault)
        self.assertGreater(len(r.outputs), 200)
        for name, row in r.s.timing_snapshot(r.now).items():
            self.assertLessEqual(row['max_gap_s'], RECEIPT_LIMITS[name])
        self.assertGreater(r.s.receipt_stats['flags']['max_gap_s'], 1.)
        self.assertGreater(r.s.receipt_stats['status']['max_gap_s'], .5)

    def test_each_true_stop_and_duplicate_freeze_latches(self):
        for name, limit in RECEIPT_LIMITS.items():
            for mode in ('stopped', 'duplicates'):
                with self.subTest(topic=name, mode=mode):
                    r = MixedRig(); r.run_to(4)
                    self.assertTrue(r.outputs)
                    last = r.s.receipts[name]
                    while r.s.fault is None and r.now < last+limit+.1:
                        r.advance(**{mode: (name,)})
                    self.assertEqual(r.s.fault, 'safety telemetry receipt stale: '+name)
                    self.assertGreater(r.now-last, limit)
                    self.assertLessEqual(r.now-last, limit+.010001)
                    self.assertEqual(r.s.receipts[name], last)
                    fault = r.s.receipt_fault
                    self.assertEqual(fault['topic'], name)
                    self.assertEqual(fault['limit_s'], limit)
                    count = len(r.outputs)
                    r.run_to(r.now+3)  # All sources return; no automatic resume.
                    self.assertEqual(len(r.outputs), count)
                    self.assertEqual(r.s.receipt_fault, fault)

    def test_missing_initial_topic_never_outputs(self):
        for name in RECEIPT_LIMITS:
            with self.subTest(topic=name):
                r = MixedRig(); r.run_to(10.1, stopped=(name,))
                self.assertFalse(r.outputs)
                self.assertIn('startup timeout', r.s.fault)

    def test_source_age_remains_independent_of_receipt_budget(self):
        for name in RECEIPT_LIMITS:
            for age in (.500001, -.050001):
                with self.subTest(topic=name, age=age):
                    r = MixedRig(); r.run_to(4)
                    before = r.s.receipts[name]
                    accepted = r.s.telemetry(name, int(200e6), age, r.now,
                                            lambda: self.fail('invalid source accepted'))
                    self.assertFalse(accepted)
                    self.assertEqual(r.s.receipts[name], before)
                    self.assertIn('invalid/stale source: '+name, r.s.fault)

    def test_receipt_boundary_inclusive_and_old_uniform_limit_removed(self):
        for name, limit in RECEIPT_LIMITS.items():
            with self.subTest(topic=name):
                r = Rig(); r.ready()
                r.s.receipts[name] = r.now-limit
                self.assertTrue(r.s.watchdog(r.now))
                r.s.receipts[name] -= .000001
                self.assertFalse(r.s.watchdog(r.now))
                self.assertEqual(r.s.receipt_fault['topic'], name)

    def test_receipt_missing_before_first_output_blocks(self):
        r = Rig()
        # Populate core via the fixture, then model an unaccepted receipt.
        r.step()
        self.assertEqual(r.s.sent, 0)
        r.s.receipts.pop('status')
        self.assertFalse(r.s.watchdog(r.now))
        self.assertIsNone(r.s.fault)


if __name__ == '__main__':
    unittest.main()
