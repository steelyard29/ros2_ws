from dataclasses import replace
from pathlib import Path
import subprocess
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from fault_bench_core import KillBenchCore
from handshake_dispatch import HandshakeDispatchGuard
from test_flight_runtime import RuntimeRig
from test_aux_switch_decoder import params


class KillRig(RuntimeRig):
    def __init__(self):
        super().__init__()
        self.c = KillBenchCore(0, params(), 20)
        self.guard = HandshakeDispatchGuard()

    def tick(self, **kwargs):
        self.c.gcs_status(True, round(self.now+.05, 4))
        previous = len(self.ev_outputs)
        out = super().tick(mode=kwargs.pop('mode', 1), **kwargs)
        self.guard.check(out, self.c, self.now, int(self.now*1e6))
        if len(self.ev_outputs) > previous:
            self.c.note_output('vehicle_visual_odometry', self.now, int(self.now*1e6))
        if out.stream:
            for name in ('offboard_control_mode', 'trajectory_setpoint'):
                self.c.note_output(name, self.now, int(self.now*1e6))
        return out

    def ready(self):
        for _ in range(150):
            self.tick()
            if self.c.window_at is not None:
                return
        raise AssertionError(self.c.report())

    def recover(self):
        for _ in range(45):
            self.tick(kill=1)
            self.c.observe_restore(self.now, False)
        for _ in range(50):
            self.tick(kill=3)
            self.c.observe_restore(self.now, self.c.kill_switch == 3)


class FaultBenchTests(unittest.TestCase):
    def test_fresh_kill_after_stream_then_restore_qualifies_without_commands(self):
        r = KillRig()
        r.ready()
        self.assertEqual(r.tick(kill=1).state, 'KILLED')
        r.recover()
        result = r.c.qualification(r.now)
        self.assertTrue(result['passed'], result)
        self.assertEqual(r.guard.command_count, 0)
        self.assertGreater(result['event']['timestamp_sample'], 0)
        self.assertFalse(result['flight_approved'])
        self.assertFalse(result['motor_cutoff_verified'])

    def test_no_event_cannot_pass(self):
        r = KillRig()
        r.ready()
        for _ in range(450):
            r.tick()
        self.assertEqual(r.c.handshake.state, 'ABORT')
        self.assertFalse(r.c.qualification(r.now)['passed'])
        self.assertEqual(r.guard.command_count, 0)

    def test_early_kill_fails(self):
        r = KillRig()
        for _ in range(10):
            r.tick()
        r.tick(kill=1)
        r.recover()
        self.assertFalse(r.c.qualification(r.now)['passed'])
        self.assertIsNone(r.c.event)

    def test_offboard_switch_cannot_send_mode(self):
        r = KillRig()
        r.ready()
        for _ in range(6):
            out = r.tick(mode=6)
            self.assertIsNone(out.command)
        self.assertTrue(r.c.failure)
        self.assertEqual(r.guard.command_count, 0)

    def test_second_guard_rejects_mode_even_if_output_tampered(self):
        r = KillRig()
        r.ready()
        with self.assertRaises(ValueError):
            r.guard.check(replace(r.output, command='OFFBOARD'), r.c, r.now, int(r.now*1e6)+1)

    def test_missing_stream_or_wrong_fault_cannot_qualify(self):
        for kind in ('stream', 'fault'):
            r = KillRig()
            r.ready()
            if kind == 'stream':
                r.c.last_outputs.clear()
            else:
                r.c.trip('unrelated visual error')
            r.tick(kill=1)
            r.recover()
            self.assertFalse(r.c.qualification(r.now)['passed'])

    def test_replayed_sample_cannot_establish_kill_event(self):
        r = KillRig()
        r.ready()
        self.assertFalse(r.c.aux_message(r.c.aux.last_stamp, r.c.aux.last_sample,
                         r.now, 0, True, 1, -1., 1.))
        self.assertIsNone(r.c.event)

    def test_restore_required_and_early_clear_rejected(self):
        r = KillRig()
        r.ready()
        r.tick(kill=1)
        self.assertFalse(r.c.qualification(r.now)['passed'])
        for _ in range(5):
            r.tick(kill=3)
        self.assertIn('cleared', r.c.failure)

    def test_any_post_event_output_latches_failure(self):
        r = KillRig()
        r.ready()
        r.tick(kill=1)
        r.c.note_output('trajectory_setpoint', r.now+.01, 123)
        r.recover()
        self.assertFalse(r.c.qualification(r.now)['passed'])

    def test_fault_after_event_cannot_be_hidden(self):
        r = KillRig()
        r.ready()
        r.tick(kill=1)
        r.c.trip('source clock reset')
        r.recover()
        self.assertFalse(r.c.qualification(r.now)['passed'])

    def test_prompt_never_requests_offboard(self):
        r = KillRig()
        self.assertIn('assert only kill', r.c.operator_action('AWAIT_SWITCH'))
        self.assertNotIn('Offboard', r.c.operator_action('AWAIT_SWITCH'))

    def test_observer_pause_cannot_count_as_silence(self):
        r = KillRig()
        r.ready()
        r.tick(kill=1)
        r.c.observe_restore(r.now+3, True)
        self.assertIn('observation', r.c.failure)
        self.assertFalse(r.c.qualification(r.now+6)['passed'])

    def test_output_after_terminal_prevents_clear_prompt(self):
        r = KillRig()
        r.ready()
        r.tick(kill=1)
        r.c.note_output('trajectory_setpoint', r.now+.01, 123)
        for _ in range(45):
            r.tick(kill=1)
        self.assertFalse(r.c.clear_ready(r.now))

    def test_entrypoint_requires_separate_authorization_before_ros(self):
        script = Path(__file__).resolve().parents[1]/'scripts/fault_bench_runtime.py'
        result = subprocess.run([sys.executable, str(script), '--prop-off'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn('--authorize-fault-bench', result.stderr)
        self.assertIn('--scenario', result.stderr)


if __name__ == '__main__':
    unittest.main()
