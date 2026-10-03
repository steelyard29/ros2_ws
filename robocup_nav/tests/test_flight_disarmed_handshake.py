from dataclasses import replace
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from disarmed_handshake import DisarmedHandshake, validate_handshake_output
from flight_supervisor import FlightOutput
from test_flight_supervisor import healthy


class HandshakeTests(unittest.TestCase):
    def test_operator_prompts_match_handshake_phase(self):
        for state in ('WAIT', 'PRESTREAM'):
            self.assertIn('keep Position', DisarmedHandshake.operator_action(state))
        self.assertIn('switch to Offboard now', DisarmedHandshake.operator_action('AWAIT_SWITCH'))
        for state in ('WAIT_MODE', 'VERIFY'):
            prompt = DisarmedHandshake.operator_action(state)
            self.assertIn('keep Offboard', prompt)
            self.assertNotIn('Position', prompt)
        for state in DisarmedHandshake.TERMINAL:
            self.assertIn('outputs stopped', DisarmedHandshake.operator_action(state))
            self.assertIn('restore Position', DisarmedHandshake.operator_action(state))
        self.assertIn('do not clear kill yet', DisarmedHandshake.operator_action('KILLED'))
        self.assertIn('do not switch or arm', DisarmedHandshake.operator_action('UNKNOWN'))

    def step(self, c, t, **changes):
        switch = {k: changes.pop(k) for k in list(changes)
                  if k in ('start', 'mode_slot', 'switch_at', 'kill_switch', 'ack_accepted')}
        opts = dict(mode_slot=6, switch_at=t, kill_switch=3, ack_accepted=True)
        opts.update(switch)
        return validate_handshake_output(c.step(t, replace(healthy(t), **changes), **opts))

    def running(self):
        c = DisarmedHandshake()
        self.step(c, 0, start=True)
        return c

    def test_default_is_silent(self):
        self.assertFalse(self.step(DisarmedHandshake(), 0).stream)

    def test_success_holds_ground_pose_never_arms_and_terminal_is_silent(self):
        c = self.running()
        out = []
        for i in range(1, 90):
            o = self.step(c, i*.05, nav_state=14 if i > 41 else 2)
            out.append(o)
            if o.stream:
                self.assertEqual(o.position, (2, -3, .7))
                self.assertEqual(o.yaw, .8)
        self.assertEqual(c.state, 'DONE')
        self.assertEqual([o.command for o in out if o.command], ['OFFBOARD'])
        self.assertFalse(out[-1].stream)

    def test_never_starts_with_bad_baseline(self):
        for changes in ({'mode_slot': 1}, {'nav_state': 14}, {'preflight_ok': False},
                        {'switch_at': -1}, {'armed': True}, {'kill_switch': 0}):
            with self.subTest(changes=changes):
                self.assertFalse(self.step(DisarmedHandshake(), 0, start=True, **changes).stream)

    def test_safety_faults_stop_and_latch(self):
        for changes in ({'armed': True}, {'landed': False}, {'ev_ok': False},
                        {'rc_valid': False}, {'status_at': -1}, {'local_at': -1},
                        {'switch_at': -1}, {'input_ownership_ok': False},
                        {'reset_counters': (1, 0, 0, 0, 0)}, {'x': 3}, {'heading': 1.2},
                        {'kill_switch': 1}, {'kill_switch': 0}, {'mode_slot': 1},
                        {'nav_state': 18}, {'manual_takeover': True}, {'failsafe': True}):
            with self.subTest(changes=changes):
                c = self.running()
                o = self.step(c, .05, **changes)
                self.assertFalse(o.stream)
                self.assertIsNone(o.command)
                self.assertFalse(self.step(c, .1, start=True).stream)

    def test_timeout_without_actual_state_never_succeeds(self):
        c = self.running()
        for i in range(1, 110):
            self.step(c, i*.05)
        self.assertEqual(c.state, 'ABORT')

    def test_old_status_cannot_confirm_mode(self):
        c = self.running()
        for i in range(1, 42):
            self.step(c, i*.05)
        self.step(c, 2.1, nav_state=14, status_at=2.)
        self.assertEqual(c.state, 'WAIT_MODE')

    def test_loop_gap_clock_duplicate_and_reverse_abort(self):
        for t in (.31, 0, -.1, float('nan')):
            c = self.running()
            self.assertFalse(self.step(c, t).stream)
            self.assertEqual(c.state, 'ABORT')

    def test_second_allowlist_rejects_flight_commands(self):
        for command in ('ARM', 'LAND', 'DISARM', 'SERVO'):
            with self.assertRaises(ValueError):
                validate_handshake_output(FlightOutput('WAIT_MODE', command=command))
        with self.assertRaises(ValueError):
            validate_handshake_output(FlightOutput('DONE', True, (0, 0, 0), 0))


if __name__ == '__main__':
    unittest.main()
