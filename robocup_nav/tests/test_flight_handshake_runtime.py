from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from handshake_runtime_core import HandshakeRuntimeCore
from flight_runtime import create_node
from test_flight_runtime import RuntimeRig
from test_aux_switch_decoder import params


class HandshakeRig(RuntimeRig):
    def __init__(self):
        super().__init__()
        self.c = HandshakeRuntimeCore(0, aux_params=params())

    def tick(self, accept=True, **kwargs):
        out = super().tick(**kwargs)
        if out.command and accept:
            self.c.ack(out.command, 0, int((self.now+1)*1e6), self.now)
        return out


class HandshakeRuntimeTests(unittest.TestCase):
    def test_real_ev_route_is_rejected_before_ros_import(self):
        with self.assertRaises(ValueError):
            create_node(HandshakeRuntimeCore(0, aux_params=params()), bench_ev=True)

    def test_nominal_ack_state_and_ev_stop(self):
        r = HandshakeRig()
        commands = []
        for _ in range(100):
            out = r.tick()
            if out.command:
                commands.append(out.command)
            if out.stream:
                self.assertEqual(out.position, (2., -3., .7))
        self.assertEqual(commands, ['OFFBOARD'])
        self.assertEqual(r.c.handshake.state, 'DONE')
        self.assertFalse(r.armed)
        n = len(r.ev_outputs)
        for _ in range(10):
            self.assertFalse(r.tick().stream)
        self.assertEqual(len(r.ev_outputs), n)
        self.assertTrue(r.c.report()['ev_stopped'])

    def test_mode_without_ack_times_out(self):
        r = HandshakeRig()
        for _ in range(140):
            r.tick(accept=False)
        self.assertEqual(r.nav, 14)
        self.assertEqual(r.c.handshake.state, 'ABORT')
        self.assertFalse(r.c.ack_accepted)

    def test_ack_rejection_stops_control_and_ev(self):
        r = HandshakeRig()
        for _ in range(100):
            if r.tick(accept=False).command:
                break
        stamp = r.c.pending['stamp']
        self.assertFalse(r.c.ack('ARM', 0, stamp, r.now))
        self.assertFalse(r.c.ack('OFFBOARD', 0, stamp-1, r.now))
        self.assertTrue(r.c.ack('OFFBOARD', 2, stamp, r.now))
        n = len(r.ev_outputs)
        self.assertEqual(r.tick().state, 'ABORT')
        self.assertEqual(len(r.ev_outputs), n)
        self.assertFalse(r.c.ack('OFFBOARD', 0, stamp+1, r.now))

    def test_mode_change_keeps_ev_until_terminal(self):
        r = HandshakeRig()
        for _ in range(100):
            if r.tick().command:
                break
        n = len(r.ev_outputs)
        for _ in range(5):
            r.tick()
        self.assertEqual(r.nav, 14)
        self.assertGreater(len(r.ev_outputs), n)
        self.assertFalse(r.c.ev.inhibit)

    def test_active_faults_stop_ev_and_cannot_restart(self):
        for change, expected in (({'kill': 1}, 'KILLED'), ({'mode': 1}, 'HANDOVER'),
                                 ({'tracking': 2}, 'ABORT'), ({'owner': False}, 'ABORT')):
            with self.subTest(change=change):
                r = HandshakeRig()
                for _ in range(40):
                    r.tick()
                self.assertEqual(r.c.handshake.state, 'PRESTREAM')
                n = len(r.ev_outputs)
                for _ in range(4):
                    r.tick(**change)
                self.assertEqual(r.c.handshake.state, expected)
                # AUX normal-state debounce may retain prior slot briefly.
                after = len(r.ev_outputs)
                for _ in range(5):
                    self.assertFalse(r.tick().stream)
                self.assertEqual(len(r.ev_outputs), after)
                if 'mode' not in change:
                    self.assertEqual(after, n)

    def test_position_and_missing_switches_cannot_start(self):
        for changes in ({'mode': 1}, {'switches': False}):
            r = HandshakeRig()
            for _ in range(80):
                self.assertFalse(r.tick(**changes).stream)
            self.assertEqual(r.c.handshake.state, 'WAIT')

    def test_source_replay_cannot_renew_and_reset_latches(self):
        r = HandshakeRig()
        for _ in range(40):
            r.tick()
        self.assertTrue(r.c.receive('vehicle_status', 100, r.now, 0))
        self.assertFalse(r.c.receive('vehicle_status', 100, r.now+.01, 0))
        self.assertFalse(r.c.receive('vehicle_status', 99, r.now+.02, 0))
        self.assertEqual(r.tick().state, 'ABORT')

    def test_battery_warning_is_still_warning_only(self):
        r = HandshakeRig()
        r.c.battery_warning = 2
        for _ in range(100):
            r.tick()
        self.assertEqual(r.c.handshake.state, 'DONE')
        self.assertEqual(r.c.report()['battery_action'], 'warning_only')


if __name__ == '__main__':
    unittest.main()
