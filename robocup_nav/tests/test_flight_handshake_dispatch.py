from dataclasses import replace
from pathlib import Path
import subprocess
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from handshake_dispatch import HandshakeDispatchGuard, handshake_routes, handshake_ownership_ok, validate_target
from handshake_runtime_core import HandshakeRuntimeCore
from flight_runtime import create_node
from test_flight_handshake_runtime import HandshakeRig
from test_aux_switch_decoder import params


class StagedRig(HandshakeRig):
    def __init__(self, switch_wait_s=20):
        super().__init__()
        self.c = HandshakeRuntimeCore(0, aux_params=params(), staged=True, switch_wait_s=switch_wait_s)
        self.guard = HandshakeDispatchGuard()

    def tick(self, **kwargs):
        if 'mode' not in kwargs:
            kwargs['mode'] = 1 if self.c.handshake.state in ('WAIT', 'PRESTREAM') else 6
        out = super().tick(**kwargs)
        self.guard.check(out, self.c, self.now, int(self.now*1e6))
        return out


class DispatchTests(unittest.TestCase):
    def test_required_gcs_loss_stops_and_latches_without_mode_request(self):
        r = self.active()
        r.c.require_gcs = True
        r.c.gcs_status(True, r.now)
        self.assertTrue(r.tick(mode=1).stream)
        count = len(r.ev_outputs)
        r.c.gcs_status(False, r.now)
        self.assertEqual(r.tick(mode=1).state, 'ABORT')
        self.assertEqual(len(r.ev_outputs), count)
        r.c.gcs_status(True, r.now)
        self.assertFalse(r.tick(mode=1).stream)
        self.assertEqual(r.guard.command_count, 0)

    def active(self, switch_wait_s=20):
        r = StagedRig(switch_wait_s=switch_wait_s)
        for _ in range(140):
            r.tick(mode=1)
            if r.c.handshake.state == 'AWAIT_SWITCH':
                return r
        self.fail('did not reach switch window')

    def test_longer_operator_window_is_bounded_and_never_sends_mode_without_edge(self):
        r = self.active(switch_wait_s=60)
        for _ in range(420):
            r.tick(mode=1)
        self.assertEqual(r.c.handshake.state, 'AWAIT_SWITCH')
        for _ in range(800):
            r.tick(mode=1)
        self.assertEqual(r.c.handshake.state, 'ABORT')
        self.assertEqual(r.guard.command_count, 0)
        with self.assertRaises(ValueError):
            StagedRig(switch_wait_s=600)

    def test_fixed_routes_and_exact_ownership(self):
        mapping = handshake_routes()
        self.assertEqual(len(mapping), 4)
        own = dict.fromkeys(mapping, 1)
        inputs = dict.fromkeys(mapping.values(), 1)
        self.assertTrue(handshake_ownership_ok(own, inputs))
        for bad in ({}, {**inputs, '/fmu/in/actuator_motors': 1},
                    {**inputs, '/fmu/in/vehicle_command': 2}):
            self.assertFalse(handshake_ownership_ok(own, bad))
        validate_target({'MAV_SYS_ID': 1, 'MAV_COMP_ID': 1})
        with self.assertRaises(ValueError):
            validate_target({'MAV_SYS_ID': 2, 'MAV_COMP_ID': 1})

    def test_position_baseline_prestream_switch_ack_done(self):
        r = StagedRig()
        for _ in range(160):
            r.tick()
        self.assertEqual(r.c.handshake.state, 'DONE')
        self.assertFalse(r.armed)
        self.assertEqual(r.guard.command_count, 1)
        self.assertTrue(r.c.ev_stopped)
        self.assertEqual(r.c.command_source_component, 191)

    def test_status_can_precede_debounced_switch(self):
        r = self.active()
        r.nav = 14
        for _ in range(4):
            r.tick(mode=1)
        for _ in range(50):
            r.tick(mode=6)
        self.assertEqual(r.c.handshake.state, 'DONE')

    def test_status_without_switch_cannot_be_success(self):
        r = self.active()
        r.nav = 14
        for _ in range(15):
            r.tick(mode=1)
        self.assertEqual(r.c.handshake.state, 'ABORT')
        self.assertEqual(r.guard.command_count, 0)

    def test_no_operator_edge_times_out_without_mode_command(self):
        r = self.active()
        for _ in range(420):
            r.tick(mode=1)
        self.assertEqual(r.c.handshake.state, 'ABORT')
        self.assertEqual(r.guard.command_count, 0)

    def test_ground_setpoint_tamper_is_rejected_and_latched(self):
        r = self.active()
        bad = replace(r.output, position=(2, -3, -.5))
        with self.assertRaises(ValueError):
            r.guard.check(bad, r.c, r.now, int(r.now*1e6)+1)
        self.assertTrue(r.c.fault)

    def test_arm_and_duplicate_mode_are_rejected(self):
        r = self.active()
        with self.assertRaises(ValueError):
            r.guard.check(replace(r.output, command='ARM'), r.c, r.now, int(r.now*1e6)+1)
        r = self.active()
        for _ in range(5):
            out = r.tick(mode=6)
            if out.command:
                break
        self.assertEqual(out.command, 'OFFBOARD')
        with self.assertRaises(ValueError):
            r.guard.check(out, r.c, r.now, int(r.now*1e6)+1)

    def test_real_route_rejects_synthetic_and_unstaged_before_ros(self):
        for core, isolated, exercise in ((HandshakeRuntimeCore(0, aux_params=params()), False, True),
                (HandshakeRuntimeCore(0, aux_params=params(), staged=True), True, True),
                (HandshakeRuntimeCore(0, aux_params=params(), staged=True), False, False)):
            with self.assertRaises(ValueError):
                create_node(core, isolated=isolated, exercise=exercise, handshake_bench=True)

    def test_entrypoint_requires_explicit_authorizations_before_ros(self):
        script = Path(__file__).resolve().parents[1]/'scripts/handshake_bench_runtime.py'
        result = subprocess.run([sys.executable, str(script), '--prop-off'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn('--authorize-mode-handshake', result.stderr)
        self.assertIn('--confirm-component-191-exclusive', result.stderr)


if __name__ == '__main__':
    unittest.main()
