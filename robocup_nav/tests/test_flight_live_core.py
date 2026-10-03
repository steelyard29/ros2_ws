from dataclasses import replace
from pathlib import Path
import subprocess
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from live_flight_core import LiveFlightCore
from flight_runtime import create_node
from test_flight_runtime import RuntimeRig
from test_aux_switch_decoder import params


class LiveRig(RuntimeRig):
    def __init__(self):
        super().__init__(params())
        self.c = LiveFlightCore(0., params())

    def tick(self, acknowledge=True, gcs=True, **kwargs):
        self.c.gcs_status(gcs, round(self.now+.05, 4))
        out = super().tick(**kwargs)
        if out.command and acknowledge:
            self.c.ack(out.command, 0, int((self.now+1)*1e6)+1, self.now)
        return out

    def ready(self):
        for _ in range(160):
            self.tick(mode=1)
        assert self.c.ready_at is not None, self.c.report()

    def airborne(self):
        self.ready()
        for _ in range(65):
            self.tick(mode=6)
        assert self.armed, self.c.report()


class LiveCoreTests(unittest.TestCase):
    def test_repeated_offboard_callbacks_before_tick_preserve_single_edge(self):
        r = LiveRig()
        r.ready()
        r.c.switches(r.now+.01, 6, 3)
        edge = r.c.edge_at
        r.c.switches(r.now+.02, 6, 3)
        self.assertIsNone(r.c.fault)
        self.assertEqual(r.c.edge_at, edge)
        out = r.c.tick(r.now+.03, start=True)
        self.assertEqual(out.state, 'STREAM')

    def test_px4_offboard_nav_after_ready_is_accepted_before_aux_slot(self):
        r = LiveRig()
        r.ready()
        r.c.status(1, r.now+.02, nav_state=14, preflight_ok=False, failsafe=False)
        out = r.c.tick(r.now+.03, start=True)
        self.assertIsNone(r.c.fault)
        self.assertIsNotNone(r.c.edge_at)
        self.assertEqual(out.state, 'STREAM')
        # Vision callback still sees the pre-debounce Position slot.
        self.assertEqual(r.c.mode_slot, 1)
        self.assertTrue(r.c.ev_allowed(r.now+.04))
        self.assertFalse(r.c.ev_stopped)
        r.c.switches(r.now+.05, 6, 3)
        self.assertTrue(r.c.ev_allowed(r.now+.06))
        self.assertFalse(r.c.ev_stopped)
        self.assertIsNone(r.c.fault)

    def test_stream_position_slot_without_offboard_nav_stops_ev(self):
        r = LiveRig()
        r.ready()
        r.c.switches(r.now+.01, 6, 3)
        r.c.tick(r.now+.03, start=True)
        self.assertEqual(r.c.controller.state, 'STREAM')
        r.c.mode_slot = 1
        r.c.status(1, r.now+.05, nav_state=2, preflight_ok=True, failsafe=False)
        self.assertFalse(r.c.ev_allowed(r.now+.06))
        self.assertTrue(r.c.ev_stopped)

    def test_ready_revocation_does_not_reopen_old_operator_window(self):
        r = LiveRig()
        r.ready()
        r.c.sample = replace(r.c.sample, preflight_ok=False)
        out = r.c.tick(r.now+.03, start=True)
        self.assertEqual(out.state, 'STOPPED')
        self.assertIn('READY conditions lost', r.c.fault)
        self.assertFalse(r.c.arm_requested)

    def test_position_waits_for_new_edge_and_never_arms_on_ready(self):
        r = LiveRig()
        r.ready()
        self.assertEqual(r.c.controller.state, 'WAIT')
        self.assertFalse(r.c.arm_requested)
        self.assertGreater(r.c.ev.publish_count, 0)
        for _ in range(65):
            r.tick(mode=6)
        self.assertTrue(r.c.arm_requested)
        self.assertTrue(r.armed)
        self.assertEqual(r.c.accepted_commands, {'OFFBOARD', 'ARM'})

    def test_early_offboard_never_arms(self):
        r = LiveRig()
        for _ in range(100):
            r.tick(mode=6)
        self.assertFalse(r.c.arm_requested)
        self.assertIsNotNone(r.c.fault)

    def test_missing_ack_never_arms_even_when_mode_observed(self):
        r = LiveRig()
        r.ready()
        for _ in range(180):
            r.tick(mode=6, acknowledge=False)
        self.assertEqual(r.nav, 14)
        self.assertFalse(r.c.arm_requested)
        self.assertIn(r.c.controller.state, {'ABORT', 'STOPPED'})

    def test_in_progress_not_accepted_and_old_ack_rejected(self):
        c = LiveFlightCore(0, params())
        c.sent('OFFBOARD', 100)
        self.assertFalse(c.ack('OFFBOARD', 0, 99, 1))
        self.assertTrue(c.ack('OFFBOARD', 5, 101, 1))
        self.assertFalse(c.accepted_commands)
        self.assertTrue(c.ack('OFFBOARD', 0, 102, 1))
        self.assertIn('OFFBOARD', c.accepted_commands)

    def test_kill_takeover_stop_ev_immediately_and_no_resume(self):
        for change, state in (({'kill': 1}, 'KILLED'), ({'mode': 1}, 'HANDOVER')):
            r = LiveRig()
            r.airborne()
            out = r.tick(**change)
            if 'mode' in change:
                for _ in range(3):  # AUX non-kill transitions require 100 ms debounce.
                    out = r.tick(**change)
            self.assertEqual(out.state, state)
            self.assertIsNone(out.command)
            count = r.c.ev.publish_count
            for _ in range(10):
                r.tick()
            self.assertEqual(r.c.ev.publish_count, count)
            self.assertFalse(r.output.stream)

    def test_vision_loss_lands_without_resuming_ev(self):
        r = LiveRig()
        r.airborne()
        out = r.tick(tracking=2)
        self.assertEqual(out.command, 'LAND')
        self.assertFalse(out.stream)
        count = r.c.ev.publish_count
        r.tick()
        self.assertEqual(count, r.c.ev.publish_count)

    def test_interrupt_requests_land_only_with_fresh_status(self):
        for fresh in (True, False):
            r = LiveRig()
            r.airborne()
            r.c.request_abort(r.now, 'operator interrupt')
            if not fresh:
                r.c.sample = replace(r.c.sample, status_at=0)
            out = r.c.tick(r.now+.05)
            self.assertEqual(out.command, 'LAND' if fresh else None)
            self.assertFalse(out.stream)

    def test_unexpected_arm_is_not_adopted(self):
        r = LiveRig()
        r.ready()
        r.armed = True
        out = r.tick(mode=1)
        self.assertEqual(out.state, 'HANDOVER')
        self.assertFalse(r.c.arm_requested)
        self.assertFalse(out.stream)
        self.assertIsNone(out.command)

    def test_closed_core_emits_nothing(self):
        r = LiveRig()
        r.airborne()
        r.c.close('exception')
        count = r.c.ev.publish_count
        out = r.tick()
        self.assertFalse(out.stream)
        self.assertIsNone(out.command)
        self.assertEqual(count, r.c.ev.publish_count)

    def test_no_gcs_no_ready(self):
        r = LiveRig()
        for _ in range(160):
            r.tick(mode=1, gcs=False)
        self.assertIsNone(r.c.ready_at)
        self.assertFalse(r.c.arm_requested)

    def test_executive_deadline_stops_stream_and_requests_land(self):
        r = LiveRig()
        r.airborne()
        out = r.c.tick(r.now+.31)
        self.assertEqual(out.state, 'ABORT')
        self.assertEqual(out.command, 'LAND')
        self.assertFalse(out.stream)
        self.assertTrue(r.c.ev_stopped)

    def test_interrupt_callback_closes_ev_and_prevents_new_arm(self):
        r = LiveRig()
        r.ready()
        r.c.abort_requested = lambda: True
        count = r.c.ev.publish_count
        for _ in range(10):
            r.tick(mode=6)
        self.assertEqual(r.c.ev.publish_count, count)
        self.assertFalse(r.c.arm_requested)
        self.assertFalse(r.output.stream)

    def test_pose_callback_cannot_outlive_executive(self):
        r = LiveRig()
        r.airborne()
        self.assertFalse(r.c.ev_allowed(r.now+.31))
        self.assertTrue(r.c.ev_stopped)
        self.assertEqual(r.c.controller.state, 'ABORT')

    def test_source_reset_stops_session(self):
        r = LiveRig()
        r.airborne()
        self.assertTrue(r.c.receive('test_source', 200, r.now, .01))
        self.assertFalse(r.c.receive('test_source', 100, r.now+.01, .01))
        out = r.c.tick(r.now+.05)
        self.assertEqual(out.command, 'LAND')
        self.assertFalse(out.stream)
        self.assertTrue(r.c.ev_stopped)

    def test_local_frame_reset_aborts(self):
        r = LiveRig()
        r.airborne()
        r.c.sample = replace(r.c.sample, reset_counters=(1,0,0,0,0))
        out = r.c.tick(r.now+.05)
        self.assertEqual(out.state, 'ABORT')
        self.assertEqual(out.command, 'LAND')

    def test_expired_ready_cannot_be_reused(self):
        r = LiveRig()
        r.ready()
        for _ in range(1300):
            r.tick(mode=1)
        self.assertEqual(r.c.controller.state, 'STOPPED')
        for _ in range(100):
            r.tick(mode=6)
        self.assertFalse(r.c.arm_requested)

    def test_create_node_rejects_unreleased_real_core_before_ros(self):
        c = LiveFlightCore(0, params())
        with self.assertRaises(ValueError):
            create_node(c, exercise=True)
        with self.assertRaises(ValueError):
            create_node(c, isolated=True, exercise=True, live_permit=object())

    def test_cli_missing_authorization_never_imports_ros(self):
        script = Path(__file__).resolve().parents[1]/'scripts/live_flight_runtime.py'
        p = subprocess.run([sys.executable, str(script), '--params', '/not-used',
                            '--release', '/not-used'], capture_output=True, text=True, timeout=5)
        self.assertEqual(p.returncode, 2)
        self.assertIn('--authorize-real-flight', p.stderr)


if __name__ == '__main__':
    unittest.main()
