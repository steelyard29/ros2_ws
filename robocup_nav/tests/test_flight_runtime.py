from dataclasses import replace
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from flight_runtime_core import FlightRuntimeCore
from flight_runtime import topics


class RuntimeRig:
    def __init__(self, aux_params=None):
        self.c = FlightRuntimeCore(0, aux_params=aux_params)
        self.now = 0
        self.armed = False
        self.nav = 2
        self.z = .7
        self.output = None
        self.ev_outputs = []

    def tick(self, *, vision=True, status=True, switches=True, kill=3, mode=6,
             owner=True, tracking=1, start=True):
        self.now = round(self.now+.05, 4)
        now = self.now
        if self.output and self.output.command == 'OFFBOARD':
            self.nav = 14
        if self.output and self.output.command == 'ARM':
            self.armed = True
        if self.output and self.output.stream and self.armed:
            self.z = self.output.position[2]
        self.c.ownership(now, owner)
        if status:
            self.c.status(2 if self.armed else 1, now, nav_state=self.nav,
                          preflight_ok=True, failsafe=False)
        self.c.local(now, x=2., y=-3., z=self.z, heading=.8,
                     vx=0., vy=0., vz=0., xy_valid=True, z_valid=True,
                     v_xy_valid=True, v_z_valid=True, reset_counters=(0, 0, 0, 0, 0))
        self.c.flags(now, ev_position=True, ev_height=True, ev_yaw=True,
                     ev_velocity=False, baro_height=True, range_height=False)
        if self.c.aux is None:
            self.c.rc(now, True)
        self.c.land(now, not self.armed)
        if switches:
            if self.c.aux:
                self.c.aux_message(int((now+1)*1e6), int((now+1)*1e6)-1000, now, 0.,
                                   True, 1, -1. if mode == 1 else 1., 1. if kill == 1 else -1.)
            else:
                self.c.switches(now, mode, kill)
        stamp = int((now+1)*1e9)
        if vision:
            self.c.tracking(tracking, stamp, now)
            ev = self.c.pose(stamp, 0, now, (0, 0, .7-self.z), (0, 0, 0, 1), True, stamp)
            if ev:
                self.ev_outputs.append(ev)
        self.output = self.c.tick(now, start=start)
        if self.output.command:
            self.c.sent(self.output.command, stamp//1000)
        return self.output

    def airborne(self):
        for _ in range(100):
            self.tick()
        assert self.c.controller.state in ('TAKEOFF', 'HOVER')


class RuntimeTests(unittest.TestCase):
    def test_topics_never_publish_to_fmu(self):
        for isolated in (False, True):
            inputs, output = topics(isolated)
            self.assertTrue(output.startswith('/robocup/'))
            if isolated:
                self.assertTrue(all(t.startswith('/robocup/') for t in inputs.values()))

    def test_ground_warmup_arms_synthetic_controller_and_keeps_ev(self):
        r = RuntimeRig()
        r.airborne()
        count = len(r.ev_outputs)
        r.tick()
        self.assertGreater(len(r.ev_outputs), count)
        self.assertTrue(r.c.ev.armed)

    def test_missing_switch_feed_never_starts(self):
        r = RuntimeRig()
        for _ in range(100):
            r.tick(switches=False)
        self.assertEqual(r.c.controller.state, 'WAIT')
        self.assertGreater(len(r.ev_outputs), 0)
        self.assertIsNone(r.output.command)

    def test_vision_failure_drives_land_and_cannot_resume(self):
        r = RuntimeRig()
        r.airborne()
        count = len(r.ev_outputs)
        out = r.tick(tracking=2)
        self.assertEqual(out.state, 'ABORT')
        self.assertEqual(out.command, 'LAND')
        self.assertFalse(out.stream)
        for _ in range(10):
            r.tick()
        self.assertEqual(len(r.ev_outputs), count)
        self.assertEqual(r.c.controller.state, 'ABORT')

    def test_silent_vio_loss_watchdog_stops_ev(self):
        r = RuntimeRig()
        r.airborne()
        for _ in range(8):
            r.tick(vision=False)
        self.assertTrue(r.c.ev.inhibit)
        self.assertEqual(r.c.controller.state, 'ABORT')

    def test_physical_kill_never_sends_land_or_rearms(self):
        r = RuntimeRig()
        r.airborne()
        out = r.tick(kill=1)
        self.assertEqual(out.state, 'KILLED')
        self.assertFalse(out.stream)
        self.assertIsNone(out.command)
        out = r.tick(kill=3)
        self.assertEqual(out.state, 'KILLED')
        self.assertIsNone(out.command)

    def test_mode_input_takeover_does_not_wait_for_vehicle_status(self):
        r = RuntimeRig()
        r.airborne()
        r.tick(mode=6)
        out = r.tick(mode=1)
        self.assertEqual(out.state, 'HANDOVER')
        self.assertIsNone(out.command)
        self.assertEqual(r.nav, 14)

    def test_owner_conflict_stops_control_and_ev_latched(self):
        r = RuntimeRig()
        r.airborne()
        count = len(r.ev_outputs)
        out = r.tick(owner=False)
        self.assertEqual(out.state, 'HANDOVER')
        self.assertIsNone(out.command)
        r.tick(owner=True)
        self.assertEqual(len(r.ev_outputs), count)

    def test_ack_rejection_aborts_but_acceptance_does_not_prove_state(self):
        r = RuntimeRig()
        for _ in range(80):
            r.tick()
            if r.output.command == 'OFFBOARD':
                break
        now_us = int((r.now+1)*1e6)
        self.assertFalse(r.c.ack('OFFBOARD', 0, now_us-1, r.now))
        self.assertTrue(r.c.ack('OFFBOARD', 0, now_us, r.now))
        self.assertEqual(r.c.controller.state, 'OFFBOARD')
        self.assertTrue(r.c.ack('OFFBOARD', 2, now_us, r.now))
        r.output = None  # No synthetic mode change on rejection.
        self.assertEqual(r.tick().state, 'STOPPED')  # Rejected on ground, already disarmed.
        self.assertIn('ABORT', [item['state'] for item in r.c.controller.history])

    def test_duplicate_telemetry_not_freshened_reset_latches(self):
        c = FlightRuntimeCore(0)
        self.assertTrue(c.receive('status', 100, .1, .01))
        self.assertFalse(c.receive('status', 100, .4, .01))
        self.assertFalse(c.receive('status', 99, .5, .01))
        self.assertIn('reset', c.fault)

    def test_battery_warning_has_no_automatic_land(self):
        r = RuntimeRig()
        r.airborne()
        r.c.battery_warning = 2
        out = r.tick()
        self.assertIn(out.state, ('TAKEOFF', 'HOVER'))
        self.assertNotEqual(out.command, 'LAND')

    def test_position_slot_cannot_start_and_stream_takeover_latches(self):
        r = RuntimeRig()
        for _ in range(50):
            r.tick(mode=1)
        self.assertEqual(r.c.controller.state, 'WAIT')
        self.assertIn('operator_offboard_slot', r.c.report()['start_blockers'])
        self.assertEqual(r.tick(mode=6).state, 'STREAM')
        out = r.tick(mode=1)
        self.assertEqual(out.state, 'HANDOVER')
        self.assertFalse(out.stream)
        self.assertIsNone(out.command)

    def test_switches_one_hz_is_valid_and_frozen_sample_not_renewed(self):
        c = FlightRuntimeCore(0)
        for sec in range(1, 4):
            self.assertTrue(c.switch_message(sec*1_000_000, sec*1_000_000-20_000,
                                            sec, .01, 6, 3, 0))
        self.assertIsNone(c.fault)
        self.assertFalse(c.switch_message(3_100_000, 2_980_000, 3.1, .01, 6, 3, 0))
        self.assertEqual(c.switch_at, 3)

    def test_invalid_switch_sample_and_dedicated_mapping_fail_closed(self):
        for sample, offboard in ((0, 0), (1, 0), (1_010_000, 0), (980_000, 1)):
            c = FlightRuntimeCore(0)
            self.assertFalse(c.switch_message(1_000_000, sample, 1., 0, 6, 3, offboard))
            self.assertIsNotNone(c.fault)
            self.assertEqual(c.switch_at, -1)

    def test_fresh_switch_takeover_does_not_require_fresh_status(self):
        r = RuntimeRig()
        r.airborne()
        r.c.sample = replace(r.c.sample, status_at=0)
        out = r.tick(status=False, mode=1)
        self.assertEqual(out.state, 'HANDOVER')
        self.assertIsNone(out.command)

    def test_aux_route_reaches_airborne_synthetic_and_kill_is_latched(self):
        from test_aux_switch_decoder import params
        r = RuntimeRig(aux_params=params())
        r.airborne()
        self.assertEqual(r.c.report()['switch_source'], 'derived_aux_mirror')
        self.assertEqual(r.tick(kill=1).state, 'KILLED')
        self.assertIsNone(r.tick(kill=3).command)

    def test_aux_source_exclusive_and_stale_threshold(self):
        from test_aux_switch_decoder import params
        c = FlightRuntimeCore(0, aux_params=params())
        c.switch_at = 1
        self.assertTrue(c.switch_fresh(1.4))
        self.assertFalse(c.switch_fresh(1.6))
        self.assertFalse(c.switch_message(1_000_000, 999_000, 1., 0., 6, 3, 0))
        self.assertIn('unexpected native', c.fault)
        c = FlightRuntimeCore(0)
        self.assertFalse(c.aux_message(1_000_000, 999_000, 1., 0., True, 1, -1., -1.))
        self.assertIn('unexpected AUX', c.fault)

    def test_aux_missing_stops_active_session(self):
        from test_aux_switch_decoder import params
        r = RuntimeRig(aux_params=params())
        r.airborne()
        for _ in range(12):
            r.tick(switches=False)
        self.assertEqual(r.c.controller.state, 'ABORT')
        self.assertFalse(r.output.stream)


if __name__ == '__main__':
    unittest.main()
