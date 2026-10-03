from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from flight_supervisor import FlightSample, FlightSupervisor
from flight_ev_adapter import FlightEvAdapter
from ev_adapter import EvAdapter


def healthy(now=0):
    return FlightSample(status_at=now, local_at=now, flags_at=now, land_at=now,
                        rc_at=now, landed=True, nav_state=2, preflight_ok=True,
                        rc_valid=True, ev_ok=True, ev_position=True, ev_height=True,
                        input_ownership_ok=True,
                        ev_yaw=True, baro_height=True, xy_valid=True, z_valid=True,
                        v_xy_valid=True, v_z_valid=True, x=2, y=-3, z=.7,
                        heading=.8, vx=0, vy=0, vz=0, reset_counters=(0, 0, 0, 0, 0))


class Rig:
    def __init__(self):
        self.now = 0
        self.c = FlightSupervisor()
        self.s = healthy()
        self.out = self.c.step(0, self.s, start=True)
        self.outputs = [self.out]

    def tick(self, **changes):
        self.now = round(self.now + .05, 4)
        defaults = dict(status_at=self.now, local_at=self.now, flags_at=self.now,
                        land_at=self.now, rc_at=self.now)
        defaults.update(changes)
        self.s = replace(self.s, **defaults)
        self.out = self.c.step(self.now, self.s)
        self.outputs.append(self.out)
        return self.out

    def takeoff(self):
        for _ in range(41):
            self.tick()
        self.tick(nav_state=14)
        self.tick(armed=True)
        assert self.c.state == 'TAKEOFF'


class FlightTests(unittest.TestCase):
    def test_default_evidence_never_starts(self):
        c = FlightSupervisor()
        self.assertFalse(c.step(0, FlightSample(), start=True).stream)

    def test_nonzero_local_origin_heading_and_rate_limit(self):
        h = Rig()
        h.takeoff()
        previous = h.c.z_sp
        for _ in range(60):
            out = h.tick(landed=False, z=h.c.z_sp)
            self.assertEqual(out.position[:2], (2, -3))
            self.assertAlmostEqual(out.yaw, .8)
            self.assertLessEqual(previous-out.position[2], .0100001)
            self.assertGreaterEqual(out.position[2], .7-.5)
            previous = out.position[2]

    def test_nominal_closed_loop(self):
        h = Rig()
        h.takeoff()
        for _ in range(200):
            h.tick(landed=False, z=h.c.z_sp)
            if h.c.state == 'LAND':
                break
        self.assertEqual(h.c.state, 'LAND')
        self.assertEqual(h.out.command, 'LAND')
        self.assertFalse(h.out.stream)
        h.tick(landed=True, armed=False, z=.7)
        self.assertEqual(h.c.state, 'DONE')

    def test_failed_mode_handshake_never_arms(self):
        h = Rig()
        for _ in range(160):
            h.tick()
        self.assertIn(h.c.state, {'ABORT', 'STOPPED'})
        self.assertFalse(any(o.command == 'ARM' for o in h.outputs))

    def test_vision_loss_lands_without_stream_and_latches(self):
        h = Rig()
        h.takeoff()
        out = h.tick(ev_ok=False)
        self.assertEqual(out.command, 'LAND')
        self.assertFalse(out.stream)
        h.tick(ev_ok=True)
        self.assertEqual(h.c.state, 'ABORT')

    def test_link_loss_does_not_claim_land_delivery(self):
        h = Rig()
        h.takeoff()
        out = h.tick(status_at=0)
        self.assertEqual(out.state, 'ABORT')
        self.assertFalse(out.stream)
        self.assertIsNone(out.command)
        h.tick()
        self.assertEqual(h.out.command, 'LAND')
        self.assertEqual(h.c.state, 'ABORT')

    def test_manual_takeover_does_not_fight_pilot(self):
        for change in ({'manual_takeover': True}, {'nav_state': 2}, {'input_ownership_ok': False}):
            h = Rig()
            h.takeoff()
            out = h.tick(**change)
            self.assertEqual(out.state, 'HANDOVER')
            self.assertFalse(out.stream)
            self.assertIsNone(out.command)
            h.tick(nav_state=14, manual_takeover=False)
            self.assertEqual(h.c.state, 'HANDOVER')

    def test_reset_stale_rc_bad_fusion_and_overshoot(self):
        for change in ({'reset_counters': (1, 0, 0, 0, 0)}, {'local_at': 0},
                       {'rc_valid': False}, {'land_at': 0}, {'ev_velocity': True},
                       {'baro_height': False}, {'x': 3}, {'z': -.1}, {'vx': math.nan}):
            with self.subTest(change=change):
                h = Rig()
                h.takeoff()
                h.tick(**change)
                self.assertEqual(h.c.state, 'ABORT')

    def test_no_force_disarm_and_no_stale_landing_completion(self):
        h = Rig()
        h.takeoff()
        h.tick(ev_ok=False, z=.7)
        h.tick(landed=True, armed=False, land_at=0)
        self.assertEqual(h.c.state, 'ABORT')
        self.assertFalse(any(o.command == 'DISARM' for o in h.outputs))
        h.tick()
        self.assertEqual(h.c.state, 'STOPPED')

    def test_actual_two_hz_status_is_not_stale(self):
        h = Rig()
        for i in range(10):
            h.tick(status_at=0)
        self.assertEqual(h.c.state, 'STREAM')


def drive_ev(adapter, armed=False, start=0, count=40):
    result = None
    for i in range(count):
        t = start + i*.033
        stamp = int((t+1)*1e9)
        adapter.on_vehicle(2 if armed else 1, t)
        adapter.on_flags(True, False, t)
        adapter.on_tracking(1, stamp, t)
        result = adapter.on_pose(stamp, 20, t, [0, 0, 0], [0, 0, 0, 1])
    return result


class FlightEvTests(unittest.TestCase):
    def test_armed_continues_only_after_ground_warmup(self):
        a = FlightEvAdapter(0)
        self.assertIsNotNone(drive_ev(a))
        self.assertIsNotNone(drive_ev(a, armed=True, start=1.32))
        self.assertIsNone(a.fault)
        self.assertTrue(a.armed)

    def test_airborne_start_and_premature_arm_rejected(self):
        for count in (0, 10):
            a = FlightEvAdapter(0)
            drive_ev(a, count=count)
            self.assertFalse(a.on_vehicle(2, .5))
            self.assertTrue(a.inhibit)

    def test_bench_still_stops_on_arm(self):
        a = EvAdapter(0, allow_ev_hgt=True)
        drive_ev(a)
        self.assertIsNone(drive_ev(a, armed=True, start=1.32))
        self.assertIn('armed', a.fault)

    def test_flight_tracking_failure_and_quality_jump_never_resume(self):
        for kind in ('tracking', 'jump', 'stale'):
            a = FlightEvAdapter(0)
            drive_ev(a)
            drive_ev(a, armed=True, start=1.32)
            count = a.publish_count
            if kind == 'tracking':
                a.on_tracking(2, 4_000_000_000, 2.64)
            elif kind == 'stale':
                a.watchdog(3.0)
            else:
                a.on_tracking(1, 3_640_000_000, 2.64)
                a.on_pose(3_640_000_000, 20, 2.64, [1.2, 0, 0], [0, 0, 0, 1])
            self.assertTrue(a.inhibit)
            drive_ev(a, armed=True, start=3.1)
            self.assertEqual(a.publish_count, count)


if __name__ == '__main__':
    unittest.main()
