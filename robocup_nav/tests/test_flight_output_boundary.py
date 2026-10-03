from dataclasses import replace
import fcntl
from pathlib import Path
import subprocess
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from flight_output_boundary import routes, ownership_ok, BenchEvGate, OUTPUTS, EV_TOPIC
from flight_runtime import create_node
from flight_runtime_core import FlightRuntimeCore
from test_flight_runtime import RuntimeRig
from test_aux_switch_decoder import params


class BoundaryTests(unittest.TestCase):
    def test_coordinated_bench_rejects_existing_runtime_lock_before_ros(self):
        with open('/tmp/robocup_flight_runtime_shadow.lock', 'a') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                self.skipTest('a real runtime currently holds the shared lock')
            result = subprocess.run(
                [sys.executable, str(Path(__file__).resolve().parents[1]/'scripts/flight_bench_check.py'), '--help'],
                capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 2)
            self.assertIn('owns the lock', result.stderr)

    def rig(self):
        r = RuntimeRig(aux_params=params())
        for _ in range(40):
            r.tick(mode=1, start=False)
        return r

    def test_only_ev_can_reach_fmu_and_no_arbitrary_mode(self):
        for bench in (False, True):
            mapping = routes(bench_ev=bench)
            real = [v for v in mapping.values() if v.startswith('/fmu/in/')]
            self.assertEqual(real, [EV_TOPIC] if bench else [])
            for name in ('vehicle_command', 'trajectory_setpoint', 'offboard_control_mode'):
                self.assertTrue(mapping[name].startswith('/robocup/flight_runtime_shadow/'))
        with self.assertRaises(ValueError):
            routes(isolated=True, bench_ev=True)

    def test_audit_rejects_extra_real_inputs_duplicate_or_missing_ev(self):
        own = dict.fromkeys(OUTPUTS, 1)
        self.assertTrue(ownership_ok(own, {}, bench_ev=False))
        self.assertTrue(ownership_ok(own, {EV_TOPIC: 1}, bench_ev=True))
        for real in ({}, {EV_TOPIC: 2}, {EV_TOPIC: 1, '/fmu/in/vehicle_command': 1}):
            self.assertFalse(ownership_ok(own, real, bench_ev=True))
        self.assertFalse(ownership_ok({**own, 'vehicle_command': 2}, {EV_TOPIC: 1}, bench_ev=True))
        self.assertFalse(ownership_ok(own, {EV_TOPIC: 1}))

    def test_no_synthetic_or_exercise_bench_before_ros_creation(self):
        for isolated, exercise, aux in ((True, False, True), (False, True, True), (False, False, False)):
            c = FlightRuntimeCore(0, aux_params=params() if aux else None)
            with self.assertRaises(ValueError):
                create_node(c, isolated=isolated, exercise=exercise, bench_ev=True)

    def test_waits_for_initial_telemetry_without_fake_readiness(self):
        c = FlightRuntimeCore(0, aux_params=params())
        g = BenchEvGate()
        self.assertFalse(g.check(c, 0))
        self.assertIsNone(g.fault)
        self.assertFalse(c.ev.inhibit)

    def test_ground_allows_ev_without_circular_preflight_dependency(self):
        r = self.rig()
        r.c.sample = replace(r.c.sample, preflight_ok=False, ev_position=False,
                             ev_height=False, ev_yaw=False, xy_valid=False)
        g = BenchEvGate()
        self.assertTrue(g.check(r.c, r.now))
        g.sent_ev()
        self.assertEqual(g.sent, 1)

    def test_arm_kill_stale_rc_land_flags_and_mode_stop_latched(self):
        changes = ({'armed': True}, {'landed': False}, {'status_at': 0},
                   {'land_at': -1}, {'rc_at': 0}, {'rc_valid': False},
                   {'flags_at': -1}, {'baro_height': False}, {'range_height': True},
                   {'ev_velocity': True}, {'failsafe': True}, {'nav_state': 14})
        for change in changes:
            with self.subTest(change=change):
                r = self.rig()
                good = r.c.sample
                g = BenchEvGate()
                self.assertTrue(g.check(r.c, r.now))
                g.sent_ev()
                r.c.sample = replace(good, **change)
                self.assertFalse(g.check(r.c, r.now))
                r.c.sample = good
                self.assertFalse(g.check(r.c, r.now))
                self.assertTrue(r.c.ev.inhibit)
        for field, value in (('mode_slot', 6), ('kill_latched', True), ('switch_at', 0),
                             ('ownership_at', 0), ('ownership_fault', True)):
            r = self.rig()
            g = BenchEvGate()
            g.sent_ev()
            setattr(r.c, field, value)
            self.assertFalse(g.check(r.c, r.now), field)

    def test_battery_warning_alone_does_not_stop(self):
        r = self.rig()
        r.c.battery_warning = 2
        self.assertTrue(BenchEvGate().check(r.c, r.now))

    def test_cli_requires_prop_off_and_rejects_exercise(self):
        script = Path(__file__).resolve().parents[1]/'scripts/flight_bench_runtime.py'
        for args in ([], ['--prop-off', '--isolated', '--aux-params', '/not-used'],
                     ['--prop-off', '--exercise-controller', '--aux-params', '/not-used']):
            result = subprocess.run([sys.executable, str(script), *args],
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 2, result.stderr)


if __name__ == '__main__':
    unittest.main()
