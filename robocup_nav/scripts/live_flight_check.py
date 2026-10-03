#!/usr/bin/env python3
"""Exercise the production live core/serializer on domain177 synthetic routes.

Never validates a live permit, imports the live entrypoint, or creates /fmu/in
publishers. Not PX4 SITL or a hardware/flight acceptance test.
"""
import argparse
import fcntl
import json
import math
from pathlib import Path
import time
import uuid

from flight_runtime_check import SyntheticPlant, synthetic_aux_params, rclpy
from flight_runtime import create_node
from live_flight_core import LiveFlightCore

SCENARIOS = ('nominal', 'vision-loss', 'link-loss', 'manual-takeover', 'kill',
    'ack-denied', 'ack-without-state', 'missing-mode-ack', 'missing-arm-ack',
    'wrong-ack-target', 'ownership-conflict', 'battery-warning', 'switch-loss',
    'interrupt', 'dispatch-error')


class Plant(SyntheticPlant):
    def __init__(self, scenario, core, runtime):
        self.core = core
        self.runtime = runtime
        self.operator_position = True
        self.dispatch_fault_injected = False
        super().__init__(scenario, use_aux=True)
        self.initial_z = self.z
        self.max_rise = 0.

    def tick(self):
        if self.core.ready_at is not None and time.monotonic()-self.core.ready_at > .3:
            self.operator_position = False
        super().tick()
        self.max_rise = max(self.max_rise, self.initial_z-self.z)
        if self.scenario == 'interrupt' and self.injected_at is not None:
            self.core.abort_requested = lambda: True
        if self.scenario == 'dispatch-error' and self.injected_at is not None and not self.dispatch_fault_injected:
            class FailedPublisher:
                def publish(self, message):
                    raise RuntimeError('injected synthetic publisher failure')
            self.runtime.pubs['trajectory_setpoint'] = FailedPublisher()
            self.dispatch_fault_injected = True

    def filter_ack(self, ack):
        if self.scenario == 'missing-mode-ack' and ack.command == 176:
            return False
        if self.scenario == 'missing-arm-ack' and ack.command == 400:
            return False
        if self.scenario == 'wrong-ack-target':
            ack.target_component = 1
        return True


def run(scenario, height=.5):
    core = LiveFlightCore(time.monotonic(), synthetic_aux_params(), height=height)
    runtime = create_node(core, isolated=True, exercise=True)
    plant = Plant(scenario, core, runtime)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(runtime)
    executor.add_node(plant)
    start = time.monotonic()
    error = None
    silence_after_exception = False
    try:
        while time.monotonic()-start < 30:
            executor.spin_once(timeout_sec=.02)
            if core.controller.state in core.controller.TERMINAL:
                deadline = time.monotonic()+.4
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.01)
                break
            if scenario == 'link-loss' and core.controller.state == 'ABORT' and not plant.armed:
                break
    except Exception as exc:
        error = str(exc)
        if scenario == 'dispatch-error' and core.closed:
            stopped_counts = dict(runtime.counts)
            drain_until = time.monotonic()+.5
            while time.monotonic() < drain_until:
                executor.spin_once(timeout_sec=.01)
            silence_after_exception = dict(runtime.counts) == stopped_counts
    finally:
        report = runtime.report()
        expected = {'nominal': 'DONE', 'battery-warning': 'DONE',
            'vision-loss': 'STOPPED', 'switch-loss': 'STOPPED', 'link-loss': 'ABORT',
            'manual-takeover': 'HANDOVER', 'ownership-conflict': 'HANDOVER', 'kill': 'KILLED',
            'ack-denied': 'STOPPED', 'ack-without-state': 'STOPPED', 'missing-mode-ack': 'STOPPED',
            'missing-arm-ack': 'STOPPED', 'wrong-ack-target': 'STOPPED',
            'interrupt': 'STOPPED', 'dispatch-error': 'TAKEOFF'}[scenario]
        passed = error is None and report['state'] == expected
        if scenario == 'dispatch-error':
            passed = (error == 'injected synthetic publisher failure' and core.closed
                      and silence_after_exception and report['ev_stopped'])
        passed &= report['real_publishers_created'] == 0 and not report['live_routes_enabled']
        passed &= all(c['source_component'] == 191 for c in plant.commands)
        if scenario in ('ack-denied', 'ack-without-state', 'missing-mode-ack', 'wrong-ack-target'):
            passed &= not any(c['command'] == 400 for c in plant.commands)
        if scenario == 'missing-arm-ack':
            passed &= not any(s['state'] in ('TAKEOFF', 'HOVER') for s in core.controller.history)
        if scenario in ('kill', 'manual-takeover', 'ownership-conflict'):
            passed &= not any(c['at'] > plant.injected_at+.1 for c in plant.commands)
        if scenario == 'nominal':
            passed &= {'OFFBOARD', 'ARM', 'LAND'} <= core.accepted_commands and core.landing_seen
            passed &= plant.max_rise >= height-1e-6
        core.close('isolated test complete')
        executor.shutdown()
        runtime.destroy_node()
        plant.destroy_node()
    return {'scenario': scenario, 'passed': bool(passed), 'expected': expected, 'error': error,
            'requested_height_m': height, 'synthetic_max_rise_m': plant.max_rise,
            'runtime': report, 'commands': plant.commands, 'plant_counts': dict(plant.counts),
            'silence_after_exception': silence_after_exception,
            'synthetic_plant': True, 'flight_validation': False, 'real_px4_sitl': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=SCENARIOS, action='append')
    parser.add_argument('--height', type=float, default=.5, help='Synthetic relative rise in metres, .2..1.3')
    args = parser.parse_args()
    if not math.isfinite(args.height) or not .2 <= args.height <= 1.3:
        parser.error('height must be finite and within .2..1.3 m')
    from flight_release import software_sha
    tested_software = software_sha()
    with open('/tmp/robocup_flight_runtime_isolated.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        rclpy.init(args=[])
        results = []
        try:
            for scenario in args.scenario or SCENARIOS:
                result = run(scenario, args.height)
                results.append(result)
                print(json.dumps({'scenario': scenario, 'passed': result['passed'],
                    'state': result['runtime']['state'], 'error': result['error']}), flush=True)
        finally:
            rclpy.try_shutdown()
    out = Path(__file__).resolve().parents[1]/'evidence'/(
        time.strftime('live_flight_check_%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:6])
    out.mkdir(parents=True, exist_ok=False)
    unchanged = software_sha() == tested_software
    passed = all(r['passed'] for r in results) and unchanged
    (out/'report.json').write_text(json.dumps({'passed': passed, 'scenarios': results,
        'requested_height_m': args.height,
        'software_sha256': tested_software, 'software_unchanged_during_test': unchanged,
        'flight_validation': False, 'real_px4_sitl': False}, indent=2)+'\n')
    print(json.dumps({'evidence': str(out), 'passed': passed}), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
