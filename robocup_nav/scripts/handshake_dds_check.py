#!/usr/bin/env python3
"""Disarmed SHADOW handshake over isolated DDS domain 177. Not PX4 SITL.

Reuses the actual runtime subscriptions, serializers and source checks. Never
starts an Agent, camera, GPIO program or real-input publisher.
"""
import argparse
import fcntl
import json
from pathlib import Path
import time
import uuid

# This module sets domain 177 / localhost before importing rclpy.
from flight_runtime_check import (rclpy, SyntheticPlant, synthetic_aux_params,
                                  create_node, VehicleCommandAck)
from handshake_runtime_core import HandshakeRuntimeCore

SCENARIOS = ('nominal', 'ack-denied', 'ack-without-state', 'state-without-ack',
             'vision-loss', 'link-loss', 'manual-takeover', 'kill',
             'ownership-conflict', 'switch-loss', 'frozen-switch-sample',
             'source-reset', 'frozen-local', 'unexpected-arm', 'battery-warning',
             'missing-switches', 'position-slot', 'aux-non-rc', 'stale-ack',
             'wrong-target-ack', 'temporary-reject', 'in-progress-only', 'low-rate-telemetry')
STAGED_EXTRA = ('status-before-switch', 'early-switch', 'early-offboard', 'offboard-at-start', 'no-switch', 'gcs-loss')


class HandshakePlant(SyntheticPlant):
    def __init__(self, scenario, staged=False):
        self.staged = staged
        self.last_local_stamp = None
        self.hold_errors = []
        self.output_receipts = []
        self.published_at = {}
        super().__init__(scenario, use_aux=True)

    def heartbeat(self, m):
        self.output_receipts.append(('heartbeat', time.monotonic()))
        super().heartbeat(m)

    def setpoint(self, m):
        self.output_receipts.append(('setpoint', time.monotonic()))
        if abs(m.position[2]-.7) > 1e-6:
            self.hold_errors.append('handshake changed ground Z')
        super().setpoint(m)

    def ev(self, m):
        self.output_receipts.append(('ev', time.monotonic()))
        super().ev(m)

    def command(self, m):
        self.output_receipts.append(('command', time.monotonic()))
        if m.command != 176 or m.param1 != 1 or m.param2 != 6:
            self.hold_errors.append('forbidden command')
        if self.staged:
            if m.source_system != 1 or m.source_component != 191:
                self.hold_errors.append('wrong source identity')
            self.commands.append({'at': time.monotonic(), 'command': int(m.command)})
            if self.scenario != 'ack-denied' and self.scenario != 'ack-without-state':
                self.mode = 14
            if self.scenario == 'state-without-ack':
                return
            ack = VehicleCommandAck()
            ack.timestamp = (m.timestamp-1 if self.scenario == 'stale-ack'
                             else self.get_clock().now().nanoseconds//1000)
            ack.command = m.command
            ack.result = {'ack-denied': 2, 'temporary-reject': 1, 'in-progress-only': 5}.get(self.scenario, 0)
            ack.target_system = 1
            ack.target_component = 1 if self.scenario == 'wrong-target-ack' else 191
            self.pubs['vehicle_command_ack'].publish(ack)
            return
        if self.scenario == 'state-without-ack' and m.command == 176:
            self.commands.append({'at': time.monotonic(), 'command': int(m.command)})
            self.mode = 14
            return
        if self.scenario in ('stale-ack', 'wrong-target-ack', 'temporary-reject', 'in-progress-only'):
            self.commands.append({'at': time.monotonic(), 'command': int(m.command)})
            self.mode = 14
            ack = VehicleCommandAck()
            ack.timestamp = (m.timestamp-1 if self.scenario == 'stale-ack'
                             else self.get_clock().now().nanoseconds//1000)
            ack.command = m.command
            ack.result = {'temporary-reject': 1, 'in-progress-only': 5}.get(self.scenario, 0)
            ack.target_system = ack.target_component = 2 if self.scenario == 'wrong-target-ack' else 1
            self.pubs['vehicle_command_ack'].publish(ack)
            return
        super().command(m)

    def publish(self, name, cls, stamp, **fields):
        if self.scenario == 'gcs-loss' and name == 'vehicle_status_v1':
            fields['gcs_connection_lost'] = self.injected_at is not None
        if self.staged and name == 'manual_control_setpoint':
            elapsed = (time.monotonic()-self.first_heartbeat_at
                       if self.first_heartbeat_at is not None else -1)
            edge_at = .3 if self.scenario == 'early-switch' else (2.5 if self.scenario == 'status-before-switch' else 2.3)
            fields['aux1'] = 1. if elapsed >= edge_at and self.scenario != 'no-switch' else -1.
            if self.scenario == 'manual-takeover' and self.injected_at is not None:
                fields['aux1'] = -1.
        # Match observed FC rates instead of relying solely on 50 Hz fixtures.
        if self.scenario == 'low-rate-telemetry':
            period = {'vehicle_status_v1': .5, 'estimator_status_flags': 1.,
                      'vehicle_land_detected': 1.}.get(name, 0.)
            now = time.monotonic()
            if now-self.published_at.get(name, -1e9) < period:
                return
            self.published_at[name] = now
        if name == 'vehicle_local_position':
            if self.injected_at is not None and self.last_local_stamp is not None:
                if self.scenario == 'source-reset':
                    stamp = self.last_local_stamp-1000
                elif self.scenario == 'frozen-local':
                    stamp = self.last_local_stamp
            else:
                self.last_local_stamp = stamp
        super().publish(name, cls, stamp, **fields)

    def tick(self):
        # Inject in prestream: this controller intentionally never arms.
        faults = {'vision-loss', 'link-loss', 'manual-takeover', 'kill',
                  'ownership-conflict', 'switch-loss', 'frozen-switch-sample',
                  'source-reset', 'frozen-local', 'unexpected-arm', 'aux-non-rc', 'gcs-loss'}
        if (self.scenario in faults and self.first_heartbeat_at is not None
                and self.injected_at is None and time.monotonic()-self.first_heartbeat_at > (2.7 if self.staged else .7)):
            self.injected_at = time.monotonic()
            if self.scenario == 'unexpected-arm':
                self.armed = True  # Synthetic telemetry only. No ARM command.
        if self.staged:
            elapsed = time.monotonic()-self.first_heartbeat_at if self.first_heartbeat_at is not None else -1
            if (self.scenario == 'offboard-at-start' or
                    (self.scenario == 'early-offboard' and elapsed > .3) or
                    (self.scenario == 'status-before-switch' and elapsed > 2.3)):
                self.mode = 14
        super().tick()


def run(scenario, staged=False):
    core = HandshakeRuntimeCore(time.monotonic(), aux_params=synthetic_aux_params(), staged=staged, require_gcs=staged)
    runtime = create_node(core, isolated=True, exercise=True)
    plant = HandshakePlant(scenario, staged=staged)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(runtime)
    executor.add_node(plant)
    started = time.monotonic()
    stopped = None
    try:
        while time.monotonic()-started < (28 if scenario == 'no-switch' else 12):
            executor.spin_once(timeout_sec=.02)
            if core.handshake.state in core.handshake.TERMINAL:
                stopped = time.monotonic()
                # Keep callbacks running to expose accidental output resumption.
                until = stopped+.6
                while time.monotonic() < until:
                    executor.spin_once(timeout_sec=.01)
                break
            if scenario in ('missing-switches', 'position-slot') and time.monotonic()-started > 4:
                break
        expected = {'nominal': 'DONE', 'battery-warning': 'DONE', 'low-rate-telemetry': 'DONE',
                    'status-before-switch': 'DONE', 'early-switch': 'HANDOVER',
                    'manual-takeover': 'HANDOVER', 'kill': 'KILLED',
                    'missing-switches': 'WAIT', 'position-slot': 'WAIT'}.get(scenario, 'ABORT')
        real = {t: runtime.count_publishers(t) for t, _ in runtime.get_topic_names_and_types()
                if t.startswith('/fmu/in/') and runtime.count_publishers(t)}
        after_stop = [kind for kind, at in plant.output_receipts if stopped is not None and at > stopped+.15]
        passed = (core.handshake.state == expected and not real and not plant.hold_errors
                  and len(plant.commands) <= 1 and all(c['command'] == 176 for c in plant.commands)
                  and not after_stop)
        if expected == 'DONE':
            passed &= core.ack_accepted and plant.mode == 14 and not plant.armed
        if scenario != 'unexpected-arm':
            passed &= not plant.armed
        return {'scenario': scenario, 'passed': bool(passed), 'expected': expected,
                'runtime': runtime.report(), 'plant_received': dict(plant.counts),
                'plant_commands': plant.commands, 'hold_errors': plant.hold_errors,
                'output_after_terminal_grace': after_stop, 'real_input_publishers': real,
                'elapsed_s': time.monotonic()-started,
                'synthetic_armed': plant.armed, 'fault_injected_at': plant.injected_at}
    finally:
        executor.shutdown()
        runtime.destroy_node()
        plant.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=SCENARIOS+STAGED_EXTRA, action='append')
    parser.add_argument('--staged', action='store_true', help='test two-stage bench candidate, still isolated/shadow ONLY')
    args = parser.parse_args()
    with open('/tmp/robocup_flight_runtime_isolated.lock', 'a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error('another isolated runtime holds the lock')
        rclpy.init(args=[])
        results = []
        try:
            default = tuple(s for s in SCENARIOS if s != 'position-slot')+STAGED_EXTRA if args.staged else SCENARIOS
            for scenario in args.scenario or default:
                result = run(scenario, staged=args.staged)
                results.append(result)
                print(json.dumps({'scenario': scenario, 'passed': result['passed'],
                                  'state': result['runtime']['state'],
                                  'reason': result['runtime']['reason']}), flush=True)
        finally:
            rclpy.try_shutdown()
    root = Path(__file__).resolve().parents[1]
    out = root/'evidence'/(time.strftime('handshake_dds_%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:6])
    out.mkdir(parents=True, exist_ok=False)
    report = {'passed': all(r['passed'] for r in results), 'domain_id': 177,
              'staged': args.staged,
              'real_px4_sitl': False, 'real_hardware': False, 'flight_approved': False,
              'scenarios': results}
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'evidence': str(out), 'passed': report['passed']}), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
