#!/usr/bin/env python3
"""Kill bench candidate over isolated domain177. Synthetic, NOT PX4 SITL.

Imports the same domain-isolating fixture as existing DDS checks. No Agent,
camera, GPIO or /fmu/in publisher. Never imports/invokes the hardware main.
"""
import fcntl
import json
from pathlib import Path
import time
import uuid

from flight_runtime_check import rclpy, SyntheticPlant, synthetic_aux_params, create_node
from fault_bench_core import KillBenchCore
from handshake_bench_runtime import restored_position


class KillPlant(SyntheticPlant):
    def __init__(self, core, case):
        self.core, self.case = core, case
        self.receipts = []
        super().__init__('position-slot', use_aux=True)

    def heartbeat(self, msg):
        self.receipts.append(('heartbeat', time.monotonic()))
        super().heartbeat(msg)

    def setpoint(self, msg):
        self.receipts.append(('setpoint', time.monotonic()))
        super().setpoint(msg)

    def ev(self, msg):
        self.receipts.append(('ev', time.monotonic()))
        super().ev(msg)

    def command(self, msg):
        self.receipts.append(('command', time.monotonic()))
        super().command(msg)

    def publish(self, name, cls, stamp, **fields):
        now = time.monotonic()
        if name == 'manual_control_setpoint':
            ready = self.core.window_at is not None and now-self.core.window_at > .3
            early = (self.case == 'early-kill' and self.first_heartbeat_at is not None
                     and now-self.first_heartbeat_at > .2)
            self.killed = early or (ready and self.case != 'mode-change')
            if (self.core.terminal_at is not None and now-self.core.terminal_at > 2.3
                    and self.case != 'no-restore'):
                self.killed = False
            fields['aux1'] = 1. if ready and self.case == 'mode-change' else -1.
            fields['aux2'] = 1. if self.killed else -1.
        super().publish(name, cls, stamp, **fields)


def run(case):
    core = KillBenchCore(time.monotonic(), synthetic_aux_params(), 20)
    runtime = create_node(core, isolated=True, exercise=True)
    plant = KillPlant(core, case)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(runtime)
    executor.add_node(plant)
    started = time.monotonic()
    try:
        while time.monotonic()-started < 14:
            executor.spin_once(timeout_sec=.02)
            now = time.monotonic()
            if core.observe_restore(now, restored_position(core, now), runtime.dispatch_guard.fault):
                break
        q = core.qualification(time.monotonic())
        expected_state = 'ABORT' if case == 'mode-change' else 'KILLED'
        real = {topic: runtime.count_publishers(topic) for topic, _ in runtime.get_topic_names_and_types()
                if topic.startswith('/fmu/in/') and runtime.count_publishers(topic)}
        late = [kind for kind, at in plant.receipts
                if core.terminal_at is not None and at > core.terminal_at+.15]
        passed = (q['passed'] == (case == 'kill') and core.handshake.state == expected_state
                  and not real and not late and not plant.commands and not plant.armed
                  and runtime.dispatch_guard.command_count == 0)
        if case == 'kill':
            passed &= all(plant.counts.get(k, 0) > 0 for k in ('ev', 'heartbeat', 'setpoint'))
        return {'case': case, 'passed': bool(passed), 'qualification': q,
                'runtime': runtime.report(), 'plant_counts': dict(plant.counts),
                'real_publishers': real, 'late_outputs': late, 'commands': plant.commands}
    finally:
        executor.shutdown()
        runtime.destroy_node()
        plant.destroy_node()


def main():
    with open('/tmp/robocup_flight_runtime_isolated.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        rclpy.init(args=[])
        results = []
        try:
            for case in ('kill', 'early-kill', 'mode-change', 'no-restore'):
                result = run(case)
                results.append(result)
                print(json.dumps({'case': case, 'passed': result['passed'],
                    'qualification': result['qualification']['passed'],
                    'failure': result['qualification']['failure']}), flush=True)
        finally:
            rclpy.try_shutdown()
    out = Path(__file__).resolve().parents[1]/'evidence'/(
        time.strftime('fault_bench_dds_%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:6])
    out.mkdir(parents=True, exist_ok=False)
    report = {'passed': all(r['passed'] for r in results), 'domain_id': 177,
              'real_hardware': False, 'real_px4_sitl': False, 'flight_approved': False,
              'cases': results}
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'evidence': str(out), 'passed': report['passed']}), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
