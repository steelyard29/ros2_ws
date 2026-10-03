#!/usr/bin/env python3
"""Offline synthetic handshake regression; no ROS, hardware or publishers."""
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import time
import uuid

from disarmed_handshake import DisarmedHandshake, validate_handshake_output
from flight_supervisor import FlightSample


def run_scenario(name):
    controller = DisarmedHandshake()
    outputs = []
    for i in range(140):
        now = i*.05
        sample = FlightSample(status_at=now, local_at=now, flags_at=now, land_at=now,
            rc_at=now, landed=True, nav_state=14 if i > 42 and name != 'mode_timeout' else 2,
            preflight_ok=True, rc_valid=True, ev_ok=True, ev_position=True, ev_height=True,
            ev_yaw=True, baro_height=True, input_ownership_ok=True, xy_valid=True,
            z_valid=True, v_xy_valid=True, v_z_valid=True, x=2., y=-3., z=.7,
            heading=.8, vx=0., vy=0., vz=0., reset_counters=(0, 0, 0, 0, 0))
        mode, kill, switch = 6, 3, now
        if i >= 50:
            faults = {
                'vision_loss': {'ev_ok': False}, 'link_loss': {'status_at': -1},
                'ownership_conflict': {'input_ownership_ok': False},
                'unexpected_arm': {'armed': True}, 'reset': {'reset_counters': (1, 0, 0, 0, 0)},
            }
            sample = replace(sample, **faults.get(name, {}))
            if name == 'takeover':
                mode = 1
            if name == 'kill':
                kill = 1
            if name == 'switch_loss':
                switch = -1
        output = validate_handshake_output(controller.step(now, sample, start=i == 0,
            mode_slot=mode, kill_switch=kill, switch_at=switch, ack_accepted=i > 42))
        outputs.append({'at': now, **asdict(output)})
    expected = {'normal': 'DONE', 'takeover': 'HANDOVER', 'kill': 'KILLED'}.get(name, 'ABORT')
    passed = (controller.state == expected
              and [o['command'] for o in outputs if o['command']] == ['OFFBOARD']
              and all(not o['stream'] or (o['position'] == (2., -3., .7) and o['yaw'] == .8)
                      for o in outputs)
              and not outputs[-1]['stream'])
    return {'scenario': name, 'passed': passed, 'expected': expected,
            'state': controller.state, 'outputs': outputs}


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    scenarios = ['normal', 'mode_timeout', 'vision_loss', 'link_loss', 'takeover',
                 'kill', 'switch_loss', 'ownership_conflict', 'unexpected_arm', 'reset']
    results = [run_scenario(name) for name in scenarios]
    report = {'kind': 'pure_python_synthetic_not_px4_sitl', 'hardware_access': False,
              'real_publishers': 0, 'flight_approved': False,
              'passed': all(r['passed'] for r in results), 'scenarios': results}
    root = Path(__file__).resolve().parents[1]
    out = root/'evidence'/(time.strftime('disarmed_handshake_%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:6])
    out.mkdir(parents=True, exist_ok=False)
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'evidence': str(out), 'passed': report['passed'],
                      'scenarios': [{k: v for k, v in r.items() if k != 'outputs'} for r in results]}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
