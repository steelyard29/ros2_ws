#!/usr/bin/env python3
"""Run isolated SITL safety scenarios: nominal, estop, link-loss, takeoff-timeout.

Host-only, ROS_DOMAIN_ID=175, no serial, no camera, no vision pose, no real FC.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts' / 'sitl_takeoff_land.py'
SCENARIOS = ('nominal', 'estop', 'link-loss', 'takeoff-timeout')


def run_scenario(scenario: str, evidence_root: Path) -> dict:
    evidence = evidence_root / scenario
    evidence.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = os.environ.get('SITL_DOMAIN_ID', '175')
    env['ROS_LOCALHOST_ONLY'] = '1'
    env['PYTHONPATH'] = f"{ROOT / 'scripts'}:{env.get('PYTHONPATH', '')}"
    cmd = [
        sys.executable, str(SCRIPT), '--offline-sitl',
        '--scenario', scenario,
        '--evidence-dir', str(evidence),
    ]
    proc = subprocess.run(cmd, cwd=str(ROOT), env=env, capture_output=True, text=True)
    report_path = evidence / 'report.json'
    report = {}
    if report_path.exists():
        report = json.loads(report_path.read_text())
    else:
        report = {
            'scenario': scenario,
            'passed': False,
            'final_state': 'NO_REPORT',
            'reason': (proc.stderr or proc.stdout or 'no report')[-500:],
        }
    report['returncode'] = proc.returncode
    if proc.returncode != 0 and proc.stderr:
        report['stderr_tail'] = proc.stderr[-800:]
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--offline-sitl', action='store_true', required=True)
    args = parser.parse_args()
    del args
    if os.environ.get('ROS_DOMAIN_ID', '175') in {'0', '174', '176'}:
        print('Refusing ROS_DOMAIN_ID used by camera/nav/default graph.', file=sys.stderr)
        return 2

    evidence_root = ROOT / 'evidence' / time.strftime('sitl_safety_%Y%m%d_%H%M%S')
    evidence_root.mkdir(parents=True, exist_ok=True)
    results = []
    all_passed = True
    for scenario in SCENARIOS:
        print(f'-- scenario {scenario}', flush=True)
        report = run_scenario(scenario, evidence_root)
        results.append({
            'scenario': scenario,
            'passed': bool(report.get('passed')),
            'final_state': report.get('final_state'),
            'reason': report.get('reason'),
            'abort_reason': report.get('abort_reason'),
            'peak_height_m': report.get('peak_height_m'),
            'final_armed': report.get('final_armed'),
            'final_height_m': report.get('final_height_m'),
            'elapsed_s': report.get('elapsed_s'),
            'returncode': report.get('returncode'),
        })
        all_passed = all_passed and bool(report.get('passed'))
        print(json.dumps(results[-1], indent=2), flush=True)

    summary = {
        'test': 'offline_sitl_safety',
        'flight_validation': False,
        'real_aircraft': False,
        'visual_odometry_injected': False,
        'gazebo': False,
        'ros_domain_id': int(os.environ.get('SITL_DOMAIN_ID', '175')),
        'passed': all_passed,
        'scenarios': results,
        'evidence': str(evidence_root),
    }
    (evidence_root / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return 0 if all_passed else 1


if __name__ == '__main__':
    sys.exit(main())
