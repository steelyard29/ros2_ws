#!/usr/bin/env python3
"""Run synthetic scenarios or replay JSONL evidence without ROS/devices."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

from competition_mission import CompetitionMission, Evidence, State


def demo_rows(scenario):
    """Reactive fake adapters. These observations are not sensor evidence."""
    m = CompetitionMission()
    for tick in range(6101):
        now = tick / 10
        active = tick > 0 and scenario != 'liftoff-timeout'
        landed = not active or m.state in {State.LAND, State.ABORT}
        target = f'synthetic-target-{len(m.released)}'
        evidence = Evidence(
            at=now, height_m=1.2 if active and not landed else 0,
            landed=landed, armed=active and not landed, preflight_ok=True,
            localization_ok=not (scenario == 'vision-loss' and now >= 12),
            link_ok=not (scenario == 'link-loss' and now >= 12),
            control_owned=True, stable=True,
            manual_override=scenario == 'manual-takeover' and now >= 12,
            target_id=target if m.state in {State.SEARCH, State.ALIGN} else '',
            target_class=('red_cross', 'armored_vehicle', 'bridge')[min(len(m.released), 2)],
            target_at=now, target_confidence=.95,
            alignment_ok=True, completion_id=m.request_id,
            release_ok=m.state == State.RELEASE and scenario != 'release-timeout',
            navigation_ok=m.state == State.RETURN)
        row = {'now': now, 'evidence': asdict(evidence), 'start': tick == 0}
        m.step(now, evidence, start=tick == 0)
        yield row
        if m.state in m.TERMINAL:
            return


def replay(rows):
    m = CompetitionMission()
    releases = []
    count = 0
    for row in rows:
        if set(row) - {'now', 'evidence', 'start'}:
            raise ValueError('unknown replay fields')
        if type(row.get('start', False)) is not bool:
            raise ValueError('start must be a boolean')
        e = Evidence(**row['evidence'])
        # JSON strings such as "false" must never be interpreted as true.
        for name, field in Evidence.__dataclass_fields__.items():
            value = getattr(e, name)
            if field.type is bool and type(value) is not bool:
                raise ValueError(f'{name} must be a boolean')
            if field.type is float and (type(value) not in (int, float)):
                raise ValueError(f'{name} must be numeric')
            if field.type is str and not isinstance(value, str):
                raise ValueError(f'{name} must be a string')
        intention = m.step(row['now'], e, start=row.get('start', False))
        if intention.kind == 'RELEASE_ONCE':
            releases.append({'at': row['now'], **asdict(intention)})
        count += 1
    return {
        'offline_only': True, 'synthetic_adapters': False,
        'flight_validation': False, 'real_commands_sent': 0,
        'final_state': m.state.value, 'reason': m.reason,
        'complete': m.state in m.TERMINAL,
        'takeoff_condition_observed': m.takeoff_observed,
        'avoidance_eligible': m.avoidance_eligible,
        'confirmed_releases': m.released, 'uncertain_slots': m.uncertain_slots,
        'release_intentions': releases, 'states': m.history, 'samples': count,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--demo', choices=['nominal', 'vision-loss', 'link-loss',
                                         'manual-takeover', 'release-timeout',
                                         'liftoff-timeout'])
    group.add_argument('--input', type=Path, help='JSONL monotonic snapshots')
    args = parser.parse_args()
    try:
        if args.demo:
            result = replay(demo_rows(args.demo))
            result['synthetic_adapters'] = True
            result['scenario'] = args.demo
        else:
            with args.input.open() as stream:
                result = replay(json.loads(line) for line in stream if line.strip())
    except (OSError, TypeError, ValueError, KeyError) as exc:
        print(f'Invalid replay: {exc}', file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['complete'] else 1


if __name__ == '__main__':
    sys.exit(main())
