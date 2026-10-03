#!/usr/bin/env python3
"""Offline pre/post calibration export review. No ROS/devices or parameter writes.

Without --baseline, preserves the supplied full export as new evidence. With
--baseline, reports all changes, separating calibration values from changes
needing explicit review. Never certifies that calibration actually happened.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import time
import uuid
from flight_readiness import read_params


CAL_VALUE = re.compile(r'(CAL_(ACC|GYRO)\d+_(XOFF|YOFF|ZOFF|XSCALE|YSCALE|ZSCALE)|SENS_BOARD_[XYZ]_OFF)')


def compare(before, after):
    changes = []
    for name in sorted(before.keys() | after.keys()):
        if before.get(name) != after.get(name):
            changes.append({'name': name, 'before': before.get(name), 'after': after.get(name),
                'calibration_value_only': bool(CAL_VALUE.fullmatch(name)) and name in before and name in after})
    return {'changes': changes,
            'other_changes_requiring_review': [x for x in changes if not x['calibration_value_only']],
            'calibration_performed_verified': False, 'flight_approved': False,
            'note': 'Changed values do not prove valid calibration; unchanged values do not prove it was skipped.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--params', type=Path, required=True)
    parser.add_argument('--baseline', type=Path)
    args = parser.parse_args()
    raw = args.params.read_bytes()
    current = read_params(args.params)
    if raw != args.params.read_bytes():
        parser.error('export changed while reading')
    out = Path(__file__).resolve().parents[1]/'evidence'/(
        time.strftime('calibration_review_%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:6])
    baseline = read_params(args.baseline) if args.baseline else None
    result = compare(baseline, current) if baseline is not None else {
        'mode': 'baseline_snapshot', 'calibration_performed_verified': False, 'flight_approved': False}
    result.update(parameter_sha256=hashlib.sha256(raw).hexdigest(), input_path=str(args.params),
                  baseline_path=str(args.baseline) if args.baseline else None)
    out.mkdir(parents=True, exist_ok=False)
    (out/'params.params.txt').write_bytes(raw)
    (out/'report.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'evidence': str(out), **result}, indent=2))
    return 1 if result.get('other_changes_requiring_review') else 0


if __name__ == '__main__':
    raise SystemExit(main())
