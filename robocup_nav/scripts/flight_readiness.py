#!/usr/bin/env python3
"""Read-only review of an operator-supplied PX4 1.16.2 export and platform YAML.

Never connects to PX4 and never changes a parameter or verification flag.
Export consistency is not evidence of switch operation or failsafe behavior.
"""
import argparse
import json
import math
from pathlib import Path
import yaml


def read_params(path):
    values = {}
    for line in Path(path).read_text().splitlines():
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        fields = line.split()
        if len(fields) != 5:
            raise ValueError('expected QGC five-column parameter export')
        name = fields[2]
        value = float(fields[3])
        if not math.isfinite(value) or name in values:
            raise ValueError('invalid or duplicate parameter: '+name)
        values[name] = value
    return values


def review(params, platform, payload='loaded'):
    if payload not in ('none', 'loaded'):
        raise ValueError('payload must be none or loaded')
    checks = []
    def add(name, ok, actual, required):
        checks.append({'check': name, 'passed': bool(ok), 'actual': actual, 'required': required})
    expected = {'EKF2_EV_CTRL': 11, 'EKF2_HGT_REF': 3, 'EKF2_EV_DELAY': 0,
                'EKF2_EV_POS_X': 0, 'EKF2_EV_POS_Y': 0, 'EKF2_EV_POS_Z': 0,
                'EKF2_EV_QMIN': 50, 'EKF2_BARO_CTRL': 1, 'EKF2_RNG_CTRL': 0,
                'COM_OBL_RC_ACT': 4, 'NAV_RCL_ACT': 3,
                'RC_MAP_FLTMODE': 6, 'RC_MAP_KILL_SW': 5, 'RC_MAP_OFFB_SW': 0,
                'COM_FLTMODE1': 2, 'COM_FLTMODE2': 2, 'COM_FLTMODE3': 2,
                'COM_FLTMODE4': 2, 'COM_FLTMODE5': 2, 'COM_FLTMODE6': 7,
                'MPC_LAND_SPEED': 0.6}
    for name, value in expected.items():
        actual = params.get(name)
        add(name, type(actual) in (int, float) and math.isfinite(actual)
            and math.isclose(actual, value, rel_tol=0., abs_tol=1e-6), actual, value)
    override = params.get('COM_RC_OVERRIDE')
    add('Offboard stick override', override in (2, 3), override, 'COM_RC_OVERRIDE bit 1 enabled')
    exceptions = params.get('COM_RCL_EXCEPT')
    add('RC loss not exempt in Offboard', exceptions is not None and
        float(exceptions).is_integer() and not (int(exceptions) & 4), exceptions, 'COM_RCL_EXCEPT bit 2 clear')
    timeout = params.get('COM_OF_LOSS_T')
    add('Offboard loss timeout', timeout is not None and 0 < timeout <= 1, timeout, '0 < seconds <= 1')
    mode, kill = params.get('RC_MAP_FLTMODE'), params.get('RC_MAP_KILL_SW')
    add('Dedicated mode and kill channels', mode is not None and kill is not None
        and float(mode).is_integer() and float(kill).is_integer() and 1 <= mode <= 18
        and 1 <= kill <= 18 and mode != kill,
        {'mode': mode, 'kill': kill}, 'distinct mapped channels; actual operation still needs test')
    offboard = params.get('RC_MAP_OFFB_SW')
    add('No duplicate Offboard switch mapping', offboard is not None and
        (offboard == 0 or (float(offboard).is_integer() and 1 <= offboard <= 18
                           and offboard not in (mode, kill))), offboard,
        'Unassigned (0), or a separate channel from mode and kill')
    for name in ('geometry_verified', 'sensor_extrinsics_verified',
                 'imu_calibration_verified', 'px4_frame_alignment_verified'):
        add(name, platform.get(name) is True, platform.get(name), 'measured and reviewed evidence')
    lower = 'below_center_without_payload_m' if payload == 'none' else 'below_center_with_payload_m'
    for name in ('length_m', 'width_m', 'above_center_m', lower):
        value = platform.get(name)
        add(name, type(value) in (int, float) and math.isfinite(value) and value > 0,
            value, 'positive measured metres')
    return {'configuration_consistent': all(c['passed'] for c in checks), 'payload': payload,
            'battery_policy': {'COM_LOW_BAT_ACT': params.get('COM_LOW_BAT_ACT'),
                               'action': 'operator-selected warning only; not a mandatory Land change'},
            'flight_approved': False, 'checks': checks,
            'unverified_physical_items': ['mode/kill switch operation', 'RC and Offboard loss behavior',
                                          'sensor extrinsics', payload+' payload airframe envelope'],
            'note': 'Checks are first-flight candidate policy, not universal PX4 requirements.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--params', required=True, type=Path)
    parser.add_argument('--payload', choices=('none', 'loaded'), default='loaded')
    parser.add_argument('--platform', type=Path,
                        default=Path(__file__).resolve().parents[1]/'config/platform.yaml')
    args = parser.parse_args()
    report = review(read_params(args.params), yaml.safe_load(args.platform.read_text()), args.payload)
    print(json.dumps({'parameter_export': str(args.params),
                      'source_firmware_contract': 'PX4 v1.16.2', **report}, indent=2))
    return 0 if report['configuration_consistent'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
