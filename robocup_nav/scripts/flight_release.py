"""Offline, fail-closed release records. Operator evidence is not auto-certified.

Never writes PX4 parameters, enables platform flags, or creates ROS nodes.
"""
import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import time
import uuid

import yaml

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE_KEYS = ('calibration', 'post_calibration_ev', 'geometry', 'extrinsics',
                 'frame_alignment', 'fault_acceptance', 'site_and_battery')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def software_sha(root=None):
    root = ROOT if root is None else root
    paths = sorted([*root.glob('scripts/*.py'), *root.glob('config/*.yaml'),
                    *root.glob('launch/*.py'), root/'run.sh'])
    digest = hashlib.sha256()
    for path in paths:
        digest.update(str(path.relative_to(root)).encode()+b'\0'+path.read_bytes()+b'\0')
    return digest.hexdigest()


def template():
    return {'schema': 1, 'session_id': str(uuid.uuid4()), 'operator': '',
        'issued_at_unix': None, 'expires_at_unix': None,
        'payload': 'none', 'height_m': .5, 'hover_s': 3.,
        'parameter_sha256': '', 'platform_sha256': '', 'software_sha256': '',
        'operator_confirms_current_board_export': False,
        'operator_confirms_component_191_exclusive': False,
        'operator_confirms_px4_v1_16_2_fmu_v6c': False,
        'operator_accepts_warning_only_battery': False,
        'evidence': {name: {'path': '', 'sha256': '', 'reviewed': False, 'performed_at_unix': None}
                     for name in EVIDENCE_KEYS}}


@dataclass(frozen=True)
class FlightPermit:
    session_id: str
    height: float
    hover: float
    expires: float
    issued: float
    release: Path
    params: Path
    platform: Path
    release_hash: str
    params_hash: str
    platform_hash: str
    software_hash: str
    evidence_files: tuple

    def unchanged(self):
        if (sha(self.release) != self.release_hash or sha(self.params) != self.params_hash
                or sha(self.platform) != self.platform_hash or software_sha() != self.software_hash
                or any(sha(path) != digest for path, digest in self.evidence_files)):
            raise ValueError('release, evidence, parameters, platform or software changed')

    def before_arm(self, now=None):
        now = time.time() if now is None else now
        if not math.isfinite(now) or not self.issued <= now <= self.expires:
            raise ValueError('release expired before ARM')
        self.unchanged()

    def consume(self, directory):
        """Atomic one-use record, intentionally retained even after a failed run."""
        self.before_arm()
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        marker = directory/(self.session_id+'.json')
        with marker.open('x') as stream:
            json.dump({'session_id': self.session_id, 'release_sha256': self.release_hash,
                       'consumed_at_unix': time.time()}, stream)
        return marker


def validate(release_path, params_path, platform_path, now=None):
    from flight_readiness import read_params, review
    from aux_switch_decoder import validate_contract
    from handshake_dispatch import validate_target
    now = time.time() if now is None else now
    release_path, params_path, platform_path = map(lambda p: Path(p).resolve(),
                                                 (release_path, params_path, platform_path))
    if platform_path != (ROOT/'config/platform.yaml').resolve():
        raise ValueError('live release must bind the actual platform.yaml used by the camera stack')
    hashes = sha(release_path), sha(params_path), sha(platform_path), software_sha()
    data = json.loads(release_path.read_text())
    platform = yaml.safe_load(platform_path.read_text())
    if not isinstance(data, dict) or not isinstance(platform, dict):
        raise ValueError('release and platform must be objects')
    if type(data.get('schema')) is not int or data.get('schema') != 1 or data.get('payload') != 'none':
        raise ValueError('only schema 1, no-payload first flight is supported')
    session_id = str(uuid.UUID(data['session_id']))
    if session_id != data['session_id'] or not isinstance(data.get('operator'), str) or not data['operator'].strip():
        raise ValueError('canonical session UUID and named operator required')
    issued, expires = data.get('issued_at_unix'), data.get('expires_at_unix')
    if (any(type(v) not in (int, float) or not math.isfinite(v) for v in (issued, expires, now))
            or not issued <= now <= expires or not 0 < expires-issued <= 900):
        raise ValueError('release must be current, with lifetime <=900 seconds')
    for key in ('operator_confirms_current_board_export',
                'operator_confirms_component_191_exclusive', 'operator_accepts_warning_only_battery',
                'operator_confirms_px4_v1_16_2_fmu_v6c'):
        if data.get(key) is not True:
            raise ValueError('operator confirmation required: '+key)
    height, hover = data.get('height_m'), data.get('hover_s')
    if (type(height) not in (int, float) or not math.isfinite(height) or not .2 <= height <= 1.3
            or type(hover) not in (int, float) or not math.isfinite(hover) or not 1 <= hover <= 5):
        raise ValueError('flight limits: relative rise .2..1.3 m, hold 1..5 s')
    if platform.get('live_flight_enabled') is not True:
        raise ValueError('live_flight_enabled is not explicitly true; no flight authorized')
    params = read_params(params_path)
    validate_contract(params)
    validate_target(params)
    bad = [c['check'] for c in review(params, platform, 'none')['checks'] if not c['passed']]
    # Preserve reviewed warning-only and loss/takeover timing contracts.
    for name, expected in {'COM_LOW_BAT_ACT': 0, 'COM_RC_LOSS_T': .5,
                           'COM_RC_STICK_OV': 30, 'COM_RC_OVERRIDE': 3, 'COM_RCL_EXCEPT': 0}.items():
        if name not in params or not math.isclose(params[name], expected, abs_tol=1e-6):
            bad.append(name)
    if bad:
        raise ValueError('unmet platform/parameter checks: '+', '.join(bad))
    for key, actual in zip(('parameter_sha256', 'platform_sha256', 'software_sha256'), hashes[1:]):
        if data.get(key) != actual:
            raise ValueError('hash mismatch: '+key)
    evidence = data.get('evidence', {})
    if not isinstance(evidence, dict):
        raise ValueError('evidence must be an object')
    files = []
    for key in EVIDENCE_KEYS:
        item = evidence.get(key, {})
        if not isinstance(item, dict):
            raise ValueError('invalid evidence entry: '+key)
        if item.get('reviewed') is not True or not item.get('path'):
            raise ValueError('missing operator-reviewed evidence: '+key)
        performed = item.get('performed_at_unix')
        if type(performed) not in (int, float) or not math.isfinite(performed) or not 0 < performed <= now:
            raise ValueError('actual evidence time required: '+key)
        if key == 'site_and_battery' and now-performed > 900:
            raise ValueError('site/battery clearance must be from this test, within 15 minutes')
        path = Path(item['path'])
        if not path.is_absolute() or not path.is_file() or path.stat().st_size == 0:
            raise ValueError('evidence must be a nonempty absolute file: '+key)
        if sha(path) != item.get('sha256'):
            raise ValueError('evidence hash mismatch: '+key)
        files.append((path.resolve(), item['sha256']))
        if key == 'post_calibration_ev':
            report = json.loads(path.read_text())
            if (not isinstance(report, dict) or report.get('passed') is not True or report.get('final_ready') is not True
                    or report.get('ever_armed') is not False or report.get('source_fault') is not None
                    or report.get('subscription_only') is not True):
                raise ValueError('post-calibration observer report did not pass current gate')
            recorded = report.get('recorded_at_unix')
            calibrated = evidence.get('calibration', {}).get('performed_at_unix')
            if (type(recorded) not in (int, float) or not math.isfinite(recorded)
                    or not calibrated <= recorded <= now or now-recorded > 86400):
                raise ValueError('need observer evidence after calibration and within 24 hours')
    permit = FlightPermit(session_id, height, hover, expires, issued, release_path, params_path,
        platform_path, *hashes, tuple(files))
    permit.unchanged()
    return permit, params


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--template', action='store_true', help='print UNAPPROVED blank JSON; no file writes')
    parser.add_argument('--release', type=Path)
    parser.add_argument('--params', type=Path)
    parser.add_argument('--fingerprints', action='store_true', help='print current hashes, NOT approval')
    args = parser.parse_args()
    if args.template:
        print(json.dumps(template(), indent=2))
        return 0
    if args.fingerprints:
        if not args.params:
            parser.error('--params required for fingerprints')
        print(json.dumps({'parameter_sha256': sha(args.params),
            'platform_sha256': sha(ROOT/'config/platform.yaml'),
            'software_sha256': software_sha(), 'flight_approved': False}, indent=2))
        return 0
    if not args.release or not args.params:
        parser.error('--release and --params are required for offline validation')
    try:
        permit, _ = validate(args.release, args.params, ROOT/'config/platform.yaml')
    except (ValueError, OSError, KeyError, TypeError) as exc:
        print(json.dumps({'release_consistent': False, 'error': str(exc), 'flight_approved': False}))
        return 1
    print(json.dumps({'release_consistent': True, 'session_id': permit.session_id,
                      'flight_approved': False, 'note': 'Offline consistency only; no flight started'}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
