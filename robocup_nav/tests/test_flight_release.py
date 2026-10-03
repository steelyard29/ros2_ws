import json
import builtins
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import flight_release as release
from test_aux_switch_decoder import params


class ReleaseTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        patcher = patch.object(release, 'ROOT', self.root)
        patcher.start()
        self.addCleanup(patcher.stop)
        (self.root/'config').mkdir()
        (self.root/'run.sh').write_text('# offline test fixture\n')
        self.platform = self.root/'config/platform.yaml'
        self.platform.write_text(yaml.safe_dump(dict(geometry_verified=True,
            sensor_extrinsics_verified=True, imu_calibration_verified=True,
            px4_frame_alignment_verified=True, live_flight_enabled=True,
            length_m=.55, width_m=.55, above_center_m=.115, below_center_without_payload_m=.15)))
        values = {**params(), 'MAV_SYS_ID': 1, 'MAV_COMP_ID': 1,
            'EKF2_EV_CTRL': 11, 'EKF2_HGT_REF': 3, 'EKF2_EV_DELAY': 0,
            'EKF2_EV_POS_X': 0, 'EKF2_EV_POS_Y': 0, 'EKF2_EV_POS_Z': 0,
            'EKF2_EV_QMIN': 50, 'EKF2_BARO_CTRL': 1, 'EKF2_RNG_CTRL': 0,
            'COM_OBL_RC_ACT': 4, 'NAV_RCL_ACT': 3, 'MPC_LAND_SPEED': .6,
            'COM_RC_OVERRIDE': 3, 'COM_RCL_EXCEPT': 0, 'COM_OF_LOSS_T': 1,
            'COM_LOW_BAT_ACT': 0, 'COM_RC_LOSS_T': .5, 'COM_RC_STICK_OV': 30}
        self.params = self.root/'params.txt'
        self.params.write_text(''.join(f'1 1 {k} {v} 9\n' for k, v in values.items()))
        self.path = self.root/'release.json'
        self.data = release.template()
        self.data.update(operator='test fixture, not a flight release', issued_at_unix=990,
            expires_at_unix=1100, parameter_sha256=release.sha(self.params),
            platform_sha256=release.sha(self.platform), software_sha256=release.software_sha(),
            operator_confirms_current_board_export=True,
            operator_confirms_px4_v1_16_2_fmu_v6c=True,
            operator_confirms_component_191_exclusive=True, operator_accepts_warning_only_battery=True)
        for key in release.EVIDENCE_KEYS:
            path = self.root/(key+'.json')
            path.write_text(json.dumps({'fixture': True, 'passed': True, 'final_ready': True,
                'ever_armed': False, 'source_fault': None, 'subscription_only': True,
                'recorded_at_unix': 980}))
            self.data['evidence'][key] = dict(path=str(path), sha256=release.sha(path),
                                            reviewed=True, performed_at_unix=900)
        self.save()

    def save(self):
        self.path.write_text(json.dumps(self.data))

    def validate(self):
        return release.validate(self.path, self.params, self.platform, now=1000)[0]

    def test_complete_fixture_and_atomic_single_use(self):
        permit = self.validate()
        with patch.object(release.time, 'time', return_value=1000):
            permit.consume(self.root/'consumed')
            with self.assertRaises(FileExistsError):
                permit.consume(self.root/'consumed')

    def test_blank_template_rejected(self):
        self.data = release.template()
        self.save()
        with self.assertRaises(ValueError):
            self.validate()

    def test_missing_evidence_never_waived(self):
        for key in release.EVIDENCE_KEYS:
            self.data['evidence'][key]['reviewed'] = False
            self.save()
            with self.assertRaises(ValueError):
                self.validate()
            self.data['evidence'][key]['reviewed'] = True

    def test_expired_and_unbounded_release(self):
        for issued, expiry in ((0, 999), (990, 2000), (1001, 1100)):
            self.data.update(issued_at_unix=issued, expires_at_unix=expiry)
            self.save()
            with self.assertRaises(ValueError):
                self.validate()

    def test_flags_and_enabled_must_be_true(self):
        original = yaml.safe_load(self.platform.read_text())
        for key in ('live_flight_enabled', 'geometry_verified', 'imu_calibration_verified',
                    'sensor_extrinsics_verified', 'px4_frame_alignment_verified'):
            self.platform.write_text(yaml.safe_dump({**original, key: False}))
            self.data.update(platform_sha256=release.sha(self.platform), software_sha256=release.software_sha())
            self.save()
            with self.assertRaises(ValueError):
                self.validate()

    def test_modified_artifacts_and_recheck(self):
        permit = self.validate()
        self.params.write_text(self.params.read_text()+'# changed\n')
        with self.assertRaises(ValueError):
            permit.unchanged()
        with self.assertRaises(ValueError):
            self.validate()

    def test_expiry_rechecked_immediately_before_arm(self):
        permit = self.validate()
        for now in (989, 1101, float('nan')):
            with self.assertRaises(ValueError):
                permit.before_arm(now)

    def test_evidence_change_invalidates_permit(self):
        permit = self.validate()
        path = Path(self.data['evidence']['calibration']['path'])
        path.write_text('changed after review')
        with self.assertRaises(ValueError):
            permit.unchanged()

    def test_software_change_invalidates_permit(self):
        permit = self.validate()
        (self.root/'run.sh').write_text('changed implementation')
        with self.assertRaises(ValueError):
            permit.unchanged()

    def test_observer_must_be_post_calibration(self):
        self.data['evidence']['calibration']['performed_at_unix'] = 990
        self.save()
        with self.assertRaises(ValueError):
            self.validate()

    def test_no_loaded_or_excess_height(self):
        for field, value in (('payload', 'loaded'), ('height_m', 1.4), ('hover_s', 10), ('height_m', True)):
            old = self.data[field]
            self.data[field] = value
            self.save()
            with self.assertRaises(ValueError):
                self.validate()
            self.data[field] = old

    def test_fake_alternate_platform_rejected(self):
        other = self.root/'other.yaml'
        other.write_bytes(self.platform.read_bytes())
        with self.assertRaises(ValueError):
            release.validate(self.path, self.params, other, now=1000)

    def test_real_route_requires_consumed_permit_before_any_ros_import(self):
        from live_flight_core import LiveFlightCore
        from flight_runtime import create_node
        from flight_readiness import read_params
        permit = self.validate()
        core = LiveFlightCore(0, read_params(self.params), permit.height, permit.hover)
        class ReachedRosBoundary(Exception):
            pass
        original_import = builtins.__import__
        def no_ros(name, *args, **kwargs):
            if name == 'rclpy':
                raise ReachedRosBoundary('test stops BEFORE importing ROS or creating publishers')
            return original_import(name, *args, **kwargs)
        with patch.object(release.time, 'time', return_value=1000), patch('builtins.__import__', side_effect=no_ros):
            with self.assertRaises(FileNotFoundError):
                create_node(core, exercise=True, live_permit=permit)
            permit.consume(self.root/'evidence/live_release_consumed')
            with self.assertRaises(ReachedRosBoundary):
                create_node(core, exercise=True, live_permit=permit)
            with self.assertRaises(ValueError):
                create_node(core, exercise=True, isolated=True, live_permit=permit)


if __name__ == '__main__':
    unittest.main()
