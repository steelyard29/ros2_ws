#!/usr/bin/env python3
"""ROS-independent tests for RTAB-Map relay gating decisions."""

import importlib.util
import math
from pathlib import Path
import sys
import unittest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'rtabmap_odom_relay.py')
SPEC = importlib.util.spec_from_file_location('rtabmap_odom_relay', SCRIPT)
relay = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = relay
SPEC.loader.exec_module(relay)


class RelayLogicTest(unittest.TestCase):
    def health(self, tf_age=0.04, vio_age=0.03, **kwargs):
        defaults = {
            'position_jump_m': 0.0,
            'orientation_jump_rad': 0.0,
            'tf_timeout_sec': 0.25,
            'vio_timeout_sec': 0.15,
            'max_position_jump_m': 0.75,
            'max_orientation_jump_rad': 0.70,
        }
        defaults.update(kwargs)
        return relay.assess_health(tf_age=tf_age, vio_age=vio_age, **defaults)

    def test_fresh_sources_are_healthy_and_report_worst_age(self):
        status = self.health(tf_age=0.04, vio_age=0.08)
        self.assertTrue(status.healthy)
        self.assertEqual(status.source, 'rtabmap_tf+vio_body_twist')
        self.assertAlmostEqual(status.age, 0.08)
        self.assertEqual(status.reason, 'ok')

    def test_missing_or_stale_tf_stops_output(self):
        self.assertEqual(
            self.health(tf_age=None).reason, 'missing_rtabmap_tf')
        status = self.health(tf_age=0.251)
        self.assertFalse(status.healthy)
        self.assertEqual(status.reason, 'rtabmap_tf_stale')

    def test_missing_stale_or_future_vio_stops_output(self):
        self.assertEqual(self.health(vio_age=None).reason, 'missing_vio')
        self.assertEqual(self.health(vio_age=0.151).reason, 'vio_stale')
        self.assertEqual(
            self.health(vio_age=-0.051).reason, 'vio_from_future')

    def test_position_and_orientation_jumps_are_rejected(self):
        position_status = self.health(position_jump_m=0.751)
        orientation_status = self.health(orientation_jump_rad=0.701)
        self.assertFalse(position_status.healthy)
        self.assertEqual(position_status.reason, 'position_jump')
        self.assertFalse(orientation_status.healthy)
        self.assertEqual(orientation_status.reason, 'orientation_jump')

    def test_frame_and_data_validation_are_reported(self):
        self.assertEqual(
            self.health(frame_valid=False).reason,
            'vio_child_frame_mismatch')
        self.assertEqual(
            self.health(data_valid=False).reason,
            'non_finite_or_invalid_data')

    def test_sample_age_rejects_zero_stamp(self):
        self.assertIsNone(relay.sample_age(10.0, 0.0))
        self.assertAlmostEqual(relay.sample_age(10.0, 9.8), 0.2)

    def test_pose_jump_handles_quaternion_sign_equivalence(self):
        distance, angle = relay.pose_jump(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            (0.3, 0.4, 0.0),
            (0.0, 0.0, 0.0, -1.0))
        self.assertAlmostEqual(distance, 0.5)
        self.assertAlmostEqual(angle, 0.0)

    def test_quaternion_angle_detects_half_turn_and_invalid_input(self):
        self.assertAlmostEqual(
            relay.quaternion_angle(
                (0.0, 0.0, 0.0, 1.0),
                (0.0, 0.0, 1.0, 0.0)),
            math.pi)
        self.assertTrue(math.isinf(
            relay.quaternion_angle(
                (0.0, 0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 1.0))))

    def test_twist_covariance_never_drops_below_configured_floor(self):
        source = [0.0] * 36
        floor = [0.0] * 36
        source[0] = 0.4
        floor[0] = 0.1
        floor[7] = 0.2
        result = relay.RtabmapOdomRelay._covariance_with_floor(source, floor)
        self.assertEqual(result[0], 0.4)
        self.assertEqual(result[7], 0.2)

    def test_geometric_match_required_until_fresh_proximity_or_loop(self):
        self.assertEqual(
            relay.assess_geometric_match(True, None, 3.0),
            'waiting_geometric_match')
        self.assertEqual(
            relay.assess_geometric_match(True, 3.01, 3.0),
            'geometric_match_stale')
        self.assertIsNone(
            relay.assess_geometric_match(True, 2.9, 3.0))
        self.assertIsNone(
            relay.assess_geometric_match(False, None, 3.0))

    def test_map_to_odom_composes_with_vio_pose(self):
        transform = relay.TransformStamped()
        transform.header.frame_id = 'map'
        transform.transform.translation.x = 1.0
        transform.transform.translation.y = 2.0
        transform.transform.rotation.z = math.sin(math.pi / 4.0)
        transform.transform.rotation.w = math.cos(math.pi / 4.0)
        odom = relay.Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = 1.0
        odom.pose.pose.orientation.w = 1.0

        result = relay.RtabmapOdomRelay._compose_map_odom_with_vio(
            transform, odom)

        self.assertAlmostEqual(result.transform.translation.x, 1.0)
        self.assertAlmostEqual(result.transform.translation.y, 3.0)
        self.assertAlmostEqual(
            result.transform.rotation.z, math.sin(math.pi / 4.0))
        self.assertAlmostEqual(
            result.transform.rotation.w, math.cos(math.pi / 4.0))


if __name__ == '__main__':
    unittest.main()
