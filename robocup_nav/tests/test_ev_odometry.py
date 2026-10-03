import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from ev_odometry import (
    ARMING_STATE_ARMED,
    EV_QMIN,
    EV_TOPIC,
    MIN_HEALTHY_PREVIEWS,
    POSITION_VARIANCE,
    POSE_FRAME_FRD,
    QUALITY_DEGRADED,
    QUALITY_HEALTHY,
    QUALITY_LOST,
    QUALITY_REJECT,
    VELOCITY_FRAME_FRD,
    VELOCITY_FRAME_UNKNOWN,
    apply_to_vehicle_odometry,
    build_ev_odometry,
    discontinuity,
    ev_quality,
    extra_flight_inputs,
    ev_position_frd,
    ev_timestamps_us,
    ev_variances,
    inhibit_reason,
    session_convert,
    should_publish_ev,
)


class EvOdometryTests(unittest.TestCase):
    def test_flu_to_local_frd_matches_alignment_math(self):
        frame, *_rest, position, orientation = session_convert(
            None, [4, 5, 6], [0, 0, 0, 1])
        p, q = frame.convert([5, 6, 7], [0, 0, 0, 1])
        np.testing.assert_allclose(position, [0, 0, 0])
        np.testing.assert_allclose(p, [1, -1, -1])
        np.testing.assert_allclose(q, [1, 0, 0, 0])
        np.testing.assert_allclose(orientation, [1, 0, 0, 0])

    def test_timestamps_ignore_timesync_offset(self):
        now_ns, sample_ns = 1_000_000_000, 999_000_000
        offset = -1_789_812_382_942_767
        self.assertEqual(ev_timestamps_us(now_ns, sample_ns, offset), (1_000_000, 999_000))
        self.assertEqual(ev_timestamps_us(now_ns, sample_ns, 0), (1_000_000, 999_000))

    def test_range_cannot_replace_vio_z(self):
        self.assertEqual(ev_position_frd([0.1, -0.2, 0.45], dist_bottom=0.099), (0.1, -0.2, 0.45))

    def test_message_contract_matches_passed_bench(self):
        ev = build_ev_odometry([0.2, -0.1, -0.05], [1, 0, 0, 0], 2_000_000_000, 1_999_000_000,
                               vo_state=1, timesync_offset_us=-123456, dist_bottom=0.37)
        self.assertEqual(ev.pose_frame, POSE_FRAME_FRD)
        self.assertEqual(ev.velocity_frame, VELOCITY_FRAME_FRD)
        self.assertNotEqual(ev.velocity_frame, VELOCITY_FRAME_UNKNOWN)
        self.assertEqual(ev.quality, QUALITY_HEALTHY)
        self.assertEqual(ev.position, (0.2, -0.1, -0.05))
        self.assertEqual(ev.position_variance, POSITION_VARIANCE)
        self.assertTrue(all(math.isnan(v) for v in ev.velocity))
        self.assertTrue(all(math.isnan(v) for v in ev.angular_velocity))
        self.assertTrue(all(math.isnan(v) for v in ev.velocity_variance))
        self.assertTrue(all(math.isfinite(v) for v in ev.position_variance))
        self.assertTrue(all(math.isfinite(v) for v in ev.orientation_variance))
        self.assertEqual(ev.timestamp, 2_000_000)
        self.assertEqual(ev.timestamp_sample, 1_999_000)
        msg = SimpleNamespace()
        apply_to_vehicle_odometry(msg, ev)
        self.assertEqual(msg.pose_frame, POSE_FRAME_FRD)
        self.assertEqual(msg.quality, QUALITY_HEALTHY)
        self.assertEqual(msg.position, [0.2, -0.1, -0.05])

    def test_quality_follows_tracking_not_unconditional_100(self):
        self.assertEqual(ev_quality(vo_state=1), QUALITY_HEALTHY)
        self.assertEqual(ev_quality(vo_state=0), QUALITY_LOST)
        self.assertEqual(ev_quality(vo_state=2), QUALITY_LOST)
        self.assertEqual(ev_quality(vo_state=1, position_step_m=0.10), QUALITY_DEGRADED)
        self.assertEqual(ev_quality(vo_state=1, position_step_m=0.20), QUALITY_REJECT)
        self.assertEqual(ev_quality(vo_state=1, dt_s=0.8), QUALITY_REJECT)
        self.assertFalse(should_publish_ev(QUALITY_LOST))
        self.assertFalse(should_publish_ev(QUALITY_REJECT))
        self.assertTrue(should_publish_ev(QUALITY_DEGRADED))
        self.assertTrue(should_publish_ev(QUALITY_HEALTHY))
        self.assertEqual(EV_QMIN, 50)
        lost = build_ev_odometry([0, 0, 0], [1, 0, 0, 0], 1_000_000_000, 999_000_000, vo_state=2)
        self.assertEqual(lost.quality, QUALITY_LOST)
        self.assertEqual(lost.position_variance, ev_variances(QUALITY_LOST)[0])
        degraded = build_ev_odometry([0, 0, 0], [1, 0, 0, 0], 1_000_000_000, 999_000_000,
                                     vo_state=1, position_step_m=0.10)
        self.assertEqual(degraded.quality, QUALITY_DEGRADED)
        self.assertEqual(degraded.position_variance, tuple(v * 4 for v in POSITION_VARIANCE))

    def test_inhibit_on_armed_height_velocity_and_extra_inputs(self):
        self.assertIsNone(inhibit_reason())
        self.assertIn('armed', inhibit_reason(armed=True).lower())
        self.assertIn('height', inhibit_reason(cs_ev_hgt=True).lower())
        self.assertIsNone(inhibit_reason(cs_ev_hgt=True, allow_ev_hgt=True))
        self.assertIn('velocity', inhibit_reason(cs_ev_vel=True).lower())
        self.assertIn('velocity', inhibit_reason(cs_ev_vel=True, allow_ev_hgt=True).lower())
        self.assertIn('/fmu/in/vehicle_command',
                      inhibit_reason(extra_inputs=['/fmu/in/vehicle_command']))
        self.assertEqual(ARMING_STATE_ARMED, 2)

    def test_own_ev_publisher_is_whitelisted(self):
        self.assertEqual(extra_flight_inputs({EV_TOPIC: 1}, own_ev_publishers=1), [])
        self.assertEqual(extra_flight_inputs({EV_TOPIC: 2}, own_ev_publishers=1), [EV_TOPIC])
        self.assertEqual(extra_flight_inputs({
            EV_TOPIC: 1, '/fmu/in/offboard_control_mode': 1,
        }, own_ev_publishers=1), ['/fmu/in/offboard_control_mode'])

    def test_discontinuity_latches_large_step(self):
        identity = np.eye(3)
        previous = (0, np.zeros(3), identity)
        self.assertIsNone(discontinuity(previous, 33_000_000, np.array([0.05, 0, 0]), identity))
        self.assertEqual(discontinuity(previous, 33_000_000, np.array([0.4, 0, 0]), identity),
                         'VIO discontinuity; new session required')
        yaw = Rotation.from_euler('z', 40, degrees=True).as_matrix()
        self.assertEqual(discontinuity(previous, 33_000_000, np.zeros(3), yaw),
                         'VIO discontinuity; new session required')

    def test_warmup_gate_unchanged(self):
        self.assertEqual(MIN_HEALTHY_PREVIEWS, 30)

    def test_local_frd_does_not_copy_px4_heading(self):
        frame, *_rest, position, orientation = session_convert(None, [0, 0, 0], [0, 0, 0, 1])
        p, q = frame.convert([1, 0, 0], [0, 0, 0, 1])
        np.testing.assert_allclose(p, [1, 0, 0])
        np.testing.assert_allclose(q, [1, 0, 0, 0])
        np.testing.assert_allclose(position, [0, 0, 0])
        np.testing.assert_allclose(orientation, [1, 0, 0, 0])


if __name__ == '__main__':
    unittest.main()
