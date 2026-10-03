import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from ev_adapter import EvAdapter
from ev_odometry import ARMING_STATE_ARMED, EV_TOPIC, QUALITY_LOST, should_publish_ev

ROOT = Path(__file__).resolve().parents[1]
IDENTITY = [0, 0, 0, 1]


def drive(adapter, n=40, t0=0.0, xyz=(0.0, 0.0, 0.0), dt=0.033, vo_state=1):
    last = None
    for i in range(n):
        now = t0 + i * dt
        stamp = int((now + 1.0) * 1e9)
        adapter.watchdog(now)
        adapter.on_vehicle(1, now)
        adapter.on_flags(False, False, now)
        adapter.on_tracking(vo_state, stamp, now)
        last = adapter.on_pose(stamp, 20.0, now, xyz, IDENTITY, now_ns=stamp + 20_000_000)
    return last


class AdapterFaultTests(unittest.TestCase):
    def test_healthy_stream_publishes_after_warmup(self):
        adapter = EvAdapter(0.0)
        drive(adapter, 40)
        self.assertIsNone(adapter.fault)
        self.assertGreaterEqual(adapter.publish_count, 10)
        self.assertFalse(adapter.inhibit)

    def test_tracking_lost_latches_and_does_not_resume(self):
        adapter = EvAdapter(0.0)
        drive(adapter, 40)
        published = adapter.publish_count
        now = 40 * 0.033
        self.assertFalse(adapter.on_tracking(2, int((now + 1.1) * 1e9), now + 0.033))
        self.assertTrue(adapter.inhibit)
        self.assertIn('Tracking failed', adapter.fault)
        drive(adapter, 10, t0=now + 0.1, vo_state=1)
        self.assertEqual(adapter.publish_count, published)

    def test_jump_with_success_state_still_latches(self):
        adapter = EvAdapter(0.0)
        drive(adapter, 40)
        published = adapter.publish_count
        now = 40 * 0.033
        stamp = int((now + 1.033) * 1e9)
        adapter.on_vehicle(1, now + 0.033)
        adapter.on_flags(False, False, now + 0.033)
        adapter.on_tracking(1, stamp, now + 0.033)
        adapter.on_pose(stamp, 20.0, now + 0.033, (1.2, 0.0, 0.0), IDENTITY,
                        now_ns=stamp + 20_000_000)
        self.assertTrue(adapter.inhibit)
        self.assertIn('discontinuity', adapter.fault)
        later = now + 0.066
        later_stamp = int((later + 1.0) * 1e9)
        adapter.on_tracking(1, later_stamp, later)
        adapter.on_pose(later_stamp, 20.0, later, (1.21, 0.0, 0.0), IDENTITY,
                        now_ns=later_stamp + 20_000_000)
        self.assertEqual(adapter.publish_count, published)

    def test_stale_tracking_watchdog_latches(self):
        adapter = EvAdapter(0.0)
        drive(adapter, 40)
        published = adapter.publish_count
        self.assertFalse(adapter.watchdog(40 * 0.033 + 0.4))
        self.assertIn('stale', adapter.fault)
        drive(adapter, 5, t0=2.0)
        self.assertEqual(adapter.publish_count, published)

    def test_armed_stops(self):
        adapter = EvAdapter(0.0)
        drive(adapter, 40)
        published = adapter.publish_count
        self.assertFalse(adapter.on_vehicle(ARMING_STATE_ARMED, 2.0))
        self.assertIn('armed', adapter.fault.lower())
        drive(adapter, 5, t0=2.1)
        self.assertEqual(adapter.publish_count, published)

    def test_safety_telemetry_timeout_after_publish(self):
        adapter = EvAdapter(0.0)
        drive(adapter, 40)
        after = None
        t0 = 40 * 0.033
        for i in range(90):
            now = t0 + i * 0.033
            stamp = int((now + 1.0) * 1e9)
            adapter.watchdog(now)
            adapter.on_tracking(1, stamp, now)
            adapter.on_pose(stamp, 20.0, now, (0.0, 0.0, 0.0), IDENTITY, now_ns=stamp)
            if adapter.inhibit:
                after = adapter.publish_count
                break
        self.assertIn('telemetry stale', adapter.fault)
        self.assertIsNotNone(after)
        drive(adapter, 5, t0=now + 0.1)
        self.assertEqual(adapter.publish_count, after)

    def test_extra_flight_input_stops(self):
        adapter = EvAdapter(0.0)
        drive(adapter, 40)
        published = adapter.publish_count
        self.assertFalse(adapter.extra_inputs(['/fmu/in/vehicle_command']))
        drive(adapter, 5, t0=2.0)
        self.assertEqual(adapter.publish_count, published)

    def test_lost_quality_is_not_published(self):
        self.assertFalse(should_publish_ev(QUALITY_LOST))
        adapter = EvAdapter(0.0)
        drive(adapter, 40)
        self.assertNotIn(str(QUALITY_LOST), adapter.quality_counts)

    def test_allow_ev_hgt_does_not_stop_on_height_flag(self):
        adapter = EvAdapter(0.0, allow_ev_hgt=True)
        drive(adapter, 35)
        self.assertTrue(adapter.on_flags(True, False, 1.2))
        self.assertIsNone(adapter.fault)
        blocked = EvAdapter(0.0, allow_ev_hgt=False)
        drive(blocked, 35)
        self.assertFalse(blocked.on_flags(True, False, 1.2))
        self.assertIn('height', blocked.fault.lower())


def _replay(path, allow_ev_hgt=True, ev_hgt=False):
    adapter = None
    t0 = None
    with Path(path).open() as handle:
        for line in handle:
            sample = json.loads(line)
            topic = sample.get('topic')
            message = sample.get('message') or {}
            if topic == '/robocup/alignment/tracking':
                stamp = int(message['stamp_ns'])
            elif topic == '/visual_slam/tracking/odometry':
                header = message.get('header') or {}
                stamp = int(header.get('stamp', {}).get('sec', 0)) * 10**9 + int(
                    header.get('stamp', {}).get('nanosec', 0))
            else:
                continue
            if t0 is None:
                t0 = stamp
                adapter = EvAdapter(0.0, allow_ev_hgt=allow_ev_hgt)
            now = max(0.0, (stamp - t0) / 1e9)
            adapter.watchdog(now)
            adapter.on_vehicle(1, now)
            adapter.on_flags(ev_hgt, False, now)
            if topic == '/robocup/alignment/tracking':
                adapter.on_tracking(int(message['vo_state']), stamp, now)
                continue
            pose = ((message.get('pose') or {}).get('pose') or {})
            position = pose.get('position') or {}
            orientation = pose.get('orientation') or {}
            header = message.get('header') or {}
            frame_ok = header.get('frame_id') == 'odom' and message.get('child_frame_id') == 'base_link'
            adapter.on_pose(
                stamp, 20.0, now,
                [position.get('x', 0.0), position.get('y', 0.0), position.get('z', 0.0)],
                [orientation.get('x', 0.0), orientation.get('y', 0.0),
                 orientation.get('z', 0.0), orientation.get('w', 1.0)],
                frame_ok=frame_ok, now_ns=stamp)
    return adapter


class RecordedSessionReplay(unittest.TestCase):
    def test_occlusion_jump_never_resumes(self):
        path = ROOT / 'evidence/ev_inject_20260921_183809/samples.jsonl'
        adapter = _replay(path, allow_ev_hgt=True, ev_hgt=True)
        self.assertGreater(adapter.publish_count, 100)
        self.assertTrue(adapter.inhibit)
        self.assertIn('discontinuity', adapter.fault)
        self.assertNotIn(EV_TOPIC, adapter.fault or '')

    def test_static_recovery_session_has_no_fault(self):
        path = ROOT / 'evidence/alignment_preview_20260921_184124/samples.jsonl'
        adapter = _replay(path, allow_ev_hgt=False, ev_hgt=False)
        self.assertGreater(adapter.preview_count, 100)
        self.assertIsNone(adapter.fault)
        self.assertFalse(adapter.inhibit)


if __name__ == '__main__':
    unittest.main()
