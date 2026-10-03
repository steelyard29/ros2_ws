from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from preflight_window import (
    ObservationWindow, bench_result_accepted, control_requests_finalize, write_control)


def good_observer():
    return {
        'passed': True, 'final_ready': True, 'ever_armed': False, 'source_fault': None,
        'ready_continuous_s_max': 40.0,
        'input_publishers': {'/fmu/in/vehicle_visual_odometry': 1},
    }


class WindowTests(unittest.TestCase):
    def test_finalize_flag_is_strict(self):
        self.assertTrue(control_requests_finalize('{"finalize": true}'))
        for text in ('', '{}', '{"finalize": false}', '{"finalize": 1}',
                     '{"finalize": "true"}', 'true', '{'):
            self.assertFalse(control_requests_finalize(text))

    def test_control_file_roundtrip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'observer_control.json'
            write_control(path, False)
            self.assertFalse(control_requests_finalize(path.read_text()))
            write_control(path, True)
            self.assertTrue(control_requests_finalize(path.read_text()))
            self.assertFalse(path.with_suffix('.json.tmp').exists())

    def test_success_order_is_final_sample_then_ev_then_camera(self):
        window = ObservationWindow()
        window.request_final_sample(True)
        window.observer_finished()
        window.stop_ev()
        window.stop_camera()
        self.assertEqual(window.phase, 'camera_stopped')

    def test_early_observer_exit_cannot_open_the_final_sample(self):
        window = ObservationWindow()
        with self.assertRaises(RuntimeError):
            window.request_final_sample(False)
        with self.assertRaises(RuntimeError):
            window.stop_ev()
        self.assertEqual(window.phase, 'publishing')

    def test_ev_stop_before_final_sample_is_rejected(self):
        window = ObservationWindow()
        window.request_final_sample(True)
        with self.assertRaises(RuntimeError):
            window.stop_ev()
        window.abort()
        self.assertEqual(window.phase, 'aborted')

    def test_camera_stop_before_ev_stop_is_rejected(self):
        window = ObservationWindow()
        window.request_final_sample(True)
        window.observer_finished()
        with self.assertRaises(RuntimeError):
            window.stop_camera()

    def test_historical_ready_interval_does_not_replace_the_final_sample(self):
        stalled = good_observer()
        stalled.update(passed=False, final_ready=False, input_publishers={})
        self.assertFalse(bench_result_accepted(
            runtime_ok=True, observer=stalled, phase='camera_stopped'))
        # A forged pass flag still cannot hide a bad final publisher map.
        stalled['passed'] = True
        self.assertFalse(bench_result_accepted(
            runtime_ok=True, observer=stalled, phase='camera_stopped'))

    def test_accept_requires_ordered_completion_and_live_ev_publisher(self):
        observer = good_observer()
        self.assertTrue(bench_result_accepted(
            runtime_ok=True, observer=observer, phase='camera_stopped'))
        self.assertFalse(bench_result_accepted(
            runtime_ok=True, observer=observer, phase='observer_done'))
        self.assertFalse(bench_result_accepted(
            runtime_ok=False, observer=observer, phase='camera_stopped'))
        armed = good_observer()
        armed['ever_armed'] = True
        self.assertFalse(bench_result_accepted(
            runtime_ok=True, observer=armed, phase='camera_stopped'))
        short = good_observer()
        short['ready_continuous_s_max'] = 9.9
        self.assertFalse(bench_result_accepted(
            runtime_ok=True, observer=short, phase='camera_stopped'))


if __name__ == '__main__':
    unittest.main()
