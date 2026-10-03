"""Coordinate one EV-only bench window. No ROS, devices, or parameters.

The independent observer must finish its final freshness, exclusivity, and
preflight sample while the EV publisher is still alive. Only then may the
bench stop EV and, after that, the camera. A long earlier ready interval
does not satisfy the final sample.
"""
import json
from pathlib import Path


def control_requests_finalize(text):
    try:
        payload = json.loads(text)
    except (json.JSONDecodeError, TypeError):
        return False
    return isinstance(payload, dict) and payload.get('finalize') is True


def write_control(path, finalize):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps({'finalize': finalize is True}) + '\n')
    tmp.replace(path)


class ObservationWindow:
    """Success path: request -> observer done -> stop EV -> stop camera."""

    def __init__(self):
        self.phase = 'publishing'

    def request_final_sample(self, observer_still_running):
        if self.phase != 'publishing' or observer_still_running is not True:
            raise RuntimeError('cannot request a final sample after the observer has exited')
        self.phase = 'finalizing'

    def observer_finished(self):
        if self.phase != 'finalizing':
            raise RuntimeError('observer final sample was not requested while EV was publishing')
        self.phase = 'observer_done'

    def stop_ev(self):
        if self.phase != 'observer_done':
            raise RuntimeError('refusing to treat EV stop as ordered: observer final sample is incomplete')
        self.phase = 'ev_stopped'

    def stop_camera(self):
        if self.phase != 'ev_stopped':
            raise RuntimeError('refusing to treat camera stop as ordered before EV stop')
        self.phase = 'camera_stopped'

    def abort(self):
        if self.phase != 'camera_stopped':
            self.phase = 'aborted'


def bench_result_accepted(*, runtime_ok, observer, phase):
    """Accept only a completed ordered window whose final sample is still ready."""
    if runtime_ok is not True or phase != 'camera_stopped' or not isinstance(observer, dict):
        return False
    continuous = observer.get('ready_continuous_s_max')
    return bool(
        observer.get('passed') is True
        and observer.get('final_ready') is True
        and observer.get('ever_armed') is False
        and observer.get('source_fault') is None
        and isinstance(continuous, (int, float)) and continuous >= 10
        and observer.get('input_publishers') == {'/fmu/in/vehicle_visual_odometry': 1})
