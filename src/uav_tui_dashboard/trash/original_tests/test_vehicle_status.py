"""Tests for vehicle status parsing and model helpers."""

from __future__ import annotations

import sys
from datetime import datetime, timezone
from importlib import import_module
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

VehicleState = import_module("uav_tui_dashboard.core.models").VehicleState
UAVStatus = import_module("uav_tui_dashboard.core.models").UAVStatus
parse_px4_vehicle_status = import_module("uav_tui_dashboard.core.datasource").parse_px4_vehicle_status


class _RosTime:
    def __init__(self, sec: int, nanosec: int) -> None:
        self.sec = sec
        self.nanosec = nanosec


def _make_message(**kwargs):
    class _Message:
        pass

    message = _Message()
    for key, value in kwargs.items():
        setattr(message, key, value)
    return message


def test_parse_px4_vehicle_status_from_ros_time() -> None:
    timestamp = _RosTime(1_700_000_000, 123_000_000)
    message = _make_message(
        valid=True,
        timestamp=timestamp,
        arming_state=2,
        nav_state=5,
        failsafe=False,
        pre_flight_checks_pass=True,
    )

    result = parse_px4_vehicle_status(message, VehicleState())

    assert result.valid is True
    assert result.arming_state == 2
    assert result.nav_state == 5
    assert result.failsafe is False
    assert result.pre_flight_checks_pass is True
    assert result.timestamp is not None
    assert result.timestamp.tzinfo is not None
    assert result.timestamp.astimezone(timezone.utc) == datetime.fromtimestamp(
        1_700_000_000 + 0.123, tz=timezone.utc
    )


def test_parse_px4_vehicle_status_fallback_microseconds() -> None:
    base_state = VehicleState(valid=False, failsafe=True)
    message = _make_message(
        timestamp=1_700_000_500_000,
        nav_state=3,
        pre_flight_checks_pass=False,
    )

    result = parse_px4_vehicle_status(message, base_state)

    assert result.valid is False  # inherits from base when missing
    assert result.failsafe is True
    assert result.nav_state == 3
    assert result.pre_flight_checks_pass is False
    assert result.timestamp is not None
    assert abs((result.timestamp - datetime.fromtimestamp(1_700_000.5, tz=timezone.utc)).total_seconds()) < 1e-6


def test_uav_status_with_vehicle_state_replaces_instance() -> None:
    base = UAVStatus.empty()
    new_state = VehicleState(valid=True)

    updated = base.with_vehicle_state(new_state)

    assert updated.vehicle_state is new_state
    assert base.vehicle_state is not new_state