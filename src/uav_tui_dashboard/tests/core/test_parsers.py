"""Unit tests for telemetry parser helpers."""

from __future__ import annotations

from datetime import datetime, timezone
from types import SimpleNamespace

import pytest

from uav_tui_dashboard.core.status import UAVStatus, VehicleState
from uav_tui_dashboard.core.parsers import (
    parse_battery_message,
    parse_px4_pose_ned,
    parse_px4_vehicle_status,
)


def test_parse_px4_pose_ned_converts_ned_to_enu() -> None:
    message = SimpleNamespace(
        translation=(1.5, -2.0, 3.25),
        orientation=(1.0, 0.0, 0.0, 0.0),
    )

    position, orientation = parse_px4_pose_ned(message)

    assert position == pytest.approx((-2.0, 1.5, -3.25))
    assert orientation == pytest.approx((0.0, 0.0, 0.0))


def test_parse_battery_message_handles_percentage_and_cell_average() -> None:
    status = UAVStatus.empty().with_battery(80.0, 12.5)
    message = SimpleNamespace(
        percentage=0.5,
        voltage=None,
        cell_voltage=[3.9, 3.8, 3.85],
    )

    battery_level, voltage = parse_battery_message(message, status)

    assert battery_level == pytest.approx(50.0)
    assert voltage == pytest.approx(sum(message.cell_voltage) / len(message.cell_voltage))


def test_parse_px4_vehicle_status_inherits_missing_fields() -> None:
    baseline = VehicleState(
        valid=False,
        timestamp=datetime(2024, 1, 1, tzinfo=timezone.utc),
        arming_state=1,
        nav_state=3,
        failsafe=False,
        pre_flight_checks_pass=True,
    )
    message = SimpleNamespace(
        timestamp=SimpleNamespace(sec=42, nanosec=250_000_000),
        arming_state="2",
        nav_state=None,
        failsafe="true",
        nav_state_valid=1,
    )

    updated = parse_px4_vehicle_status(message, baseline)

    assert updated.valid is True
    assert updated.arming_state == 2
    assert updated.nav_state == baseline.nav_state
    assert updated.failsafe is True
    assert updated.pre_flight_checks_pass == baseline.pre_flight_checks_pass
    assert updated.timestamp == datetime.fromtimestamp(42.25, tz=timezone.utc)
