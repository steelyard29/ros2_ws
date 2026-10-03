"""Tests for the flight snapshot queue helpers."""

from __future__ import annotations

import json
import sys
from datetime import datetime, timedelta, timezone
from importlib import import_module
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

models = import_module("uav_tui_dashboard.core.models")
FlightSnapshotQueue = models.FlightSnapshotQueue
UAVStatus = models.UAVStatus
VehicleState = models.VehicleState


def _make_status(index: int) -> UAVStatus:
    return (
        UAVStatus.empty()
        .with_position((float(index), float(index) + 0.25, -float(index)))
        .with_orientation((index * 0.1, index * 0.2, index * 0.3))
        .with_battery(75.0 - index, 12.0 + index * 0.05)
        .with_vehicle_state(
            VehicleState(
                valid=True,
                timestamp=datetime(2025, 1, 1, tzinfo=timezone.utc) + timedelta(seconds=index),
                arming_state=2,
                nav_state=5,
                failsafe=False,
                pre_flight_checks_pass=True,
            )
        )
    )


def test_snapshot_queue_creates_independent_copy() -> None:
    queue = FlightSnapshotQueue(maxlen=5)
    status = _make_status(0)

    snapshot = queue.capture(status)

    status.vehicle_state.valid = False
    status.vehicle_state.arming_state = 1

    assert snapshot.status.vehicle_state.valid is True
    assert snapshot.status.vehicle_state.arming_state == 2
    assert len(queue) == 1


def test_snapshot_queue_respects_maxlen_and_export(tmp_path: Path) -> None:
    queue = FlightSnapshotQueue(maxlen=2)

    for idx in range(3):
        queue.capture(_make_status(idx), captured_at=datetime(2025, 1, 1, tzinfo=timezone.utc) + timedelta(minutes=idx))

    snapshots = queue.list()
    assert len(snapshots) == 2
    assert snapshots[0].status.position[0] == 1.0
    assert snapshots[1].status.position[0] == 2.0

    exported = queue.export(tmp_path)
    assert exported.exists()

    payload = json.loads(exported.read_text(encoding="utf-8"))
    assert len(payload) == 2
    assert payload[0]["status"]["position"][0] == 1.0
    assert payload[0]["captured_at"].startswith("2025-01-01T")


def test_snapshot_queue_export_requires_snapshots(tmp_path: Path) -> None:
    queue = FlightSnapshotQueue()

    with pytest.raises(ValueError):
        queue.export(tmp_path)
