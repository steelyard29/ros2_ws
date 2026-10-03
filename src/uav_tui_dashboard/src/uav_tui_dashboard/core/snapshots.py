"""Snapshot management components for the UAV TUI dashboard."""

from __future__ import annotations

import json
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock
from typing import Any, Iterable, Optional

from .status import UAVStatus


@dataclass(slots=True)
class FlightSnapshot:
    """Immutable snapshot of a UAV status captured at a point in time."""

    captured_at: datetime
    status: UAVStatus

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serialisable representation of the snapshot."""

        timestamp = self.captured_at
        if timestamp.tzinfo is None:
            timestamp = timestamp.replace(tzinfo=timezone.utc)

        return {
            "captured_at": timestamp.astimezone(timezone.utc).isoformat(),
            "status": self.status.to_dict(),
        }


class FlightSnapshotQueue:
    """Thread-safe FIFO queue that stores recent flight snapshots."""

    def __init__(self, *, maxlen: Optional[int] = 20) -> None:
        self._snapshots: deque[FlightSnapshot] = deque(maxlen=maxlen)
        self._lock = Lock()
        self._maxlen = maxlen

    def capture(self, status: UAVStatus, *, captured_at: Optional[datetime] = None) -> FlightSnapshot:
        """Capture the provided status and append it to the queue."""

        snapshot = FlightSnapshot(
            captured_at=captured_at or datetime.now(tz=timezone.utc),
            status=status.copy_for_snapshot(),
        )
        with self._lock:
            self._snapshots.append(snapshot)
        return snapshot

    def list(self) -> list[FlightSnapshot]:
        """Return a shallow copy of the current snapshot list."""

        with self._lock:
            return list(self._snapshots)

    def clear(self) -> None:
        """Remove all stored snapshots."""

        with self._lock:
            self._snapshots.clear()

    def __len__(self) -> int:  # pragma: no cover - trivial forwarding
        with self._lock:
            return len(self._snapshots)

    def __iter__(self) -> Iterable[FlightSnapshot]:  # pragma: no cover - convenience
        return iter(self.list())

    def export(self, destination: Path | str | None = None) -> Path:
        """Export the stored snapshots to a JSON file.

        Args:
            destination: Either a concrete file path, a directory in which the
                snapshots will be exported (with an auto-generated name), or
                ``None`` to export into the current working directory.

        Returns:
            Path to the written file.

        Raises:
            ValueError: If no snapshots are available to export.
        """

        snapshots = self.list()
        if not snapshots:
            raise ValueError("No snapshots available for export.")

        data = [snapshot.to_dict() for snapshot in snapshots]

        if destination is None:
            dest_path = Path.cwd()
            filename = datetime.now(tz=timezone.utc).strftime("uav_snapshots_%Y%m%d-%H%M%S.json")
            output = dest_path / filename
        else:
            destination_path = Path(destination)
            if destination_path.is_dir():
                filename = datetime.now(tz=timezone.utc).strftime("uav_snapshots_%Y%m%d-%H%M%S.json")
                output = destination_path / filename
            else:
                output = destination_path

        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        return output


__all__ = [
    "FlightSnapshot",
    "FlightSnapshotQueue",
]