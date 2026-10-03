"""Primary package for the UAV TUI dashboard."""

from __future__ import annotations

from importlib import metadata

from .core import DataSource, Ros2DataSource, SimDataSource, UAVStatus, Vector3
from .ui import UAVDashboardApp

try:
    __version__ = metadata.version("uav-tui-dashboard")
except metadata.PackageNotFoundError:  # pragma: no cover - local dev fallback
    __version__ = "0.0.0"

__all__ = [
    "DataSource",
    "Ros2DataSource",
    "SimDataSource",
    "UAVDashboardApp",
    "UAVStatus",
    "Vector3",
    "__version__",
]
