"""Backward-compatible shim for the legacy `core.datasource` module.

Import from the dedicated submodules (`base`, `sim`, `ros`, `parsers`) instead.
"""

from __future__ import annotations

import warnings

from .base import DataSource, VehicleStateParser
from .parsers import (
    parse_battery_message,
    parse_odometry_message,
    parse_px4_battery_status,
    parse_px4_pose_ned,
    parse_px4_vehicle_status,
    parse_vehicle_status_message,
)
from .ros import Ros2DataSource
from .sim import SimDataSource

warnings.warn(
    "`uav_tui_dashboard.core.datasource` 已拆分为多个子模块，"
    "请改为从 `core.base` / `core.sim` / `core.ros` / `core.parsers` 导入。",
    DeprecationWarning,
    stacklevel=2,
)

__all__ = [
    "DataSource",
    "VehicleStateParser",
    "SimDataSource",
    "Ros2DataSource",
    "parse_odometry_message",
    "parse_battery_message",
    "parse_vehicle_status_message",
    "parse_px4_pose_ned",
    "parse_px4_battery_status",
    "parse_px4_vehicle_status",
]
