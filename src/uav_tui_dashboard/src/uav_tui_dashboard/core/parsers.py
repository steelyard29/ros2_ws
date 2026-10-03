"""Message parser utilities shared across telemetry data sources."""

from __future__ import annotations

import logging
import math
from datetime import datetime, timezone
from typing import Any, Optional

from .status import UAVStatus, Vector3, VehicleState

logger = logging.getLogger(__name__)


def parse_odometry_message(message: Any) -> tuple[Vector3, Vector3]:
    """Extract position/orientation vectors from a ROS odometry-like message."""

    if hasattr(message, "pose") and hasattr(message.pose, "pose"):
        pose = message.pose.pose
        position_msg = pose.position
        orientation_msg = pose.orientation
        position = (
            float(getattr(position_msg, "x", 0.0)),
            float(getattr(position_msg, "y", 0.0)),
            float(getattr(position_msg, "z", 0.0)),
        )
        orientation = _quaternion_to_euler(
            float(getattr(orientation_msg, "x", 0.0)),
            float(getattr(orientation_msg, "y", 0.0)),
            float(getattr(orientation_msg, "z", 0.0)),
            float(getattr(orientation_msg, "w", 1.0)),
        )
        return position, orientation

    if hasattr(message, "position") and hasattr(message, "q"):
        pos = getattr(message, "position")
        quat = getattr(message, "q")
        position = (
            float(pos[0]) if len(pos) > 0 else 0.0,
            float(pos[1]) if len(pos) > 1 else 0.0,
            float(pos[2]) if len(pos) > 2 else 0.0,
        )
        orientation = _quaternion_to_euler(
            float(quat[0]) if len(quat) > 0 else 0.0,
            float(quat[1]) if len(quat) > 1 else 0.0,
            float(quat[2]) if len(quat) > 2 else 0.0,
            float(quat[3]) if len(quat) > 3 else 1.0,
        )
        return position, orientation

    if hasattr(message, "position") and hasattr(message, "orientation"):
        position_msg = getattr(message, "position")
        orientation_msg = getattr(message, "orientation")
        position = (
            float(getattr(position_msg, "x", 0.0)),
            float(getattr(position_msg, "y", 0.0)),
            float(getattr(position_msg, "z", 0.0)),
        )
        orientation = _quaternion_to_euler(
            float(getattr(orientation_msg, "x", 0.0)),
            float(getattr(orientation_msg, "y", 0.0)),
            float(getattr(orientation_msg, "z", 0.0)),
            float(getattr(orientation_msg, "w", 1.0)),
        )
        return position, orientation

    raise TypeError(
        "无法从消息中解析位置和姿态信息，" f"收到类型: {type(message).__name__}"
    )


def parse_battery_message(message: Any, current_status: UAVStatus) -> tuple[float, float]:
    """Extract battery percentage (0-100) and voltage from message."""

    battery_level = current_status.battery_level
    voltage = current_status.voltage

    percentage = getattr(message, "percentage", None)
    if percentage is not None:
        try:
            percentage_value = float(percentage)
        except (TypeError, ValueError):
            logger.debug("忽略无法解析的电量百分比值: %s", percentage)
        else:
            if not math.isnan(percentage_value):
                # ROS BatteryState.percentage is 0-1, but may already be 0-100 in some stacks.
                if percentage_value <= 1.0:
                    battery_level = max(0.0, min(100.0, percentage_value * 100.0))
                else:
                    battery_level = max(0.0, min(100.0, percentage_value))

    power_supply_status = getattr(message, "power_supply_status", None)
    if power_supply_status is not None:
        logger.debug("Battery power supply status: %s", power_supply_status)

    msg_voltage = getattr(message, "voltage", None)
    if msg_voltage is not None:
        try:
            voltage_value = float(msg_voltage)
        except (TypeError, ValueError):
            logger.debug("忽略无法解析的电压值: %s", msg_voltage)
        else:
            if not math.isnan(voltage_value) and voltage_value > 0.0:
                voltage = voltage_value
    if voltage == current_status.voltage:
        cell_voltages = getattr(message, "cell_voltage", None)
        if cell_voltages:
            values: list[float] = []
            for cell in cell_voltages:
                try:
                    cell_value = float(cell)
                except (TypeError, ValueError):
                    continue
                if not math.isnan(cell_value) and cell_value > 0.0:
                    values.append(cell_value)
            if values:
                voltage = sum(values) / len(values)

    return battery_level, voltage


def parse_vehicle_status_message(message: Any, current_state: VehicleState) -> VehicleState:
    """Parse a generic vehicle status style message into :class:`VehicleState`."""

    valid = _coerce_bool(getattr(message, "valid", current_state.valid), current_state.valid)

    timestamp_raw = getattr(message, "timestamp", None)
    timestamp = _ros_time_to_datetime(timestamp_raw) or current_state.timestamp

    arming_state = _coerce_optional_int(
        getattr(message, "arming_state", current_state.arming_state), current_state.arming_state
    )
    nav_state = _coerce_optional_int(
        getattr(message, "nav_state", current_state.nav_state), current_state.nav_state
    )

    failsafe = _coerce_bool(getattr(message, "failsafe", current_state.failsafe), current_state.failsafe)
    preflight_default = current_state.pre_flight_checks_pass
    preflight_raw = getattr(message, "pre_flight_checks_pass", preflight_default)
    pre_flight_checks_pass = _coerce_optional_bool(preflight_raw, preflight_default)

    return VehicleState(
        valid=valid,
        timestamp=timestamp,
        arming_state=arming_state,
        nav_state=nav_state,
        failsafe=failsafe,
        pre_flight_checks_pass=pre_flight_checks_pass,
    )


def parse_px4_pose_ned(message: Any) -> tuple[Vector3, Vector3]:
    """Parse px4_interface/msg/PoseNED message into ENU position and Euler angles."""

    translation = getattr(message, "translation", None)
    if translation is None:
        raise TypeError("PoseNED 消息缺少 translation 字段")
    translation_values = list(translation)
    if len(translation_values) < 3:
        raise TypeError("PoseNED translation 字段长度不足 3")
    try:
        north = float(translation_values[0])
        east = float(translation_values[1])
        down = float(translation_values[2])
    except (TypeError, ValueError) as exc:
        raise TypeError("PoseNED translation 字段无法解析为浮点数") from exc

    # Convert NED (North, East, Down) to ENU (East, North, Up) for UI display.
    position: Vector3 = (east, north, -down)

    orientation_components = getattr(message, "orientation", None)
    orientation: Vector3 = (0.0, 0.0, 0.0)
    if orientation_components is not None:
        components = list(orientation_components)
        if len(components) >= 4:
            try:
                w = float(components[0])
                x = float(components[1])
                y = float(components[2])
                z = float(components[3])
            except (TypeError, ValueError) as exc:
                raise TypeError("PoseNED orientation 字段无法解析为浮点数") from exc
            orientation = _quaternion_to_euler(x, y, z, w)

    return position, orientation


def parse_px4_battery_status(message: Any, current_status: UAVStatus) -> tuple[float, float]:
    """Parse px4_interface/msg/BatteryStatus message into percentage and voltage."""

    battery_level = current_status.battery_level
    voltage = current_status.voltage

    remaining = getattr(message, "remaining", None)
    if remaining is not None:
        try:
            remaining_value = float(remaining)
        except (TypeError, ValueError):
            logger.debug("忽略无法解析的剩余电量: %s", remaining)
        else:
            if not math.isnan(remaining_value):
                if 0.0 <= remaining_value <= 1.0:
                    battery_level = max(0.0, min(100.0, remaining_value * 100.0))
                else:
                    battery_level = max(0.0, min(100.0, remaining_value))

    voltage_v = getattr(message, "voltage_v", None)
    if voltage_v is not None:
        try:
            voltage_value = float(voltage_v)
        except (TypeError, ValueError):
            logger.debug("忽略无法解析的电压值: %s", voltage_v)
        else:
            if not math.isnan(voltage_value) and voltage_value > 0.0:
                voltage = voltage_value

    return battery_level, voltage


def parse_px4_vehicle_status(message: Any, current_state: VehicleState) -> VehicleState:
    """Parse px4_interface/msg/VehicleStatus into :class:`VehicleState`."""

    timestamp = (
        _ros_time_to_datetime(getattr(message, "timestamp", None))
        or _ros_time_to_datetime(getattr(message, "timestamp_sample", None))
        or _px4_microseconds_to_datetime(getattr(message, "timestamp", None))
        or current_state.timestamp
    )

    arming_state = _coerce_optional_int(getattr(message, "arming_state", None), current_state.arming_state)
    nav_state = _coerce_optional_int(getattr(message, "nav_state", None), current_state.nav_state)
    failsafe = _coerce_bool(getattr(message, "failsafe", current_state.failsafe), current_state.failsafe)

    preflight_raw = getattr(message, "pre_flight_checks_pass", current_state.pre_flight_checks_pass)
    pre_flight_checks_pass = _coerce_optional_bool(preflight_raw, current_state.pre_flight_checks_pass)

    valid = _coerce_bool(
        getattr(message, "nav_state_valid", getattr(message, "valid", current_state.valid)),
        current_state.valid,
    )

    return VehicleState(
        valid=valid,
        timestamp=timestamp,
        arming_state=arming_state,
        nav_state=nav_state,
        failsafe=failsafe,
        pre_flight_checks_pass=pre_flight_checks_pass,
    )


def _ros_time_to_datetime(value: Any) -> Optional[datetime]:
    if value is None:
        return None
    if isinstance(value, datetime):
        return value if value.tzinfo else value.replace(tzinfo=timezone.utc)

    sec = getattr(value, "sec", None)
    nanosec = getattr(value, "nanosec", None)
    if sec is not None and nanosec is not None:
        try:
            sec_value = int(sec)
            nanosec_value = int(nanosec)
        except (TypeError, ValueError):
            return None
        timestamp = sec_value + nanosec_value / 1_000_000_000
        try:
            return datetime.fromtimestamp(timestamp, tz=timezone.utc)
        except OSError:  # pragma: no cover - defensive against platform limits
            return None

    if isinstance(value, (tuple, list)) and len(value) >= 2:
        try:
            sec_value = int(value[0])
            nanosec_value = int(value[1])
        except (TypeError, ValueError):
            return None
        timestamp = sec_value + nanosec_value / 1_000_000_000
        return datetime.fromtimestamp(timestamp, tz=timezone.utc)

    return None


def _px4_microseconds_to_datetime(value: Any) -> Optional[datetime]:
    if value is None:
        return None
    try:
        raw = int(value)
    except (TypeError, ValueError):
        return None
    if raw <= 0:
        return None

    seconds = raw / 1_000_000
    try:
        return datetime.fromtimestamp(seconds, tz=timezone.utc)
    except OSError:  # pragma: no cover - defensive
        return None


def _coerce_optional_int(value: Any, default: Optional[int]) -> Optional[int]:
    if value is None:
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _coerce_bool(value: Any, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "1", "yes", "on"}:
            return True
        if lowered in {"false", "0", "no", "off"}:
            return False
    return default


def _coerce_optional_bool(value: Any, default: Optional[bool]) -> Optional[bool]:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "1", "yes", "on"}:
            return True
        if lowered in {"false", "0", "no", "off"}:
            return False
    return default


def _quaternion_to_euler(x: float, y: float, z: float, w: float) -> Vector3:
    """Convert quaternion to Euler angles (roll, pitch, yaw) in radians."""

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


__all__ = [
    "parse_odometry_message",
    "parse_battery_message",
    "parse_vehicle_status_message",
    "parse_px4_pose_ned",
    "parse_px4_battery_status",
    "parse_px4_vehicle_status",
]
