"""Command line interface wiring the data source and UI layers."""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
import sys
from pathlib import Path
from typing import Any, Callable, Coroutine, Optional, Sequence
import tomli as tomllib

from .core import (
    DataSource,
    Ros2DataSource,
    SimDataSource,
    parse_px4_battery_status,
    parse_px4_pose_ned,
    parse_px4_vehicle_status,
)
from .logging_config import LoggingSetupResult, configure_logging
from .shutdown import GracefulShutdown
from .ui import UAVDashboardApp

def _run_coroutine(factory: Callable[[], Coroutine[Any, Any, Any]]) -> None:
    try:
        asyncio.run(factory())
    except RuntimeError:
        loop = asyncio.new_event_loop()
        try:
            loop.run_until_complete(factory())
        finally:
            loop.close()


def _ensure_data_source_stopped(data_source: DataSource) -> None:
    async def _stop() -> None:
        await data_source.stop()

    _run_coroutine(_stop)


def _request_app_exit(app: UAVDashboardApp) -> None:
    try:
        if getattr(app, "is_running", False):
            app.call_from_thread(app.exit)
            return
    except RuntimeError:
        logger.debug("调用 call_from_thread 失败，尝试直接退出", exc_info=True)

    app.exit()

logger = logging.getLogger(__name__)

def _load_config(config_path: Path) -> dict[str, Any]:
    """Load configuration from TOML file."""
    if not config_path.exists():
        return {}
    try:
        with open(config_path, 'rb') as f:
            config = tomllib.load(f)
        # Convert empty strings to None for optional paths
        if 'logging' in config:
            logging_config = config['logging']
            if logging_config.get('dir') == '':
                logging_config['dir'] = None
            if logging_config.get('config') == '':
                logging_config['config'] = None
        if 'ros2' in config:
            ros2_config = config['ros2']
            if ros2_config.get('namespace') == '':
                ros2_config['namespace'] = None
            if ros2_config.get('profile') == '':
                ros2_config['profile'] = None
        return config
    except Exception as e:
        logger.warning("Failed to load config from %s: %s", config_path, e)
        return {}

LOG_CONFIG_ENV_VAR = "UAV_TUI_LOG_CONFIG"
LOG_DIR_ENV_VAR = "UAV_TUI_LOG_DIR"

DEFAULT_ODOMETRY_TOPIC = "/uav/odometry"
DEFAULT_BATTERY_TOPIC = "/uav/battery"
DEFAULT_ODOMETRY_TYPE = "nav_msgs.msg.Odometry"
DEFAULT_BATTERY_TYPE = "sensor_msgs.msg.BatteryState"
DEFAULT_VEHICLE_STATUS_TOPIC = ""
DEFAULT_VEHICLE_STATUS_TYPE = "px4_interface.msg.VehicleStatus"

ROS_PROFILE_PX4_INTERFACE = "px4_interface"

ROS_PROFILES: dict[str, dict[str, Any]] = {
    ROS_PROFILE_PX4_INTERFACE: {
        "odometry_topic": "/cache/vehicle_odometry",
        "odometry_type": "px4_interface.msg.PoseNED",
        "battery_topic": "/cache/battery_status",
        "battery_type": "px4_interface.msg.BatteryStatus",
        "odometry_parser": parse_px4_pose_ned,
        "battery_parser": parse_px4_battery_status,
        "vehicle_status_topic": "/cache/vehicle_status",
        "vehicle_status_type": "px4_interface.msg.VehicleStatus",
        "vehicle_status_parser": parse_px4_vehicle_status,
    }
}


def _resolve_optional_path(value: Optional[Path | str]) -> Optional[Path]:
    if value is None or value == "":
        return None
    path = Path(value).expanduser()
    return path


def _configure_logging_from_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> LoggingSetupResult:
    config_path = _resolve_optional_path(args.log_config)
    if config_path is None:
        env_config = os.getenv(LOG_CONFIG_ENV_VAR)
        config_path = _resolve_optional_path(env_config)

    log_dir = _resolve_optional_path(args.log_dir)
    if log_dir is None:
        env_dir = os.getenv(LOG_DIR_ENV_VAR)
        log_dir = _resolve_optional_path(env_dir)

    try:
        result = configure_logging(
            level=args.log_level,
            console=not args.no_log_console,
            log_dir=log_dir,
            config_path=config_path,
        )
    except FileNotFoundError as exc:
        parser.error(f"指定的日志配置文件不存在: {exc}")
    except ValueError as exc:
        parser.error(f"日志配置文件解析失败: {exc}")
    except OSError as exc:  # pragma: no cover - defensive path
        parser.error(f"初始化日志系统失败: {exc}")

    logger.info("日志已初始化，输出目录：%s", result.directory)
    for name, file_path in result.handler_files.items():
        logger.debug("日志处理器 %s 写入文件 %s", name, file_path)

    return result


def _install_global_exception_hook() -> None:
    original_hook = sys.excepthook

    def _handle(exc_type: type[BaseException], exc_value: BaseException, exc_traceback) -> None:
        if issubclass(exc_type, KeyboardInterrupt):
            original_hook(exc_type, exc_value, exc_traceback)
            return
        logger.critical("未处理异常导致应用退出", exc_info=(exc_type, exc_value, exc_traceback))
        original_hook(exc_type, exc_value, exc_traceback)

    sys.excepthook = _handle


def _create_config_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("config/default.toml"),
        help="Path to configuration file (TOML).",
    )
    return parser


def _create_parser(config: dict[str, Any]) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="UAV TUI dashboard")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("config/default.toml"),
        help="Path to configuration file (TOML).",
    )
    parser.add_argument(
        "--mode",
        choices=("sim", "ros2"),
        default=config.get("general", {}).get("mode", "sim"),
        help="选择遥测数据源模式：'sim' 为内置模拟器，'ros2' 为订阅 ROS2 主题。",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=config.get("general", {}).get("poll_interval", 1.0),
        help="Polling interval (seconds) for fetching telemetry data.",
    )
    parser.add_argument(
        "--log-level",
        default=config.get("logging", {}).get("level", "INFO"),
        help="Python logging level (e.g. INFO, DEBUG).",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=config.get("logging", {}).get("dir"),
        help="日志文件输出目录，默认使用系统日志目录。",
    )
    parser.add_argument(
        "--log-config",
        type=Path,
        default=config.get("logging", {}).get("config"),
        help="日志配置文件（TOML/JSON），覆盖默认配置。",
    )
    parser.add_argument(
        "--no-log-console",
        action="store_true",
        default=config.get("logging", {}).get("no_console", False),
        help="关闭控制台日志输出，仅写入文件。",
    )
    parser.add_argument(
        "--ros-namespace",
        default=config.get("ros2", {}).get("namespace"),
        help="ROS2 数据源使用的命名空间，默认使用全局命名空间。",
    )
    parser.add_argument(
        "--ros-odometry-topic",
        default=config.get("ros2", {}).get("odometry_topic", DEFAULT_ODOMETRY_TOPIC),
        help="提供位置与姿态信息的里程计主题 (nav_msgs/msg/Odometry)。",
    )
    parser.add_argument(
        "--ros-battery-topic",
        default=config.get("ros2", {}).get("battery_topic", DEFAULT_BATTERY_TOPIC),
        help="提供电池信息的主题 (sensor_msgs/msg/BatteryState)。传入空字符串以禁用。",
    )
    parser.add_argument(
        "--ros-odometry-type",
        default=config.get("ros2", {}).get("odometry_type", DEFAULT_ODOMETRY_TYPE),
        help="里程计主题的消息类型，使用完整的模块路径 (如 nav_msgs.msg.Odometry)。",
    )
    parser.add_argument(
        "--ros-battery-type",
        default=config.get("ros2", {}).get("battery_type", DEFAULT_BATTERY_TYPE),
        help="电池主题的消息类型，使用完整的模块路径。",
    )
    parser.add_argument(
        "--ros-vehicle-status-topic",
        default=config.get("ros2", {}).get("vehicle_status_topic", DEFAULT_VEHICLE_STATUS_TOPIC),
        help=(
            "提供飞行器状态信息的主题。默认为空字符串以禁用该订阅。"
        ),
    )
    parser.add_argument(
        "--ros-vehicle-status-type",
        default=config.get("ros2", {}).get("vehicle_status_type", DEFAULT_VEHICLE_STATUS_TYPE),
        help="飞行器状态主题的消息类型，使用完整的模块路径。",
    )
    parser.add_argument(
        "--ros-arg",
        action="append",
        dest="ros_args",
        default=config.get("ros2", {}).get("args", []),
        help="传递给 rclpy.init() 的额外参数，可多次使用。",
    )
    if ROS_PROFILES:
        parser.add_argument(
            "--ros-profile",
            choices=tuple(ROS_PROFILES.keys()),
            default=config.get("ros2", {}).get("profile"),
            help="应用预设的 ROS2 订阅配置（例如 px4_interface）。",
        )
    return parser


def _apply_ros_profile(args: argparse.Namespace) -> dict[str, Any] | None:
    profile_name = getattr(args, "ros_profile", None)
    if profile_name is None:
        return None

    profile = ROS_PROFILES.get(profile_name)
    if profile is None:
        logger.warning("未找到 ROS profile '%s'，忽略", profile_name)
        return None

    if args.mode != "ros2":
        logger.debug("ROS profile %s 强制切换数据源模式为 ros2", profile_name)
        args.mode = "ros2"

    if args.ros_odometry_topic == DEFAULT_ODOMETRY_TOPIC:
        args.ros_odometry_topic = profile["odometry_topic"]
    if args.ros_odometry_type == DEFAULT_ODOMETRY_TYPE:
        args.ros_odometry_type = profile["odometry_type"]
    if args.ros_battery_topic == DEFAULT_BATTERY_TOPIC:
        args.ros_battery_topic = profile["battery_topic"]
    if args.ros_battery_type == DEFAULT_BATTERY_TYPE:
        args.ros_battery_type = profile["battery_type"]
    if args.ros_vehicle_status_topic == DEFAULT_VEHICLE_STATUS_TOPIC:
        args.ros_vehicle_status_topic = profile.get("vehicle_status_topic", DEFAULT_VEHICLE_STATUS_TOPIC)
    if args.ros_vehicle_status_type == DEFAULT_VEHICLE_STATUS_TYPE:
        args.ros_vehicle_status_type = profile.get("vehicle_status_type", DEFAULT_VEHICLE_STATUS_TYPE)

    logger.info("已应用 ROS profile: %s", profile_name)
    return profile


def _make_data_source(
    parser: argparse.ArgumentParser,
    args: argparse.Namespace,
    profile: dict[str, Any] | None,
) -> DataSource:
    if args.mode == "sim":
        return SimDataSource()
    if args.mode == "ros2":
        battery_topic = args.ros_battery_topic or None
        try:
            ros2_kwargs: dict[str, Any] = {
                "namespace": args.ros_namespace,
                "odometry_topic": args.ros_odometry_topic,
                "battery_topic": battery_topic,
                "odometry_msg_type": args.ros_odometry_type,
                "battery_msg_type": args.ros_battery_type,
                "vehicle_status_topic": args.ros_vehicle_status_topic or None,
                "vehicle_status_msg_type": (
                    args.ros_vehicle_status_type if args.ros_vehicle_status_topic else None
                ),
                "ros_args": args.ros_args,
            }
            if profile is not None:
                odometry_parser = profile.get("odometry_parser")
                battery_parser = profile.get("battery_parser")
                if odometry_parser is not None:
                    ros2_kwargs["odometry_parser"] = odometry_parser
                if battery_parser is not None:
                    ros2_kwargs["battery_parser"] = battery_parser
                vehicle_status_parser = profile.get("vehicle_status_parser")
                if vehicle_status_parser is not None:
                    ros2_kwargs["vehicle_status_parser"] = vehicle_status_parser

            return Ros2DataSource(**ros2_kwargs)
        except RuntimeError as exc:
            parser.error(str(exc))
        except ValueError as exc:
            parser.error(f"ROS2 数据源配置无效: {exc}")

    raise ValueError(f"Unknown mode: {args.mode}")


def main(argv: Optional[Sequence[str]] = None) -> None:
    config_parser = _create_config_parser()
    config_args, remaining = config_parser.parse_known_args(argv)
    config = _load_config(config_args.config)
    parser = _create_parser(config)
    args = parser.parse_args(remaining)
    profile = _apply_ros_profile(args)
    _configure_logging_from_args(parser, args)
    _install_global_exception_hook()
    data_source = _make_data_source(parser, args, profile)
    app = UAVDashboardApp(data_source=data_source, poll_interval=args.poll_interval)

    with GracefulShutdown() as shutdown:
        shutdown.register_callback(lambda: _request_app_exit(app))

        try:
            app.run()
        except KeyboardInterrupt:
            logger.info("检测到用户中断，正在退出…")
            shutdown.request_shutdown("用户中断 (Ctrl+C)")
            return
        finally:
            if not shutdown.triggered:
                shutdown.request_shutdown("主循环退出")
            shutdown.wait(timeout=2.0)
            _ensure_data_source_stopped(data_source)


__all__ = ["main"]
