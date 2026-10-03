"""ROS2-backed telemetry provider for the UAV TUI dashboard."""

from __future__ import annotations

import asyncio
import contextlib
import importlib
import logging
import threading
from typing import Any, Callable, Optional

from .base import DataSource, VehicleStateParser
from .status import UAVStatus, Vector3
from .parsers import (
    parse_battery_message,
    parse_odometry_message,
    parse_vehicle_status_message,
)

logger = logging.getLogger(__name__)


class Ros2DataSource(DataSource):
    """ROS2-backed telemetry provider."""

    def __init__(
        self,
        *,
        node_name: str = "uav_dashboard_listener",
        namespace: str | None = None,
        odometry_topic: str = "/uav/odometry",
        battery_topic: str | None = "/uav/battery",
        odometry_msg_type: str = "nav_msgs.msg.Odometry",
        battery_msg_type: str = "sensor_msgs.msg.BatteryState",
        odometry_parser: Optional[Callable[[Any], tuple[Vector3, Vector3]]] = None,
        battery_parser: Optional[Callable[[Any, UAVStatus], tuple[float, float]]] = None,
        vehicle_status_topic: str | None = None,
        vehicle_status_msg_type: str | None = None,
        vehicle_status_parser: Optional[VehicleStateParser] = None,
        qos_depth: int = 10,
        ros_args: Optional[list[str]] = None,
        spin_timeout: float = 0.1,
    ) -> None:
        self._node_name = node_name
        self._namespace = namespace
        self._odometry_topic = odometry_topic
        self._battery_topic = battery_topic
        self._odometry_msg_type = odometry_msg_type
        self._battery_msg_type = battery_msg_type
        self._odometry_parser = odometry_parser or parse_odometry_message
        self._battery_parser = battery_parser or parse_battery_message
        self._vehicle_status_topic = vehicle_status_topic
        self._vehicle_status_msg_type = vehicle_status_msg_type
        self._vehicle_status_parser = vehicle_status_parser or parse_vehicle_status_message
        self._qos_depth = qos_depth
        self._ros_args = ros_args
        self._spin_timeout = spin_timeout

        self._status = UAVStatus.empty()
        self._status_lock = threading.Lock()
        self._running = False

        # Lazy-imported ROS2 components
        self._rclpy: Any = None
        self._context: Any = None
        self._executor: Any = None
        self._node: Any = None
        self._subscriptions: list[Any] = []
        self._spin_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._odometry_msg_cls: Optional[type[Any]] = None
        self._battery_msg_cls: Optional[type[Any]] = None
        self._vehicle_status_msg_cls: Optional[type[Any]] = None

    async def start(self) -> None:
        if self._running:
            logger.debug("ROS2 数据源已启动，无需重复初始化")
            return

        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._start_blocking)

    def _start_blocking(self) -> None:
        try:
            self._initialise_ros_components()
        except Exception:
            logger.exception("初始化 ROS2 数据源失败")
            raise

        self._stop_event.clear()
        self._spin_thread = threading.Thread(
            target=self._spin_executor,
            name="uav-tui-ros2-spin",
            daemon=True,
        )
        self._spin_thread.start()
        self._running = True
        topics = [self._odometry_topic]
        if self._battery_topic:
            topics.append(self._battery_topic)
        if self._vehicle_status_topic:
            topics.append(self._vehicle_status_topic)
        logger.info("ROS2 数据源已启动，订阅主题: %s", ", ".join(topics))

    def _initialise_ros_components(self) -> None:
        try:
            self._rclpy = importlib.import_module("rclpy")
            context_module = importlib.import_module("rclpy.context")
            executors_module = importlib.import_module("rclpy.executors")
            qos_module = importlib.import_module("rclpy.qos")
            node_module = importlib.import_module("rclpy.node")
        except ImportError as exc:  # pragma: no cover - depends on ROS env
            raise RuntimeError(
                "ROS2 模式需要安装 rclpy 以及对应的 ROS2 运行时，请在 ROS2 环境中运行或安装相关依赖。"
            ) from exc

        QoSProfile = getattr(qos_module, "QoSProfile")
        Node = getattr(node_module, "Node")
        SingleThreadedExecutor = getattr(executors_module, "SingleThreadedExecutor")

        self._odometry_msg_cls = _resolve_message_type(self._odometry_msg_type)
        if self._battery_topic is not None:
            self._battery_msg_cls = _resolve_message_type(self._battery_msg_type)
        if self._vehicle_status_topic and self._vehicle_status_msg_type:
            self._vehicle_status_msg_cls = _resolve_message_type(self._vehicle_status_msg_type)

        self._context = context_module.Context()
        self._rclpy.init(args=self._ros_args, context=self._context)
        self._node = Node(
            self._node_name,
            namespace=self._namespace,
            context=self._context,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        qos_profile = QoSProfile(depth=self._qos_depth)

        self._subscriptions = []
        self._subscriptions.append(
            self._node.create_subscription(
                self._odometry_msg_cls,
                self._odometry_topic,
                self._handle_odometry,
                qos_profile,
            )
        )

        if self._battery_topic and self._battery_msg_cls is not None:
            self._subscriptions.append(
                self._node.create_subscription(
                    self._battery_msg_cls,
                    self._battery_topic,
                    self._handle_battery,
                    qos_profile,
                )
            )
        if (
            self._vehicle_status_topic
            and self._vehicle_status_msg_cls is not None
            and self._vehicle_status_parser is not None
        ):
            self._subscriptions.append(
                self._node.create_subscription(
                    self._vehicle_status_msg_cls,
                    self._vehicle_status_topic,
                    self._handle_vehicle_status,
                    qos_profile,
                )
            )

        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)

    def _spin_executor(self) -> None:
        assert self._executor is not None
        try:
            while not self._stop_event.is_set():
                self._executor.spin_once(timeout_sec=self._spin_timeout)
        except Exception:  # pragma: no cover - defensive logging
            logger.exception("ROS2 executor spin loop terminated unexpectedly")

    async def stop(self) -> None:
        if not self._running:
            return

        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._stop_blocking)

    def _stop_blocking(self) -> None:
        self._stop_event.set()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
            if self._spin_thread.is_alive():  # pragma: no cover - defensive path
                logger.warning("ROS2 executor 线程未在超时内退出，将继续后台运行")
        self._spin_thread = None

        if self._executor is not None:
            with contextlib.suppress(Exception):
                self._executor.remove_node(self._node)
            self._executor.shutdown()
        self._executor = None

        if self._node is not None:
            for subscription in self._subscriptions:
                with contextlib.suppress(Exception):
                    self._node.destroy_subscription(subscription)
            self._subscriptions.clear()
            with contextlib.suppress(Exception):
                self._node.destroy_node()
        self._node = None

        if self._rclpy is not None and self._context is not None:
            try:
                if self._rclpy.ok(context=self._context):
                    self._rclpy.shutdown(context=self._context)
            except Exception:  # pragma: no cover - defensive path
                logger.exception("关闭 ROS2 上下文失败")

        self._context = None
        self._running = False
        logger.info("ROS2 数据源已停止")

    async def fetch(self) -> UAVStatus:
        if not self._running:
            logger.debug("ROS2 数据源尚未启动，返回最近一次缓存的状态")

        with self._status_lock:
            return self._status

    def _handle_odometry(self, message: Any) -> None:
        try:
            position, orientation = self._odometry_parser(message)
        except Exception:
            logger.exception("解析里程计消息失败: %s", type(message).__name__)
            return

        with self._status_lock:
            self._status = self._status.with_position(position).with_orientation(orientation)

    def _handle_battery(self, message: Any) -> None:
        try:
            battery_level, voltage = self._battery_parser(message, self._status)
        except Exception:
            logger.exception("解析电池消息失败: %s", type(message).__name__)
            return

        with self._status_lock:
            self._status = self._status.with_battery(battery_level, voltage)

    def _handle_vehicle_status(self, message: Any) -> None:
        if self._vehicle_status_parser is None:
            return
        try:
            vehicle_state = self._vehicle_status_parser(message, self._status.vehicle_state)
        except Exception:
            logger.exception("解析 VehicleStatus 消息失败: %s", type(message).__name__)
            return

        with self._status_lock:
            self._status = self._status.with_vehicle_state(vehicle_state)


def _resolve_message_type(dotted_path: str) -> type[Any]:
    try:
        module_name, attr = dotted_path.rsplit(".", 1)
    except ValueError as exc:  # pragma: no cover - defensive path
        raise ValueError(f"无效的消息类型路径: {dotted_path}") from exc

    module = importlib.import_module(module_name)
    try:
        msg_cls = getattr(module, attr)
    except AttributeError as exc:
        raise ValueError(f"模块 {module_name!r} 中不存在消息类型 {attr!r}") from exc

    return msg_cls


__all__ = ["Ros2DataSource"]
