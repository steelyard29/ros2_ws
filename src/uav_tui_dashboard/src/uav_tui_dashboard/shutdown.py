"""Utilities for gracefully shutting down the dashboard runtime."""

from __future__ import annotations

import logging
import signal
import threading
from types import FrameType
from typing import Any, Callable, Iterable, List, Optional, cast

LOGGER = logging.getLogger(__name__)


class GracefulShutdown:
    """Coordinate signal handling and shutdown callbacks."""

    def __init__(self, *, signals_to_handle: Optional[Iterable[int]] = None) -> None:
        if signals_to_handle is None:
            default_signals = [signal.SIGINT, signal.SIGTERM]
            if hasattr(signal, "SIGHUP"):
                default_signals.append(getattr(signal, "SIGHUP"))
            signals_to_handle = default_signals

        self._signals = [sig for sig in signals_to_handle if isinstance(sig, int)]
        self._event = threading.Event()
        self._callbacks: List[Callable[[], None]] = []
        self._previous_handlers: dict[int, Any] = {}
        self._lock = threading.RLock()
        self._callbacks_invoked = False

    def __enter__(self) -> "GracefulShutdown":
        self.install()
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        self.restore()
        if exc_type is KeyboardInterrupt:
            self.request_shutdown("用户中断 (Ctrl+C)")
            return True
        return False

    @property
    def triggered(self) -> bool:
        """Whether a shutdown has been requested."""
        return self._event.is_set()

    def install(self) -> None:
        """Install signal handlers for coordinated shutdown."""
        with self._lock:
            for sig in self._signals:
                try:
                    self._previous_handlers[sig] = signal.getsignal(sig)
                    signal.signal(sig, self._handle_signal)
                except (OSError, RuntimeError, ValueError) as exc:  # pragma: no cover - platform specific
                    LOGGER.debug("跳过信号 %s: %s", sig, exc)

    def restore(self) -> None:
        """Restore previous signal handlers."""
        with self._lock:
            for sig, handler in self._previous_handlers.items():
                try:
                    signal.signal(sig, cast(signal.Handlers, handler))
                except (OSError, RuntimeError, ValueError):  # pragma: no cover - platform specific
                    LOGGER.debug("恢复信号 %s 失败", sig)
            self._previous_handlers.clear()

    def register_callback(self, callback: Callable[[], None]) -> None:
        """Register a callback invoked asynchronously when shutdown is requested."""
        invoke_direct: Optional[Callable[[], None]] = None
        with self._lock:
            self._callbacks.append(callback)
            triggered = self._event.is_set()
            callbacks_already_invoked = self._callbacks_invoked
        if triggered:
            if callbacks_already_invoked:
                invoke_direct = callback
            else:
                self._run_callbacks_async()
        if invoke_direct is not None:
            threading.Thread(target=self._invoke_single_callback, args=(invoke_direct,), name="shutdown-callback", daemon=True).start()

    def request_shutdown(self, reason: str) -> None:
        """Trigger the shutdown sequence if not already triggered."""
        should_invoke = False
        with self._lock:
            if not self._event.is_set():
                LOGGER.info("触发关闭：%s", reason)
                self._event.set()
                should_invoke = True
            elif not self._callbacks_invoked and self._callbacks:
                should_invoke = True
        if should_invoke:
            self._run_callbacks_async()

    def wait(self, timeout: Optional[float] = None) -> bool:
        """Block until shutdown is requested or *timeout* elapses."""
        return self._event.wait(timeout)

    def _handle_signal(self, signum: int, _frame: Optional[FrameType]) -> None:  # pragma: no cover - signal handler
        try:
            name = signal.Signals(signum).name
        except ValueError:  # pragma: no cover - defensive fallback
            name = str(signum)
        self.request_shutdown(f"收到系统信号 {name}")

    def _run_callbacks_async(self) -> None:
        with self._lock:
            if self._callbacks_invoked or not self._callbacks:
                return
            self._callbacks_invoked = True
        threading.Thread(target=self._invoke_callbacks, name="shutdown-callbacks", daemon=True).start()

    def _invoke_callbacks(self) -> None:
        for callback in list(self._callbacks):
            try:
                callback()
            except Exception:  # noqa: BLE001
                LOGGER.exception("执行关闭回调失败")
                raise

    def _invoke_single_callback(self, callback: Callable[[], None]) -> None:
        try:
            callback()
        except Exception:  # noqa: BLE001
            LOGGER.exception("执行关闭回调失败")
            raise


__all__ = ["GracefulShutdown"]
