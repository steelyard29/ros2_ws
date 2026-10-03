"""Tests for the logging configuration and graceful shutdown helpers."""
# pyright: reportMissingImports=false

from __future__ import annotations

import logging
import sys
import textwrap
import threading
from importlib import import_module
from pathlib import Path
from collections.abc import Iterator
from typing import TYPE_CHECKING

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

if TYPE_CHECKING:  # pragma: no cover
    import pytest as _pytest

pytest = import_module("pytest")
configure_logging = import_module("uav_tui_dashboard.logging_config").configure_logging
GracefulShutdown = import_module("uav_tui_dashboard.shutdown").GracefulShutdown


@pytest.fixture(autouse=True)
def reset_logging() -> Iterator[None]:
    """Ensure each test starts with a clean logging configuration."""
    try:
        yield
    finally:
        root = logging.getLogger()
        for handler in list(root.handlers):
            handler.close()
            root.removeHandler(handler)
        logging.shutdown()


def _read_file(path: Path) -> str:
    with path.open("r", encoding="utf-8") as stream:
        return stream.read()


def test_configure_logging_writes_to_expected_files(tmp_path: Path) -> None:
    result = configure_logging(level="DEBUG", console=False, log_dir=tmp_path)

    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)
    root_logger.debug("debug message")
    root_logger.error("error message")

    for handler in logging.getLogger().handlers:
        flush = getattr(handler, "flush", None)
        if callable(flush):
            flush()

    logging.shutdown()

    # runtime handler should capture debug message
    runtime_log = result.handler_files["runtime"]
    error_log = result.handler_files["errors"]

    assert runtime_log.exists()
    assert "debug message" in _read_file(runtime_log)

    assert error_log.exists()
    assert "error message" in _read_file(error_log)


def test_configure_logging_from_toml(tmp_path: Path) -> None:
    config_content = textwrap.dedent(
        """
        level = "WARNING"
        console = false

        [handlers.runtime]
        filename = "custom-info.log"
        level = "INFO"

        [handlers.errors]
        filename = "custom-error.log"
        level = "ERROR"
        """
    )
    config_path = tmp_path / "config.toml"
    config_path.write_text(config_content, encoding="utf-8")

    result = configure_logging(config_path=config_path, log_dir=tmp_path)

    assert result.handler_files["runtime"].name == "custom-info.log"
    assert result.handler_files["errors"].name == "custom-error.log"


def test_graceful_shutdown_triggers_callbacks() -> None:
    shutdown = GracefulShutdown(signals_to_handle=[])
    callback_invoked = threading.Event()

    with shutdown:
        shutdown.register_callback(callback_invoked.set)
        shutdown.request_shutdown("test")
        assert shutdown.triggered is True
        assert callback_invoked.wait(timeout=1.0), "shutdown callback was not invoked"

    # ensure subsequent callbacks run immediately even after shutdown
    second_invoked = threading.Event()
    shutdown.register_callback(second_invoked.set)
    assert second_invoked.wait(timeout=1.0)
