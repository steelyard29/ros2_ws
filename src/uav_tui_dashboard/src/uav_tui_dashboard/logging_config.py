"""Logging configuration helpers for the UAV TUI dashboard."""

from __future__ import annotations

import json
import logging
import logging.config
import logging.handlers
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

from platformdirs import user_log_dir

import tomli as tomllib  # type: ignore[no-redef]

LOGGER = logging.getLogger(__name__)

DEFAULT_LOG_FORMAT = "%(asctime)s %(levelname)s [%(name)s] %(message)s"
DEFAULT_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"
DEFAULT_INFO_FILE = "uav-dashboard.log"
DEFAULT_ERROR_FILE = "uav-dashboard-errors.log"
DEFAULT_MAX_BYTES = 5 * 1024 * 1024
DEFAULT_BACKUP_COUNT = 5


@dataclass
class FileHandlerSettings:
    """Configuration for a single rotating file handler."""

    filename: str
    level: str = "INFO"
    max_bytes: int = DEFAULT_MAX_BYTES
    backup_count: int = DEFAULT_BACKUP_COUNT
    formatter: str = "detailed"

    def resolved_filename(self, directory: Path) -> Path:
        """Return the full path for the configured file inside *directory*."""
        return directory / self.filename


@dataclass
class LoggingSettings:
    """Full logging configuration for the application."""

    level: str = "INFO"
    console: bool = True
    directory: Optional[Path] = None
    format: str = DEFAULT_LOG_FORMAT
    datefmt: str = DEFAULT_DATE_FORMAT
    handlers: Dict[str, FileHandlerSettings] = field(default_factory=dict)

    @classmethod
    def default(cls) -> "LoggingSettings":
        """Return default settings with runtime and error file handlers."""
        return cls(
            handlers={
                "runtime": FileHandlerSettings(filename=DEFAULT_INFO_FILE, level="DEBUG"),
                "errors": FileHandlerSettings(
                    filename=DEFAULT_ERROR_FILE,
                    level="ERROR",
                ),
            }
        )

    @classmethod
    def from_mapping(cls, mapping: Mapping[str, Any]) -> "LoggingSettings":
        """Construct settings from a generic mapping (e.g. parsed TOML/JSON)."""
        settings = cls.default()

        if "level" in mapping:
            settings.level = str(mapping["level"]).upper()
        if "console" in mapping:
            settings.console = bool(mapping["console"])
        if "format" in mapping:
            settings.format = str(mapping["format"])
        if "datefmt" in mapping:
            settings.datefmt = str(mapping["datefmt"])
        if "directory" in mapping and mapping["directory"]:
            settings.directory = Path(str(mapping["directory"]))

        handlers_mapping = mapping.get("handlers")
        if isinstance(handlers_mapping, Mapping):
            new_handlers: Dict[str, FileHandlerSettings] = {}
            for name, raw in handlers_mapping.items():
                if not isinstance(raw, Mapping):
                    raise ValueError(f"Handler '{name}' must be a mapping")
                if "filename" not in raw:
                    raise ValueError(f"Handler '{name}' must define a filename")
                new_handlers[name] = FileHandlerSettings(
                    filename=str(raw["filename"]),
                    level=str(raw.get("level", "INFO")).upper(),
                    max_bytes=int(raw.get("max_bytes", DEFAULT_MAX_BYTES)),
                    backup_count=int(raw.get("backup_count", DEFAULT_BACKUP_COUNT)),
                    formatter=str(raw.get("formatter", "detailed")),
                )
            settings.handlers = new_handlers
        return settings

    def merge_overrides(
        self,
        *,
        level: Optional[str] = None,
        console: Optional[bool] = None,
        directory: Optional[Path] = None,
    ) -> None:
        """Override specific fields if corresponding CLI arguments are provided."""
        if level is not None:
            self.level = level.upper()
        if console is not None:
            self.console = console
        if directory is not None:
            self.directory = directory


@dataclass(frozen=True)
class LoggingSetupResult:
    """Result information returned after configuring logging."""

    directory: Path
    handler_files: Dict[str, Path]


def _load_mapping_from_file(path: Path) -> Mapping[str, Any]:
    suffix = path.suffix.lower()
    if suffix in {".toml", ".tml"}:
        with path.open("rb") as stream:
            return tomllib.load(stream)
    if suffix == ".json":
        with path.open("r", encoding="utf-8") as stream:
            return json.load(stream)
    raise ValueError(f"Unsupported log config format: {path.suffix}")


def _ensure_directory(directory: Optional[Path]) -> Path:
    base_dir = directory or Path(user_log_dir("uav-tui-dashboard", "UAV"))
    base_dir.mkdir(parents=True, exist_ok=True)
    return base_dir


def _build_dict_config(settings: LoggingSettings, directory: Path) -> Dict[str, Any]:
    formatter_names = {handler.formatter for handler in settings.handlers.values()} | {"detailed"}

    config: Dict[str, Any] = {
        "version": 1,
        "disable_existing_loggers": False,
        "formatters": {
            name: {
                "format": settings.format,
                "datefmt": settings.datefmt,
            }
            for name in formatter_names
        },
        "handlers": {},
        "root": {
            "level": settings.level,
            "handlers": [],
        },
    }

    for handler_name, handler_settings in settings.handlers.items():
        handler_path = handler_settings.resolved_filename(directory)
        config["handlers"][handler_name] = {
            "class": "logging.handlers.RotatingFileHandler",
            "level": handler_settings.level,
            "formatter": handler_settings.formatter,
            "filename": str(handler_path),
            "maxBytes": handler_settings.max_bytes,
            "backupCount": handler_settings.backup_count,
            "encoding": "utf-8",
        }
        config["root"]["handlers"].append(handler_name)

    if settings.console:
        config["formatters"].setdefault(
            "console",
            {
                "format": "%(levelname)s: %(message)s",
                "datefmt": settings.datefmt,
            },
        )
        config["handlers"]["console"] = {
            "class": "logging.StreamHandler",
            "level": settings.level,
            "formatter": "console",
            "stream": "ext://sys.stdout",
        }
        config["root"]["handlers"].append("console")

    return config


def configure_logging(
    *,
    level: Optional[str] = None,
    console: Optional[bool] = None,
    log_dir: Optional[Path] = None,
    config_path: Optional[Path] = None,
) -> LoggingSetupResult:
    """Initialise Python logging based on defaults, config file, and CLI overrides."""

    settings = LoggingSettings.default()

    if config_path is not None:
        mapping = _load_mapping_from_file(config_path)
        settings = LoggingSettings.from_mapping(mapping)

    settings.merge_overrides(
        level=level,
        console=console,
        directory=log_dir,
    )

    target_directory = _ensure_directory(settings.directory)

    dict_config = _build_dict_config(settings, target_directory)
    logging.config.dictConfig(dict_config)

    handler_files = {
        name: cfg.resolved_filename(target_directory)
        for name, cfg in settings.handlers.items()
    }

    LOGGER.debug("Logging configured: directory=%s handlers=%s", target_directory, handler_files)

    return LoggingSetupResult(directory=target_directory, handler_files=handler_files)


__all__ = ["configure_logging", "LoggingSetupResult", "LoggingSettings", "FileHandlerSettings"]
