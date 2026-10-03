"""Abstract data source primitives for UAV telemetry providers."""

from __future__ import annotations

import abc
from typing import Any, Callable

from .status import UAVStatus, VehicleState


class DataSource(abc.ABC):
    """Provides UAV telemetry snapshots for the UI layer."""

    @abc.abstractmethod
    async def start(self) -> None:
        """Initialise resources before polling begins."""

    @abc.abstractmethod
    async def stop(self) -> None:
        """Release any resources when polling stops."""

    @abc.abstractmethod
    async def fetch(self) -> UAVStatus:
        """Return the latest UAV status."""


VehicleStateParser = Callable[[Any, VehicleState], VehicleState]

__all__ = ["DataSource", "VehicleStateParser"]
