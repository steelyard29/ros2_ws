"""Textual UI application for the UAV dashboard."""

from __future__ import annotations

import logging
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from rich.text import Text
from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical
from textual.timer import Timer
from textual.widgets import Button, DataTable, Footer, Header, ListItem, ListView, Static

from ..core import DataSource
from ..core.snapshots import FlightSnapshot, FlightSnapshotQueue
from ..core.status import UAVStatus

logger = logging.getLogger(__name__)


class UAVDashboardApp(App):
    """Textual application that renders UAV telemetry."""

    CSS_PATH = Path(__file__).with_name("dashboard.css")
    FIELD_COLUMN_KEY = "field"
    VALUE_COLUMN_KEY = "value"

    ARMING_STATE_LABELS = {
        0: "INIT",
        1: "STANDBY",
        2: "ARMED",
        3: "STANDBY ERROR",
        4: "SHUTDOWN",
        5: "IN-AIR RESTORE",
    }

    NAV_STATE_LABELS = {
        0: "MANUAL",
        1: "ALTCTL",
        2: "POSCTL",
        3: "AUTO MISSION",
        4: "AUTO LOITER",
        5: "AUTO RTL",
        6: "AUTO LAND",
        7: "AUTO RTGS",
        8: "AUTO READY",
        9: "AUTO TAKEOFF",
        10: "AUTO LAND ENG",
        11: "ACRO",
        12: "DESCEND",
        13: "TERMINATION",
        14: "OFFBOARD",
    }

    def __init__(self, data_source: DataSource, *, poll_interval: float = 1.0) -> None:
        super().__init__()
        self._data_source = data_source
        self._poll_interval = poll_interval
        self._status = UAVStatus.empty()
        self._status_timer: Optional[Timer] = None
        self._snapshot_queue = FlightSnapshotQueue(maxlen=20)

    def compose(self) -> ComposeResult:
        table = DataTable(id="status_table")
        snapshot_panel = Vertical(
            Static("Flight Snapshots", id="snapshot_title"),
            ListView(id="snapshot_list"),
            Horizontal(
                Button("Capture", id="snapshot_capture", variant="success"),
                Button("Export", id="snapshot_export", variant="primary"),
                Button("Clear", id="snapshot_clear", variant="warning"),
                id="snapshot_actions",
            ),
            id="snapshot_panel",
        )

        yield Header(show_clock=True)
        yield Horizontal(table, snapshot_panel, id="content_area")
        yield Footer()

    async def on_mount(self) -> None:
        await self._data_source.start()
        table = self.query_one("#status_table", DataTable)
        table.add_column("Field", key=self.FIELD_COLUMN_KEY)
        table.add_column("Value", key=self.VALUE_COLUMN_KEY)
        self._add_rows(table)
        self._status_timer = self.set_interval(self._poll_interval, self._tick, name="status_poll")
        self._refresh_snapshot_list()

    async def on_unmount(self) -> None:
        if self._status_timer is not None:
            self._status_timer.stop()
        await self._data_source.stop()

    def _add_rows(self, table: DataTable) -> None:
        table.add_row("Position X", "0.00", key="position_x")
        table.add_row("Position Y", "0.00", key="position_y")
        table.add_row("Position Z", "0.00", key="position_z")
        table.add_row("Roll", "0.00", key="roll")
        table.add_row("Pitch", "0.00", key="pitch")
        table.add_row("Yaw", "0.00", key="yaw")
        table.add_row("Battery Level", "100.0%", key="battery_level")
        table.add_row("Voltage", "12.50V", key="voltage")
        table.add_row("Vehicle Link", self._bool_text(False, true_style="bold green"), key="vehicle_valid")
        table.add_row("Telemetry Timestamp", self._timestamp_text(None), key="vehicle_timestamp")
        table.add_row("Arming State", self._state_text(None, self.ARMING_STATE_LABELS), key="vehicle_arming_state")
        table.add_row("Navigation State", self._state_text(None, self.NAV_STATE_LABELS), key="vehicle_nav_state")
        table.add_row(
            "Failsafe",
            self._bool_text(False, true_label="ACTIVE", false_label="OK", true_style="bold red", false_style="bold green"),
            key="vehicle_failsafe",
        )
        table.add_row(
            "Pre-flight Checks",
            self._bool_text(None, true_label="PASS", false_label="FAIL"),
            key="vehicle_preflight",
        )

    async def _tick(self) -> None:
        try:
            status = await self._data_source.fetch()
        except Exception:  # pragma: no cover - defensive path
            logger.exception("Failed to fetch UAV status")
            return

        self._status = status
        self._update_table(status)

    def _update_table(self, status: UAVStatus) -> None:
        table = self.query_one("#status_table", DataTable)
        table.update_cell("position_x", self.VALUE_COLUMN_KEY, f"{status.position[0]:.2f}")
        table.update_cell("position_y", self.VALUE_COLUMN_KEY, f"{status.position[1]:.2f}")
        table.update_cell("position_z", self.VALUE_COLUMN_KEY, f"{status.position[2]:.2f}")
        table.update_cell("roll", self.VALUE_COLUMN_KEY, f"{status.orientation[0]:.2f}")
        table.update_cell("pitch", self.VALUE_COLUMN_KEY, f"{status.orientation[1]:.2f}")
        table.update_cell("yaw", self.VALUE_COLUMN_KEY, f"{status.orientation[2]:.2f}")
        table.update_cell("battery_level", self.VALUE_COLUMN_KEY, f"{status.battery_level:.1f}%")
        table.update_cell("voltage", self.VALUE_COLUMN_KEY, f"{status.voltage:.2f}V")
        vehicle_state = status.vehicle_state
        table.update_cell("vehicle_valid", self.VALUE_COLUMN_KEY, self._bool_text(vehicle_state.valid, true_style="bold green"))
        table.update_cell("vehicle_timestamp", self.VALUE_COLUMN_KEY, self._timestamp_text(vehicle_state.timestamp))
        table.update_cell(
            "vehicle_arming_state",
            self.VALUE_COLUMN_KEY,
            self._state_text(vehicle_state.arming_state, self.ARMING_STATE_LABELS),
        )
        table.update_cell(
            "vehicle_nav_state",
            self.VALUE_COLUMN_KEY,
            self._state_text(vehicle_state.nav_state, self.NAV_STATE_LABELS),
        )
        table.update_cell(
            "vehicle_failsafe",
            self.VALUE_COLUMN_KEY,
            self._bool_text(
                vehicle_state.failsafe,
                true_label="ACTIVE",
                false_label="OK",
                true_style="bold red",
                false_style="bold green",
            ),
        )
        table.update_cell(
            "vehicle_preflight",
            self.VALUE_COLUMN_KEY,
            self._bool_text(
                vehicle_state.pre_flight_checks_pass,
                true_label="PASS",
                false_label="FAIL",
            ),
        )

    async def on_button_pressed(self, event: Button.Pressed) -> None:
        button_id = event.button.id
        if button_id == "snapshot_capture":
            snapshot = self._snapshot_queue.capture(self._status)
            self._refresh_snapshot_list()
            self.notify(f"Captured snapshot at {self._format_snapshot_time(snapshot)}")
        elif button_id == "snapshot_export":
            try:
                path = self._snapshot_queue.export()
            except ValueError:
                self.notify("No snapshots to export yet.", severity="warning")
            else:
                self.notify(f"Exported {len(self._snapshot_queue)} snapshots → {path}", severity="information")
        elif button_id == "snapshot_clear":
            if len(self._snapshot_queue) == 0:
                self.notify("Snapshot queue already empty.", severity="warning")
                return
            self._snapshot_queue.clear()
            self._refresh_snapshot_list()
            self.notify("Cleared all snapshots.", severity="information")

    def _refresh_snapshot_list(self) -> None:
        list_view = self.query_one("#snapshot_list", ListView)
        if hasattr(list_view, "clear"):
            list_view.clear()  # type: ignore[attr-defined]
        else:  # pragma: no cover - compatibility path
            for child in list(list_view.children):
                child.remove()
        snapshots = self._snapshot_queue.list()
        if not snapshots:
            list_view.append(ListItem(Static("No snapshots yet", classes="snapshot-empty")))
            return

        for index, snapshot in enumerate(snapshots, start=1):
            list_view.append(ListItem(Static(self._format_snapshot_entry(index, snapshot), classes="snapshot-entry")))

    def _format_snapshot_entry(self, index: int, snapshot: FlightSnapshot) -> str:
        ts_text = self._format_snapshot_time(snapshot)
        return f"{index:02d} · {ts_text} · {snapshot.status.battery_level:.1f}%"

    @staticmethod
    def _format_snapshot_time(snapshot: FlightSnapshot) -> str:
        timestamp = snapshot.captured_at
        if timestamp.tzinfo is None:
            timestamp = timestamp.replace(tzinfo=timezone.utc)
        return timestamp.astimezone(timezone.utc).strftime("%H:%M:%S UTC")

    @staticmethod
    def _bool_text(
        value: Optional[bool],
        *,
        true_label: str = "Yes",
        false_label: str = "No",
        none_label: str = "Unknown",
        true_style: str = "bold green",
        false_style: str = "bold red",
        none_style: str = "dim italic",
    ) -> Text:
        if value is None:
            return Text(none_label, style=none_style)
        return Text(true_label if value else false_label, style=true_style if value else false_style)

    @staticmethod
    def _timestamp_text(timestamp: Optional[datetime]) -> Text:
        if timestamp is None:
            return Text("—", style="dim")
        if timestamp.tzinfo is None:
            timestamp = timestamp.replace(tzinfo=timezone.utc)
        ts_utc = timestamp.astimezone(timezone.utc)
        display = ts_utc.strftime("%Y-%m-%d %H:%M:%S UTC")
        return Text(display, style="cyan")

    @staticmethod
    def _state_text(value: Optional[int], labels: dict[int, str]) -> Text:
        if value is None:
            return Text("—", style="dim")
        label = labels.get(value)
        if label is None:
            return Text(str(value), style="bold")
        return Text(f"{value} · {label}", style="bold")


__all__ = ["UAVDashboardApp"]
