"""Allow running the package via ``python -m uav_tui_dashboard``."""

from __future__ import annotations

from .cli import main


def run() -> None:
    """Entrypoint used by ``python -m uav_tui_dashboard``."""
    main()


if __name__ == "__main__":  # pragma: no cover
    run()
