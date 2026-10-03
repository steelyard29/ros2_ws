#!/usr/bin/env python3
"""Explicit EV-only prop-off entrypoint. Not a live flight controller.

Only EV can reach /fmu/in. No real heartbeat/setpoint/mode/arm/land command.
Requires --prop-off, current AUX export, and live_flight_enabled=false.
"""
from flight_runtime import main

if __name__ == '__main__':
    raise SystemExit(main(bench_ev=True))
