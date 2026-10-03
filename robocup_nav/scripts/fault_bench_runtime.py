#!/usr/bin/env python3
"""Explicit prop-off kill-stop test. No mode, ARM, LAND or actuator commands.

Requires a separate authorization and current parameter export. Does not
start/stop camera, Agent, GPIO, or inject faults. Never executed by unit tests.
"""
from handshake_bench_runtime import main

if __name__ == '__main__':
    raise SystemExit(main(fault_bench=True))
