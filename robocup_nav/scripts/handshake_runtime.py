#!/usr/bin/env python3
"""Explicit shadow/isolated disarmed handshake runtime. No real PX4 inputs written."""
from flight_runtime import main

if __name__ == '__main__':
    raise SystemExit(main(handshake=True))
