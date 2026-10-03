"""Foreground boot Agent; default describes, never adopts/kills an existing Agent."""
import argparse
import os
from pathlib import Path
from xrce_serial_agent import running_pid, occupants, configure_port

DEVICE='/dev/ttyTHS1'
BAUD=921600
AGENT='/usr/local/bin/MicroXRCEAgent'


def validate_start(exists, existing, held):
    if not exists:raise RuntimeError('DDS serial device absent')
    if existing is not None or held:
        raise RuntimeError('DDS already running or serial occupied; no duplicate/restart')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true')
    a=p.parse_args()
    if not a.execute:
        print('Describe only: one foreground DDS Agent, /dev/ttyTHS1 921600; no EV/control publisher.')
        return 0
    validate_start(Path(DEVICE).exists(),running_pid(DEVICE,BAUD),occupants(DEVICE))
    from px4_runtime_transport import configure
    configure()
    configure_port(DEVICE,BAUD)
    os.execv(AGENT,[AGENT,'serial','-D',DEVICE,'-b',str(BAUD),'-v','4'])


if __name__=='__main__':raise SystemExit(main())
