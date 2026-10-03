#!/usr/bin/env python3
"""Explicit navigation task entry. Default CHECK ONLY, no ROS/device startup.

The live option requires separate reviewed task evidence, current original
release, consumed one-use session and a local interactive operator. Never used
by automated tests. Does not start cameras/Agent or modify PX4 parameters.
"""
import argparse
import fcntl
import json
from pathlib import Path
from flight_release import ROOT
from task_flight_release import validate


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--release',type=Path,required=True)
    p.add_argument('--params',type=Path,required=True)
    p.add_argument('--authorize-real-flight',action='store_true')
    p.add_argument('--resident-status',type=Path,
                   help='bind existing resident EV; task owns only control outputs')
    a=p.parse_args()
    try:
        permit,params=validate(a.release,a.params,ROOT/'config/platform.yaml')
        binding=None
        if a.resident_status:
            from resident_status_binding import ResidentStatusBinding
            binding=ResidentStatusBinding(a.resident_status)
        if not a.authorize_real_flight:
            print(json.dumps(dict(release_consistent=True,flight_started=False,
                profile='roundtrip_1m_2p5m',landing='H alignment then PX4 AUTO_LAND')))
            return 0
        from live_flight_runtime import confirm,run
        lock_path=('/tmp/robocup_task_control.lock' if binding is not None
                   else '/tmp/robocup_flight_runtime_shadow.lock')
        with open(lock_path,'a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            confirm(permit.session_id)
            if binding is not None:binding.check()
            permit.consume(ROOT/'evidence/live_release_consumed')
            return run(permit,params,task=True,resident_binding=binding)
    except (ValueError,OSError,KeyError,TypeError,RuntimeError) as exc:p.error(str(exc))


if __name__=='__main__':raise SystemExit(main())
