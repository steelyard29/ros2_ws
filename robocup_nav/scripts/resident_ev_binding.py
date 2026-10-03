"""Exclusive resident EV owner, using the existing legacy flight-output lock.

No ROS or commands. Holding this lock excludes existing bench/live entrypoints;
the future external-EV task entry must own a separate control-only lock.
"""
import fcntl
from pathlib import Path
from resident_vio_input import ScopedVioInput
from resident_vio_transport import ResidentVioPort

LOCK_PATH=Path('/tmp/robocup_flight_runtime_shadow.lock')


class ResidentEvOwner:
    def __init__(self,session):
        if not isinstance(session,str) or not session:raise ValueError('missing EV session')
        self.session=session;self._file=None;self.used=False

    @property
    def held(self):return self._file is not None and not self._file.closed

    def acquire(self):
        if self.used:raise RuntimeError('EV owner cannot reacquire in same session')
        self.used=True
        handle=LOCK_PATH.open('a')
        try:fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except Exception:handle.close();raise
        self._file=handle

    def close(self):
        if self._file:self._file.close()

    def validate(self,session,port):
        if not self.held or session!=self.session:
            raise ValueError('resident EV ownership/session invalid')
        if (type(port) is not ScopedVioInput or port.isolated or port.domain_id!=176
                or type(port._node) is not ResidentVioPort
                or port._node.session!=session or port._node.fault):
            raise ValueError('resident EV requires same-session real VIO transport')
