"""Owner lease for a persistent reader; no ROS, processes or hardware startup.

The owner renews from its serviced loop, not an independent heartbeat thread.
One-second expiry is process cleanup, not the shorter EV freshness threshold.
Expired/invalid/stopped leases never recover inside the same reader instance.
"""
import json
import math
import os
from pathlib import Path
import time
from disarmed_ev_lifecycle import atomic_json


def clock_identity():
    return dict(boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                time_namespace=os.readlink('/proc/self/ns/time'))


class ReaderLease:
    def __init__(self,session,identity,*,clock=time.monotonic):
        if not session or set(identity)!={'boot_id','time_namespace'} or not all(identity.values()):
            raise ValueError('invalid reader lease identity')
        self.session,self.identity,self.clock=session,dict(identity),clock
        self.seq=0;self.renewed=None;self.reason=None

    def check(self,data):
        if self.reason is not None:return False
        try:
            if (data['session']!=self.session or data['clock_identity']!=self.identity
                    or type(data['stop']) is not bool or type(data['seq']) is not int
                    or data['seq']<1 or data['seq']<self.seq):
                raise ValueError('reader lease identity/schema/sequence mismatch')
            stamp=data['renewed_at'];age=self.clock()-stamp
            if not math.isfinite(age) or not 0<=age<=1.:
                raise ValueError('reader owner lease expired or future')
            if self.renewed is not None and (
                    stamp<self.renewed or (data['seq']==self.seq and stamp!=self.renewed)):
                raise ValueError('reader lease replay/mutation')
            if data['stop']:
                self.reason='owner_requested_stop';return False
            self.seq=data['seq'];self.renewed=stamp
            return True
        except (KeyError,TypeError,ValueError) as exc:
            self.reason=str(exc);return False


class ReaderLeaseOwner:
    def __init__(self,path,session,*,identity=None,clock=time.monotonic):
        self.path=Path(path);self.session=session;self.clock=clock
        self.identity=clock_identity() if identity is None else dict(identity)
        ReaderLease(session,self.identity,clock=clock)
        self.seq=0;self.stopped=False

    def renew(self,*,stop=False):
        if self.stopped:raise RuntimeError('reader owner already stopped; no renewal')
        self.seq+=1
        atomic_json(self.path,dict(session=self.session,seq=self.seq,stop=bool(stop),
            renewed_at=self.clock(),clock_identity=self.identity))
        self.stopped=bool(stop)


def service_loop(lease,control,spin,pump,*,deadline=None,clock=time.monotonic):
    """Pump only under a live lease, including after a potentially slow spin.

    Optional deadline bounds isolated tests. Production ownership uses renewal,
    never repeatedly launches bounded bench runs. Exceptions propagate to the
    worker's single cleanup/final-receipt path.
    """
    def alive():
        if deadline is not None and clock()>=deadline:
            lease.reason='isolated_deadline';return False
        try:data=json.loads(Path(control).read_text())
        except (OSError,ValueError) as exc:
            lease.reason='reader owner control unavailable: '+str(exc);return False
        return lease.check(data)
    while alive():
        spin()
        if not alive():break
        pump()
    return lease.reason
