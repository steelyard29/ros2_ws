"""Persistent EV-owner core, ROS-free and NOT a runnable flight service.

No commands, publishers or automatic restart. Runtime integration must enforce
unique endpoint identities, call watchdog, and acknowledge only actual sends.
Candidate generation is deliberately separate from successful dispatch.
"""
import math
from flight_ev_adapter import FlightEvAdapter


class ResidentEvCore:
    def __init__(self, started, session):
        if not session or not math.isfinite(started):raise ValueError('invalid session')
        self.session=session
        self.adapter=FlightEvAdapter(started)
        self.ground_seen=False
        self.sources=None
        self.owner_at=None
        self.pending=None
        self.sent_count=0
        self.last_sent_at=None
        self.last_sent_stamp=None

    @property
    def fault(self):return self.adapter.fault

    def stop(self, reason):
        self.pending=None
        self.adapter.trip(reason)
        return False

    def ownership(self, identities, ev_publishers, now):
        if self.adapter.inhibit:return False
        required={'vio','tracking','status','flags','land'}
        if (set(identities)!=required or any(not v for v in identities.values())
                or ev_publishers!=1):
            return self.stop('missing/duplicate EV owner or source identity')
        current={k:tuple(v) for k,v in identities.items()}
        if self.sources is not None and current!=self.sources:
            return self.stop('source generation changed; new session required')
        self.sources=current;self.owner_at=now
        return True

    def vehicle(self, arming_state, landed, now):
        if self.adapter.inhibit:return False
        if not self.ground_seen:
            if arming_state!=1 or landed is not True:
                return self.stop('resident EV must start disarmed on ground')
            self.ground_seen=True
        if arming_state==2 and not self.sent_count:
            return self.stop('armed before actual EV delivery')
        return self.adapter.on_vehicle(arming_state,now)

    def flags(self, ev_height, ev_velocity, now):
        return self.adapter.on_flags(ev_height,ev_velocity,now)

    def tracking(self, state, stamp_ns, now):
        return self.adapter.on_tracking(state,stamp_ns,now)

    def watchdog(self, now):
        if self.adapter.inhibit:return False
        if self.owner_at is None or not 0<=now-self.owner_at<=.5:
            return self.stop('resident EV ownership stale')
        return self.adapter.watchdog(now)

    def pose(self, stamp_ns, now_ns, now, position, quaternion, frame_ok=True):
        if self.adapter.inhibit:return None
        if self.pending is not None:
            self.stop('previous EV send not acknowledged');return None
        age=(now_ns-stamp_ns)/1e9
        if not math.isfinite(age) or not -.05<=age<=.25:
            self.stop('resident EV source age invalid');return None
        if not self.ground_seen:return None
        if self.owner_at is None or not 0<=now-self.owner_at<=.5:
            self.stop('resident EV ownership stale');return None
        # Sample-in-hand interval checks remain in the established adapter.
        result=self.adapter.on_pose(stamp_ns,age*1000,now,position,quaternion,
                                    frame_ok=frame_ok,now_ns=now_ns)
        if result is not None:self.pending=result.timestamp_sample
        return result

    def dispatched(self, sample_us, now):
        if self.adapter.inhibit:return False
        if self.pending is None or sample_us!=self.pending:
            return self.stop('unexpected EV dispatch acknowledgement')
        self.pending=None;self.sent_count+=1
        self.last_sent_at=now;self.last_sent_stamp=sample_us
        return True

    def health(self, now):
        return dict(session=self.session,fault=self.fault,sent_count=self.sent_count,
            last_sent_stamp_us=self.last_sent_stamp,
            recent_dispatch=bool(not self.adapter.inhibit and self.last_sent_at is not None
                                 and 0<=now-self.last_sent_at<=.3),
            flight_ready=False)
