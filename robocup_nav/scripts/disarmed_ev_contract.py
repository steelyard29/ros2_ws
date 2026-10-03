"""Disarmed, stationary EV commissioning only; no controller dispatch.

Keeps the existing BenchEvGate (RC/Position/landed/height/ownership) intact.
Additional limits: <=50 s publication budget, 250 ms vision age, 500 ms
PX4 source age, per-topic receipt budgets, 10 s startup. These are commissioning limits, NOT
certification for flight. A fault permanently closes this instance.
"""
import math
from collections import deque
from flight_runtime_core import FlightRuntimeCore
from flight_output_boundary import BenchEvGate

# Stationary commissioning: match BenchEvGate, not a flight timeout contract.
# Observed periods: status ~0.51 s, flags/land ~1.01 s, RC <=0.032 s.
RECEIPT_LIMITS = dict(status=1.5, flags=2.0, land=2.0, rc=0.5)


class DisarmedEvSession:
    def __init__(self, started, deadline, session, aux_params=None):
        if not all(math.isfinite(x) for x in (started, deadline)) or not 0 < deadline-started <= 50:
            raise ValueError('active budget must be positive and <=50 seconds')
        if not session:
            raise ValueError('session identity required')
        self.started, self.deadline, self.session = started, deadline, session
        self.core = FlightRuntimeCore(started, aux_params=aux_params)
        self.gate = BenchEvGate()
        self.fault = None
        self.sequence = 0
        self.receipts = {}
        self.receipt_stats = {}
        self.receipt_fault = None
        self.sent = 0
        self.vision_trace = deque(maxlen=4096)

    def stop(self, reason):
        self.fault = self.fault or reason
        self.core.trip(self.fault)
        return False

    def clock_ok(self, now):
        if self.fault:
            return False
        if not math.isfinite(now) or now < self.started or now >= self.deadline:
            return self.stop('active deadline or invalid clock')
        return True

    def telemetry(self, name, stamp, age, now, callback):
        if not self.clock_ok(now):
            return False
        if not self.core.receive(name, stamp, now, age):
            if self.core.fault:
                self.stop(self.core.fault)
            return False
        stats = self.receipt_stats.setdefault(name, dict(count=0, max_gap_s=0., max_source_age_s=age))
        if name in self.receipts:
            stats['max_gap_s'] = max(stats['max_gap_s'], now-self.receipts[name])
        stats['count'] += 1
        stats['max_source_age_s'] = max(stats['max_source_age_s'], age)
        self.receipts[name] = now
        callback()
        if self.core.sample.armed or self.core.fault or self.core.ev.inhibit:
            return self.stop(self.core.fault or 'aircraft armed or EV inhibited')
        return True

    def timing_snapshot(self, now):
        return {name: dict(limit_s=limit,
                    age_s=now-self.receipts[name] if name in self.receipts else None,
                    **self.receipt_stats.get(name, {}))
                for name, limit in RECEIPT_LIMITS.items()}

    def watchdog(self, now, *, vision_pending=False):
        if not self.clock_ok(now):
            return False
        if self.core.fault or self.core.ev.inhibit:
            return self.stop(self.core.fault or self.core.ev.fault)
        timing = self.timing_snapshot(now)
        stale = [name for name, row in timing.items()
                 if row['age_s'] is None or not 0 <= row['age_s'] <= row['limit_s']]
        if stale and self.sent:
            name = stale[0]
            self.receipt_fault = dict(topic=name, at=now, **timing[name], all_topics=timing)
            return self.stop('safety telemetry receipt stale: '+name)
        # Pipe silence of 1 s means the reader stopped. The 0.3 s source-interval
        # gate still rejects a real vision hole when the next sample arrives.
        # Live flight keeps its own 0.3 s idle check.
        if not vision_pending and not self.core.ev.watchdog(now, idle_s=1.0):
            return self.stop(self.core.ev.fault)
        ready = self.gate.check(self.core, now)
        if self.gate.fault:
            return self.stop(self.gate.fault)
        if now-self.started >= 10 and not self.sent:
            return self.stop('startup timeout before any EV publication')
        return ready and not stale

    def packet(self, data, now, now_ns):
        guard = self.core.ev.guard
        kind = data.get('kind')
        before = guard.pose_stamp if kind == 'pose' else guard.tracking_stamp
        result = self._packet(data, now, now_ns)
        after = guard.pose_stamp if kind == 'pose' else guard.tracking_stamp
        stamp = data.get('stamp_ns')
        self.vision_trace.append(dict(kind=kind, seq=data.get('seq'), stamp_ns=stamp,
            host_dispatch_monotonic_s=now,
            host_receipt_monotonic_s=data.get('host_receipt_monotonic_s'),
            reader_receipt_monotonic_s=data.get('reader_receipt_monotonic_s'),
            reader_enqueued_monotonic_s=data.get('reader_enqueued_monotonic_s'),
            source_gap_s=(stamp-before)/1e9 if type(stamp) is int and before else None,
            source_age_s=(now_ns-stamp)/1e9 if type(stamp) is int else None,
            accepted_by_guard=after != before, ev_candidate=result is not None,
            blockers=list(self.gate.blockers), fault=self.fault))
        return result

    def _packet(self, data, now, now_ns):
        if not self.clock_ok(now):
            return None
        try:
            if data['session'] != self.session or type(data['seq']) is not int or data['seq'] != self.sequence+1:
                raise ValueError('session/sequence mismatch')
            self.sequence = data['seq']
            if data['kind'] == 'fault':
                self.stop('sensor reader: '+str(data['reason']))
                return None
            stamp = data['stamp_ns']
            if type(stamp) is not int or stamp <= 0 or not -.05 <= (now_ns-stamp)/1e9 <= .25:
                raise ValueError('stale/future/invalid sensor source time')
            if data['kind'] == 'tracking':
                self.core.tracking(data['state'], stamp, now)
                if self.core.ev.inhibit:
                    self.stop(self.core.ev.fault)
                return None
            if data['kind'] != 'pose' or data['frame'] != 'odom' or data['child'] != 'base_link':
                raise ValueError('unexpected sensor type or frame')
            if not self.watchdog(now, vision_pending=True):
                return None
            result = self.core.pose(stamp, (now_ns-stamp)/1e6, now, data['position'],
                                    data['quaternion'], True, now_ns)
            if self.core.fault or self.core.ev.inhibit:
                self.stop(self.core.fault or self.core.ev.fault)
                return None
            return result
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            self.stop('invalid sensor packet: '+str(exc))
            return None

    def dispatched(self):
        self.sent += 1
        self.gate.sent_ev()
