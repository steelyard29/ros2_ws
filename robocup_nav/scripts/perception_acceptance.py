"""ROS-free, one-shot startup/measurement gate; never controls hardware.

Original rollout limits: VIO receipt/source gap .3s, age .25s; depth age
.5s; map receipt gap/age .5s. Depth idle .5s also rejects silent input.
One second plus 30 VIO samples is startup qualification, NOT flight proof.
"""
import math


class AcceptanceWindow:
    def __init__(self, started, navigation=False, startup_s=18., duration_s=12.):
        self.started = started
        self.startup_s = startup_s
        self.duration_s = duration_s
        self.limits = {'vio': (.3, .25, .3), 'depth': (.5, .5, None)}
        if navigation:
            self.limits['map'] = (.5, .5, None)
        self.phase = 'WAIT_INPUTS'
        self.last = {}
        self.counts = {k: 0 for k in self.limits}
        self.ready_since = None
        self.measure_started = None
        self.finished = None
        self.fault = None
        self.events = []

    def reject(self, now, reason):
        self.events.append({'at': now, 'phase': self.phase, 'reason': reason})
        if self.phase == 'MEASURE':
            self.phase = 'FAILED'
            self.fault = reason
            self.finished = now
        elif self.phase == 'WAIT_INPUTS':
            self.ready_since = None
            self.counts = {k: 0 for k in self.limits}

    def sample(self, key, now, stamp_ns, age):
        if key not in self.limits or self.phase in ('FAILED', 'PASSED'):
            return
        gap, max_age, source_gap = self.limits[key]
        old = self.last.get(key)
        valid = (math.isfinite(now) and math.isfinite(age) and stamp_ns > 0
                 and 0 <= age <= max_age)
        reason = key + ': invalid stamp/age'
        if valid and old:
            dt = (stamp_ns - old[1]) / 1e9
            valid = (now >= old[0] and now-old[0] <= gap and dt > 0
                     and (source_gap is None or dt <= source_gap))
            reason = key + ': receipt/source discontinuity'
        if not valid:
            self.reject(now, reason)
            # A new baseline is allowed ONLY during startup; failure never clears.
            self.last.pop(key, None)
            if self.phase != 'WAIT_INPUTS' or not (stamp_ns > 0 and math.isfinite(age) and 0 <= age <= max_age):
                return
        self.last[key] = (now, stamp_ns, age)
        self.counts[key] += 1

    def tick(self, now, prerequisites):
        if self.phase in ('FAILED', 'PASSED'):
            return self.phase
        if self.phase == 'WAIT_INPUTS' and now-self.started >= self.startup_s:
            self.phase = 'FAILED'
            self.fault = 'startup deadline'
            self.finished = now
            return self.phase
        fresh = all(k in self.last and 0 <= now-self.last[k][0] <= v[0]
                    and 0 <= self.last[k][2]+now-self.last[k][0] <= v[1]
                    for k, v in self.limits.items())
        if not prerequisites or not fresh:
            # Record transitions, not one event per spin while waiting for discovery.
            if self.phase == 'MEASURE' or self.ready_since is not None:
                self.reject(now, 'prerequisites lost or input stopped/stale')
            self.ready_since = None
            self.counts = {k: 0 for k in self.limits}
        elif self.phase == 'WAIT_INPUTS':
            if self.ready_since is None:
                self.ready_since = now
                self.counts = {k: 0 for k in self.limits}
            if now-self.ready_since >= 1. and self.counts['vio'] >= 30:
                self.measure_started = now
                self.phase = 'MEASURE'
                self.events.append({'at': now, 'phase': 'MEASURE'})
        if self.phase == 'MEASURE' and now-self.measure_started >= self.duration_s:
            self.phase = 'PASSED'
            self.finished = now
        return self.phase

    def report(self):
        return dict(phase=self.phase, passed=self.phase == 'PASSED', fault=self.fault,
                    startup_limit_s=self.startup_s, required_measurement_s=self.duration_s,
                    measure_started=self.measure_started, finished=self.finished,
                    limits=self.limits, events=self.events, flight_ready=False)
