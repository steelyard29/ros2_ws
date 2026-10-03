"""Pure PX4-local takeoff/hold/land controller, not a live-flight entrypoint.

All timestamps are caller monotonic seconds. PX4 local x/y/z and heading are
captured while disarmed: hold that x/y/heading, ramp z upward (negative down).
No ENU/north assumption and no dist_bottom substitution. Navigation-frame
alignment for later horizontal missions remains a separate prerequisite.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FlightSample:
    status_at: float = -1.0
    local_at: float = -1.0
    flags_at: float = -1.0
    land_at: float = -1.0
    rc_at: float = -1.0
    armed: bool = False
    landed: bool = False
    nav_state: int = -1
    preflight_ok: bool = False
    rc_valid: bool = False
    manual_takeover: bool = False
    kill_active: bool = False
    input_ownership_ok: bool = False
    failsafe: bool = False
    ev_ok: bool = False
    ev_position: bool = False
    ev_height: bool = False
    ev_yaw: bool = False
    ev_velocity: bool = False
    baro_height: bool = False
    range_height: bool = False
    xy_valid: bool = False
    z_valid: bool = False
    v_xy_valid: bool = False
    v_z_valid: bool = False
    x: float = math.nan
    y: float = math.nan
    z: float = math.nan
    heading: float = math.nan
    vx: float = math.nan
    vy: float = math.nan
    vz: float = math.nan
    reset_counters: tuple = ()


@dataclass(frozen=True)
class FlightOutput:
    state: str
    stream: bool = False
    position: tuple | None = None
    yaw: float | None = None
    command: str | None = None
    reason: str = ''


class FlightSupervisor:
    OFFBOARD = 14
    AUTO_LAND = 18
    TERMINAL = {'DONE', 'STOPPED', 'HANDOVER', 'KILLED'}

    def __init__(self, height=0.5, hover=3.0):
        if not math.isfinite(height) or not 0.2 <= height <= 1.3:
            raise ValueError('height must be 0.2..1.3 m')
        if not math.isfinite(hover) or not 1 <= hover <= 15:
            raise ValueError('hover must be 1..15 s')
        self.height, self.hover = height, hover
        self.state = 'WAIT'
        self.started = self.entered = self.last_now = None
        self.origin = None
        self.resets = None
        self.z_sp = None
        self.hover_since = None
        self.offboard_seen = False
        self.last_command = -math.inf
        self.reason = ''
        self.history = []

    def _enter(self, state, now, reason=''):
        self.state, self.entered, self.reason = state, now, reason
        self.last_command = -math.inf
        self.history.append({'at': now, 'state': state, 'reason': reason})

    @staticmethod
    def fresh(at, now, limit=.5):
        return math.isfinite(at) and at >= 0 and 0 <= now - at <= limit

    def ready(self, s, now):
        return (self.fresh(s.status_at, now, 1.5) and self.fresh(s.land_at, now, 2.0)
                and all(self.fresh(at, now) for at in (s.local_at, s.rc_at))
                and self.fresh(s.flags_at, now, 2.0) and s.preflight_ok
                and not s.armed and s.landed and s.rc_valid
                and not s.failsafe and not s.manual_takeover and not s.kill_active
                and s.input_ownership_ok
                and self.position_healthy(s) and len(s.reset_counters) == 5)

    @staticmethod
    def position_healthy(s):
        return (s.ev_ok and s.ev_position and s.ev_height and s.ev_yaw
                and not s.ev_velocity and s.baro_height and not s.range_height
                and s.xy_valid and s.z_valid and s.v_xy_valid and s.v_z_valid
                and all(math.isfinite(v) for v in
                        (s.x, s.y, s.z, s.heading, s.vx, s.vy, s.vz)))

    def _output(self, now, command=None, stream=False):
        if command and now - self.last_command < 1.0:
            command = None
        if command:
            self.last_command = now
        return FlightOutput(self.state, stream,
                            (self.origin[0], self.origin[1], self.z_sp) if stream else None,
                            self.origin[3] if stream else None, command, self.reason)

    def step(self, now, s, *, start=False, accepted_commands=None):
        if not math.isfinite(now) or (self.last_now is not None and now < self.last_now):
            raise ValueError('invalid monotonic time')
        dt = 0 if self.last_now is None else now - self.last_now
        self.last_now = now
        if self.state in self.TERMINAL:
            return self._output(now)
        if s.kill_active:
            self._enter('KILLED', now, 'physical kill input; never restart motors')
            return self._output(now)
        if self.state == 'WAIT':
            if start and self.ready(s, now):
                self.origin = (s.x, s.y, s.z, s.heading)
                self.resets = s.reset_counters
                self.z_sp = s.z
                self.started = now
                self._enter('STREAM', now)
            return self._output(now, stream=self.state == 'STREAM')

        status_fresh = self.fresh(s.status_at, now, 1.5)
        land_fresh = self.fresh(s.land_at, now, 2.0)
        if s.manual_takeover:
            self._enter('HANDOVER', now, 'manual takeover; no re-entry')
            return self._output(now)
        if not s.input_ownership_ok:
            self._enter('HANDOVER', now, 'input ownership lost; no competing commands')
            return self._output(now)
        # A mode change away from Offboard is authoritative. Never command a
        # pilot back into Offboard or override PX4's own failsafe mode.
        if status_fresh and self.offboard_seen and s.nav_state not in (self.OFFBOARD, self.AUTO_LAND):
            self._enter('HANDOVER', now, 'PX4 left Offboard; relinquish control')
            return self._output(now)
        if self.state in {'LAND', 'ABORT'}:
            if status_fresh and land_fresh and s.landed and not s.armed:
                self._enter('DONE' if self.state == 'LAND' else 'STOPPED', now, self.reason)
                return self._output(now)
            # No stale LINK assumptions: stop heartbeats, rely on PX4 when
            # status is absent. Never force-disarm from stale/low height data.
            return self._output(now, command='LAND' if status_fresh and s.armed else None)

        fault = ''
        if not status_fresh or dt > .3:
            fault = 'link/status or executive watchdog lost; onboard failsafe required'
        elif not self.fresh(s.local_at, now) or not self.fresh(s.flags_at, now, 2):
            fault = 'estimator telemetry stale'
        elif not land_fresh or not self.fresh(s.rc_at, now) or not s.rc_valid:
            fault = 'land detector or RC unavailable'
        elif not self.position_healthy(s):
            fault = 'EV/estimator unhealthy'
        elif s.reset_counters != self.resets:
            fault = 'PX4 local frame reset'
        elif s.failsafe:
            fault = 'PX4 failsafe'
        elif s.nav_state == self.AUTO_LAND:
            fault = 'PX4 entered Auto Land'
        elif now - self.started >= 60:
            fault = 'bounded takeoff test timeout'
        elif math.hypot(s.x-self.origin[0], s.y-self.origin[1]) > .3:
            fault = 'horizontal departure >0.3 m'
        elif self.origin[2] - s.z > self.height + .2:
            fault = 'height overshoot >0.2 m'
        if fault:
            self._enter('ABORT', now, fault)
            return self._output(now, command='LAND' if status_fresh and s.armed else None)

        age = now - self.entered
        if self.state == 'STREAM':
            if s.armed:
                self._enter('ABORT', now, 'unexpected arming before mode handshake')
                return self._output(now, command='LAND')
            if age >= 2:
                self._enter('OFFBOARD', now)
        elif self.state == 'OFFBOARD':
            if s.nav_state == self.OFFBOARD and (accepted_commands is None or 'OFFBOARD' in accepted_commands):
                self.offboard_seen = True
                self._enter('ARM', now)
            elif age >= 5:
                self._enter('ABORT', now, 'Offboard not acknowledged by status')
        elif self.state == 'ARM':
            if s.armed and (accepted_commands is None or 'ARM' in accepted_commands):
                self._enter('TAKEOFF', now)
            elif age >= 5 or not s.preflight_ok:
                self._enter('ABORT', now, 'arming timeout or preflight failed')
        elif self.state in {'TAKEOFF', 'HOVER'}:
            if not s.armed:
                self._enter('ABORT', now, 'unexpected disarm')
            else:
                target_z = self.origin[2] - self.height
                self.z_sp = max(target_z, self.z_sp - .2 * dt)
                stable = (not s.landed and abs(s.z-target_z) <= .08
                          and math.hypot(s.vx, s.vy) <= .15 and abs(s.vz) <= .1)
                if stable:
                    if self.hover_since is None:
                        self.hover_since = now
                        self._enter('HOVER', now)
                    if now - self.hover_since >= self.hover:
                        self._enter('LAND', now)
                else:
                    self.hover_since = None
                if self.state == 'TAKEOFF' and age >= 15:
                    self._enter('ABORT', now, 'takeoff timeout')

        command = {'OFFBOARD': 'OFFBOARD', 'ARM': 'ARM', 'LAND': 'LAND', 'ABORT': 'LAND'}.get(self.state)
        if command == 'ARM' and s.armed:
            command = None  # Actual arming may precede ACK; never resend ARM while armed.
        if command == 'LAND' and not s.armed:
            command = None
        stream = self.state in {'STREAM', 'OFFBOARD', 'ARM', 'TAKEOFF', 'HOVER'}
        return self._output(now, command=command, stream=stream)

    def abort(self, now, reason):
        """External adapter failure; recovery never resets this session."""
        if self.state not in self.TERMINAL and self.state != 'ABORT':
            self._enter('ABORT', now, reason)
