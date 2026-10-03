"""Pure, bounded mode-handshake candidate. No ROS, GPIO, devices or arming.

This is NOT a live entrypoint. Caller must validate source timestamps, an
explicit operator request, physical setup and exclusive output ownership.
Success stops streaming; operator must restore Position before a new session.
"""
import math

from flight_supervisor import FlightOutput, FlightSupervisor


class DisarmedHandshake:
    TERMINAL = {'DONE', 'ABORT', 'HANDOVER', 'KILLED'}

    @staticmethod
    def operator_action(state):
        """Bench display only; never changes control state or sends commands."""
        if state == 'AWAIT_SWITCH':
            return 'switch to Offboard now; keep disarmed, kill off, props removed'
        if state in ('WAIT_MODE', 'VERIFY'):
            return 'keep Offboard, disarmed, kill off, props removed; await result'
        if state == 'KILLED':
            return 'outputs stopped; keep disarmed, props removed; restore Position; do not clear kill yet'
        if state in DisarmedHandshake.TERMINAL:
            return 'outputs stopped; restore Position; keep disarmed, props removed'
        if state in ('WAIT', 'PRESTREAM'):
            return 'keep Position, disarmed, kill off, props removed'
        return 'unknown state; do not switch or arm; await operator review'

    def __init__(self, staged=False, switch_wait_s=20):
        if type(switch_wait_s) is not int or switch_wait_s not in (20, 60) or (not staged and switch_wait_s != 20):
            raise ValueError('staged switch wait must be explicitly bounded to 20 or 60 seconds')
        self.switch_wait_s = switch_wait_s
        self.staged = staged
        self.baseline_slot = None
        self.switch_wait = self.nav_ahead_since = None
        self.state = 'WAIT'
        self.reason = ''
        self.last_now = None
        self.started = self.requested = self.confirmed = None
        self.origin = self.resets = None

    def slot_allowed(self, slot):
        if not self.staged:
            return slot in range(1, 7) if self.state == 'WAIT' else slot == 6
        if self.state == 'WAIT':
            return slot in range(1, 6)
        if self.state == 'PRESTREAM':
            return slot == self.baseline_slot
        if self.state == 'AWAIT_SWITCH':
            return slot in (self.baseline_slot, 6)
        return slot == 6

    def nav_allowed(self, nav):
        if self.state == 'WAIT' or (self.staged and self.state == 'PRESTREAM'):
            return nav == 2
        return nav in (2, 14)

    def stop(self, state, reason):
        self.state, self.reason = state, reason
        return FlightOutput(state, reason=reason)

    def step(self, now, sample, *, start=False, mode_slot=0,
             switch_at=-1., kill_switch=0, ack_accepted=False):
        if not math.isfinite(now) or (self.last_now is not None and now <= self.last_now):
            return self.stop('ABORT', 'non-increasing monotonic time')
        dt = 0 if self.last_now is None else now-self.last_now
        self.last_now = now
        if self.state in self.TERMINAL:
            return FlightOutput(self.state, reason=self.reason)
        s = sample
        if kill_switch == 1 or s.kill_active:
            return self.stop('KILLED', 'kill: no command or stream')
        if s.armed:
            return self.stop('ABORT', 'unexpected arm: stop; never issue disarm or LAND')
        if not FlightSupervisor.fresh(switch_at, now, .5) or kill_switch != 3:
            if self.state != 'WAIT' or start:
                return self.stop('ABORT', 'missing fresh clear switch state')
            return FlightOutput('WAIT')
        if self.state != 'WAIT' and (not self.slot_allowed(mode_slot) or s.manual_takeover):
            return self.stop('HANDOVER', 'operator takeover: no re-entry')
        if self.state == 'WAIT':
            if not start:
                return FlightOutput('WAIT')
            slot_ok = mode_slot in range(1, 6) if self.staged else mode_slot == 6
            if not slot_ok or s.nav_state != 2 or not FlightSupervisor().ready(s, now):
                return self.stop('ABORT', 'start requires healthy disarmed Position and explicit Offboard slot')
            self.baseline_slot = mode_slot
            self.origin = (s.x, s.y, s.z, s.heading)
            self.resets = s.reset_counters
            self.started = now
            self.state = 'PRESTREAM'
        else:
            # ready() deliberately includes disarmed and landed, unlike flight.
            if dt > .3 or not FlightSupervisor().ready(s, now):
                return self.stop('ABORT', 'stale/unhealthy input or loop gap')
            if s.reset_counters != self.resets:
                return self.stop('ABORT', 'estimator reset')
            if (math.dist((s.x, s.y, s.z), self.origin[:3]) > .15
                    or abs(math.atan2(math.sin(s.heading-self.origin[3]),
                                      math.cos(s.heading-self.origin[3]))) > .15):
                return self.stop('ABORT', 'bench moved from captured pose')
            if not self.nav_allowed(s.nav_state):
                return self.stop('HANDOVER', 'unexpected PX4 mode')
        if now-self.started >= (self.switch_wait_s+10 if self.staged else 10):
            return self.stop('ABORT', 'bounded session expired')
        command = None
        if self.state == 'PRESTREAM' and now-self.started >= 2:
            if self.staged:
                self.state, self.switch_wait = 'AWAIT_SWITCH', now
            else:
                self.state, self.requested = 'WAIT_MODE', now
                command = 'OFFBOARD'  # One request only. Never ARM or LAND.
        elif self.state == 'AWAIT_SWITCH':
            if now-self.switch_wait >= self.switch_wait_s:
                return self.stop('ABORT', 'operator switch window expired')
            if mode_slot == 6:
                self.state, self.requested = 'WAIT_MODE', now
                command = 'OFFBOARD'
            elif s.nav_state == 14:
                # VehicleStatus and debounced AUX can arrive in either order.
                self.nav_ahead_since = now if self.nav_ahead_since is None else self.nav_ahead_since
                if now-self.nav_ahead_since > .5:
                    return self.stop('ABORT', 'Offboard state without corresponding fresh switch edge')
        elif self.state == 'WAIT_MODE':
            if now-self.requested >= 3:
                return self.stop('ABORT', 'mode state confirmation timeout')
            if ack_accepted and s.nav_state == 14 and s.status_at > self.requested:
                self.state, self.confirmed = 'VERIFY', now
        elif self.state == 'VERIFY':
            if s.nav_state != 14:
                return self.stop('HANDOVER', 'PX4 left Offboard')
            if now-self.confirmed >= 1:
                return self.stop('DONE', 'fresh disarmed Offboard observed; restore Position manually')
        return FlightOutput(self.state, True, self.origin[:3], self.origin[3], command)


def validate_handshake_output(output):
    """Second, independent allowlist before any future serializer/publisher."""
    if output.command not in (None, 'OFFBOARD'):
        raise ValueError('disarmed handshake forbids arm, land and actuator commands')
    if output.command and (output.state != 'WAIT_MODE' or not output.stream):
        raise ValueError('mode request outside streaming handshake')
    if output.state in DisarmedHandshake.TERMINAL and (output.stream or output.command):
        raise ValueError('terminal handshake must be silent')
    if output.stream and (output.state not in {'PRESTREAM', 'AWAIT_SWITCH', 'WAIT_MODE', 'VERIFY'}
                          or output.position is None or len(output.position) != 3
                          or output.yaw is None
                          or not all(math.isfinite(v) for v in (*output.position, output.yaw))):
        raise ValueError('invalid handshake hold setpoint')
    return output
