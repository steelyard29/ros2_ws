"""Independent dispatch contract. No ROS, publishers or flight authority.

This validates a proposed output against an independent snapshot. Passing it
does NOT enable live routes or replace release evidence / operator permission.
"""
import math

from flight_supervisor import FlightSupervisor


class FlightDispatchContract:
    STREAM_STATES = {'STREAM', 'OFFBOARD', 'ARM', 'TAKEOFF', 'HOVER'}
    QUIET_STATES = {'WAIT', 'DONE', 'STOPPED', 'HANDOVER', 'KILLED'}

    def __init__(self):
        self.fault = None
        self.last_now = None
        self.last_stamp = None
        self.last_command_at = {}
        self.terminal = False

    def check(self, output, sample, *, now, stamp, state, origin, height,
              switch_fresh, mode_slot, kill_switch, exclusive):
        """Latch any violation. Quiet output is allowed on stale telemetry.

        LAND intentionally does not require healthy EV/local position: vision
        failure is a reason to land. It requires fresh armed status and no
        observed takeover/kill. RC freshness is required for position streams,
        not a LAND intent while status is still fresh during link degradation.
        No intent here proves command delivery.
        """
        try:
            if self.fault:
                raise ValueError(self.fault)
            if (not math.isfinite(now) or now < 0
                    or (self.last_now is not None and now <= self.last_now)
                    or type(stamp) is not int or stamp <= 0
                    or (self.last_stamp is not None and stamp <= self.last_stamp)):
                raise ValueError('dispatch clocks must advance')
            self.last_now, self.last_stamp = now, stamp
            if output.state != state:
                raise ValueError('output/state mismatch')
            allowed = self.STREAM_STATES | self.QUIET_STATES | {'LAND', 'ABORT'}
            if state not in allowed:
                raise ValueError('unknown state')
            active = bool(output.stream or output.command)
            if self.terminal and active:
                raise ValueError('terminal session cannot resume')
            if state in self.QUIET_STATES:
                if active:
                    raise ValueError('quiet state produced output')
                if state != 'WAIT':
                    self.terminal = True
            if not active:
                return
            fresh = FlightSupervisor.fresh
            # AUX slot 6 can lag the RC switch that already put PX4 in Offboard.
            aux_debounce = (state == 'STREAM' and mode_slot in range(1, 6)
                and sample.nav_state == 14 and not sample.armed and sample.landed
                and kill_switch == 3)
            if (not exclusive or not sample.input_ownership_ok
                    or (mode_slot != 6 and not aux_debounce) or kill_switch != 3
                    or sample.kill_active or sample.manual_takeover
                    or not fresh(sample.status_at, now, 1.5)):
                raise ValueError('unsafe dispatch ownership/status/RC')
            if output.command not in (None, 'OFFBOARD', 'ARM', 'LAND'):
                raise ValueError('command not allowed')
            expected = {'OFFBOARD': 'OFFBOARD', 'ARM': 'ARM'}
            if output.command in expected and state != expected[output.command]:
                raise ValueError('command/state mismatch')
            if output.command == 'LAND':
                if (state not in {'LAND', 'ABORT'} or output.stream or not sample.armed
                        or sample.nav_state not in (14, 18)):
                    raise ValueError('invalid LAND snapshot')
            if output.stream:
                if (state not in self.STREAM_STATES or sample.failsafe
                        or not switch_fresh or not sample.rc_valid
                        or not fresh(sample.rc_at, now, .5)
                        or not fresh(sample.local_at, now, .5)
                        or not fresh(sample.flags_at, now, 2.)
                        or not fresh(sample.land_at, now, 2.)
                        or not FlightSupervisor.position_healthy(sample)):
                    raise ValueError('unsafe position stream')
                if (origin is None or len(origin) != 4
                        or not all(math.isfinite(v) for v in origin)
                        or not math.isfinite(height) or not .2 <= height <= 1.3
                        or output.position is None or len(output.position) != 3
                        or output.yaw is None
                        or not all(math.isfinite(v) for v in (*output.position, output.yaw))):
                    raise ValueError('invalid setpoint/origin')
                x, y, z = output.position
                if (abs(x-origin[0]) > 1e-6 or abs(y-origin[1]) > 1e-6
                        or abs(output.yaw-origin[3]) > 1e-6
                        or not origin[2]-height-1e-6 <= z <= origin[2]+1e-6):
                    raise ValueError('setpoint outside vertical-only envelope')
                if state in {'STREAM', 'OFFBOARD', 'ARM'}:
                    awaiting_arm_ack = state == 'ARM' and sample.armed and output.command is None
                    if (((sample.armed or not sample.landed) and not awaiting_arm_ack) or abs(z-origin[2]) > 1e-6
                            or sample.nav_state not in (2, 14)):
                        raise ValueError('pre-arm stream must hold landed origin')
                elif not sample.armed or sample.nav_state != 14:
                    raise ValueError('airborne stream requires armed Offboard')
            if output.command in {'OFFBOARD', 'ARM'}:
                if not output.stream or not sample.preflight_ok:
                    raise ValueError('arming/mode requires healthy prestream')
                if output.command == 'ARM' and sample.nav_state != 14:
                    raise ValueError('ARM requires observed Offboard')
            if output.command:
                previous = self.last_command_at.get(output.command)
                if previous is not None and now-previous < 1.0:
                    raise ValueError('command rate exceeds 1 Hz')
                self.last_command_at[output.command] = now
        except (TypeError, ValueError) as exc:
            self.fault = self.fault or str(exc)
            raise ValueError(self.fault) from exc
