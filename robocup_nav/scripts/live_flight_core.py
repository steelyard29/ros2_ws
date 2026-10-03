"""Single-sortie guarded core; ROS-free and usable in isolated tests.

No authority is conferred by constructing this class. Real routes additionally
require a validated, consumed release permit in the separate CLI.
"""
from dataclasses import replace
import math

from flight_runtime_core import FlightRuntimeCore
from flight_supervisor import FlightOutput, FlightSupervisor


class LiveFlightCore(FlightRuntimeCore):
    live_core = True
    command_source_component = 191

    def __init__(self, started, aux_params, height=.5, hover=3., abort_requested=None):
        if aux_params is None:
            raise ValueError('live flight requires reviewed AUX parameters')
        super().__init__(started, height, hover, aux_params)
        self.gcs_connected = False
        self.gcs_at = -1.
        self.baseline_since = None
        self.baseline_key = None
        self.ready_at = None
        self.edge_at = None
        self.accepted_commands = set()
        self.ev_stopped = False
        self.closed = False
        self.stop_reason = None
        self.arm_requested = False
        self.landing_seen = False
        self.abort_requested = abort_requested or (lambda: False)

    def gcs_status(self, connected, now):
        self.gcs_connected, self.gcs_at = bool(connected), now

    def switches(self, now, mode_slot, kill_switch):
        previous = self.mode_slot
        super().switches(now, mode_slot, kill_switch)
        if self.controller.state == 'WAIT' and mode_slot == 6:
            # RC callbacks can arrive twice before the executive consumes the
            # edge. A repeated stable slot is not a second operator action.
            if self.edge_at is not None:
                return
            if self.ready_at is None or previous not in range(1, 6):
                self.request_abort(now, 'Offboard selected before READY; new session required')
            else:
                self.edge_at = now

    def status(self, arming_state, now, **fields):
        super().status(arming_state, now, **fields)
        if arming_state not in (1, 2):
            self.request_abort(now, 'unknown arming state')
        elif arming_state == 2 and not self.arm_requested:
            self.request_abort(now, 'unexpected arming; no autonomous recovery')
            self.manual_latched = True
        if arming_state == 2 and fields.get('nav_state') == 18:
            self.landing_seen = True

    def sent(self, command, stamp_us):
        super().sent(command, stamp_us)
        self.arm_requested |= command == 'ARM'

    def ack(self, command, result, stamp_us, now):
        accepted = super().ack(command, result, stamp_us, now)
        if accepted and result == 0:
            self.accepted_commands.add(command)
        if accepted and result not in (0, 1, 5):
            self.command_fault = f'{command} rejected/unknown ACK result {result}'
        return accepted

    def request_abort(self, now, reason):
        self.stop_reason = self.stop_reason or reason
        self.trip(reason)
        self.ev_stopped = True
        self.controller.abort(now, reason)

    def close(self, reason):
        self.closed = True
        self.ev_stopped = True
        self.stop_reason = self.stop_reason or reason

    def ev_allowed(self, now):
        if self.closed or self.ev_stopped:
            return False
        if self.abort_requested():
            self.request_abort(now, 'operator interrupt')
            return False
        if self.last_tick is not None and not 0 <= now-self.last_tick <= .3:
            self.request_abort(now, 'EV callback observed executive deadline fault')
            return False
        if (self.closed or self.ev_stopped or self.controller.state in
                (self.controller.TERMINAL | {'ABORT', 'LAND'}) or self.fault
                or self.command_fault or self.kill_latched or self.manual_latched):
            return False
        s, fresh = self.sample, FlightSupervisor.fresh
        healthy = (self._owns(now) and fresh(s.status_at, now, 1.5)
            and fresh(s.land_at, now, 2.) and fresh(s.rc_at, now, .5) and s.rc_valid
            and fresh(s.flags_at, now, 2.) and s.baro_height
            and not s.range_height and not s.ev_velocity and not s.failsafe
            and self.switch_fresh(now) and self.kill_switch == 3
            and not s.manual_takeover)
        if self.controller.state == 'WAIT':
            # The pilot switch itself moves PX4 to Offboard before debounced AUX
            # reports slot 6. After READY that is the edge, not a health loss.
            if (self.ready_at is not None and not s.armed and self.kill_switch == 3
                    and s.nav_state == 14 and self.gcs_connected and not self.kill_latched):
                if self.edge_at is None:
                    self.edge_at = now
                return not self.ev.inhibit and self._owns(now)
            healthy &= (not s.armed and s.landed and s.nav_state == 2
                and self.mode_slot in range(1, 6) and self.gcs_connected
                and fresh(self.gcs_at, now, 1.5))
        elif self.controller.state in {'STREAM', 'OFFBOARD', 'ARM'}:
            # PX4 can already be Offboard while AUX debounce still reports Position.
            # That window must keep EV; stopping here cannot be undone when slot 6 arrives.
            slot_pending = (not s.armed and s.nav_state == 14 and self.mode_slot in range(1, 6))
            healthy &= (self.mode_slot == 6 or slot_pending) and s.nav_state in (2, 14)
        else:
            healthy &= self.mode_slot == 6 and s.armed and s.nav_state == 14
        if not healthy and self.ev.publish_count:
            # A fresh READY edge may arrive before the next executive tick.
            # Stop EV briefly, but let that tick consume the edge first.
            if self.controller.state == 'WAIT' and self.edge_at is not None:
                return False
            self.ev_stopped = True
        return bool(healthy) and not self.ev.inhibit

    def pose(self, *args, **kwargs):
        now = args[2] if len(args) > 2 else kwargs['now']
        if not self.ev_allowed(now):
            return None
        return super().pose(*args, **kwargs)

    def tracking(self, state, stamp_ns, now):
        if self.closed or self.ev_stopped:
            return False
        return super().tracking(state, stamp_ns, now)

    def tick(self, now, start=False):
        if self.closed:
            return FlightOutput('STOPPED', reason=self.stop_reason or 'closed')
        if self.abort_requested():
            self.request_abort(now, 'operator interrupt')
        if (not math.isfinite(now) or (self.last_tick is not None
                and not 0 < now-self.last_tick <= .3)):
            self.request_abort(now, 'executive time/deadline fault')
        self.last_tick = now
        if not self.ev_stopped:
            self.ev.watchdog(now)
        s = replace(self.sample, ev_ok=self.ev_receipt_recent(now)
                    and not self.ev.inhibit and not self.ev_stopped,
                    input_ownership_ok=self._owns(now),
                    manual_takeover=self.sample.manual_takeover or self.manual_latched,
                    kill_active=self.kill_latched)
        self.sample = s
        phase = self.controller.state
        if phase == 'WAIT':
            if self.fault or self.command_fault or self.ev.inhibit or self.ev_stopped:
                self.request_abort(now, self.fault or self.command_fault or self.ev.fault or 'EV gate stopped')
            else:
                if (self.edge_at is None and self.ready_at is not None and not s.armed
                        and self.kill_switch == 3 and s.nav_state == self.controller.OFFBOARD
                        and self.gcs_connected and self.controller.fresh(self.gcs_at, now, 1.5)):
                    self.edge_at = now
                if self.edge_at is None:
                    good = (start and self.controller.ready(s, now) and s.nav_state == 2
                        and self.mode_slot in range(1, 6) and self.switch_fresh(now)
                        and self.gcs_connected and self.controller.fresh(self.gcs_at, now, 1.5))
                    key = (self.mode_slot, s.reset_counters)
                    if not good:
                        if self.ready_at is not None:
                            self.request_abort(now, 'READY conditions lost; new session required')
                        self.baseline_since = self.baseline_key = self.ready_at = None
                    elif self.baseline_since is None or key != self.baseline_key:
                        self.baseline_since, self.baseline_key = now, key
                        self.ready_at = None
                    elif now-self.baseline_since >= 5 and self.ready_at is None:
                        self.ready_at = now
                    if self.ready_at is not None and now-self.ready_at > 60:
                        self.request_abort(now, 'READY switch window expired')
                else:
                    # PX4 clears preflight when the switch enters Offboard before setpoints.
                    start_sample = s
                    if (s.nav_state == self.controller.OFFBOARD and not s.preflight_ok
                            and not s.armed and s.landed and self.controller.position_healthy(s)):
                        start_sample = replace(s, preflight_ok=True)
                    if (self.ready_at is None or now-self.ready_at > 60 or now-self.edge_at > .5
                            or not self.controller.ready(start_sample, now) or not self.gcs_connected
                            or not self.controller.fresh(self.gcs_at, now, 1.5)):
                        self.request_abort(now, 'unsafe/expired READY edge')
                    else:
                        self.sample = start_sample
        elif phase not in self.controller.TERMINAL and phase not in {'LAND', 'ABORT'}:
            if (self.fault or self.command_fault or self.ev.inhibit or self.ev_stopped
                    or not self.switch_fresh(now)):
                self.controller.abort(now, self.fault or self.command_fault or self.ev.fault
                                      or 'EV/switch telemetry unavailable')
        begin = (start and self.controller.state == 'WAIT' and self.edge_at is not None
                 and not self.start_consumed and not self.fault)
        if begin:
            self.start_consumed = True
        output = self.controller.step(now, self.sample, start=begin,
                                      accepted_commands=self.accepted_commands)
        if output.state in self.controller.TERMINAL or output.state in {'ABORT', 'LAND'}:
            self.ev_stopped = True
        return output

    def report(self):
        return {**super().report(), 'live_core': True, 'ready_at': self.ready_at,
            'operator_edge_at': self.edge_at, 'accepted_commands': sorted(self.accepted_commands),
            'arm_requested': self.arm_requested, 'landing_seen': self.landing_seen,
            'gcs_connected': self.gcs_connected, 'ev_stopped': self.ev_stopped,
            'closed': self.closed, 'stop_reason': self.stop_reason}
