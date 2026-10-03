"""Disarmed handshake + EV coordination, no I/O.

Reuse source-time, AUX and ACK correlation contracts, never call the flight
supervisor's step(). EV bootstraps before estimator readiness; mode transition
is permitted only in an active reviewed handshake. Terminal sessions stop EV.
The staged variant also backs the separately authorized prop-off bench route.
"""
from dataclasses import replace
import math

from disarmed_handshake import DisarmedHandshake, validate_handshake_output
from flight_runtime_core import FlightRuntimeCore
from flight_supervisor import FlightSupervisor


class HandshakeRuntimeCore(FlightRuntimeCore):
    handshake_only = True

    def __init__(self, started, aux_params=None, staged=False, require_gcs=False, switch_wait_s=20):
        super().__init__(started, aux_params=aux_params)
        self.handshake = DisarmedHandshake(staged=staged, switch_wait_s=switch_wait_s)
        self.staged = staged
        self.baseline_since = self.baseline_key = None
        self.command_source_component = 191 if staged else 1
        self.require_gcs = require_gcs
        self.gcs_connected = False
        self.gcs_at = -1.
        self.handshake_history = []
        self.ack_accepted = False
        self.ev_stopped = False
        self.ev_blockers = []

    def gcs_status(self, connected, now):
        # Called only after the runtime's source timestamp validation.
        self.gcs_connected, self.gcs_at = bool(connected), now

    def switches(self, now, mode_slot, kill_switch):
        if self.handshake.state not in ('WAIT', *self.handshake.TERMINAL) and not self.handshake.slot_allowed(mode_slot):
            self.manual_latched = True
        super().switches(now, mode_slot, kill_switch)

    def sent(self, command, stamp_us):
        if command != 'OFFBOARD' or self.handshake.state != 'WAIT_MODE':
            self.trip('invalid handshake command dispatch')
            raise ValueError('handshake can dispatch only OFFBOARD while waiting for mode')
        super().sent(command, stamp_us)

    def ack(self, command, result, stamp_us, now):
        if self.handshake.state in self.handshake.TERMINAL or command != 'OFFBOARD':
            return False
        if not super().ack(command, result, stamp_us, now):
            return False
        if result == 0:
            self.ack_accepted = True
        elif result not in (1, 2, 3, 4, 5, 6):
            self.command_fault = 'unknown mode ACK result'
        return True

    def ev_allowed(self, now):
        if self.ev_stopped or self.handshake.state in self.handshake.TERMINAL:
            return False
        s = self.sample
        fresh = FlightSupervisor.fresh
        checks = {
            'disarmed': fresh(s.status_at, now, 1.5) and not s.armed,
            'landed': fresh(s.land_at, now, 2) and s.landed,
            'rc': fresh(s.rc_at, now, .5) and s.rc_valid,
            'switches': self.switch_fresh(now) and self.kill_switch == 3 and not self.kill_latched,
            'slot': self.handshake.slot_allowed(self.mode_slot),
            'nav': self.handshake.nav_allowed(s.nav_state),
            'flags': fresh(s.flags_at, now, 2) and s.baro_height and not s.range_height and not s.ev_velocity,
            'ownership': self._owns(now),
            'session': not self.fault and not self.ev.inhibit and not self.command_fault,
            'failsafe': not s.failsafe,
            'gcs_monitor': not self.require_gcs or (self.gcs_connected and fresh(self.gcs_at, now, 1.5)),
            'takeover': not (s.manual_takeover or self.manual_latched),
        }
        self.ev_blockers = [k for k, v in checks.items() if not v]
        if self.ev_blockers and (self.ev.publish_count or s.armed or self.kill_latched
                                 or self.ownership_fault or self.fault or self.command_fault):
            self.trip('handshake EV gate: '+', '.join(self.ev_blockers))
        return not self.ev_blockers and not self.ev.inhibit

    def pose(self, stamp_ns, age_ms, now, position, quaternion, frame_ok, now_ns):
        if not self.ev_allowed(now):
            return None
        return super().pose(stamp_ns, age_ms, now, position, quaternion, frame_ok, now_ns)

    def tick(self, now, start=False):
        previous = self.handshake.state
        if previous in self.handshake.TERMINAL:
            self.last_tick = now
            return validate_handshake_output(self.handshake.step(now, self.sample))
        if (not math.isfinite(now) or (self.last_tick is not None
                and not 0 < now-self.last_tick <= .3)):
            self.trip('invalid handshake executive time/deadline')
        self.last_tick = now
        self.ev.watchdog(now)
        self.ev_allowed(now)
        if self.ev.inhibit and self.ev_fault_at is None:
            self.ev_fault_at = now
        s = replace(self.sample,
                    ev_ok=not self.ev.inhibit and 0 <= now-self.last_ev_at <= .3,
                    input_ownership_ok=self._owns(now),
                    manual_takeover=self.sample.manual_takeover or self.manual_latched,
                    kill_active=self.kill_latched)
        self.sample = s
        # Preserve kill and takeover identity even when their EV gate also trips.
        if self.kill_latched:
            output = self.handshake.stop('KILLED', 'physical kill; all shadow output stopped')
        elif previous != 'WAIT' and (s.manual_takeover or
                (self.switch_fresh(now) and not self.handshake.slot_allowed(self.mode_slot))):
            output = self.handshake.stop('HANDOVER', 'manual takeover; no re-entry')
        elif self.fault or self.command_fault or self.ev.inhibit:
            output = self.handshake.stop('ABORT', self.command_fault or self.fault or self.ev.fault)
        else:
            # Missing startup telemetry waits; it never creates a healthy snapshot.
            slot_ok = self.mode_slot in range(1, 6) if self.staged else self.mode_slot == 6
            begin = (start and not self.start_consumed and self.switch_fresh(now)
                     and self.kill_switch == 3 and slot_ok
                     and s.nav_state == 2 and self.controller.ready(s, now))
            if self.staged and previous == 'WAIT':
                key = (self.mode_slot, s.reset_counters)
                if not begin:
                    self.baseline_since = self.baseline_key = None
                elif self.baseline_since is None or key != self.baseline_key:
                    self.baseline_since, self.baseline_key = now, key
                begin = begin and self.baseline_since is not None and now-self.baseline_since >= 1.
            if begin:
                self.start_consumed = True
            output = self.handshake.step(now, s, start=begin, mode_slot=self.mode_slot,
                switch_at=self.switch_at, kill_switch=self.kill_switch,
                ack_accepted=self.ack_accepted)
        if output.state in self.handshake.TERMINAL:
            self.ev_stopped = True
        if previous != output.state:
            self.handshake_history.append({'at': now, 'state': output.state, 'reason': output.reason})
        return validate_handshake_output(output)

    def report(self):
        return {**super().report(), 'state': self.handshake.state,
                'reason': self.handshake.reason, 'states': self.handshake_history,
                'handshake_only': True, 'mode_ack_accepted': self.ack_accepted,
                'staged_handshake': self.staged, 'command_source_component': self.command_source_component,
                'gcs_monitor_required': self.require_gcs, 'gcs_connected': self.gcs_connected,
                'switch_wait_seconds': self.handshake.switch_wait_s,
                'ev_stopped': self.ev_stopped, 'handshake_ev_blockers': self.ev_blockers,
                'mode_handshake_validation': False}
