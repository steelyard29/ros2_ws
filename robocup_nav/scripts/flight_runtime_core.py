"""Integrated, ROS-free EV + flight supervisor session.

The runtime is shadow-only. Missing switch telemetry is never synthesized
from the operator's historical QGC checks. All faults latch for the session.
Battery warnings intentionally have no autonomous action (operator policy).
"""
from dataclasses import replace
import math

from flight_ev_adapter import FlightEvAdapter
from flight_supervisor import FlightSample, FlightSupervisor
from aux_switch_decoder import AuxSwitchDecoder


class FlightRuntimeCore:
    def __init__(self, started, height=.5, hover=3, aux_params=None):
        self.started = started
        self.ev = FlightEvAdapter(started)
        self.controller = FlightSupervisor(height, hover)
        self.sample = FlightSample()
        self.source_stamps = {}
        self.source_rejection = None
        self.fault = None
        self.last_ev_at = -1.0
        self.external_ev = None
        self.ev_fault_at = None
        self.last_tick = None
        self.switch_at = -1.0
        self.mode_slot = None
        self.kill_switch = None
        self.manual_latched = False
        self.kill_latched = False
        self.ownership_at = -1.0
        self.ownership_ok = False
        self.ownership_fault = False
        self.pending = None
        self.acks = []
        self.command_fault = None
        self.battery_warning = None
        self.start_consumed = False
        self.aux = AuxSwitchDecoder(aux_params) if aux_params is not None else None
        self.switch_source = 'derived_aux_mirror' if self.aux else 'native_dds'

    def aux_message(self, stamp, sample_stamp, now, age_s, valid, source, aux1, aux2):
        if self.aux is None:
            self.trip('unexpected AUX input in native switch session')
            return False
        decoded = self.aux.receive(stamp=stamp, sample=sample_stamp, now=now, age=age_s,
                                   valid=valid, source=source, aux1=aux1, aux2=aux2)
        if self.aux.fault:
            self.trip(self.aux.fault)
            return False
        # Only new raw RC samples renew RC validity, including debounce samples.
        if self.aux.rejection != 'duplicate sample; freshness not renewed':
            self.rc(now, valid and source == 1)
        if decoded:
            self.switches(now, decoded.mode_slot, decoded.kill_switch)
        return decoded is not None

    def switch_fresh(self, now):
        return 0 <= now-self.switch_at <= (.5 if self.aux else 2.)

    def receive(self, source, stamp, now, age_s):
        """Fresh receipt alone is insufficient: reject replay/reset source time."""
        if (type(stamp) is not int or stamp <= 0 or not math.isfinite(now)
                or not math.isfinite(age_s) or not -.05 <= age_s <= .5):
            if self.source_rejection is None:
                self.source_rejection=dict(source=source,stamp_us=stamp,
                    receipt_monotonic_s=now,source_age_s=age_s,
                    previous_accepted_stamp_us=self.source_stamps.get(source),
                    allowed_age_s=[-.05,.5])
            self.trip('invalid/stale source: '+source)
            return False
        previous = self.source_stamps.get(source)
        if previous is not None and stamp < previous:
            self.trip('source clock reset: '+source)
            return False
        if previous == stamp:
            return False  # Never renew freshness with duplicated packets.
        self.source_stamps[source] = stamp
        return True

    def trip(self, reason):
        self.fault = self.fault or reason
        self.ev.trip(self.fault)

    def status(self, arming_state, now, **fields):
        self.ev.on_vehicle(arming_state, now)
        self.sample = replace(self.sample, status_at=now, armed=arming_state == 2, **fields)

    def local(self, now, **fields):
        self.sample = replace(self.sample, local_at=now, **fields)

    def flags(self, now, **fields):
        self.ev.on_flags(fields['ev_height'], fields['ev_velocity'], now)
        self.sample = replace(self.sample, flags_at=now, **fields)

    def land(self, now, landed):
        self.sample = replace(self.sample, land_at=now, landed=landed)

    def rc(self, now, valid):
        self.sample = replace(self.sample, rc_at=now, rc_valid=valid)

    def switches(self, now, mode_slot, kill_switch):
        self.switch_at = now
        if kill_switch not in (1, 3) or mode_slot not in range(1, 7):
            self.trip('invalid/unassigned mode or kill input')
            return
        if (self.controller.state != 'WAIT' and self.mode_slot is not None
                and mode_slot != self.mode_slot and mode_slot != 6):
            self.manual_latched = True
        self.mode_slot, self.kill_switch = mode_slot, kill_switch
        self.kill_latched |= kill_switch == 1

    def switch_message(self, stamp, sample_stamp, now, age_s, mode, kill, offboard):
        """Validate both publication and actual RC sample; never synthesize inputs.

        PX4 1.16.2 republishes stable switches at ~1 Hz. Its publication time
        alone must not make a frozen raw RC sample look fresh.
        """
        if self.aux is not None:
            self.trip('unexpected native switches in AUX session')
            return False
        if (type(sample_stamp) is not int or sample_stamp <= 0
                or type(stamp) is not int or not 0 <= stamp-sample_stamp <= 500_000
                or offboard != 0):
            self.trip('invalid switch sample or unexpected dedicated Offboard mapping')
            return False
        if not self.receive('manual_control_switches', stamp, now, age_s):
            return False
        if not self.receive('manual_control_switches_sample', sample_stamp, now,
                            age_s+(stamp-sample_stamp)/1e6):
            return False
        self.switches(now, mode, kill)
        return self.fault is None

    def ownership(self, now, ok):
        self.ownership_at = now
        self.ownership_ok = bool(ok)
        if not ok:
            self.ownership_fault = True
            self.ev.trip('publisher ownership conflict')

    def tracking(self, state, stamp_ns, now):
        return self.ev.on_tracking(state, stamp_ns, now)

    def pose(self, stamp_ns, age_ms, now, position, quaternion, frame_ok, now_ns):
        if not self._owns(now):
            return None
        result = self.ev.on_pose(stamp_ns, age_ms, now, position, quaternion,
                                 frame_ok=frame_ok, now_ns=now_ns)
        if self.external_ev is not None:
            return None  # Local VIO checks remain; never publish/renew EV from a candidate.
        if result:
            self.last_ev_at = now
        return result

    def bind_external_ev(self, evidence):
        from external_ev_evidence import ExternalEvEvidence
        if (type(evidence) is not ExternalEvEvidence or self.external_ev is not None
                or self.ev.publish_count or self.start_consumed):
            raise ValueError('external EV must bind once before candidate generation/start')
        self.external_ev=evidence

    def external_ev_received(self, *args, **kwargs):
        if self.external_ev is None:raise ValueError('external EV not bound')
        accepted=self.external_ev.receive(*args,**kwargs)
        if self.external_ev.fault:self.trip(self.external_ev.fault)
        if accepted:self.last_ev_at=self.external_ev.last_at
        return accepted

    def ev_receipt_recent(self, now):
        if self.external_ev is not None:
            healthy=self.external_ev.healthy(now)
            if self.external_ev.fault:self.trip(self.external_ev.fault)
            return healthy
        return 0<=now-self.last_ev_at<=.3

    def _owns(self, now):
        return (not self.ownership_fault and self.ownership_ok
                and 0 <= now-self.ownership_at <= .5)

    def sent(self, command, stamp_us):
        # Repeated requests of the same command retain earliest outstanding
        # time, so a delayed ACK to the first request is still correlated.
        if self.pending is None or self.pending['command'] != command:
            self.pending = {'command': command, 'stamp': stamp_us}

    def ack(self, command, result, stamp_us, now):
        if (self.pending is None or command != self.pending['command']
                or stamp_us < self.pending['stamp']):
            return False
        self.acks.append({'command': command, 'result': result, 'at': now})
        if result in (2, 3, 4, 6):
            self.command_fault = f'{command} rejected: ACK result {result}'
        # ACCEPTED/IN_PROGRESS do NOT prove mode/arm/land state changed.
        return True

    def tick(self, now, start=False):
        if self.last_tick is not None and now-self.last_tick > .3:
            self.trip('runtime executive deadline missed')
        self.last_tick = now
        self.ev.watchdog(now)
        if self.ev.inhibit and self.ev_fault_at is None:
            self.ev_fault_at = now
        switch_fresh = self.switch_fresh(now)
        s = replace(self.sample,
                    ev_ok=self.ev_receipt_recent(now) and not self.ev.inhibit,
                    input_ownership_ok=self._owns(now),
                    manual_takeover=self.sample.manual_takeover or self.manual_latched,
                    kill_active=self.kill_latched)
        self.sample = s
        active = self.controller.state != 'WAIT'
        if active and (self.fault or self.command_fault or not switch_fresh):
            self.controller.abort(now, self.fault or self.command_fault or 'switch telemetry stale')
        if (start and not self.start_consumed and switch_fresh and self.kill_switch == 3
                and self.mode_slot == 6 and not self.fault and not self.command_fault):
            start = self.controller.ready(s, now)
        else:
            start = False
        if start:
            self.start_consumed = True
        return self.controller.step(now, s, start=start)

    def report(self):
        now = self.last_tick if self.last_tick is not None else self.started
        s = self.sample
        checks = {
            'fresh_vehicle_status': self.controller.fresh(s.status_at, now, 1.5),
            'fresh_local_position': self.controller.fresh(s.local_at, now),
            'fresh_estimator_flags': self.controller.fresh(s.flags_at, now, 2),
            'fresh_land_detector': self.controller.fresh(s.land_at, now, 2),
            'fresh_valid_rc': self.controller.fresh(s.rc_at, now) and s.rc_valid,
            'fresh_mode_kill_switches': self.switch_fresh(now),
            'kill_off': self.kill_switch == 3 and not self.kill_latched,
            'operator_offboard_slot': self.mode_slot == 6,
            'exclusive_inputs': self._owns(now),
            'ground_disarmed': s.landed and not s.armed,
            'px4_preflight': s.preflight_ok,
            'healthy_ev_and_estimator': self.controller.position_healthy(s),
        }
        return {'state': self.controller.state, 'reason': self.controller.reason,
                'source_rejection': self.source_rejection,
                'fault': self.fault, 'ev_fault': self.ev.fault,
                'ev_generated': self.ev.publish_count, 'states': self.controller.history,
                'last_ev_at': self.last_ev_at, 'ev_fault_at': self.ev_fault_at,
                'acks': self.acks, 'command_fault': self.command_fault,
                'mode_slot': self.mode_slot, 'kill_switch': self.kill_switch,
                'switch_input_received': self.switch_at >= 0,
                'switch_source': self.switch_source,
                'aux_decoder_fault': self.aux.fault if self.aux else None,
                'ownership_fault': self.ownership_fault,
                'battery_warning': self.battery_warning, 'battery_action': 'warning_only',
                'start_conditions': checks,
                'start_blockers': [name for name, passed in checks.items() if not passed],
                'flight_validation': False}
