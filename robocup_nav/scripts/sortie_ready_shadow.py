"""Isolated composition of production READY/AUX/EV policy and full sortie.

Not a LiveFlightCore, no FlightPermit or real output route. The policy's
FlightOutput is a receipt only; mixed mission messages use SortieShadowWriter.
Faults fence candidates, NOT a claim that a physical aircraft stops safely.
"""
import math
from live_flight_core import LiveFlightCore
from flight_supervisor import FlightOutput, FlightSupervisor
from sortie_shadow import SortieShadow


class _ControllerPort:
    def __init__(self, owner):
        self.owner = owner

    def __getattr__(self, name):
        return getattr(self.owner.sortie.takeoff, name)

    @property
    def state(self):
        return self.owner.sortie.state

    @property
    def reason(self):
        return self.owner.sortie.reason

    def abort(self, now, reason):
        self.owner.sortie.stop(reason)

    def ready(self, sample, now):
        return self.owner.ground_context_ok(now) and self.owner.sortie.takeoff.ready(sample, now)

    def step(self, now, sample, *, start=False, accepted_commands=None):
        o = self.owner
        c = o.context
        if o.sortie.state == 'WAIT' and not start:
            o.result = o.sortie.result({})
            return FlightOutput('WAIT')
        result = o.sortie.step(now, sample, c['scene'], c['landing'], c['space'],
            timestamp_us=c['stamp'], vio_session=c['session'], start=start,
            accepted_commands=accepted_commands or frozenset(),
            switch_fresh=o.policy.switch_fresh(now), mode_slot=o.policy.mode_slot,
            kill_switch=o.policy.kill_switch)
        o.result = result
        cmd = result['messages'].get('vehicle_command')
        command = {176:'OFFBOARD', 400:'ARM'}.get(int(cmd.command)) if cmd else None
        return FlightOutput(result['state'], command=command, reason=result['reason'])


class _ReadyPolicy(LiveFlightCore):
    def __init__(self, owner, started, aux_params):
        super().__init__(started, aux_params, owner.sortie.takeoff.height, owner.sortie.takeoff.hover)
        self.owner = owner
        self.controller = _ControllerPort(owner)

    def ev_allowed(self, now):
        # Automatic disarm can precede completion of the contact dwell. Continue
        # observation ONLY in the already-established terminal landing region;
        # never restart EV after a previous fault or adopt an unexpected disarm.
        if not self.owner.contact_pending(now):
            return super().ev_allowed(now)
        s, fresh = self.sample, FlightSupervisor.fresh
        return bool(not self.closed and not self.ev_stopped and not self.ev.inhibit
            and not self.fault and not self.command_fault and not self.kill_latched
            and not self.manual_latched and not self.abort_requested()
            and self.last_tick is not None and 0 <= now-self.last_tick <= .3
            and self._owns(now) and fresh(s.status_at, now, 1.5)
            and fresh(s.land_at, now, 2.) and fresh(s.rc_at, now, .5) and s.rc_valid
            and fresh(s.flags_at, now, 2.) and s.baro_height
            and not s.range_height and not s.ev_velocity and not s.failsafe
            and self.switch_fresh(now) and self.kill_switch == 3 and self.mode_slot == 6
            and not s.manual_takeover and s.nav_state == 14)


class ReadySortieShadow:
    isolated_only = True

    def __init__(self, sortie, started, aux_params):
        if type(sortie) is not SortieShadow:
            raise ValueError('dedicated SortieShadow required')
        self.sortie, self.context = sortie, None
        self.result = sortie.result({})
        self.policy = _ReadyPolicy(self, started, aux_params)

    def ground_context_ok(self, now):
        c = self.context
        if c is None:
            return False
        s, space = c['scene'], c['space']
        fresh = FlightSupervisor.fresh
        return bool(space is not None and space.column_verified and fresh(space.at, now, .25)
            and fresh(s.at, now, .25) and s.localization_ok and len(s.position) == 3
            and all(math.isfinite(v) for v in s.position)
            and abs(s.position[2]+self.sortie.takeoff.height
                    -self.sortie.sequence.navigation.mission.band.center) <= 1e-5)

    def contact_pending(self, now):
        c, seq, s = self.context, self.sortie.sequence, self.policy.sample
        pending = bool(c is not None and not s.armed and s.landed
            and self.sortie.state not in self.sortie.TERMINAL
            and seq.descent.state in ('TERMINAL_DESCEND','TOUCHDOWN') and seq.floor is not None
            and FlightSupervisor.fresh(s.local_at, now, .5)
            and FlightSupervisor.fresh(c['scene'].at, now, .25)
            and abs(c['scene'].position[2]-seq.floor) <= .03)
        if not pending:
            return False
        # DDS land/status callbacks can precede the next scene executive tick.
        # Use both fresh local altitude and the last near-floor scene here;
        # SortieShadow still requires matching current landed observations
        # before emitting a terminal descent batch.
        a = seq.navigation.alignment
        try:
            floor = a.position((*c['scene'].position[:2],seq.floor),c['session'],s.reset_counters)
            return abs(s.z-floor[2]) <= .03
        except (ValueError,TypeError,IndexError):
            return False

    def tick(self, now, scene, landing, space, *, timestamp_us, vio_session, exercise=True):
        self.context = dict(scene=scene, landing=landing, space=space,
                            stamp=timestamp_us, session=vio_session)
        self.result = self.sortie.result({})  # No stale batch reuse on exceptions/closed policy.
        try:
            receipt = self.policy.tick(now, start=exercise)
            if self.policy.closed:
                return self.sortie.stop(self.policy.stop_reason or 'policy closed')
            if receipt.state != self.sortie.state:
                raise ValueError('READY/sortie state mismatch')
            command = self.result['messages'].get('vehicle_command')
            if command is not None:
                command.source_component = 191
            return self.result
        except Exception as exc:
            self.policy.close('isolated task exception: '+str(exc))
            return self.sortie.stop('isolated task exception: '+str(exc))

    def sent(self, result):
        """Call only after the sole writer accepted the whole candidate batch."""
        msg = result['messages'].get('vehicle_command')
        if msg is not None:
            self.policy.sent({176:'OFFBOARD',400:'ARM'}[int(msg.command)], int(msg.timestamp))

    def report(self):
        return dict(policy=self.policy.report(), sortie_state=self.sortie.state,
                    isolated_only=True, real_route_available=False, flight_validation=False)
