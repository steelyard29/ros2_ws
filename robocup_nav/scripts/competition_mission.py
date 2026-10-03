"""2026 competition executive, offline only; no ROS, GPIO or PX4 imports.

Outputs are intentions, NOT actuator commands. Navigation/perception adapters
must supply fresh, correlated evidence. No target coordinates are configured.
Time is monotonic seconds supplied by the caller; step must run at >= 10 Hz.
"""
from dataclasses import dataclass
from enum import Enum
import math


class State(str, Enum):
    READY = 'READY'
    TAKEOFF = 'TAKEOFF'
    SEARCH = 'SEARCH'
    ALIGN = 'ALIGN'
    RELEASE = 'RELEASE'
    RETURN = 'RETURN'
    LAND = 'LAND'
    ABORT = 'ABORT'
    HANDOVER = 'HANDOVER'
    DONE = 'DONE'
    STOPPED = 'STOPPED'


@dataclass(frozen=True)
class Evidence:
    """Atomic adapter snapshot. Missing safety evidence defaults to unhealthy.

    height_m is validated height above takeoff ground (not raw dist_bottom).
    stable is the flight adapter's velocity/attitude stability verdict.
    landed must come from a land detector, not a height threshold.
    completion_id identifies the current intention; release_ok is physical
    release feedback, never merely successful transmission of a servo command.
    """
    at: float
    height_m: float = 0.0
    landed: bool = True
    armed: bool = False
    preflight_ok: bool = False
    localization_ok: bool = False
    link_ok: bool = False
    control_owned: bool = False
    stable: bool = False
    manual_override: bool = False
    collision: str = ''  # '', vegetation, wall, net, ground, gate, crash
    target_id: str = ''
    target_class: str = ''
    target_at: float = -1.0
    target_confidence: float = 0.0
    alignment_ok: bool = False
    completion_id: str = ''
    release_ok: bool = False
    navigation_ok: bool = False
    search_exhausted: bool = False


@dataclass(frozen=True)
class Intention:
    kind: str
    request_id: str = ''
    target_id: str = ''
    payload_slot: int = -1
    reason: str = ''


class CompetitionMission:
    """Direct-return route for three payloads; gate route is not implemented.

    Limits: rule p15 30 s liftoff; p11/12/15 600 s flight cap and 4 m;
    p12/13 >1 m stable >10 s. Return starts 60 s before the conservative
    deadline counted from start authorization. No numerical score is inferred.
    """
    TERMINAL = {State.HANDOVER, State.DONE, State.STOPPED}
    CLASSES = {'tent', 'bunker', 'bridge', 'armored_vehicle', 'tank', 'red_cross'}

    def __init__(self):
        self.state = State.READY
        self.started = None
        self.entered = None
        self.last_now = None
        self.hover_since = None
        self.align_since = None
        self.lifted = False
        self.takeoff_observed = False
        self.avoidance_eligible = True
        self.reason = ''
        self.target = ''
        self.target_class = ''
        self.released = []
        self.pending_slot = None
        self.uncertain_slots = []
        self.attempted_targets = set()
        self.sequence = 0
        self.request_id = ''
        self.release_sent = False
        self.history = []

    def _enter(self, state, now, reason=''):
        self.state, self.entered, self.reason = state, now, reason
        self.sequence += 1
        self.request_id = f'mission-{self.sequence}'
        self.align_since = None
        self.history.append({'at': now, 'state': state.value, 'reason': reason})

    def _interrupt_release(self):
        if self.pending_slot is not None and self.release_sent:
            if self.pending_slot not in self.uncertain_slots:
                self.uncertain_slots.append(self.pending_slot)

    def _abort(self, now, reason):
        self._interrupt_release()
        self._enter(State.ABORT, now, reason)

    def _target_valid(self, e, now):
        return (bool(e.target_id) and e.target_class in self.CLASSES
                and math.isfinite(e.target_at) and 0 <= now - e.target_at <= 0.3
                and math.isfinite(e.target_confidence)
                and 0.8 <= e.target_confidence <= 1.0)

    def step(self, now: float, e: Evidence, *, start: bool = False) -> Intention:
        if self.state in self.TERMINAL:
            return Intention('NONE', reason=self.reason)
        if not math.isfinite(now) or (self.last_now is not None and now < self.last_now):
            raise ValueError('monotonic finite time required')
        gap = self.last_now is not None and now - self.last_now > 0.5
        self.last_now = now
        fresh = math.isfinite(e.at) and 0 <= now - e.at <= 0.5
        height_ok = math.isfinite(e.height_m) and 0 <= e.height_m <= 4.0

        if self.state == State.READY:
            if start and fresh and height_ok and e.landed and not e.armed and (
                    e.preflight_ok and e.localization_ok and e.link_ok
                    and e.control_owned and not e.manual_override and not e.collision):
                self.started = now
                self._enter(State.TAKEOFF, now)
            return self._intention()

        # Manual intervention relinquishes ownership, including during abort.
        if fresh and e.manual_override:
            self._interrupt_release()
            self._enter(State.HANDOVER, now, 'manual intervention; no autonomous resume')
            return self._intention()

        # Abort remains latched; missing link cannot be repaired by a LAND intent.
        if self.state == State.ABORT:
            if fresh and e.landed and not e.armed:
                self._enter(State.STOPPED, now, self.reason)
            return self._intention()

        if fresh and e.collision:
            self.avoidance_eligible = False
        reason = ''
        if not fresh or gap:
            reason = 'telemetry or executive watchdog expired'
        elif not height_ok:
            reason = 'invalid height or 4 m ceiling exceeded'
        elif not e.link_ok:
            reason = 'flight-controller link lost; onboard failsafe required'
        elif not e.control_owned:
            reason = 'control ownership lost'
        elif e.collision and e.collision != 'vegetation':
            reason = 'collision: ' + e.collision
        elif now - self.started >= 600:
            reason = '600 s deadline'
        elif not e.localization_ok:
            reason = 'localization lost; new ground session required'
        if reason:
            self._abort(now, reason)
            return self._intention()
        if e.collision:
            self.avoidance_eligible = False

        if not e.landed:
            self.lifted = True
        if self.state not in {State.TAKEOFF, State.LAND} and (e.landed or not e.armed):
            self._abort(now, 'unexpected landing or disarm')
            return self._intention()

        if now - self.started >= 540 and self.state not in {State.RETURN, State.LAND}:
            self._interrupt_release()
            self._enter(State.RETURN, now, '60 s landing reserve')

        if self.state == State.TAKEOFF:
            if not self.lifted and now - self.started >= 30:
                self._abort(now, '30 s liftoff deadline')
            elif now - self.entered >= 60:
                self._abort(now, 'takeoff stabilization timeout')
            elif e.armed and not e.landed and e.height_m > 1.0 and e.stable:
                if self.hover_since is None:
                    self.hover_since = now
                if now - self.hover_since > 10:
                    self.takeoff_observed = True
                    self._enter(State.SEARCH, now)
            else:
                self.hover_since = None
        elif self.state == State.SEARCH:
            if e.search_exhausted and e.completion_id == self.request_id:
                self._enter(State.RETURN, now, 'reachable search area exhausted')
            elif self._target_valid(e, now) and e.target_id not in self.attempted_targets:
                self.target, self.target_class = e.target_id, e.target_class
                self._enter(State.ALIGN, now)
        elif self.state == State.ALIGN:
            if now - self.entered >= 20:
                self.attempted_targets.add(self.target)
                self._enter(State.SEARCH, now, 'alignment timeout')
            elif (self._target_valid(e, now) and e.target_id == self.target
                  and e.target_class == self.target_class and e.alignment_ok
                  and e.stable and e.completion_id == self.request_id):
                if self.align_since is None:
                    self.align_since = now
                if now - self.align_since >= 1.0:
                    self.pending_slot = len(self.released)
                    self.release_sent = False
                    self._enter(State.RELEASE, now)
            else:
                self.align_since = None
        elif self.state == State.RELEASE:
            if (e.release_ok and e.completion_id == self.request_id
                    and self.release_sent):
                self.released.append({'slot': self.pending_slot, 'target': self.target,
                                      'class': self.target_class, 'at': now})
                self.pending_slot = None
                self.attempted_targets.add(self.target)
                self._enter(State.RETURN if len(self.released) == 3 else State.SEARCH, now)
            elif now - self.entered >= 3:
                self._interrupt_release()
                self._enter(State.RETURN, now, 'release unconfirmed; no retry')
        elif self.state == State.RETURN:
            if e.navigation_ok and e.completion_id == self.request_id:
                self._enter(State.LAND, now)
            elif now - self.entered >= 40:
                self._abort(now, 'return navigation timeout')
        elif self.state == State.LAND:
            if e.landed and not e.armed:
                self._enter(State.DONE, now, 'direct-return landing observed; no landing score claimed')
            elif now - self.entered >= 30:
                self._abort(now, 'landing timeout')
        return self._intention()

    def _intention(self):
        kind = {
            State.READY: 'NONE', State.TAKEOFF: 'TAKEOFF_AND_HOLD',
            State.SEARCH: 'SEARCH_UNKNOWN_TARGETS', State.ALIGN: 'ALIGN_TARGET',
            State.RELEASE: 'WAIT_RELEASE_CONFIRMATION', State.RETURN: 'RETURN_TO_START',
            State.LAND: 'LAND', State.ABORT: 'ABORT_LAND_INTENT',
        }.get(self.state, 'NONE')
        if self.state == State.RELEASE and not self.release_sent:
            kind = 'RELEASE_ONCE'
            self.release_sent = True
        return Intention(kind, self.request_id, self.target,
                         self.pending_slot if self.pending_slot is not None else -1,
                         self.reason)
