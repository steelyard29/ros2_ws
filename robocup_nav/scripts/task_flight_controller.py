"""Navigation handoff around, not a replacement for, the original vertical controller.

No ROS publishers. Normal completion hands H-aligned cruise to original PX4
AUTO_LAND: this is NOT continuous visual correction during descent.
"""
from dataclasses import dataclass
from collections import deque
import math
from flight_supervisor import FlightSupervisor, FlightOutput
from live_flight_core import LiveFlightCore


@dataclass(frozen=True)
class TaskOutput(FlightOutput):
    velocity_xy: tuple | None = None


class TaskFlightSupervisor(FlightSupervisor):
    MAP_WAIT_SECONDS = 10.  # Bounded commissioning wait, not map coverage evidence.

    def __init__(self, hover=3.):
        super().__init__(height=.86, hover=hover)
        self.mission = self.alignment = self.scene = None
        self.session = None
        self.column_at = -1.
        self.handoff_at = None
        self.last_intention = None
        self.context_fault = None
        self.map_wait_since = None

    def context(self, mission, alignment, scene, session, column_at):
        if self.mission is not None and (mission is not self.mission or alignment is not self.alignment):
            raise ValueError('task context cannot be replaced within session')
        self.mission, self.alignment, self.scene = mission, alignment, scene
        self.session, self.column_at = session, column_at

    def ready(self, s, now):
        return bool(super().ready(s, now) and self.mission is not None
            and self.alignment is not None and self.scene is not None
            and self.fresh(self.column_at, now, .25)
            and self.fresh(self.scene.at, now, .25) and self.scene.localization_ok
            and self.context_fault is None)

    def _enter(self, state, now, reason=''):
        if state == 'LAND' and self.state == 'HOVER' and not reason:
            # Keep the ORIGINAL XYZ position-hold while online mapping/planning
            # starts. Do not switch to zero XY velocity over an unknown footprint.
            if self.map_wait_since is None:
                self.map_wait_since = now
            if now-self.map_wait_since >= self.MAP_WAIT_SECONDS:
                super()._enter('LAND',now,'online map/path unavailable within hover budget; original LAND, not H landing')
                return
            scene=self.scene
            available=(scene is not None and self.fresh(scene.at,now,.25)
                and scene.localization_ok and scene.airborne
                and self.fresh(scene.map_at,now,.5) and scene.footprint_known_free
                and self.fresh(scene.navigation_at,now,.25)
                and scene.navigation_goal_id==self.mission.goal_id
                and self.mission.stopping_clear(scene,scene.navigation_velocity_odom))
            if not available:
                self.reason='holding takeoff XY: waiting for online map and current-leg path'
                return
            state = 'NAVIGATE'
            self.handoff_at = now
        super()._enter(state, now, reason)

    def step(self, now, s, *, start=False, accepted_commands=None):
        if self.state not in {'NAVIGATE', 'ALIGN_H'}:
            if (self.state in {'STREAM','OFFBOARD','ARM','TAKEOFF','HOVER'}
                    and (not self.fresh(self.column_at,now,.25) or self.context_fault)):
                self.abort(now, self.context_fault or 'takeoff column evidence stale')
            output = super().step(now,s,start=start,accepted_commands=accepted_commands)
            if self.state != 'NAVIGATE':
                return output
            # No gap or second publisher on the handoff tick.
            return self._navigation(now,s,0.)
        dt = 0. if self.last_now is None else now-self.last_now
        self.last_now = now
        return self._navigation(now,s,dt)

    def _navigation(self, now, s, dt):
        if s.kill_active:
            self._enter('KILLED',now,'kill; no re-entry');return self._output(now)
        if (s.manual_takeover or not s.input_ownership_ok
                or (self.fresh(s.status_at,now,1.5) and s.nav_state not in (14,18))):
            self._enter('HANDOVER',now,'pilot/mode/ownership takeover');return self._output(now)
        try:
            scene=self.scene
            if (not math.isfinite(now) or not 0<=dt<=.3 or self.context_fault
                    or not self.fresh(s.status_at,now,1.5)
                    or not self.fresh(s.local_at,now,.5) or not self.fresh(s.flags_at,now,2.)
                    or not self.fresh(s.rc_at,now,.5) or not self.fresh(s.land_at,now,2.)
                    or not s.rc_valid or not s.armed or s.landed or s.failsafe
                    or s.nav_state!=14 or not self.position_healthy(s)
                    or s.reset_counters!=self.resets or now-self.started>=120.):
                raise ValueError(self.context_fault or 'navigation health/time/reset guard')
            if scene is None or not self.fresh(scene.at,now,.25) or not scene.localization_ok:
                raise ValueError('real navigation scene unavailable')
            self.alignment.validate(self.session,s.reset_counters)
            paired=self.alignment.position(scene.position,self.session,s.reset_counters)
            if math.dist(paired,(s.x,s.y,s.z))>.10:
                raise ValueError('VIO/PX4 paired position discrepancy')
            if scene.landed!=s.landed or not scene.airborne:
                raise ValueError('conflicting airborne observations')
            intent=self.mission.step(now,scene);self.last_intention=intent
            phase=intent['state']
            if phase=='HANDOVER':
                self._enter('HANDOVER',now,intent['reason']);return self._output(now)
            if phase=='LAND_REQUEST':
                # Return to original LAND command/ACK/landed-disarmed observation.
                super()._enter('LAND',now)
                return super().step(now,s,accepted_commands=frozenset())
            if phase not in ('NAVIGATE','ALIGN_H'):
                raise ValueError('mission cannot command cruise: '+intent['reason'])
            self.state=phase;self.reason=intent['reason']
            local_v=self.alignment.velocity((*intent['velocity_odom_xy'],0.),self.session,s.reset_counters)
            return TaskOutput(self.state,True,(math.nan,math.nan,self.origin[2]-self.height),
                self.origin[3],None,self.reason,tuple(float(v) for v in local_v[:2]))
        except (ValueError,TypeError,IndexError,ArithmeticError) as exc:
            self.abort(now,str(exc))
            return super().step(now,s,accepted_commands=frozenset())


class TaskFlightCore(LiveFlightCore):
    task_core=True

    def __init__(self,started,aux_params,task_config,hover=3.,abort_requested=None):
        super().__init__(started,aux_params,.86,hover,abort_requested)
        self.controller=TaskFlightSupervisor(hover)
        self.task_config=task_config
        self.local_history=deque(maxlen=128)

    def local(self,now,**fields):
        super().local(now,**fields)
        stamp=self.source_stamps.get('vehicle_local_position',0)*1000
        if stamp>0 and (not self.local_history or stamp>self.local_history[-1][0]):
            self.local_history.append((stamp,self.sample))

    def report(self):
        return {**super().report(),'task_profile':'roundtrip_1m_2p5m',
            'handoff_at':self.controller.handoff_at,
            'map_wait_since':self.controller.map_wait_since,
            'task_intention':self.controller.last_intention,
            'landing_method':'H alignment then original PX4 AUTO_LAND',
            'continuous_visual_descent':False}
