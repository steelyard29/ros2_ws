"""Post-takeoff mission intentions only. No ROS/PX4 imports or live route.

Cruise navigation and H alignment are distinct states. LAND_REQUEST is only a
reviewable intention for a future single-writer adapter, never a command.
Takeoff and failure handling remain responsibilities of the flight supervisor.
"""
from dataclasses import dataclass
import math
from landing_projection import AlignmentPreview
from navigation_limits import horizontal_velocity
from task_site_bounds import validate_bounds,inside_bounds


@dataclass(frozen=True)
class Scene:
    at: float
    position: tuple
    velocity_xy: tuple=(0.,0.)
    localization_ok: bool=False
    airborne: bool=False
    map_at: float=-1.
    footprint_known_free: bool=False
    navigation_velocity_odom: tuple=(0.,0.)
    navigation_at: float=-1.
    goal_reached: bool=False
    h_error_odom: tuple=(0.,0.)
    h_at: float=-1.
    h_metric_verified: bool=False
    landing_boundary_verified: bool=False
    manual_override: bool=False
    landed: bool=False
    tilt_rad: float=float('nan')
    navigation_goal_id: str=''


class MissionShadow:
    def __init__(self,band,bounds,stopping_check=None,landing_error_limit=.05):
        validate_bounds(bounds)
        self.band,self.bounds=band,bounds
        self.state='WAIT_CRUISE';self.last=None;self.started=None
        self.landing_error_limit=landing_error_limit
        self.align=AlignmentPreview(landing_error_limit);self.h_missing=None
        self.stopping_check=stopping_check

    def stopping_clear(self,s,command):
        # Explicit map-backed evidence required; current footprint alone is not enough.
        if self.stopping_check is None:return False
        try:
            return bool(self.stopping_check(s.position[:2],s.velocity_xy,command))
        except (ValueError,TypeError,IndexError,ArithmeticError):
            return False

    def output(self,velocity=(0.,0.),reason=''):
        return {'state':self.state,'velocity_odom_xy':list(velocity),'reason':reason,
                'flight_authorized':False,'height_target_odom':self.band.center}

    def step(self,now,s):
        if self.state in ('ABORT','HANDOVER','DONE'):return self.output(reason='latched terminal state')
        if s.manual_override:self.state='HANDOVER';return self.output(reason='manual takeover')
        if (not math.isfinite(now) or (self.last is not None and not 0<now-self.last<=.3)
                or not 0<=now-s.at<=.25 or not s.localization_ok):
            self.state='ABORT';return self.output(reason='stale/reset/unhealthy localization')
        self.last=now
        if self.started is None:self.started=now
        if now-self.started>90:self.state='ABORT';return self.output(reason='bounded shadow mission timeout')
        if len(s.position)!=3 or not all(math.isfinite(v) for v in s.position):
            self.state='ABORT';return self.output(reason='invalid pose')
        x,y,z=s.position
        if not inside_bounds(self.bounds,(x,y)):self.state='ABORT';return self.output(reason='outside reviewed odom region')
        if self.state=='LAND_REQUEST':
            if s.landed and not s.airborne:
                self.state='DONE'
                return self.output(reason='shadow landed observation; no flight dispatch')
            valid=(s.airborne and self.band.contains_vehicle(z,s.tilt_rad)
                   and s.h_metric_verified and 0<=now-s.h_at<=.25
                   and s.landing_boundary_verified and s.footprint_known_free
                   and 0<=now-s.map_at<=.5 and self.stopping_clear(s,(0.,0.)))
            aligned=self.align.step(now,s.h_at,s.at,s.h_error_odom,s.velocity_xy)
            if not valid or not aligned['aligned']:
                self.state='ALIGN_H';self.align=AlignmentPreview(self.landing_error_limit)
                return self.output(reason='pending landing intention withdrawn: latest gates lost')
            return self.output(reason='intention only; no automatic flight dispatch')
        if not s.airborne or not self.band.contains_vehicle(z,s.tilt_rad):
            self.state='WAIT_CRUISE';self.align=AlignmentPreview(self.landing_error_limit)
            return self.output(reason='ground/ascent/descent: horizontal navigation inhibited')
        if self.state=='WAIT_CRUISE':self.state='NAVIGATE'
        if self.state=='NAVIGATE':
            if s.goal_reached:self.state='ALIGN_H'
            elif (not 0<=now-s.map_at<=.5 or not s.footprint_known_free
                  or not 0<=now-s.navigation_at<=.25):
                return self.output(reason='unknown footprint or stale map/command')
            else:
                v=s.navigation_velocity_odom
                try:
                    v=horizontal_velocity(v)
                except (TypeError,ValueError,OverflowError):
                    self.state='ABORT';return self.output(reason='invalid navigation velocity')
                if not self.stopping_clear(s,v):return self.output(reason='stopping space unverified/blocked')
                return self.output(v)
        if self.state=='ALIGN_H':
            if not s.h_metric_verified or not 0<=now-s.h_at<=.25:
                self.align=AlignmentPreview(self.landing_error_limit)
                if self.h_missing is None:self.h_missing=now
                if now-self.h_missing>3:self.state='ABORT'
                return self.output(reason='H metric observation missing; no blind descent')
            self.h_missing=None
            # Alignment translation also requires observed free swept space.
            if not s.footprint_known_free or not 0<=now-s.map_at<=.5:
                self.align=AlignmentPreview(self.landing_error_limit);return self.output(reason='alignment space unknown/stale')
            out=self.align.step(now,s.h_at,s.at,s.h_error_odom,s.velocity_xy)
            if not self.stopping_clear(s,out['velocity_xy']):
                self.align=AlignmentPreview(self.landing_error_limit)
                return self.output(reason='alignment stopping space unverified/blocked')
            if out['aligned'] and s.landing_boundary_verified:self.state='LAND_REQUEST'
            return self.output(out['velocity_xy'] if self.state=='ALIGN_H' else (0.,0.))
        return self.output()
