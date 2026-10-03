"""Vertical landing rehearsal only; no ROS, PX4, arming, disarming or live route.

Clearance means lowest airframe point above the reviewed landing plane. It is
NOT unvalidated TFmini range. Frozen odom target and error bounds require prior
independent verification. Zero requested velocity is not proof of actual stop.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DescentObservation:
    at: float
    clearance_m: float
    center_error_m: float
    xy_speed_mps: float
    localization_ok: bool=False
    same_session: bool=False
    landing_column_clear: bool=False
    bounds_verified: bool=False
    h_visible: bool=False
    h_at: float=-1.
    landing_anchor_verified: bool=False
    landed: bool=False
    armed: bool=True
    manual_override: bool=False
    anchor_at: float=-1.
    additional_anchor_error_m: float=float('nan')
    anchor_drift_model_verified: bool=False


class DescentShadow:
    def __init__(self,allowed_center_error_m,*,allow_verified_anchor=False,max_anchor_age_s=20.):
        if not math.isfinite(allowed_center_error_m) or not 0<allowed_center_error_m<=.1:
            raise ValueError('positive reviewed containment error budget <= 0.1m required')
        self.limit=allowed_center_error_m;self.state='WAIT'
        if not math.isfinite(max_anchor_age_s) or not 0<max_anchor_age_s<=20:
            raise ValueError('bounded anchor observation age within 0..20s required')
        self.allow_anchor=allow_verified_anchor;self.max_anchor_age=max_anchor_age_s
        self.last=None;self.started=None;self.commit=None;self.touch_since=None

    def output(self,vz=0.,reason=''):
        return dict(state=self.state,vertical_velocity_odom=vz,reason=reason,
                    flight_authorized=False,vehicle_command=None)

    def step(self,now,s,*,begin=False):
        if self.state in ('ABORT','HANDOVER','DONE'):return self.output(reason='terminal latch')
        if s.manual_override:self.state='HANDOVER';return self.output(reason='manual takeover')
        values=(now,s.at,s.clearance_m,s.center_error_m,s.xy_speed_mps)
        if (not all(math.isfinite(v) for v in values) or not 0<=now-s.at<=.25
            or (self.last is not None and not 0<now-self.last<=.3)
            or min(s.clearance_m,s.center_error_m,s.xy_speed_mps)<0):
            self.state='ABORT';return self.output(reason='invalid/stale observations')
        self.last=now
        healthy=(s.localization_ok and s.same_session and s.landing_column_clear
                 and s.bounds_verified and s.landing_anchor_verified)
        fresh_h=s.h_visible and 0<=now-s.h_at<=.25
        bounded_anchor=(self.allow_anchor and s.anchor_drift_model_verified
            and math.isfinite(s.anchor_at) and 0<=now-s.anchor_at<=self.max_anchor_age
            and math.isfinite(s.additional_anchor_error_m) and s.additional_anchor_error_m>=0
            and s.center_error_m+s.additional_anchor_error_m<self.limit)
        aligned=s.center_error_m<self.limit and s.xy_speed_mps<.05
        if self.state=='WAIT':
            if not (begin and healthy and fresh_h and aligned):return self.output(reason='entry prerequisites missing')
            self.state='DESCEND';self.started=now
        if not healthy or now-self.started>30:
            self.state='ABORT';return self.output(reason='landing geometry/localization/session invalid or timeout')
        if s.landed:
            # Do not issue an in-air disarm; a separate validated executor/PX4
            # must supply fresh actual land + arming status, not a timer guess.
            self.state='TOUCHDOWN'
            if not s.armed:
                if self.touch_since is None:self.touch_since=now
                if now-self.touch_since>=2:self.state='DONE'
            else:self.touch_since=None
            return self.output(reason='waiting for continuous landed and disarmed confirmation')
        if self.state=='TOUCHDOWN':
            self.state='ABORT';return self.output(reason='touchdown lost; no automatic re-descent')
        if not aligned:
            self.state='ABORT';return self.output(reason='outside landing error/speed budget')
        if s.clearance_m>.25:
            if not fresh_h and not bounded_anchor:return self.output(reason='H/verified bounded anchor unavailable; hold descent')
            return self.output(-.1,'visible target descent preview' if fresh_h else 'verified bounded anchor descent preview')
        if self.commit is None:
            if not fresh_h and not bounded_anchor:return self.output(reason='cannot enter terminal descent without H or verified anchor')
            self.commit=now
        if self.allow_anchor and not fresh_h and not bounded_anchor:
            self.state='ABORT';return self.output(reason='terminal anchor error/age budget expired')
        if now-self.commit>6:
            self.state='ABORT';return self.output(reason='terminal descent budget expired')
        self.state='TERMINAL_DESCEND'
        # A locked ground target can survive H leaving the FOV only while all
        # previously checked bounds remain valid. No default assumed bounds.
        return self.output(-.05 if s.clearance_m>0 else 0.,'bounded locked-target terminal preview')
