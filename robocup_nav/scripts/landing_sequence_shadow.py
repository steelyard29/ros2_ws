"""One OFFLINE message path from navigation through H alignment and descent.

No ROS node/publisher, hardware command or flight enable. Inputs called verified
must be backed by independent evidence; synthetic tests certify no real geometry.
Existing live-flight baseline is intentionally not imported or modified.
"""
from dataclasses import dataclass
import math
from descent_shadow import DescentShadow,DescentObservation
from navigation_frame_contract import build_shadow_messages
from navigation_limits import horizontal_velocity


@dataclass(frozen=True)
class LandingEvidence:
    at: float
    ground_z: float
    body_lowest_offset_m: float
    geometry_verified: bool=False
    column_clear: bool=False
    bounds_verified: bool=False
    armed: bool=True
    anchor_drift_rate_bound_mps: float=float('nan')
    anchor_drift_model_verified: bool=False


class LandingSequenceShadow:
    def __init__(self,navigation,allowed_center_error_m,*,allow_verified_anchor=False):
        self.navigation=navigation
        self.descent=DescentShadow(allowed_center_error_m,allow_verified_anchor=allow_verified_anchor)
        if navigation.mission.landing_error_limit>allowed_center_error_m:
            raise ValueError('alignment must settle inside descent containment budget')
        self.anchor=None;self.floor=None;self.reference=None
        self.z_target=None;self.last=None;self.fault=None
        self.last_visual=None

    def refused(self,reason,state='ABORT'):
        self.fault=reason;self.descent.state=state
        return dict(state=state,messages={},fault=reason,flight_authorized=False)

    def step(self,now,scene,landing=None,**frame):
        if self.fault:return self.refused(self.fault,self.descent.state)
        if self.descent.state=='DONE':
            return dict(state='DONE',messages={},flight_authorized=False)
        if scene.manual_override:return self.refused('manual takeover','HANDOVER')
        if self.anchor is None:
            result=self.navigation.step(now,scene,**frame)
            result['state']=result.get('intention',{}).get('state','ABORT')
            if result.get('fault') or result['state'] in ('ABORT','HANDOVER'):
                return self.refused(result.get('fault',result.get('intention',{}).get('reason','navigation failed')),
                                    result['state'])
            if result['state']!='LAND_REQUEST':return result
            if not self.evidence_valid(now,landing):
                result['reason']='landing evidence missing; cruise hold only'
                return result
            self.anchor=tuple(scene.position[i]+scene.h_error_odom[i] for i in (0,1))
            self.reference=(landing.ground_z,landing.body_lowest_offset_m)
            self.floor=sum(self.reference);self.z_target=scene.position[2];self.last=now
            self.last_visual=scene.h_at
        try:
            if not frame.get('exclusive_writer_verified',False):raise ValueError('control ownership lost/unverified')
            self.navigation.alignment.validate(frame['vio_session'],frame['reset_counters'])
            if not self.evidence_valid(now,landing) or (landing.ground_z,landing.body_lowest_offset_m)!=self.reference:
                raise ValueError('landing geometry expired/changed')
            if not 0<=now-self.last<=.3:raise ValueError('executive deadline')
            dt=now-self.last;self.last=now
            from task_site_bounds import inside_bounds
            x,y,z=scene.position
            if not (inside_bounds(self.navigation.mission.bounds,(x,y)) and math.isfinite(scene.tilt_rad) and abs(scene.tilt_rad)<math.radians(10)):
                raise ValueError('landing position/tilt outside reviewed envelope')
            fresh_space=scene.footprint_known_free and 0<=now-scene.map_at<=.5
            error=tuple(self.anchor[i]-scene.position[i] for i in (0,1))
            h_fresh=scene.h_metric_verified and 0<=now-scene.h_at<=.25
            if h_fresh and math.dist(error,scene.h_error_odom)>self.descent.limit:
                raise ValueError('observed H moved outside locked-target budget')
            if h_fresh:self.last_visual=scene.h_at
            drift=landing.anchor_drift_rate_bound_mps
            model_ok=landing.anchor_drift_model_verified and math.isfinite(drift) and drift>=0
            observation=DescentObservation(at=scene.at,clearance_m=max(0.,z-self.floor),
                center_error_m=math.hypot(*error),xy_speed_mps=math.hypot(*scene.velocity_xy),
                localization_ok=scene.localization_ok,same_session=True,
                landing_column_clear=landing.column_clear and fresh_space,
                bounds_verified=landing.bounds_verified,h_visible=h_fresh,h_at=scene.h_at,
                landing_anchor_verified=landing.geometry_verified,
                landed=scene.landed,armed=landing.armed,anchor_at=self.last_visual,
                additional_anchor_error_m=drift*(now-self.last_visual) if model_ok else float('nan'),
                anchor_drift_model_verified=model_ok)
            out=self.descent.step(now,observation,begin=self.descent.state=='WAIT')
            if out['state'] in ('ABORT','HANDOVER'):return self.refused(out['reason'],out['state'])
            if out['state']=='DONE':return dict(state='DONE',messages={},flight_authorized=False)
            command=horizontal_velocity(tuple(.3*v for v in error),limit=.1)
            if not self.navigation.mission.stopping_clear(scene,command):
                raise ValueError('landing horizontal stopping space invalid')
            if abs(z-self.z_target)>.12:raise ValueError('descent tracking error exceeded')
            if out['vertical_velocity_odom']==0:
                self.z_target=z  # Freeze descent reference, do not keep integrating stale demand.
            else:
                self.z_target=max(self.floor,self.z_target+out['vertical_velocity_odom']*dt)
            if out['state']=='TOUCHDOWN':command=(0.,0.)
            messages=build_shadow_messages(self.navigation.alignment,frame['timestamp_us'],
                now=now,pose_at=scene.at,command_at=now,
                position_odom=(*self.anchor,self.z_target),velocity_odom=(*command,0.),
                yaw_odom=self.navigation.yaw,vio_session=frame['vio_session'],
                reset_counters=frame['reset_counters'],exclusive_writer_verified=True)
            return dict(state=out['state'],messages=messages,reason=out['reason'],
                        anchor_odom=self.anchor,height_target_odom=self.z_target,
                        anchor_age_s=now-self.last_visual,
                        additional_anchor_error_m=observation.additional_anchor_error_m,
                        flight_authorized=False)
        except (ValueError,TypeError,IndexError,ArithmeticError) as exc:
            return self.refused(str(exc))

    @staticmethod
    def evidence_valid(now,s):
        return bool(s is not None and all(math.isfinite(v) for v in (now,s.at,s.ground_z,s.body_lowest_offset_m))
                    and 0<=now-s.at<=.25 and .02<=s.body_lowest_offset_m<=1.
                    and s.geometry_verified and s.column_clear and s.bounds_verified)
