"""Joint mission/frame/message candidate, no ROS node, routes, or live authority.

Caller supplies synchronized verified observations; this does not certify them.
No automatic arm/land: LAND_REQUEST stays a pending intention with zero XY
velocity at cruise altitude. Real control ownership/handshake is NOT wired.
"""
import math
from navigation_frame_contract import build_shadow_messages


class ShadowPipeline:
    def __init__(self,mission,alignment,yaw_odom):
        if not math.isfinite(yaw_odom):raise ValueError('finite reviewed heading required')
        self.mission=mission;self.alignment=alignment;self.yaw=yaw_odom
        self.last=None;self.fault=None

    def step(self,now,scene,*,timestamp_us,vio_session,reset_counters,
             exclusive_writer_verified=False,yaw_rate_odom=0.):
        if self.fault:return {'messages':{},'fault':self.fault,'flight_authorized':False}
        try:
            if not exclusive_writer_verified:raise ValueError('control ownership lost/unverified')
            self.alignment.validate(vio_session,reset_counters)
            if not math.isfinite(yaw_rate_odom) or abs(yaw_rate_odom)>.2+1e-12:
                raise ValueError('invalid yaw rate')
            # DWB may emit 0.20000000000000007 at the configured endpoint.
            # Tolerate only roundoff and enforce the exact original limit.
            yaw_rate_odom=max(-.2,min(.2,yaw_rate_odom))
            intention=self.mission.step(now,scene)
            dt=0. if self.last is None else now-self.last
            self.last=now
            if intention['state'] not in ('NAVIGATE','ALIGN_H','LAND_REQUEST'):
                return {'messages':{},'intention':intention,'flight_authorized':False}
            # Only navigation may yaw. Stale/blocked commands produce a reason
            # and must not continue rotating while translation is inhibited.
            if intention['state']=='NAVIGATE' and not intention['reason']:
                self.yaw=math.atan2(math.sin(self.yaw+yaw_rate_odom*dt),
                                   math.cos(self.yaw+yaw_rate_odom*dt))
            velocity=(*intention['velocity_odom_xy'],0.)
            target=(*scene.position[:2],intention['height_target_odom'])
            messages=build_shadow_messages(self.alignment,timestamp_us,now=now,
                pose_at=scene.at,command_at=now,position_odom=target,
                velocity_odom=velocity,yaw_odom=self.yaw,vio_session=vio_session,
                reset_counters=reset_counters,exclusive_writer_verified=True)
            return {'messages':messages,'intention':intention,'flight_authorized':False}
        except (ValueError,TypeError,IndexError,ArithmeticError) as exc:
            self.fault=str(exc)
            return {'messages':{},'fault':self.fault,'flight_authorized':False}
