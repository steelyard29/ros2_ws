"""ROS odom FLU to PX4 session-local frame, offline integration contract.

No north assumption, publishing, automatic calibration, or flight enable.
Both paired poses/headings must be independently validated and synchronized.
PX4 reset counters and VIO session changes invalidate the alignment forever.
"""
import math
import numpy as np
from navigation_limits import horizontal_velocity


def vec3(v):
    a=np.asarray(v,float)
    if a.shape!=(3,) or not np.isfinite(a).all():raise ValueError('finite 3D vector required')
    return a


def rz(yaw):
    if not math.isfinite(yaw):raise ValueError('finite heading required')
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])


class SessionAlignment:
    def __init__(self,odom_position,odom_yaw,local_position,local_heading,
                 vio_session,reset_counters,*,verified=False):
        if not verified:raise ValueError('independently verified paired alignment required')
        if not isinstance(vio_session,str) or not vio_session:raise ValueError('VIO session required')
        if len(reset_counters)!=5 or any(type(x) is not int or not 0<=x<=255 for x in reset_counters):
            raise ValueError('five PX4 reset counters required')
        self.odom_origin=vec3(odom_position);self.local_origin=vec3(local_position)
        self.rotation=rz(local_heading)@np.diag([1.,-1.,-1.])@rz(-odom_yaw)
        self.session=vio_session;self.resets=tuple(reset_counters);self.invalidated=False

    def validate(self,session,resets):
        if session!=self.session or tuple(resets)!=self.resets:self.invalidated=True
        if self.invalidated:raise ValueError('alignment invalidated; new reviewed session required')

    def position(self,p,session,resets):
        self.validate(session,resets)
        return self.local_origin+self.rotation@(vec3(p)-self.odom_origin)

    def velocity(self,v,session,resets):
        self.validate(session,resets)
        return self.rotation@vec3(v)

    def heading(self,yaw,session,resets):
        self.validate(session,resets)
        forward=self.rotation@rz(yaw)[:,0]
        return math.atan2(forward[1],forward[0])


def shadow_setpoint_fields(alignment,*,now,pose_at,command_at,position_odom,
                           velocity_odom,yaw_odom,vio_session,reset_counters,
                           exclusive_writer_verified=False):
    """Message-field preview only; intentionally not accepted by live serializer.

Mixed PX4 target: horizontal velocity and vertical position, no XY position
feed-forward. Actual mixed-mode behavior needs PX4 version/SITL validation.
"""
    if not exclusive_writer_verified:raise ValueError('single-writer ownership required')
    if not all(math.isfinite(v) for v in (now,pose_at,command_at)) or not (0<=now-pose_at<=.25 and 0<=now-command_at<=.25):
        raise ValueError('stale/future pose or navigation command')
    v=vec3(velocity_odom)
    v[:2]=horizontal_velocity(v[:2])
    if v[2]!=0:raise ValueError('horizontal trial limits exceeded')
    p=alignment.position(position_odom,vio_session,reset_counters)
    local_v=alignment.velocity(v,vio_session,reset_counters)
    return {'position':[math.nan,math.nan,float(p[2])],
            'velocity':[float(local_v[0]),float(local_v[1]),math.nan],
            'yaw':alignment.heading(yaw_odom,vio_session,reset_counters),
            'offboard_position_priority':True,'vehicle_command':None,
            'flight_authorized':False,'integration_status':'offline_contract_only'}


def build_shadow_messages(alignment,timestamp_us,**inputs):
    """Construct actual message objects WITHOUT a publisher or live route.

PX4 54f0455ffc PositionControl InputCombinationsPositionVelocity covers this
combination. Building messages does not prove the integrated flight behavior.
"""
    if type(timestamp_us) is not int or timestamp_us<=0:
        raise ValueError('positive integer ROS timestamp required')
    fields=shadow_setpoint_fields(alignment,**inputs)
    from px4_msgs.msg import OffboardControlMode,TrajectorySetpoint
    mode=OffboardControlMode();mode.timestamp=timestamp_us;mode.position=True
    target=TrajectorySetpoint();target.timestamp=timestamp_us
    target.position=fields['position'];target.velocity=fields['velocity']
    target.acceleration=[math.nan]*3;target.jerk=[math.nan]*3
    target.yaw=fields['yaw'];target.yawspeed=math.nan
    return {'offboard_control_mode':mode,'trajectory_setpoint':target}
