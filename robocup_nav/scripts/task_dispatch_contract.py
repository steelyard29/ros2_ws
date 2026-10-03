"""Separate bounded XY-velocity/Z-position boundary; old vertical guard retained."""
import math
from flight_dispatch_contract import FlightDispatchContract
from flight_supervisor import FlightSupervisor
from flight_messages import build_messages
from navigation_limits import horizontal_velocity


class TaskDispatchContract:
    def __init__(self):
        self.vertical=FlightDispatchContract()
        self.fault=None
        self.last_now=self.last_stamp=None
        self.horizontal_started=False
        self.terminal=False
        self.landing_started=False
        self.previous_state=None

    def check(self,output,sample,*,now,stamp,state,origin,height,
              switch_fresh,mode_slot,kill_switch,exclusive):
        try:
            if self.fault:raise ValueError(self.fault)
            if (not math.isfinite(now) or now<0 or type(stamp) is not int or stamp<=0
                    or self.last_now is not None and now<=self.last_now
                    or self.last_stamp is not None and stamp<=self.last_stamp):
                raise ValueError('task dispatch replay/clock fault')
            self.last_now,self.last_stamp=now,stamp
            if self.terminal and (output.stream or output.command):
                raise ValueError('terminal task cannot resume')
            if state not in ('NAVIGATE','ALIGN_H'):
                if self.horizontal_started and state in self.vertical.STREAM_STATES:
                    raise ValueError('cannot re-enter takeoff after navigation')
                if getattr(output,'velocity_xy',None) is not None:
                    raise ValueError('horizontal fields outside task phase')
                self.vertical.check(output,sample,now=now,stamp=stamp,state=state,origin=origin,
                    height=height,switch_fresh=switch_fresh,mode_slot=mode_slot,
                    kill_switch=kill_switch,exclusive=exclusive)
                if state in FlightSupervisor.TERMINAL or state in ('LAND','ABORT'):
                    self.terminal=state in FlightSupervisor.TERMINAL
                    self.landing_started=True
                self.previous_state=state
                return
            if self.landing_started or (not self.horizontal_started and self.previous_state!='HOVER'):
                raise ValueError('navigation requires stable-hover handoff, no LAND re-entry')
            if output.state!=state or not output.stream or output.command is not None:
                raise ValueError('horizontal task must be stream only')
            f=FlightSupervisor.fresh
            if (not exclusive or not sample.input_ownership_ok or not switch_fresh
                    or mode_slot!=6 or kill_switch!=3 or sample.kill_active or sample.manual_takeover
                    or not sample.armed or sample.landed or sample.nav_state!=14 or sample.failsafe
                    or not sample.rc_valid or not f(sample.status_at,now,1.5)
                    or not f(sample.local_at,now,.5) or not f(sample.rc_at,now,.5)
                    or not f(sample.flags_at,now,2.) or not f(sample.land_at,now,2.)
                    or not FlightSupervisor.position_healthy(sample)):
                raise ValueError('horizontal dispatch telemetry/ownership guard')
            if (origin is None or len(origin)!=4 or not all(math.isfinite(v) for v in origin)
                    or not math.isclose(height,.86,abs_tol=1e-9)
                    or output.position is None or len(output.position)!=3
                    or not all(math.isnan(v) for v in output.position[:2])
                    or not math.isfinite(output.position[2]) or not math.isfinite(output.yaw)
                    or abs(output.position[2]-(origin[2]-height))>1e-6
                    or abs(output.yaw-origin[3])>1e-6):
                raise ValueError('horizontal height/yaw/mixed-mode envelope')
            horizontal_velocity(output.velocity_xy)
            if abs(sample.z-(origin[2]-height))>.08:
                raise ValueError('horizontal output outside cruise altitude')
            self.horizontal_started=True
            self.previous_state=state
        except (TypeError,ValueError,OverflowError) as exc:
            self.fault=self.fault or str(exc);raise ValueError(self.fault) from exc


def build_task_messages(output,stamp):
    if getattr(output,'velocity_xy',None) is None:
        return build_messages(output,stamp)
    if (output.state not in ('NAVIGATE','ALIGN_H') or not output.stream or output.command is not None
            or type(stamp) is not int or stamp<=0 or output.position is None
            or len(output.position)!=3 or not all(math.isnan(v) for v in output.position[:2])
            or not math.isfinite(output.position[2]) or not math.isfinite(output.yaw)):
        raise ValueError('invalid task serialization')
    v=horizontal_velocity(output.velocity_xy)
    from px4_msgs.msg import OffboardControlMode,TrajectorySetpoint
    mode=OffboardControlMode();mode.timestamp=stamp;mode.position=True
    target=TrajectorySetpoint();target.timestamp=stamp
    target.position=list(output.position);target.velocity=[float(v[0]),float(v[1]),math.nan]
    target.yaw=float(output.yaw);target.yawspeed=math.nan
    target.acceleration=[math.nan]*3;target.jerk=[math.nan]*3
    return dict(offboard_control_mode=mode,trajectory_setpoint=target)
