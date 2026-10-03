"""Isolated test fixture only: ideal horizontal response, NOT PX4 dynamics.

Importing this module creates no ROS context or process. The optional ROS class
is instantiated only by an explicitly authorized isolated integration runner.
"""
import math
import time


class MixedXY:
    def __init__(self):
        self.x,self.y=2.,-3.
        self.vx=self.vy=0.
        self.target=(0.,0.)
        self.at=-math.inf
        self.stamp=0

    def accept(self,msg,now):
        p,v=tuple(msg.position),tuple(msg.velocity)
        if (len(p)!=3 or len(v)!=3 or not math.isfinite(now) or now<self.at
                or msg.timestamp<=self.stamp or not math.isfinite(msg.yaw)
                or abs(msg.yaw-.8)>1e-6 or not math.isfinite(p[2])
                or not -.160001<=p[2]<=.700001):
            raise ValueError('invalid synthetic task setpoint/time/height/yaw')
        if all(math.isnan(t) for t in p[:2]):
            if (not all(math.isfinite(t) for t in v[:2]) or not math.isnan(v[2])
                    or math.hypot(*v[:2])>.150001 or abs(p[2]+.16)>1e-6):
                raise ValueError('invalid synthetic mixed XY velocity/Z position')
            # Generated float arrays yield numpy.float32, while scalar ROS
            # message setters require native Python float.
            target=tuple(float(t) for t in v[:2])
        elif (all(math.isfinite(t) for t in p[:2]) and abs(p[0]-2.)<1e-6
              and abs(p[1]+3.)<1e-6 and all(math.isnan(t) for t in v)):
            target=(0.,0.)  # Original fixed-XY takeoff/hover, not a second planner.
        else:raise ValueError('unsupported synthetic XY position/velocity combination')
        self.target=target;self.at=now;self.stamp=msg.timestamp
        return float(p[2])

    def step(self,now,dt,*,armed,offboard,heartbeat_fresh):
        if not math.isfinite(dt) or not 0<=dt<=.1:raise ValueError('synthetic timestep invalid')
        enabled=armed and offboard and heartbeat_fresh and 0<=now-self.at<=.25
        self.vx,self.vy=self.target if enabled else (0.,0.)
        self.x+=self.vx*dt;self.y+=self.vy*dt

    def odom(self):
        # Inverse of Rz(.8) diag(1,-1,-1), independent scalar implementation.
        c,s=math.cos(.8),math.sin(.8);dx,dy=self.x-2.,self.y+3.
        return (c*dx+s*dy,s*dx-c*dy),(c*self.vx+s*self.vy,s*self.vx-c*self.vy)


def plant_type():
    """Load the existing ROS fixture lazily; caller sets isolated domain next."""
    from flight_runtime_check import SyntheticPlant

    class MixedTaskPlant(SyntheticPlant):
        def __init__(self):
            from rclpy.utilities import get_default_context
            import os
            if get_default_context().get_domain_id()!=182 or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
                raise ValueError('mixed test plant requires isolated domain182')
            self.xy=MixedXY();self.previous_tick=None
            super().__init__('nominal',use_aux=True)
            self.local_vio_publisher=self.pubs['vio']
            owner=self
            class VioAdapter:
                def publish(self,msg):
                    p,v=owner.xy.odom()
                    msg.pose.pose.position.x,msg.pose.pose.position.y=p
                    msg.twist.twist.linear.x,msg.twist.twist.linear.y=v
                    msg.twist.twist.linear.z=-owner.vz
                    owner.local_vio_publisher.publish(msg)
            self.pubs['vio']=VioAdapter()

        def setpoint(self,msg):
            self.counts['setpoint']+=1
            self.target_z=self.xy.accept(msg,time.monotonic())

        def tick(self):
            now=time.monotonic()
            dt=0. if self.previous_tick is None else now-self.previous_tick
            self.previous_tick=now
            self.xy.step(now,dt,armed=self.armed,offboard=self.mode==14,
                         heartbeat_fresh=0<=now-self.last_hb<=.25)
            super().tick()

        def publish(self,name,cls,stamp,**fields):
            if name=='vehicle_local_position':
                fields.update(x=self.xy.x,y=self.xy.y,vx=self.xy.vx,vy=self.xy.vy)
            super().publish(name,cls,stamp,**fields)

    return MixedTaskPlant
