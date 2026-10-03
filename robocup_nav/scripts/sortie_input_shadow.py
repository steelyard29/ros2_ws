"""Real ROS message schemas, fixed synthetic input/EV routes, domain176 only.

No drivers, agent, serial devices, /fmu subscriptions or configurable routes.
Telemetry -> existing production freshness/AUX/EV policy -> ReadySortieShadow.
Scene/map/H geometry is supplied separately, not inferred from telemetry.
"""
import json
import os
import time
from collections import Counter
from ev_odometry import apply_to_vehicle_odometry
from sortie_ready_shadow import ReadySortieShadow


class SortieInputShadow:
    INPUT = '/robocup/sortie_shadow/input/'
    EV = '/robocup/sortie_shadow/output/vehicle_visual_odometry'

    def __init__(self,node,ready,writer,clock=time.monotonic):
        if os.environ.get('ROS_DOMAIN_ID')!='176' or type(ready) is not ReadySortieShadow:
            raise ValueError('isolated domain176 and dedicated readiness core required')
        from rclpy.qos import qos_profile_sensor_data as qos
        from px4_msgs.msg import (VehicleStatus,VehicleLocalPosition,EstimatorStatusFlags,
            VehicleLandDetected,ManualControlSetpoint,VehicleCommandAck,VehicleOdometry)
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        self.node,self.ready,self.writer,self.clock=node,ready,writer,clock
        self.policy=ready.policy
        self.started=clock()
        self.counts=Counter()
        self.ev_count=0
        self.ev_pub=node.create_publisher(VehicleOdometry,self.EV,10)
        self.ev_type=VehicleOdometry
        specs=[(VehicleStatus,'vehicle_status_v1',self.status),
            (VehicleLocalPosition,'vehicle_local_position',self.local),
            (EstimatorStatusFlags,'estimator_status_flags',self.flags),
            (VehicleLandDetected,'vehicle_land_detected',lambda m:self.policy.land(clock(),bool(m.landed))),
            (ManualControlSetpoint,'manual_control_setpoint',self.aux),
            (VehicleCommandAck,'vehicle_command_ack',self.ack)]
        self.inputs=[self.INPUT+name for _,name,_ in specs]+[self.INPUT+'vio',self.INPUT+'tracking']
        self.subs=[node.create_subscription(cls,self.INPUT+name,self.wrap(name,callback),qos)
                   for cls,name,callback in specs]
        self.subs.append(node.create_subscription(Odometry,self.INPUT+'vio',self.pose,qos))
        self.subs.append(node.create_subscription(String,self.INPUT+'tracking',self.tracking,10))

    def owns(self):
        own=self.writer.owns() and self.node.count_publishers(self.EV)==1
        own &= all(self.node.count_publishers(t)<=1 for t in self.inputs)
        own &= self.node.count_publishers(self.INPUT+'manual_control_switches')==0
        return bool(own)

    def audit(self):
        own=self.owns()
        if own or self.clock()-self.started>2:
            self.policy.ownership(self.clock(),own)
        return bool(own)

    def wrap(self,name,callback):
        def receive(msg):
            try:
                stamp=int(msg.timestamp)
                age=(self.node.get_clock().now().nanoseconds/1000-stamp)/1e6
                if self.policy.receive(name,stamp,self.clock(),age):
                    self.counts[name]+=1
                    callback(msg)
            except (ValueError,TypeError,KeyError,AttributeError) as exc:
                self.policy.trip('invalid isolated telemetry: '+str(exc))
        return receive

    def status(self,m):
        p=self.policy;now=self.clock()
        p.gcs_status(not bool(m.gcs_connection_lost),now)
        p.status(int(m.arming_state),now,nav_state=int(m.nav_state),
            preflight_ok=bool(m.pre_flight_checks_pass),failsafe=bool(m.failsafe),
            manual_takeover=bool(m.failsafe_and_user_took_over))

    def local(self,m):
        fields={k:float(getattr(m,k)) for k in ('x','y','z','heading','vx','vy','vz')}
        fields.update({k:bool(getattr(m,k)) for k in ('xy_valid','z_valid','v_xy_valid','v_z_valid')})
        fields['reset_counters']=tuple(int(getattr(m,k)) for k in
            ('xy_reset_counter','z_reset_counter','heading_reset_counter','vxy_reset_counter','vz_reset_counter'))
        self.policy.local(self.clock(),**fields)

    def flags(self,m):
        self.policy.flags(self.clock(),ev_position=bool(m.cs_ev_pos),ev_yaw=bool(m.cs_ev_yaw),
            ev_height=bool(m.cs_ev_hgt),ev_velocity=bool(m.cs_ev_vel),
            baro_height=bool(m.cs_baro_hgt),range_height=bool(m.cs_rng_hgt))

    def aux(self,m):
        age=(self.node.get_clock().now().nanoseconds/1000-int(m.timestamp))/1e6
        self.policy.aux_message(int(m.timestamp),int(m.timestamp_sample),self.clock(),age,
            bool(m.valid),int(m.data_source),float(m.aux1),float(m.aux2))

    def ack(self,m):
        if int(m.target_system)==1 and int(m.target_component)==191:
            self.policy.ack({176:'OFFBOARD',400:'ARM',21:'LAND'}.get(int(m.command)),
                int(m.result),int(m.timestamp),self.clock())

    def tracking(self,m):
        try:
            d=json.loads(m.data)
            self.policy.tracking(int(d['vo_state']),int(d['stamp_ns']),self.clock())
            self.counts['tracking']+=1
        except (ValueError,TypeError,KeyError):
            self.policy.trip('invalid isolated tracking JSON')

    def pose(self,m):
        stamp=int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
        now_ns=self.node.get_clock().now().nanoseconds
        p,q=m.pose.pose.position,m.pose.pose.orientation
        result=self.policy.pose(stamp,(now_ns-stamp)/1e6,self.clock(),(p.x,p.y,p.z),
            (q.x,q.y,q.z,q.w),m.header.frame_id=='odom' and m.child_frame_id=='base_link',now_ns)
        self.counts['vio']+=1
        if result is not None and self.audit():
            try:
                msg=self.ev_type();apply_to_vehicle_odometry(msg,result)
                self.ev_pub.publish(msg);self.ev_count+=1
            except Exception:
                self.policy.close('isolated EV publication failed')
                self.ready.sortie.stop('isolated EV publication failed')
                raise
