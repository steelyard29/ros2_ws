"""Resident EV-only ROS factory, explicit bound owner for real routing; no CLI.

Real deployment needs reviewed dual-domain source/session binding. Never starts
drivers/Agent, changes parameters, or creates command/setpoint publishers.
"""
import json
import time
from resident_ev_core import ResidentEvCore
from ev_odometry import apply_to_vehicle_odometry


def create_node(session, *, isolated=False, clock=time.monotonic, perception_node=None, owner=None,
                observation=None):
    if not isolated:
        from resident_ev_binding import ResidentEvOwner
        if type(owner) is not ResidentEvOwner:raise ValueError('resident real routing requires bound owner')
        owner.validate(session,perception_node)
    elif owner is not None:raise ValueError('isolated EV cannot use real owner')
    if perception_node is not None:
        from resident_vio_input import ScopedVioInput
        if type(perception_node) is not ScopedVioInput or perception_node.isolated!=isolated:
            raise ValueError('resident requires matching scoped perception input')
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data as qos
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from px4_msgs.msg import VehicleOdometry,VehicleStatus,VehicleLandDetected,EstimatorStatusFlags

    class ResidentNode(Node):
        def __init__(self):
            super().__init__('resident_ev_isolated',enable_rosout=False,
                             start_parameter_services=False,use_global_arguments=False)
            if self.context.get_domain_id() not in (range(180,188) if isolated else (0,)):
                self.destroy_node();raise ValueError('resident EV requires isolated domain')
            if perception_node is not None and (
                    perception_node.domain_id not in (range(180,188) if isolated else (176,))
                    or perception_node.domain_id==self.context.get_domain_id()):
                self.destroy_node();raise ValueError('resident split input requires distinct isolated domain')
            # The caller owns/pumps the perception port. Never forward sensor
            # topics into the flight domain or create a second EV publisher.
            self.perception_node=perception_node if perception_node is not None else self
            self.core=ResidentEvCore(clock(),session)
            self.started=clock();self.ready=False;self.closed=False
            self.receipts={};self.stamps={};self.status_msg=None;self.landed=None
            prefix='/robocup/runtime_test/input/'
            self.topics=dict(vio=prefix+'vio',tracking=prefix+'tracking',
                status=prefix+'vehicle_status_v1',flags=prefix+'estimator_status_flags',
                land=prefix+'vehicle_land_detected')
            self.output='/robocup/runtime_test/output/vehicle_visual_odometry'
            if not isolated:
                self.topics=dict(vio='/visual_slam/tracking/odometry',tracking='/robocup/alignment/tracking',
                    status='/fmu/out/vehicle_status_v1',flags='/fmu/out/estimator_status_flags',
                    land='/fmu/out/vehicle_land_detected')
                self.output='/fmu/in/vehicle_visual_odometry'
            self.pub=self.create_publisher(VehicleOdometry,self.output,qos)
            if observation is not None:
                from px4_msgs.msg import VehicleLocalPosition
                topic=prefix+'vehicle_local_position' if isolated else '/fmu/out/vehicle_local_position'
                self.create_subscription(VehicleLocalPosition,topic,
                    self.telemetry('local',self.observe_local),qos)
            for cls,key,cb in ((VehicleStatus,'status',self.status),
                              (VehicleLandDetected,'land',self.land),
                              (EstimatorStatusFlags,'flags',self.flags)):
                self.create_subscription(cls,self.topics[key],self.telemetry(key,cb),qos)
            self.perception_node.create_subscription(String,self.topics['tracking'],self.tracking,10)
            self.perception_node.create_subscription(Odometry,self.topics['vio'],self.pose,qos)
            self.create_timer(.1,self.audit)
            self.create_timer(.02,self.watchdog)

        def fail(self,reason):
            self.core.stop(reason)
            self.close_output()

        def close_output(self):
            self.closed=True
            if self.pub is not None:self.destroy_publisher(self.pub);self.pub=None

        def audit(self):
            if self.closed:return
            if owner is not None:
                try:owner.validate(session,self.perception_node)
                except Exception:self.fail('resident owner or input session lost');return
            identities={};missing=False
            for key,topic in self.topics.items():
                source_node=self.perception_node if key in ('vio','tracking') else self
                try:ends=source_node.get_publishers_info_by_topic(topic)
                except Exception:
                    self.fail('source graph unavailable');return
                if len(ends)>1:self.fail('duplicate source');return
                if not ends:missing=True
                else:identities[key]=tuple(ends[0].endpoint_gid)
            count=self.count_publishers(self.output)
            if missing or count==0:
                if self.ready or clock()-self.started>=10:self.fail('source/EV discovery lost or expired')
                return
            if not self.core.ownership(identities,count,clock()):self.close_output();return
            self.ready=True

        def telemetry(self,key,callback):
            def receive(m):
                if self.closed:return
                now=clock();stamp=int(m.timestamp)
                age=(self.get_clock().now().nanoseconds-stamp*1000)/1e9
                if stamp<=0 or not -.05<=age<=.5:self.fail('invalid telemetry source time');return
                old=self.stamps.get(key)
                if old is not None and stamp<old:self.fail('telemetry source reset');return
                if old==stamp:return
                self.stamps[key]=stamp;self.receipts[key]=now
                callback(m,now)
            return receive

        def vehicle_update(self):
            now=clock()
            if self.status_msg is None or self.landed is None:return
            if not self.telemetry_fresh(now):return
            self.core.vehicle(int(self.status_msg.arming_state),self.landed,self.receipts['status'])
            if self.core.fault:self.close_output()

        def status(self,m,now):
            self.status_msg=m
            if int(m.arming_state) not in (1,2):self.fail('unknown arming state');return
            if not self.core.ground_seen and int(m.arming_state)==2:self.fail('airborne startup');return
            self.vehicle_update()

        def land(self,m,now):
            self.landed=bool(m.landed);self.vehicle_update()

        def flags(self,m,now):
            if observation is not None:
                observation.flags_message(m,now,self.get_clock().now().nanoseconds)
            if not m.cs_baro_hgt or m.cs_rng_hgt:self.fail('height source contract mismatch');return
            self.core.flags(bool(m.cs_ev_hgt),bool(m.cs_ev_vel),now)
            self.vehicle_update()
            if self.core.fault:self.close_output()

        def observe_local(self,m,now):
            observation.local_message(m,now,
                self.status_msg is None or int(self.status_msg.arming_state)!=1,
                self.landed is True and self.telemetry_fresh(now))

        def telemetry_fresh(self,now):
            return all(k in self.receipts and 0<=now-self.receipts[k]<=limit
                       for k,limit in (('status',1.5),('land',2.),('flags',2.)))

        def tracking(self,m):
            if self.closed or not self.ready:return
            try:
                d=json.loads(m.data);stamp=int(d['stamp_ns'])
                age=(self.get_clock().now().nanoseconds-stamp)/1e9
                if not -.05<=age<=.25:raise ValueError('tracking age')
                self.core.tracking(int(d['vo_state']),stamp,clock())
            except (ValueError,TypeError,KeyError):self.fail('invalid tracking');return
            if self.core.fault:self.close_output()

        def pose(self,m):
            if self.closed or not self.ready:return
            if owner is not None:
                try:owner.validate(session,self.perception_node)
                except Exception:self.fail('resident owner or input session lost');return
            now=clock()
            if not self.telemetry_fresh(now):
                if self.core.sent_count:self.fail('telemetry stale')
                return
            stamp=int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
            p,q=m.pose.pose.position,m.pose.pose.orientation
            ev=self.core.pose(stamp,self.get_clock().now().nanoseconds,now,
                [p.x,p.y,p.z],[q.x,q.y,q.z,q.w],
                m.header.frame_id=='odom' and m.child_frame_id=='base_link')
            if self.core.fault:self.close_output();return
            if ev is None:return
            try:
                msg=VehicleOdometry();apply_to_vehicle_odometry(msg,ev)
                self.pub.publish(msg)
                self.core.dispatched(ev.timestamp_sample,clock())
                if observation is not None:
                    observation.pose(m,clock(),self.get_clock().now().nanoseconds)
            except Exception:
                self.fail('EV publication exception');raise

        def watchdog(self):
            if self.closed:return
            now=clock()
            if not self.ready:
                if now-self.started>=10:self.fail('discovery timeout')
                return
            if self.core.sent_count and not self.telemetry_fresh(now):self.fail('telemetry stale');return
            if not self.core.watchdog(now):self.close_output()
    return ResidentNode()
