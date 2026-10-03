"""Actual ROS sensor/nav inputs -> task scene; never publishes flight commands.

Config verification entries require reviewed evidence, not a successful import.
Missing geometry/coverage is reported as a blocker, never synthesized as free.
"""
from collections import deque
import json
import math
import time
import uuid
import numpy as np
from task_map_profile import task_band,MapBandEvidence
from navigation_frame_contract import SessionAlignment
from navigation_landing_shadow import Scene
from navigation_limits import horizontal_velocity
from nav_math import rotation, yaw
from roundtrip_flight_test import RoundTripMission
from stopping_space import stopping_space_clear
from landing_projection import target_on_ground,nominal_landing_clearance
from task_pose_pairing import closest_local
from task_site_bounds import resolve_bounds,inside_bounds


class TaskSceneInput:
    TOPICS = dict(vio='/visual_slam/tracking/odometry',
        depth='/camera/camera/depth/image_rect_raw',map='/nvblox_node/static_map_slice',
        bounds='/nvblox_node/esdf_slice_bounds',
        h='/robocup/landing/h_candidate',navigation='/robocup/task/navigation_command')

    def __init__(self,node,core,clock=time.monotonic,isolated=False,*,read_only=False):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from nvblox_msgs.msg import DistanceMapSlice
        from visualization_msgs.msg import Marker
        from rclpy.qos import qos_profile_sensor_data as qos
        self.node,self.core,self.clock=node,core,clock
        self.read_only=read_only
        self.cfg=core.task_config
        self.session=str(uuid.uuid4())
        self.seen={};self.stamps={};self.poses=deque(maxlen=40)
        self.pose_data=self.grid=self.h_target=self.command=None
        self.mission=self.alignment=None
        self.band_evidence=MapBandEvidence();self.mapper_identity=None
        self.blockers=['waiting for actual inputs']
        self.navigation_blockers=['waiting for airborne map/path']
        self.fault=None
        self.String=String
        # Tests explicitly remap every input; no real input subscriptions in tests.
        self.topics={k:('/robocup/task_test/input/'+k if isolated else v) for k,v in self.TOPICS.items()}
        if isolated:
            self.topics['vio']='/robocup/runtime_test/input/vio'
        self.goal_topic='/robocup/task_test/navigation_goal' if isolated else '/robocup/task/navigation_goal'
        self.goal_pub=None if read_only else node.create_publisher(String,self.goal_topic,10)
        self.status_pub=None if read_only else node.create_publisher(String,
            '/robocup/task_test/readiness' if isolated else '/robocup/task/readiness',10)
        self.subs=[node.create_subscription(cls,self.topics[key],cb,quality) for cls,key,cb,quality in (
            (Odometry,'vio',self.pose,qos),(Image,'depth',self.depth,qos),
            (Marker,'bounds',self.bounds,10),
            (DistanceMapSlice,'map',self.map,10),(String,'h',self.h,10),
            (String,'navigation',self.navigation,10))]
        self.last_status=-1.

    def owns(self):
        return (all(self.node.count_publishers(t)<=1 for t in self.topics.values())
            and self.node.count_publishers(self.goal_topic)==(0 if self.read_only else 1))

    def now_ns(self):return self.node.get_clock().now().nanoseconds

    def stamp(self,key,stamp,limit):
        self.seen.pop(key,None)
        if (type(stamp) is not int or stamp<=0 or stamp<=self.stamps.get(key,0)
                or not 0<=self.now_ns()-stamp<=int(limit*1e9)):
            return False
        self.stamps[key]=stamp
        # Source age, not callback reception, determines validity.
        self.seen[key]=self.clock()-(self.now_ns()-stamp)/1e9
        return True

    @staticmethod
    def ns(msg):return int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec)

    def fail(self,reason):
        self.fault=self.fault or reason
        self.core.controller.context_fault=self.fault

    def pose(self,m):
        try:
            stamp=self.ns(m)
            if stamp<=self.stamps.get('vio',0):self.fail('VIO source timestamp reset/replay')
            if not self.stamp('vio',stamp,.25):return
            if m.header.frame_id!='odom' or m.child_frame_id!='base_link':raise ValueError('VIO frames')
            p,q,v=m.pose.pose.position,m.pose.pose.orientation,m.twist.twist.linear
            a=np.array([p.x,p.y,p.z]);quat=np.array([q.x,q.y,q.z,q.w])
            if not np.isfinite(a).all() or abs(np.linalg.norm(quat)-1)>.01:raise ValueError('VIO pose')
            r=rotation(quat);world=r@np.array([v.x,v.y,v.z])
            if not np.isfinite(world).all():raise ValueError('VIO twist')
            if self.pose_data is not None:
                old=self.pose_data
                delta=abs(math.atan2(math.sin(yaw(quat)-old[4]),math.cos(yaw(quat)-old[4])))
                if math.dist(a,old[1])>.5 or delta>.5:raise ValueError('VIO jump/reset')
            self.pose_data=(stamp,a,r,world,yaw(quat))
            self.poses.append(self.pose_data)
        except (ValueError,TypeError,AttributeError) as exc:self.fail(str(exc))

    def depth(self,m):
        if m.width>0 and m.height>0 and len(m.data)>0:self.stamp('depth',self.ns(m),.5)
        else:self.seen.pop('depth',None)

    def map(self,m):
        self.grid=None
        try:
            if not self.stamp('map',self.ns(m),.5):return
            data=np.asarray(m.data,dtype=float)
            if (m.header.frame_id!='odom' or m.width*m.height!=len(data) or not len(data)
                    or not math.isfinite(m.resolution) or m.resolution<=0
                    or not np.isfinite(data).all()
                    or not all(math.isfinite(v) for v in (m.origin.x,m.origin.y))):
                raise ValueError('invalid ESDF slice')
            free=((data!=m.unknown_value)&(data>0)).reshape(m.height,m.width)
            self.grid=(free,float(m.resolution),(float(m.origin.x),float(m.origin.y)))
            self.band_evidence.slice(m.origin.z,self.seen['map'])
        except (ValueError,TypeError,AttributeError):self.seen.pop('map',None)

    def bounds(self,m):
        key='bounds:'+m.ns
        if self.stamp(key,self.ns(m),.75):
            self.band_evidence.marker(m,self.seen[key])
        elif m.ns in self.band_evidence.bounds:
            self.band_evidence.bounds.pop(m.ns,None)

    def mapper_owned(self):
        groups=[self.node.get_publishers_info_by_topic(self.topics[k]) for k in ('map','bounds')]
        if any(len(x)!=1 for x in groups):return False
        a,b=(x[0] for x in groups)
        if (a.node_name,a.node_namespace)!=(b.node_name,b.node_namespace):return False
        identity=(bytes(a.endpoint_gid),bytes(b.endpoint_gid))
        if self.mapper_identity is not None and identity!=self.mapper_identity:
            self.fail('mapper publisher changed; new map/VIO session required');return False
        self.mapper_identity=identity
        return True

    def navigation(self,m):
        self.command=None
        try:
            d=json.loads(m.data)
            if (type(d.get('schema')) is not int or d['schema']!=1 or d.get('frame_id')!='odom'
                    or self.mission is None or d.get('goal_id')!=self.mission.goal_id
                    or type(d.get('path_stamp_ns')) is not int
                    or not 0<=self.now_ns()-d['path_stamp_ns']<=500_000_000
                    or not self.stamp('navigation',d['stamp_ns'],.25)):return
            v=horizontal_velocity(d['velocity_xy'])
            self.command=(d['goal_id'],tuple(v))
        except (ValueError,TypeError,KeyError,OverflowError):return

    def h(self,m):
        self.h_target=None
        try:
            d=json.loads(m.data)
            if d.get('candidate_stable') is not True or not self.stamp('h',d['source_stamp_ns'],.25):return
            if self.mission is None or not self.poses:return
            p=min(self.poses,key=lambda x:abs(x[0]-d['source_stamp_ns']))
            if abs(p[0]-d['source_stamp_ns'])>100_000_000:return
            camera=self.cfg['camera']
            if d.get('frame_id')!=camera['frame_id']:return
            self.h_target=target_on_ground(d['center_px'],camera['k'],camera['distortion'],
                d['image_size'],camera['image_size'],p[1],p[2],camera['body_from_optical'],
                camera['offset_flu_m'],self.mission.band.floor,
                calibration_verified=camera['calibration_use_authorized'],
                axes_verified=camera['axes_reviewed'])
        except (ValueError,TypeError,KeyError,IndexError):return

    def stopping(self,position,velocity,command):
        now=self.clock()
        if (self.grid is None or not self.cfg['full_height_slice_reviewed']
                or not self.mapper_owned()
                or self.mission is None or not self.band_evidence.check(self.mission.band,now)
                or not all(0<=now-self.seen.get(k,-1e9)<=.5 for k in ('map','depth'))):return False
        speed=max(math.hypot(*velocity),math.hypot(*command))
        model=self.cfg['stopping']
        excursion=speed*model['latency']+speed*speed/(2*model['braking'])
        if not inside_bounds(self.mission.bounds,position,excursion):return False
        return stopping_space_clear(*self.grid,position,velocity,command,**model)

    def prepare(self,now):
        c=self.core.controller;s=self.core.sample
        self.blockers=[]
        self.navigation_blockers=[]
        if self.fault:self.blockers.append(self.fault)
        for k,limit in (('vio',.25),):
            if not 0<=now-self.seen.get(k,-1e9)<=limit:self.blockers.append('stale/missing '+k)
        for k,limit in (('map',.5),('depth',.5)):
            if not 0<=now-self.seen.get(k,-1e9)<=limit:self.navigation_blockers.append('stale/missing '+k)
        if not self.owns():self.blockers.append('task input/goal ownership')
        if not self.mapper_owned():self.navigation_blockers.append('map/bounds publisher identity unavailable')
        if self.mission is None:
            if not self.cfg['alignment_reviewed']:self.blockers.append('paired alignment evidence not reviewed')
            try:resolve_bounds(self.cfg,(0.,0.),0.)
            except (ValueError,TypeError,KeyError):self.blockers.append('reviewed robot-center bounds not set')
            if not c.fresh(s.local_at,now,.1) or s.armed or not s.landed:self.blockers.append('ground PX4 pose missing')
            pair=(closest_local(self.pose_data[0],getattr(self.core,'local_history',()),
                               self.now_ns(),now,s.reset_counters) if self.pose_data is not None else None)
            self.pairing_evidence=dict(matched=pair is not None,
                stamp_field='VehicleLocalPosition.timestamp',
                vio_stamp_ns=self.pose_data[0] if self.pose_data is not None else None,
                px4_stamp_ns=pair[0] if pair else None,
                difference_ms=abs(pair[0]-self.pose_data[0])/1e6 if pair else None)
            if pair is None:self.blockers.append('fresh same-reset VIO/PX4 pair within 50ms missing')
            if not self.blockers:
                try:
                    _,p,_,_,heading=self.pose_data
                    band=task_band(p[2])
                    paired_sample=pair[1]
                    self.alignment=SessionAlignment(p,heading,
                        (paired_sample.x,paired_sample.y,paired_sample.z),paired_sample.heading,
                        self.session,paired_sample.reset_counters,verified=self.cfg['alignment_reviewed'])
                    bounds=resolve_bounds(self.cfg,p[:2],heading)
                    self.mission=RoundTripMission(band,bounds,self.stopping,
                        ground_origin=p,yaw_odom=heading,session_id=self.session)
                except (ValueError,TypeError) as exc:self.fail(str(exc))
        if self.mission is not None and self.pose_data is not None:
            _,p,r,v,_=self.pose_data
            if not self.band_evidence.check(self.mission.band,now):
                self.navigation_blockers.append(self.band_evidence.reason)
            try:self.alignment.validate(self.session,s.reset_counters)
            except ValueError as exc:self.fail(str(exc))
            error=(0.,0.) if self.h_target is None else tuple(self.h_target[:2]-p[:2])
            budget=self.cfg['landing_error_budget_m']
            clearance=nominal_landing_clearance(self.cfg['landing_geometry'])
            boundary=(budget is not None and math.isfinite(budget) and budget>=0
                and math.hypot(*error)+budget<clearance and self.cfg['landing_column_reviewed'])
            scene=Scene(at=self.seen.get('vio',-1.),position=tuple(p),velocity_xy=tuple(v[:2]),
                localization_ok=not self.fault and not self.core.ev.inhibit,
                airborne=s.armed and not s.landed,landed=s.landed,
                map_at=self.seen.get('map',-1.),footprint_known_free=self.stopping(p[:2],v[:2],(0.,0.)),
                navigation_velocity_odom=self.command[1] if self.command else (0.,0.),
                navigation_goal_id=self.command[0] if self.command else '',
                navigation_at=self.seen.get('navigation',-1.) if self.command else -1.,
                h_error_odom=error,h_at=self.seen.get('h',-1.),h_metric_verified=self.h_target is not None,
                landing_boundary_verified=boundary,manual_override=s.manual_takeover,
                tilt_rad=math.acos(max(-1.,min(1.,float(r[2,2])))))
            # Reviewed initial cleared volume is explicit evidence; not inferred
            # from a 2D cruise slice or described as sensor-verified 3D coverage.
            # Online mapping/H visibility are NOT prerequisites for the reviewed
            # vertical takeoff column. Their gates apply before lateral motion.
            column=(self.cfg['takeoff_column_reviewed'] and not self.blockers and not self.fault)
            if not getattr(self,'read_only',False):
                c.context(self.mission,self.alignment,scene,self.session,now if column else -1.)
            if not getattr(self,'read_only',False) and c.state in ('HOVER','NAVIGATE','ALIGN_H'):
                self.goal_pub.publish(self.String(data=json.dumps(self.mission.planner_goal())))
        if not getattr(self,'read_only',False) and now-self.last_status>=.5:
            self.last_status=now
            self.status_pub.publish(self.String(data=json.dumps(self.report())))

    def report(self):
        config_blockers=[k for k in ('alignment_reviewed','full_height_slice_reviewed',
            'takeoff_column_reviewed','landing_column_reviewed') if self.cfg.get(k) is not True]
        try:resolve_bounds(self.cfg,(0.,0.),0.)
        except (ValueError,TypeError,KeyError):config_blockers.append('bounds_odom or complete takeoff site')
        if self.cfg.get('landing_error_budget_m') is None:config_blockers.append('landing_error_budget_m')
        if not self.cfg.get('camera',{}).get('axes_reviewed'):config_blockers.append('camera optical-to-body rotation')
        return dict(blockers=self.blockers,fault=self.fault,initialized=self.mission is not None,
            read_only=getattr(self,'read_only',False),
            accepted_source_stamps=dict(self.stamps),
            initial_pose_pairing=getattr(self,'pairing_evidence',None),
            input_source_ages_s={key:self.clock()-at for key,at in self.seen.items()},
            navigation_blockers=self.navigation_blockers,
            configuration_blockers=config_blockers,
            map_height_consistency=self.band_evidence.reason,
            input_topics=self.topics,session_id=self.session,
            full_height_slice_reviewed=self.cfg['full_height_slice_reviewed'],
            takeoff_column_reviewed=self.cfg['takeoff_column_reviewed'],
            h_metric_available=self.h_target is not None,flight_authorized=False)
