#!/usr/bin/env python3
"""Bounded full new runtime test: ideal DDS plant, synthetic map/H/nav commands.

Actual sensor/PX4 message types and production task adapter/serializer. No
real A*/APF in THIS test, no firmware SITL, hardware, flight permit or /fmu/in.
"""
import os
os.environ.update(ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1')
os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE',None)
from pathlib import Path
import sys
import json
import time
import math
import yaml
import numpy as np
import cv2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from nvblox_msgs.msg import DistanceMapSlice
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from task_map_fixture import markers
from flight_runtime_check import SyntheticPlant,synthetic_aux_params
from flight_runtime import create_node
from task_flight_controller import TaskFlightCore


class Plant(SyntheticPlant):
    def __init__(self):
        self.xy=np.zeros(2);self.vxy=np.zeros(2);self.step_at=time.monotonic()
        self.maximum_forward=0.;self.horizontal_messages=0
        self.heading=.8
        self.rot=np.array([[math.cos(.8),math.sin(.8)],[math.sin(.8),-math.cos(.8)]])
        super().__init__('nominal',use_aux=True)
        self.operator_position=True
        class PosePublisher:
            def __init__(port,pub):port.pub=pub
            def publish(port,m):
                m.pose.pose.position.x,m.pose.pose.position.y=map(float,self.xy)
                m.twist.twist.linear.x,m.twist.twist.linear.y=map(float,self.vxy)
                port.pub.publish(m)
        class LocalPublisher:
            def __init__(port,pub):port.pub=pub
            def publish(port,m):
                p=np.array([2.,-3.])+self.rot@self.xy;v=self.rot@self.vxy
                m.x,m.y=map(float,p);m.vx,m.vy=map(float,v);port.pub.publish(m)
        self.pubs['vio']=PosePublisher(self.pubs['vio'])
        self.pubs['vehicle_local_position']=LocalPublisher(self.pubs['vehicle_local_position'])

    def setpoint(self,m):
        if all(math.isnan(v) for v in m.position[:2]):
            assert math.isfinite(m.position[2]) and math.isnan(m.velocity[2])
            assert math.hypot(*m.velocity[:2])<=.15000003
            assert abs(m.yaw-.8)<1e-6
            self.vxy=self.rot.T@np.array(m.velocity[:2]);self.target_z=float(m.position[2])
            self.horizontal_messages+=1
        else:super().setpoint(m)

    def tick(self):
        now=time.monotonic();dt=min(.05,now-self.step_at);self.step_at=now
        if self.armed and self.mode==14:self.xy+=self.vxy*dt
        else:self.vxy[:]=0.
        self.maximum_forward=max(self.maximum_forward,float(self.xy[0]))
        super().tick()


def run(online_bootstrap=False):
    root=Path(__file__).resolve().parents[1]
    cfg=yaml.safe_load((root/'config/roundtrip_task.yaml').read_text())
    cfg.update(alignment_reviewed=True,bounds_odom=[-1,4,-2,2],full_height_slice_reviewed=True,
        takeoff_column_reviewed=True,landing_column_reviewed=True,landing_error_budget_m=.01)
    cfg['camera'].update(axes_reviewed=True,body_from_optical=[[0,-1,0],[-1,0,0],[0,0,-1]])
    rclpy.init(args=[]);plant=runtime=None;ex=SingleThreadedExecutor()
    started=time.monotonic();report={'passed':False,'hardware':False,'firmware_sitl':False,
        'online_bootstrap_fixture':online_bootstrap,
        'real_planner_used':False,'scene':'synthetic free map, ideal H centers, goal-seeking velocity fixture'}
    try:
        plant=Plant();core=TaskFlightCore(started,synthetic_aux_params(),cfg,hover=1.)
        runtime=create_node(core,isolated=True,exercise=True)
        pubs={k:plant.create_publisher(cls,runtime.task_input.topics[k],10) for k,cls in
              [('map',DistanceMapSlice),('bounds',Marker),('depth',Image),('navigation',String),('h',String)]}
        goal={'data':None}
        bootstrap={'first_map_at':None,'first_map_state':None,'horizontal_before_map':None,
                   'pre_takeoff_h_samples':0,'post_landing_h_samples':0}
        plant.create_subscription(String,runtime.task_input.goal_topic,
            lambda m:goal.update(data=json.loads(m.data)),10)
        def sensors():
            if online_bootstrap:
                wait=core.controller.map_wait_since
                if wait is None or time.monotonic()-wait<1.:
                    return  # No depth/map/H samples until airborne hover wait.
            if bootstrap['first_map_at'] is None:
                bootstrap.update(first_map_at=time.monotonic(),first_map_state=core.controller.state,
                                 horizontal_before_map=plant.horizontal_messages)
            stamp=plant.get_clock().now().to_msg();ns=stamp.sec*10**9+stamp.nanosec
            depth=Image();depth.header.stamp=stamp;depth.width=depth.height=1;depth.data=[1]
            pubs['depth'].publish(depth)
            m=DistanceMapSlice();m.header.stamp=stamp;m.header.frame_id='odom'
            m.width=m.height=100;m.resolution=.1;m.origin.x=m.origin.y=-5.
            m.origin.z=.86
            for bound in markers(stamp):pubs['bounds'].publish(bound)
            m.unknown_value=-1000.;m.data=[2.]*10000;pubs['map'].publish(m)
            g=goal['data']
            if g:
                v=.6*(np.array(g['position'][:2])-plant.xy);speed=np.linalg.norm(v)
                if speed>.15:v*=.15/speed
                d=dict(schema=1,frame_id='odom',goal_id=g['goal_id'],stamp_ns=ns,
                    path_stamp_ns=ns,velocity_xy=v.tolist())
                pubs['navigation'].publish(String(data=json.dumps(d)))
            camera=cfg['camera'];body=np.array([*plant.xy,.7-plant.z])
            ray=np.array(camera['body_from_optical']).T@(np.array([0,0,-.14])-body-np.array([.12,0,-.055]))
            stable=False;pixel=None
            if ray[2]>.05:
                pixel=cv2.projectPoints(ray.reshape(1,3),np.zeros(3),np.zeros(3),
                    np.array(camera['k']),np.array(camera['distortion']))[0].reshape(2)
                stable=bool(0<=pixel[0]<1280 and 0<=pixel[1]<720)
            h=dict(candidate_stable=stable,source_stamp_ns=ns,frame_id=camera['frame_id'],
                   center_px=pixel.tolist() if pixel is not None else [0,0],image_size=[1280,720])
            if not plant.armed:
                key=('pre_takeoff_h_samples' if core.controller.handoff_at is None
                     else 'post_landing_h_samples')
                bootstrap[key]+=1
            pubs['h'].publish(String(data=json.dumps(h)))
        plant.create_timer(.05,sensors);ex.add_node(plant);ex.add_node(runtime)
        while time.monotonic()-started<105:
            ex.spin_once(timeout_sec=.01)
            if core.ready_at is not None:plant.operator_position=False
            if core.controller.state in core.controller.TERMINAL:break
        report.update(runtime=runtime.report(),elapsed_s=time.monotonic()-started,
            bootstrap=bootstrap,
            final_xy=plant.xy.tolist(),maximum_forward_m=plant.maximum_forward,
            horizontal_messages=plant.horizontal_messages,commands=plant.commands)
        assert core.controller.state=='DONE',core.report()
        assert core.controller.handoff_at is not None and plant.horizontal_messages>50
        assert 2.4<plant.maximum_forward<2.6 and np.linalg.norm(plant.xy)<.05
        assert not plant.armed and core.landing_seen and 'LAND' in core.accepted_commands
        assert not any(t.startswith('/fmu/in/') for t,_ in runtime.get_topic_names_and_types())
        if online_bootstrap:
            assert bootstrap['first_map_state']=='HOVER',bootstrap
            assert bootstrap['horizontal_before_map']==0 and bootstrap['pre_takeoff_h_samples']==0,bootstrap
            assert core.controller.handoff_at>bootstrap['first_map_at'],bootstrap
        report['passed']=True
    except Exception as exc:
        report['error']=str(exc)
        if runtime:report['runtime']=runtime.report()
        raise
    finally:
        if runtime:runtime.destroy_node()
        if plant:plant.destroy_node()
        ex.shutdown();rclpy.try_shutdown()
        out=root/'evidence'/time.strftime('task_handoff_dds_%Y%m%d_%H%M%S');out.mkdir(parents=True,exist_ok=False)
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(evidence=str(out),passed=report['passed'],error=report.get('error'),
                             elapsed_s=report.get('elapsed_s')),indent=2))


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--online-bootstrap',action='store_true')
    run(parser.parse_args().online_bootstrap)
