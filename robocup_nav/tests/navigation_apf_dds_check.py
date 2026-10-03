#!/usr/bin/env python3
"""Actual C++ APF + actual action bridge on synthetic domain176.

Planner is an explicitly FAKE straight-line ComputePathToPose server. This
checks action/goal/path/command correlation, not A* obstacle avoidance.
"""
import os
os.environ.update(ROS_DOMAIN_ID='176',ROS_LOCALHOST_ONLY='1')
os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE',None)
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry,Path as RosPath
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from navigation_goal_bridge import make_node
from flight_release import sha


def run():
    root=Path(__file__).resolve().parents[1]
    out=root/'evidence'/time.strftime('task_apf_link_%Y%m%d_%H%M%S');out.mkdir(parents=True,exist_ok=False)
    binary=root/'host_task_build/apf/legacy_apf_shadow'
    report=dict(passed=False,hardware=False,real_apf=True,real_astar=False,
                planner='synthetic straight-line action server',binary_sha256=sha(binary))
    log=(out/'apf.log').open('w');process=None;fixture=bridge=None;ex=None
    rclpy.init(args=[])
    try:
        process=subprocess.Popen([str(binary),'--ros-args','-p','real_sensor_inputs:=true',
            '-p','path_guided:=true','-p','arrival_tolerance:=0.05',
            '-p','controller_config_file:='+str(root/'config/legacy_apf_shadow.yaml')],stdout=log,stderr=log)
        fixture=Node('task_apf_fixture',use_global_arguments=False)
        bridge=make_node();ex=SingleThreadedExecutor();ex.add_node(fixture);ex.add_node(bridge)
        counts={'plans':0};commands=[]
        def plan(handle):
            counts['plans']+=1;request=handle.request
            result=ComputePathToPose.Result();path=RosPath();path.header=request.goal.header
            for i in range(26):
                p=PoseStamped();p.header=path.header;p.pose.orientation.w=1.
                p.pose.position.x=request.goal.pose.position.x*i/25.
                p.pose.position.y=request.goal.pose.position.y*i/25.
                path.poses.append(p)
            result.path=path;handle.succeed();return result
        server=ActionServer(fixture,ComputePathToPose,'/compute_path_to_pose',plan)
        pubs={k:fixture.create_publisher(cls,topic,10) for k,cls,topic in (
            ('goal',String,'/robocup/task/navigation_goal'),('odom',Odometry,'/visual_slam/tracking/odometry'),
            ('scan',LaserScan,'/robocup/legacy_apf/scan'))}
        fixture.create_subscription(String,'/robocup/task/navigation_command',
            lambda m:commands.append((time.monotonic(),json.loads(m.data))),10)
        leg={'name':'OUTBOUND','x':2.5,'send':True}
        def tick():
            stamp=fixture.get_clock().now().to_msg()
            m=Odometry();m.header.stamp=stamp;m.header.frame_id='odom';m.child_frame_id='base_link'
            m.pose.pose.orientation.w=1.;pubs['odom'].publish(m)
            s=LaserScan();s.header=m.header;s.header.frame_id='base_link'
            s.angle_min=-math.pi;s.angle_increment=2*math.pi/720;s.angle_max=s.angle_min+719*s.angle_increment
            s.range_min=.1;s.range_max=12.;s.ranges=[8.]*720;pubs['scan'].publish(s)
            if leg['send']:
                pubs['goal'].publish(String(data=json.dumps(dict(frame_id='odom',goal_id='test:'+leg['name'],
                    position=[leg['x'],0,.86],yaw=0.))))
        fixture.create_timer(.05,tick)
        def spin(seconds):
            until=time.monotonic()+seconds
            while time.monotonic()<until:
                if process.poll() is not None:raise RuntimeError('APF exited; inspect apf.log')
                ex.spin_once(timeout_sec=.01)
        spin(3.)
        assert any(d['goal_id']=='test:OUTBOUND' and d['velocity_xy'][0]>0 for _,d in commands),commands
        switched=time.monotonic();leg.update(name='RETURN',x=-2.5)
        spin(2.)
        assert any(t>switched and d['goal_id']=='test:RETURN' and d['velocity_xy'][0]<0 for t,d in commands)
        assert not any(t>switched+.15 and d['goal_id']=='test:OUTBOUND' for t,d in commands)
        leg['send']=False;stopped=time.monotonic();spin(1.2)
        assert not any(t>stopped+.65 for t,_ in commands),'expired goal lease produced navigation output'
        assert not any(t.startswith('/fmu/in/') for t,_ in fixture.get_topic_names_and_types())
        report.update(passed=True,plan_requests=counts['plans'],tagged_commands=len(commands),
                      return_sign_verified=True,expired_goal_inhibited=True)
        server.destroy()
    except Exception as exc:
        report['error']=str(exc);raise
    finally:
        if bridge:bridge.destroy_node()
        if fixture:fixture.destroy_node()
        if ex:ex.shutdown()
        rclpy.try_shutdown()
        if process:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=3)
            report['apf_process_exited']=process.poll() is not None
        log.close();(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(evidence=str(out),**report),indent=2))


if __name__=='__main__':run()
