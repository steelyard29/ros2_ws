#!/usr/bin/env python3
"""Actual task A*/nvblox plugin/goal bridge/APF, synthetic static scene only.

No GPU mapper, PX4, sensor driver or simulated flight dynamics. Never calls the
real launch: that launch intentionally puts APF in domain 0. Here every child
inherits localhost-only domain 176. Overall deadline 55 s plus bounded cleanup.
"""
import os
os.environ.update(ROS_DOMAIN_ID='176', ROS_LOCALHOST_ONLY='1')
os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
import importlib.util
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
from nvblox_msgs.msg import DistanceMapSlice

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from navigation_goal_bridge import make_node
from flight_release import sha


def run():
    out=ROOT/'evidence'/time.strftime('task_astar_apf_%Y%m%d_%H%M%S')
    out.mkdir(parents=True, exist_ok=False)
    spec=importlib.util.spec_from_file_location('task_launch', ROOT/'launch/task_navigation.launch.py')
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    cfg=module.planner_parameters()
    params=out/'planner.yaml'; params.write_text(yaml.safe_dump(cfg))
    binary=ROOT/'host_task_build/apf/legacy_apf_shadow'
    plugin=ROOT/'host_task_install/lib/libnvblox_nav2.so'
    sources=[ROOT/'launch/task_navigation.launch.py',ROOT/'config/navigation.yaml',
        ROOT/'config/legacy_apf_shadow.yaml',ROOT/'scripts/navigation_goal_bridge.py',
        ROOT/'scripts/navigation_goal_link.py',ROOT/'legacy_apf_shadow/node.cpp',
        Path(__file__),binary,plugin]
    report=dict(passed=False,hardware=False,real_astar=True,real_apf=True,
        real_nvblox_costmap_plugin=True,gpu_mapper=False,flight_dynamics=False,px4_sitl=False,
        domain=176,source_sha256={str(p):sha(p) for p in sources})
    processes=[]; logs=[]; fixture=bridge=executor=None
    start=time.monotonic(); deadline=start+55
    rclpy.init(args=[])
    try:
        fixture=Node('task_astar_fixture',use_global_arguments=False)
        bridge=make_node();executor=SingleThreadedExecutor()
        executor.add_node(fixture);executor.add_node(bridge)
        def spin(seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                if time.monotonic()>deadline:raise TimeoutError('55 s total test deadline')
                if any(p.poll() is not None for p in processes):raise RuntimeError('child exited; inspect logs')
                executor.spin_once(timeout_sec=.01)
        def wait(predicate,seconds=8):
            end=min(deadline,time.monotonic()+seconds)
            while not predicate():
                if time.monotonic()>end:raise TimeoutError('condition timeout')
                spin(.02)
        spin(.4)
        names=[name for name,_ in fixture.get_node_names_and_namespaces()]
        if any(n in names for n in ('planner_server','legacy_apf_shadow')):
            raise RuntimeError('isolated domain already occupied')
        if any(n.startswith('/fmu/in/') for n,_ in fixture.get_topic_names_and_types()):
            raise RuntimeError('forbidden flight input in test domain')
        def spawn(label,args):
            log=(out/(label+'.log')).open('w');logs.append(log)
            processes.append(subprocess.Popen(args,stdout=log,stderr=log,start_new_session=True))
        spawn('planner',['/opt/ros/humble/lib/nav2_planner/planner_server','--ros-args','--params-file',str(params)])
        spawn('lifecycle',['/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager','--ros-args',
            '-r','__node:=isolated_task_planner_lifecycle','-p','autostart:=true','-p','node_names:=[planner_server]'])
        spawn('apf',[str(binary),'--ros-args','-p','real_sensor_inputs:=true','-p','path_guided:=true',
            '-p','arrival_tolerance:=0.05','-p','controller_config_file:='+str(ROOT/'config/legacy_apf_shadow.yaml')])
        pubs={key:fixture.create_publisher(cls,topic,10) for key,cls,topic in (
            ('slice',DistanceMapSlice,'/nvblox_node/static_map_slice'),
            ('odom',Odometry,'/visual_slam/tracking/odometry'),
            ('scan',LaserScan,'/robocup/legacy_apf/scan'),
            ('goal',String,'/robocup/task/navigation_goal'))}
        tf=TransformBroadcaster(fixture); paths=[];commands=[]
        fixture.create_subscription(RosPath,'/robocup/legacy_apf/path',lambda m:paths.append((time.monotonic(),m)),10)
        fixture.create_subscription(String,'/robocup/task/navigation_command',
            lambda m:commands.append((time.monotonic(),json.loads(m.data))),10)
        state=dict(x=1.,goal_x=5.,leg='OUTBOUND',send=True,map_kind='wall')
        msg=DistanceMapSlice();msg.header.frame_id='odom'
        msg.resolution=.05;msg.width=120;msg.height=120;msg.origin.z=.86;msg.unknown_value=-1000.
        def set_map(kind):
            cells=np.full((120,120),2.,dtype=np.float32)
            cells[[0,-1],:]=-.01;cells[:,[0,-1]]=-.01
            if kind=='wall':cells[28:73,57:63]=-.01
            else:cells[:,57:63]=msg.unknown_value if kind=='unknown' else -.01
            msg.data=cells.ravel().tolist();state['map_kind']=kind
        set_map('wall')
        def tick():
            stamp=fixture.get_clock().now().to_msg()
            od=Odometry();od.header.stamp=stamp;od.header.frame_id='odom';od.child_frame_id='base_link'
            od.pose.pose.orientation.w=1.;od.pose.pose.position.x=state['x']
            od.pose.pose.position.y=2.5;od.pose.pose.position.z=.86;pubs['odom'].publish(od)
            t=TransformStamped();t.header=od.header;t.child_frame_id='base_footprint'
            t.transform.rotation.w=1.;t.transform.translation.x=state['x'];t.transform.translation.y=2.5
            tf.sendTransform(t)
            msg.header.stamp=stamp;pubs['slice'].publish(msg)
            s=LaserScan();s.header.stamp=stamp;s.header.frame_id='base_link'
            s.angle_min=-math.pi;s.angle_increment=2*math.pi/720;s.angle_max=s.angle_min+719*s.angle_increment
            s.range_min=.1;s.range_max=12.
            # Exact slab ray intersections with the same synthetic wall/boundary.
            boxes=[(0.,.05,0.,6.),(5.95,6.,0.,6.),(0.,6.,0.,.05),(0.,6.,5.95,6.)]
            if state['map_kind']=='wall':boxes.append((2.85,3.15,1.4,3.65))
            elif state['map_kind']=='blocked':boxes.append((2.85,3.15,0.,6.))
            ranges=[]
            for i in range(720):
                a=s.angle_min+i*s.angle_increment;dx,dy=math.cos(a),math.sin(a);nearest=11.
                for x0,x1,y0,y1 in boxes:
                    enter,leave=0.,12.
                    for origin,d,lo,hi in ((state['x'],dx,x0,x1),(2.5,dy,y0,y1)):
                        if abs(d)<1e-10:
                            if not lo<=origin<=hi:leave=-1.
                        else:
                            aa,bb=sorted(((lo-origin)/d,(hi-origin)/d));enter=max(enter,aa);leave=min(leave,bb)
                    if leave>=enter and enter>0:nearest=min(nearest,enter)
                ranges.append(nearest)
            s.ranges=ranges;pubs['scan'].publish(s)
            if state['send']:
                pubs['goal'].publish(String(data=json.dumps(dict(frame_id='odom',goal_id='astar:'+state['leg'],
                    position=[state['goal_x'],2.5,.86],yaw=0.))))
        fixture.create_timer(.05,tick)
        life=fixture.create_client(GetState,'/planner_server/get_state')
        wait(life.service_is_ready,12)
        for _ in range(12):
            f=life.call_async(GetState.Request());wait(f.done)
            if f.result().current_state.id==3:break
            spin(.4)
        else:raise RuntimeError('planner not active')
        pclient=fixture.create_client(GetParameters,'/planner_server/get_parameters')
        wait(pclient.service_is_ready)
        req=GetParameters.Request();req.names=['GridBased.use_astar','GridBased.allow_unknown','GridBased.tolerance']
        f=pclient.call_async(req);wait(f.done);values=f.result().values
        assert values[0].bool_value and not values[1].bool_value and abs(values[2].double_value-.05)<1e-9
        report['active_planner_parameters_verified']=True
        wait(lambda:any(len(p.poses)>2 and max(abs(q.pose.position.y-2.5) for q in p.poses)>1.
                        for _,p in paths))
        wait(lambda:any(d['goal_id']=='astar:OUTBOUND' and d['velocity_xy'][0]>0 for _,d in commands))
        report['astar_detour_and_tagged_apf_outbound']=True
        switched=time.monotonic();state.update(x=5.,goal_x=1.,leg='RETURN')
        wait(lambda:any(t>switched and d['goal_id']=='astar:RETURN' and d['velocity_xy'][0]<0 for t,d in commands))
        spin(.5)
        assert not any(t>switched+.2 and d['goal_id']=='astar:OUTBOUND' for t,d in commands)
        report['new_return_goal_rejects_old_leg']=True
        for kind in ('blocked','unknown'):
            set_map(kind);changed=time.monotonic();spin(2.)
            assert any(t>changed+1. and not p.poses for t,p in paths),'failed plan did not clear path'
            assert not any(t>changed+1.3 for t,d in commands),'blocked/unknown corridor still produced command'
            report[kind+'_barrier_inhibited']=True
            set_map('wall');changed=time.monotonic()
            wait(lambda:any(t>changed+.6 and d['velocity_xy'][0]<0 for t,d in commands))
        state['send']=False;stopped=time.monotonic();spin(1.2)
        assert not any(t>stopped+.65 for t,d in commands),'expired task goal produced output'
        assert not any(n.startswith('/fmu/in/') for n,_ in fixture.get_topic_names_and_types())
        assert report['source_sha256']=={str(p):sha(p) for p in sources}
        report.update(passed=True,expired_goal_inhibited=True,no_flight_inputs=True,
            paths=len([p for _,p in paths if p.poses]),tagged_commands=len(commands))
    except Exception as exc:
        report['error']=repr(exc)
    finally:
        if bridge:bridge.destroy_node()
        if fixture:fixture.destroy_node()
        if executor:executor.shutdown()
        rclpy.try_shutdown()
        for p in processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
        for log in logs:log.close()
        report.update(elapsed_s=time.monotonic()-start,children_exited=all(p.poll() is not None for p in processes))
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(evidence=str(out),**report),indent=2))
    return report['passed']


if __name__=='__main__':raise SystemExit(0 if run() else 1)
