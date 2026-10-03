"""One <=165s domain186 actual NavFn A*/legacy APF task replay. No PX4/driver.

Synthetic 1.2m two-box gap and current site dimensions. Ideal dynamics/H/LAND;
not SITL, real sensors, new host pipe validation, or flight permission.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))


def launch_child_failure(text):
    """A live launch supervisor may outlive a failed required child."""
    for line in text.splitlines():
        if 'process has died' in line:
            return line.strip()
    return None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-isolated',action='store_true')
    parser.add_argument('--apf',type=Path)
    a=parser.parse_args()
    if not a.run_isolated:
        print('Describe only: explicit domain186 synthetic A*/APF task, no real outputs.');return 0
    if (os.environ.get('ROS_DOMAIN_ID')!='186' or os.environ.get('ROS_LOCALHOST_ONLY')!='1'
            or a.apf is None or not a.apf.is_file()):
        parser.error('domain186, ROS_LOCALHOST_ONLY=1 and existing dedicated APF binary required')
    start=time.monotonic()
    import numpy as np
    import yaml
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry,Path as NavPath
    from sensor_msgs.msg import LaserScan,Image
    from std_msgs.msg import String
    from nvblox_msgs.msg import DistanceMapSlice
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
    from test_task_handoff import TaskRig
    from roundtrip_flight_test import RoundTripMission
    from task_site_bounds import resolve_bounds,inside_bounds
    from stopping_space import stopping_space_clear,select_stopping_safe_velocity
    out=ROOT/'evidence'/time.strftime('task_actual_navigation_%Y%m%d_%H%M%S');out.mkdir()
    report=dict(passed=False,hardware=False,px4_sitl=False,flight_ready=False,domain=186,
                actual_astar_apf=True,host_pipe_used=False,assumptions=['ideal dynamics',
                'synthetic binary slice and laser','synthetic H error','ideal LAND/ACK'])
    proc=node=None;initialized=False
    log=(out/'navigation.log').open('w')
    def interrupted(*_):raise KeyboardInterrupt()
    previous=signal.signal(signal.SIGTERM,interrupted)
    try:
        rclpy.init(domain_id=186);initialized=True
        cfg=yaml.safe_load((ROOT/'config/roundtrip_task.yaml').read_text());rig=TaskRig(hover=3.)
        report['hover_s']=rig.c.hover
        resolution=.05;origin=(-1.8,-2.5);free=np.ones((100,160),dtype=bool)
        free[0,:]=free[-1,:]=free[:,0]=free[:,-1]=False
        # x 1.2..2.0m; y -1.4..-.6m and .6..1.4m. Inner gap 1.2m.
        free[22:38,60:76]=False;free[62:78,60:76]=False
        bounds=resolve_bounds(cfg,(0.,0.),0.)
        def clear(p,v,c):
            speed=max(math.hypot(*v),math.hypot(*c));model=cfg['stopping']
            return (inside_bounds(bounds,p,speed*model['latency']+speed*speed/(2*model['braking']))
                and stopping_space_clear(free,resolution,origin,p,v,c,**model))
        rig.m=RoundTripMission(rig.m.band,bounds,clear,ground_origin=(0,0,0),yaw_odom=0.,session_id='unit-task')
        node=Node('isolated_task_navigation_fixture',enable_rosout=False)
        od=node.create_publisher(Odometry,'/visual_slam/tracking/odometry',10)
        depth=node.create_publisher(Image,'/camera/camera/depth/image_rect_raw',10)
        status=node.create_publisher(VisualSlamStatus,'/visual_slam/status',10)
        scan=node.create_publisher(LaserScan,'/scan',10)
        maps=node.create_publisher(DistanceMapSlice,'/nvblox_node/static_map_slice',10)
        goal=node.create_publisher(String,'/robocup/task/navigation_goal',10)
        state=dict(command=None,commands=0,paths=0,path_points=0,contact=False,max_forward=0.,max_height=.14)
        samples=[]
        def command(m):
            state['command']=json.loads(m.data);state['commands']+=1
        def path(m):
            if m.poses:state['paths']+=1;state['path_points']=max(state['path_points'],len(m.poses))
        node.create_subscription(String,'/robocup/task/navigation_command',command,10)
        node.create_subscription(NavPath,'/robocup/legacy_apf/path',path,10)
        angles=np.arange(720)*2*math.pi/720-math.pi
        distances=np.arange(.1,10.,.025)
        ray_x=np.cos(angles)[:,None]*distances;ray_y=np.sin(angles)[:,None]*distances
        grid=DistanceMapSlice();grid.header.frame_id='odom';grid.resolution=resolution
        grid.width=160;grid.height=100;grid.origin.x,grid.origin.y=origin;grid.origin.z=.86
        grid.unknown_value=-1000.;grid.data=np.where(free,2.,-.01).ravel().tolist()
        def publish():
            stamp=node.get_clock().now().to_msg()
            m=Odometry();m.header.stamp=stamp;m.header.frame_id='odom';m.child_frame_id='base_link'
            m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,rig.p)
            m.pose.pose.orientation.w=1.;m.twist.twist.linear.x,m.twist.twist.linear.y=map(float,rig.v[:2]);od.publish(m)
            d=Image();d.header.stamp=stamp;d.header.frame_id='camera_depth_optical_frame'
            d.width=d.height=1;d.step=2;d.encoding='16UC1';d.data=[208,7];depth.publish(d)
            s=VisualSlamStatus();s.header.stamp=stamp;s.vo_state=1;status.publish(s)
            grid.header.stamp=stamp;maps.publish(grid)
            ix=np.floor((rig.p[0]+ray_x-origin[0])/resolution).astype(int)
            iy=np.floor((rig.p[1]+ray_y-origin[1])/resolution).astype(int)
            valid=(ix>=0)&(ix<160)&(iy>=0)&(iy<100)
            hit=~(valid&free[np.clip(iy,0,99),np.clip(ix,0,159)])
            laser=LaserScan();laser.header.stamp=stamp;laser.header.frame_id='base_link'
            laser.angle_min=-math.pi;laser.angle_increment=2*math.pi/720
            laser.angle_max=laser.angle_min+719*laser.angle_increment;laser.range_min=.1;laser.range_max=12.
            laser.ranges=distances[np.argmax(hit,axis=1)].astype(float).tolist();scan.publish(laser)
            goal.publish(String(data=json.dumps(rig.m.planner_goal())))
        node.create_timer(.05,publish)
        env=dict(os.environ,AMENT_PREFIX_PATH=str(ROOT.parent/'build/uav_task/ament_cmake_index')+':'+os.environ.get('AMENT_PREFIX_PATH',''))
        proc=subprocess.Popen(['ros2','launch',str(ROOT/'launch/task_navigation.launch.py'),'center_z:=0.86',
            'isolated_task_test:=true','apf_executable:='+str(a.apf)],env=env,stdout=log,stderr=log,start_new_session=True)
        active=None;next_step=0.;next_log_check=0.
        while time.monotonic()<start+150:
            rclpy.spin_once(node,timeout_sec=.005);now=time.monotonic()
            if proc.poll() is not None:raise RuntimeError('navigation launcher exited')
            if now>=next_log_check:
                next_log_check=now+.25
                failure=launch_child_failure((out/'navigation.log').read_text(errors='replace'))
                if failure:raise RuntimeError('required navigation child failed: '+failure)
            if any(t.startswith('/fmu/') for t,_ in node.get_topic_names_and_types()):
                raise RuntimeError('unexpected PX4 topic in isolated domain')
            if node.count_publishers('/robocup/task/navigation_goal')>1:raise RuntimeError('competing goal writer')
            msg=state['command'];ns=node.get_clock().now().nanoseconds
            fresh=bool(msg and msg.get('goal_id')==rig.m.goal_id and
                0<=ns-msg['stamp_ns']<=250_000_000 and 0<=ns-msg['path_stamp_ns']<=500_000_000)
            if active is None:
                if fresh and state['paths']>0:active=now;next_step=now
                elif now-start>35:raise RuntimeError('actual planner/APF startup timeout')
                else:continue
            if now<next_step:continue
            next_step=now+.05
            if now-active>110:raise RuntimeError('bounded task did not finish')
            velocity=(0.,0.)
            if fresh:velocity,_=select_stopping_safe_velocity(msg['velocity_xy'],lambda c:clear(rig.p[:2],rig.v[:2],c))
            output,_=rig.tick(scene_changes=dict(navigation_goal_id=msg['goal_id'] if msg else '',
                navigation_velocity_odom=velocity,navigation_at=rig.now+.05 if fresh else -1.,
                footprint_known_free=clear(rig.p[:2],(0.,0.),(0.,0.))))
            state['contact'] |= not stopping_space_clear(free,resolution,origin,rig.p[:2],(0.,0.),(0.,0.),
                body_radius=.43,uncertainty=0.,latency=0.,braking=1.)
            state['max_forward']=max(state['max_forward'],float(rig.p[0]))
            state['max_height']=max(state['max_height'],float(rig.p[2])+.14)
            samples.append(dict(at=now,state=output.state,leg=rig.m.leg,position=rig.p.tolist(),
                                reason=output.reason,fresh_navigation=fresh))
            if output.state in rig.c.TERMINAL:
                report['passed']=bool(output.state=='DONE' and rig.m.leg=='RETURN' and not state['contact']
                    and state['max_forward']>2.4 and not rig.s.armed and rig.s.landed and state['paths']>1)
                break
        report.update(state={k:v for k,v in state.items() if k!='command'},terminal=rig.c.state,
            commands=sorted(rig.commands),simulated_seconds=rig.now,final_position=rig.p.tolist(),
            states=list(dict.fromkeys(x.state for x in rig.trace)),production_axes_reviewed=cfg['camera']['axes_reviewed'])
        (out/'samples.json').write_text(json.dumps(samples))
    except BaseException as exc:
        report['passed']=False
        report['error']=repr(exc)
    finally:
        if 'state' in locals():
            report.update(state={k:v for k,v in state.items() if k!='command'},terminal=rig.c.state,
                reason=rig.c.reason,commands=sorted(rig.commands),simulated_seconds=rig.now,
                final_position=rig.p.tolist())
            (out/'samples.json').write_text(json.dumps(samples))
        if proc:
            for sig,seconds in ((signal.SIGINT,6),(signal.SIGTERM,3),(signal.SIGKILL,1)):
                if proc.poll() is not None:break
                os.killpg(proc.pid,sig)
                try:proc.wait(timeout=seconds)
                except subprocess.TimeoutExpired:continue
            report['launcher_exit']=proc.poll()
        if node:node.destroy_node()
        if initialized:rclpy.try_shutdown()
        signal.signal(signal.SIGTERM,previous);log.close()
        report['elapsed_s']=time.monotonic()-start
        report['apf_sha256']=hashlib.sha256(a.apf.read_bytes()).hexdigest()
        report['source_sha256']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [Path(__file__),ROOT/'launch/task_navigation.launch.py',ROOT/'legacy_apf_shadow/node.cpp',
                      ROOT/'legacy_apf_shadow/domain_policy.hpp',
                      ROOT/'scripts/navigation_goal_bridge.py',ROOT/'config/roundtrip_task.yaml']}
        if report['elapsed_s']>165:report['passed']=False
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(evidence=str(out),**report),indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
