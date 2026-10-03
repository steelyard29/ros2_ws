#!/usr/bin/env python3
"""REAL sensors/GPU map/A*+APF candidates ONLY, isolated domain176, no PX4.

Ground position is sampled from VIO before initializing the 0.86m cruise band.
H may be cropped at ground level; this is not a full-H or landing acceptance.
"""
import os
os.environ.update(ROS_DOMAIN_ID='176',ROS_LOCALHOST_ONLY='1')
os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE',None)
import argparse
import importlib.util
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
import yaml
import cv2
import rclpy
from rclpy.qos import qos_profile_sensor_data as qos
from sensor_msgs.msg import Image,Imu,LaserScan,CompressedImage,CameraInfo
from nav_msgs.msg import Odometry,Path as RosPath
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from nvblox_msgs.msg import DistanceMapSlice
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
# Legacy loader resolves uav_task before applying its explicit config override.
# Host symlink-install index points outside the container; use the existing
# actual build index, as in the previous isolated APF comparisons.
os.environ['AMENT_PREFIX_PATH']=str(ROOT.parent/'build/uav_task/ament_cmake_index')+':'+os.environ.get('AMENT_PREFIX_PATH','')
from task_map_profile import mapper_profile,task_band,MapBandEvidence
from bench_session_deadline import DeadlineAlarm
from nav_math import yaw
from stopping_space import stopping_space_clear


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--active-end',type=float,required=True)
    parser.add_argument('--out',type=Path,required=True);args=parser.parse_args()
    if not 5<args.active_end-time.monotonic()<=70:parser.error('expired or invalid session deadline')
    out=args.out
    if not out.is_dir() or out.parent!=ROOT/'evidence':parser.error('invalid evidence directory')
    rclpy.init();node=rclpy.create_node('task_sensor_probe')
    processes=[];logs=[];records={};last={};positions=[];status=[];paths=[];commands=[];h=[]
    bounds=MapBandEvidence();initial=None;band=None;goal=None
    report=dict(flight_authorized=False,px4_inputs=False,servo_outputs=False,hardware_retry=False,
        domain=176,passed=False,task_flight_ready=False,ground_navigation_enabled=False)
    def receive(key,m):
        now=time.monotonic();ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        age=(node.get_clock().now().nanoseconds-ns)/1e9
        records.setdefault(key,[]).append((now,ns,age));last[key]=m
        if key=='vio' and 0<=age<.25:
            p=m.pose.pose.position
            if m.header.frame_id=='odom' and m.child_frame_id=='base_link':positions.append([p.x,p.y,p.z])
        if key=='status':status.append(int(m.vo_state))
        if key=='map':bounds.slice(m.origin.z,now-age)
        if key=='bounds':bounds.marker(m,now-age)
    for key,topic,kind in (
        ('vio','/visual_slam/tracking/odometry',Odometry),('status','/visual_slam/status',VisualSlamStatus),
        ('depth','/camera/camera/depth/image_rect_raw',Image),('imu','/camera/camera/imu',Imu),
        ('scan','/scan',LaserScan),('body_scan','/robocup/legacy_apf/scan',LaserScan),
        ('map','/nvblox_node/static_map_slice',DistanceMapSlice),('bounds','/nvblox_node/esdf_slice_bounds',Marker),
        ('downward','/robocup/downward/image_raw/compressed',CompressedImage),
        ('camera_info','/robocup/downward/camera_info',CameraInfo)):
        node.create_subscription(kind,topic,lambda m,k=key:receive(k,m),
            10 if key in ('map','bounds') else qos)
    node.create_subscription(RosPath,'/robocup/legacy_apf/path',lambda m:paths.append(m),10)
    node.create_subscription(String,'/robocup/task/navigation_command',lambda m:commands.append(json.loads(m.data)),10)
    node.create_subscription(String,'/robocup/landing/h_candidate',lambda m:h.append(json.loads(m.data)),10)
    pub=node.create_publisher(String,'/robocup/task/navigation_goal',10)
    def publish_goal():
        if goal:pub.publish(String(data=json.dumps(goal)))
    node.create_timer(.1,publish_goal)
    def launch(name,cmd):
        if time.monotonic()>args.active_end-5:raise TimeoutError('no startup time remaining')
        f=(out/(name+'.log')).open('w');logs.append(f)
        processes.append(subprocess.Popen(cmd,stdout=f,stderr=f,start_new_session=True))
    def roslaunch(name,**values):
        return ['ros2','launch',str(ROOT/'launch'/name)]+[k+':='+str(v) for k,v in values.items()]
    def graph():
        names=node.get_topic_names_and_types()
        bad=[n for n,_ in names if n.startswith('/fmu/') or n.startswith('/servo/')]
        if bad:raise RuntimeError('unexpected flight/servo graph: '+str(bad))
        return names
    alarm=DeadlineAlarm(args.active_end)
    try:
        alarm.arm()
        # Discovery before starting any sensor; never reuse an unknown session.
        until=time.monotonic()+.5
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
        graph()
        for t in ('/visual_slam/tracking/odometry','/scan','/nvblox_node/static_map_slice','/robocup/downward/image_raw'):
            if node.count_publishers(t):raise RuntimeError('existing sensor publisher: '+t)
        launch('perception',roslaunch('perception.launch.py',nvblox_depth='true',imu_fusion='false',publish_map_tf='false'))
        launch('downward',roslaunch('downward_camera_preview.launch.py',h_transport='compressed'))
        launch('lidar',[str(ROOT.parent/'build/rplidar_ros/rplidar_node'),'--ros-args',
            '-p','channel_type:=serial','-p','serial_port:=/dev/rplidar','-p','serial_baudrate:=115200',
            '-p','frame_id:=laser','-p','inverted:=false','-p','angle_compensate:=true'])
        # Same mount as operator-retained sensors_extra configuration, not remeasured.
        launch('lidar_tf',['ros2','run','tf2_ros','static_transform_publisher','--x','0','--y','0',
            '--z','0.1','--roll','0','--pitch','0','--yaw','0','--frame-id','base_link','--child-frame-id','laser'])
        launch('scan_bridge',['python3',str(ROOT/'scripts/task_scan_bridge.py')])
        wait_until=min(args.active_end-20,time.monotonic()+28)
        while len(positions)<20 or len(status)<5 or status[-1]!=1:
            if time.monotonic()>wait_until:raise TimeoutError('no healthy ground VIO reference within startup window')
            rclpy.spin_once(node,timeout_sec=.02)
        xyz=np.asarray(positions[-20:]);initial=np.median(xyz,axis=0)
        if np.linalg.norm(xyz-initial,axis=1).max()>.05:raise RuntimeError('ground reference not stable')
        m=last['vio'];q=m.pose.pose.orientation;heading=yaw((q.x,q.y,q.z,q.w))
        profile=mapper_profile(float(initial[2]));band=task_band(float(initial[2]))
        report.update(initial_position=initial.tolist(),initial_heading=heading,mapper_profile=profile)
        launch('nvblox',roslaunch('nvblox.launch.py',**profile))
        spec=importlib.util.spec_from_file_location('task_launch',ROOT/'launch/task_navigation.launch.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        params=out/'planner.yaml';params.write_text(yaml.safe_dump(module.planner_parameters()))
        launch('planner',['ros2','run','nav2_planner','planner_server','--ros-args','--params-file',str(params)])
        launch('lifecycle',['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args',
            '-r','__node:=sensor_task_lifecycle','-p','autostart:=true','-p','node_names:=[planner_server]'])
        launch('guard',['python3',str(ROOT/'scripts/nvblox_guard.py'),'--ros-args','-p','center_z:='+str(profile['center_z'])])
        launch('goal_bridge',['python3',str(ROOT/'scripts/navigation_goal_bridge.py')])
        launch('apf',[str(ROOT/'host_task_build/apf/legacy_apf_shadow'),'--ros-args',
            '-p','real_sensor_inputs:=true','-p','path_guided:=true','-p','arrival_tolerance:=0.05',
            '-p','controller_config_file:='+str(ROOT/'config/legacy_apf_shadow.yaml')])
        goal=dict(frame_id='odom',goal_id='sensor-only:OUTBOUND',position=[float(initial[0]+2.5*math.cos(heading)),
            float(initial[1]+2.5*math.sin(heading)),band.center],yaw=heading)
        report['candidate_goal']=goal;next_graph=0
        while time.monotonic()<args.active_end-.5:
            rclpy.spin_once(node,timeout_sec=.02)
            if time.monotonic()>next_graph:
                graph();next_graph=time.monotonic()+1
                if any(p.poll() is not None for p in processes):raise RuntimeError('owned child exited; inspect logs; no restart')
        report['graph']=graph()
    except Exception as exc:report['error']=repr(exc)
    finally:
        alarm.cancel();stopped=time.monotonic()
        report['child_exit_codes_before_stop']=[p.poll() for p in processes]
        report['map_publishers']=[dict(name=e.node_name,namespace=e.node_namespace,gid=list(e.endpoint_gid))
            for e in node.get_publishers_info_by_topic('/nvblox_node/static_map_slice')]
        report['bounds_publishers']=[dict(name=e.node_name,namespace=e.node_namespace,gid=list(e.endpoint_gid))
            for e in node.get_publishers_info_by_topic('/nvblox_node/esdf_slice_bounds')]
        for p in processes:
            if p.poll() is None:
                try:os.killpg(p.pid,signal.SIGINT)
                except ProcessLookupError:pass
        # Parallel group shutdown, NOT N children times an 8-second wait.
        for sig,until in ((None,stopped+5),(signal.SIGTERM,stopped+7),(signal.SIGKILL,stopped+8)):
            if sig:
                for p in processes:
                    try:os.killpg(p.pid,sig)
                    except ProcessLookupError:pass
            while time.monotonic()<until and any(p.poll() is None for p in processes):time.sleep(.03)
        report['children_exited']=all(p.poll() is not None for p in processes)
        report['child_exit_codes']=[p.poll() for p in processes]
        for f in logs:f.close()
        report['streams']={}
        for key,rows in records.items():
            a=np.asarray(rows);span=a[-1,0]-a[0,0]
            valid_ages=[row[2] for row in rows if row[1]>0]
            report['streams'][key]=dict(count=len(rows),hz=(len(rows)-1)/span if span>0 else 0,
                max_receive_gap_s=float(np.diff(a[:,0]).max()) if len(rows)>1 else None,
                zero_stamp_count=sum(row[1]==0 for row in rows),
                age_p50_p95_max_s=np.quantile(valid_ages,[.5,.95,1]).tolist() if valid_ages else None)
        report.update(vo_states=sorted(set(status)),nonempty_paths=sum(bool(p.poses) for p in paths),
            linked_commands=len(commands),h_messages=len(h),h_stable=sum(bool(x.get('candidate_stable')) for x in h),
            last_h=h[-1] if h else None)
        report['map_height_consistent']=bool(band and bounds.check(band,stopped))
        report['map_height_reason']=bounds.reason
        if positions:report['static_vio_max_displacement_m']=float(np.linalg.norm(np.asarray(positions)-positions[0],axis=1).max())
        if 'map' in last and initial is not None:
            m=last['map'];a=np.asarray(m.data).reshape(m.height,m.width)
            free=np.isfinite(a)&(a!=m.unknown_value)&(a>0)
            report['known_map_cells']=int(np.sum(np.isfinite(a)&(a!=m.unknown_value)))
            report['obstacle_map_cells']=int(np.sum(np.isfinite(a)&(a!=m.unknown_value)&(a<=0)))
            report['start_stopping_space_clear']=stopping_space_clear(free,m.resolution,(m.origin.x,m.origin.y),
                initial[:2],(0.,0.),(0.,0.),body_radius=.43,uncertainty=.05,latency=.3,braking=.2)
            np.savez_compressed(out/'esdf_slice.npz',data=a,width=m.width,height=m.height,resolution=m.resolution,
                unknown=m.unknown_value,origin=[m.origin.x,m.origin.y,m.origin.z])
        if 'downward' in last:
            im=cv2.imdecode(np.frombuffer(last['downward'].data,dtype=np.uint8),cv2.IMREAD_COLOR)
            if im is not None:cv2.imwrite(str(out/'downward.png'),im)
        report['passed']=bool('error' not in report and report['children_exited'] and
            all(len(records.get(k,[]))>5 for k in ('vio','depth','scan','body_scan','map','bounds','downward')))
        report['pass_scope']='sensor/map input collection only; navigation and H/landing readiness assessed separately'
        (out/'stream_timing.json').write_text(json.dumps(records))
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        node.destroy_node();rclpy.try_shutdown();print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
