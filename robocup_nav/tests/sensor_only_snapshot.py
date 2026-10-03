"""One bounded, operator-authorized sensor capture. NO PX4/servo/control nodes.

Run only inside the existing container after device ownership inspection.
Domain179 is isolated from flight. Does not retry failed hardware starts.
Reports stream evidence, not safe-to-fly or H landing readiness.
"""
import os
os.environ['ROS_DOMAIN_ID']='179'
os.environ['ROS_LOCALHOST_ONLY']='1'
import json
import argparse
import math
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import cv2
import yaml
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,Imu,LaserScan,CameraInfo,CompressedImage
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from cv_bridge import CvBridge

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--transport',choices=('raw','compressed'),default='raw')
    args=parser.parse_args()
    out=ROOT/'evidence'/time.strftime('sensor_only_%Y%m%d_%H%M%S')
    out.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=rclpy.create_node('sensor_only_snapshot')
    records={};last_messages={};vo_states={};h_status=[];positions=[]
    def receive(key,msg):
        now=time.monotonic();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        age=node.get_clock().now().nanoseconds/1e9-stamp
        records.setdefault(key,[]).append((now,stamp,age));last_messages[key]=msg
        if key=='vio':
            p=msg.pose.pose.position;positions.append([p.x,p.y,p.z])
        if key=='status':vo_states[int(msg.vo_state)]=vo_states.get(int(msg.vo_state),0)+1
    for key,topic,kind in (
        ('left','/camera/camera/infra1/image_rect_raw',Image),
        ('depth','/camera/camera/depth/image_rect_raw',Image),
        ('imu','/camera/camera/imu',Imu),
        ('vio','/visual_slam/tracking/odometry',Odometry),
        ('status','/visual_slam/status',VisualSlamStatus),
        ('scan','/scan',LaserScan),
        ('downward','/robocup/downward/image_raw'+('/compressed' if args.transport=='compressed' else ''),
         CompressedImage if args.transport=='compressed' else Image),
        ('camera_info','/robocup/downward/camera_info',CameraInfo)):
        node.create_subscription(kind,topic,lambda msg,k=key:receive(k,msg),qos_profile_sensor_data)
    node.create_subscription(String,'/robocup/landing/h_candidate',lambda m:h_status.append(json.loads(m.data)),10)
    commands=[
        ('perception',['ros2','launch',str(ROOT/'launch/perception.launch.py'),
                       'nvblox_depth:=true','imu_fusion:=false','publish_map_tf:=false']),
        ('downward',['ros2','launch',str(ROOT/'launch/downward_camera_preview.launch.py'),
                     'h_transport:='+args.transport]),
        ('lidar',[str(ROOT.parent/'build/rplidar_ros/rplidar_node'),'--ros-args',
                  '-p','channel_type:=serial','-p','serial_port:=/dev/rplidar',
                  '-p','serial_baudrate:=115200','-p','frame_id:=laser',
                  '-p','inverted:=false','-p','angle_compensate:=true'])]
    processes=[];logs=[];graph_violations=set();graph=[]
    report=dict(flight_authorized=False,px4_input_started=False,servos_started=False,
                domain=179,duration_s=60,hardware_start_retry=False,passed=False,transport=args.transport)
    try:
        for name,command in commands:
            log=open(out/(name+'.log'),'w');logs.append(log)
            processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        capture_start=time.monotonic();until=capture_start+60;next_graph=0.
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.02)
            if time.monotonic()>=next_graph:
                graph=node.get_topic_names_and_types();next_graph=time.monotonic()+1
                graph_violations.update(n for n,_ in graph if n.startswith('/fmu/in/') or n.startswith('/servo/'))
                if graph_violations:raise RuntimeError('unexpected flight/servo topics in isolated domain')
        streams={}
        for key,a in records.items():
            stamps=np.array(a);span=stamps[-1,0]-stamps[0,0]
            streams[key]=dict(count=len(a),receive_hz=(len(a)-1)/span if span>0 else 0,
                max_receive_gap_s=float(np.diff(stamps[:,0]).max()) if len(a)>1 else None,
                age_min_p50_p95_max_s=np.quantile(stamps[:,2],[0,.5,.95,1]).tolist(),
                nonmonotonic_stamps=int(np.sum(np.diff(stamps[:,1])<=0)),
                frame_id=last_messages[key].header.frame_id)
        report.update(streams=streams,vo_states=vo_states,h_status_count=len(h_status),
            h_stable_count=sum(bool(x.get('candidate_stable')) for x in h_status),
            last_h_status=h_status[-1] if h_status else None,graph=graph)
        # Keep actual timestamps for independent startup vs steady-state review.
        (out/'stream_timing.json').write_text(json.dumps(records))
        steady={}
        for key,rows in records.items():
            a=np.array([row for row in rows if row[0]>=capture_start+10])
            if len(a)>1:
                steady[key]=dict(count=len(a),max_receive_gap_s=float(np.diff(a[:,0]).max()),
                    age_min_p50_p95_max_s=np.quantile(a[:,2],[0,.5,.95,1]).tolist())
        report['steady_after_10s']=steady
        if positions:
            p=np.array(positions)
            report['static_vio_max_displacement_m']=float(np.linalg.norm(p-p[0],axis=1).max())
            report['initial_vio_position']=p[0].tolist();report['final_vio_position']=p[-1].tolist()
            (out/'vio_positions.json').write_text(json.dumps(positions))
        scan=last_messages.get('scan')
        if scan:
            a=np.array(scan.ranges);valid=np.isfinite(a)&(a>=scan.range_min)&(a<=scan.range_max)
            report['scan']=dict(beams=len(a),valid_fraction=float(valid.mean()),
                angle_min=scan.angle_min,angle_max=scan.angle_max,angle_increment=scan.angle_increment,
                range_min=scan.range_min,range_max=scan.range_max,
                nearest_valid_m=float(a[valid].min()) if valid.any() else None,
                physical_extrinsics_verified=False)
            (out/'scan.json').write_text(json.dumps(dict(ranges=[float(v) if math.isfinite(v) else None for v in a],
                    angle_min=scan.angle_min,angle_increment=scan.angle_increment)))
        image=last_messages.get('downward');info=last_messages.get('camera_info')
        cal=yaml.safe_load((ROOT/'config/downward_camera_info.yaml').read_text())
        decoded=(CvBridge().compressed_imgmsg_to_cv2(image,'bgr8') if args.transport=='compressed'
                 else CvBridge().imgmsg_to_cv2(image,'bgr8')) if image else None
        matching=bool(decoded is not None and info and decoded.shape[1]==info.width==cal['image_width']
            and decoded.shape[0]==info.height==cal['image_height']
            and np.allclose(info.k,cal['camera_matrix']['data'])
            and len(info.d)==len(cal['distortion_coefficients']['data'])
            and np.allclose(info.d,cal['distortion_coefficients']['data']))
        report['downward_calibration_matches']=matching
        if image:
            if not cv2.imwrite(str(out/'downward.png'),decoded):
                raise RuntimeError('could not save downward image')
        required=('left','depth','vio','status','scan','downward','camera_info')
        report['all_required_streams_received']=all(len(records.get(k,[]))>5 for k in required)
        report['passed']=report['all_required_streams_received'] and matching and set(vo_states)=={1} and not graph_violations
        report['pass_scope']='Basic sensor stream availability and calibration loading only; not motion, metric H, coverage, or timing acceptance'
    except Exception as exc:report['error']=str(exc)
    finally:
        report['forbidden_topics']=sorted(graph_violations)
        report['process_status_before_stop']=[p.poll() for p in processes]
        for p in processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
        for log in logs:log.close()
        node.destroy_node();rclpy.try_shutdown()
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(out);print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
