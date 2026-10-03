#!/usr/bin/env python3
"""Twelve-second passive observer of ALREADY running perception, domain176.

Never starts/stops drivers, publishes sensor/control data, or requires a ground H.
"""
import os
os.environ.update(ROS_DOMAIN_ID='176',ROS_LOCALHOST_ONLY='1')
import json
import argparse
from pathlib import Path
import time
import numpy as np
import yaml
import rclpy
from rclpy.qos import qos_profile_sensor_data as qos
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image,Imu,CompressedImage,CameraInfo,LaserScan
from std_msgs.msg import String
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from nvblox_msgs.msg import DistanceMapSlice
from visualization_msgs.msg import Marker
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from task_map_profile import MapBandEvidence,task_band
from perception_acceptance import AcceptanceWindow


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--navigation',action='store_true')
    parser.add_argument('--initial-z',type=float)
    parser.add_argument('--qualified-window',action='store_true',
                        help='bounded startup, then a complete 12s one-shot acceptance window')
    args=parser.parse_args()
    if args.navigation and (args.initial_z is None or not np.isfinite(args.initial_z)):
        parser.error('navigation observation requires explicit ground reference')
    out=ROOT/'evidence'/time.strftime('perception_recovery_%Y%m%d_%H%M%S')
    out.mkdir(parents=True,exist_ok=False)
    rclpy.init(args=[])
    node=rclpy.create_node('perception_recovery_observer',enable_rosout=False,start_parameter_services=False)
    rows={};last={};states=[];h=[];positions=[];band_evidence=MapBandEvidence()
    started=time.monotonic()
    gate=AcceptanceWindow(started,navigation=args.navigation) if args.qualified_window else None
    cfg=yaml.safe_load((ROOT/'config/downward_camera_info.yaml').read_text())
    from tf2_ros import Buffer, TransformListener
    tf_buffer=Buffer();tf_listener=TransformListener(tf_buffer,node)
    topics=dict(vio='/visual_slam/tracking/odometry',depth='/camera/camera/depth/image_rect_raw',
        imu='/camera/camera/imu',status='/visual_slam/status',
        downward='/robocup/downward/image_raw/compressed',camera_info='/robocup/downward/camera_info')
    kinds=[('vio',Odometry),('depth',Image),('imu',Imu),('status',VisualSlamStatus),
           ('downward',CompressedImage),('camera_info',CameraInfo)]
    if args.navigation:
        topics.update(scan='/scan',body_scan='/robocup/legacy_apf/scan',
            map='/nvblox_node/static_map_slice',bounds='/nvblox_node/esdf_slice_bounds',
            yaw_odom='/robocup/odom')
        kinds += [('scan',LaserScan),('body_scan',LaserScan),('map',DistanceMapSlice),
                  ('bounds',Marker),('yaw_odom',Odometry)]
    def receive(k,m):
        now=time.monotonic();ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        rows.setdefault(k,[]).append((now,ns,(node.get_clock().now().nanoseconds-ns)/1e9))
        last[k]=m
        if gate:
            gate.sample(k,now,ns,rows[k][-1][2])
            if gate.phase=='MEASURE' and ((k=='status' and m.vo_state!=1) or
                    (k=='vio' and (m.header.frame_id!='odom' or m.child_frame_id!='base_link'))):
                gate.reject(now,'tracking or VIO frame invalid')
            if k=='vio':
                p=m.pose.pose.position;q=m.pose.pose.orientation
                if not np.isfinite([p.x,p.y,p.z,q.x,q.y,q.z,q.w]).all():
                    gate.reject(now,'nonfinite VIO pose')
        if (k=='vio' and m.header.frame_id=='odom' and m.child_frame_id=='base_link'
                and 0<=rows[k][-1][2]<=.25):
            p=m.pose.pose.position;positions.append([p.x,p.y,p.z])
        if k=='status':states.append(int(m.vo_state))
        if k=='map':band_evidence.slice(m.origin.z,now-rows[k][-1][2])
        if k=='bounds':band_evidence.marker(m,now-rows[k][-1][2])
    for key,kind in kinds:
        node.create_subscription(kind,topics[key],lambda m,k=key:receive(k,m),
                                 10 if key in ('map','bounds','yaw_odom') else qos)
    node.create_subscription(String,'/robocup/landing/h_candidate',lambda m:h.append(json.loads(m.data)),10)
    report=dict(passed=False,flight_ready=False,domain=176,passive_only=True,
        hardware_state='operator_reports_props_installed_disarmed_unloaded',full_h_required=False)
    source_identity=None
    def prerequisites():
        nonlocal source_identity
        required=['vio','depth','downward','status','camera_info']
        if args.navigation:required+=['map','bounds','scan','body_scan','yaw_odom']
        groups={k:node.get_publishers_info_by_topic(topics[k]) for k in required}
        if any(len(g)!=1 for g in groups.values()):return False
        identities={k:list(g[0].endpoint_gid) for k,g in groups.items()}
        if gate.phase=='MEASURE' and identities!=source_identity:return False
        source_identity=identities
        if any(n.startswith('/fmu/') for n,_ in node.get_topic_names_and_types()):return False
        now=time.monotonic()
        if any(len(rows.get(k,[]))<=5 or now-rows[k][-1][0]>.5 for k in required):return False
        if int(last['status'].vo_state)!=1:return False
        if last['vio'].header.frame_id!='odom' or last['vio'].child_frame_id!='base_link':return False
        p=last['vio'].pose.pose.position;q=last['vio'].pose.pose.orientation
        if not np.isfinite([p.x,p.y,p.z,q.x,q.y,q.z,q.w]).all():return False
        if not np.allclose(last['camera_info'].k,cfg['camera_matrix']['data'],rtol=0,atol=1e-9):return False
        try:
            tr=tf_buffer.lookup_transform('base_link','camera_link',rclpy.time.Time()).transform
            actual=[tr.translation.x,tr.translation.y,tr.translation.z,
                    tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]
            if not np.allclose(actual,[.196,.025,-.05,0,0,0,1],rtol=0,atol=1e-6):return False
        except Exception:return False
        if args.navigation:
            if not band_evidence.check(task_band(args.initial_z),now):return False
            owners=[(groups[k][0].node_namespace,groups[k][0].node_name) for k in ('map','bounds')]
            if owners[0]!=owners[1]:return False
            if node.count_publishers('/robocup/task/navigation_goal')!=0:return False
            if node.count_publishers('/robocup/task/navigation_command')!=1:return False
        return True
    try:
        next_check=0.;ready=False
        while time.monotonic()-started<(31 if gate else 12):
            rclpy.spin_once(node,timeout_sec=.02)
            if gate:
                now=time.monotonic()
                if now>=next_check:
                    ready=prerequisites();next_check=now+.1
                if gate.tick(time.monotonic(),ready) in ('PASSED','FAILED'):break
        # Raw startup records stay on disk; only a prospective gate qualifies.
        measured=rows if not gate or gate.measure_started is None else {
            k:[r for r in rr if r[0]>=gate.measure_started] for k,rr in rows.items()}
        report['fmu_topics']=[n for n,_ in node.get_topic_names_and_types() if n.startswith('/fmu/')]
        report['publishers']={k:[e.node_namespace+'/'+e.node_name
            for e in node.get_publishers_info_by_topic(t)] for k,t in topics.items()}
        report['streams']={}
        for k,rr in measured.items():
            if not rr:continue
            a=np.asarray(rr);span=a[-1,0]-a[0,0]
            report['streams'][k]=dict(count=len(rr),hz=(len(rr)-1)/span if span>0 else 0.,
                max_gap_s=float(np.diff(a[:,0]).max()) if len(rr)>1 else None,
                max_source_gap_s=max(((b[1]-a[1])/1e9 for a,b in zip(rr,rr[1:])),default=None),
                max_source_age_s=float(a[:,2].max()),zero_stamps=int(np.sum(a[:,1]==0)))
        report.update(vo_states=sorted(set(states)),h_messages=len(h),
                      stable_h=sum(bool(d.get('candidate_stable')) for d in h))
        if 'vio' in last:
            p=last['vio'].pose.pose.position
            report['last_vio_position']=[p.x,p.y,p.z]
        if len(positions)>=20:
            window=np.asarray(positions[-60:]);reference=np.median(window,axis=0)
            report['ground_reference']=dict(position=reference.tolist(),samples=len(window),
                max_displacement_m=float(np.linalg.norm(window-reference,axis=1).max()),
                scope='stationary operator-confirmed ground sample, not absolute accuracy')
        try:
            tr=tf_buffer.lookup_transform('base_link','camera_link',rclpy.time.Time()).transform
            report['camera_tf']=[tr.translation.x,tr.translation.y,tr.translation.z,
                                  tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]
        except Exception as exc:report['camera_tf_error']=str(exc)
        ci=last.get('camera_info')
        report['authorized_intrinsics_loaded']=bool(ci is not None and
            np.allclose(ci.k,cfg['camera_matrix']['data'],rtol=0,atol=1e-9))
        report['passed']=bool(not report['fmu_topics'] and
            all(len(rows.get(k,[]))>5 and len(report['publishers'][k])==1
                for k in ('vio','depth','downward','status','camera_info'))
            and (gate is not None or report['vo_states']==[1]) and report['authorized_intrinsics_loaded'])
        report['pass_scope']='stream recovery/ownership/intrinsics only; not flight, mapping or H landing acceptance'
        if args.navigation:
            report['map_height_consistent']=band_evidence.check(task_band(args.initial_z),time.monotonic())
            report['map_height_reason']=band_evidence.reason
            groups=[node.get_publishers_info_by_topic(topics[k]) for k in ('map','bounds')]
            report['one_mapper_identity']=bool(all(len(g)==1 for g in groups) and
                (groups[0][0].node_name,groups[0][0].node_namespace)==
                (groups[1][0].node_name,groups[1][0].node_namespace))
            report['navigation_goal_publishers']=node.count_publishers('/robocup/task/navigation_goal')
            report['navigation_command_publishers']=node.count_publishers('/robocup/task/navigation_command')
            if 'map' in last:
                m=last['map'];data=np.asarray(m.data)
                report['map_center_z']=m.origin.z
                report['known_cells']=int(np.sum(np.isfinite(data)&(data!=m.unknown_value)))
                report['obstacle_cells']=int(np.sum(np.isfinite(data)&(data!=m.unknown_value)&(data<=0)))
            report['passed']=bool(report['passed'] and report['map_height_consistent'] and
                report['one_mapper_identity'] and report['navigation_command_publishers']==1 and
                all(len(rows.get(k,[]))>5 and len(report['publishers'][k])==1
                    for k in ('map','bounds','scan','body_scan','yaw_odom')))
            report['pass_scope']='real sensor/map/candidate chain input delivery and height; no requested route, PX4 output or flight acceptance'
        if gate:
            report['acceptance_window']=gate.report()
            report['startup_counts']={k:sum(r[0]<(gate.measure_started or float('inf')) for r in rr)
                                      for k,rr in rows.items()}
            report['qualified_source_identity']=source_identity
            report['prerequisite_poll_period_s']=.1
            report['other_required_stream_receipt_limit_s']=.5
            report['passed']=bool(report['passed'] and gate.phase=='PASSED')
            report['pass_scope']+='; prospective 12s qualified window, startup evidence retained'
    except Exception as exc:
        report['error']=repr(exc);report['passed']=False
    finally:
        report['elapsed_s']=time.monotonic()-started
        node.destroy_node();rclpy.try_shutdown()
        # Preserve event times for later startup-vs-steady-state attribution;
        # aggregate maxima alone cannot establish when a delay happened.
        (out/'stream_timing.json').write_text(json.dumps(rows)+'\n')
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(evidence=str(out),**report),indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
