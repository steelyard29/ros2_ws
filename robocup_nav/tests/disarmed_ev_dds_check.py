#!/usr/bin/env python3
"""Isolated domain179 actual ROS node with synthetic PX4 and sensor packets.

No Agent, container, hardware topics, or real perception subscriptions.
"""
import os
os.environ.update(ROS_DOMAIN_ID='179',ROS_LOCALHOST_ONLY='1')
os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE',None)
import json
import argparse
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import rclpy
from rclpy.qos import qos_profile_sensor_data as qos
from px4_msgs.msg import (VehicleStatus,EstimatorStatusFlags,VehicleLandDetected,
                         ManualControlSetpoint,VehicleLocalPosition,VehicleOdometry)
from disarmed_ev_contract import DisarmedEvSession
from disarmed_ev_session import create_node


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--case',choices=['normal','missing','duplicate','mixed',
        'stop-status','stop-flags','stop-land','stop-rc'],default='normal')
    args=parser.parse_args()
    rclpy.init(args=[])
    root=Path(__file__).resolve().parents[1]
    out=root/'evidence'/(time.strftime('disarmed_ev_dds_%Y%m%d_%H%M%S')+'_'+args.case)
    out.mkdir(exist_ok=False)
    params={'RC_MAP_AUX1':6,'RC_MAP_AUX2':5,'RC_MAP_FLTMODE':6,'RC_MAP_KILL_SW':5,
        'RC_MAP_OFFB_SW':0,'RC_MAP_FLTM_BTN':0,'RC_KILLSWITCH_TH':.75,'COM_RC_IN_MODE':0,
        **{f'COM_FLTMODE{i}':2 if i<6 else 7 for i in range(1,7)}}
    start=time.monotonic();s=DisarmedEvSession(start,start+12,'dds-test',params)
    n=create_node(s,isolated=True)
    src=rclpy.create_node('isolated_disarmed_ev_fixture',enable_rosout=False,start_parameter_services=False)
    specs=[('vehicle_status_v1',VehicleStatus),('estimator_status_flags',EstimatorStatusFlags),
        ('vehicle_land_detected',VehicleLandDetected),('manual_control_setpoint',ManualControlSetpoint),
        ('vehicle_local_position',VehicleLocalPosition)]
    pubs={k:src.create_publisher(t,'/robocup/disarmed_ev_test/out/'+k,qos) for k,t in specs}
    if args.case=='missing':
        src.destroy_publisher(pubs['vehicle_local_position']);pubs['vehicle_local_position']=None
    duplicate=(src.create_publisher(VehicleStatus,'/robocup/disarmed_ev_test/out/vehicle_status_v1',qos)
               if args.case=='duplicate' else None)
    rows=[]
    src.create_subscription(VehicleOdometry,'/robocup/disarmed_ev_test/in/vehicle_visual_odometry',
                            lambda m:rows.append(m),qos)
    report=dict(passed=False,hardware=False,domain=179,scope='actual ROS node, synthetic inputs; no reader/Agent/PX4 test')
    seq=0
    mixed = args.case=='mixed' or args.case.startswith('stop-')
    aliases = dict(zip(pubs, ['status','flags','land','rc','local']))
    periods = dict(status=.51, flags=1.01, land=1., rc=.02, local=.02)
    due = dict.fromkeys(pubs, start)
    stopped_topic = args.case[5:] if args.case.startswith('stop-') else None
    def step(armed=False):
        nonlocal seq
        ns=src.get_clock().now().nanoseconds;us=ns//1000
        st=VehicleStatus();st.timestamp=us;st.arming_state=2 if armed else 1;st.nav_state=2
        st.pre_flight_checks_pass=True;st.gcs_connection_lost=False
        fl=EstimatorStatusFlags();fl.timestamp=us;fl.cs_baro_hgt=True
        fl.cs_ev_pos=True;fl.cs_ev_yaw=True;fl.cs_ev_hgt=True
        la=VehicleLandDetected();la.timestamp=us;la.landed=True
        rc=ManualControlSetpoint();rc.timestamp=us;rc.timestamp_sample=us-1000
        rc.valid=True;rc.data_source=1;rc.aux1=-1.;rc.aux2=-1.
        lp=VehicleLocalPosition();lp.timestamp=us;lp.xy_valid=True;lp.z_valid=True
        for key,m in zip(pubs,[st,fl,la,rc,lp]):
            now = time.monotonic()
            if stopped_topic == aliases[key] and now-start >= 4:
                continue
            if pubs[key] is not None and (not mixed or armed or now >= due[key]):
                pubs[key].publish(m)
                due[key] = now + periods[aliases[key]]
        end=time.monotonic()+.04
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.001);rclpy.spin_once(src,timeout_sec=.001)
        n.audit();s.watchdog(time.monotonic())
        for kind,fields in [('tracking',dict(state=1)),('pose',dict(frame='odom',child='base_link',
                position=[.1,.2,.3],quaternion=[0,0,0,1]))]:
            seq+=1;n.dispatch(dict(session='dds-test',seq=seq,kind=kind,stamp_ns=ns,**fields))
    try:
        while time.monotonic()-start<(7 if mixed else 4) and not s.fault:step()
        if args.case in ('missing','duplicate'):
            wanted='discovery_timeout' if args.case=='missing' else 'conflict'
            assert s.fault and s.fault.startswith(wanted),(s.fault,s.sent)
            assert s.sent==0 and not rows
            report.update(passed=True,case=args.case,sent=0,fault=s.fault,
                          discovery=n.discovery_history[-1])
            return 0
        assert s.sent>20,(s.sent,s.fault,s.gate.blockers)
        before=s.sent
        if stopped_topic:
            assert s.fault=='safety telemetry receipt stale: '+stopped_topic,s.fault
            detail=s.receipt_fault
            assert detail['limit_s'] < detail['age_s'] <= detail['limit_s']+.15,detail
            stopped_topic=None  # Restore real callbacks, not just synthetic state.
        else:
            assert s.fault is None,s.fault
            step(armed=True)
            assert s.fault and s.sent==before,(s.fault,s.sent,before)
        for _ in range(5):step()
        assert s.sent==before,'resumed after disarm'
        flight=[t for t,_ in n.get_topic_names_and_types() if t.startswith('/fmu/')]
        assert not flight,flight
        assert rows,'no DDS output received'
        report.update(passed=True,sent=s.sent,received=len(rows),fault=s.fault,
            no_resume_after_recovery=True,case=args.case,fmu_topics=flight,real_control_publishers=0,
            actual_output_topic=n.ev_topic,paired_local_samples=len(n.paired))
    except Exception as exc:report['error']=repr(exc)
    finally:
        n.close_output();n.destroy_node();src.destroy_node();rclpy.try_shutdown()
        report['elapsed_s']=time.monotonic()-start
        report['telemetry_timing']=s.timing_snapshot(time.monotonic())
        report['telemetry_receipt_fault']=s.receipt_fault
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(evidence=str(out),**report),indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
