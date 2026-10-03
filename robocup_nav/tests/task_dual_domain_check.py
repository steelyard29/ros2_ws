#!/usr/bin/env python3
"""Explicit <=12s synthetic DDS domains182/183; no real graph or /fmu route.

Actual runtime/scene callbacks and actual generated messages; stationary WAIT
only, not SITL, physics, real planner, real H detection or flight acceptance.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-isolated',action='store_true')
    p.add_argument('--fault',choices=('none','stop-vio','duplicate-vio'),default='none')
    args=p.parse_args()
    if not args.run_isolated:
        print('Describe only: actual DDS, synthetic domains182/183, <=12s, no hardware.')
        return 0
    import yaml
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    from nvblox_msgs.msg import DistanceMapSlice
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker
    from task_map_fixture import markers
    from flight_runtime_check import SyntheticPlant,synthetic_aux_params
    from flight_runtime import create_node
    from task_flight_controller import TaskFlightCore
    from task_domain_io import PerceptionDomain
    from bench_session_deadline import DeadlineAlarm
    # The imported historical fixture sets domain177; explicitly override BEFORE init.
    os.environ.update(ROS_DOMAIN_ID='182',ROS_LOCALHOST_ONLY='1',
        FASTRTPS_DEFAULT_PROFILES_FILE=str(ROOT/'config/perception_dds_shm16m.xml'),
        FASTDDS_DEFAULT_PROFILES_FILE=str(ROOT/'config/perception_dds_shm16m.xml'))
    cfg=yaml.safe_load((ROOT/'config/roundtrip_task.yaml').read_text())
    cfg.update(alignment_reviewed=True,bounds_odom=[-1,4,-2,2],full_height_slice_reviewed=True,
               takeoff_column_reviewed=True)
    out=ROOT/'evidence'/time.strftime('task_dual_domain_%Y%m%d_%H%M%S');out.mkdir()
    start=time.monotonic();alarm=DeadlineAlarm(start+10)
    report=dict(passed=False,hardware=False,flight_ready=False,domains=[182,183],fault_fixture=args.fault)
    plant=runtime=perception=sensors=executor=core=None;initialized=False
    counters={'goal_sensor_domain':0,'goal_px4_domain':0};fault_pub=None;stop_vio={'value':False}
    try:
        alarm.arm();rclpy.init(args=[],domain_id=182);initialized=True
        perception=PerceptionDomain(domain_id=183,isolated=True)
        sensors=Node('dual_domain_sensor_fixture',context=perception.context,
                     enable_rosout=False,start_parameter_services=False)
        perception.executor.add_node(sensors)
        plant=SyntheticPlant('nominal',use_aux=True);plant.operator_position=True
        # Eliminate the fixture's domain182 VIO writers, then provide those exact
        # original messages in183. No copy across domains or changed source stamps.
        for key,kind in (('vio',Odometry),('tracking',String)):
            plant.destroy_publisher(plant.pubs[key])
            pub=sensors.create_publisher(kind,plant.inputs[key],10)
            if key=='vio':
                class VioPort:
                    def publish(self,m):
                        if not stop_vio['value']:pub_vio.publish(m)
                pub_vio=pub;plant.pubs[key]=VioPort()
            else:plant.pubs[key]=pub
        core=TaskFlightCore(time.monotonic(),synthetic_aux_params(),cfg)
        runtime=create_node(core,isolated=True,exercise=False,perception_node=perception.port)
        pubmap={k:sensors.create_publisher(kind,runtime.task_input.topics[k],10) for k,kind in
            [('map',DistanceMapSlice),('bounds',Marker),('depth',Image),('navigation',String),('h',String)]}
        def sensor_tick():
            stamp=sensors.get_clock().now().to_msg();ns=stamp.sec*10**9+stamp.nanosec
            depth=Image();depth.header.stamp=stamp;depth.width=depth.height=1;depth.data=[1]
            pubmap['depth'].publish(depth)
            m=DistanceMapSlice();m.header.stamp=stamp;m.header.frame_id='odom'
            m.width=m.height=100;m.resolution=.1;m.origin.x=m.origin.y=-5.;m.origin.z=.86
            m.unknown_value=-1000.;m.data=[2.]*10000;pubmap['map'].publish(m)
            for b in markers(stamp):pubmap['bounds'].publish(b)
            mission=runtime.task_input.mission
            if mission:
                pubmap['navigation'].publish(String(data=json.dumps(dict(schema=1,frame_id='odom',
                    goal_id=mission.goal_id,stamp_ns=ns,path_stamp_ns=ns,velocity_xy=[.1,0.]))))
        sensors.create_timer(.05,sensor_tick)
        def count(key):
            def callback(m):counters[key]+=1
            return callback
        sensors.create_subscription(String,runtime.task_input.goal_topic,count('goal_sensor_domain'),10)
        plant.create_subscription(String,runtime.task_input.goal_topic,count('goal_px4_domain'),10)
        executor=SingleThreadedExecutor();executor.add_node(plant);executor.add_node(runtime)
        nominal=None;goal_sent=False;injected=False;recovered=False;ev_before_recovery=None
        while time.monotonic()-start<8:
            perception.pump();executor.spin_once(timeout_sec=.005)
            elapsed=time.monotonic()-start
            if elapsed>3 and nominal is None:
                nominal=dict(initialized=runtime.task_input.mission is not None,
                    navigation_received=runtime.task_input.command is not None,
                    known_free=bool(core.controller.scene and core.controller.scene.footprint_known_free),
                    ev_count=plant.counts['ev'])
            if elapsed>3 and not goal_sent:
                # Test the existing selective goal port, not autonomous flight.
                runtime.task_input.goal_pub.publish(String(data='{"fixture":"routing-only"}'))
                goal_sent=True
            if elapsed>4 and args.fault!='none' and not injected:
                injected=True
                if args.fault=='stop-vio':stop_vio['value']=True
                else:fault_pub=sensors.create_publisher(Odometry,plant.inputs['vio'],10)
            if elapsed>6 and injected and not recovered:
                recovered=True;ev_before_recovery=runtime.counts['vehicle_visual_odometry']
                stop_vio['value']=False
                if fault_pub:sensors.destroy_publisher(fault_pub);fault_pub=None
        assert nominal and all(nominal.values()),nominal
        assert counters['goal_sensor_domain']==1 and counters['goal_px4_domain']==0,counters
        assert not plant.armed and not plant.commands,plant.commands
        assert runtime.report()['dds_domains']==dict(px4=182,perception=183)
        assert plant.count_publishers(plant.inputs['vio'])==0,'VIO leaked into PX4 domain'
        assert sensors.count_publishers(plant.inputs['vehicle_status_v1'])==0,'PX4 source leaked into perception domain'
        for node in (plant,sensors):
            assert not any(t.startswith('/fmu/') for t,_ in node.get_topic_names_and_types())
        if args.fault!='none':
            assert core.ev.inhibit and recovered,core.report()
            assert runtime.counts['vehicle_visual_odometry']==ev_before_recovery,'EV automatically resumed'
            if args.fault=='stop-vio':
                assert core.controller.state=='STOPPED' and 'VIO stale' in core.fault,core.report()
            else:assert core.controller.state=='HANDOVER' and core.ownership_fault,core.report()
        else:assert runtime.task_input.command is not None and core.controller.state=='WAIT',runtime.report()
        report.update(passed=True,nominal=nominal,routing=counters,runtime=runtime.report(),
                      source_recovered=recovered,ev_before_recovery=ev_before_recovery)
    except Exception as exc:report['error']=repr(exc)
    finally:
        alarm.cancel()
        if core:core.close('isolated dual-domain test ended')
        if executor:executor.shutdown(timeout_sec=.5)
        if runtime:runtime.destroy_node()
        if plant:plant.destroy_node()
        if sensors:sensors.destroy_node()
        if perception:perception.close()
        if initialized:rclpy.try_shutdown()
        report.update(elapsed_s=time.monotonic()-start,contexts_closed=True)
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(evidence=str(out),**report),indent=2))
    return 0 if report['passed'] and report['elapsed_s']<=12 else 1


if __name__=='__main__':raise SystemExit(main())
