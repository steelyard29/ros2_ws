"""Bounded real ROS message transport on domain187, synthetic inputs ONLY.

One process, three real ROS nodes; not cross-container or flight validation.
Default describes. No real /fmu routes or hardware subscriptions.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-isolated',action='store_true')
    a=p.parse_args()
    if not a.run_isolated:
        print('Describe only: <=18s synthetic ROS domain187; no real PX4/sensors.');return 0
    from bench_session_deadline import DeadlineAlarm
    start=time.monotonic();alarm=DeadlineAlarm(start+17);alarm.arm()
    out=ROOT/'evidence'/('resident_ev_dds_'+uuid.uuid4().hex);out.mkdir()
    report=dict(passed=False,hardware=False,flight_ready=False,domain=187)
    ros=executor=resident=plant=runtime=None
    try:
        from flight_runtime_check import SyntheticPlant
        from resident_ev_node import create_node as resident_node
        from flight_runtime import create_node as task_node
        from flight_runtime_core import FlightRuntimeCore
        from external_ev_evidence import ExternalEvEvidence
        import rclpy as ros
        from rclpy.executors import SingleThreadedExecutor
        # The legacy fixture sets domain177 on import; explicitly supersede it.
        os.environ.update(ROS_DOMAIN_ID='187',ROS_LOCALHOST_ONLY='1')
        os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE',None)
        os.environ.pop('FASTDDS_DEFAULT_PROFILES_FILE',None)
        ros.init(domain_id=187)
        executor=SingleThreadedExecutor()
        plant=SyntheticPlant('position-slot');plant.operator_position=True
        resident=resident_node(out.name,isolated=True)
        executor.add_node(plant);executor.add_node(resident)
        while time.monotonic()<start+7 and resident.core.sent_count<30:
            executor.spin_once(timeout_sec=.002)
            if resident.core.fault:raise RuntimeError(resident.core.fault)
        assert resident.core.sent_count>=30,'resident startup/send missing'
        ends=plant.get_publishers_info_by_topic(resident.output)
        assert len(ends)==1,'EV owner count'
        core=FlightRuntimeCore(time.monotonic())
        core.bind_external_ev(ExternalEvEvidence(out.name,tuple(ends[0].endpoint_gid)))
        runtime=task_node(core,isolated=True,exercise=False)
        executor.add_node(runtime)
        while time.monotonic()<start+12 and core.external_ev.count<50:
            executor.spin_once(timeout_sec=.002)
            if core.fault or resident.core.fault:
                raise RuntimeError(core.fault or resident.core.fault)
        assert core.external_ev.count>=50,'external EV subscription missing'
        assert 'vehicle_visual_odometry' not in runtime.pubs,'duplicate task EV publisher'
        assert core.ev_receipt_recent(time.monotonic()),'receipt not fresh'
        report.update(sent=resident.core.sent_count,received=core.external_ev.count,
            plant_ev=plant.counts['ev'],task_ev_outputs=runtime.counts['vehicle_visual_odometry'],
            unique_ev_publishers=plant.count_publishers(resident.output))
        assert report['unique_ev_publishers']==1
        bad=[name for name,_ in plant.get_topic_names_and_types() if name.startswith('/fmu/')]
        assert not bad,'unexpected real-topic graph'
        resident.close_output()
        stop=time.monotonic()
        while time.monotonic()<stop+.8:executor.spin_once(timeout_sec=.002)
        report.update(stop_fault=core.fault,external_fault=core.external_ev.fault,
                      commands=plant.commands,armed=plant.armed)
        assert core.fault and not core.ev_receipt_recent(time.monotonic()),'loss not latched'
        assert not plant.commands and not plant.armed,'unexpected synthetic motion request'
        report['passed']=True
    except Exception as exc:report['error']=repr(exc)
    finally:
        if resident:resident.close_output()
        if executor:executor.shutdown(timeout_sec=.2)
        for node in (runtime,resident,plant):
            if node:node.destroy_node()
        if ros:ros.try_shutdown()
        alarm.cancel()
        report['elapsed_s']=time.monotonic()-start
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(evidence=str(out),**report),indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
