"""Explicit <=30s dual-domain task-input observation. Default describes only.

No sensor/Agent startup, EV, goals, commands, controller tick or parameter writes.
Use the existing host nvBlox message library paths in the calling environment.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]


def input_delivery_observed(node,core):
    # Receipt evidence only, not continuity, geometry or flight readiness.
    return bool(
        all(node.input_counts.get(k,0)>0 for k in ('vehicle_status_v1','vehicle_local_position',
            'estimator_status_flags','vehicle_land_detected','manual_control_setpoint','vio_observed'))
        and all(k in node.task_input.seen for k in ('vio','map','depth'))
        and core.ev.guard.tracking_stamp is not None
        and core.ev.guard.tracking_stamp>0)


def result_code(report):
    return 0 if (report.get('observation_completed') and not report.get('error')
        and not report.get('cleanup_errors') and report.get('contexts_closed')
        and report.get('elapsed_s',float('inf'))<28) else 1


def worker(started,out,container_inputs=False):
    from bench_session_deadline import DeadlineAlarm
    from disarmed_ev_lifecycle import atomic_json
    alarm=DeadlineAlarm(started+25.)
    report=dict(observation_completed=False,flight_ready=False,read_only=True,
        no_controller_tick=True,domains=dict(px4=0,perception=176))
    initialized=False;node=perception=None
    try:
        alarm.arm()
        os.environ.update(ROS_DOMAIN_ID='0',ROS_LOCALHOST_ONLY='1',
            FASTRTPS_DEFAULT_PROFILES_FILE=str(ROOT/'config/perception_dds_shm16m.xml'),
            FASTDDS_DEFAULT_PROFILES_FILE=str(ROOT/'config/perception_dds_shm16m.xml'))
        if container_inputs:
            from px4_runtime_transport import configure
            configure()  # Match the production pipe path, not old direct dual contexts.
        import yaml
        import rclpy
        from flight_readiness import read_params
        from aux_switch_decoder import validate_contract
        from task_domain_io import PerceptionDomain
        from task_flight_controller import TaskFlightCore
        from flight_runtime import create_node
        params=read_params(Path('/home/cfly/param.params.txt'));validate_contract(params)
        platform=yaml.safe_load((ROOT/'config/platform.yaml').read_text())
        if platform.get('live_flight_enabled') is not False:raise ValueError('flight permit must remain false')
        cfg=yaml.safe_load((ROOT/'config/roundtrip_task.yaml').read_text())
        rclpy.init(args=[],domain_id=0);initialized=True
        if container_inputs:
            from task_perception_pipe import ContainerTaskInputs
            perception=ContainerTaskInputs(out,started+23.)
        else:perception=PerceptionDomain()
        core=TaskFlightCore(time.monotonic(),params,cfg)
        node=create_node(core,task_preview=True,perception_node=perception.port)
        if container_inputs:perception.start()
        while time.monotonic()<started+20.:
            perception.pump();rclpy.spin_once(node,timeout_sec=.005)
            if node.armed_on_real_input or core.fault:
                raise RuntimeError('observation stopped: '+str(core.fault))
        report['runtime']=node.report()
        report['observation_completed']=True
        report['input_delivery_observed']=input_delivery_observed(node,core)
        report['scope']='actual task-input parsing only; configuration blockers retained; no route or control acceptance'
    except Exception as exc:
        report['observation_completed']=False
        report['error']=repr(exc)
        if node:report['runtime']=node.report()
    finally:
        alarm.cancel()
        errors=[]
        for label,close in [('runtime',lambda:node.destroy_node() if node else None),
                ('perception',lambda:perception.close() if perception else None),
                ('px4_context',lambda:rclpy.try_shutdown() if initialized else None)]:
            try:close()
            except Exception as exc:errors.append(dict(component=label,error=repr(exc)))
        report.update(cleanup_errors=errors,contexts_closed=not errors,
            elapsed_s=time.monotonic()-started)
        report['container_inputs']=container_inputs
        if container_inputs and perception:report['perception_pipe']=perception.report()
        atomic_json(out/'report.json',report)
    return result_code(report)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observe-existing-inputs',action='store_true')
    parser.add_argument('--container-inputs',action='store_true',help='explicit read-only pipe candidate, no flight routing')
    args=parser.parse_args()
    if not args.observe_existing_inputs:
        print(json.dumps(dict(default='describe only; no ROS/device access',limit_s=30,
            observation_target_s=20,task_output_publishers_created=0,starts_agent=False,starts_sensors=False)))
        return 0
    from disarmed_ev_session import supervise
    from disarmed_ev_lifecycle import atomic_json
    with open('/tmp/robocup_flight_runtime_shadow.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        out=ROOT/'evidence'/('task_readonly_'+uuid.uuid4().hex);out.mkdir()
        started=time.monotonic();pid=os.fork()
        if pid==0:
            os.setsid()
            try:code=worker(started,out,args.container_inputs)
            except BaseException:code=2
            os._exit(code)
        code=supervise(pid,started+28.)
        atomic_json(out/'supervisor.json',dict(exit_code=code,elapsed_s=time.monotonic()-started,
            limit_s=30,hardware_retry=False))
        print(str(out));return code


if __name__=='__main__':raise SystemExit(main())
