"""Explicit <=160s isolated full task transport/control integration.

Domains182/183, test topics only. Ideal free map/navigation/H/dynamics/ACK,
not A*/APF, PX4 SITL, hardware, calibration, or a flight permit.
Default describes and exits without ROS or Docker access.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))


def source(a):
    from task_dynamic_sources import messages
    from task_perception_pipe import specifications
    import yaml
    import rclpy
    from rosidl_runtime_py.utilities import get_message
    from std_msgs.msg import String
    os.environ.update(ROS_DOMAIN_ID='183',ROS_LOCALHOST_ONLY='1',
        FASTRTPS_DEFAULT_PROFILES_FILE=str(ROOT/'config/perception_dds_shm16m.xml'),
        FASTDDS_DEFAULT_PROFILES_FILE=str(ROOT/'config/perception_dds_shm16m.xml'))
    rclpy.init(domain_id=183);node=None
    try:
        node=rclpy.create_node('moving_task_fixture',enable_rosout=False,start_parameter_services=False)
        reads,_,_=specifications(True)
        pubs={t:node.create_publisher(get_message(kind),t,10) for t,kind in reads.items()}
        camera=yaml.safe_load((ROOT/'config/roundtrip_task.yaml').read_text())['camera']
        observed={'goal':None,'goals':0};last_stamp=0;count=0
        def goal(msg):
            observed['goal']=json.loads(msg.data);observed['goals']+=1
        node.create_subscription(String,'/robocup/task_test/navigation_goal',goal,10)
        while time.monotonic()<a.deadline:
            rclpy.spin_once(node,timeout_sec=.005)
            record=json.loads(a.snapshot.read_text())
            if record.get('session')!=a.session:raise ValueError('fixture session changed')
            if record.get('stop'):break
            # Startup construction may pause the ground snapshot. Never restamp
            # it as fresh: send nothing and let the runtime's own age gate act.
            if time.monotonic()-record['at']>.15:continue
            if record['stamp_ns']<=last_stamp:
                continue
            batch=messages(record,a.session,time.monotonic(),camera,observed['goal'])
            for topic,pub in pubs.items():
                key=topic.rsplit('/',1)[-1]
                msg=batch[key]
                for item in msg if isinstance(msg,list) else [msg]:pub.publish(item)
            last_stamp=record['stamp_ns'];count+=1
            if any(t.startswith('/fmu/') for t,_ in node.get_topic_names_and_types()):
                raise RuntimeError('unexpected PX4 topic in source domain')
        print(json.dumps(dict(source_samples=count,goals=observed['goals'])),flush=True)
    finally:
        if node:node.destroy_node()
        rclpy.try_shutdown()


def run():
    started=time.monotonic()
    import yaml
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from task_mixed_plant import plant_type
    Plant=plant_type()  # Legacy module sets env; select our domain afterwards.
    from flight_runtime_check import synthetic_aux_params
    from task_perception_pipe import ContainerTaskInputs
    from task_flight_controller import TaskFlightCore
    from flight_runtime import create_node
    from live_flight_runtime import pump_task_inputs
    from disarmed_ev_lifecycle import atomic_json
    from bench_session_deadline import DeadlineAlarm
    os.environ.update(ROS_DOMAIN_ID='182',ROS_LOCALHOST_ONLY='1',
        FASTRTPS_DEFAULT_PROFILES_FILE=str(ROOT/'config/perception_dds_shm16m.xml'),
        FASTDDS_DEFAULT_PROFILES_FILE=str(ROOT/'config/perception_dds_shm16m.xml'))
    out=ROOT/'evidence'/time.strftime('task_full_pipe_%Y%m%d_%H%M%S');out.mkdir()
    report=dict(passed=False,hardware=False,flight_ready=False,px4_sitl=False,actual_astar_apf=False,
        domains=[182,183],assumptions=['ideal XY response','free map','ideal goal-seeking navigation',
                                    'same-model synthetic H pixels','ideal LAND and ACK'])
    transport=ContainerTaskInputs(out,started+145,isolated=True,navigation_writes=True,isolated_sortie=True)
    snapshot=out/'synthetic_pose.json';child=plant=runtime=core=executor=None
    initialized=False;trace=[];max_x=0.;max_height=.14;next_snapshot=0.
    alarm=DeadlineAlarm(started+153);log=(out/'source.log').open('w')
    def write_pose(stop=False):
        nonlocal next_snapshot
        now=time.monotonic()
        if not stop and now<next_snapshot:return
        p,v=plant.xy.odom()
        atomic_json(snapshot,dict(session=transport.session,at=now,stamp_ns=time.time_ns(),
            position=[*p,.7-plant.z],velocity=[*v,-plant.vz],stop=stop))
        next_snapshot=now+.04
    previous=signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        alarm.arm();rclpy.init(domain_id=182);initialized=True
        plant=Plant();plant.operator_position=True
        # Only container perception enters the runtime; remove unused host VIO sources.
        class Sink:
            def publish(self,msg):pass
        plant.destroy_publisher(plant.local_vio_publisher);plant.pubs['vio']=Sink()
        plant.destroy_publisher(plant.pubs['tracking']);plant.pubs['tracking']=Sink()
        write_pose()
        remote=Path('/workspaces/ros2_ws/robocup_nav')
        child=subprocess.Popen(['docker','exec','isaac_ros_dev','timeout','--signal=TERM','--kill-after=1','148',
            'bash','-c','source /workspaces/ros2_ws/isaac_vio_debug/scripts/container_env.sh; exec "$@"',
            'moving_task_fixture','python3',str(remote/'tests/task_full_pipe_check.py'),
            '--source','--snapshot',str(remote/snapshot.relative_to(ROOT)),
            '--session',transport.session,'--deadline',str(started+145)],stdout=log,stderr=log)
        transport.start(discovery_only=True)
        while time.monotonic()<started+18:
            write_pose();transport.pump()
            if child.poll() is not None:raise RuntimeError('dynamic source exited during startup')
            if all(transport.proxy.count_publishers(t)==1 for t in transport.proxy.reads):break
            time.sleep(.002)
        else:raise RuntimeError('dynamic source discovery timeout')
        cfg=yaml.safe_load((ROOT/'config/roundtrip_task.yaml').read_text())
        # Synthetic ground truth only; never persisted to production config.
        cfg.update(alignment_reviewed=True,full_height_slice_reviewed=True,
                   takeoff_column_reviewed=True,landing_column_reviewed=True,landing_error_budget_m=.03)
        cfg['camera']['axes_reviewed']=True
        core=TaskFlightCore(time.monotonic(),synthetic_aux_params(),cfg,hover=3.)
        runtime=create_node(core,isolated=True,exercise=True,perception_node=transport.port)
        write_pose();transport.activate_inputs()
        executor=SingleThreadedExecutor();executor.add_node(plant);executor.add_node(runtime)
        edge=False
        while time.monotonic()<started+140:
            write_pose();pump_task_inputs(transport,runtime);executor.spin_once(timeout_sec=.002)
            if child.poll() is not None:raise RuntimeError('dynamic source exited')
            if core.fault or core.closed or runtime.task_transport_fault:
                raise RuntimeError(core.fault or core.stop_reason or runtime.task_transport_fault)
            if core.ready_at is not None and not edge:
                plant.operator_position=False;plant.mode=14;edge=True
            p,_=plant.xy.odom();max_x=max(max_x,p[0]);max_height=max(max_height,.84-plant.z)
            state=core.controller.state
            if not trace or trace[-1]['state']!=state:
                trace.append(dict(at=time.monotonic()-started,state=state,position=[*p,.7-plant.z]))
            if any(t.startswith('/fmu/') for t,_ in plant.get_topic_names_and_types()):
                raise RuntimeError('unexpected PX4 topic in plant domain')
            if state in core.controller.TERMINAL:break
        states={x['state'] for x in trace};p,_=plant.xy.odom()
        report.update(runtime=runtime.report(),trace=trace,max_forward_m=max_x,max_px4_height_m=max_height,
            final_position=[*p,.7-plant.z],plant_commands=plant.commands,plant_counts=dict(plant.counts))
        assert core.controller.state=='DONE',core.controller.reason
        assert {'NAVIGATE','ALIGN_H','LAND','DONE'}<=states
        assert runtime.task_input.mission.leg=='RETURN' and 2.4<max_x<=2.5
        assert abs(max_height-1.)<.01 and sum(t*t for t in p)<.05**2
        assert not plant.armed and plant.z>=.695 and edge
        report['passed']=True
    except BaseException as exc:
        report.update(passed=False,error=repr(exc),trace=trace)
        if runtime:report['runtime']=runtime.report()
    finally:
        alarm.cancel()
        if core:core.close('isolated full pipe test ended')
        if executor:executor.shutdown(timeout_sec=.5)
        if runtime:runtime.destroy_node()
        try:transport.close()
        except Exception as exc:report.update(passed=False,cleanup_error=repr(exc))
        if plant:
            write_pose(stop=True);plant.destroy_node()
        if initialized:rclpy.try_shutdown()
        if child:
            try:child.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                child.terminate()
                try:child.wait(timeout=1.)
                except subprocess.TimeoutExpired:child.kill();child.wait(timeout=1.)
                report.update(passed=False,source_cleanup_forced=True)
            report['source_exit']=child.returncode
            if child.returncode!=0:report['passed']=False
        signal.signal(signal.SIGTERM,previous);log.close()
        report.update(elapsed_s=time.monotonic()-started,pipe=transport.report())
        files=[Path(__file__),ROOT/'tests/task_mixed_plant.py',ROOT/'tests/task_dynamic_sources.py',
               *ROOT.glob('scripts/task_*.py'),ROOT/'scripts/flight_runtime.py',ROOT/'scripts/live_flight_runtime.py']
        report['source_sha256']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
        if report['elapsed_s']>160:report['passed']=False
        atomic_json(out/'report.json',report)
        print(json.dumps(dict(evidence=str(out),**report),indent=2))
    return 0 if report['passed'] else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-isolated',action='store_true');parser.add_argument('--source',action='store_true')
    parser.add_argument('--snapshot',type=Path);parser.add_argument('--session');parser.add_argument('--deadline',type=float)
    a=parser.parse_args()
    if a.source:
        if (a.run_isolated or not a.snapshot or not a.session or a.deadline is None
                or not 0<a.deadline-time.monotonic()<=150):parser.error('bounded private source arguments required')
        source(a);return 0
    if not a.run_isolated:
        print('Describe only: full synthetic task pipe, domains182/183, <=160s, no hardware.');return 0
    return run()


if __name__=='__main__':raise SystemExit(main())
