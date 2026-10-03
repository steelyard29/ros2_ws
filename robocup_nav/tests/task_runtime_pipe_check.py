"""Explicit <=22s: actual task runtime with host synthetic PX4 + container inputs.

Domains182/183 only, no /fmu. Position mode held throughout; no synthetic ARM.
Tests startup, EV delivery on test topic, planning request and accepted response.
Not flight physics, an A* planner, real hardware or a flight release.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-isolated',action='store_true')
    parser.add_argument('--transport-fault-check',action='store_true')
    a=parser.parse_args()
    if not a.run_isolated:
        print('Describe only: explicit isolated domains182/183 runtime test; no hardware.');return 0
    import yaml
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    from flight_runtime_check import SyntheticPlant,synthetic_aux_params
    from task_perception_pipe import ContainerTaskInputs
    from task_flight_controller import TaskFlightCore
    from flight_runtime import create_node
    from disarmed_ev_lifecycle import atomic_json
    from bench_session_deadline import DeadlineAlarm
    from live_flight_runtime import pump_task_inputs
    os.environ.update(ROS_DOMAIN_ID='182',ROS_LOCALHOST_ONLY='1',
        FASTRTPS_DEFAULT_PROFILES_FILE=str(ROOT/'config/perception_dds_shm16m.xml'),
        FASTDDS_DEFAULT_PROFILES_FILE=str(ROOT/'config/perception_dds_shm16m.xml'))
    out=ROOT/'evidence'/time.strftime('task_runtime_pipe_%Y%m%d_%H%M%S');out.mkdir()
    start=time.monotonic();alarm=DeadlineAlarm(start+19.)
    report=dict(passed=False,hardware=False,flight_ready=False,domains=[182,183])
    transport=ContainerTaskInputs(out,start+16,isolated=True,navigation_writes=True)
    child=plant=runtime=executor=core=None;initialized=False
    log=(out/'fixture.log').open('w')
    try:
        alarm.arm()
        child=subprocess.Popen(['docker','exec','isaac_ros_dev','timeout','--signal=TERM','--kill-after=1','20',
            'bash','-c','source /workspaces/ros2_ws/isaac_vio_debug/scripts/container_env.sh; exec "$@"',
            'task_runtime_fixture','python3','/workspaces/ros2_ws/robocup_nav/tests/task_pipe_check.py',
            '--fixture-deadline',str(start+17),'--navigation-check','--runtime-fixture'],stdout=log,stderr=log)
        transport.start(discovery_only=True)
        while time.monotonic()<start+5:
            transport.pump()
            if all(transport.proxy.count_publishers(t)==1 for t in transport.proxy.reads):break
            time.sleep(.002)
        assert all(transport.proxy.count_publishers(t)==1 for t in transport.proxy.reads),'startup source discovery'
        rclpy.init(domain_id=182);initialized=True
        plant=SyntheticPlant('nominal',use_aux=True);plant.operator_position=True
        class Sink:
            def publish(self,msg):pass
        for key in ('vio','tracking'):
            plant.destroy_publisher(plant.pubs[key]);plant.pubs[key]=Sink()
        cfg=yaml.safe_load((ROOT/'config/roundtrip_task.yaml').read_text())
        cfg.update(alignment_reviewed=True,bounds_odom=[-1,4,-2,2],takeoff_column_reviewed=True)
        core=TaskFlightCore(time.monotonic(),synthetic_aux_params(),cfg)
        runtime=create_node(core,isolated=True,exercise=True,perception_node=transport.port)
        transport.activate_inputs()
        executor=SingleThreadedExecutor();executor.add_node(plant);executor.add_node(runtime)
        goal_sent=False
        while time.monotonic()<start+(11 if a.transport_fault_check else 13):
            pump_task_inputs(transport,runtime);executor.spin_once(timeout_sec=.002)
            if core.fault:raise RuntimeError(core.fault)
            if runtime.task_input.mission is not None and not goal_sent:
                runtime.task_input.goal_pub.publish(String(data=json.dumps(runtime.task_input.mission.planner_goal())))
                goal_sent=True
        report.update(runtime=json.loads(json.dumps(runtime.report())),plant_counts=dict(plant.counts),
                      plant_commands=plant.commands,goal_sent=goal_sent)
        assert goal_sent and runtime.task_input.command is not None
        assert plant.counts['ev']>0 and not plant.armed and not plant.commands
        assert core.controller.state=='WAIT' and not core.fault
        assert core.ready_at is not None,'actual runtime never reached operator-edge readiness'
        assert transport.proxy.command_seq==transport.proxy.ack_seq
        assert not any(t.startswith('/fmu/') for t,_ in plant.get_topic_names_and_types())
        if a.transport_fault_check:
            before=dict(runtime.input_counts)
            seq=transport.proxy.command_seq
            runtime.task_transport_failed('isolated requested task input interruption')
            pump_task_inputs(transport,runtime)
            ev_after_fault=runtime.counts['vehicle_visual_odometry']
            while time.monotonic()<start+13:
                pump_task_inputs(transport,runtime)
                executor.spin_once(timeout_sec=.002)
            assert runtime.input_counts['vehicle_status_v1']>before['vehicle_status_v1']
            assert runtime.input_counts['vehicle_local_position']>before['vehicle_local_position']
            assert runtime.counts['vehicle_visual_odometry']==ev_after_fault
            assert transport.proxy.command_seq==seq and not plant.commands
            assert core.ev_stopped and not core.closed
            report['interruption']=dict(px4_callbacks_continued=True,
                new_ev_after_fault=runtime.counts['vehicle_visual_odometry']-ev_after_fault,
                new_navigation_requests=transport.proxy.command_seq-seq,
                airborne=False,runtime=runtime.report())
        report['passed']=True
    except BaseException as exc:
        report['error']=repr(exc)
        if runtime:report['runtime']=runtime.report()
    finally:
        alarm.cancel()
        if core:core.close('isolated runtime ended')
        if executor:executor.shutdown(timeout_sec=.5)
        if runtime:runtime.destroy_node()
        if plant:plant.destroy_node()
        try:transport.close()
        except Exception as exc:report.update(passed=False,cleanup_error=repr(exc))
        report['pipe']=transport.report()
        if initialized:rclpy.try_shutdown()
        if child:
            try:child.wait(timeout=max(.1,start+21-time.monotonic()))
            except subprocess.TimeoutExpired:
                child.terminate();child.wait(timeout=1);report['passed']=False
            report['fixture_exit_code']=child.returncode
            if child.returncode!=0:report['passed']=False
        log.close();report['elapsed_s']=time.monotonic()-start
        if report['elapsed_s']>=22:report['passed']=False
        atomic_json(out/'report.json',report)
        print(json.dumps(dict(evidence=str(out),**report)))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
