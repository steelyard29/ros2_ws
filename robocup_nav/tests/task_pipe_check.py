"""Explicit <=20s synthetic container -> pipe -> real TaskSceneInput check.

Domain183 only. No PX4 context, real sensors, goal writers or controller ticks.
"""
import argparse
from array import array
import json
import os
from pathlib import Path
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))


def fixture(deadline,navigation_check=False,runtime_check=False):
    from task_perception_pipe import specifications
    os.environ.update(ROS_DOMAIN_ID='183',ROS_LOCALHOST_ONLY='1',
        RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
        FASTRTPS_DEFAULT_PROFILES_FILE=str(ROOT/'config/perception_dds_shm16m.xml'),
        FASTDDS_DEFAULT_PROFILES_FILE=str(ROOT/'config/perception_dds_shm16m.xml'))
    import rclpy
    from rosidl_runtime_py.utilities import get_message
    rclpy.init(domain_id=183)
    node=rclpy.create_node('task_pipe_synthetic_sources',enable_rosout=False,start_parameter_services=False)
    reads,_,_=specifications(True)
    pubs={t:node.create_publisher(get_message(k),t,10) for t,k in reads.items()}
    observed=dict(goal=None,goal_count=0,status_count=0)
    if navigation_check:
        from std_msgs.msg import String
        def goal(m):
            observed['goal']=json.loads(m.data);observed['goal_count']+=1
        def status(m):observed['status_count']+=1
        node.create_subscription(String,'/robocup/task_test/navigation_goal',goal,10)
        node.create_subscription(String,'/robocup/task_test/readiness',status,10)
    seq=0
    try:
        while time.monotonic()<deadline:
            stamp=node.get_clock().now().to_msg();ns=stamp.sec*10**9+stamp.nanosec
            for topic,kind in reads.items():
                m=get_message(kind)()
                if hasattr(m,'header'):m.header.stamp=stamp;m.header.frame_id='odom'
                if kind.endswith('/Odometry'):
                    m.child_frame_id='base_link';m.pose.pose.orientation.w=1.;m.pose.pose.position.x=.01
                elif kind.endswith('/Image'):
                    m.width=640;m.height=480;m.encoding='16UC1';m.step=1280
                    m.data=array('B',[0])*614400
                elif kind.endswith('/DistanceMapSlice'):
                    m.width=m.height=100;m.resolution=.1;m.origin.z=.86
                    m.unknown_value=-1000.;m.data=[2.]*10000
                elif kind.endswith('/Marker'):
                    m.ns='fixture';m.pose.orientation.w=1.
                else:
                    m.data=json.dumps(dict(vo_state=1,stamp_ns=ns,source_stamp_ns=ns,
                        candidate_stable=False,fixture_seq=seq))
                    if topic.endswith('/navigation') and observed['goal']:
                        m.data=json.dumps(dict(schema=1,frame_id='odom',goal_id=observed['goal']['goal_id'],
                            stamp_ns=ns,path_stamp_ns=ns,velocity_xy=[.1,0.],synthetic_echo=True))
                pubs[topic].publish(m)
            seq+=1;rclpy.spin_once(node,timeout_sec=.04)
        assert not any(t.startswith('/fmu/') for t,_ in node.get_topic_names_and_types())
        if navigation_check:
            assert observed['goal_count']==1 and (observed['status_count']>=1 if runtime_check
                                                 else observed['status_count']==1),observed
            print(json.dumps(observed))
    finally:node.destroy_node();rclpy.try_shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-isolated',action='store_true')
    parser.add_argument('--fixture-deadline',type=float)
    parser.add_argument('--navigation-check',action='store_true')
    parser.add_argument('--runtime-fixture',action='store_true')
    a=parser.parse_args()
    if a.fixture_deadline is not None:
        if not 0<a.fixture_deadline-time.monotonic()<=18:parser.error('fixture budget invalid')
        fixture(a.fixture_deadline,a.navigation_check,a.runtime_fixture);return 0
    if not a.run_isolated:
        print('Describe only: synthetic domain183, <=20s, no hardware.');return 0
    import yaml
    from task_perception_pipe import ContainerTaskInputs
    from task_flight_controller import TaskFlightCore
    from task_scene_input import TaskSceneInput
    from test_aux_switch_decoder import params
    from std_msgs.msg import String
    from disarmed_ev_lifecycle import atomic_json
    out=ROOT/'evidence'/time.strftime('task_pipe_%Y%m%d_%H%M%S');out.mkdir()
    start=time.monotonic();report=dict(passed=False,hardware=False,flight_ready=False)
    transport=ContainerTaskInputs(out,start+14,isolated=True,navigation_writes=a.navigation_check)
    cfg=yaml.safe_load((ROOT/'config/roundtrip_task.yaml').read_text())
    core=TaskFlightCore(start,params(),cfg)
    scene=TaskSceneInput(transport.port,core,isolated=True,read_only=True)
    tracking=[]
    topic='/robocup/runtime_test/input/tracking'
    transport.port.create_subscription(String,topic,lambda m:tracking.append(json.loads(m.data)),10)
    responses=[];goal_pub=status_pub=None
    if a.navigation_check:
        goal_pub=transport.port.create_publisher(String,'/robocup/task_test/navigation_goal',10)
        status_pub=transport.port.create_publisher(String,'/robocup/task_test/readiness',10)
        transport.port.create_subscription(String,'/robocup/task_test/input/navigation',
            lambda m:responses.append(json.loads(m.data)),10)
    log=(out/'fixture.log').open('w');child=None
    try:
        root='/workspaces/ros2_ws/robocup_nav'
        child=subprocess.Popen(['docker','exec','isaac_ros_dev','timeout','--signal=TERM','--kill-after=1','18',
            'bash','-c','source /workspaces/ros2_ws/isaac_vio_debug/scripts/container_env.sh; exec "$@"',
            'synthetic_task_sources','python3',root+'/tests/task_pipe_check.py',
            '--fixture-deadline',str(start+16),*(['--navigation-check'] if a.navigation_check else [])],stdout=log,stderr=log)
        transport.start()
        sent=False
        while time.monotonic()<start+10:
            transport.pump();time.sleep(.002)
            if a.navigation_check and not sent and time.monotonic()>start+5:
                goal_pub.publish(String(data=json.dumps(dict(frame_id='odom',goal_id='pipe:OUTBOUND',
                    position=[2.5,0.,.86],yaw=0.))))
                status_pub.publish(String(data='{"fixture":true}'))
                sent=True
        report.update(scene=scene.report(),pipe_before_close=transport.report(),tracking_count=len(tracking))
        assert all(transport.proxy.received.get(t,0)>=10 for t in transport.proxy.reads),transport.report()
        assert all(k in scene.seen for k in ('vio','map','depth')),scene.report()
        assert scene.pose_data is not None and scene.grid is not None
        assert scene.mapper_owned()
        assert core.task_config['alignment_reviewed'] is False
        assert transport.proxy.fault is None
        if a.navigation_check:
            assert transport.proxy.command_seq==transport.proxy.ack_seq==2
            echoed=[d for d in responses if d.get('synthetic_echo') and d.get('goal_id')=='pipe:OUTBOUND']
            assert echoed,'goal did not traverse container ROS subscriber and return'
            report.update(navigation_check=True,synthetic_returned_commands=len(echoed),
                          note='request/ROS subscription/response transport only; no A* or flight control')
        report['passed']=True
    except Exception as exc:report['error']=repr(exc)
    finally:
        try:transport.close()
        except Exception as exc:report.update(passed=False,cleanup_error=repr(exc))
        report['pipe_after_close']=transport.report()
        if child:
            try:child.wait(timeout=max(.1,start+19-time.monotonic()))
            except subprocess.TimeoutExpired:
                child.terminate();child.wait(timeout=1);report['passed']=False
            report['fixture_exit_code']=child.returncode
            if child.returncode!=0:report['passed']=False
        log.close();report['elapsed_s']=time.monotonic()-start
        if report['elapsed_s']>=20:report['passed']=False
        atomic_json(out/'report.json',report)
        print(json.dumps(dict(evidence=str(out),**report)))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
