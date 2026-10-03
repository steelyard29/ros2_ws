"""Allowlisted task subscriptions; optional permit-bound navigation publishers.

Default only describes. Explicit reader invocation needs a private control file,
session, close receipt and bounded deadline. Isolated mode uses domain183.
"""
import argparse
import base64
import json
import os
from pathlib import Path
import signal
import select
import sys
import time
from task_perception_pipe import specifications,depth_evidence,MAX_PACKET
from disarmed_ev_lifecycle import atomic_json,stop_requested
from disarmed_ev_transport import PacketWriter
from bench_session_deadline import DeadlineAlarm


def run(a,state):
    if stop_requested(a.control_file,a.session):return 0
    root=Path(__file__).resolve().parents[1]
    domain=183 if a.isolated else 176
    os.environ.update(ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1',
        RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
        FASTRTPS_DEFAULT_PROFILES_FILE=str(root/'config/perception_dds_shm16m.xml'),
        FASTDDS_DEFAULT_PROFILES_FILE=str(root/'config/perception_dds_shm16m.xml'))
    import rclpy
    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import serialize_message
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init(domain_id=domain)
    state['ros_closed']=False
    node=None
    try:
        node=rclpy.create_node('robocup_task_pipe_reader',enable_rosout=False,start_parameter_services=False)
        reads,writes,depth_topic=specifications(a.isolated)
        graph={};seq=0
        publishers={};request_buffer=b'';request_guard=None
        if a.isolated_navigation_writes or a.authorized_task_navigation:
            from std_msgs.msg import String
            from task_navigation_pipe import NavigationRequestGuard
            publishers={t:node.create_publisher(String,t,10) for t in writes}
            request_guard=NavigationRequestGuard(a.session,writes,time.monotonic)
            os.set_blocking(sys.stdin.fileno(),False)
        os.set_blocking(sys.stdout.fileno(),False)
        writer=PacketWriter(sys.stdout.fileno(),capacity=2*MAX_PACKET)
        state['transport']=writer.stats
        def emit(kind,**data):
            nonlocal seq
            seq+=1
            packet=dict(session=a.session,seq=seq,at=time.monotonic(),domain=domain,kind=kind,**data)
            writer.enqueue(packet)
        def audit():
            nonlocal graph
            graph={topic:[dict(name=x.node_name,namespace=x.node_namespace,gid=list(x.endpoint_gid))
                         for x in node.get_publishers_info_by_topic(topic)] for topic in reads.keys()|writes}
            emit('graph',graph=graph)
        def receive(topic,msg):
            ends=graph.get(topic,[])
            if len(ends)!=1 or stop_requested(a.control_file,a.session):return
            if topic==depth_topic:emit('data',topic=topic,gid=ends[0]['gid'],depth=depth_evidence(msg))
            else:
                data=serialize_message(msg)
                if len(data)>MAX_PACKET//2:raise BufferError('task source message too large')
                emit('data',topic=topic,gid=ends[0]['gid'],cdr=base64.b64encode(data).decode('ascii'))
        for topic,kind in reads.items():
            qos=qos_profile_sensor_data if kind in ('nav_msgs/msg/Odometry','sensor_msgs/msg/Image') else 10
            node.create_subscription(get_message(kind),topic,lambda m,t=topic:receive(t,m),qos)
        node.create_timer(.1,audit)
        while time.monotonic()<a.deadline and not stop_requested(a.control_file,a.session):
            rclpy.spin_once(node,timeout_sec=.005);writer.pump()
            if request_guard:
                if select.select([sys.stdin],[],[],0)[0]:
                    chunk=os.read(sys.stdin.fileno(),65536)
                    if not chunk:raise RuntimeError('navigation request stream closed')
                    request_buffer+=chunk
                    if len(request_buffer)>131072:raise BufferError('navigation request buffer exhausted')
                for _ in range(8):
                    if b'\n' not in request_buffer:break
                    line,request_buffer=request_buffer.split(b'\n',1)
                    packet=json.loads(line)
                    topic,data=request_guard.accept(packet)
                    ends=node.get_publishers_info_by_topic(topic)
                    if len(ends)!=1 or ends[0].node_name!=node.get_name():
                        raise RuntimeError('navigation output publisher competition')
                    if stop_requested(a.control_file,a.session) or time.monotonic()>=a.deadline:
                        raise RuntimeError('navigation request after stop/deadline')
                    publishers[topic].publish(String(data=data))
                    emit('write_ack',topic=topic,command_seq=packet['seq'])
                    state['navigation_requests_published']=request_guard.seq
        state['reason']='stop_requested' if stop_requested(a.control_file,a.session) else 'deadline'
    finally:
        if node:node.destroy_node()
        rclpy.try_shutdown();state['ros_closed']=True
    return 0


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session');p.add_argument('--deadline',type=float)
    p.add_argument('--control-file',type=Path);p.add_argument('--receipt-file',type=Path)
    p.add_argument('--isolated',action='store_true')
    p.add_argument('--isolated-navigation-writes',action='store_true')
    p.add_argument('--authorized-task-navigation',action='store_true')
    p.add_argument('--isolated-sortie',action='store_true')
    a=p.parse_args()
    if a.isolated_navigation_writes and not a.isolated:
        p.error('navigation writes currently require isolated domain183')
    if a.authorized_task_navigation and (a.isolated or a.isolated_navigation_writes):
        p.error('real task authority cannot select isolated mode')
    from task_pipe_authority import reader_budget,verify_grant
    try:
        limit=reader_budget(isolated=a.isolated,navigation_writes=a.isolated_navigation_writes,
                            live=a.authorized_task_navigation,isolated_sortie=a.isolated_sortie)
    except ValueError as exc:p.error(str(exc))
    if a.session is None:
        print('Describe only: no ROS/device access; explicit bounded read-only worker.');return 0
    if (not a.control_file or not a.receipt_file or a.deadline is None
            or not 0<a.deadline-time.monotonic()<=limit):p.error('requires bounded deadline and private files')
    if a.authorized_task_navigation:
        try:
            control=json.loads(a.control_file.read_text())
            if control.get('session')!=a.session or control.get('stop') is not False:
                raise ValueError('inactive or mismatched task control')
            verify_grant(control.get('task_authority'),a.session,a.deadline,Path(__file__).resolve().parents[1])
        except (ValueError,TypeError,OSError,KeyError) as exc:p.error(str(exc))
    alarm=DeadlineAlarm(a.deadline)
    state=dict(session=a.session,pid=os.getpid(),started_at=time.monotonic(),ros_closed=True)
    code=2
    def interrupted(signum,frame):raise KeyboardInterrupt()
    previous=signal.signal(signal.SIGTERM,interrupted)
    try:alarm.arm();code=run(a,state)
    except BaseException as exc:state['reason']=repr(exc)
    finally:
        alarm.cancel();signal.signal(signal.SIGTERM,previous)
        state.update(closed_at=time.monotonic(),exit_code=code)
        atomic_json(a.receipt_file,state)
    return code


if __name__=='__main__':raise SystemExit(main())
