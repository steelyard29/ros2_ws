"""Read-only container worker: two subscriptions -> private stdout pipe.

Never publishes ROS data. No parameter writes, drivers, or PX4 interfaces.
Exits by shared monotonic deadline even if its docker client disconnects.
"""
import argparse
import json
import os
import signal
import sys
import time
from pathlib import Path
from disarmed_ev_lifecycle import atomic_json, stop_requested
from disarmed_ev_transport import PacketWriter
from ev_clock_evidence import snapshot
from reader_timing_evidence import ReaderTiming


def run(a, state):
    timing=ReaderTiming()
    state['reader_timing']=timing.report
    def stopped():
        return timing.call('stop_file',stop_requested,a.control_file,a.session)
    if stopped():
        state['reason']='stop_requested_before_start';return 0
    import rclpy
    from nav_msgs.msg import Odometry
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros import Buffer, TransformListener
    state['ros_closed']=False
    rclpy.init(domain_id=180 if a.isolated else 176)
    n = rclpy.create_node('disarmed_ev_sensor_reader', enable_rosout=False, start_parameter_services=False)
    b = Buffer(); listener = TransformListener(b, n)
    seq = 0; source_ids = {}; ready = False; failed = False
    os.set_blocking(sys.stdout.fileno(), False)
    writer = PacketWriter(sys.stdout.fileno())
    state['transport'] = writer.stats
    topics = ({'pose': '/robocup/ev_reader_test/pose', 'tracking': '/robocup/ev_reader_test/status'}
              if a.isolated else {'pose': '/visual_slam/tracking/odometry', 'tracking': '/visual_slam/status'})
    if a.trace_image_inputs:
        topics.update({kind: (f'/robocup/ev_reader_test/{kind}' if a.isolated else
                       f'/camera/camera/{kind}/image_rect_raw') for kind in ('infra1', 'infra2')})
    def emit(kind, **data):
        nonlocal seq
        seq += 1
        writer.enqueue(dict(session=a.session, seq=seq, kind=kind,
                            reader_enqueued_monotonic_s=time.monotonic(), **data))
    def audit():
        nonlocal ready, failed
        for key, topic in topics.items():
            ends = timing.call('graph_'+key,n.get_publishers_info_by_topic,topic)
            if len(ends) != 1:
                if ready or len(ends) > 1:
                    failed = True; emit('fault', reason='sensor source missing/duplicate')
                return
            gid = tuple(ends[0].endpoint_gid)
            if key in source_ids and source_ids[key] != gid:
                failed = True; emit('fault', reason='sensor source generation changed'); return
            source_ids[key] = gid
        try:
            tr = timing.call('extrinsic_lookup',b.lookup_transform,
                             'base_link','camera_link',rclpy.time.Time()).transform
            vals = [tr.translation.x, tr.translation.y, tr.translation.z,
                    tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w]
            if any(abs(x-y) > 1e-6 for x,y in zip(vals, [.196,.025,-.05,0,0,0,1])):
                failed = True; emit('fault', reason='installation TF does not match reviewed session'); return
        except Exception:
            if ready:
                failed = True; emit('fault', reason='installation TF missing')
            return
        ready = True
    def receive(kind, m):
        received = time.monotonic()
        if not ready or failed or stopped():
            return
        stamp = m.header.stamp.sec*10**9+m.header.stamp.nanosec
        clocks=snapshot(lambda:n.get_clock().now().nanoseconds)
        if kind == 'tracking':
            emit(kind, stamp_ns=stamp, state=int(m.vo_state), reader_receipt_monotonic_s=received,
                 reader_clock=clocks)
        elif kind in ('infra1', 'infra2'):
            # Do not copy, encode, save or forward pixel data. This observer still
            # adds DDS deserialization load, which must be disclosed in analysis.
            emit(kind, stamp_ns=stamp, width=int(m.width), height=int(m.height),
                 reader_receipt_monotonic_s=received,reader_clock=clocks)
        else:
            p,q = m.pose.pose.position,m.pose.pose.orientation
            emit(kind, stamp_ns=stamp, reader_receipt_monotonic_s=received,
                 reader_clock=clocks,
                 frame=m.header.frame_id, child=m.child_frame_id,
                 position=[p.x,p.y,p.z], quaternion=[q.x,q.y,q.z,q.w])
    n.create_subscription(Odometry, topics['pose'], lambda m: receive('pose',m), qos_profile_sensor_data)
    n.create_subscription(VisualSlamStatus, topics['tracking'], lambda m: receive('tracking',m), qos_profile_sensor_data)
    if a.trace_image_inputs:
        from sensor_msgs.msg import Image
        for kind in ('infra1', 'infra2'):
            n.create_subscription(Image, topics[kind], lambda m, k=kind: receive(k, m), qos_profile_sensor_data)
    n.create_timer(.1, lambda:timing.call('audit',audit))
    try:
        while not failed and time.monotonic() < a.deadline:
            if stopped():
                state['reason']='stop_requested';break
            timing.call('spin_once',rclpy.spin_once,n,timeout_sec=.005)
            timing.call('pipe_pump',writer.pump)
    finally:
        n.destroy_node(); rclpy.try_shutdown(); state['ros_closed']=True
    if failed:state['reason']='sensor_fault'
    return 1 if failed else 0


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',required=True)
    p.add_argument('--deadline',required=True,type=float)
    p.add_argument('--control-file',required=True,type=Path)
    p.add_argument('--receipt-file',required=True,type=Path)
    p.add_argument('--isolated',action='store_true',help='synthetic domain180 only; never domain176')
    p.add_argument('--trace-image-inputs', action='store_true',
                   help='explicit read-only timing experiment: include stereo image header metadata')
    a=p.parse_args()
    if not 0<a.deadline-time.monotonic()<=50:p.error('expired or excessive active budget')
    from bench_session_deadline import DeadlineAlarm
    alarm=DeadlineAlarm(a.deadline)
    state=dict(session=a.session,pid=os.getpid(),started_at=time.monotonic(),
               ros_closed=True,reason='deadline')
    code=2
    def interrupted(signum,frame):raise KeyboardInterrupt()
    old=signal.signal(signal.SIGTERM,interrupted)
    try:
        # Callbacks only enqueue. Partial writes are retained; a full pipe never
        # blocks ROS/stop handling. Queue exhaustion explicitly fails the session.
        alarm.arm();code=run(a,state)
    except TimeoutError:state['reason']='deadline';code=0 if state['ros_closed'] else 2
    except KeyboardInterrupt:state['reason']='interrupted';code=130
    except Exception as exc:state['reason']=repr(exc);code=2
    finally:
        alarm.cancel();signal.signal(signal.SIGTERM,old)
        state.update(closed_at=time.monotonic(),exit_code=code)
        atomic_json(a.receipt_file,state)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
