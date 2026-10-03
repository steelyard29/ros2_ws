"""Lease-owned VIO worker. Default describes; explicit sensor-only or isolated mode.

No camera startup, PX4 input, command publisher or automatic retry. A future
production owner must complete source/TF/session binding before real routing.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time
from disarmed_ev_lifecycle import atomic_json
from resident_reader_lifecycle import ReaderLease,clock_identity,service_loop


def configure_transport(domain):
    """Match perception transport instead of inheriting a PX4 bridge profile."""
    profile=Path(__file__).resolve().parents[1]/'config/perception_dds_shm16m.xml'
    if not profile.is_file():raise FileNotFoundError(profile)
    os.environ.update(ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1',
        RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
        FASTRTPS_DEFAULT_PROFILES_FILE=str(profile),
        FASTDDS_DEFAULT_PROFILES_FILE=str(profile))
    return str(profile)


def run(session,control,receipt,domain,deadline,*,sensor_only=False):
    if sensor_only and (domain!=176 or deadline is not None):
        raise ValueError('sensor-only worker requires domain176 and lease lifetime')
    if not sensor_only and (domain not in range(180,188) or deadline is None
                           or not 0<deadline-time.monotonic()<=25):
        raise ValueError('isolated worker domain/budget invalid')
    if Path(control).resolve()==Path(receipt).resolve():raise ValueError('control and receipt must differ')
    state=dict(session=session,pid=os.getpid(),started_at=time.monotonic(),
               ros_closed=True,reason='startup',exit_code=2,sensor_only=sensor_only,domain=domain)
    node=sender=None;initialized=False
    def interrupt(signum,frame):raise KeyboardInterrupt()
    previous=signal.signal(signal.SIGTERM,interrupt)
    try:
        lease=ReaderLease(session,clock_identity())
        if not lease.check(json.loads(Path(control).read_text())):
            state['reason']=lease.reason;return 2
        state['transport_profile']=configure_transport(domain)
        import rclpy
        from resident_vio_sender import ResidentVioSender,create_node
        os.environ['ROS_LOCALHOST_ONLY']='1'
        rclpy.init(args=[],domain_id=domain);initialized=True;state['ros_closed']=False
        sender=ResidentVioSender(sys.stdout.fileno(),session,domain=domain,isolated=not sensor_only)
        if sensor_only:
            from resident_real_source import create_sensor_node
            node=create_sensor_node(sender)
        else:node=create_node(sender,isolated=True)
        state['reason']=service_loop(lease,control,
            lambda:rclpy.spin_once(node,timeout_sec=.005),sender.pump,deadline=deadline)
        state['exit_code']=0 if state['reason'] in ('owner_requested_stop','isolated_deadline') else 2
    except KeyboardInterrupt:state.update(reason='interrupted',exit_code=130)
    except Exception as exc:state.update(reason=repr(exc),exit_code=2)
    finally:
        if sender:
            sender.stop('worker exit');state['transport']=dict(sender.writer.stats)
            state['input_timing']=getattr(sender,'input_timing',{})
        errors=[]
        if node:
            try:node.destroy_node()
            except Exception as exc:errors.append(repr(exc))
        if initialized:
            try:rclpy.try_shutdown()
            except Exception as exc:errors.append(repr(exc))
        state['ros_closed']=not errors
        if errors:state.update(cleanup_errors=errors,exit_code=2)
        signal.signal(signal.SIGTERM,previous)
        state['closed_at']=time.monotonic()
        atomic_json(receipt,state)
    return state['exit_code']


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--run-isolated',action='store_true')
    mode.add_argument('--lease-sensor-only',action='store_true',
                      help='explicit domain176 sensor subscriptions; no PX4 or ROS output')
    parser.add_argument('--session');parser.add_argument('--control',type=Path)
    parser.add_argument('--receipt',type=Path);parser.add_argument('--domain',type=int,default=183)
    parser.add_argument('--deadline',type=float)
    args=parser.parse_args()
    if not args.run_isolated and not args.lease_sensor_only:
        print(json.dumps(dict(default='describe only',real_ev_route=False,
            sensor_only_available=True,
            subscriptions=['vio','tracking'],ros_publishers=[],lease_expiry_s=1.,
            max_isolated_budget_s=25,automatic_restart=False)))
        return 0
    if not all((args.session,args.control,args.receipt)):
        parser.error('execution requires session, control and receipt')
    if args.run_isolated and args.deadline is None:parser.error('isolated execution requires deadline')
    return run(args.session,args.control,args.receipt,args.domain,args.deadline,
               sensor_only=args.lease_sensor_only)


if __name__=='__main__':raise SystemExit(main())
