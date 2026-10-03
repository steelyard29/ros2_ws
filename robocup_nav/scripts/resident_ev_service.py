"""Persistent EV-only owner. Default is describe, never implicit execution.

Explicit real execution subscribes PX4 + existing VIO and publishes EV only.
No arming, mode, trajectory, parameter or servo interfaces. No automatic restart.
Not installed/enabled as a boot service by this file.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import time
import uuid
from disarmed_ev_lifecycle import atomic_json

ROOT=Path(__file__).resolve().parents[1]


class ResidentServiceLoop:
    def __init__(self,node,inputs,spin,*,clock=time.monotonic):
        self.node,self.inputs,self.spin,self.clock=node,inputs,spin,clock
        self.stopped=False;self.reason=None
        from resident_status_binding import process_identity
        self.process_identity=process_identity(os.getpid())

    def step(self):
        if self.stopped:raise RuntimeError('resident service stopped; no restart')
        try:
            self.spin()
            if self.node.closed or self.node.core.fault:
                raise RuntimeError(self.node.core.fault or 'EV output closed')
            self.inputs.pump()
            if self.node.closed or self.node.core.fault:
                raise RuntimeError(self.node.core.fault or 'EV output closed')
        except Exception as exc:self.stop(str(exc));raise

    def stop(self,reason):
        if self.stopped:return
        self.stopped=True;self.reason=str(reason)
        # Inhibit output before asking the reader to stop; no stale queue replay.
        try:self.node.close_output()
        finally:self.inputs.stop(self.reason)

    def status(self):
        data=self.node.core.health(self.clock())
        data.update(pid=os.getpid(),updated_monotonic_s=self.clock(),
                    process_identity=self.process_identity,source_gids=getattr(self.node.core,'sources',None),
                    stopped=self.stopped,stop_reason=self.reason,
                    output_topic=self.node.output,source_domain=176,host_domain=0,
                    fusion_verified=False,flight_ready=False)
        ends=self.node.get_publishers_info_by_topic(self.node.output) if not self.stopped else []
        data['ev_publisher_gids']=[list(e.endpoint_gid) for e in ends]
        # Dispatch receipt is not PX4 EKF acceptance; never infer fusion here.
        return data


def run():
    from resident_container_inputs import ContainerResidentInputs
    from resident_ev_binding import ResidentEvOwner
    from resident_ev_node import create_node
    from px4_runtime_transport import configure
    from resident_ev_observation import ResidentObservation
    out=ROOT/'evidence'/('resident_ev_service_'+uuid.uuid4().hex);out.mkdir()
    inputs=ContainerResidentInputs(out/'reader',sensor_only=True,domain=176)
    owner=ResidentEvOwner(inputs.session)
    observation=ResidentObservation(inputs.session)
    ros=node=loop=None;initialized=False;stop_requested=False
    report=dict(session=inputs.session,started_at=time.monotonic(),flight_ready=False,
                mode='EV only',closed=False,exit_code=2)
    def stop_signal(signum,frame):
        nonlocal stop_requested
        stop_requested=True
    previous={s:signal.signal(s,stop_signal) for s in (signal.SIGTERM,signal.SIGINT)}
    try:
        owner.acquire();configure()
        import rclpy as ros
        from rclpy.signals import SignalHandlerOptions
        ros.init(args=[],domain_id=0,signal_handler_options=SignalHandlerOptions.NO);initialized=True
        node=create_node(inputs.session,perception_node=inputs.port,owner=owner,observation=observation)
        loop=ResidentServiceLoop(node,inputs,lambda:ros.spin_once(node,timeout_sec=.005))
        if stop_requested:raise RuntimeError('stop requested during startup')
        inputs.start()
        last_status=-float('inf')
        while not stop_requested:
            loop.step()
            if time.monotonic()-last_status>=.2:
                atomic_json(out/'status.json',loop.status());last_status=time.monotonic()
        report.update(reason='operator requested stop',exit_code=0)
    except Exception as exc:report['reason']=repr(exc)
    finally:
        errors=[]
        try:
            if loop:loop.stop(report.get('reason','service exit'))
            else:
                if node:node.close_output()
                inputs.stop('service startup failed')
        except Exception as exc:errors.append('inhibit/stop: '+repr(exc))
        try:
            end=time.monotonic()+3
            while True:
                cleanup=inputs.poll_close()
                if (not cleanup.get('running') and not cleanup.get('awaiting_receipt')) or time.monotonic()>=end:break
                time.sleep(.01)
            report['reader_cleanup']=cleanup
        except Exception as exc:errors.append('reader cleanup: '+repr(exc))
        if node:
            report['ev']=node.core.health(time.monotonic())
            guard=node.core.adapter.guard
            report['input_timing']=dict(tracking_at=guard.tracking_at,pose_at=guard.pose_at,
                tracking_stamp_ns=guard.tracking_stamp,pose_stamp_ns=guard.pose_stamp,
                checked_at=time.monotonic(),warmup_count=guard.count,
                telemetry_receipts=dict(node.receipts))
            try:node.destroy_node()
            except Exception as exc:errors.append('node cleanup: '+repr(exc))
        if initialized:
            try:ros.try_shutdown()
            except Exception as exc:errors.append('ROS cleanup: '+repr(exc))
        owner.close()
        for s,handler in previous.items():signal.signal(s,handler)
        cleanup=report.get('reader_cleanup',{})
        report['closed']=not errors and (cleanup.get('confirmed') or cleanup.get('process_started') is False)
        if errors or not report['closed']:report['exit_code']=2
        report.update(cleanup_errors=errors,closed_at=time.monotonic())
        report['observation']=observation.report()
        atomic_json(out/'status.json',dict(session=inputs.session,stopped=True,flight_ready=False,
            recent_dispatch=False,updated_monotonic_s=time.monotonic(),reason=report.get('reason')))
        atomic_json(out/'report.json',report)
        # Full samples remain in report.json; avoid dumping thousands of pairs
        # into journald or a terminal on shutdown.
        print(json.dumps(dict(evidence=str(out),session=inputs.session,
            reason=report.get('reason'),exit_code=report['exit_code'],closed=report['closed'],
            ev=report.get('ev'),fusion=report['observation']['fusion'],
            matched_pairs=report['observation']['matched_retained']),indent=2))
    return report['exit_code']


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-real-ev',action='store_true',
        help='start continuous real EV-only input; requires current physical authorization')
    args=parser.parse_args()
    if not args.execute_real_ev:
        print(json.dumps(dict(default='describe only',output='/fmu/in/vehicle_visual_odometry',
            automatic_restart=False,commands=False,installed=False,
            start_requirement='disarmed, landed, stationary; existing DDS and VIO')))
        return 0
    return run()


if __name__=='__main__':raise SystemExit(main())
