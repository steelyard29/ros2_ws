#!/usr/bin/env python3
"""Stationary disarmed EV-only commissioning candidate; default is describe.

No Agent start, sensor restart, mode/arm/servo/setpoint/parameter APIs.
Requires an ALREADY connected Agent; never kills an existing Agent.
Hardware execution requires a fresh explicit operator-authorized invocation.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
from ev_clock_evidence import snapshot

ROOT = Path(__file__).resolve().parents[1]
EV_TOPIC = '/fmu/in/vehicle_visual_odometry'
DESCRIPTION = dict(purpose='stationary disarmed EV only', default='no ROS/device access',
    real_output_allowlist=[EV_TOPIC], active_limit_s=50, supervisor_limit_s=55,
    authorization_limit_s=60, starts_agent=False, starts_sensors=False,
    requires='existing Agent, fresh RC/Position/landed/disarmed, unique sources',
    physical_validation=False)


def wait_px4_graph(deadline, *, clock=time.monotonic, probe=None):
    """Pre-output graph only; no publisher/subscription, no freshness claim.

    Cold XRCE sessions may exist before their topics. This wait shares the
    original overall budget; the later 2s ownership and data guards stay intact.
    """
    import rclpy
    owned=probe is None
    if owned:
        probe=rclpy.create_node('disarmed_ev_preoutput_discovery',enable_rosout=False,
                               start_parameter_services=False,use_global_arguments=False)
    topics=['/fmu/out/'+name for name in ('vehicle_status_v1','estimator_status_flags',
            'vehicle_land_detected','manual_control_setpoint','vehicle_local_position')]
    report=dict(state='waiting',history=[],publisher_created=False)
    try:
        while clock()<deadline:
            rclpy.spin_once(probe,timeout_sec=min(.05,max(0.,deadline-clock())))
            now=clock()
            # Deadline wins even if discovery arrives on the boundary.
            if now>=deadline:break
            counts={t:probe.count_publishers(t) for t in topics}
            writers={t:probe.count_publishers(t) for t,_ in probe.get_topic_names_and_types()
                     if t.startswith('/fmu/in/') and probe.count_publishers(t)>0}
            missing=[t for t,n in counts.items() if n==0]
            duplicates=[t for t,n in counts.items() if n>1]
            report['history'].append(dict(at=now,counts=counts,competing=writers))
            if writers or duplicates:
                report.update(state='conflict',missing=missing,duplicates=duplicates);return report
            if not missing:
                report.update(state='ready',ready_at=now);return report
        report.update(state='timeout',deadline=deadline);return report
    finally:
        if owned:probe.destroy_node()


def create_node(session, *, isolated=False, defer_output=False):
    """One publisher only. Isolated testing uses a non-flight topic namespace."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data as qos
    from px4_msgs.msg import (VehicleOdometry, VehicleStatus, VehicleLocalPosition,
                             EstimatorStatusFlags, VehicleLandDetected, ManualControlSetpoint)
    from ev_odometry import apply_to_vehicle_odometry
    from disarmed_ev_lifecycle import discovery_state
    prefix = '/robocup/disarmed_ev_test' if isolated else '/fmu'
    class Observer(Node):
        def __init__(self):
            super().__init__('disarmed_ev_only', enable_rosout=False, start_parameter_services=False,
                             use_global_arguments=False)
            self.ev_topic = prefix+'/in/vehicle_visual_odometry'
            self.pub = None
            self.activated = False
            self.sources = {}; self.gids = {}; self.last_audit = -1.
            self.paired = []; self.fusion = []; self.output_count = 0
            self.pose_pairs = []; self.pair_attempts = 0
            self.fusion_samples=[];self.fusion_samples_truncated=False
            self.discovery_history=[];self.discovery_ready=False
            if not defer_output:self.activate(session)
        def activate(self, active_session):
            nonlocal session
            if self.activated or active_session is None:
                raise RuntimeError('EV observer activation requires a session and is single-use')
            session=active_session
            self.activated=True
            self.pub = self.create_publisher(VehicleOdometry, self.ev_topic, qos)
            specs = [(VehicleStatus,'vehicle_status_v1','status',self.status),
                (EstimatorStatusFlags,'estimator_status_flags','flags',self.flags),
                (VehicleLandDetected,'vehicle_land_detected','land',lambda m: session.core.land(time.monotonic(),bool(m.landed))),
                (ManualControlSetpoint,'manual_control_setpoint','rc',self.rc),
                (VehicleLocalPosition,'vehicle_local_position','local',self.local)]
            for cls, topic, key, cb in specs:
                self.sources[key] = prefix+'/out/'+topic
                self.create_subscription(cls,self.sources[key],self.wrap(key,cb),qos)
        def wrap(self,key,cb):
            def receive(m):
                if key == 'status' and int(m.arming_state) != 1:
                    session.stop('not explicitly disarmed'); return
                stamp = int(m.timestamp)
                age = (self.get_clock().now().nanoseconds-stamp*1000)/1e9
                session.telemetry(key,stamp,age,time.monotonic(),lambda:cb(m))
            return receive
        def status(self,m):
            if m.gcs_connection_lost:
                session.stop('GCS connection lost'); return
            session.core.status(int(m.arming_state),time.monotonic(),nav_state=int(m.nav_state),
                preflight_ok=bool(m.pre_flight_checks_pass),failsafe=bool(m.failsafe),
                manual_takeover=bool(m.failsafe_and_user_took_over))
        def flags(self,m):
            now=time.monotonic()
            row=dict(ev_position=bool(m.cs_ev_pos),ev_yaw=bool(m.cs_ev_yaw),
                ev_height=bool(m.cs_ev_hgt),ev_velocity=bool(m.cs_ev_vel),
                baro_height=bool(m.cs_baro_hgt),range_height=bool(m.cs_rng_hgt))
            # This callback is invoked only after telemetry source validation.
            # Keep EVERY accepted observation, not only flag changes; no output policy change.
            if len(self.fusion_samples)<400:
                self.fusion_samples.append(dict(at=now,stamp_us=int(m.timestamp),
                    source_age_s=(self.get_clock().now().nanoseconds-int(m.timestamp)*1000)/1e9,**row))
            else:self.fusion_samples_truncated=True
            if not self.fusion or any(self.fusion[-1][k]!=v for k,v in row.items()):
                if len(self.fusion)<400:
                    self.fusion.append(dict(at=now,**row))
            session.core.flags(now,**row)
        def rc(self,m):
            now=time.monotonic()
            session.core.aux_message(int(m.timestamp),int(m.timestamp_sample),now,
                (self.get_clock().now().nanoseconds-m.timestamp*1000)/1e9,
                bool(m.valid),int(m.data_source),float(m.aux1),float(m.aux2))
        def local(self,m):
            self.paired.append(dict(at=time.monotonic(),stamp=int(m.timestamp),
                position=[float(m.x),float(m.y),float(m.z)],heading=float(m.heading),
                xy_valid=bool(m.xy_valid),z_valid=bool(m.z_valid),
                armed=session.core.sample.armed,landed=session.core.sample.landed,
                reset_all=[int(getattr(m,k)) for k in ('xy_reset_counter','z_reset_counter',
                    'heading_reset_counter','vxy_reset_counter','vz_reset_counter')],
                reset=[int(m.xy_reset_counter),int(m.z_reset_counter),int(m.heading_reset_counter)]))
            if len(self.paired)>2000:self.paired.pop(0)
        def audit(self):
            now=time.monotonic()
            if now-self.last_audit<.1:return
            self.last_audit=now
            present={t:self.count_publishers(t) for t,_ in self.get_topic_names_and_types()
                     if t.startswith(prefix+'/in/') and self.count_publishers(t)}
            counts={};changed=[];identities={}
            for key,topic in self.sources.items():
                ends=self.get_publishers_info_by_topic(topic)
                counts[topic]=len(ends)
                identities[topic]=[dict(node=e.node_name,namespace=e.node_namespace,
                                       gid=list(e.endpoint_gid)) for e in ends]
                if len(ends)!=1:continue
                gid=tuple(ends[0].endpoint_gid)
                if key in self.gids and self.gids[key]!=gid:
                    changed.append(topic)
                self.gids[key]=gid
            d=discovery_state(present,self.ev_topic,counts,changed,now-session.started,self.discovery_ready)
            d['source_identities']=identities
            self.discovery_history.append(d)
            if d['state']=='ready':
                self.discovery_ready=True;session.core.ownership(now,True)
            elif d['state']!='discovering':
                session.stop(d['state']+': '+d['reason'])
        def dispatch(self,data):
            # Audit first. The sample clock must not precede the ownership stamp,
            # or the 0.5 s exclusive-output window fails closed on a fresh audit.
            self.audit()
            now=time.monotonic()
            data['host_clock']=snapshot(lambda:self.get_clock().now().nanoseconds)
            ev=session.packet(data,now,data['host_clock']['ros_ns'])
            # Observation only: reuse the task's unchanged pairing criteria while
            # EV is active. Never initialize a mission or alter output eligibility.
            if ev is not None and data.get('kind') == 'pose' and self.paired:
                from types import SimpleNamespace
                from task_pose_pairing import closest_local
                self.pair_attempts += 1
                history=[(r['stamp']*1000,SimpleNamespace(local_at=r['at'],
                    xy_valid=r['xy_valid'],z_valid=r['z_valid'],armed=r['armed'],
                    landed=r['landed'],reset_counters=r['reset_all'])) for r in self.paired[-128:]]
                pair=closest_local(data['stamp_ns'],history,self.get_clock().now().nanoseconds,
                                   now,self.paired[-1]['reset_all'])
                if pair and len(self.pose_pairs)<2000:
                    row=next(r for r in reversed(self.paired) if r['stamp']*1000==pair[0])
                    self.pose_pairs.append(dict(at=now,vio_stamp_ns=data['stamp_ns'],
                        px4_stamp_ns=pair[0],difference_ms=abs(pair[0]-data['stamp_ns'])/1e6,
                        vio_position=data['position'],vio_quaternion=data['quaternion'],px4=row))
            if ev is not None and not session.fault and session.watchdog(now):
                msg=VehicleOdometry();apply_to_vehicle_odometry(msg,ev)
                self.pub.publish(msg);session.dispatched();self.output_count+=1
        def close_output(self):
            if self.pub is not None:self.destroy_publisher(self.pub);self.pub=None
    return Observer()


def worker(deadline, session_id, out):
    from bench_session_deadline import DeadlineAlarm
    alarm=DeadlineAlarm(deadline);alarm.arm()
    import yaml
    import rclpy
    from disarmed_ev_contract import DisarmedEvSession
    from flight_readiness import read_params,review
    from aux_switch_decoder import validate_contract
    from xrce_serial_agent import running_pid
    from disarmed_ev_lifecycle import atomic_json, stop_reader
    node=reader=log=None
    packets=[]
    report=dict(passed=False,flight_ready=False,hardware_attempted=False,session=session_id,
        pass_scope='EV delivery only; paired data requires separate alignment/fusion analysis; no control handoff')
    started=time.monotonic()
    gate=None
    control=out/'reader_control.json';receipt=out/'reader_closed.json'
    try:
        platform=yaml.safe_load((ROOT/'config/platform.yaml').read_text())
        if platform.get('live_flight_enabled') is not False:raise RuntimeError('live flight permit must remain false')
        params=read_params(Path('/home/cfly/param.params.txt'));validate_contract(params)
        physical={'geometry_verified','sensor_extrinsics_verified','imu_calibration_verified','px4_frame_alignment_verified'}
        bad=[c['check'] for c in review(params,platform,'none')['checks'] if not c['passed'] and c['check'] not in physical]
        if bad:raise RuntimeError('parameter snapshot mismatch: '+str(bad))
        if running_pid('/dev/ttyTHS1',921600) is None:raise RuntimeError('existing Telem1 Agent required; none started here')
        os.environ.update(ROS_DOMAIN_ID='0',ROS_LOCALHOST_ONLY='0',
            FASTRTPS_DEFAULT_PROFILES_FILE='/home/cfly/ros2_ws/config/fastdds_bridge.xml')
        rclpy.init(args=[])
        # deadline is fixed at the invocation's start+50. Allow only the first
        # 10s of that SAME invocation for cold discovery (including Agent start).
        # Retain the actual node: destroying a temporary probe discards its
        # discovery context and makes the Observer repeat cold discovery.
        # The dormant Observer has no subscriptions or flight publisher.
        node=create_node(None,defer_output=True)
        report['preoutput_discovery']=wait_px4_graph(deadline-40.,probe=node)
        if report['preoutput_discovery']['state']!='ready':
            raise RuntimeError('pre-output DDS graph '+report['preoutput_discovery']['state'])
        gate=DisarmedEvSession(time.monotonic(),deadline,session_id,params)
        node.activate(gate)
        report['discovery_handoff']='same_node_deferred_endpoints'
        report['hardware_attempted']=True
        log=(out/'reader.stderr.log').open('w')
        atomic_json(control,dict(session=session_id,stop=False))
        container_control=str(control).replace('/home/cfly/ros2_ws/','/workspaces/ros2_ws/')
        container_receipt=str(receipt).replace('/home/cfly/ros2_ws/','/workspaces/ros2_ws/')
        cmd=['docker','exec','-e','ROS_LOCALHOST_ONLY=1','isaac_ros_dev','bash','-c',
            'source /workspaces/ros2_ws/isaac_vio_debug/scripts/container_env.sh; '
            'exec python3 /workspaces/ros2_ws/robocup_nav/scripts/disarmed_ev_sensor_reader.py '
            f'--session {session_id} --deadline {deadline} '
            f'--control-file {container_control} --receipt-file {container_receipt}']
        reader=subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=log)
        os.set_blocking(reader.stdout.fileno(),False)
        buffer=b''
        # Leave one second before the independent alarm for a normal final report.
        while time.monotonic()<deadline-1 and not gate.fault:
            rclpy.spin_once(node,timeout_sec=.005)
            chunk=os.read(reader.stdout.fileno(),65536) if __import__('select').select([reader.stdout],[],[],0)[0] else None
            if chunk==b'':gate.stop('sensor reader ended');break
            if chunk:
                buffer+=chunk
                if len(buffer)>131072:gate.stop('sensor pipe backlog');break
                while b'\n' in buffer and not gate.fault:
                    line,buffer=buffer.split(b'\n',1)
                    data=json.loads(line)
                    data['host_receipt_monotonic_s']=time.monotonic()
                    packets.append(data);node.dispatch(data)
            node.audit()
            chunk=os.read(reader.stdout.fileno(),65536) if __import__('select').select([reader.stdout],[],[],0)[0] else None
            if chunk==b'':gate.stop('sensor reader ended');break
            if chunk:
                buffer+=chunk
                if len(buffer)>131072:gate.stop('sensor pipe backlog');break
                while b'\n' in buffer and not gate.fault:
                    line,buffer=buffer.split(b'\n',1)
                    data=json.loads(line)
                    data['host_receipt_monotonic_s']=time.monotonic()
                    packets.append(data);node.dispatch(data)
            if not gate.fault:
                gate.watchdog(time.monotonic())
        report.update(sent=gate.sent,fault=gate.fault,paired_px4=node.paired,
                      fusion_flags=node.fusion,sensor_packets=packets,
                      elapsed_active_s=time.monotonic()-started)
        # Data delivery is not alignment, EKF fusion, control handoff or flight acceptance.
        report['passed']=gate.sent>=30 and gate.fault in (None,'active deadline or invalid clock')
    except Exception as exc:report['error']=repr(exc)
    finally:
        # EV output is destroyed before any subprocess cleanup.
        if gate:
            report.update(sent=gate.sent,fault=gate.fault,sensor_packets=packets)
            report['telemetry_timing'] = gate.timing_snapshot(time.monotonic())
            report['telemetry_receipt_fault'] = gate.receipt_fault
            report['vision_trace'] = list(gate.vision_trace)
            report['vision_guard_at_exit'] = dict(pose_at=gate.core.ev.guard.pose_at,
                tracking_at=gate.core.ev.guard.tracking_at,
                pose_stamp=gate.core.ev.guard.pose_stamp,
                tracking_stamp=gate.core.ev.guard.tracking_stamp)
        if node:
            node.close_output()
            report['paired_px4']=node.paired
            report['output_count']=node.output_count
            report['synchronous_pairing']=dict(attempts=node.pair_attempts,
                matched=len(node.pose_pairs),pairs=node.pose_pairs,
                scope='same-session stationary timestamp pairing only; no flight release')
            report['discovery_history']=node.discovery_history
            from disarmed_ev_fusion_evidence import summarize
            from disarmed_ev_contract import RECEIPT_LIMITS
            report['fusion_samples']=node.fusion_samples
            report['fusion_observation']=summarize(node.fusion_samples,
                max_receipt_gap_s=RECEIPT_LIMITS['flags'],truncated=node.fusion_samples_truncated)
        if reader:
            try:
                cleanup=stop_reader(reader,control,receipt,session_id)
                report['reader_cleanup']=cleanup
                if not cleanup['confirmed']:
                    report['reader_cleanup_unconfirmed']=True;report['passed']=False
            except Exception as exc:
                report['reader_cleanup_error']=repr(exc);report['passed']=False
        if node:node.destroy_node()
        try:rclpy.try_shutdown()
        except Exception:pass
        alarm.cancel()
        if reader and reader.poll() is None:
            # Failure is already recorded; terminating the client is NOT proof
            # of container exit. Its stop file/deadline remains active.
            reader.terminate()
            try:reader.wait(timeout=.5)
            except subprocess.TimeoutExpired:reader.kill();reader.wait(timeout=.5)
        if reader:report['reader_exit_code']=reader.returncode
        if log:log.close()
        report['elapsed_worker_s']=time.monotonic()-started
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    return 0 if report['passed'] else 1


def supervise(pid, deadline):
    """Wait only for this owned process group; kill on timeout/parent interrupt."""
    result=None
    try:
        while time.monotonic()<deadline:
            got,status=os.waitpid(pid,os.WNOHANG)
            if got:
                result=os.waitstatus_to_exitcode(status);break
            time.sleep(.02)
    finally:
        if result is None:
            try:os.killpg(pid,signal.SIGKILL)
            except ProcessLookupError:
                try:os.kill(pid,signal.SIGKILL)
                except ProcessLookupError:pass
            os.waitpid(pid,0);result=2
    return result


def main():
    started=time.monotonic()
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute-ev-only',action='store_true')
    p.add_argument('--confirm-props-installed-disarmed',action='store_true')
    p.add_argument('--start-and-keep-agent',action='store_true',
                   help='explicitly authorized Agent recovery within the SAME total budget; retain Agent afterwards')
    args=p.parse_args()
    if not args.execute_ev_only:
        print(json.dumps(DESCRIPTION,indent=2));return 0
    if not args.confirm_props_installed_disarmed:p.error('explicit installed-props/disarmed confirmation required')
    # Exclusive lock shared with old bench/flight runtime; held by parent throughout.
    with open('/tmp/robocup_flight_runtime_shadow.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        sid=uuid.uuid4().hex
        out=ROOT/'evidence'/('disarmed_ev_'+sid);out.mkdir()
        if args.start_and_keep_agent:
            from bench_session_deadline import DeadlineAlarm
            from xrce_serial_agent import ensure
            alarm=DeadlineAlarm(started+8)
            try:
                alarm.arm()
                info=ensure('/dev/ttyTHS1',921600)
                (out/'agent.json').write_text(json.dumps(dict(**info,retained=True,
                    elapsed_s=time.monotonic()-started))+'\n')
            except Exception as exc:
                (out/'supervisor.json').write_text(json.dumps(dict(exit_code=2,
                    error=repr(exc),elapsed_s=time.monotonic()-started,ev_worker_started=False))+'\n')
                print(str(out));return 2
            finally:alarm.cancel()
        # forked child has a separate process group and only EV output APIs.
        pid=os.fork()
        if pid==0:
            os.setsid()
            try:code=worker(started+50,sid,out)
            except BaseException:code=2
            os._exit(code)
        result=supervise(pid,started+55)
        (out/'supervisor.json').write_text(json.dumps(dict(elapsed_s=time.monotonic()-started,
            exit_code=result,hard_limit_s=55,authorization_limit_s=60,session=sid))+'\n')
        print(str(out));return result


if __name__=='__main__':
    raise SystemExit(main())
