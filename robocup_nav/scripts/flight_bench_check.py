#!/usr/bin/env python3
"""Bounded prop-off camera + integrated EV-only bench check.

Never arms or commands a mode/setpoint/heartbeat to PX4. Reuses (never stops)
the existing Agent. Owns only its camera supervisor and EV-only runtime.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid
from bench_session_deadline import SessionBudget, DeadlineAlarm

ROOT = Path(__file__).resolve().parents[1]


def main():
    # Same domain-0 ownership lock as the standalone shadow/EV runtime.
    # Keep the inode after release so cooperating processes cannot bypass it.
    with open('/tmp/robocup_flight_runtime_shadow.lock', 'a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('another domain-0 flight runtime owns the lock', file=sys.stderr)
            return 2
        return run_check()


def run_check():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prop-off', required=True, action='store_true')
    parser.add_argument('--params', required=True, type=Path)
    parser.add_argument('--duration', type=int, default=45)
    parser.add_argument('--session-deadline', type=float,
                        help='optional host/container shared monotonic deadline, including cleanup')
    args = parser.parse_args()
    try:
        budget = SessionBudget(args.session_deadline)
    except ValueError as exc:
        parser.error(str(exc))
    if not 20 <= args.duration <= 90:
        parser.error('duration must be 20..90 seconds')
    import yaml
    from flight_readiness import read_params, review
    from aux_switch_decoder import validate_contract, AuxSwitchDecoder
    from flight_runtime import create_node
    from flight_runtime_core import FlightRuntimeCore
    from preflight_window import ObservationWindow, bench_result_accepted, write_control
    import xrce_serial_agent
    platform = yaml.safe_load((ROOT/'config/platform.yaml').read_text())
    if platform.get('live_flight_enabled') is not False:
        parser.error('requires explicit live_flight_enabled=false')
    param_hash = hashlib.sha256(args.params.read_bytes()).hexdigest()
    params = read_params(args.params)
    validate_contract(params)
    config = review(params, platform, 'none')
    # Bench EV is allowed before physical flight certification, never after
    # silently changing those flags. All other candidate config checks apply.
    physical = {'geometry_verified', 'sensor_extrinsics_verified',
                'imu_calibration_verified', 'px4_frame_alignment_verified'}
    bad = [c['check'] for c in config['checks'] if not c['passed'] and c['check'] not in physical]
    if bad:
        parser.error('configuration mismatch: '+', '.join(bad))
    agent_pid = xrce_serial_agent.running_pid('/dev/ttyTHS1', 921600)
    if agent_pid is None:
        parser.error('existing serial Agent required; this check does not start/restart it')
    running = subprocess.check_output(['docker', 'inspect', '-f', '{{.State.Running}}', 'isaac_ros_dev'], text=True).strip()
    if running != 'true':
        parser.error('existing camera container must already be running')
    os.environ.update(ROS_DOMAIN_ID='0', ROS_LOCALHOST_ONLY='0',
                      FASTRTPS_DEFAULT_PROFILES_FILE='/home/cfly/ros2_ws/config/fastdds_bridge.xml')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data as qos
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import VehicleStatus, ManualControlSetpoint
    rclpy.init(args=[])
    wait_node = Node('robocup_bench_readiness', enable_rosout=False,
                     start_parameter_services=False, use_global_arguments=False)
    readiness = {'vio_count': 0, 'vio_at': -1., 'status_at': -1., 'disarmed': False,
                 'switch_at': -1., 'safe_switches': False, 'ever_armed': False}
    decoder = AuxSwitchDecoder(params)

    def vio(m):
        stamp = m.header.stamp.sec*10**9+m.header.stamp.nanosec
        age = (wait_node.get_clock().now().nanoseconds-stamp)/1e9
        if m.header.frame_id == 'odom' and m.child_frame_id == 'base_link' and -.05 <= age <= .3:
            readiness['vio_at'] = time.monotonic()
            readiness['vio_count'] += 1

    def status(m):
        readiness['ever_armed'] |= m.arming_state == 2
        age = (wait_node.get_clock().now().nanoseconds/1000-m.timestamp)/1e6
        if -.05 <= age <= .5:
            readiness.update(status_at=time.monotonic(), disarmed=m.arming_state == 1)

    def rc(m):
        now = time.monotonic()
        d = decoder.receive(stamp=int(m.timestamp), sample=int(m.timestamp_sample), now=now,
                            age=(wait_node.get_clock().now().nanoseconds/1000-m.timestamp)/1e6,
                            valid=bool(m.valid), source=int(m.data_source), aux1=float(m.aux1), aux2=float(m.aux2))
        if d:
            readiness.update(switch_at=now, safe_switches=d.mode_slot in range(1, 6) and d.kill_switch == 3)

    wait_node.create_subscription(Odometry, '/visual_slam/tracking/odometry', vio, qos)
    wait_node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', status, qos)
    wait_node.create_subscription(ManualControlSetpoint, '/fmu/out/manual_control_setpoint', rc, qos)
    out = ROOT/'evidence'/(time.strftime('flight_bench_check_%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:6])
    out.mkdir(parents=True, exist_ok=False)
    control = out/'camera_control.json'
    control.write_text(json.dumps({'stop': False})+'\n')
    camera = runtime = log = observer = observer_log = None
    window = ObservationWindow()
    runtime_ok = False
    report = {'flight_approved': False, 'real_control_publishers_created': 0,
              'parameter_sha256': param_hash, 'agent_pid': agent_pid,
              'physical_verification_waived_for_flight': False, 'passed': False}
    alarm = DeadlineAlarm(budget.active_end)
    try:
        budget.require_active()
        alarm.arm()
        deadline = budget.stage_end(2)
        while time.monotonic() < deadline:
            rclpy.spin_once(wait_node, timeout_sec=.05)
        def audit():
            for topic, _ in wait_node.get_topic_names_and_types():
                if topic.startswith('/fmu/in/') and wait_node.count_publishers(topic):
                    raise RuntimeError('existing flight-input publisher: '+topic)
            if readiness['ever_armed'] or decoder.fault:
                raise RuntimeError('unsafe readiness state: '+str(decoder.fault))
        audit()
        for topic in ('/visual_slam/tracking/odometry', '/camera/camera/imu', '/robocup/alignment/tracking'):
            if wait_node.count_publishers(topic):
                raise RuntimeError('existing perception owner: '+topic)
        container_control = str(control).replace('/home/cfly/ros2_ws/', '/workspaces/ros2_ws/')
        log = (out/'camera.log').open('w')
        budget.require_active()
        camera_deadline = ('' if args.session_deadline is None else
                           f' --stop-at-monotonic {budget.active_end}')
        camera = subprocess.Popen(['docker', 'exec', '-e', 'ROS_DOMAIN_ID=0', '-e', 'ROS_LOCALHOST_ONLY=1',
            '-e', 'FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/ros2_ws/config/fastdds_bridge.xml',
            'isaac_ros_dev', 'bash', '-c',
            'source /workspaces/ros2_ws/isaac_vio_debug/scripts/container_env.sh; '
            'exec python3 /workspaces/ros2_ws/robocup_nav/scripts/alignment_camera.py '
            f'--visual-only-diagnostic --duration {args.duration+60} --control-file {container_control}'
            +camera_deadline],
            stdout=log, stderr=subprocess.STDOUT)
        print('Evidence: '+str(out), flush=True)
        deadline = budget.stage_end(35)
        ready = False
        while time.monotonic() < deadline:
            rclpy.spin_once(wait_node, timeout_sec=.05)
            audit()
            now = time.monotonic()
            ready = (readiness['vio_count'] >= 5 and now-readiness['vio_at'] < .3
                     and readiness['disarmed'] and now-readiness['status_at'] < 1.5
                     and readiness['safe_switches'] and now-readiness['switch_at'] < .5)
            if ready:
                break
            if camera.poll() is not None:
                raise RuntimeError('camera supervisor exited')
        if not ready:
            raise RuntimeError('fresh VIO/disarmed/Position/kill-off readiness timeout')
        if hashlib.sha256(args.params.read_bytes()).hexdigest() != param_hash:
            raise RuntimeError('parameter export changed during startup')
        audit()
        budget.require_active()
        observer_log = (out/'observer.log').open('w')
        # Backstop only. The parent window requests the final sample while EV
        # is still publishing; the observer must not keep its own later deadline.
        write_control(out/'observer_control.json', False)
        observer = subprocess.Popen([sys.executable, str(ROOT/'scripts/preflight_observer.py'),
                                     '--duration', str(min(180, args.duration+15)),
                                     '--control-file', str(out/'observer_control.json')],
                                    stdout=observer_log, stderr=subprocess.STDOUT)
        core = FlightRuntimeCore(time.monotonic(), aux_params=params)
        runtime = create_node(core, bench_ev=True)
        # No wait-node audit after this point: the runtime audits its own single EV publisher.
        observation_started = time.monotonic()
        deadline = budget.stage_end(args.duration)
        while time.monotonic() < deadline:
            rclpy.spin_once(runtime, timeout_sec=.02)
            if runtime.armed_on_real_input or runtime.bench_gate.fault or camera.poll() is not None:
                raise RuntimeError('bench/camera stopped: '+str(runtime.bench_gate.fault))
            if observer.poll() is not None:
                raise RuntimeError('observer exited before the coordinated final sample')
        report['observation_elapsed_s'] = time.monotonic()-observation_started
        report['observation_truncated'] = report['observation_elapsed_s']+0.01 < args.duration
        window.request_final_sample(observer.poll() is None)
        write_control(out/'observer_control.json', True)
        grace = budget.stage_end(5, final=True)
        while observer.poll() is None and time.monotonic() < grace:
            rclpy.spin_once(runtime, timeout_sec=.02)
            if runtime.armed_on_real_input or runtime.bench_gate.fault or camera.poll() is not None:
                raise RuntimeError('bench/camera stopped: '+str(runtime.bench_gate.fault))
        if observer.poll() is None:
            raise RuntimeError('observer did not finish its final sample while EV was publishing')
        window.observer_finished()
        runtime_ok = (not report['observation_truncated']
                      and runtime.bench_gate.sent >= 30 and runtime.preflight_max_s >= 10
                      and not core.ev.inhibit and not core.fault)
        report['passed'] = runtime_ok
    except KeyboardInterrupt:
        report['error'] = 'operator interrupted'
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        alarm.cancel()
        if runtime:
            report['runtime'] = runtime.report()
            try:
                window.stop_ev()
            except RuntimeError as exc:
                report['window_error'] = str(exc)
                window.abort()
                report['passed'] = False
            runtime.destroy_node()  # Stop EV before ending camera; never stop Agent.
        report['readiness'] = readiness
        control.write_text(json.dumps({'stop': True})+'\n')
        if args.session_deadline is not None and observer and observer.poll() is None:
            observer.terminate()  # Read-only child; no waiting before stopping camera.
        if camera:
            try:
                camera.wait(timeout=budget.cleanup_timeout(20))
                report['camera_exit_code'] = camera.returncode
            except subprocess.TimeoutExpired:
                report['camera_cleanup_error'] = 'supervisor did not stop; inspect container, do not kill Agent'
            if camera.returncode != 0:
                report['passed'] = False
                runtime_ok = False
            elif not report.get('camera_cleanup_error'):
                try:
                    window.stop_camera()
                except RuntimeError as exc:
                    report['window_error'] = str(exc)
                    window.abort()
                    report['passed'] = False
                    runtime_ok = False
        if log:
            log.close()
        report['observation_phase'] = window.phase
        if observer:
            try:
                observer.wait(timeout=budget.cleanup_timeout(5))
            except subprocess.TimeoutExpired:
                observer.terminate()  # Own read-only child only; never the Agent.
                try:
                    observer.wait(timeout=budget.cleanup_timeout(5))
                except subprocess.TimeoutExpired:
                    observer.kill()  # This invocation's read-only child only.
                report['observer_error'] = 'observer did not finish within its collection window'
            observer_log.close()
            try:
                independent = json.loads((out/'observer.log').read_text())
                report['independent_observer'] = independent
                report['passed'] = bench_result_accepted(
                    runtime_ok=runtime_ok is True and observer.returncode == 0 and not report.get('error')
                    and not report.get('window_error') and not report.get('camera_cleanup_error'),
                    observer=independent, phase=window.phase)
            except (ValueError, OSError) as exc:
                report['observer_error'] = str(exc)
                report['passed'] = False
        wait_node.destroy_node()
        rclpy.try_shutdown()
        report['session_deadline_monotonic'] = args.session_deadline
        report['session_elapsed_s'] = time.monotonic()-budget.started
        report['session_deadline_met'] = (args.session_deadline is None
                                         or time.monotonic() <= args.session_deadline)
        if not report['session_deadline_met']:
            report['passed'] = False
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({'evidence': str(out), **report}, indent=2), flush=True)
    return 0 if report['passed'] and not report.get('error') else 1


if __name__ == '__main__':
    raise SystemExit(main())
