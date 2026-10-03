#!/usr/bin/env python3
"""Explicitly authorized prop-off mode handshake. NEVER arms or takes off.

Writes only EV, ground-hold heartbeat/setpoint and one OFFBOARD request.
Requires an existing camera/Agent, Position baseline and independent RC kill.
Does not start/stop camera, Agent, GPIO or change parameters. Not run by tests.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]


def restored_position(core, now):
    from flight_supervisor import FlightSupervisor
    s = core.sample
    return (FlightSupervisor.fresh(s.status_at, now, 1.5) and not s.armed
            and core.gcs_connected and FlightSupervisor.fresh(core.gcs_at, now, 1.5)
            and s.nav_state == 2 and FlightSupervisor.fresh(s.land_at, now, 2.) and s.landed
            and core.switch_fresh(now) and core.mode_slot in range(1, 6) and core.kill_switch == 3)


def main(*, fault_bench=False):
    parser = argparse.ArgumentParser(description=(
        'Prop-off Position kill-stop qualification; forbids all commands; no flight approval'
        if fault_bench else __doc__))
    parser.add_argument('--prop-off', action='store_true', required=True)
    parser.add_argument('--authorize-fault-bench' if fault_bench else '--authorize-mode-handshake',
                        action='store_true', required=True,
                        help='explicit authorization for this prop-off bench ONLY; never arm')
    if fault_bench:
        parser.add_argument('--scenario', choices=('kill',), required=True)
    parser.add_argument('--confirm-component-191-exclusive', action='store_true', required=True,
                        help='operator confirms no other command sender uses system 1/component 191')
    parser.add_argument('--params', type=Path, required=True)
    parser.add_argument('--duration', type=int, default=60)
    parser.add_argument('--switch-wait-seconds', type=int, choices=(20, 60), default=20,
                        help='bounded operator response window; 60 requires duration >=80')
    args = parser.parse_args()
    args.fault_bench = fault_bench
    if not 30 <= args.duration <= 90:
        parser.error('duration must be 30..90 seconds')
    if args.duration < args.switch_wait_seconds+20:
        parser.error('duration must leave at least 20 seconds outside the switch window')
    import yaml
    from flight_readiness import read_params, review
    from aux_switch_decoder import validate_contract
    from handshake_dispatch import validate_target
    platform = yaml.safe_load((ROOT/'config/platform.yaml').read_text())
    if platform.get('live_flight_enabled') is not False:
        parser.error('requires live_flight_enabled=false; not a flight entrypoint')
    param_hash = hashlib.sha256(args.params.read_bytes()).hexdigest()
    params = read_params(args.params)
    validate_contract(params)
    validate_target(params)
    physical = {'geometry_verified', 'sensor_extrinsics_verified',
                'imu_calibration_verified', 'px4_frame_alignment_verified'}
    bad = [c['check'] for c in review(params, platform, 'none')['checks']
           if not c['passed'] and c['check'] not in physical]
    if bad:
        parser.error('parameter/platform contract mismatch: '+', '.join(bad))
    with open('/tmp/robocup_flight_runtime_shadow.lock', 'a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error('another domain-0 runtime holds the lock')
        return run(args, params, param_hash)


def run(args, params, param_hash):
    os.environ.update(ROS_DOMAIN_ID='0', ROS_LOCALHOST_ONLY='0',
                      FASTRTPS_DEFAULT_PROFILES_FILE='/home/cfly/ros2_ws/config/fastdds_bridge.xml')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data as qos
    from px4_msgs.msg import VehicleStatus, ManualControlSetpoint
    from flight_runtime import create_node
    from handshake_runtime_core import HandshakeRuntimeCore
    from flight_supervisor import FlightSupervisor
    fault_bench = getattr(args, 'fault_bench', False)
    prefix = 'fault_bench_' if fault_bench else 'handshake_bench_'
    out = ROOT/'evidence'/(time.strftime(prefix+'%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:6])
    out.mkdir(parents=True, exist_ok=False)
    report = {'flight_approved': False, 'passed': False, 'parameter_sha256': param_hash,
              'physical_verification_waived_for_flight': False, 'restore_confirmed': False,
              'command_identity': {'system': 1, 'component': 191},
              'operator_authorized_mode_only': not fault_bench,
              'operator_authorized_kill_bench': fault_bench}
    rclpy.init(args=[])
    node = runtime = core = None
    try:
        # Subscription-only check before creating any actual input publisher.
        probe = HandshakeRuntimeCore(time.monotonic(), aux_params=params, staged=True, require_gcs=True)
        node = Node('robocup_handshake_precheck', enable_rosout=False,
                    start_parameter_services=False, use_global_arguments=False)
        def status(m):
            now = time.monotonic()
            age = (node.get_clock().now().nanoseconds/1000-m.timestamp)/1e6
            if probe.receive('status', int(m.timestamp), now, age):
                probe.gcs_status(not bool(m.gcs_connection_lost), now)
                if int(m.arming_state) != 1:
                    probe.trip('precheck requires explicit disarmed state')
                probe.status(int(m.arming_state), now, nav_state=int(m.nav_state),
                             preflight_ok=bool(m.pre_flight_checks_pass), failsafe=bool(m.failsafe))
        def rc(m):
            probe.aux_message(int(m.timestamp), int(m.timestamp_sample), time.monotonic(),
                (node.get_clock().now().nanoseconds/1000-m.timestamp)/1e6,
                bool(m.valid), int(m.data_source), float(m.aux1), float(m.aux2))
        node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', status, qos)
        node.create_subscription(ManualControlSetpoint, '/fmu/out/manual_control_setpoint', rc, qos)
        ready_since = None
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            for topic, _ in node.get_topic_names_and_types():
                if topic.startswith('/fmu/in/') and node.count_publishers(topic):
                    raise RuntimeError('existing real input publisher: '+topic)
            if probe.sample.armed or probe.fault or probe.kill_latched:
                raise RuntimeError('unsafe precheck state')
            s = probe.sample
            ready = (FlightSupervisor.fresh(s.status_at, now, 1.5) and not s.armed
                and probe.gcs_connected
                and s.nav_state == 2 and not s.failsafe and probe.switch_fresh(now)
                and probe.mode_slot in range(1, 6) and probe.kill_switch == 3
                and node.count_publishers('/visual_slam/tracking/odometry') == 1
                and node.count_publishers('/robocup/alignment/tracking') == 1)
            ready_since = (now if ready_since is None else ready_since) if ready else None
            if ready_since is not None and now-ready_since >= 1:
                break
        else:
            raise RuntimeError('need connected GCS, stable disarmed Position/kill-off and existing camera; precheck timeout')
        if hashlib.sha256(args.params.read_bytes()).hexdigest() != param_hash:
            raise RuntimeError('parameter export changed during review')
        node.destroy_node()
        node = None
        if fault_bench:
            from fault_bench_core import KillBenchCore
            core = KillBenchCore(time.monotonic(), params, args.switch_wait_seconds)
        else:
            core = HandshakeRuntimeCore(time.monotonic(), aux_params=params, staged=True, require_gcs=True,
                                        switch_wait_s=args.switch_wait_seconds)
        runtime = create_node(core, exercise=True, handshake_bench=True)
        print('Evidence: '+str(out), flush=True)
        deadline = time.monotonic()+args.duration
        restored_since = None
        while time.monotonic() < deadline:
            rclpy.spin_once(runtime, timeout_sec=.02)
            if runtime.armed_on_real_input:
                raise RuntimeError('unexpected armed status; outputs stopped, use operator safety procedure')
            if core.handshake.state in core.handshake.TERMINAL:
                now = time.monotonic()
                restored = restored_position(core, now)
                if fault_bench:
                    # Wait for output silence BEFORE requesting kill clear. This
                    # never clears anything itself and never resumes the core.
                    if core.clear_ready(now) and not core.clear_prompted:
                        core.clear_prompted = True
                        print(json.dumps({'operator_action':
                            'outputs stopped; restore Position, then clear kill; keep disarmed/props removed',
                            'fault_bench_failure': core.failure}), flush=True)
                    if core.observe_restore(now, restored, runtime.dispatch_guard.fault):
                        report['restore_confirmed'] = True
                        break
                    continue
                restored_since = (now if restored_since is None else restored_since) if restored else None
                if restored_since is not None and now-restored_since >= 1:
                    report['restore_confirmed'] = True
                    break
        report['passed'] = (core.handshake.state == 'DONE' and report['restore_confirmed']
                            and not core.fault and not core.command_fault and not runtime.dispatch_guard.fault)
        if fault_bench:
            report['fault_qualification'] = core.qualification(time.monotonic())
            report['passed'] = report['fault_qualification']['passed'] and report['restore_confirmed']
    except KeyboardInterrupt:
        report['error'] = 'operator interrupted; restore Position manually, keep disarmed'
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        if core:
            core.ev_stopped = True
            if fault_bench:
                report['fault_qualification'] = core.qualification(time.monotonic())
        if report.get('error'):
            report['passed'] = False
        if runtime:
            report['runtime'] = runtime.report()
            runtime.destroy_node()
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({'evidence': str(out), **report}, indent=2), flush=True)
    return 0 if report['passed'] and not report.get('error') else 1


if __name__ == '__main__':
    raise SystemExit(main())
