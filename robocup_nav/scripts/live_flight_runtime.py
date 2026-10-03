#!/usr/bin/env python3
"""EXPLICIT real, no-payload, single takeoff/hold/land entrypoint.

Requires completed operator release and interactive authorization. Never run
by automated tests. Does not start camera/Agent or write aircraft parameters.
Ctrl-C/SIGTERM requests bounded abort/landing; kill/takeover always wins.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import select
import signal
import sys
import time

from flight_release import ROOT, validate


def pump_task_inputs(perception, runtime):
    """Contain task transport failures while PX4 callbacks/ticks remain alive."""
    if runtime.task_transport_fault is None:
        try:perception.pump()
        except Exception as exc:
            runtime.task_transport_failed('task transport failed: '+str(exc))
    if runtime.task_transport_fault is not None:
        # A prepare/publish failure may also have been caught inside runtime.tick.
        perception.quarantine(runtime.task_transport_fault)


def confirm(session_id):
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        raise ValueError('real flight requires an interactive local operator terminal')
    phrase = 'AUTHORIZE FLIGHT '+session_id
    print('REAL FLIGHT: props may turn after the READY switch edge.\n'
          'Keep Position, kill OFF, disarmed. Confirm the cleared area and RC operator.\n'
          'Within 30 seconds type exactly: '+phrase, flush=True)
    if not select.select([sys.stdin], [], [], 30)[0] or sys.stdin.readline().strip() != phrase:
        raise ValueError('operator authorization missing or expired')


def precheck(rclpy, params, interrupted,perception=None,resident_binding=None):
    """Subscribe only before any real input publisher is constructed."""
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data as qos
    from px4_msgs.msg import VehicleStatus, ManualControlSetpoint
    from live_flight_core import LiveFlightCore
    node = Node('robocup_live_readonly_precheck', use_global_arguments=False,
                enable_rosout=False, start_parameter_services=False)
    probe = LiveFlightCore(time.monotonic(), params)
    def status(m):
        now = time.monotonic()
        age = (node.get_clock().now().nanoseconds/1000-m.timestamp)/1e6
        if probe.receive('status', int(m.timestamp), now, age):
            probe.gcs_status(not m.gcs_connection_lost, now)
            probe.status(int(m.arming_state), now, nav_state=int(m.nav_state),
                         preflight_ok=bool(m.pre_flight_checks_pass), failsafe=bool(m.failsafe))
    def rc(m):
        probe.aux_message(int(m.timestamp), int(m.timestamp_sample), time.monotonic(),
            (node.get_clock().now().nanoseconds/1000-m.timestamp)/1e6,
            bool(m.valid), int(m.data_source), float(m.aux1), float(m.aux2))
    node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', status, qos)
    node.create_subscription(ManualControlSetpoint, '/fmu/out/manual_control_setpoint', rc, qos)
    since = None
    deadline = time.monotonic()+10
    try:
        while time.monotonic() < deadline:
            if interrupted():
                raise ValueError('operator interrupted before flight publishers')
            if perception:perception.pump()
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            resident_ready=True
            if resident_binding is not None:
                resident_ready=resident_binding.verify_graph(node,perception.port if perception else node,
                                                              allow_missing=True)
            for topic, _ in node.get_topic_names_and_types():
                if topic.startswith('/fmu/in/') and node.count_publishers(topic):
                    if (resident_binding is not None and topic=='/fmu/in/vehicle_visual_odometry'
                            and node.count_publishers(topic)==1):continue
                    raise ValueError('another real input publisher exists: '+topic)
            if probe.fault or probe.kill_latched or probe.sample.armed:
                raise ValueError('unsafe read-only precheck: '+str(probe.fault))
            ready = (resident_ready and probe.controller.fresh(probe.sample.status_at, now, 1.5)
                and probe.sample.nav_state == 2 and not probe.sample.failsafe
                and probe.gcs_connected and probe.switch_fresh(now)
                and probe.mode_slot in range(1, 6) and probe.kill_switch == 3
                and (perception.port if perception else node).count_publishers('/visual_slam/tracking/odometry') == 1
                and (perception.port if perception else node).count_publishers('/robocup/alignment/tracking') == 1)
            since = (now if since is None else since) if ready else None
            if since is not None and now-since >= 1:
                return
        raise ValueError('precheck timed out; need fresh disarmed Position/RC/GCS and camera inputs')
    finally:
        node.destroy_node()


def run(permit, params, *, task=False,resident_binding=None):
    if resident_binding is not None and not task:
        raise ValueError('resident binding requires dedicated task runtime')
    from px4_runtime_transport import configure
    configure()  # ContainerTaskInputs owns perception in a different process.
    import rclpy
    from flight_runtime import create_node
    from live_flight_core import LiveFlightCore
    report = {'session_id': permit.session_id, 'passed': False, 'flight_approved': False,
              'operator_release_sha256': permit.release_hash,
              'landing_confirmed': False, 'software_simulation': False}
    out = ROOT/'evidence'/('live_flight_'+permit.session_id)
    out.mkdir(parents=True, exist_ok=False)
    stop = {'requested': False}
    def interrupt(signum, frame):
        stop['requested'] = True
    previous = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    runtime = core = perception = None
    initialized = False
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        if task:
            from task_perception_pipe import ContainerTaskInputs
            from task_pipe_authority import SORTIE_SECONDS
            perception=ContainerTaskInputs(out,time.monotonic()+SORTIE_SECONDS,
                                           navigation_writes=True,live_permit=permit)
            perception.start(discovery_only=True)
        precheck(rclpy, params, lambda: stop['requested'],perception=perception,
                 resident_binding=resident_binding)
        permit.before_arm()
        if task:
            from task_flight_controller import TaskFlightCore
            from task_flight_release import TaskFlightPermit
            if type(permit) is not TaskFlightPermit:
                raise ValueError('task runtime requires dedicated task permit')
            core = TaskFlightCore(time.monotonic(), params, permit.task_config, permit.hover,
                                  abort_requested=lambda: stop['requested'])
        else:
            core = LiveFlightCore(time.monotonic(), params, permit.height, permit.hover,
                                  abort_requested=lambda: stop['requested'])
        if resident_binding is not None:
            resident_binding.check()
            core.bind_external_ev(resident_binding.evidence)
        runtime = create_node(core, exercise=True, live_permit=permit,
                              perception_node=perception.port if perception else None,
                              resident_binding=resident_binding)
        if perception:
            from task_pipe_authority import EXECUTIVE_SECONDS
            if perception.deadline-time.monotonic()<EXECUTIVE_SECONDS+3:
                raise ValueError('startup exhausted single-sortie input budget; no ARM')
            perception.activate_inputs()
        print('Evidence: '+str(out)+'\nWAIT: remain in Position until READY.', flush=True)
        started = time.monotonic()
        abort_at = None
        restored_since = None
        ready_announced = False
        while True:
            now = time.monotonic()
            if stop['requested'] or now-started > 150:
                if abort_at is None:
                    abort_at = now
                    core.request_abort(now, 'operator interrupt' if stop['requested'] else 'session deadline')
            if core.controller.state == 'ABORT' and abort_at is None:
                abort_at = now
            if abort_at is not None and now-abort_at > 20:
                report['error'] = 'abort observation deadline; landing unconfirmed, operator must act'
                break
            if perception:
                pump_task_inputs(perception,runtime)
            rclpy.spin_once(runtime, timeout_sec=.02)
            now = time.monotonic()
            if core.ready_at is not None and not ready_announced:
                print('READY: within 60 s switch Position -> Offboard ONCE. '
                      'This authorizes automatic ARM and takeoff; keep RC available.', flush=True)
                ready_announced = True
            s = core.sample
            terminal = core.controller.state in core.controller.TERMINAL
            if terminal:
                landed = (core.controller.fresh(s.status_at, now, 1.5)
                    and core.controller.fresh(s.land_at, now, 2.) and not s.armed and s.landed)
                restored_since = (now if restored_since is None else restored_since) if landed else None
                if core.controller.state in {'HANDOVER', 'KILLED'}:
                    report['error'] = 'operator has control / kill active; no autonomous re-entry'
                    break
                if restored_since is not None and now-restored_since >= 2:
                    report['landing_confirmed'] = True
                    report['passed'] = (core.controller.state == 'DONE'
                        and {'OFFBOARD', 'ARM', 'LAND'} <= core.accepted_commands
                        and core.landing_seen and not stop['requested'] and not core.command_fault)
                    break
            if now-started > 175:
                report['error'] = 'final observation timeout; no verified landing'
                break
    except Exception as exc:
        report['error'] = str(exc)
        report['passed'] = False
    finally:
        if core:
            core.close('runtime exit; all publishers removed')
        if runtime:
            report['runtime'] = runtime.report()
            runtime.destroy_node()
        if perception:
            try:perception.close()
            except Exception as exc:
                report['passed']=False;report['reader_cleanup_error']=str(exc)
            report['task_pipe']=perception.report()
        if initialized:
            rclpy.try_shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({'evidence': str(out), **report}, indent=2), flush=True)
        print('Outputs stopped. No automatic restart. If airborne/unconfirmed, use RC/PX4 safety procedure.', flush=True)
    return 0 if report['passed'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release', type=Path, required=True)
    parser.add_argument('--params', type=Path, required=True)
    parser.add_argument('--authorize-real-flight', action='store_true', required=True)
    args = parser.parse_args()
    try:
        permit, params = validate(args.release, args.params, ROOT/'config/platform.yaml')
        with open('/tmp/robocup_flight_runtime_shadow.lock', 'a') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise ValueError('another domain-0 flight/bench runtime owns the lock')
            confirm(permit.session_id)
            permit.consume(ROOT/'evidence/live_release_consumed')
            return run(permit, params)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        parser.error(str(exc))


if __name__ == '__main__':
    raise SystemExit(main())
