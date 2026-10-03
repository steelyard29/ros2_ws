#!/usr/bin/env python3
"""Bounded subscription-only observer; no Agent/camera or /fmu/in startup."""
import argparse
from dataclasses import asdict, replace
import json
import math
import os
from pathlib import Path
import time

from flight_supervisor import FlightSample, FlightSupervisor
from preflight_source_guard import SourceGuard, observer_passed, telemetry_observation
from preflight_window import control_requests_finalize


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=int, default=30)
    parser.add_argument('--control-file', type=Path)
    args = parser.parse_args()
    if not 10 <= args.duration <= 180:
        parser.error('duration must be 10..180 s')
    os.environ['ROS_DOMAIN_ID'] = '0'
    os.environ['ROS_LOCALHOST_ONLY'] = '0'
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = '/home/cfly/ros2_ws/config/fastdds_bridge.xml'
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data as qos
    from px4_msgs.msg import (VehicleStatus, VehicleLocalPosition, EstimatorStatusFlags,
                             VehicleLandDetected, ManualControlSetpoint, VehicleOdometry)
    rclpy.init()
    node = Node('robocup_preflight_readonly', enable_rosout=False, start_parameter_services=False)
    sample = FlightSample()
    counts = {}
    ev_at = -1.0
    supervisor = FlightSupervisor()
    source_guard = SourceGuard()
    ever_armed = False
    gcs_connected = False

    def guarded(name, callback, sampled=False, sample_lead_us=0):
        def receive(m):
            nonlocal ever_armed
            # Even an invalid packet claiming ARM must not be certified safe.
            if name == 'status' and m.arming_state == 2:
                ever_armed = True
            stamp = int(m.timestamp)
            age = (node.get_clock().now().nanoseconds/1000-stamp)/1e6
            if source_guard.accept(name, stamp, time.monotonic(), age,
                                   int(m.timestamp_sample) if sampled else None,
                                   sample_lead_us=sample_lead_us,
                                   observed=telemetry_observation(name, m)):
                callback(m)
        return receive
    def update(topic, **fields):
        nonlocal sample
        counts[topic] = counts.get(topic, 0) + 1
        sample = replace(sample, **fields)
    def status(m):
        nonlocal gcs_connected
        gcs_connected = not bool(m.gcs_connection_lost)
        if int(m.arming_state) != 1:
            source_guard.latch_fault('status is not explicitly disarmed', 'status')
        update('status', status_at=time.monotonic(), armed=m.arming_state == 2,
               nav_state=int(m.nav_state), preflight_ok=bool(m.pre_flight_checks_pass),
               failsafe=bool(m.failsafe))
    def local(m):
        update('local', local_at=time.monotonic(), x=float(m.x), y=float(m.y), z=float(m.z),
               heading=float(m.heading), vx=float(m.vx), vy=float(m.vy), vz=float(m.vz),
               xy_valid=bool(m.xy_valid), z_valid=bool(m.z_valid),
               v_xy_valid=bool(m.v_xy_valid), v_z_valid=bool(m.v_z_valid),
               reset_counters=tuple(int(getattr(m, n)) for n in
                    ('xy_reset_counter', 'z_reset_counter', 'heading_reset_counter',
                     'vxy_reset_counter', 'vz_reset_counter')))
    def flags(m):
        update('flags', flags_at=time.monotonic(), ev_position=bool(m.cs_ev_pos),
               ev_height=bool(m.cs_ev_hgt), ev_yaw=bool(m.cs_ev_yaw),
               ev_velocity=bool(m.cs_ev_vel), baro_height=bool(m.cs_baro_hgt),
               range_height=bool(m.cs_rng_hgt))
    def ev(m):
        nonlocal ev_at
        age = (node.get_clock().now().nanoseconds / 1000 - m.timestamp_sample) / 1e6
        if (m.quality >= 50 and m.pose_frame == 2 and -.05 <= age <= .4
                and all(math.isfinite(float(v)) for v in m.position)):
            ev_at = time.monotonic()
    node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', guarded('status', status), qos)
    node.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', guarded('local', local), qos)
    node.create_subscription(EstimatorStatusFlags, '/fmu/out/estimator_status_flags', guarded('flags', flags), qos)
    node.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected',
        guarded('land', lambda m: update('land', land_at=time.monotonic(), landed=bool(m.landed))), qos)
    node.create_subscription(ManualControlSetpoint, '/fmu/out/manual_control_setpoint',
        guarded('rc', lambda m: update('rc', rc_at=time.monotonic(),
            rc_valid=bool(m.valid) and int(m.data_source) == 1), sampled=True), qos)
    node.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry',
        guarded('ev', ev, sampled=True, sample_lead_us=60_000), qos)
    started = time.monotonic()
    since = None
    max_ready = 0.0
    publishers = {}
    samples = []
    last_audit = -math.inf
    ready = False

    def snapshot_publishers():
        return {name: node.count_publishers(name)
                for name, _ in node.get_topic_names_and_types()
                if name.startswith('/fmu/in/') and node.count_publishers(name)}

    def audit(now):
        nonlocal sample, publishers, ready, since, max_ready, last_audit
        sample = replace(sample, ev_ok=0 <= now-ev_at <= .3)
        publishers = snapshot_publishers()
        exclusive = publishers == {'/fmu/in/vehicle_visual_odometry': 1}
        sample = replace(sample, input_ownership_ok=exclusive)
        ready = supervisor.ready(sample, now) and exclusive and not source_guard.fault and gcs_connected
        since = (now if since is None else since) if ready else None
        max_ready = max(max_ready, now-since if since is not None else 0)
        samples.append({'elapsed': now-started, 'ready': ready,
                        'source_fault': source_guard.fault,
                        'telemetry_frozen': source_guard.fault is not None,
                        'sample': asdict(sample), 'publishers': publishers})
        last_audit = now

    def finalize_requested():
        if args.control_file is None:
            return False
        try:
            return control_requests_finalize(args.control_file.read_text())
        except OSError:
            return False

    try:
        while time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            finalize = finalize_requested()
            if finalize or now-last_audit >= .2:
                audit(now)
            if finalize or sample.armed or ever_armed:
                ever_armed = sample.armed or ever_armed
                break
    finally:
        now = time.monotonic()
        sample = replace(sample, ev_ok=0 <= now-ev_at <= .3)
        publishers = snapshot_publishers()
        exclusive = publishers == {'/fmu/in/vehicle_visual_odometry': 1}
        sample = replace(sample, input_ownership_ok=exclusive)
        final_instant_ready = (supervisor.ready(sample, now) and exclusive
                               and not source_guard.fault and gcs_connected)
        samples.append({'elapsed': now-started, 'ready': final_instant_ready, 'final': True,
                        'source_fault': source_guard.fault,
                        'telemetry_frozen': source_guard.fault is not None,
                        'sample': asdict(sample), 'publishers': publishers})
        # The last periodic ready flag cannot stand in for this exit sample.
        final_ready = (ready and final_instant_ready and 0 <= now-last_audit <= .3
                       and gcs_connected and supervisor.ready(sample, now)
                       and source_guard.fault is None)
        report = {'flight_validation': False, 'subscription_only': True,
                  'recorded_at_unix': time.time(),
                  'duration_s': now-started, 'counts': counts,
                  'ready_continuous_s_max': max_ready,
                  'passed': observer_passed(max_ready, final_ready, ever_armed, source_guard.fault),
                  'final_ready': final_ready, 'source_fault': source_guard.fault,
                  'duplicate_samples': source_guard.duplicates, 'gcs_connected': gcs_connected,
                  'ever_armed': ever_armed, 'latest': asdict(sample),
                  'source_diagnostics': source_guard.diagnostics(now),
                  'latest_semantics': 'telemetry fields are last accepted per topic; '
                                      'ev_ok and ownership are separately evaluated at exit',
                  'input_publishers': publishers,
                  'note': 'Telemetry gate only; no geometry/RC-switch/failsafe certification'}
        out = Path(__file__).resolve().parents[1] / 'evidence' / time.strftime('preflight_readonly_%Y%m%d_%H%M%S')
        out.mkdir(parents=True, exist_ok=False)
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        (out/'samples.json').write_text(json.dumps(samples)+'\n')
        print(json.dumps({'evidence': str(out), **report}, indent=2), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
