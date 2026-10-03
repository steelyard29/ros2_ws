#!/usr/bin/env python3
"""Bounded serial XRCE session and PX4 telemetry check.

Default is subscription-only. `--ev-inject` publishes only
`/fmu/in/vehicle_visual_odometry` for a prop-off bench. It never writes
parameters, never arms, and never publishes commands or setpoints.
The serial MicroXRCEAgent is kept running: PX4 1.16 does not reconnect after
the Agent is killed. This probe reuses that Agent and does not stop it.
"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import argparse
import math
import numpy as np
import xrce_serial_agent

os.environ['ROS_DOMAIN_ID'] = '0'
# Host localhost-only discovery failed across the existing Docker boundary.
# Container remains localhost-only; this process receives telemetry only and
# publishes exclusively to the isolated preview namespace.
os.environ['ROS_LOCALHOST_ONLY'] = '0'
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = '/home/cfly/ros2_ws/config/fastdds_bridge.xml'
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from rosidl_runtime_py.convert import message_to_ordereddict
from px4_msgs.msg import VehicleStatus, VehicleAttitude, VehicleLocalPosition, TimesyncStatus, EstimatorStatusFlags
from px4_msgs.msg import VehicleOdometry, FailsafeFlags, ManualControlSetpoint
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from alignment_math import rpy_degrees
from ev_odometry import (
    ARMING_STATE_ARMED,
    EV_TOPIC,
    MIN_HEALTHY_PREVIEWS,
    apply_to_vehicle_odometry,
    build_ev_odometry,
    discontinuity,
    extra_flight_inputs,
    inhibit_reason,
    motion_step,
    session_convert,
    should_publish_ev,
)

ROOT = Path(__file__).resolve().parents[1]


def _json_default(value):
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(f'Object of type {type(value).__name__} is not JSON serializable')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', default='/dev/ttyTHS1')
    parser.add_argument('--baud', type=int, default=921600)
    parser.add_argument('--duration', type=int, default=30)
    parser.add_argument('--alignment-preview', action='store_true')
    parser.add_argument('--dynamic-session', action='store_true')
    parser.add_argument('--visual-only-diagnostic', action='store_true')
    parser.add_argument('--imu-copy-diagnostic', action='store_true')
    parser.add_argument('--ev-inject', action='store_true',
                        help='Bounded prop-off publish to /fmu/in/vehicle_visual_odometry')
    parser.add_argument('--allow-ev-hgt', action='store_true',
                        help='Do not stop when cs_ev_hgt is true (EV height fusion expected)')
    parser.add_argument('--session-wait', type=int, default=25)
    args = parser.parse_args()
    duration_limit = 900 if args.dynamic_session else (180 if args.ev_inject else 120)
    if args.dynamic_session and not args.alignment_preview:
        parser.error('dynamic-session requires alignment-preview')
    if args.visual_only_diagnostic and not args.alignment_preview:
        parser.error('visual-only-diagnostic requires alignment-preview')
    if args.imu_copy_diagnostic and not args.alignment_preview:
        parser.error('imu-copy-diagnostic requires alignment-preview')
    if args.visual_only_diagnostic and args.imu_copy_diagnostic:
        parser.error('choose only one IMU diagnostic mode')
    if args.ev_inject and not args.alignment_preview:
        parser.error('ev-inject requires alignment-preview')
    if args.ev_inject and not args.visual_only_diagnostic:
        parser.error('ev-inject requires visual-only-diagnostic')
    if args.ev_inject and args.imu_copy_diagnostic:
        parser.error('ev-inject cannot use IMU copy mode')
    if args.allow_ev_hgt and not args.ev_inject:
        parser.error('allow-ev-hgt requires ev-inject')
    if args.device not in ('/dev/ttyTHS1', '/dev/ttyUSB0') or not 10 <= args.duration <= duration_limit:
        parser.error('unsupported port or duration')
    if not 5 <= args.session_wait <= 60:
        parser.error('unsupported session wait')
    if not Path(args.device).exists():
        raise RuntimeError('Serial device absent')
    agent_info = xrce_serial_agent.ensure(args.device, args.baud)
    prefix = 'ev_inject_' if args.ev_inject else (
        'alignment_preview_' if args.alignment_preview else 'px4_telemetry_')
    out = ROOT/'evidence'/time.strftime(prefix+'%Y%m%d_%H%M%S')
    out.mkdir(exist_ok=False, parents=True)
    control_path = out/'control.json'
    control_path.write_text(json.dumps({'stage': 'baseline', 'stop': False})+'\n')
    print('Evidence:', out, flush=True)
    report = {'device': args.device, 'baud': args.baud, 'ros_domain': 0,
              'flight_validation': False, 'streams': {}, 'input_publishers': {},
              'xrce_agent': agent_info,
              'visual_only_diagnostic': args.visual_only_diagnostic,
              'imu_copy_diagnostic': args.imu_copy_diagnostic,
              'enable_imu_fusion': bool(args.imu_copy_diagnostic),
              'ev_inject': bool(args.ev_inject),
              'allow_ev_hgt': bool(args.allow_ev_hgt)}
    rclpy.init()
    node_name = 'robocup_px4_ev_bench' if args.ev_inject else 'robocup_px4_readonly_probe'
    node = Node(node_name, enable_rosout=False, start_parameter_services=False)
    camera = None
    camera_log = None
    last_health_check = 0.
    last_stage = None
    raw = (out/'samples.jsonl').open('w')
    alignment = {'frame': None, 'tracking': None, 'fault': None, 'fault_detail': None, 'previous': None,
                 'preview_count': 0, 'pairs': [], 'ages_ms': [], 'states': {}}
    ev_state = {'pub': None, 'count': 0, 'inhibit': False, 'quality': {}, 'flags': {
        'cs_ev_pos': 0, 'cs_ev_yaw': 0, 'cs_ev_hgt': 0, 'cs_ev_vel': 0,
        'xy_valid': 0, 'heading_good_for_control': 0}}
    # Keep attitude and VIO/preview samples dense for motion comparison. Large
    # IMU and local-position JSON objects are diagnostic only, so record them
    # sparsely to avoid making the probe itself compete with cuVSLAM.
    raw_stride = {
        '/camera/camera/imu': 20,
        '/fmu/out/vehicle_attitude': 5,
        '/fmu/out/vehicle_local_position': 10,
        '/fmu/out/vehicle_local_position_v1': 10,
    }
    def receive(topic, msg):
        now = time.monotonic()
        entry = report['streams'].setdefault(topic, {'count': 0, 'first_received': now})
        entry['count'] += 1
        entry['last_received'] = now
        if topic == '/camera/camera/imu':
            stamp_ns = int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec)
            previous_stamp_ns = entry.get('_last_stamp_ns')
            if previous_stamp_ns is not None:
                dt_ms = (stamp_ns-previous_stamp_ns)/1e6
                entry['stamp_dt_ms_min'] = min(entry.get('stamp_dt_ms_min', dt_ms), dt_ms)
                entry['stamp_dt_ms_max'] = max(entry.get('stamp_dt_ms_max', dt_ms), dt_ms)
                if dt_ms <= 0:
                    entry['stamp_nonmonotonic_count'] = entry.get('stamp_nonmonotonic_count', 0)+1
                if dt_ms > 20:
                    entry['stamp_gap_gt_20ms_count'] = entry.get('stamp_gap_gt_20ms_count', 0)+1
            entry['_last_stamp_ns'] = stamp_ns
        if topic == '/fmu/out/vehicle_attitude':
            payload = {'timestamp_sample': int(msg.timestamp_sample),
                       'q': [float(x) for x in msg.q]}
        else:
            payload = None
        stride = raw_stride.get(topic, 1)
        if entry['count'] == 1 or entry['count'] % stride == 0:
            if payload is None:
                payload = dict(message_to_ordereddict(msg))
            entry['latest'] = payload
            raw.write(json.dumps({
                'topic': topic,
                'receipt_ns': node.get_clock().now().nanoseconds,
                'message': payload,
            }, default=_json_default)+'\n')
        elif payload is not None:
            entry['latest'] = payload
    if args.alignment_preview:
        node.create_subscription(Imu, '/camera/camera/imu',
            lambda msg: receive('/camera/camera/imu', msg), qos_profile_sensor_data)
        pub = node.create_publisher(VehicleOdometry, '/robocup/alignment/visual_odometry_preview', 10)
        def tracking(msg):
            data = json.loads(msg.data)
            alignment['tracking'] = (data, time.monotonic())
            key = str(data['vo_state'])
            alignment['states'][key] = alignment['states'].get(key, 0)+1
            raw.write(json.dumps({'topic': '/robocup/alignment/tracking', 'message': data})+'\n')
        node.create_subscription(String, '/robocup/alignment/tracking', tracking, 10)
        def vio(msg):
            receive('/visual_slam/tracking/odometry', msg)
            now_ns = node.get_clock().now().nanoseconds
            stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            age = (now_ns-stamp)/1e6
            alignment['ages_ms'].append(age)
            tr = alignment['tracking']
            if alignment['fault'] or tr is None or tr[0]['vo_state'] != 1:
                return
            # Compare tracking to this VIO stamp, not wall time. Receipt delay
            # plus a previous-frame tracking stamp was exceeding 300ms and
            # dropping nearly all previews while vo_state stayed 1.
            if time.monotonic()-tr[1] > .3:
                return
            if not -50 <= (stamp-tr[0]['stamp_ns'])/1e6 <= 300:
                return
            if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link' or not -50 <= age <= 400:
                return
            pos, q = msg.pose.pose.position, msg.pose.pose.orientation
            p, quat = [pos.x, pos.y, pos.z], [q.x, q.y, q.z, q.w]
            try:
                alignment['frame'], validated, matrix, position, orientation = session_convert(
                    alignment['frame'], p, quat)
                dt, disp, ang = motion_step(alignment['previous'], stamp, validated, matrix)
                jump = discontinuity(alignment['previous'], stamp, validated, matrix)
                if jump == 'timestamp moved backwards or duplicated':
                    return
                if jump:
                    alignment['fault'] = jump
                    alignment['fault_detail'] = {
                        'previous_position': alignment['previous'][1].tolist(),
                        'current_position': validated.tolist(),
                    }
                    return
                alignment['previous'] = (stamp, validated, matrix)
            except ValueError:
                alignment['fault'] = 'Invalid VIO pose'
                return
            preview = VehicleOdometry()
            preview.timestamp = now_ns//1000
            preview.timestamp_sample = stamp//1000
            preview.pose_frame = VehicleOdometry.POSE_FRAME_FRD
            preview.position = position.tolist()
            preview.q = orientation.tolist()
            preview.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
            preview.velocity = [math.nan]*3
            preview.angular_velocity = [math.nan]*3
            preview.position_variance = [math.nan]*3
            preview.orientation_variance = [math.nan]*3
            preview.velocity_variance = [math.nan]*3
            preview.quality = -1  # Isolated preview; never use this quality on /fmu/in.
            pub.publish(preview)
            alignment['preview_count'] += 1
            if (ev_state['pub'] is not None and not ev_state['inhibit']
                    and alignment['fault'] is None
                    and alignment['preview_count'] >= MIN_HEALTHY_PREVIEWS):
                ev_msg = VehicleOdometry()
                apply_to_vehicle_odometry(
                    ev_msg, build_ev_odometry(
                        position, orientation, now_ns, stamp,
                        vo_state=int(tr[0]['vo_state']),
                        position_step_m=disp, angle_step_rad=ang, dt_s=dt))
                ev_state['quality'][str(ev_msg.quality)] = ev_state['quality'].get(str(ev_msg.quality), 0) + 1
                if should_publish_ev(ev_msg.quality):
                    ev_state['pub'].publish(ev_msg)
                    ev_state['count'] += 1
                    if ev_state['count'] == 1 or ev_state['count'] % 30 == 0:
                        raw.write(json.dumps({
                            'topic': '/fmu/in/vehicle_visual_odometry',
                            'receipt_ns': now_ns,
                            'message': dict(message_to_ordereddict(ev_msg)),
                        }, default=_json_default)+'\n')
            attitude = report['streams'].get('/fmu/out/vehicle_attitude')
            if attitude is not None and time.monotonic()-attitude['last_received'] < .25:
                att = attitude['latest']
                if abs(stamp//1000-att['timestamp_sample']) < 200000:
                    alignment['pairs'].append([stamp, *rpy_degrees(orientation), *rpy_degrees(att['q'])])
            raw.write(json.dumps({'topic': '/robocup/alignment/visual_odometry_preview',
                                  'message': dict(message_to_ordereddict(preview))})+'\n')
        node.create_subscription(Odometry, '/visual_slam/tracking/odometry', vio, qos_profile_sensor_data)
    for names, kind in [
        (('vehicle_status', 'vehicle_status_v1'), VehicleStatus),
        (('vehicle_attitude',), VehicleAttitude),
        (('vehicle_local_position', 'vehicle_local_position_v1'), VehicleLocalPosition),
        (('timesync_status',), TimesyncStatus),
        (('estimator_status_flags',), EstimatorStatusFlags),
        (('failsafe_flags',), FailsafeFlags),
        (('manual_control_setpoint',), ManualControlSetpoint),
    ]:
        for name in names:
            topic = '/fmu/out/'+name
            node.create_subscription(kind, topic, lambda msg, t=topic: receive(t, msg), qos_profile_sensor_data)
    def audit():
        for topic, _ in node.get_topic_names_and_types():
            if not topic.startswith('/fmu/in/'):
                continue
            count = node.count_publishers(topic)
            if not count:
                continue
            report['input_publishers'][topic] = count
            own = 1 if (args.ev_inject and ev_state['pub'] is not None) else 0
            extras = extra_flight_inputs({topic: count}, own_ev_publishers=own)
            if extras:
                raise RuntimeError('Existing flight-input publisher: '+topic)
    def px4_alive():
        for topic, entry in report['streams'].items():
            if topic.startswith('/fmu/out/') and entry.get('count', 0) >= 5:
                return True
        return False
    try:
        end = time.monotonic()+3
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        audit()
        log_mark = xrce_serial_agent.log_offset()
        session_end = time.monotonic()+args.session_wait
        while time.monotonic() < session_end:
            rclpy.spin_once(node, timeout_sec=.1)
            audit()
            if px4_alive():
                break
            if xrce_serial_agent.running_pid(args.device, args.baud) is None:
                raise RuntimeError('Persistent XRCE Agent exited')
        report['xrce_session'] = {
            'px4_topics': px4_alive(),
            'agent_client_key': xrce_serial_agent.session_established(since=log_mark),
        }
        if not report['xrce_session']['px4_topics']:
            raise RuntimeError(
                'PX4 XRCE client did not create a session. Keep MicroXRCEAgent running '
                'and power-cycle the flight controller; do not restart the Agent alone.')
        if args.ev_inject:
            ev_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE,
                                history=HistoryPolicy.KEEP_LAST)
            ev_state['pub'] = node.create_publisher(
                VehicleOdometry, EV_TOPIC, ev_qos)
            control_path.write_text(json.dumps({'stage': 'ev_inject', 'stop': False})+'\n')
        end = time.monotonic()+args.duration
        if args.alignment_preview:
            camera_log = (out/'camera.log').open('w')
            camera = subprocess.Popen(['docker', 'exec', '-e', 'ROS_DOMAIN_ID=0',
                '-e', 'ROS_LOCALHOST_ONLY=1',
                '-e', 'FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/ros2_ws/config/fastdds_bridge.xml',
                'isaac_ros_dev', 'bash', '-c',
                'source /workspaces/ros2_ws/isaac_vio_debug/scripts/container_env.sh; '
                'exec python3 /workspaces/ros2_ws/robocup_nav/scripts/alignment_camera.py '
                '--duration '+str(args.duration)+' --control-file '+
                str(control_path).replace('/home/cfly/ros2_ws/', '/workspaces/ros2_ws/')+
                (' --visual-only-diagnostic' if args.visual_only_diagnostic else '')+
                (' --imu-copy-diagnostic' if args.imu_copy_diagnostic else '')],
                stdout=camera_log, stderr=subprocess.STDOUT)
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic()-last_health_check >= .5:
                audit()
                if xrce_serial_agent.running_pid(args.device, args.baud) is None:
                    raise RuntimeError('Persistent XRCE Agent exited')
                last_health_check = time.monotonic()
                control = json.loads(control_path.read_text())
                if control.get('stage') != last_stage:
                    last_stage = control.get('stage')
                    raw.write(json.dumps({'event': 'stage', 'stage': last_stage,
                        'receipt_ns': node.get_clock().now().nanoseconds})+'\n')
                fresh = {topic: round(time.monotonic()-entry['last_received'], 3)
                         for topic, entry in report['streams'].items()}
                vio_stream = report['streams'].get('/visual_slam/tracking/odometry')
                if (args.alignment_preview and vio_stream is not None
                        and vio_stream.get('count', 0) >= 5
                        and time.monotonic()-vio_stream['last_received'] > 1.0
                        and alignment['fault'] is None):
                    alignment['fault'] = 'VIO output stale; new session required'
                    alignment['fault_detail'] = {
                        'vio_receipt_age_s': float(
                            time.monotonic()-vio_stream['last_received']),
                        'tracking_receipt_age_s': float(
                            time.monotonic()-alignment['tracking'][1])
                            if alignment['tracking'] is not None else None,
                    }
                    ev_state['inhibit'] = True
                flags = (report['streams'].get('/fmu/out/estimator_status_flags') or {}).get('latest') or {}
                local = (report['streams'].get('/fmu/out/vehicle_local_position') or {}).get('latest') or {}
                if args.ev_inject:
                    for key in ('cs_ev_pos', 'cs_ev_yaw', 'cs_ev_hgt', 'cs_ev_vel'):
                        if flags.get(key):
                            ev_state['flags'][key] += 1
                    if local.get('xy_valid'):
                        ev_state['flags']['xy_valid'] += 1
                    if local.get('heading_good_for_control'):
                        ev_state['flags']['heading_good_for_control'] += 1
                    reason = inhibit_reason(cs_ev_hgt=bool(flags.get('cs_ev_hgt')),
                                            cs_ev_vel=bool(flags.get('cs_ev_vel')),
                                            allow_ev_hgt=bool(args.allow_ev_hgt))
                    if reason and alignment['fault'] is None:
                        ev_state['inhibit'] = True
                        alignment['fault'] = reason
                report_status = {'stage': last_stage, 'seconds_remaining': round(end-time.monotonic()),
                    'preview_count': alignment['preview_count'], 'fault': alignment['fault'],
                    'fault_detail': alignment['fault_detail'],
                    'tracking_states': alignment['states'], 'receipt_age_s': fresh,
                    'latest_vio': report['streams'].get('/visual_slam/tracking/odometry', {}).get('latest'),
                    'ev_publish_count': ev_state['count'],
                    'estimator_flags': {k: flags.get(k) for k in
                        ('cs_tilt_align', 'cs_yaw_align', 'cs_ev_pos', 'cs_ev_yaw',
                         'cs_ev_hgt', 'cs_ev_vel', 'cs_rng_hgt', 'cs_inertial_dead_reckoning')},
                    'local_xy_valid': local.get('xy_valid'),
                    'heading_good_for_control': local.get('heading_good_for_control'),
                    'range': {
                        'dist_bottom': local.get('dist_bottom'),
                        'dist_bottom_valid': local.get('dist_bottom_valid'),
                        'dist_bottom_sensor_bitfield': local.get('dist_bottom_sensor_bitfield'),
                        'z': local.get('z'),
                        'z_valid': local.get('z_valid'),
                    },
                    'receipt_ns': node.get_clock().now().nanoseconds}
                status_temp = out/'live_status.tmp'
                status_temp.write_text(json.dumps(report_status, default=_json_default)+'\n')
                status_temp.replace(out/'live_status.json')
                raw.flush()
                if control.get('stop'):
                    report['stop_reason'] = 'operator_complete'
                    break
                if alignment['fault']:
                    raise RuntimeError(alignment['fault'])
            if camera is not None and camera.poll() is not None:
                raise RuntimeError('Camera supervisor exited before collection finished')
            for topic, entry in report['streams'].items():
                if 'vehicle_status' in topic and entry['latest']['arming_state'] == ARMING_STATE_ARMED:
                    raise RuntimeError(inhibit_reason(armed=True))
        report['topics'] = node.get_topic_names_and_types()
        if args.dynamic_session and report.get('stop_reason') != 'operator_complete':
            report['stop_reason'] = 'timeout_incomplete'
        if args.ev_inject and 'stop_reason' not in report:
            report['stop_reason'] = 'duration_complete'
        required = ['/fmu/out/vehicle_attitude', '/fmu/out/vehicle_status_v1',
                    '/fmu/out/vehicle_local_position', '/fmu/out/timesync_status']
        if args.alignment_preview:
            required += ['/visual_slam/tracking/odometry', '/camera/camera/imu']
        missing = [t for t in required if report['streams'].get(t, {}).get('count', 0) < 5]
        if missing:
            raise RuntimeError('Missing or insufficient telemetry: '+', '.join(missing))
        if args.alignment_preview and (alignment['preview_count'] < 20 or len(alignment['pairs']) < 20 or alignment['fault']):
            raise RuntimeError('Insufficient valid paired previews: '+str(alignment['fault']))
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        control_path.write_text(json.dumps({'stage': last_stage, 'stop': True})+'\n')
        ev_state['inhibit'] = True
        if ev_state['pub'] is not None:
            node.destroy_publisher(ev_state['pub'])
            ev_state['pub'] = None
        if camera is not None:
            try:
                camera.wait(timeout=20)
            except subprocess.TimeoutExpired:
                report['camera_cleanup_error'] = 'Supervisor did not finish; inspect container processes'
            if camera_log is not None:
                camera_log.close()
        try:
            shutil.copyfile(xrce_serial_agent.LOG, out/'agent.log')
        except OSError:
            (out/'agent.log').write_text('persistent agent log unavailable\n')
        for entry in report['streams'].values():
            span = entry['last_received']-entry['first_received']
            entry['hz'] = (entry['count']-1)/span if span > 0 else 0
            entry.pop('_last_stamp_ns', None)
        if args.alignment_preview:
            pairs = np.asarray(alignment['pairs'])
            ages = np.asarray(alignment['ages_ms'])
            report['alignment'] = {'preview_count': alignment['preview_count'],
                'tracking_states': alignment['states'], 'fault': alignment['fault'],
                'fault_detail': alignment['fault_detail'],
                'matched_pairs': len(pairs), 'max_pair_dt_ms': 200,
                'quality_and_covariance_verified': False, 'alignment_accepted': False,
                'ev_inject': bool(args.ev_inject),
                'ev_publish_count': ev_state['count'],
                'ev_flag_samples': ev_state['flags'],
                'note': 'Approximate timestamp pairing; static comparison is not extrinsic calibration'}
            if args.ev_inject:
                note = (
                    'Bounded prop-off EV input; quality from vo_state and motion step, '
                    'FRD pose, NaN velocity, no timesync offset, no range Z. ')
                if args.allow_ev_hgt:
                    note += 'EV height fusion allowed; stop on EV velocity fusion. '
                else:
                    note += 'Stop on EV height/velocity fusion. '
                report['alignment']['note'] = note + 'Not flight approval.'
                report['alignment']['allow_ev_hgt'] = bool(args.allow_ev_hgt)
                report['alignment']['quality_from_tracking'] = True
                report['alignment']['ev_quality_counts'] = ev_state['quality']
            if len(ages):
                report['alignment']['vio_receipt_age_ms_p50_p95_max'] = np.percentile(ages, [50,95,100]).tolist()
            if len(pairs):
                report['alignment']['preview_rpy_median_deg'] = np.median(pairs[:, 1:4], axis=0).tolist()
                report['alignment']['px4_rpy_median_deg'] = np.median(pairs[:, 4:7], axis=0).tolist()
                delta = (pairs[:, 1:4]-pairs[:, 4:7]+180)%360-180
                report['alignment']['rpy_difference_median_deg'] = np.median(delta, axis=0).tolist()
            frame = alignment['frame']
            if frame is not None:
                report['alignment']['session_origin_in_odom'] = frame.origin.tolist()
                report['alignment']['rotation_frd_from_odom'] = frame.world.tolist()
        status = (report['streams'].get('/fmu/out/vehicle_status_v1')
                  or report['streams'].get('/fmu/out/vehicle_status') or {}).get('latest') or {}
        failsafe = (report['streams'].get('/fmu/out/failsafe_flags') or {}).get('latest') or {}
        rc = (report['streams'].get('/fmu/out/manual_control_setpoint') or {}).get('latest') or {}
        report['preflight'] = {
            'flight_validation': False,
            'arming_state': status.get('arming_state'),
            'armed': status.get('arming_state') == ARMING_STATE_ARMED,
            'nav_state': status.get('nav_state'),
            'pre_flight_checks_pass': status.get('pre_flight_checks_pass'),
            'gcs_connection_lost': failsafe.get('gcs_connection_lost', status.get('gcs_connection_lost')),
            'manual_control_signal_lost': failsafe.get('manual_control_signal_lost'),
            'manual_control_valid': rc.get('valid'),
            'manual_control_source': rc.get('data_source'),
            'failsafe': status.get('failsafe'),
            'local_position_invalid': failsafe.get('local_position_invalid'),
            'local_altitude_invalid': failsafe.get('local_altitude_invalid'),
            'home_position_invalid': failsafe.get('home_position_invalid'),
            'offboard_control_signal_lost': failsafe.get('offboard_control_signal_lost'),
            'input_publishers': dict(report['input_publishers']),
        }
        raw.close()
        node.destroy_node()
        rclpy.shutdown()
        (out/'report.json').write_text(json.dumps(report, indent=2, default=_json_default)+'\n')
        print('Evidence:', out)
        print(json.dumps({k: {'count': v['count'], 'hz': v['hz']} for k, v in report['streams'].items()}, indent=2))
        print('Error:', report.get('error'))
    return 0 if report['streams'] and 'error' not in report else 1


if __name__ == '__main__':
    raise SystemExit(main())
