#!/usr/bin/env python3
"""Default CLI is SHADOW ONLY; it cannot request real flight.

Real input mode reads the existing FC/VIO graph; isolated mode reads only
/robocup/runtime_test/input. Neither starts an Agent, camera or actuator.
The separate explicit flight_bench_runtime entrypoint can route EV only to
PX4 through BenchEvGate. This default CLI cannot select that route.
The separately authorized handshake_bench_runtime entrypoint can route a
bounded DISARMED mode handshake through its own guard; never ARM or LAND.
The live_flight_runtime entrypoint passes a reviewed, consumed FlightPermit;
task_flight_runtime additionally requires its own TaskFlightPermit and separate
horizontal dispatch contract. There is no --live shortcut in this default CLI.
"""
import argparse
from collections import Counter, deque
import fcntl
import hashlib
import json
import os
from pathlib import Path
import time
import uuid

from flight_runtime_core import FlightRuntimeCore
from flight_messages import build_messages
from ev_odometry import apply_to_vehicle_odometry
from flight_output_boundary import routes, ownership_ok, BenchEvGate

ROOT = Path(__file__).resolve().parents[1]


def topics(isolated):
    prefix = '/robocup/runtime_test/input' if isolated else '/fmu/out'
    mapping = {name: prefix+'/'+name for name in (
        'vehicle_status_v1', 'vehicle_local_position', 'estimator_status_flags',
        'vehicle_land_detected', 'manual_control_setpoint', 'manual_control_switches',
        'vehicle_command_ack', 'failsafe_flags')}
    mapping['vio'] = prefix+'/vio' if isolated else '/visual_slam/tracking/odometry'
    mapping['tracking'] = prefix+'/tracking' if isolated else '/robocup/alignment/tracking'
    output = '/robocup/runtime_test/output' if isolated else '/robocup/flight_runtime_shadow'
    return mapping, output


def create_node(core, *, isolated=False, exercise=False, clock=time.monotonic, bench_ev=False,
                handshake_bench=False, live_permit=None, task_preview=False,perception_node=None,
                resident_binding=None):
    external_ev=getattr(core,'external_ev',None)
    if resident_binding is not None:
        from resident_status_binding import ResidentStatusBinding
        if (type(resident_binding) is not ResidentStatusBinding or isolated or not live_permit
                or not getattr(core,'task_core',False) or resident_binding.evidence is not external_ev):
            raise ValueError('real external EV requires matching resident task binding and permit')
        resident_binding.check()
    if external_ev is not None and (bench_ev or handshake_bench or task_preview
            or (not isolated and resident_binding is None) or (isolated and live_permit)):
        raise ValueError('external EV real routing not integrated; isolated validation only')
    output_routes = routes(isolated=isolated, bench_ev=bench_ev)
    live = live_permit is not None
    task = getattr(core, 'task_core', False)
    if perception_node is not None:
        from task_domain_io import ScopedPerceptionNode
        if (not task or type(perception_node) is not ScopedPerceptionNode
                or perception_node.isolated!=isolated):
            raise ValueError('split-domain input requires matching scoped task port')
    if task:
        from task_flight_controller import TaskFlightCore
        if type(core) is not TaskFlightCore or bench_ev or handshake_bench:
            raise ValueError('dedicated task core required; no bench route')
        if task_preview and (live or isolated or exercise):
            raise ValueError('task preview is real-input observation only, no exercise/live route')
        if live:
            from task_flight_release import TaskFlightPermit
            if type(live_permit) is not TaskFlightPermit or core.task_config != live_permit.task_config:
                raise ValueError('navigation requires a separate task permit and matching config')
        elif not isolated and not task_preview:
            raise ValueError('task needs isolated test, observation preview, or reviewed task permit')
    elif task_preview:
        raise ValueError('task preview requires task core')
    if live:
        from flight_release import FlightPermit, ROOT as release_root
        from live_flight_core import LiveFlightCore
        if (isolated or bench_ev or handshake_bench or not exercise
                or not isinstance(core, LiveFlightCore) or not isinstance(live_permit, FlightPermit)
                or core.controller.height != live_permit.height or core.controller.hover != live_permit.hover):
            raise ValueError('live route requires dedicated core and consumed release, no synthetic/bench mode')
        live_permit.before_arm()
        marker = release_root/'evidence/live_release_consumed'/(live_permit.session_id+'.json')
        if json.loads(marker.read_text()).get('release_sha256') != live_permit.release_hash:
            raise ValueError('release has not been consumed for this invocation')
        from handshake_dispatch import handshake_routes
        output_routes = handshake_routes()
    elif getattr(core, 'live_core', False) and not isolated and not task_preview:
        raise ValueError('live core without release is allowed only in isolated tests')
    if handshake_bench:
        if (isolated or bench_ev or not exercise or core.aux is None
                or not getattr(core, 'handshake_only', False) or not getattr(core, 'staged', False)
                or not getattr(core, 'require_gcs', False)):
            raise ValueError('real handshake requires exclusive staged AUX disarmed core, no synthetic inputs')
        from handshake_dispatch import handshake_routes
        output_routes = handshake_routes()
    if bench_ev and getattr(core, 'handshake_only', False):
        raise ValueError('handshake is shadow-only; cannot use real EV bench routing')
    if bench_ev and (exercise or core.aux is None):
        raise ValueError('EV-only bench requires AUX input and forbids controller exercise')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data as qos
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from px4_msgs.msg import (VehicleStatus, VehicleLocalPosition, EstimatorStatusFlags,
        VehicleLandDetected, ManualControlSetpoint, ManualControlSwitches, VehicleCommandAck,
        FailsafeFlags, VehicleOdometry, OffboardControlMode, TrajectorySetpoint, VehicleCommand)

    class RuntimeNode(Node):
        def __init__(self):
            super().__init__('robocup_flight_runtime_shadow', enable_rosout=False,
                             start_parameter_services=False, use_global_arguments=False)
            self.inputs, self.prefix = topics(isolated)
            self.perception_node=perception_node if perception_node is not None else self
            self.task_transport_fault = None
            if perception_node is not None:
                domain=self.context.get_domain_id()
                if (domain==perception_node.domain_id
                        or (not isolated and (domain!=0 or perception_node.domain_id!=176))
                        or (isolated and (domain not in (180,181,182,183)
                            or perception_node.domain_id not in (180,181,182,183)))):
                    self.destroy_node()
                    raise ValueError('invalid or non-isolated split-domain runtime layout')
            self.output_routes = output_routes
            self.bench_gate = BenchEvGate() if bench_ev else None
            from handshake_dispatch import HandshakeDispatchGuard
            self.dispatch_guard = HandshakeDispatchGuard() if getattr(core, 'staged', False) else None
            # Exercise the candidate flight boundary only against synthetic
            # inputs. This adds no real flight route or operator authority.
            from flight_dispatch_contract import FlightDispatchContract
            self.candidate_flight_guard = (FlightDispatchContract() if (isolated or live)
                and not getattr(core, 'handshake_only', False) else None)
            if task and not task_preview:
                from task_dispatch_contract import TaskDispatchContract
                self.candidate_flight_guard = TaskDispatchContract()
            self.preflight_since = None
            self.preflight_max_s = 0.
            self.preview_graph=None
            self.pubs = {} if task_preview else {name: self.create_publisher(cls, self.output_routes[name], qos)
                         for name, cls in (('vehicle_visual_odometry', VehicleOdometry),
                            ('offboard_control_mode', OffboardControlMode),
                            ('trajectory_setpoint', TrajectorySetpoint), ('vehicle_command', VehicleCommand))
                         if not (external_ev is not None and name=='vehicle_visual_odometry')}
            self.counts = Counter()
            self.control_trace = []
            self.armed_on_real_input = False
            self.last_output = None
            self.started = clock()
            self.audit_at = -1.0
            self.input_counts = Counter()
            self.flags_timing = deque(maxlen=120)
            if external_ev is not None:
                self.create_subscription(VehicleOdometry,self.output_routes['vehicle_visual_odometry'],
                                         self.external_odometry,qos)
            callbacks = (
                (VehicleStatus, 'vehicle_status_v1', self.status),
                (VehicleLocalPosition, 'vehicle_local_position', self.local),
                (EstimatorStatusFlags, 'estimator_status_flags', self.flags),
                (VehicleLandDetected, 'vehicle_land_detected', lambda m: core.land(clock(), bool(m.landed))),
                (ManualControlSetpoint, 'manual_control_setpoint', lambda m: core.rc(clock(), bool(m.valid))),
                (ManualControlSwitches, 'manual_control_switches',
                 self.switches),
                (VehicleCommandAck, 'vehicle_command_ack', self.ack),
                (FailsafeFlags, 'failsafe_flags', self.failsafe))
            for cls, name, callback in callbacks:
                if core.aux and name == 'manual_control_switches':
                    continue  # Never synthesize or subscribe to native switches in AUX mode.
                if core.aux and name == 'manual_control_setpoint':
                    self.create_subscription(cls, self.inputs[name], self.aux_rc, qos)
                    continue
                self.create_subscription(cls, self.inputs[name],
                    callback if name == 'manual_control_switches' else self.wrap(name, callback), qos)
            self.perception_node.create_subscription(String, self.inputs['tracking'], self.tracking, 10)
            self.perception_node.create_subscription(Odometry, self.inputs['vio'], self.pose, qos)
            self.task_input = None
            if task:
                from task_scene_input import TaskSceneInput
                self.task_input = TaskSceneInput(self.perception_node, core, clock, isolated,read_only=task_preview)
            self.create_timer(.02, self.safe_tick)

        def external_odometry(self,m):
            # Installed Humble executor passes only msg (not MessageInfo).
            # Attribute by current unique graph endpoint, not per-message proof.
            ends=self.get_publishers_info_by_topic(self.output_routes['vehicle_visual_odometry'])
            if len(ends)!=1:
                core.trip('external EV missing/duplicate graph endpoint');return
            accepted=external_ev.receive_message(m,ends[0].endpoint_gid,
                self.get_clock().now().nanoseconds,clock())
            if external_ev.fault:core.trip(external_ev.fault)
            if accepted:
                core.last_ev_at=external_ev.last_at
                self.input_counts['external_ev_received']+=1

        def safe_tick(self):
            if getattr(core, 'closed', False):
                return
            try:
                self.tick()
            except Exception:
                if getattr(core, 'live_core', False):
                    core.close('dispatch/executive exception; all outputs stopped')
                raise

        def wrap(self, name, callback):
            def receive(msg):
                stamp = int(msg.timestamp)
                age = (self.get_clock().now().nanoseconds/1000-stamp)/1e6
                if name=='estimator_status_flags':
                    self.flags_timing.append(dict(stamp_us=stamp,at=clock(),age_s=age))
                if core.receive(name, stamp, clock(), age):
                    self.input_counts[name] += 1
                    callback(msg)
            return receive

        def status(self, m):
            if hasattr(core, 'gcs_status'):
                core.gcs_status(not bool(m.gcs_connection_lost), clock())
            if not isolated and not live and m.arming_state == 2:
                self.armed_on_real_input = True
                core.trip('read-only task preview observed real aircraft armed' if task_preview
                          else 'prop-off runtime observed real aircraft armed')
            core.status(int(m.arming_state), clock(), nav_state=int(m.nav_state),
                        preflight_ok=bool(m.pre_flight_checks_pass), failsafe=bool(m.failsafe),
                        manual_takeover=bool(m.failsafe_and_user_took_over))

        def switches(self, m):
            stamp = int(m.timestamp)
            age = (self.get_clock().now().nanoseconds/1000-stamp)/1e6
            if core.switch_message(stamp, int(m.timestamp_sample), clock(), age,
                                   int(m.mode_slot), int(m.kill_switch), int(m.offboard_switch)):
                self.input_counts['manual_control_switches'] += 1

        def aux_rc(self, m):
            stamp = int(m.timestamp)
            age = (self.get_clock().now().nanoseconds/1000-stamp)/1e6
            self.input_counts['manual_control_setpoint'] += 1
            if core.aux_message(stamp, int(m.timestamp_sample), clock(), age, bool(m.valid),
                                int(m.data_source), float(m.aux1), float(m.aux2)):
                self.input_counts['derived_aux_switches'] += 1

        def local(self, m):
            fields = {k: float(getattr(m, k)) for k in ('x', 'y', 'z', 'heading', 'vx', 'vy', 'vz')}
            fields.update({k: bool(getattr(m, k)) for k in ('xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid')})
            fields['reset_counters'] = tuple(int(getattr(m, k)) for k in (
                'xy_reset_counter', 'z_reset_counter', 'heading_reset_counter', 'vxy_reset_counter', 'vz_reset_counter'))
            core.local(clock(), **fields)

        def flags(self, m):
            core.flags(clock(), ev_position=bool(m.cs_ev_pos), ev_yaw=bool(m.cs_ev_yaw),
                       ev_height=bool(m.cs_ev_hgt), ev_velocity=bool(m.cs_ev_vel),
                       baro_height=bool(m.cs_baro_hgt), range_height=bool(m.cs_rng_hgt))

        def failsafe(self, m):
            # Log warning only. Do not add a battery-triggered controller abort.
            core.battery_warning = int(m.battery_warning)

        def ack(self, m):
            mapping = {176: 'OFFBOARD', 400: 'ARM', 21: 'LAND'}
            if int(m.target_system) == 1 and int(m.target_component) == getattr(core, 'command_source_component', 1):
                core.ack(mapping.get(int(m.command)), int(m.result), int(m.timestamp), clock())

        def tracking(self, m):
            try:
                data = json.loads(m.data)
                core.tracking(int(data['vo_state']), int(data['stamp_ns']), clock())
            except (ValueError, TypeError, KeyError):
                core.trip('invalid tracking JSON')

        def pose(self, m):
            if task_preview:
                self.input_counts['vio_observed']+=1
                return  # TaskSceneInput independently validates this pose; no EV adapter/output.
            if self.bench_gate and not self.bench_gate.check(core, clock()):
                return
            stamp = int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
            now_ns = self.get_clock().now().nanoseconds
            p, q = m.pose.pose.position, m.pose.pose.orientation
            result = core.pose(stamp, (now_ns-stamp)/1e6, clock(), (p.x, p.y, p.z),
                               (q.x, q.y, q.z, q.w),
                               m.header.frame_id == 'odom' and m.child_frame_id == 'base_link', now_ns)
            if result:
                if getattr(core, 'live_core', False) and core.abort_requested():
                    core.request_abort(clock(), 'operator interrupt before EV dispatch')
                    return
                msg = VehicleOdometry()
                apply_to_vehicle_odometry(msg, result)
                self.pubs['vehicle_visual_odometry'].publish(msg)
                self.counts['vehicle_visual_odometry'] += 1
                if hasattr(core, 'note_output'):
                    core.note_output('vehicle_visual_odometry', clock(), int(msg.timestamp))
                if self.bench_gate:
                    self.bench_gate.sent_ev()

        def audit(self, now):
            if resident_binding is not None:
                try:
                    if not resident_binding.verify_graph(self,self.perception_node,
                            allow_missing=now-self.started<=2):return
                except Exception as exc:core.trip('resident binding lost: '+str(exc));return
            # Default requires no real publishers. EV-only bench requires
            # exactly one EV publisher and no other real flight inputs.
            output_counts = {name: self.count_publishers(topic) for name, topic in self.output_routes.items()}
            real_inputs = {topic: self.count_publishers(topic)
                           for topic, _ in self.get_topic_names_and_types()
                           if topic.startswith('/fmu/in/') and self.count_publishers(topic)}
            sources_ok = all((self.perception_node if key in ('vio','tracking') else self)
                             .count_publishers(topic) <= 1 for key,topic in self.inputs.items())
            if external_ev is not None:
                ends=self.get_publishers_info_by_topic(self.output_routes['vehicle_visual_odometry'])
                sources_ok &= (len(ends)==1 and tuple(ends[0].endpoint_gid)==external_ev.publisher_gid)
            if self.task_input:
                sources_ok &= self.task_input.owns()
            if core.aux:
                sources_ok &= self.count_publishers(self.inputs['manual_control_switches']) == 0
            if task_preview:
                self.preview_graph=dict(at=now,real_input_writers=real_inputs,
                    sources_unique_or_absent=sources_ok,output_publishers_created=0)
                if real_inputs or not sources_ok:core.trip('read-only task preview input/writer conflict')
                return  # Never fabricate flight-output ownership for observation.
            own_visible = all(count > 0 for count in output_counts.values())
            # Allow initial discovery settling, but never publish while waiting.
            if own_visible or now-self.started > 2:
                if handshake_bench or live:
                    from handshake_dispatch import handshake_ownership_ok
                    own = handshake_ownership_ok(output_counts, real_inputs)
                else:
                    own = ownership_ok(output_counts, real_inputs, bench_ev=bench_ev)
                core.ownership(now, own and sources_ok)

        def task_transport_failed(self, reason):
            """Latch input loss without destroying the independent PX4 executor.

            No recovery or cached scene replay. Original abort/kill/takeover
            rules decide whether LAND may be requested; this is not a landing ACK.
            """
            if self.task_input is None:
                raise ValueError('transport fault requires task inputs')
            if self.task_transport_fault is not None:
                return
            self.task_transport_fault = str(reason)
            self.task_input.fail(self.task_transport_fault)
            self.task_input.command = self.task_input.h_target = None
            if task_preview:
                core.trip(self.task_transport_fault)
            else:
                core.request_abort(clock(), self.task_transport_fault)

        def tick(self):
            now = clock()
            if now-self.audit_at >= .1:
                self.audit(now)
                self.audit_at = now
            if self.bench_gate:
                self.bench_gate.check(core, now)
            if self.task_input and self.task_transport_fault is None:
                try:
                    self.task_input.prepare(now)
                except Exception as exc:
                    self.task_transport_failed('task input preparation failed: '+str(exc))
            if task_preview:return  # No executive tick, planner goal, EV or control dispatch.
            output = core.tick(now, start=exercise)
            if self.bench_gate:
                s = core.sample
                healthy = (not self.bench_gate.fault and self.bench_gate.sent > 0
                           and not self.bench_gate.blockers and s.preflight_ok
                           and core.controller.position_healthy(s))
                self.preflight_since = (now if self.preflight_since is None else self.preflight_since) if healthy else None
                if self.preflight_since is not None:
                    self.preflight_max_s = max(self.preflight_max_s, now-self.preflight_since)
            stamp = self.get_clock().now().nanoseconds//1000
            if self.candidate_flight_guard:
                self.candidate_flight_guard.check(output, core.sample, now=now, stamp=stamp,
                    state=core.controller.state, origin=core.controller.origin,
                    height=core.controller.height, switch_fresh=core.switch_fresh(now),
                    mode_slot=core.mode_slot, kill_switch=core.kill_switch,
                    exclusive=core._owns(now))
            if live and output.command in ('OFFBOARD', 'ARM'):
                # Recheck the immutable release immediately before hazardous
                # commands, not merely when parsing arguments.
                try:
                    live_permit.before_arm()
                    if clock()-now > .1:
                        raise ValueError('release recheck exceeded dispatch freshness budget')
                    if (not core.gcs_connected or not core.controller.fresh(core.gcs_at, now, 1.5)):
                        raise ValueError('GCS monitor unavailable before mode/ARM')
                except (ValueError, OSError) as exc:
                    core.request_abort(now, str(exc))
                    return  # No mode/ARM or stream from this rejected batch.
            if getattr(core, 'handshake_only', False):
                from disarmed_handshake import validate_handshake_output
                validate_handshake_output(output)
            if self.dispatch_guard:
                from handshake_dispatch import build_handshake_messages
                self.dispatch_guard.check(output, core, now, stamp)
                messages = build_handshake_messages(output, stamp)
            else:
                if task:
                    from task_dispatch_contract import build_task_messages
                    messages = build_task_messages(output, stamp)
                else:
                    messages = build_messages(output, stamp)
                if getattr(core, 'live_core', False) and 'vehicle_command' in messages:
                    messages['vehicle_command'].source_component = 191
            for name, msg in messages.items():
                if (getattr(core, 'live_core', False) and core.abort_requested()
                        and (output.stream or output.command in ('OFFBOARD', 'ARM'))):
                    core.request_abort(clock(), 'operator interrupt before control dispatch')
                    return
                self.pubs[name].publish(msg)
                self.counts[name] += 1
                if hasattr(core, 'note_output'):
                    core.note_output(name, clock(), int(msg.timestamp))
            if output.command:
                core.sent(output.command, stamp)
            if output.state != getattr(self.last_output, 'state', None) or output.command:
                self.control_trace.append({'at': now, 'state': output.state,
                                           'command': output.command, 'stream': output.stream,
                                           'reason': output.reason})
                if handshake_bench:
                    print(json.dumps({'handshake_state': output.state, 'reason': output.reason,
                        'operator_action': getattr(core, 'operator_action', core.handshake.operator_action)(output.state)}), flush=True)
            self.last_output = output

        def report(self):
            return {**core.report(), 'shadow_only': not (bench_ev or handshake_bench or live), 'isolated': isolated,
                    'dds_domains':dict(px4=self.context.get_domain_id(),
                        perception=perception_node.domain_id if perception_node is not None else self.context.get_domain_id()),
                    'task_inputs': self.task_input.report() if self.task_input else None,
                    'task_transport_fault': self.task_transport_fault,
                    'live_routes_enabled': live,
                    'task_read_only':task_preview,'preview_graph':self.preview_graph,
                    'disarmed_handshake_bench': handshake_bench,
                    'dispatch_fault': self.dispatch_guard.fault if self.dispatch_guard else None,
                    'candidate_flight_guard_active': self.candidate_flight_guard is not None,
                    'candidate_flight_guard_fault': (self.candidate_flight_guard.fault
                        if self.candidate_flight_guard else None),
                    'ev_only_bench': bench_ev, 'real_control_publishers_created': 3 if (handshake_bench or live) else 0,
                    'bench_ev_fault': self.bench_gate.fault if self.bench_gate else None,
                    'bench_ev_blockers': self.bench_gate.blockers if self.bench_gate else [],
                    'bench_preflight_continuous_s_max': self.preflight_max_s,
                    'exercise_requested': exercise, 'real_publishers_created': 4 if (handshake_bench or live) else (1 if bench_ev else 0),
                    'output_topics': [] if task_preview else list(self.output_routes.values()),
                    'published': dict(self.counts), 'received': dict(self.input_counts),
                    'flags_timing': list(self.flags_timing),
                    'armed_on_real_input': self.armed_on_real_input,
                    'control_trace': self.control_trace}

    return RuntimeNode()


def main(*, bench_ev=False, handshake=False):
    if bench_ev and handshake:
        raise ValueError('handshake has no real-output mode')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--isolated', action='store_true')
    parser.add_argument('--exercise-controller', action='store_true', help='SHADOW intents only; requires all inputs')
    parser.add_argument('--duration', type=int, default=60)
    parser.add_argument('--aux-params', type=Path,
                        help='explicit current QGC export for derived AUX SHADOW input; no live mode')
    if bench_ev:
        parser.add_argument('--prop-off', action='store_true', required=True,
                            help='operator-confirmed props removed; NOT arm/mode permission')
    args = parser.parse_args()
    if bench_ev and (args.isolated or args.exercise_controller or not args.aux_params):
        parser.error('EV-only bench needs current AUX export; no isolated inputs or controller exercise')
    if not 10 <= args.duration <= 180:
        parser.error('duration must be 10..180 s')
    import yaml
    platform = yaml.safe_load((ROOT/'config/platform.yaml').read_text())
    if platform.get('live_flight_enabled') is not False:
        parser.error('runtime requires explicit live_flight_enabled=false')
    os.environ['ROS_DOMAIN_ID'] = '177' if args.isolated else '0'
    os.environ['ROS_LOCALHOST_ONLY'] = '1' if args.isolated else '0'
    if args.isolated:
        os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
    else:
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = '/home/cfly/ros2_ws/config/fastdds_bridge.xml'
    # Cooperating processes cannot start a second runtime in the same mode.
    # Keep the lock inode; deleting it could permit an overlapping new lock.
    lock_path = '/tmp/robocup_flight_runtime_'+('isolated' if args.isolated else 'shadow')+'.lock'
    with open(lock_path, 'a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error('another runtime owns this session')
        import rclpy
        rclpy.init(args=[])
        aux_params = None
        aux_hash = None
        if args.aux_params:
            from flight_readiness import read_params
            from aux_switch_decoder import validate_contract
            try:
                aux_hash = hashlib.sha256(args.aux_params.read_bytes()).hexdigest()
                aux_params = read_params(args.aux_params)
                validate_contract(aux_params)
                if hashlib.sha256(args.aux_params.read_bytes()).hexdigest() != aux_hash:
                    raise ValueError('parameter file changed during review')
            except (OSError, ValueError) as exc:
                parser.error(str(exc))
        if handshake:
            from handshake_runtime_core import HandshakeRuntimeCore
            core = HandshakeRuntimeCore(time.monotonic(), aux_params=aux_params)
        else:
            core = FlightRuntimeCore(time.monotonic(), aux_params=aux_params)
        node = create_node(core, isolated=args.isolated, exercise=args.exercise_controller, bench_ev=bench_ev)
        error = None
        try:
            end = time.monotonic()+args.duration
            while time.monotonic() < end and not node.armed_on_real_input:
                rclpy.spin_once(node, timeout_sec=.05)
        except KeyboardInterrupt:
            error = 'operator interrupted shadow runtime'
        except Exception as exc:
            error = str(exc)
            core.trip('runtime exception: '+error)
        finally:
            report = {**node.report(), 'error': error, 'aux_parameter_sha256': aux_hash,
                      'aux_parameter_export': str(args.aux_params) if args.aux_params else None}
            prefix = 'handshake_runtime_' if handshake else ('flight_bench_' if bench_ev else 'flight_runtime_')
            out = ROOT/'evidence'/(time.strftime(prefix+'%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:6])
            out.mkdir(parents=True, exist_ok=False)
            (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
            print(json.dumps({'evidence': str(out), **report}, indent=2), flush=True)
            node.destroy_node()
            rclpy.try_shutdown()
    passed = error is None and core.ev.publish_count and not core.ev.inhibit
    if bench_ev:
        passed = passed and node.bench_gate.sent >= 30 and node.preflight_max_s >= 10
    if handshake:
        passed = passed and not core.fault and not core.command_fault
        if args.exercise_controller:
            passed = passed and core.handshake.state == 'DONE'
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
