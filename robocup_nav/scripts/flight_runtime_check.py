#!/usr/bin/env python3
"""Exercise the actual integrated runtime with a DDS synthetic plant.

Domain 177 and fixed /robocup/runtime_test topics only. Not PX4 SITL.
All safety observations traverse subscriptions; no direct healthy snapshot.
"""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import time
import uuid

os.environ['ROS_DOMAIN_ID'] = '177'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as qos
from px4_msgs.msg import (VehicleStatus, VehicleLocalPosition, EstimatorStatusFlags,
    VehicleLandDetected, ManualControlSetpoint, ManualControlSwitches, VehicleCommandAck,
    FailsafeFlags, VehicleOdometry, OffboardControlMode, TrajectorySetpoint, VehicleCommand)
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from flight_runtime import create_node, topics
from flight_runtime_core import FlightRuntimeCore

SCENARIOS = ('nominal', 'vision-loss', 'link-loss', 'manual-takeover', 'kill',
             'ack-denied', 'battery-warning', 'missing-switches',
             'ownership-conflict', 'ack-without-state', 'slow-switches',
             'switch-loss', 'frozen-switch-sample', 'position-slot', 'prearm-takeover')
AUX_EXTRA = ('aux-non-rc', 'aux-nan', 'aux-boundary')


def synthetic_aux_params():
    # Isolated fixture ONLY. Real-input CLI requires an actual export.
    return {'RC_MAP_AUX1': 6, 'RC_MAP_AUX2': 5, 'RC_MAP_FLTMODE': 6,
            'RC_MAP_KILL_SW': 5, 'RC_MAP_OFFB_SW': 0, 'RC_MAP_FLTM_BTN': 0,
            'RC_KILLSWITCH_TH': .75, 'COM_RC_IN_MODE': 0,
            **{f'COM_FLTMODE{i}': 2 if i < 6 else 7 for i in range(1, 7)}}


class SyntheticPlant(Node):
    def __init__(self, scenario, use_aux=False):
        super().__init__('runtime_test_plant', use_global_arguments=False,
                         enable_rosout=False, start_parameter_services=False)
        self.scenario = scenario
        self.use_aux = use_aux
        self.inputs, self.output = topics(True)
        classes = (('vehicle_status_v1', VehicleStatus), ('vehicle_local_position', VehicleLocalPosition),
                   ('estimator_status_flags', EstimatorStatusFlags), ('vehicle_land_detected', VehicleLandDetected),
                   ('manual_control_setpoint', ManualControlSetpoint), ('manual_control_switches', ManualControlSwitches),
                   ('vehicle_command_ack', VehicleCommandAck), ('failsafe_flags', FailsafeFlags),
                   ('vio', Odometry), ('tracking', String))
        self.pubs = {k: self.create_publisher(cls, self.inputs[k], 10 if k == 'tracking' else qos)
                     for k, cls in classes if not (use_aux and k == 'manual_control_switches')}
        self.armed = False
        self.mode = 2
        self.z = self.target_z = .7
        self.vz = 0.
        self.last_hb = -math.inf
        self.last_ev = -math.inf
        self.armed_at = None
        self.injected_at = None
        self.landing = False
        self.killed = False
        self.counts = Counter()
        self.commands = []
        self.ev_after_fault = 0
        self.conflicting_pub = None
        self.first_heartbeat_at = None
        self.switch_at = -math.inf
        self.switch_sample = None
        self.create_subscription(OffboardControlMode, self.output+'/offboard_control_mode', self.heartbeat, qos)
        self.create_subscription(TrajectorySetpoint, self.output+'/trajectory_setpoint', self.setpoint, qos)
        self.create_subscription(VehicleCommand, self.output+'/vehicle_command', self.command, qos)
        self.create_subscription(VehicleOdometry, self.output+'/vehicle_visual_odometry', self.ev, qos)
        self.create_timer(.02, self.tick)

    def drop_link(self):
        return self.scenario == 'link-loss' and self.injected_at is not None

    def heartbeat(self, m):
        self.counts['heartbeat'] += 1
        if self.first_heartbeat_at is None:
            self.first_heartbeat_at = time.monotonic()
        if not self.drop_link():
            self.last_hb = time.monotonic()

    def setpoint(self, m):
        self.counts['setpoint'] += 1
        if not self.drop_link():
            assert abs(m.position[0]-2) < 1e-6 and abs(m.position[1]+3) < 1e-6
            assert abs(m.yaw-.8) < 1e-6 and all(math.isnan(v) for v in m.velocity)
            self.target_z = float(m.position[2])

    def ev(self, m):
        self.counts['ev'] += 1
        if self.scenario == 'vision-loss' and self.injected_at is not None:
            self.ev_after_fault += 1
        if not self.drop_link():
            self.last_ev = time.monotonic()
        assert m.pose_frame == 2 and all(math.isnan(v) for v in m.velocity)

    def command(self, m):
        if self.drop_link():
            return
        assert m.param2 != 21196
        self.commands.append({'at': time.monotonic(), 'command': int(m.command),
                              'source_component': int(m.source_component)})
        result = 0
        if m.command == 176:
            if self.scenario == 'ack-denied':
                result = 2
            elif self.scenario != 'ack-without-state':
                self.mode = 14
        elif m.command == 400:
            assert m.param1 == 1
            if self.mode == 14:
                self.armed = True
                self.armed_at = time.monotonic()
            else:
                result = 2
        elif m.command == 21:
            assert math.isnan(m.param5) and math.isnan(m.param6)
            self.landing, self.mode = True, 18
        ack = VehicleCommandAck()
        ack.timestamp = self.get_clock().now().nanoseconds//1000
        ack.command, ack.result = m.command, result
        ack.target_system, ack.target_component = int(m.source_system), int(m.source_component)
        if not hasattr(self, 'filter_ack') or self.filter_ack(ack):
            self.pubs['vehicle_command_ack'].publish(ack)

    def publish(self, name, cls, stamp, **fields):
        msg = cls()
        msg.timestamp = stamp
        for name_field, value in fields.items():
            setattr(msg, name_field, value)
        self.pubs[name].publish(msg)

    def tick(self):
        now = time.monotonic()
        if (self.scenario == 'prearm-takeover' and self.first_heartbeat_at is not None
                and now-self.first_heartbeat_at > .4 and self.injected_at is None):
            self.injected_at = now
        if self.armed_at is not None and now-self.armed_at > 1 and self.injected_at is None:
            self.injected_at = now
        if self.injected_at is not None:
            if self.scenario == 'manual-takeover':
                self.mode = 2
            elif self.scenario == 'kill':
                self.killed, self.armed = True, False
            elif self.scenario == 'ownership-conflict' and self.conflicting_pub is None:
                self.conflicting_pub = self.create_publisher(
                    OffboardControlMode, self.output+'/offboard_control_mode', qos)
        if self.armed and self.mode == 14 and now-self.last_hb > 1:
            self.landing, self.mode = True, 18  # Explicitly synthetic onboard fallback.
        if self.armed:
            target = .7 if self.landing else self.target_z
            delta = max(-.004, min(.012, target-self.z))
            self.z += delta
            self.vz = delta/.02
        if self.landing and abs(self.z-.7) < .005:
            self.armed, self.vz = False, 0.
        stamp_ns = self.get_clock().now().nanoseconds
        stamp = stamp_ns//1000
        lost_vision = self.scenario == 'vision-loss' and self.injected_at is not None
        # VIO continues on link loss: only FC transport is disconnected.
        self.pubs['tracking'].publish(String(data=json.dumps({'vo_state': 2 if lost_vision else 1,
                                                             'stamp_ns': stamp_ns})))
        pose = Odometry()
        pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(stamp_ns, 10**9)
        pose.header.frame_id, pose.child_frame_id = 'odom', 'base_link'
        pose.pose.pose.position.z = .7-self.z
        pose.pose.pose.orientation.w = 1.
        self.pubs['vio'].publish(pose)
        if self.drop_link():
            return
        ev_valid = now-self.last_ev < .3
        self.publish('vehicle_status_v1', VehicleStatus, stamp, arming_state=2 if self.armed else 1,
                     nav_state=self.mode, pre_flight_checks_pass=ev_valid, failsafe=False)
        self.publish('vehicle_local_position', VehicleLocalPosition, stamp, x=2., y=-3., z=self.z,
                     heading=.8, vx=0., vy=0., vz=self.vz, xy_valid=ev_valid, z_valid=True,
                     v_xy_valid=ev_valid, v_z_valid=True)
        self.publish('estimator_status_flags', EstimatorStatusFlags, stamp, cs_ev_pos=ev_valid,
                     cs_ev_hgt=ev_valid, cs_ev_yaw=ev_valid, cs_ev_vel=False,
                     cs_baro_hgt=True, cs_rng_hgt=False)
        self.publish('vehicle_land_detected', VehicleLandDetected, stamp, landed=not self.armed and self.z >= .695)
        position = self.scenario == 'position-slot' or (
            self.scenario in ('manual-takeover', 'prearm-takeover') and self.injected_at is not None)
        position = position or getattr(self, 'operator_position', False)
        if self.use_aux:
            frozen = self.scenario == 'frozen-switch-sample' and self.injected_at is not None
            if not frozen or self.switch_sample is None:
                self.switch_sample = stamp-20_000
            missing = self.scenario == 'missing-switches' or (
                self.scenario == 'switch-loss' and self.injected_at is not None)
            if not missing:
                faulty = self.injected_at is not None
                self.publish('manual_control_setpoint', ManualControlSetpoint, stamp, valid=True,
                             timestamp_sample=self.switch_sample,
                             data_source=2 if faulty and self.scenario == 'aux-non-rc' else 1,
                             aux1=math.nan if faulty and self.scenario == 'aux-nan' else (-1. if position else 1.),
                             aux2=.5 if faulty and self.scenario == 'aux-boundary' else (1. if self.killed else -1.))
        else:
            self.publish('manual_control_setpoint', ManualControlSetpoint, stamp, valid=True)
        switch_due = self.scenario != 'slow-switches' or now-self.switch_at >= 1.
        switch_lost = self.scenario == 'switch-loss' and self.injected_at is not None
        if not self.use_aux and self.scenario != 'missing-switches' and switch_due and not switch_lost:
            frozen = self.scenario == 'frozen-switch-sample' and self.injected_at is not None
            if not frozen or self.switch_sample is None:
                self.switch_sample = stamp-20_000
            position = self.scenario == 'position-slot' or (
                self.scenario in ('manual-takeover', 'prearm-takeover') and self.injected_at is not None)
            self.publish('manual_control_switches', ManualControlSwitches, stamp,
                         timestamp_sample=self.switch_sample,
                         mode_slot=1 if position else 6,
                         kill_switch=1 if self.killed else 3)
            self.switch_at = now
        self.publish('failsafe_flags', FailsafeFlags, stamp,
                     battery_warning=2 if self.scenario == 'battery-warning' else 0)


def run(scenario, use_aux=False):
    core = FlightRuntimeCore(time.monotonic(), aux_params=synthetic_aux_params() if use_aux else None)
    runtime = create_node(core, isolated=True, exercise=True)
    plant = SyntheticPlant(scenario, use_aux=use_aux)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(runtime)
    executor.add_node(plant)
    started = time.monotonic()
    try:
        while time.monotonic()-started < 25:
            executor.spin_once(timeout_sec=.02)
            if core.controller.state in core.controller.TERMINAL:
                # Drain queued output so command suppression can be asserted.
                until = time.monotonic()+.3
                while time.monotonic() < until:
                    executor.spin_once(timeout_sec=.01)
                break
            if scenario in ('missing-switches', 'position-slot') and time.monotonic()-started > 4:
                break
            if (scenario == 'link-loss' and core.controller.state == 'ABORT' and not plant.armed
                    and time.monotonic()-plant.injected_at > 2.3):
                break
        expected = {'nominal': 'DONE', 'battery-warning': 'DONE', 'vision-loss': 'STOPPED',
                    'link-loss': 'ABORT', 'manual-takeover': 'HANDOVER', 'kill': 'KILLED',
                    'ack-denied': 'STOPPED', 'missing-switches': 'WAIT',
                    'ownership-conflict': 'HANDOVER', 'ack-without-state': 'STOPPED',
                    'slow-switches': 'DONE', 'switch-loss': 'STOPPED',
                    'frozen-switch-sample': 'STOPPED', 'position-slot': 'WAIT',
                    'prearm-takeover': 'HANDOVER', 'aux-non-rc': 'STOPPED',
                    'aux-nan': 'STOPPED', 'aux-boundary': 'STOPPED'}[scenario]
        passed = core.controller.state == expected
        if scenario in ('ack-denied', 'missing-switches', 'ack-without-state', 'position-slot', 'prearm-takeover'):
            passed &= not any(c['command'] == 400 for c in plant.commands)
        if scenario in ('manual-takeover', 'kill', 'ownership-conflict', 'prearm-takeover'):
            passed &= not any(c['command'] in (400, 21) and c['at'] > plant.injected_at+.1 for c in plant.commands)
        if scenario == 'vision-loss':
            passed &= plant.ev_after_fault <= 1  # One already queued before fault can be delivered.
        if scenario == 'link-loss':
            passed &= core.ev.inhibit
        report = {'scenario': scenario, 'passed': bool(passed), 'expected': expected,
                  'elapsed_s': time.monotonic()-started, 'runtime': runtime.report(),
                  'plant_received': dict(plant.counts), 'plant_commands': plant.commands,
                  'ev_after_fault': plant.ev_after_fault, 'synthetic_plant': True,
                  'real_px4_sitl': False, 'synthetic_plant_armed': plant.armed}
    finally:
        executor.shutdown()
        runtime.destroy_node()
        plant.destroy_node()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=SCENARIOS+AUX_EXTRA, action='append')
    parser.add_argument('--switch-source', choices=('native', 'aux'), default='native')
    args = parser.parse_args()
    use_aux = args.switch_source == 'aux'
    scenarios = args.scenario or ([s for s in SCENARIOS if s != 'slow-switches']+list(AUX_EXTRA)
                                  if use_aux else SCENARIOS)
    if (use_aux and 'slow-switches' in scenarios) or (not use_aux and any(s in AUX_EXTRA for s in scenarios)):
        parser.error('scenario not applicable to selected switch source')
    rclpy.init(args=[])
    results = []
    try:
        for scenario in scenarios:
            result = run(scenario, use_aux=use_aux)
            results.append(result)
            print(json.dumps({'scenario': scenario, 'passed': result['passed'],
                              'state': result['runtime']['state'],
                              'ev_fault': result['runtime']['ev_fault']}), flush=True)
    finally:
        rclpy.try_shutdown()
    out = Path(__file__).resolve().parents[1]/'evidence'/(time.strftime('flight_runtime_check_%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:6])
    out.mkdir(parents=True, exist_ok=False)
    report = {'passed': all(r['passed'] for r in results), 'flight_validation': False,
              'switch_source': args.switch_source,
              'real_px4_sitl': False, 'scenarios': results}
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'evidence': str(out), 'passed': report['passed']}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
