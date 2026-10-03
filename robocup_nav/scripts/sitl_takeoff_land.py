#!/usr/bin/env python3
"""Offline takeoff-hover-land state machine against the isolated SITL plant.

Safety:
  - Forces ROS_DOMAIN_ID=175 and ROS_LOCALHOST_ONLY=1
  - Refuses if a serial MicroXRCE-DDS / MAVLink agent is attached to a UART
  - Arms only after /robocup/sitl/identity proves this is the SITL plant
  - Airborne timeout / estop / telemetry loss command a controlled land
  - Never opens serial, never publishes vehicle_visual_odometry
  - Not flight validation and not a real PX4 Gazebo SITL binary
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path

SITL_DOMAIN = os.environ.get('SITL_DOMAIN_ID', '175')
os.environ['ROS_DOMAIN_ID'] = SITL_DOMAIN
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from px4_msgs.msg import (
    EstimatorStatusFlags,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String

from sitl_px4_plant import (
    ESTOP_TOPIC,
    IDENTITY_TOPIC,
    SITL_IDENTITY,
    SitlPx4Plant,
)

ROOT = Path(__file__).resolve().parents[1]
SERIAL_HINTS = ('/dev/ttyusb', '/dev/ttyacm', '/dev/ttyths', 'serial', 'multiserial')
ABORT_REASON_TOPIC = '/robocup/sitl/abort_reason'


@dataclass(frozen=True)
class Timeouts:
    wait_plant_s: float = 5.0
    offboard_s: float = 5.0
    arm_s: float = 5.0
    takeoff_s: float = 20.0
    land_s: float = 20.0
    abort_land_s: float = 15.0
    disarm_s: float = 5.0
    mission_s: float = 60.0
    telemetry_s: float = 0.5
    identity_s: float = 1.0
    near_ground_force_disarm_s: float = 2.0


def _px4_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _reliable_latched() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _cmdline(pid: str) -> str:
    try:
        raw = Path(f'/proc/{pid}/cmdline').read_bytes()
    except OSError:
        return ''
    return raw.replace(b'\x00', b' ').decode('utf-8', 'replace').lower()


def serial_flight_bridge_running() -> str | None:
    """Return a reason if a real-FC serial DDS/MAVLink bridge is live."""
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        text = _cmdline(pid)
        if not text:
            continue
        if 'microxrceagent' in text and any(h in text for h in SERIAL_HINTS):
            return f'MicroXRCEAgent serial: {text.strip()[:160]}'
        if 'mavlink' in text and any(h in text for h in ('/dev/ttyusb', '/dev/ttyacm', '/dev/ttyths')):
            return f'mavlink serial: {text.strip()[:160]}'
    return None


def visual_odometry_injection_topics(node: Node) -> list[str]:
    forbidden = []
    for name, _types in node.get_topic_names_and_types():
        if name in (
            '/fmu/in/vehicle_visual_odometry',
            '/fmu/in/vehicle_mocap_odometry',
        ):
            forbidden.append(name)
    return forbidden


class SitlTakeoffLand(Node):
    INIT = 'INIT'
    WAIT_PLANT = 'WAIT_PLANT'
    STREAM = 'STREAM'
    OFFBOARD = 'OFFBOARD'
    ARM = 'ARM'
    TAKEOFF = 'TAKEOFF'
    HOVER = 'HOVER'
    LAND = 'LAND'
    ABORT_LAND = 'ABORT_LAND'
    DISARM = 'DISARM'
    DONE = 'DONE'
    ABORTED = 'ABORTED'
    FAIL = 'FAIL'
    TERMINAL = (DONE, ABORTED, FAIL)

    def __init__(self, target_height_m: float, hover_s: float, done: threading.Event,
                 timeouts: Timeouts | None = None, scenario: str = 'nominal'):
        super().__init__('robocup_sitl_takeoff_land')
        self.target_height_m = float(target_height_m)
        self.hover_s = float(hover_s)
        self.done = done
        self.timeouts = timeouts or Timeouts()
        self.scenario = scenario
        self.state = self.INIT
        self.reason = ''
        self.history = []
        self.trace = []
        self.identity = ''
        self.status = None
        self.local = None
        self.flags = None
        self.status_at = 0.0
        self.local_at = 0.0
        self.identity_at = 0.0
        self.state_entered = time.monotonic()
        self.hover_entered = 0.0
        self.peak_height = 0.0
        self.hover_height = None
        self.started = time.monotonic()
        self.stream_enabled = True
        self.estop = False
        self.abort_reason = ''
        self.inject_armed_at = 0.0
        self._lock = threading.Lock()
        qos = _px4_qos()
        latched = _reliable_latched()
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.estop_pub = self.create_publisher(Bool, ESTOP_TOPIC, latched)
        self.abort_pub = self.create_publisher(String, ABORT_REASON_TOPIC, latched)
        self.create_subscription(Bool, ESTOP_TOPIC, self._on_estop, latched)
        self.create_subscription(
            String, IDENTITY_TOPIC, self._on_identity, latched)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self._on_status, qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._on_local, qos)
        self.create_subscription(
            EstimatorStatusFlags, '/fmu/out/estimator_status_flags',
            self._on_flags, qos)
        self.create_timer(0.05, self._tick)
        self._enter(self.WAIT_PLANT, 'waiting for SITL plant identity')

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds // 1000)

    def _on_identity(self, msg: String):
        self.identity = msg.data.strip()
        self.identity_at = time.monotonic()

    def _on_status(self, msg: VehicleStatus):
        self.status = msg
        self.status_at = time.monotonic()

    def _on_local(self, msg: VehicleLocalPosition):
        self.local = msg
        self.local_at = time.monotonic()
        self.peak_height = max(self.peak_height, float(msg.dist_bottom))
        self.trace.append({
            't': round(time.monotonic() - self.started, 3),
            'state': self.state,
            'armed': self._armed(),
            'nav_state': int(self.status.nav_state) if self.status else None,
            'failsafe': bool(self.status.failsafe) if self.status else False,
            'ned_z': float(msg.z),
            'height_m': float(msg.dist_bottom),
            'stream': self.stream_enabled,
        })

    def _on_flags(self, msg: EstimatorStatusFlags):
        self.flags = msg

    def _on_estop(self, msg: Bool):
        if msg.data:
            with self._lock:
                self.estop = True
                if self.state not in self.TERMINAL + (self.ABORT_LAND, self.DISARM):
                    self._abort('estop')

    def _enter(self, state: str, note: str):
        self.state = state
        self.state_entered = time.monotonic()
        item = {'t': round(time.monotonic() - self.started, 3), 'state': state, 'note': note}
        self.history.append(item)
        self.get_logger().info(f'{state}: {note}')

    def _age(self) -> float:
        return time.monotonic() - self.state_entered

    def _plant_ok(self) -> bool:
        now = time.monotonic()
        return (
            self.identity == SITL_IDENTITY
            and now - self.identity_at <= self.timeouts.identity_s
            and now - self.status_at <= self.timeouts.telemetry_s
            and now - self.local_at <= self.timeouts.telemetry_s
        )

    def _armed(self) -> bool:
        return bool(self.status and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def _offboard(self) -> bool:
        return bool(self.status and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def _landing(self) -> bool:
        return bool(self.status and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND)

    def _height(self) -> float:
        if self.local is None:
            return float('nan')
        return float(self.local.dist_bottom)

    def _airborne(self) -> bool:
        height = self._height()
        return self._armed() and math.isfinite(height) and height > 0.12

    def _stream(self, z_ned: float):
        if not self.stream_enabled:
            return
        hb = OffboardControlMode()
        hb.timestamp = self._now_us()
        hb.position = True
        self.offboard_pub.publish(hb)
        sp = TrajectorySetpoint()
        sp.timestamp = self._now_us()
        sp.position = [0.0, 0.0, float(z_ned)]
        sp.velocity = [math.nan, math.nan, math.nan]
        sp.acceleration = [math.nan, math.nan, math.nan]
        sp.jerk = [math.nan, math.nan, math.nan]
        sp.yaw = 0.0
        sp.yawspeed = math.nan
        self.setpoint_pub.publish(sp)

    def _command(self, command: int, **params):
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = command
        msg.param1 = float(params.get('param1', 0.0))
        msg.param2 = float(params.get('param2', 0.0))
        msg.param3 = float(params.get('param3', 0.0))
        msg.param4 = float(params.get('param4', 0.0))
        msg.param5 = float(params.get('param5', 0.0))
        msg.param6 = float(params.get('param6', 0.0))
        msg.param7 = float(params.get('param7', 0.0))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def _request_land(self):
        self._command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def _force_disarm(self):
        self._command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0, param2=21196.0)

    def _finish(self, state: str, reason: str):
        self.reason = reason
        self._enter(state, reason)
        self.done.set()

    def _fail(self, reason: str):
        if self._airborne() or self.state in (self.TAKEOFF, self.HOVER, self.LAND, self.ABORT_LAND):
            self._abort(reason)
            return
        self._finish(self.FAIL, reason)

    def _abort(self, reason: str):
        if self.state in self.TERMINAL:
            return
        self.abort_reason = reason
        self.abort_pub.publish(String(data=reason))
        if self.state != self.ABORT_LAND:
            self._enter(self.ABORT_LAND, f'controlled land: {reason}')
        self._request_land()

    def _maybe_inject(self):
        if self.inject_armed_at <= 0.0 or not self._armed():
            return
        delay = time.monotonic() - self.inject_armed_at
        if self.scenario == 'estop' and delay >= 1.2 and not self.estop:
            self.get_logger().warn('scenario estop: publishing /robocup/sitl/estop')
            self.estop_pub.publish(Bool(data=True))
            self.estop = True
            self._abort('estop')
        elif self.scenario == 'link-loss' and delay >= 1.2 and self.stream_enabled:
            self.get_logger().warn('scenario link-loss: stopping offboard heartbeat')
            self.stream_enabled = False

    def _handle_abort_land(self):
        height = self._height()
        self._request_land()
        near_ground = math.isfinite(height) and height < 0.15
        if (not self._armed()) and math.isfinite(height) and height < 0.12:
            self._finish(self.ABORTED, f'safe abort on ground: {self.abort_reason}')
            return
        if near_ground and self._age() >= self.timeouts.near_ground_force_disarm_s:
            self._force_disarm()
        if self._age() > self.timeouts.abort_land_s:
            if near_ground or (math.isfinite(height) and height < 0.20):
                self._force_disarm()
                if not self._armed():
                    self._finish(self.ABORTED, f'safe abort after land timeout: {self.abort_reason}')
                    return
            self._finish(self.FAIL, f'abort land timeout height={height:.2f}: {self.abort_reason}')

    def _tick(self):
        with self._lock:
            self._tick_locked()

    def _tick_locked(self):
        if self.state in self.TERMINAL:
            return
        if time.monotonic() - self.started > self.timeouts.mission_s:
            self._fail('mission exceeded timeout cap')
            return
        injected = visual_odometry_injection_topics(self)
        if injected:
            self._fail(f'forbidden vision topics present: {injected}')
            return
        if self._armed() and self.inject_armed_at <= 0.0:
            self.inject_armed_at = time.monotonic()
        self._maybe_inject()

        if self.state == self.WAIT_PLANT:
            if self._plant_ok():
                if self.flags is not None and (
                        self.flags.cs_ev_pos or self.flags.cs_ev_vel
                        or self.flags.cs_ev_hgt or self.flags.cs_ev_yaw):
                    self._fail('SITL plant reported external-vision fusion; aborting')
                    return
                self._enter(self.STREAM, 'plant identity confirmed; streaming offboard setpoints')
            elif self._age() > self.timeouts.wait_plant_s:
                self._fail('SITL plant identity/status not received')
            return

        if self.state == self.ABORT_LAND:
            self._handle_abort_land()
            return

        if not self._plant_ok():
            self._fail('lost SITL plant telemetry')
            return

        if self.estop:
            self._abort('estop')
            return
        if self._landing() and self.state in (self.TAKEOFF, self.HOVER):
            self._abort('plant entered AUTO_LAND')
            return

        if self.state == self.STREAM:
            self._stream(0.0)
            if self._age() >= 2.0:
                self._enter(self.OFFBOARD, 'requesting Offboard')
            return

        if self.state == self.OFFBOARD:
            self._stream(0.0)
            self._command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            if self._offboard():
                self._enter(self.ARM, 'Offboard active; arming SITL only')
            elif self._age() > self.timeouts.offboard_s:
                self._fail('did not enter Offboard')
            return

        if self.state == self.ARM:
            self._stream(0.0)
            self._command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            if self._armed():
                self._enter(self.TAKEOFF, f'armed; climbing to {self.target_height_m:.2f}m')
            elif self._age() > self.timeouts.arm_s:
                self._fail('SITL arm timeout')
            return

        if self.state == self.TAKEOFF:
            self._stream(-self.target_height_m)
            height = self._height()
            if math.isfinite(height) and abs(height - self.target_height_m) < 0.08:
                self.hover_entered = time.monotonic()
                self._enter(self.HOVER, f'reached {height:.2f}m')
            elif self._age() > self.timeouts.takeoff_s:
                self._abort(f'takeoff timeout at {height:.2f}m')
            return

        if self.state == self.HOVER:
            self._stream(-self.target_height_m)
            height = self._height()
            if not math.isfinite(height) or abs(height - self.target_height_m) > 0.15:
                self._abort(f'hover departed target: {height:.2f}m')
                return
            if time.monotonic() - self.hover_entered >= self.hover_s:
                self.hover_height = height
                self._enter(self.LAND, 'hover complete; landing')
            return

        if self.state == self.LAND:
            self._request_land()
            height = self._height()
            if (not self._armed()) and math.isfinite(height) and height < 0.12:
                self._enter(self.DISARM, 'on ground and disarmed by land detector')
            elif self._age() > self.timeouts.land_s:
                self._abort(f'land timeout armed={self._armed()} height={height:.2f}')
            return

        if self.state == self.DISARM:
            if self._armed():
                self._force_disarm()
            if not self._armed():
                self._finish(self.DONE, 'takeoff-hover-land completed on SITL plant')
            elif self._age() > self.timeouts.disarm_s:
                self._fail('force-disarm timeout')


def write_report(path: Path, node: SitlTakeoffLand, serial_reason: str | None,
                 extra: dict | None = None) -> dict:
    safe_abort = node.state == SitlTakeoffLand.ABORTED and not node._armed()
    nominal = node.state == SitlTakeoffLand.DONE and not node._armed()
    passed = nominal if node.scenario == 'nominal' else safe_abort
    report = {
        'test': 'offline_sitl_takeoff_land',
        'scenario': node.scenario,
        'flight_validation': False,
        'real_aircraft': False,
        'px4_sitl_kind': 'px4_dds_contract_plant',
        'gazebo': False,
        'visual_odometry_injected': False,
        'serial_flight_bridge': serial_reason,
        'ros_domain_id': int(os.environ['ROS_DOMAIN_ID']),
        'localhost_only': os.environ.get('ROS_LOCALHOST_ONLY') == '1',
        'sitl_identity': node.identity,
        'passed': passed,
        'final_state': node.state,
        'reason': node.reason,
        'abort_reason': node.abort_reason,
        'target_height_m': node.target_height_m,
        'peak_height_m': round(node.peak_height, 4),
        'hover_height_m': None if node.hover_height is None else round(node.hover_height, 4),
        'elapsed_s': round(time.monotonic() - node.started, 3),
        'timeouts': asdict(node.timeouts),
        'states': node.history,
        'samples': len(node.trace),
        'ev_fusion_bits': None if node.flags is None else {
            'cs_ev_pos': bool(node.flags.cs_ev_pos),
            'cs_ev_vel': bool(node.flags.cs_ev_vel),
            'cs_ev_hgt': bool(node.flags.cs_ev_hgt),
            'cs_ev_yaw': bool(node.flags.cs_ev_yaw),
        },
        'final_armed': node._armed(),
        'final_height_m': None if node.local is None else round(float(node.local.dist_bottom), 4),
    }
    if extra:
        report.update(extra)
    path.write_text(json.dumps(report, indent=2) + '\n')
    (path.parent / 'trace.json').write_text(json.dumps(node.trace, indent=2) + '\n')
    return report


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Offline SITL takeoff/hover/land')
    parser.add_argument('--offline-sitl', action='store_true', required=True,
                        help='Required confirmation: do not talk to the real flight controller')
    parser.add_argument('--height', type=float, default=1.0)
    parser.add_argument('--hover', type=float, default=3.0)
    parser.add_argument('--scenario', choices=('nominal', 'estop', 'link-loss', 'takeoff-timeout'),
                        default='nominal')
    parser.add_argument('--takeoff-timeout', type=float, default=None)
    parser.add_argument('--plant-vz-max', type=float, default=None)
    parser.add_argument('--evidence-dir', default='')
    return parser.parse_args(argv)


def run_once(args) -> dict:
    if os.environ['ROS_DOMAIN_ID'] in {'0', '174', '176'}:
        raise RuntimeError('Refusing ROS_DOMAIN_ID that is default or used by camera/nav stacks.')
    serial_reason = serial_flight_bridge_running()
    if serial_reason:
        raise RuntimeError(f'Refusing to run: {serial_reason}')

    timeouts = Timeouts()
    vz_max = 0.6
    if args.scenario == 'takeoff-timeout':
        timeouts = Timeouts(takeoff_s=args.takeoff_timeout or 1.5, abort_land_s=8.0)
        vz_max = args.plant_vz_max or 0.08
    elif args.takeoff_timeout is not None:
        timeouts = Timeouts(takeoff_s=args.takeoff_timeout)
    if args.plant_vz_max is not None and args.scenario != 'takeoff-timeout':
        vz_max = args.plant_vz_max

    evidence_dir = Path(args.evidence_dir) if args.evidence_dir else (
        ROOT / 'evidence' / time.strftime(f'sitl_{args.scenario}_%Y%m%d_%H%M%S'))
    evidence_dir.mkdir(parents=True, exist_ok=True)

    created_context = False
    if not rclpy.ok():
        rclpy.init()
        created_context = True
    done = threading.Event()
    plant = SitlPx4Plant(vz_max=vz_max)
    node = SitlTakeoffLand(args.height, args.hover, done, timeouts=timeouts,
                           scenario=args.scenario)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    executor.add_node(plant)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    wait_s = max(args.hover + 15.0, timeouts.mission_s + 5.0)
    finished = done.wait(timeout=wait_s)
    if not finished:
        with node._lock:
            node._fail('executor wait timeout')
        done.wait(timeout=2.0)

    report = write_report(evidence_dir / 'report.json', node, serial_reason, extra={
        'evidence': str(evidence_dir),
        'plant_vz_max': vz_max,
    })
    executor.shutdown()
    node.destroy_node()
    plant.destroy_node()
    if created_context and rclpy.ok():
        rclpy.shutdown()
    return report


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        report = run_once(args)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    print(json.dumps({
        'passed': report['passed'],
        'scenario': report['scenario'],
        'final_state': report['final_state'],
        'reason': report['reason'],
        'abort_reason': report['abort_reason'],
        'peak_height_m': report['peak_height_m'],
        'hover_height_m': report['hover_height_m'],
        'elapsed_s': report['elapsed_s'],
        'evidence': report.get('evidence'),
        'flight_validation': False,
    }, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
