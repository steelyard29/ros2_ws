#!/usr/bin/env python3
"""PX4 uXRCE-DDS contract SITL plant. No Gazebo, no serial, no vision pose.

Publishes /fmu/out/* and consumes /fmu/in offboard/setpoint/command only.
Never publishes /fmu/in/vehicle_visual_odometry.
"""
from __future__ import annotations

import math
import time

from px4_msgs.msg import (
    EstimatorStatusFlags,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleOdometry,
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

SITL_IDENTITY = 'OFFLINE_PX4_DDS_SITL_PLANT_V1'
IDENTITY_TOPIC = '/robocup/sitl/identity'
ESTOP_TOPIC = '/robocup/sitl/estop'
PLANT_FAILSAFE_TOPIC = '/robocup/sitl/plant_failsafe'


def _px4_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class SitlPx4Plant(Node):
    """First-order position plant that speaks PX4 ROS2 topics."""

    def __init__(self, dt: float = 0.02, vz_max: float = 0.6,
                 heartbeat_timeout_s: float = 0.6):
        super().__init__('robocup_sitl_px4_plant')
        self.dt = dt
        self.t0 = time.monotonic()
        self.ned = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.armed = False
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        self.failsafe = False
        self.failsafe_reason = ''
        self.setpoint = None
        self.setpoint_at = 0.0
        self.heartbeat_at = 0.0
        self.land_requested = False
        self.vxy_max = 1.0
        self.vz_max = float(vz_max)
        self.heartbeat_timeout_s = float(heartbeat_timeout_s)
        self.tau = 0.45

        qos = _px4_qos()
        ident_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.identity_pub = self.create_publisher(String, IDENTITY_TOPIC, ident_qos)
        self.status_pub = self.create_publisher(
            VehicleStatus, '/fmu/out/vehicle_status_v1', qos)
        self.local_pub = self.create_publisher(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', qos)
        self.odom_pub = self.create_publisher(
            VehicleOdometry, '/fmu/out/vehicle_odometry', qos)
        self.att_pub = self.create_publisher(
            VehicleAttitude, '/fmu/out/vehicle_attitude', qos)
        self.flags_pub = self.create_publisher(
            EstimatorStatusFlags, '/fmu/out/estimator_status_flags', qos)
        self.failsafe_pub = self.create_publisher(
            String, PLANT_FAILSAFE_TOPIC, ident_qos)

        self.create_subscription(
            Bool, ESTOP_TOPIC, self._on_estop, ident_qos)
        self.create_subscription(
            OffboardControlMode, '/fmu/in/offboard_control_mode',
            self._on_heartbeat, qos)
        self.create_subscription(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
            self._on_setpoint, qos)
        self.create_subscription(
            VehicleCommand, '/fmu/in/vehicle_command',
            self._on_command, qos)

        self.create_timer(dt, self._tick)
        self.identity_pub.publish(String(data=SITL_IDENTITY))
        self.get_logger().info(
            'SITL plant online: isolated PX4 DDS contract, no serial, no EV')

    def _now_us(self) -> int:
        return int((time.monotonic() - self.t0) * 1e6)

    def _fresh(self, stamp: float, timeout: float = 0.5) -> bool:
        return stamp > 0.0 and (time.monotonic() - stamp) <= timeout

    def _trigger_failsafe(self, reason: str):
        if not self.failsafe:
            self.get_logger().warn(f'plant failsafe: {reason}')
        self.failsafe = True
        self.failsafe_reason = reason
        self.land_requested = True
        if self.armed:
            self.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LAND

    def _on_estop(self, msg: Bool):
        if msg.data:
            self._trigger_failsafe('estop')

    def _on_heartbeat(self, _msg: OffboardControlMode):
        self.heartbeat_at = time.monotonic()

    def _on_setpoint(self, msg: TrajectorySetpoint):
        self.setpoint = msg
        self.setpoint_at = time.monotonic()

    def _on_command(self, msg: VehicleCommand):
        if msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
            if abs(msg.param1 - 1.0) < 1e-3 and abs(msg.param2 - 6.0) < 1e-3:
                if self.failsafe:
                    return
                if self._fresh(self.heartbeat_at):
                    self.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
                    self.land_requested = False
        elif msg.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            if msg.param1 >= 0.5:
                if self.failsafe:
                    return
                if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                        and self._fresh(self.heartbeat_at)
                        and self._fresh(self.setpoint_at)):
                    self.armed = True
            else:
                force = abs(msg.param2 - 21196.0) < 1e-3
                if force or abs(self.ned[2]) < 0.15:
                    self.armed = False
                    self.land_requested = False
                    if not self.armed:
                        self.failsafe = False
                    if self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        elif msg.command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
            self.land_requested = True
            self.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LAND

    def _track_towards(self, target, vmax):
        err = [target[i] - self.ned[i] for i in range(3)]
        vel = [err[i] / self.tau for i in range(3)]
        horiz = math.hypot(vel[0], vel[1])
        if horiz > vmax[0] > 0.0:
            scale = vmax[0] / horiz
            vel[0] *= scale
            vel[1] *= scale
        vel[2] = max(-vmax[1], min(vmax[1], vel[2]))
        self.vel = vel
        self.ned = [self.ned[i] + vel[i] * self.dt for i in range(3)]
        if self.ned[2] > 0.0:
            self.ned[2] = 0.0
            self.vel[2] = 0.0

    def _tick(self):
        self.identity_pub.publish(String(data=SITL_IDENTITY))
        if (self.armed
                and self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and not self._fresh(self.heartbeat_at, self.heartbeat_timeout_s)):
            self._trigger_failsafe(
                f'offboard heartbeat lost >{self.heartbeat_timeout_s:.2f}s')

        if not self.armed:
            self._track_towards([self.ned[0], self.ned[1], 0.0], (0.4, 0.8))
            self.vel = [0.0, 0.0, 0.0]
            self.ned[2] = 0.0
        elif self.land_requested or self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            self.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LAND
            self._track_towards([self.ned[0], self.ned[1], 0.0], (0.3, 0.4))
            if abs(self.ned[2]) < 0.08:
                self.ned[2] = 0.0
                self.vel = [0.0, 0.0, 0.0]
                self.armed = False
                self.land_requested = False
                self.failsafe = False
                self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        elif (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
              and self._fresh(self.setpoint_at) and self.setpoint is not None):
            target = list(self.ned)
            sp = self.setpoint.position
            for i in range(3):
                if math.isfinite(sp[i]):
                    target[i] = float(sp[i])
            if math.isfinite(self.setpoint.yaw):
                self.yaw = float(self.setpoint.yaw)
            self._track_towards(target, (self.vxy_max, self.vz_max))
        else:
            self.vel = [0.0, 0.0, 0.0]

        self._publish()

    def _publish(self):
        now = self._now_us()
        height = max(0.0, -self.ned[2])
        q = [math.cos(self.yaw / 2.0), 0.0, 0.0, math.sin(self.yaw / 2.0)]

        status = VehicleStatus()
        status.timestamp = now
        status.arming_state = (
            VehicleStatus.ARMING_STATE_ARMED if self.armed
            else VehicleStatus.ARMING_STATE_DISARMED)
        status.nav_state = self.nav_state
        status.failsafe = self.failsafe
        status.pre_flight_checks_pass = True
        self.status_pub.publish(status)
        self.failsafe_pub.publish(String(data=self.failsafe_reason))

        local = VehicleLocalPosition()
        local.timestamp = now
        local.timestamp_sample = now
        local.xy_valid = True
        local.z_valid = True
        local.v_xy_valid = True
        local.v_z_valid = True
        local.x, local.y, local.z = (float(v) for v in self.ned)
        local.vx, local.vy, local.vz = (float(v) for v in self.vel)
        local.heading = float(self.yaw)
        local.heading_good_for_control = True
        local.dist_bottom_valid = True
        local.dist_bottom = float(height)
        local.dist_bottom_sensor_bitfield = (
            VehicleLocalPosition.DIST_BOTTOM_SENSOR_RANGE)
        local.eph = 0.02
        local.epv = 0.03
        local.dead_reckoning = False
        local.vxy_max = float('inf')
        local.vz_max = float('inf')
        local.hagl_min = 0.0
        local.hagl_max_z = float('inf')
        local.hagl_max_xy = float('inf')
        self.local_pub.publish(local)

        odom = VehicleOdometry()
        odom.timestamp = now
        odom.timestamp_sample = now
        odom.pose_frame = VehicleOdometry.POSE_FRAME_NED
        odom.position = [float(v) for v in self.ned]
        odom.q = [float(v) for v in q]
        odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        odom.velocity = [float(v) for v in self.vel]
        odom.angular_velocity = [0.0, 0.0, 0.0]
        odom.quality = 100
        self.odom_pub.publish(odom)

        att = VehicleAttitude()
        att.timestamp = now
        att.timestamp_sample = now
        att.q = [float(v) for v in q]
        self.att_pub.publish(att)

        flags = EstimatorStatusFlags()
        flags.timestamp = now
        flags.timestamp_sample = now
        flags.cs_tilt_align = True
        flags.cs_yaw_align = True
        flags.cs_gnss_pos = True
        flags.cs_baro_hgt = True
        flags.cs_rng_hgt = True
        flags.cs_in_air = self.armed and height > 0.12
        flags.cs_vehicle_at_rest = (not self.armed) and height < 0.05
        # Explicitly not fusing external vision in this offline plant.
        flags.cs_ev_pos = False
        flags.cs_ev_vel = False
        flags.cs_ev_hgt = False
        flags.cs_ev_yaw = False
        flags.cs_inertial_dead_reckoning = False
        self.flags_pub.publish(flags)
