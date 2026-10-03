#!/usr/bin/env python3
"""
一键起飞/降落交互控制 (ROS2 Humble + PX4 uXRCE-DDS)

用法:
    ros2 launch uav_task takeoff_control.launch.py

操作:
    按 1 → 起飞至机体中心离地悬停（默认 0.5m，慢爬验收）
            (TFmini 测距目标 ≈ 中心AGL − 0.05m)
    再按 1 → 降落

环境变量（可选）:
    TAKEOFF_CENTER_AGL=0.5     机体中心目标离地高度 m（恢复 1.5m 试飞时改回）
    TAKEOFF_CLIMB_RATE=0.15    爬升速度 m/s

架构 (Layer 3 — 决策规划与控制层):
    订阅 Layer 1 标准化接口 /uav/state/* (ENU 坐标系)
    发布 setpoint 到 /fmu/in/* (NED 坐标系, PX4 原生协议)
"""

import json
import os
import sys
import time
import threading
import math

# 漂移验收默认：低高度 + 慢爬升；可用环境变量覆盖
TAKEOFF_CENTER_AGL_M = float(os.environ.get('TAKEOFF_CENTER_AGL', '1.0'))
TAKEOFF_CLIMB_RATE_MPS = float(os.environ.get('TAKEOFF_CLIMB_RATE', '0.25'))
# 单段直接爬升（删除了两段式）
TWO_STAGE_CLIMB = False
TWO_STAGE_MIDPOINT_RATIO = float(os.environ.get('TWO_STAGE_MIDPOINT_RATIO', '0.5'))
TWO_STAGE_PAUSE_S = float(os.environ.get('TWO_STAGE_PAUSE_S', '3.0'))
# 起飞初始 kick: 斜坡从 origin_z 起步时推力不足，需要最小离地 setpoint
TAKEOFF_KICK_M = float(os.environ.get('TAKEOFF_KICK_M', '0.3'))

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from px4_msgs.msg import (
    ActuatorMotors,
    ActuatorOutputs,
    EstimatorStatusFlags,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleAttitude,
    VehicleLocalPosition,
    VehicleOdometry,
)
from px4_interface.msg import VehicleStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


# ═══════════════════════════════════════════════════════════════
#  坐标转换工具 (ENU ← → NED)
# ═══════════════════════════════════════════════════════════════

def enu_to_ned(enu_x, enu_y, enu_z):
    """ENU → NED: ned_x=enu_y, ned_y=enu_x, ned_z=-enu_z"""
    return (enu_y, enu_x, -enu_z)


def _fmt_rad_deg(value: float) -> str:
    if math.isnan(value):
        return 'nan'
    return f'{value:.3f}rad/{math.degrees(value):.1f}deg'


def _fmt_motor_values(values, prefix: str) -> str:
    if not values:
        return f'{prefix}: 无数据'
    fmt = '{:.2f}' if max(abs(float(v)) for v in values) <= 1.5 else '{:.0f}'
    return (
        f'{prefix}: M1={fmt.format(values[0])}, M2={fmt.format(values[1])}, '
        f'M3={fmt.format(values[2])}, M4={fmt.format(values[3])}'
    )


def print_takeoff_diagnostics(node, origin_x: float, origin_y: float,
                              origin_z: float, label: str):
    dx = node.pos_x - origin_x
    dy = node.pos_y - origin_y
    dz = node.pos_z - origin_z
    xy_error = math.hypot(dx, dy)
    print(
        f'  [{label}] dE={dx:+.2f}m, dN={dy:+.2f}m, '
        f'dZ={dz:+.2f}m, XY误差={xy_error:.2f}m'
    )
    print(
        f'        姿态: roll={_fmt_rad_deg(node.roll_ned)}, '
        f'pitch={_fmt_rad_deg(node.pitch_ned)}, '
        f'yaw={_fmt_rad_deg(node.yaw_ned)}'
    )
    if node.actuator_outputs:
        print(f'        {_fmt_motor_values(node.actuator_outputs, "PWM输出")}')
    if node.actuator_motors:
        print(f'        {_fmt_motor_values(node.actuator_motors, "电机控制")}')
    return xy_error, dz


# ═══════════════════════════════════════════════════════════════
#  PX4 飞行控制器 (Layer 3)
# ═══════════════════════════════════════════════════════════════

class PX4Controller(Node):
    """PX4 Offboard 控制器

    输入:  /uav/state/pose   (ENU, geometry_msgs/PoseStamped)
           /uav/state/status (px4_interface/VehicleStatus)
    输出:  /fmu/in/offboard_control_mode
           /fmu/in/trajectory_setpoint   (NED)
           /fmu/in/vehicle_command
    """

    def __init__(self):
        super().__init__('takeoff_control_node')

        # QoS: 与 PX4 官方示例一致
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 发布者 (→ Layer 1 / PX4)
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # 订阅者 (← Layer 1 标准化接口)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/uav/state/pose',
            self._pose_cb,
            10,
        )
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/uav/state/status',
            self._status_cb,
            10,
        )
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self._odom_cb,
            qos,
        )
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._local_position_cb,
            qos,
        )
        self.estimator_flags_sub = self.create_subscription(
            EstimatorStatusFlags,
            '/fmu/out/estimator_status_flags',
            self._estimator_flags_cb,
            qos,
        )
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self._attitude_cb,
            qos,
        )
        self.actuator_outputs_sub = self.create_subscription(
            ActuatorOutputs,
            '/fmu/out/actuator_outputs',
            self._actuator_outputs_cb,
            qos,
        )
        self.actuator_motors_sub = self.create_subscription(
            ActuatorMotors,
            '/fmu/out/actuator_motors',
            self._actuator_motors_cb,
            qos,
        )

        # VSLAM 里程计监控 (用于检测跟踪质量退化)
        # Isaac VSLAM 发布 VOLATILE；不可复用 PX4 的 TRANSIENT_LOCAL，否则 QoS 不匹配收不到数。
        qos_vslam = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._vslam_odom_received_at = 0.0
        self._vslam_odom_count = 0
        self._vslam_odom_hz = 0.0
        self._vslam_odom_sub = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self._vslam_odom_cb,
            qos_vslam,
        )
        self._rtabmap_relay_received_at = 0.0
        self._rtabmap_status_received_at = 0.0
        self._rtabmap_status_healthy = False
        self._rtabmap_status_text = ''
        self._rtabmap_good_at = 0.0
        self._rtabmap_relay_sub = self.create_subscription(
            Odometry, '/rtabmap/relay/odometry',
            self._rtabmap_relay_cb, qos_vslam)
        self._rtabmap_status_sub = self.create_subscription(
            String, '/rtabmap/relay/status',
            self._rtabmap_status_cb, 10)

        # 状态 (ENU 坐标系)
        self._pose = PoseStamped()
        self._status = VehicleStatus()
        self._has_pose = False
        self._first_pose_received_at = 0.0
        self._pose_received_at = 0.0
        self._last_pose_jump_at = 0.0
        self._pending_pose_jump_z = 0.0
        self._roll_ned = float('nan')
        self._pitch_ned = float('nan')
        self._yaw_ned = float('nan')
        self._horizontal_speed_m_s = float('nan')
        self._actuator_outputs = None
        self._actuator_motors = None
        self._require_valid_rangefinder = self.declare_parameter(
            'require_valid_rangefinder', True).value
        self._range_timeout_s = float(self.declare_parameter(
            'range_timeout_s', 0.5).value)
        self._rangefinder_loss_timeout_s = float(self.declare_parameter(
            'rangefinder_loss_timeout_s', 10.0).value)
        self._air_pose_loss_timeout_s = float(self.declare_parameter(
            'air_pose_loss_timeout_s', 1.0).value)
        self._ev_fusion_loss_timeout_s = float(self.declare_parameter(
            'ev_fusion_loss_timeout_s', 2.0).value)
        self._require_rtabmap_relay_if_enabled = bool(self.declare_parameter(
            'require_rtabmap_relay_if_enabled', True).value)
        self._rtabmap_relay_timeout_s = float(self.declare_parameter(
            'rtabmap_relay_timeout_s', 0.5).value)
        self._rtabmap_status_timeout_s = float(self.declare_parameter(
            'rtabmap_status_timeout_s', 2.0).value)
        self._rtabmap_loss_timeout_s = float(self.declare_parameter(
            'rtabmap_loss_timeout_s', 0.75).value)
        self._min_vslam_odom_hz = float(self.declare_parameter(
            'min_vslam_odom_hz', 8.0).value)
        self._estimator_flags_timeout_s = float(self.declare_parameter(
            'estimator_flags_timeout_s', 2.5).value)
        self._range_received_at = 0.0
        self._range_good_at = 0.0
        self._flight_monitor_started_at = 0.0
        self._dist_bottom_valid = False
        self._dist_bottom = float('nan')
        self._estimator_flags_received_at = 0.0
        self._estimator_flags_seen = False
        self._cs_rng_hgt = False
        self._cs_ev_pos = False
        self._cs_ev_hgt = False
        self._cs_ev_vel = False
        self._cs_inertial_dead_reckoning = False
        self._rng_fusion_lost_since = 0.0
        self._ev_fusion_lost_since = 0.0
        self._min_dist_bottom_m = float(self.declare_parameter(
            'min_dist_bottom_m', 0.05).value)
        # PX4 deliberately reports dist_bottom_valid=false below its HAGL
        # minimum (observed: 0.10 m raw range vs hagl_min=0.40 m).  That is
        # acceptable on the ground, but not after the commanded vehicle has
        # crossed the valid measurement envelope.  Keep takeoff possible from
        # the ground, then require the independent range aid to become valid
        # before continuing the climb.
        self._range_validity_gate_m = float(self.declare_parameter(
            'range_validity_gate_m', 0.45).value)
        self._range_measurement_min_m = float(self.declare_parameter(
            'range_measurement_min_m', 0.35).value)
        self._range_validity_timeout_s = float(self.declare_parameter(
            'range_validity_timeout_s', 2.0).value)
        self._range_validity_lost_since = 0.0
        # PX4/CG 中心相对 TFmini 测距点的高度差 (中心更高为正)
        self._center_above_sensor_m = float(self.declare_parameter(
            'center_above_sensor_m', 0.05).value)
        self._stationary_check_s = float(self.declare_parameter(
            'stationary_check_s', 5.0).value)
        self._max_ground_xy_drift_m = float(self.declare_parameter(
            'max_ground_xy_drift_m', 0.15).value)
        self._max_ground_xy_speed_m_s = float(self.declare_parameter(
            'max_ground_xy_speed_m_s', 0.12).value)
        # Abort a powered takeoff that never leaves the floor.  This protects
        # the VIO stream from long ground-sliding/vibration episodes and is
        # deliberately based on raw range relative to the takeoff baseline.
        self._no_liftoff_timeout_s = float(self.declare_parameter(
            'no_liftoff_timeout_s', 8.0).value)
        self._no_liftoff_delta_m = float(self.declare_parameter(
            'no_liftoff_delta_m', 0.05).value)
        self._no_liftoff_max_height_m = float(self.declare_parameter(
            'no_liftoff_max_height_m', 0.20).value)
        # Abort if yaw jumps > this after arm (EV yaw / bad alignment).
        self._max_yaw_jump_deg = float(self.declare_parameter(
            'max_yaw_jump_deg', 20.0).value)

        self.get_logger().info(
            '起飞控制节点初始化完成 (Layer 3, ENU; 激光定高+5cm 中心偏置)')

    # ── 回调 ────────────────────────────────────────────────

    def _pose_cb(self, msg: PoseStamped):
        now = time.monotonic()
        if self._has_pose:
            dz = msg.pose.position.z - self._pose.pose.position.z
            dt = now - self._pose_received_at
            if dt <= 2.0 and abs(dz) > 0.75:
                self._pending_pose_jump_z += dz
                self._last_pose_jump_at = now
                self.get_logger().warn(
                    f'/uav/state/pose Z jump detected: {dz:+.2f}m; '
                    'will rebase active setpoints if already flying')
        self._pose = msg
        if not self._has_pose:
            self._first_pose_received_at = now
            self._has_pose = True
        self._pose_received_at = now

    def _status_cb(self, msg: VehicleStatus):
        self._status = msg

    def _odom_cb(self, msg: VehicleOdometry):
        q = msg.q
        if len(q) < 4 or math.isnan(q[0]):
            return
        self._roll_ned, self._pitch_ned, self._yaw_ned = self._quat_to_euler(q)
        if len(msg.velocity) >= 2:
            vx = float(msg.velocity[0])
            vy = float(msg.velocity[1])
            if math.isfinite(vx) and math.isfinite(vy):
                self._horizontal_speed_m_s = math.hypot(vx, vy)

    def _local_position_cb(self, msg: VehicleLocalPosition):
        now = time.monotonic()
        self._range_received_at = now
        self._dist_bottom_valid = bool(msg.dist_bottom_valid)
        self._dist_bottom = float(msg.dist_bottom)
        # Raw laser freshness is independent of EKF cs_rng_hgt. PX4 often
        # keeps dist_bottom_valid=false while still publishing a usable range.
        if (math.isfinite(self._dist_bottom)
                and self._dist_bottom >= self._min_dist_bottom_m):
            self._range_good_at = now

    def _estimator_flags_cb(self, msg: EstimatorStatusFlags):
        now = time.monotonic()
        self._estimator_flags_received_at = now
        self._estimator_flags_seen = True
        prev_rng = self._cs_rng_hgt
        self._cs_rng_hgt = bool(msg.cs_rng_hgt)
        self._cs_ev_pos = bool(msg.cs_ev_pos)
        self._cs_ev_hgt = bool(msg.cs_ev_hgt)
        self._cs_ev_vel = bool(msg.cs_ev_vel)
        self._cs_inertial_dead_reckoning = bool(
            msg.cs_inertial_dead_reckoning)
        if self._cs_rng_hgt:
            self._rng_fusion_lost_since = 0.0
        elif self._rng_fusion_lost_since <= 0.0:
            self._rng_fusion_lost_since = now
            if prev_rng and self.rangefinder_raw_ready:
                self.get_logger().warn(
                    'EKF dropped cs_rng_hgt while raw dist_bottom is still '
                    'fresh — climb accel gate / innovation reject likely; '
                    'takeoff will continue on raw range',
                    throttle_duration_sec=2.0)
        if (self._cs_ev_pos
                and not self._cs_inertial_dead_reckoning):
            self._ev_fusion_lost_since = 0.0
        elif self._ev_fusion_lost_since <= 0.0:
            self._ev_fusion_lost_since = now

    def _attitude_cb(self, msg: VehicleAttitude):
        q = msg.q
        if len(q) < 4 or math.isnan(q[0]):
            return
        self._roll_ned, self._pitch_ned, self._yaw_ned = self._quat_to_euler(q)

    def _actuator_outputs_cb(self, msg: ActuatorOutputs):
        self._actuator_outputs = list(msg.output[:4])

    def _actuator_motors_cb(self, msg: ActuatorMotors):
        self._actuator_motors = list(msg.control[:4])

    def _vslam_odom_cb(self, msg: Odometry):
        """监控 VSLAM 里程计频率，用于检测跟踪质量退化"""
        now = time.monotonic()
        self._vslam_odom_received_at = now
        self._vslam_odom_count += 1

    def _rtabmap_relay_cb(self, msg: Odometry):
        self._rtabmap_relay_received_at = time.monotonic()

    def _rtabmap_status_cb(self, msg: String):
        self._rtabmap_status_text = msg.data.strip()
        self._rtabmap_status_received_at = time.monotonic()
        try:
            status = json.loads(self._rtabmap_status_text)
        except (json.JSONDecodeError, TypeError):
            status = None
        if isinstance(status, dict) and isinstance(
                status.get('healthy'), bool):
            self._rtabmap_status_healthy = status['healthy']
            return

        # Backward-compatible parser for older plain-text relay status.
        text = self._rtabmap_status_text.lower()
        unhealthy = (
            'unhealthy', 'error', 'fault', 'stale', 'lost', 'degraded',
            'not ready', 'waiting',
        )
        healthy = ('healthy', 'ok', 'active', 'ready', 'running')
        self._rtabmap_status_healthy = (
            any(token in text for token in healthy)
            and not any(token in text for token in unhealthy)
        )

    def measure_vslam_odom_hz(self, duration_s: float = 2.0) -> float:
        """在 duration 内积极 spin，用到帧计数测真实频率（避免 EMA/sleep 低估）。"""
        duration_s = max(0.5, float(duration_s))
        count0 = self._vslam_odom_count
        t0 = time.monotonic()
        while time.monotonic() - t0 < duration_s:
            rclpy.spin_once(self, timeout_sec=0.05)
        dt = time.monotonic() - t0
        n = self._vslam_odom_count - count0
        hz = (n / dt) if dt > 0.0 else 0.0
        self._vslam_odom_hz = hz
        return hz

    @property
    def vslam_odom_stale(self) -> bool:
        """VSLAM 里程计是否超过 0.5 秒未更新"""
        if self._vslam_odom_received_at == 0.0:
            return True  # 从未收到过
        return time.monotonic() - self._vslam_odom_received_at > 0.5

    @property
    def vslam_odom_hz(self) -> float:
        """最近一次测得的 VSLAM 频率 (Hz)"""
        return self._vslam_odom_hz

    @property
    def vslam_odom_healthy(self) -> bool:
        """VSLAM 里程计是否健康 (频率达标且不过期)"""
        return (not self.vslam_odom_stale
                and (self._vslam_odom_hz == 0.0  # 刚启动，尚未统计
                     or self._vslam_odom_hz >= self._min_vslam_odom_hz))

    @property
    def ev_fusion_healthy(self) -> bool:
        """VSLAM水平位置融合是否正常（EV速度因升降假速度而禁用）。"""
        return (self._cs_ev_pos
                and not self._cs_inertial_dead_reckoning)

    @property
    def ev_fusion_lost_duration(self) -> float:
        """EV 融合丢失持续时间 (秒); 0 表示当前正常"""
        if self.ev_fusion_healthy and self.estimator_flags_fresh:
            return 0.0
        if self._ev_fusion_lost_since <= 0.0:
            reference = max(
                self._estimator_flags_received_at,
                self._flight_monitor_started_at,
            )
            return max(0.0, time.monotonic() - reference) if reference > 0.0 else 0.0
        return time.monotonic() - self._ev_fusion_lost_since

    @property
    def rng_fusion_lost_duration(self) -> float:
        """激光高度融合丢失持续时间 (秒); 0 表示当前正常"""
        if self._cs_rng_hgt or not self.estimator_flags_fresh:
            return 0.0
        if self._rng_fusion_lost_since <= 0.0:
            return 0.0
        return time.monotonic() - self._rng_fusion_lost_since

    @property
    def rangefinder_lost_duration(self) -> float:
        if self.rangefinder_ready:
            self._range_good_at = time.monotonic()
            return 0.0
        reference = max(self._range_good_at, self._flight_monitor_started_at)
        return max(0.0, time.monotonic() - reference) if reference > 0.0 else 0.0

    def mark_flight_monitor_started(self):
        self._flight_monitor_started_at = time.monotonic()
        if self.rangefinder_ready:
            self._range_good_at = self._flight_monitor_started_at
        if (not self.rtabmap_monitoring_enabled()
                or self.rtabmap_relay_healthy):
            self._rtabmap_good_at = self._flight_monitor_started_at

    def rtabmap_monitoring_enabled(self) -> bool:
        if not self._require_rtabmap_relay_if_enabled:
            return False
        node_names = {name.rsplit('/', 1)[-1] for name in self.get_node_names()}
        return (
            'rtabmap_odom_relay' in node_names
            or bool(self.get_publishers_info_by_topic('/rtabmap/relay/odometry'))
            or bool(self.get_publishers_info_by_topic('/rtabmap/relay/status'))
        )

    @property
    def rtabmap_relay_healthy(self) -> bool:
        now = time.monotonic()
        relay_fresh = (
            self._rtabmap_relay_received_at > 0.0
            and now - self._rtabmap_relay_received_at <= self._rtabmap_relay_timeout_s
        )
        status_enabled = bool(
            self.get_publishers_info_by_topic('/rtabmap/relay/status'))
        if not status_enabled:
            return relay_fresh
        status_fresh = (
            self._rtabmap_status_received_at > 0.0
            and now - self._rtabmap_status_received_at <= self._rtabmap_status_timeout_s
        )
        healthy = relay_fresh and status_fresh and self._rtabmap_status_healthy
        if healthy:
            self._rtabmap_good_at = now
        return healthy

    @property
    def rtabmap_relay_lost_duration(self) -> float:
        if not self.rtabmap_monitoring_enabled():
            return 0.0
        if self.rtabmap_relay_healthy:
            return 0.0
        reference = max(
            self._rtabmap_good_at, self._flight_monitor_started_at)
        return (
            max(0.0, time.monotonic() - reference)
            if reference > 0.0 else 0.0)

    @staticmethod
    def _quat_to_euler(q):
        w, x, y, z = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        roll = math.atan2(
            2.0 * (w * x + y * z),
            1.0 - 2.0 * (x * x + y * y),
        )
        sin_pitch = 2.0 * (w * y - z * x)
        sin_pitch = max(-1.0, min(1.0, sin_pitch))
        pitch = math.asin(sin_pitch)
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        return roll, pitch, yaw

    # ── 属性 ──

    @property
    def armed(self) -> bool:
        return self._status.arming_state == 2

    @property
    def preflight_pass(self) -> bool:
        return self._status.pre_flight_checks_pass

    @property
    def failsafe(self) -> bool:
        return self._status.failsafe

    @property
    def nav_state(self) -> int:
        return self._status.nav_state

    @property
    def in_offboard(self) -> bool:
        return self._status.nav_state == 14

    @property
    def pose_fresh(self) -> bool:
        return self._has_pose and time.monotonic() - self._pose_received_at <= 0.5

    def pose_stable_for(self, seconds: float) -> bool:
        if not self.pose_fresh:
            return False
        stable_since = max(self._first_pose_received_at, self._last_pose_jump_at)
        return time.monotonic() - stable_since >= seconds

    @property
    def rangefinder_raw_ready(self) -> bool:
        """Fresh finite TFmini reading, independent of EKF fusion flags."""
        if not self._require_valid_rangefinder:
            return True
        fresh = time.monotonic() - self._range_received_at <= self._range_timeout_s
        return (
            fresh
            and math.isfinite(self._dist_bottom)
            and self._dist_bottom >= self._min_dist_bottom_m
        )

    @property
    def rangefinder_ready(self) -> bool:
        """Airborne / AGL uses raw range. Pre-arm still checks cs_rng_hgt
        separately via estimator_fusion_ready."""
        return self.rangefinder_raw_ready

    def airborne_range_gate_failure(self, commanded_sensor_agl: float):
        """Require PX4's independent range aid once the climb is airborne.

        The raw TFmini value can be fresh while PX4 marks it invalid near the
        floor.  Do not reject takeoff for that expected ground condition, but
        never keep the motors running indefinitely with an invalid height
        source after the commanded setpoint is above the HAGL envelope.
        """
        if commanded_sensor_agl < self._range_validity_gate_m:
            self._range_validity_lost_since = 0.0
            return None

        now = time.monotonic()
        fresh_range = (
            self._range_received_at > 0.0
            and now - self._range_received_at <= self._range_timeout_s)
        # With EKF2_EV_CTRL=15, bridge supplies Z via VPOS.
        # Only require raw TFmini data freshness; EKF fusion flags handled elsewhere.
        # Skip min-distance gate during climb: dist_bottom starts at ~0.10m and
        # only exceeds 0.35m after the drone has actually climbed.
        healthy = (
            fresh_range
            and math.isfinite(self._dist_bottom))
        if healthy:
            self._range_validity_lost_since = 0.0
            return None
        if self._range_validity_lost_since <= 0.0:
            self._range_validity_lost_since = now
            return None
        elapsed = now - self._range_validity_lost_since
        if elapsed <= self._range_validity_timeout_s:
            return None
        return (
            '目标测距高度已越过 %.2fm，但 TFmini/EKF 高度仍无效 %.1fs '
            '(dist_bottom=%.3f, valid=%s, cs_rng_hgt=%s)'
            % (self._range_validity_gate_m, elapsed,
               self._dist_bottom, self._dist_bottom_valid, self._cs_rng_hgt))

    @property
    def estimator_flags_fresh(self) -> bool:
        if not self._estimator_flags_seen:
            return False
        return (time.monotonic() - self._estimator_flags_received_at
                <= self._estimator_flags_timeout_s)

    @property
    def estimator_fusion_ready(self) -> bool:
        # Indoor: EV pos + laser or EV height
        return (self.estimator_flags_fresh
                and self._cs_ev_pos
                and (self._cs_rng_hgt or self._cs_ev_hgt)
                and not self._cs_inertial_dead_reckoning)

    @property
    def dist_bottom(self) -> float:
        return self._dist_bottom

    def consume_pose_jump_z(self) -> float:
        jump = self._pending_pose_jump_z
        self._pending_pose_jump_z = 0.0
        return jump

    @property
    def pos_x(self) -> float:
        """当前 ENU X (东)"""
        return self._pose.pose.position.x

    @property
    def pos_y(self) -> float:
        """当前 ENU Y (北)"""
        return self._pose.pose.position.y

    @property
    def pos_z(self) -> float:
        """当前 ENU Z (上, 正=向上)"""
        return self._pose.pose.position.z

    @property
    def yaw_ned(self) -> float:
        """当前 NED yaw (rad)，无效时为 NaN"""
        return self._yaw_ned

    @property
    def roll_ned(self) -> float:
        """当前 NED roll (rad)，无效时为 NaN"""
        return self._roll_ned

    @property
    def pitch_ned(self) -> float:
        """当前 NED pitch (rad)，无效时为 NaN"""
        return self._pitch_ned

    @property
    def horizontal_speed_m_s(self) -> float:
        return self._horizontal_speed_m_s

    @property
    def actuator_outputs(self):
        return self._actuator_outputs

    @property
    def actuator_motors(self):
        return self._actuator_motors

    def _now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    # ── 发布方法 ──

    def publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._now_us()
        self.offboard_mode_pub.publish(msg)

    def publish_setpoint(self, x: float, y: float, z: float,
                         yaw: float = float('nan')):
        """发布位置 setpoint (输入 ENU, 内部转 NED 发送)"""
        ned_x, ned_y, ned_z = enu_to_ned(x, y, z)
        msg = TrajectorySetpoint()
        msg.position = [float(ned_x), float(ned_y), float(ned_z)]
        msg.yaw = float(yaw)
        msg.yawspeed = float('nan')
        msg.velocity = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.timestamp = self._now_us()
        self.setpoint_pub.publish(msg)

    def publish_cmd(self, command: int, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self.cmd_pub.publish(msg)

    # ── 高级命令 ──

    def arm(self):
        self.publish_cmd(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm 命令已发送')

    def disarm(self, force: bool = False):
        # PX4: param2=21196 forces disarm even if land-detector has not fired.
        params = {'param1': 0.0}
        if force:
            params['param2'] = 21196.0
        self.publish_cmd(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, **params)
        self.get_logger().info(
            'Force Disarm 命令已发送' if force else 'Disarm 命令已发送')

    def engage_offboard(self):
        self.publish_cmd(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard 模式切换命令已发送')

    def land_cmd(self):
        self.publish_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Land 命令已发送')

    def wait_for_arming_change(self, target_armed: bool, timeout: float = 5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.armed == target_armed:
                return True
        return False

    def force_disarm_sequence(self, rounds: int = 8, interval_s: float = 0.2) -> bool:
        """Normal disarm first, then PX4 force-disarm (21196)."""
        for _ in range(3):
            self.disarm(force=False)
            time.sleep(interval_s)
            rclpy.spin_once(self, timeout_sec=0.01)
            if not self.armed:
                return True
        for _ in range(max(1, rounds)):
            self.disarm(force=True)
            time.sleep(interval_s)
            rclpy.spin_once(self, timeout_sec=0.01)
            if not self.armed:
                return True
        return self.wait_for_arming_change(False, timeout=2.0)

    def stream_ground_setpoint(self, enu_x: float, enu_y: float, enu_z: float,
                               yaw: float = float('nan')):
        self.publish_offboard_heartbeat()
        self.publish_setpoint(x=enu_x, y=enu_y, z=enu_z, yaw=yaw)


# ═══════════════════════════════════════════════════════════════
#  起飞 → 悬停 → 等待降落
# ═══════════════════════════════════════════════════════════════

def wait_for_pose_ready(node: PX4Controller, stable_seconds: float,
                        timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.pose_stable_for(stable_seconds):
            return True
        time.sleep(0.05)
    rclpy.spin_once(node, timeout_sec=0.05)
    return node.pose_stable_for(stable_seconds)


def wait_for_rangefinder_ready(node: PX4Controller, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.rangefinder_ready:
            return True
        time.sleep(0.05)
    return node.rangefinder_ready


def wait_for_estimator_fusion_ready(node: PX4Controller, timeout: float) -> bool:
    """Wait for ~1 Hz estimator_status_flags and required fusion bits."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.estimator_fusion_ready:
            return True
        time.sleep(0.05)
    rclpy.spin_once(node, timeout_sec=0.05)
    return node.estimator_fusion_ready


def wait_for_rtabmap_relay_ready(node: PX4Controller, timeout: float) -> bool:
    """Require a fresh relay output/status only when RTAB relay is enabled."""
    if not node.rtabmap_monitoring_enabled():
        return True
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.rtabmap_relay_healthy:
            return True
        time.sleep(0.05)
    return node.rtabmap_relay_healthy


def _rtabmap_status_reason(node: PX4Controller) -> str:
    raw = node._rtabmap_status_text or ''
    try:
        status = json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        return raw or '无状态消息'
    if not isinstance(status, dict):
        return raw or '无状态消息'
    reason = status.get('reason', '')
    loop_id = status.get('loop_closure_id', 0)
    proximity_id = status.get('proximity_detection_id', 0)
    match_age = status.get('match_age')
    bits = [str(reason)] if reason else []
    bits.append(f'loop={loop_id} proximity={proximity_id}')
    if match_age is not None:
        bits.append(f'match_age={match_age:.2f}s')
    return ', '.join(bits)


def airborne_localization_failure(node: PX4Controller):
    """Return a controlled-landing reason after a persistent airborne loss."""
    pose_age = (
        time.monotonic() - node._pose_received_at
        if node._pose_received_at > 0.0 else float('inf')
    )
    if pose_age > node._air_pose_loss_timeout_s:
        return (
            f'/uav/state/pose 已中断 {pose_age:.1f}s '
            f'(阈值 {node._air_pose_loss_timeout_s:.1f}s)'
        )
    rtabmap_lost = node.rtabmap_relay_lost_duration
    if rtabmap_lost > node._rtabmap_loss_timeout_s:
        return (
            f'RTAB-Map 定位 relay 已丢失 {rtabmap_lost:.1f}s '
            f'(阈值 {node._rtabmap_loss_timeout_s:.2f}s, '
            f'status={node._rtabmap_status_text or "无状态"})'
        )
    ev_lost = node.ev_fusion_lost_duration
    if ev_lost > node._ev_fusion_loss_timeout_s:
        return (
            f'EV 水平位置融合已丢失 {ev_lost:.1f}s '
            f'(阈值 {node._ev_fusion_loss_timeout_s:.1f}s, '
            f'cs_ev_pos={node._cs_ev_pos}, cs_ev_vel={node._cs_ev_vel}, '
            f'dead_reckoning={node._cs_inertial_dead_reckoning})'
        )
    # Abort only when the raw laser sample is stale/out of range. Brief
    # cs_rng_hgt dropouts during climb are expected (EKF2_RNG_A_VMAX) and must
    # not force a landing while dist_bottom is still streaming.
    range_lost = node.rangefinder_lost_duration
    if range_lost > node._rangefinder_loss_timeout_s:
        return (
            f'TFmini 原始测距已丢失/超限 {range_lost:.1f}s '
            f'(阈值 {node._rangefinder_loss_timeout_s:.1f}s, '
            f'dist_bottom={node.dist_bottom}, '
            f'valid={node._dist_bottom_valid}, cs_rng_hgt={node._cs_rng_hgt})'
        )
    return None


def trigger_controlled_landing(node: PX4Controller, reason: str) -> bool:
    print(f'✗ {reason}，触发受控降落。')
    node.land_cmd()
    return False


def check_stationary_xy(node: PX4Controller) -> bool:
    """Block arming when the grounded EKF position or velocity is drifting."""
    duration = max(1.0, node._stationary_check_s)
    start_x = node.pos_x
    start_y = node.pos_y
    max_displacement = 0.0
    max_speed = 0.0
    deadline = time.monotonic() + duration
    print(f'静止漂移检查: 保持机体不动 {duration:.0f}s...')

    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if not node.pose_fresh:
            print('✗ 静止检查期间位姿数据中断，已中止起飞。')
            return False
        displacement = math.hypot(node.pos_x - start_x, node.pos_y - start_y)
        max_displacement = max(max_displacement, displacement)
        if math.isfinite(node.horizontal_speed_m_s):
            max_speed = max(max_speed, node.horizontal_speed_m_s)
        time.sleep(0.05)

    print(f'  地面XY最大漂移={max_displacement:.3f}m, '
          f'最大水平速度={max_speed:.3f}m/s')
    if max_displacement > node._max_ground_xy_drift_m:
        print(f'✗ 地面XY漂移超过 {node._max_ground_xy_drift_m:.2f}m，禁止解锁。')
        print('   检查 D435i TF、VSLAM 跟踪质量和 EKF2_EV_POS_X/Y/Z=0。')
        return False
    if max_speed > node._max_ground_xy_speed_m_s:
        print(f'✗ 地面水平速度超过 {node._max_ground_xy_speed_m_s:.2f}m/s，禁止解锁。')
        print('   检查 VSLAM 速度、光照/纹理和相机固定是否牢靠。')
        return False
    print('  ✓ 地面水平定位稳定')
    return True


def _yaw_delta_deg(current_yaw: float, reference_yaw: float) -> float:
    """Signed yaw error in degrees, wrapped to [-180, 180]."""
    if math.isnan(current_yaw) or math.isnan(reference_yaw):
        return float('nan')
    err = math.atan2(math.sin(current_yaw - reference_yaw),
                     math.cos(current_yaw - reference_yaw))
    return math.degrees(err)


def _center_agl(node: PX4Controller) -> float:
    """机体中心离地高度 ≈ dist_bottom(激光测距点 AGL) + center_above_sensor。"""
    if not math.isfinite(node.dist_bottom):
        return float('nan')
    return node.dist_bottom + node._center_above_sensor_m


def _trim_z_for_sensor_agl(node: PX4Controller, desired_sensor_agl: float,
                           fallback_z: float) -> float:
    """One-shot ENU z trim from dist_bottom (slow hover trim only)."""
    if node.rangefinder_ready and math.isfinite(node.dist_bottom):
        return node.pos_z + (desired_sensor_agl - node.dist_bottom)
    return fallback_z


def _enter_hover_setpoints(node: PX4Controller, target_sensor_agl: float,
                           origin_z: float, target_z: float,
                           origin_x: float, origin_y: float):
    """Hold takeoff XY origin and trim Z to AGL.

    Locking current XY used to freeze climb drift (~1m); keep commanding the
    takeoff point so MPC can pull back while EKF still sees the error.
    """
    hover_x = origin_x
    hover_y = origin_y
    hover_z = _trim_z_for_sensor_agl(node, target_sensor_agl, target_z)
    xy_err = math.hypot(node.pos_x - origin_x, node.pos_y - origin_y)
    print(f'悬停锁定: XY=起飞原点 ({hover_x:.2f}, {hover_y:.2f})'
          f' (当前误差 {xy_err:.2f}m，将尝试拉回)')
    if node.rangefinder_ready and math.isfinite(node.dist_bottom):
        center = _center_agl(node)
        print(f'高度微调: 测距 {node.dist_bottom:.2f}m → 目标 {target_sensor_agl:.2f}m '
              f'(中心≈{center:.2f}m)')
    return hover_x, hover_y, hover_z


def do_takeoff_and_hover(node: PX4Controller, target_altitude: float,
                         stop_event: threading.Event):
    """起飞到机体中心目标离地高度并悬停。

    用起飞瞬间的 dist_bottom 换算一次固定 ENU 高度 setpoint，悬停时不再
    用 AGL 闭环改 z（避免与 EKF 抢高度导致乱飞/抖动）。
    """

    target_center_agl = abs(float(target_altitude))
    sensor_offset = max(0.0, float(node._center_above_sensor_m))
    # dist_bottom ≈ 机脚/测距点离地；中心更高 sensor_offset
    target_sensor_agl = max(node._min_dist_bottom_m, target_center_agl - sensor_offset)
    print(f'\n═══ 起飞: 机体中心离地 {target_center_agl}m '
          f'(测距目标 {target_sensor_agl:.2f}m) ═══\n')

    # 安全检查
    if node.armed:
        print('✗ 飞控已解锁! 请先锁定后再试。')
        return False
    if not node.preflight_pass:
        print('✗ 飞前检查未通过! 无法起飞。')
        return False
    if not node.rangefinder_ready:
        print('等待有效的 PX4 向下测距数据...')
        if not wait_for_rangefinder_ready(node, timeout=3.0):
            if (math.isfinite(node.dist_bottom)
                    and node.dist_bottom < node._min_dist_bottom_m):
                print(f'✗ 向下测距 {node.dist_bottom:.2f}m 低于最小阈值 '
                      f'{node._min_dist_bottom_m:.2f}m，已中止起飞。')
                print('   提高 TFmini 安装高度，或运行: '
                      'ros2 run uav_task takeoff_control.py --ros-args '
                      '-p min_dist_bottom_m:=0.05')
            else:
                print('✗ 向下测距未就绪（无新鲜 dist_bottom），已中止起飞。')
                print('   检查 TFmini 供电、TX→TELEM2 RX、SENS_TFMINI_CFG=102。')
            return False
    print('等待 EKF 融合状态 (激光定高 + VSLAM 水平)...')
    if not wait_for_estimator_fusion_ready(node, timeout=6.0):
        if not node.estimator_flags_fresh:
            print('✗ 无新鲜 EKF 融合状态，无法确认高度和水平位置源，已中止起飞。')
            print('   /fmu/out/estimator_status_flags 约 1Hz；请确认 SLAM/PX4 bridge 在线。')
        elif not node._cs_rng_hgt:
            print('✗ 起飞前 TFmini 高度未被 EKF 融合 (cs_rng_hgt=false)，已中止起飞。')
            print('   请设置 EKF2_HGT_REF=2、EKF2_RNG_CTRL=2、EKF2_BARO_CTRL=0，保存并重启 PX4。')
        else:
            print('✗ VSLAM 水平位置未融合，无法可靠定点悬停。')
            print(f'   当前 cs_ev_pos={node._cs_ev_pos}, cs_ev_vel={node._cs_ev_vel}；'
                  f'dead_reckoning={node._cs_inertial_dead_reckoning}；'
                  '请检查 EKF2_EV_CTRL=1 和视觉里程计质量。')
        return False
    if node._cs_ev_hgt:
        print('⚠  cs_ev_hgt=true：EV 垂直仍在融合，室内激光定高应使用 EKF2_EV_CTRL=1。')
    takeoff_sensor_agl = float(node.dist_bottom)
    takeoff_center_agl = takeoff_sensor_agl + sensor_offset
    print(
        f'激光测距: {takeoff_sensor_agl:.2f}m '
        f'(raw_ready, valid={node._dist_bottom_valid}, '
        f'cs_rng_hgt={node._cs_rng_hgt}), '
        f'估计中心离地≈{takeoff_center_agl:.2f}m '
        f'(中心=激光+{sensor_offset:.2f}m)')
    print('  爬升中若 cs_rng_hgt 短暂掉线但 dist_bottom 仍刷新，将继续飞行；'
          '仅原始测距丢失才会受控降落。')
    if takeoff_sensor_agl >= target_sensor_agl:
        print(f'✗ 当前测距 {takeoff_sensor_agl:.2f}m 已不低于目标 '
              f'{target_sensor_agl:.2f}m，已中止。')
        return False
    if not node.pose_stable_for(2.0):
        print('等待新鲜且稳定的 /uav/state/pose...')
        if not wait_for_pose_ready(node, stable_seconds=2.0, timeout=5.0):
            if not node.pose_fresh:
                print('✗ 无新鲜 /uav/state/pose，高度参考无效，已中止。')
                print('   请检查 state_bridge 是否收到 '
                      '/fmu/out/vehicle_odometry 或 vehicle_local_position。')
            else:
                print('✗ /uav/state/pose 高度刚发生跳变或尚未稳定，已中止。')
                print('   请等待 EKF 高度稳定后再起飞。')
            return False
    if not check_stationary_xy(node):
        return False

    # ── VSLAM 里程计健康检查 ──
    # D435i+Isaac 常见 15–30 Hz。用时间窗计数，勿用 sleep+EMA（会把 ~20Hz 误测成 2Hz）。
    min_vslam_hz = float(node._min_vslam_odom_hz)
    print('检查 VSLAM 里程计频率...')
    measured_hz = node.measure_vslam_odom_hz(2.0)
    if node.vslam_odom_stale and measured_hz <= 0.0:
        print('✗ VSLAM 里程计无数据! 请确认:')
        print('   - Docker 容器 isaac_ros_dev 是否运行')
        print('   - VSLAM 节点是否启动: docker exec isaac_ros_dev ros2 node list | grep visual_slam')
        print('   - 宿主机: ros2 topic hz /visual_slam/tracking/odometry')
        print('   - 若有 QoS DURABILITY 警告，请更新 takeoff_control（VSLAM 须 VOLATILE）')
        return False
    if measured_hz < min_vslam_hz:
        print(f'✗ VSLAM 里程计频率过低: {measured_hz:.1f} Hz '
              f'(< {min_vslam_hz:.0f} Hz)。')
        print('   可能原因: 相机帧抖动过大、D435i USB 带宽不足、容器负载过高。')
        print('   建议: 检查相机减震、提高 slam_rtabmap 中 image_jitter_threshold_ms。')
        return False
    print(f'  ✓ VSLAM 里程计频率: {measured_hz:.1f} Hz')

    if node.rtabmap_monitoring_enabled():
        print('检测到 RTAB-Map relay，等待几何匹配(proximity/loop)后的健康输出...')
        if not wait_for_rtabmap_relay_ready(node, timeout=15.0):
            status = _rtabmap_status_reason(node)
            print('✗ RTAB-Map relay 已启用但不健康，禁止解锁。')
            print(f'   relay/status={status}')
            print('   仅 initialpose + 低协方差不够：中心地图空洞时会假锁定并跟漂。')
            print('   请在有墙/节点的区域等到 proximity/loop>0，或重建覆盖起飞点的地图。')
            return False
        print(f'  ✓ RTAB-Map relay healthy ({_rtabmap_status_reason(node)})')
    else:
        print('  RTAB-Map relay 未启用：保持无 RTAB 诊断模式，不作为解锁条件。')

    # 记录起飞原点；相对爬升按测距目标一次换算，悬停用固定 ENU z
    origin_x = node.pos_x
    origin_y = node.pos_y
    origin_z = node.pos_z
    origin_yaw = node.yaw_ned
    relative_climb = target_sensor_agl - takeoff_sensor_agl
    target_z = origin_z + relative_climb
    print(f'起飞原点 (ENU): x={origin_x:.2f}, y={origin_y:.2f}, z={origin_z:.2f}')
    print(f'目标: 中心AGL={target_center_agl:.2f}m, 测距AGL={target_sensor_agl:.2f}m '
          f'(中心−机脚={sensor_offset:.2f}m), 相对爬升≈{relative_climb:.2f}m')
    if math.isnan(origin_yaw):
        print('当前 yaw: 无有效 vehicle_odometry，yaw setpoint 将保持 NaN')
    else:
        print(f'当前 yaw: {origin_yaw:.2f} rad ({math.degrees(origin_yaw):.1f} deg)，起飞期间锁定此航向')
    print_takeoff_diagnostics(node, origin_x, origin_y, origin_z, '起飞前')

    # ── 第 1 步: 发心跳 + 地面 setpoints 建立信任 ──
    print('第 1 步: 发送 Offboard 心跳 + 地面 setpoints...')
    for i in range(30):
        jump_z = node.consume_pose_jump_z()
        if abs(jump_z) > 0.75:
            print(f'✗ Offboard 预热期间检测到高度参考跳变 {jump_z:+.2f}m，已中止。')
            print('   请等待 EKF 高度稳定后再起飞。')
            return False
        node.stream_ground_setpoint(origin_x, origin_y, origin_z, yaw=origin_yaw)
        time.sleep(0.1)
        rclpy.spin_once(node, timeout_sec=0.01)
    xy_error, _ = print_takeoff_diagnostics(node, origin_x, origin_y, origin_z,
                                            'Offboard预热后')
    if xy_error > 0.35:
        print('✗ 起飞前水平位置已明显漂移，已中止。')
        print('   这通常是 VSLAM/EKF 坐标或速度输入异常，不建议继续带桨测试。')
        return False

    # ── 第 2 步: 切换 Offboard ──
    print('第 2 步: 切换 Offboard 模式...')
    offboard_start = time.time()
    command_sent_at = 0.0
    while time.time() - offboard_start < 5.0:
        jump_z = node.consume_pose_jump_z()
        if abs(jump_z) > 0.75:
            print(f'✗ 切换 Offboard 前检测到高度参考跳变 {jump_z:+.2f}m，已中止。')
            print('   请等待 EKF 高度稳定后再起飞。')
            return False
        node.stream_ground_setpoint(origin_x, origin_y, origin_z, yaw=origin_yaw)
        now = time.time()
        if now - command_sent_at >= 0.5:
            node.engage_offboard()
            command_sent_at = now
        time.sleep(0.1)
        rclpy.spin_once(node, timeout_sec=0.01)
        if node.in_offboard:
            break
    print(f'  当前 nav_state = {node.nav_state}')
    if not node.in_offboard:
        print('✗ 未进入 Offboard 模式，已中止起飞。')
        print('   电机只会怠速转、不会响应位置起飞 setpoint。')
        return False

    # ── 第 3 步: 解锁 ──
    print('第 3 步: 解锁...')
    if (node.rtabmap_monitoring_enabled()
            and not wait_for_rtabmap_relay_ready(node, timeout=1.0)):
        print('✗ RTAB-Map relay 在解锁前失去健康状态，已中止。')
        return False
    arm_start = time.time()
    command_sent_at = 0.0
    while time.time() - arm_start < 5.0:
        jump_z = node.consume_pose_jump_z()
        if abs(jump_z) > 0.75:
            print(f'✗ 解锁前检测到高度参考跳变 {jump_z:+.2f}m，已中止。')
            print('   请等待 EKF 高度稳定后再起飞。')
            return False
        node.stream_ground_setpoint(origin_x, origin_y, origin_z, yaw=origin_yaw)
        now = time.time()
        if now - command_sent_at >= 0.5:
            node.arm()
            command_sent_at = now
        time.sleep(0.1)
        rclpy.spin_once(node, timeout_sec=0.01)
        if node.armed:
            break
    if not node.wait_for_arming_change(True, timeout=5.0):
        print('✗ 解锁超时!')
        return False
    print('  ✓ 解锁成功')
    node.mark_flight_monitor_started()
    arm_yaw = node.yaw_ned
    if not math.isnan(arm_yaw):
        print(f'  解锁 yaw 基准: {math.degrees(arm_yaw):.1f} deg '
              f'(跳变 >{node._max_yaw_jump_deg:.0f}° 将中止)')

    # ── 第 4 步: 斜坡爬升到固定 ENU 高度 ──
    # Z 轴高度由 EV 通道激光 dist_bottom 提供 (vslam_odom_bridge 已注入)
    # 两段式爬升: 中途暂停让 VSLAM 恢复特征跟踪，减少帧丢失
    if TWO_STAGE_CLIMB:
        midpoint_ratio = max(0.2, min(0.8, TWO_STAGE_MIDPOINT_RATIO))
        midpoint_z = origin_z + relative_climb * midpoint_ratio
        climb_stages = [
            (midpoint_z, f'中途 {midpoint_ratio*100:.0f}% (ENU z={midpoint_z:.2f})'),
            (target_z, f'目标高度 (ENU z={target_z:.2f})'),
        ]
        print(f'第 4 步: 两段式爬升至中心离地 {target_center_agl}m')
        print(f'  ① → {midpoint_ratio*100:.0f}% 悬停 {TWO_STAGE_PAUSE_S:.0f}s → ② → 目标')
    else:
        climb_stages = [(target_z, f'目标高度 (ENU z={target_z:.2f})')]
        print(f'第 4 步: 爬升至中心离地 {target_center_agl}m '
              f'(固定 ENU z={target_z:.2f})...')

    climb_rate = max(0.05, min(TAKEOFF_CLIMB_RATE_MPS, 0.5))
    print(f'  爬升速度: {climb_rate:.2f} m/s')
    dt = 0.1               # 10Hz
    max_xy_error_m = 0.35  # 全程 XY 中止阈值（不再仅贴地检查）
    # 初始 setpoint 比当前高度高一点，给足够推力突破地面效应
    current_z = origin_z + max(0.1, TAKEOFF_KICK_M)
    if TWO_STAGE_CLIMB:
        current_z = min(current_z, climb_stages[0][0])
    else:
        current_z = min(current_z, target_z)
    print(f'  初始起飞 setpoint: z={current_z:.2f} (当前 z={origin_z:.2f})')
    climb_start_time = time.time()
    last_diag_time = 0.0
    stall_warned = False

    for stage_idx, (stage_target_z, stage_label) in enumerate(climb_stages):
        if stage_idx > 0:
            # ── 两段式暂停: 在上一段终点悬停等待 VSLAM 恢复 ──
            pause_s = TWO_STAGE_PAUSE_S if TWO_STAGE_CLIMB else 0.0
            if pause_s > 0:
                print(f'\n⏸  中途暂停 {pause_s:.0f}s @ ENU z={current_z:.2f}，等待 VSLAM 稳定...')
                pause_start = time.time()
                while not stop_event.is_set() and (time.time() - pause_start) < pause_s:
                    node.publish_offboard_heartbeat()
                    node.publish_setpoint(
                        x=origin_x, y=origin_y, z=current_z, yaw=origin_yaw)
                    time.sleep(dt)
                    rclpy.spin_once(node, timeout_sec=0.001)

                    # 暂停期间仍需安全检查
                    if node.failsafe:
                        print('✗ 飞控触发故障保护! 中止起飞。')
                        return False
                    if not node.armed:
                        print('✗ 飞控意外锁定! 中止起飞。')
                        return False
                    range_gate_failure = node.airborne_range_gate_failure(
                        takeoff_sensor_agl + current_z - origin_z)
                    if range_gate_failure:
                        return trigger_controlled_landing(
                            node, range_gate_failure)
                    failure = airborne_localization_failure(node)
                    if failure:
                        return trigger_controlled_landing(node, failure)

                    xy_error = math.hypot(
                        node.pos_x - origin_x, node.pos_y - origin_y)
                    if xy_error > max_xy_error_m:
                        print(f'✗ 暂停期间水平漂移过大: XY误差={xy_error:.2f}m，已中止。')
                        node.land_cmd()
                        return False

                if stop_event.is_set():
                    return False
                print(f'✓ 暂停结束，继续爬升至 {stage_label}\n')

        print(f'  爬升 → {stage_label}')
        # 清除上一段的稳定标记，本段重新判断
        if hasattr(node, '_center_stable_since'):
            del node._center_stable_since
        # 如果中途悬停时有高度跳变，同步修正
        jump_z = node.consume_pose_jump_z()
        if abs(jump_z) > 0.75:
            origin_z += jump_z
            stage_target_z += jump_z
            current_z += jump_z
            print(f'⚠  检测到高度参考跳变 {jump_z:+.2f}m，已同步平移悬停目标。')

        while not stop_event.is_set():
            if current_z < stage_target_z:
                current_z = min(stage_target_z, current_z + climb_rate * dt)
            else:
                current_z = stage_target_z

            node.publish_offboard_heartbeat()
            node.publish_setpoint(x=origin_x, y=origin_y, z=current_z, yaw=origin_yaw)
            time.sleep(dt)
            rclpy.spin_once(node, timeout_sec=0.001)

            jump_z = node.consume_pose_jump_z()
            if abs(jump_z) > 0.75:
                origin_z += jump_z
                stage_target_z += jump_z
                current_z += jump_z
                print(f'⚠  检测到高度参考跳变 {jump_z:+.2f}m，已同步平移悬停目标。')

            yaw_err = _yaw_delta_deg(node.yaw_ned, arm_yaw)
            if math.isfinite(yaw_err) and abs(yaw_err) > node._max_yaw_jump_deg:
                print(f'✗ 解锁后 yaw 跳变 {yaw_err:+.1f}° '
                      f'(阈值 ±{node._max_yaw_jump_deg:.0f}°)，已中止。')
                print('   检查坐标对齐；保持 EKF2_EV_CTRL=1（不融合EV yaw/速度）。')
                node.land_cmd()
                return False

            if node.failsafe:
                print('✗ 飞控触发故障保护! 中止起飞。')
                return False
            if not node.armed:
                print('✗ 飞控意外锁定! 中止起飞。')
                return False

            range_gate_failure = node.airborne_range_gate_failure(
                takeoff_sensor_agl + current_z - origin_z)
            if range_gate_failure:
                return trigger_controlled_landing(node, range_gate_failure)

            failure = airborne_localization_failure(node)
            if failure:
                return trigger_controlled_landing(node, failure)

            sensor_agl = node.dist_bottom if math.isfinite(node.dist_bottom) else float('nan')
            center_agl = _center_agl(node)
            actual_h = node.pos_z - origin_z
            setpoint_h = current_z - origin_z
            xy_error = math.hypot(node.pos_x - origin_x, node.pos_y - origin_y)

            # 全程监控：水平误差过大立即降落，避免爬到目标高度才发现已偏出
            if xy_error > max_xy_error_m:
                print(f'✗ 爬升中水平漂移过大: XY误差={xy_error:.2f}m '
                      f'(阈值 {max_xy_error_m:.2f}m)，已中止并降落。')
                print(f'   当前位置 ENU=({node.pos_x:.2f},{node.pos_y:.2f}) '
                      f'相对原点 dE={node.pos_x - origin_x:+.2f} '
                      f'dN={node.pos_y - origin_y:+.2f}')
                print('   CSV对照: 若 vslam XY ≫ EKF XY → VSLAM 滑动(非”未融合”); '
                      '查外参/纹理/EVP_NOISE。')
                print(f'   融合标志: cs_ev_pos={node._cs_ev_pos} cs_ev_vel={node._cs_ev_vel} '
                      f'cs_rng_hgt={node._cs_rng_hgt}')
                node.land_cmd()
                return False

            elapsed = time.time() - climb_start_time
            if elapsed - last_diag_time >= 1.0:
                last_diag_time = elapsed
                xy_error, _ = print_takeoff_diagnostics(
                    node, origin_x, origin_y, origin_z, '爬升中')
                if math.isfinite(center_agl):
                    print(f'        中心AGL: {center_agl:.2f}m / 目标 {target_center_agl:.2f}m '
                          f'(激光 {sensor_agl:.2f}m, dZ={actual_h:.1f}/{setpoint_h:.1f}m)')
                else:
                    print(f'        高度: dZ={actual_h:.1f}m / 设定 {setpoint_h:.1f}m (无测距)')

            agl_low = (not math.isfinite(sensor_agl)) or sensor_agl < (
                takeoff_sensor_agl + node._no_liftoff_delta_m)
            if (elapsed > node._no_liftoff_timeout_s and agl_low
                    and actual_h < node._no_liftoff_max_height_m):
                sensor_text = (f'{sensor_agl:.2f}m'
                               if math.isfinite(sensor_agl) else '无效')
                reason = (
                    f'起飞 {elapsed:.1f}s 后 TFmini 未较基线升高 '
                    f'{node._no_liftoff_delta_m:.2f}m '
                    f'(当前 {sensor_text}，基线 {takeoff_sensor_agl:.2f}m)，'
                    f'估计爬升仅 {actual_h:.2f}m；为防止贴地滑移污染视觉里程计，'
                    '立即降落。')
                print(f'✗ {reason}')
                node.land_cmd()
                return False

            # 到达: setpoint 到位 + 实际高度接近阶段目标（或最终目标）
            sp_ready = abs(current_z - stage_target_z) < 1e-3
            is_final_stage = (stage_target_z == climb_stages[-1][0])
            if is_final_stage:
                # 最终段: 必须实际到达 target_center_agl
                center_ok = (
                    math.isfinite(center_agl)
                    and abs(center_agl - target_center_agl) < 0.05
                )
            else:
                # 中间段: setpoint 到位即可，不等实际高度（让悬停时自然到达）
                center_ok = True
            if sp_ready and center_ok:
                if not hasattr(node, '_center_stable_since'):
                    node._center_stable_since = time.time()
                elif time.time() - node._center_stable_since >= 1.0:
                    break  # 本段到达，继续下一段或进入悬停
            else:
                if hasattr(node, '_center_stable_since'):
                    del node._center_stable_since

    if stop_event.is_set():
        print('收到停止请求，结束爬升并进入降落流程...', flush=True)
        node.land_cmd()
        return True

    center_final = _center_agl(node)
    sensor_final = node.dist_bottom if math.isfinite(node.dist_bottom) else float('nan')
    if math.isfinite(center_final):
        print(f'✓ 已到达目标附近: 中心AGL≈{center_final:.2f}m '
              f'(激光 {sensor_final:.2f}m)，进入悬停...')
    else:
        print(f'✓ 已到达目标高度附近 (相对原点 {node.pos_z - origin_z:.1f}m)，进入悬停...')
    print('')

    # 悬停: XY 拉回起飞原点（不再锁死漂移位置），Z 按测距一次性微调
    hover_x, hover_y, hover_z = _enter_hover_setpoints(
        node, target_sensor_agl, origin_z, target_z, origin_x, origin_y)
    print('  按 1 降落')

    # ── 第 5 步: 定点悬停 ──
    # dist_bottom 已经是 PX4 EKF 的融合输出，不能再用它连续修改 z setpoint，
    # 否则会形成 EKF 高度环之外的第二个闭环，放大测距噪声和延迟。
    # 但可以用低频+阻尼的方式做缓慢 Z 微调，补偿 EKF 长期漂移而不引发振荡。
    hover_start_time = time.time()
    last_diag_time = 0.0     # 诊断打印间隔计时
    last_trim_time = 0.0     # Z 微调间隔计时
    _trim_damping = 0.3       # Z 微调阻尼因子 (0~1, 越小越保守)
    _trim_interval = 5.0      # Z 微调间隔 (秒)
    _trim_threshold = 0.08    # 触发微调的 AGL 偏差阈值 (m)
    _max_trim_per_cycle = 0.1 # 单次最大 Z 调整量 (m)

    while not stop_event.is_set():
        jump_z = node.consume_pose_jump_z()
        if abs(jump_z) > 0.75:
            hover_z += jump_z
            print(f'⚠  悬停中检测到高度参考跳变 {jump_z:+.2f}m，已同步平移悬停目标。')

        yaw_err = _yaw_delta_deg(node.yaw_ned, arm_yaw)
        if math.isfinite(yaw_err) and abs(yaw_err) > node._max_yaw_jump_deg:
            print(f'✗ 悬停中 yaw 跳变 {yaw_err:+.1f}° '
                  f'(阈值 ±{node._max_yaw_jump_deg:.0f}°)，已中止降落。')
            node.land_cmd()
            return False

        failure = airborne_localization_failure(node)
        if failure:
            return trigger_controlled_landing(node, failure)

        node.publish_offboard_heartbeat()
        node.publish_setpoint(
            x=hover_x,
            y=hover_y,
            z=hover_z,
            yaw=origin_yaw,
        )
        time.sleep(0.1)
        rclpy.spin_once(node, timeout_sec=0.001)

        # ── 悬停诊断 + 水平中止 ──
        elapsed = time.time() - hover_start_time
        dx = node.pos_x - hover_x
        dy = node.pos_y - hover_y
        xy_err = math.hypot(dx, dy)
        if xy_err > max_xy_error_m:
            print(f'✗ 悬停中水平漂移过大: XY误差={xy_err:.2f}m '
                  f'(阈值 {max_xy_error_m:.2f}m)，已中止并降落。')
            node.land_cmd()
            return False

        if elapsed - last_diag_time >= 2.0:
            last_diag_time = elapsed
            sensor_agl = node.dist_bottom if math.isfinite(node.dist_bottom) else float('nan')
            center_agl = _center_agl(node)
            if math.isfinite(center_agl):
                print(f'[悬停 {elapsed:.0f}s] 中心AGL: {center_agl:.2f}m '
                      f'/ 目标 {target_center_agl:.2f}m '
                      f'(激光 {sensor_agl:.2f}m, XY漂移={xy_err:.2f}m)')
            else:
                print(f'[悬停 {elapsed:.0f}s] dZ={node.pos_z - origin_z:.1f}m '
                      f'(无测距, XY漂移={xy_err:.2f}m)')

        # ── 缓慢 Z 微调 (低频 + 阻尼, 补偿 EKF 长期漂移) ──
        if (elapsed - last_trim_time >= _trim_interval
                and node.rangefinder_ready):
            last_trim_time = elapsed
            center_agl = _center_agl(node)
            if math.isfinite(center_agl):
                agl_error = target_center_agl - center_agl
                if abs(agl_error) > _trim_threshold:
                    trim = agl_error * _trim_damping
                    trim = max(-_max_trim_per_cycle,
                               min(_max_trim_per_cycle, trim))
                    hover_z += trim
                    print(f'  ↳ Z微调: AGL偏差 {agl_error:+.2f}m → '
                          f'调整 setpoint {trim:+.2f}m '
                          f'(新 hover_z={hover_z:.2f})')

    return True


# ═══════════════════════════════════════════════════════════════
#  降落流程
# ═══════════════════════════════════════════════════════════════

def do_land(node: PX4Controller) -> bool:
    """一键降落；近地/卡住时用测距 AGL + PX4 force-disarm(21196)。"""
    print('\n═══ 降落 ═══\n')

    if not node.armed:
        print('飞控未解锁，无需降落')
        return True

    print('发送 NAV_LAND 命令...')
    for _ in range(5):
        node.land_cmd()
        time.sleep(0.2)

    # 等待飞控进入 AUTO_LAND
    print('等待飞控进入 AUTO_LAND...')
    t0 = time.time()
    in_auto_land = False
    while time.time() - t0 < 5.0:
        time.sleep(0.2)
        rclpy.spin_once(node, timeout_sec=0.01)
        if node.nav_state == 18:  # AUTO_LAND
            print('✓ 飞控已进入 AUTO_LAND')
            in_auto_land = True
            break
    if not in_auto_land:
        print(f'⚠  未进入 AUTO_LAND (nav_state={node.nav_state})，继续等待并准备强制锁定')

    # 等待降落完成（此阶段不再处理按键 1；紧急请 Ctrl+C）
    print('等待降落完成...（此阶段按 1 无效，卡住请 Ctrl+C）', flush=True)
    t0 = time.time()
    last_z = node.pos_z
    stuck_since = time.time()
    near_ground_since = 0.0
    last_status_print = 0.0
    ground_agl_m = 0.12  # 激光离地低于此值视为已触地/可贴地强制锁定
    while time.time() - t0 < 45.0:
        time.sleep(0.2)
        rclpy.spin_once(node, timeout_sec=0.01)
        if not node.armed:
            print('✓ 降落完成，飞控已自动锁定!')
            return True

        sensor_agl = node.dist_bottom if math.isfinite(node.dist_bottom) else float('nan')
        # EKF z 在丢激光融合后可卡在负值；优先用测距判断触地
        near_ground = (
            (math.isfinite(sensor_agl) and sensor_agl <= ground_agl_m)
            or (math.isfinite(node.pos_z) and node.pos_z < 0.08)
        )
        if near_ground:
            if near_ground_since <= 0.0:
                near_ground_since = time.time()
            held = time.time() - near_ground_since
            # 贴地超过 2s：不要再空转等待 COM_DISARM_LAND（常因 cs_rng_hgt=0 永不触发）
            if held >= 2.0:
                agl_txt = f'{sensor_agl:.2f}m' if math.isfinite(sensor_agl) else 'n/a'
                print(f'⚠  近地 (激光AGL={agl_txt}, z={node.pos_z:.2f}m) '
                      f'超过 {held:.0f}s，强制锁定...', flush=True)
                break
        else:
            near_ground_since = 0.0

        if abs(node.pos_z - last_z) < 0.05:
            if time.time() - stuck_since > 6.0:
                print(f'⚠  高度卡住约 z={node.pos_z:.2f}m 超过 6s，强制锁定...',
                      flush=True)
                break
        else:
            last_z = node.pos_z
            stuck_since = time.time()

        now = time.time()
        if now - last_status_print >= 1.0:
            last_status_print = now
            agl_txt = f'{sensor_agl:.2f}m' if math.isfinite(sensor_agl) else 'n/a'
            print(f'  等待上锁: z={node.pos_z:.2f}m 激光AGL={agl_txt} '
                  f'cs_rng_hgt={node._cs_rng_hgt} nav={node.nav_state}',
                  flush=True)

    print('降落超时/卡住，发送强制锁定 (param2=21196)...')
    if node.force_disarm_sequence():
        print('✓ 强制锁定成功')
        return True
    print('✗ 强制锁定失败，请手动断桨/用 QGC 锁定!')
    return False


# ═══════════════════════════════════════════════════════════════
#  主程序
# ═══════════════════════════════════════════════════════════════

def main():
    rclpy.init(args=sys.argv)
    node = PX4Controller()

    # 等待飞控状态和稳定本地位姿 (DDS 发现/高度估计稳定可能需要数秒)
    print('等待飞控状态和稳定本地位姿...')
    for i in range(200):
        time.sleep(0.1)
        rclpy.spin_once(node, timeout_sec=0.01)
        if node._status.valid and node.pose_stable_for(2.0):
            print(f'  DDS 链路已连接 (第 {i+1} 次检测)')
            break

    if not node._status.valid or not node.pose_stable_for(2.0):
        print('⚠  警告: 无法同时获取飞控状态和稳定本地位姿 (等待 20s 超时)')
        print('   请先启动: ros2 launch px4_interface vslam_px4.launch.py')
        print('   并确认 /fmu/out/vehicle_odometry 正在发布')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # 记录初始高度参考 (EKF 可能已漂移, ENU)
    initial_z = node.pos_z

    # 打印飞控状态
    armed_str = 'ARMED (已解锁)' if node.armed else 'DISARMED (锁定)'
    print(f'  解锁状态: {armed_str}')
    print(f'  飞前检查: {"✓ 通过" if node.preflight_pass else "✗ 未通过!"}')
    print(f'  当前高度: {node.pos_z - initial_z:.2f}m (EKF原始: {node.pos_z:.2f}m)')
    print(f'  故障保护: {"⚠  触发!" if node.failsafe else "正常"}')
    print('')

    if node.armed:
        print('⚠  飞控当前已解锁! 将直接进入悬停/降落模式。')
        print('')

    # ═══════════════════════════════════════════
    #  交互主循环
    # ═══════════════════════════════════════════

    stop_event = threading.Event()
    takeoff_thread = None

    try:
        # ── 等待起飞指令 ──
        while True:
            try:
                cmd = input(
                    f'按 1 起飞 (中心离地{TAKEOFF_CENTER_AGL_M:.2f}m悬停): '
                ).strip()
            except EOFError:
                print('')
                break

            if cmd == '1':
                break
            else:
                print('  输入 1 起飞，Ctrl+C 退出')

        # ── 起飞 ──
        stop_event.clear()
        takeoff_thread = threading.Thread(
            target=do_takeoff_and_hover,
            args=(node, TAKEOFF_CENTER_AGL_M, stop_event),
            daemon=True,
        )
        takeoff_thread.start()

        # ── 等待降落指令（必须输入 1 后按回车；高度监控刷屏时仍要回车才生效）──
        print('飞行中: 输入 1 后按回车 → 降落 | Ctrl+C → 紧急停止', flush=True)
        while takeoff_thread.is_alive():
            try:
                cmd = input()
            except EOFError:
                print('')
                break

            if cmd.strip() == '1':
                print('收到降落指令!', flush=True)
                stop_event.set()
                break
            elif cmd.strip():
                print('飞行中请输入 1 后回车降落，或 Ctrl+C 紧急停止', flush=True)

        # 等待起飞/悬停线程结束（中止时可能已发 NAV_LAND）
        if takeoff_thread.is_alive():
            takeoff_thread.join(timeout=5.0)

        time.sleep(0.5)

        # ── 降落（含近地 force-disarm；此阶段不再读键盘，卡住请 Ctrl+C）──
        if node.armed:
            do_land(node)
        else:
            print('飞控已锁定，跳过降落流程。')

    except KeyboardInterrupt:
        print('\n用户中断')
        stop_event.set()
        if takeoff_thread and takeoff_thread.is_alive():
            takeoff_thread.join(timeout=2.0)
        # 如果仍在飞行，尝试降落
        if node.armed:
            # SIGINT can already have shut down the rclpy context.  Do not
            # turn an otherwise safe, disarmed cleanup into a misleading
            # publish exception; a valid context is required to send LAND.
            if rclpy.ok():
                print('紧急降落...')
                do_land(node)
            else:
                print('ROS 上下文已关闭，无法发送降落命令；请确认飞控已锁定。')

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print('\n程序退出。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
