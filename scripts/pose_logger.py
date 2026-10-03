#!/usr/bin/env python3
"""
pose_logger.py — 飞行位姿自动记录节点

订阅:
  /uav/state/pose                      EKF ENU 位置
  /fmu/out/vehicle_attitude            NED 姿态
  /visual_slam/tracking/odometry       原始 VSLAM ENU（对照漂移源）
  /fmu/in/vehicle_visual_odometry      Bridge→PX4 EV（含 quality）
  /fmu/out/estimator_status_flags      EV/测距融合开关

用法:
    python3 pose_logger.py [--rate 10] [--dir ~/ros2_ws/logs]

输出:
    ~/ros2_ws/logs/flight_pose_YYYYMMDD_HHMMSS.csv

由 takeoff_test.sh 自动启动 (POSE_LOG=1)。
"""

import argparse
import csv
import math
import os
import signal
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    ActuatorMotors,
    DistanceSensor,
    EstimatorStatusFlags,
    TimesyncStatus,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleStatus,
)


def quat_to_euler_ned(q):
    """四元数 → NED Euler 角 (rad)。q = [w, x, y, z]。"""
    w, x, y, z = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def quat_to_yaw_enu(q):
    """geometry_msgs 四元数 (x,y,z,w) → ENU yaw (rad)。"""
    x, y, z, w = (float(q.x), float(q.y), float(q.z), float(q.w))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _fmt(v, nd=4):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return ''
    return f'{v:.{nd}f}'


def _fmt_bool(v):
    if v is None:
        return ''
    return '1' if v else '0'


class PoseLogger(Node):
    """订阅位姿话题，定时写入 CSV 日志"""

    def __init__(self, rate_hz: float, log_dir: str):
        super().__init__('pose_logger')

        self._rate_hz = max(1.0, min(rate_hz, 50.0))
        self._period = 1.0 / self._rate_hz

        self._pos_x = float('nan')
        self._pos_y = float('nan')
        self._pos_z = float('nan')
        self._pos_at = 0.0
        self._has_pos = False

        self._roll = float('nan')
        self._pitch = float('nan')
        self._yaw = float('nan')
        self._att_at = 0.0
        self._has_att = False

        self._vslam_x = float('nan')
        self._vslam_y = float('nan')
        self._vslam_z = float('nan')
        self._vslam_yaw = float('nan')
        self._vslam_at = 0.0
        self._has_vslam = False

        self._ev_n = float('nan')
        self._ev_e = float('nan')
        self._ev_d = float('nan')
        self._ev_quality = float('nan')
        self._ev_vn = float('nan')
        self._ev_ve = float('nan')
        self._ev_vd = float('nan')
        self._ev_reset_counter = 0
        self._ev_pos_var_xy = float('nan')
        self._ev_vel_var_xy = float('nan')
        self._ev_at = 0.0
        self._has_ev = False

        self._local = None
        self._local_at = 0.0
        self._setpoint = None
        self._setpoint_at = 0.0
        self._range = None
        self._range_at = 0.0
        self._timesync = None
        self._timesync_at = 0.0

        self._cs_ev_pos = None
        self._cs_ev_vel = None
        self._cs_ev_hgt = None
        self._cs_ev_yaw = None
        self._cs_rng_hgt = None
        self._flags_at = 0.0
        self._has_flags = False
        self._reject_hor_pos = None
        self._reject_hor_vel = None
        self._reject_ver_pos = None
        self._reject_hagl = None
        self._dead_reckoning = None
        self._pose_source_stamp_ns = 0
        self._vslam_source_stamp_ns = 0
        self._ev_timestamp_us = 0
        self._ev_timestamp_sample_us = 0
        self._flags_timestamp_us = 0
        self._flags_timestamp_sample_us = 0

        self._rtab_x = float('nan')
        self._rtab_y = float('nan')
        self._rtab_z = float('nan')
        self._rtab_at = 0.0
        self._rtab_source_stamp_ns = 0
        self._laser_x = float('nan')
        self._laser_y = float('nan')
        self._laser_z = float('nan')
        self._laser_at = 0.0
        self._laser_source_stamp_ns = 0
        self._localization_x = float('nan')
        self._localization_y = float('nan')
        self._localization_z = float('nan')
        self._localization_cov_x = float('nan')
        self._localization_cov_y = float('nan')
        self._localization_at = 0.0
        self._localization_source_stamp_ns = 0
        self._loop_closure_id = 0
        self._proximity_detection_id = 0
        self._info_at = 0.0

        self._motor_m1 = float('nan')
        self._motor_m2 = float('nan')
        self._motor_m3 = float('nan')
        self._motor_m4 = float('nan')
        self._motor_at = 0.0
        self._has_motor = False
        self._arming_state = None
        self._nav_state = None
        self._vehicle_status_at = 0.0

        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_ev = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 必须挂到 self，否则 Subscription 被 GC 后收不到数据
        self._pose_sub = self.create_subscription(
            PoseStamped, '/uav/state/pose', self._pose_cb, 10)
        self._att_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self._att_cb, qos_px4)
        self._vslam_sub = self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry', self._vslam_cb, qos_ev)
        self._ev_sub = self.create_subscription(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry',
            self._ev_cb, qos_ev)
        self._flags_sub = self.create_subscription(
            EstimatorStatusFlags, '/fmu/out/estimator_status_flags',
            self._flags_cb, qos_px4)
        self._local_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._local_cb, qos_px4)
        self._setpoint_sub = self.create_subscription(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
            self._setpoint_cb, qos_ev)
        self._range_sub = self.create_subscription(
            DistanceSensor, '/fmu/out/distance_sensor',
            self._range_cb, qos_px4)
        self._timesync_sub = self.create_subscription(
            TimesyncStatus, '/fmu/out/timesync_status',
            self._timesync_cb, qos_px4)
        self._motor_sub = self.create_subscription(
            ActuatorMotors, '/fmu/out/actuator_motors',
            self._motor_cb, qos_px4)
        self._vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self._vehicle_status_cb, qos_px4)
        self._rtab_sub = self.create_subscription(
            Odometry, '/rtabmap/relay/odometry', self._rtab_cb, qos_ev)
        # /laser_odom 随激光 SLAM 于 2026-09-14 移除：该订阅再无发布者，
        # laser_* CSV 列会保持为空（列位保留以维持既有 CSV/分析脚本兼容）。
        self._laser_sub = self.create_subscription(
            Odometry, '/laser_odom', self._laser_cb, qos_ev)
        self._localization_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/rtabmap/localization_pose',
            self._localization_cb, qos_ev)
        try:
            from rtabmap_msgs.msg import Info as RtabmapInfo
            self._info_sub = self.create_subscription(
                RtabmapInfo, '/rtabmap/info', self._info_cb, qos_ev)
        except Exception as exc:
            self.get_logger().warn(f'rtabmap info logging disabled: {exc}')
            self._info_sub = None

        os.makedirs(log_dir, exist_ok=True)
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(log_dir, f'flight_pose_{timestamp_str}.csv')
        self._csv_file = open(self._csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)

        # 前 10 列保持旧格式兼容；后续为对照诊断列
        self._csv_writer.writerow([
            'timestamp',
            'x', 'y', 'z',
            'roll_deg', 'pitch_deg', 'yaw_deg',
            'roll_rad', 'pitch_rad', 'yaw_rad',
            'vslam_x', 'vslam_y', 'vslam_z', 'vslam_yaw_deg',
            'ev_n', 'ev_e', 'ev_d', 'ev_quality',
            'cs_ev_pos', 'cs_ev_vel', 'cs_ev_hgt', 'cs_ev_yaw', 'cs_rng_hgt',
            'distance_from_origin',
            'pose_stamp_ns', 'pose_source_age_ms', 'pose_rx_age_ms',
            'vslam_stamp_ns', 'vslam_source_age_ms', 'vslam_rx_age_ms',
            'ev_timestamp_us', 'ev_timestamp_sample_us', 'ev_rx_age_ms',
            'flags_timestamp_us', 'flags_timestamp_sample_us', 'flags_rx_age_ms',
            'reject_hor_pos', 'reject_hor_vel', 'reject_ver_pos',
            'reject_hagl', 'cs_inertial_dead_reckoning',
            'rtabmap_x', 'rtabmap_y', 'rtabmap_z',
            'rtabmap_stamp_ns', 'rtabmap_source_age_ms', 'rtabmap_rx_age_ms',
            'localization_x', 'localization_y', 'localization_z',
            'localization_cov_x', 'localization_cov_y',
            'localization_stamp_ns', 'localization_source_age_ms',
            'localization_rx_age_ms',
            'loop_closure_id', 'proximity_detection_id',
            'motor_m1', 'motor_m2', 'motor_m3', 'motor_m4',
            'arming_state', 'nav_state',
            'vslam_vx', 'vslam_vy', 'vslam_vz_body',
            'ev_vn', 'ev_ve', 'ev_vd', 'ev_reset_counter',
            'ev_pos_var_xy', 'ev_vel_var_xy',
            'local_n', 'local_e', 'local_d',
            'local_vn', 'local_ve', 'local_vd',
            'local_xy_valid', 'local_z_valid', 'local_dead_reckoning',
            'local_eph', 'local_epv', 'dist_bottom', 'dist_bottom_valid',
            'local_xy_reset_counter', 'local_z_reset_counter',
            'setpoint_n', 'setpoint_e', 'setpoint_d',
            'setpoint_vn', 'setpoint_ve', 'setpoint_vd', 'setpoint_yaw',
            'setpoint_xy_error_m', 'setpoint_z_error_m',
            'range_m', 'range_quality', 'range_variance',
            'timesync_offset_us', 'timesync_rtt_us', 'timesync_source',
            'laser_x', 'laser_y', 'laser_z',
            'laser_stamp_ns', 'laser_source_age_ms', 'laser_rx_age_ms',
        ])
        self._csv_file.flush()
        self._last_flush_bucket = -1

        self.get_logger().info(
            f'位姿日志已启动: {self._csv_path}  '
            f'(频率 {self._rate_hz:.1f} Hz, 含 VSLAM/EV/flags 对照)'
        )
        self._timer = self.create_timer(self._period, self._tick)

    def _pose_cb(self, msg: PoseStamped):
        self._pos_x = float(msg.pose.position.x)
        self._pos_y = float(msg.pose.position.y)
        self._pos_z = float(msg.pose.position.z)
        self._pos_at = time.monotonic()
        self._pose_source_stamp_ns = self._stamp_ns(msg.header.stamp)
        self._has_pos = True

    def _att_cb(self, msg: VehicleAttitude):
        q = msg.q
        if len(q) >= 4 and not math.isnan(q[0]):
            r, p, y = quat_to_euler_ned(q)
            self._roll, self._pitch, self._yaw = r, p, y
            self._att_at = time.monotonic()
            self._has_att = True

    def _vslam_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._vslam_x = float(p.x)
        self._vslam_y = float(p.y)
        self._vslam_z = float(p.z)
        self._vslam_yaw = quat_to_yaw_enu(msg.pose.pose.orientation)
        self._vslam_vx = float(msg.twist.twist.linear.x)
        self._vslam_vy = float(msg.twist.twist.linear.y)
        self._vslam_vz_body = float(msg.twist.twist.linear.z)
        self._vslam_at = time.monotonic()
        self._vslam_source_stamp_ns = self._stamp_ns(msg.header.stamp)
        self._has_vslam = True

    def _ev_cb(self, msg: VehicleOdometry):
        self._ev_n = float(msg.position[0])
        self._ev_e = float(msg.position[1])
        self._ev_d = float(msg.position[2])
        self._ev_quality = float(msg.quality)
        self._ev_vn = float(msg.velocity[0])
        self._ev_ve = float(msg.velocity[1])
        self._ev_vd = float(msg.velocity[2])
        self._ev_reset_counter = int(msg.reset_counter)
        self._ev_pos_var_xy = max(
            float(msg.position_variance[0]), float(msg.position_variance[1]))
        self._ev_vel_var_xy = max(
            float(msg.velocity_variance[0]), float(msg.velocity_variance[1]))
        self._ev_at = time.monotonic()
        self._ev_timestamp_us = int(msg.timestamp)
        self._ev_timestamp_sample_us = int(msg.timestamp_sample)
        self._has_ev = True

    def _flags_cb(self, msg: EstimatorStatusFlags):
        self._cs_ev_pos = bool(msg.cs_ev_pos)
        self._cs_ev_vel = bool(msg.cs_ev_vel)
        self._cs_ev_hgt = bool(msg.cs_ev_hgt)
        self._cs_ev_yaw = bool(msg.cs_ev_yaw)
        self._cs_rng_hgt = bool(msg.cs_rng_hgt)
        self._reject_hor_pos = bool(msg.reject_hor_pos)
        self._reject_hor_vel = bool(msg.reject_hor_vel)
        self._reject_ver_pos = bool(msg.reject_ver_pos)
        self._reject_hagl = bool(msg.reject_hagl)
        self._dead_reckoning = bool(msg.cs_inertial_dead_reckoning)
        self._flags_timestamp_us = int(msg.timestamp)
        self._flags_timestamp_sample_us = int(msg.timestamp_sample)
        self._flags_at = time.monotonic()
        self._has_flags = True

    def _local_cb(self, msg: VehicleLocalPosition):
        self._local = msg
        self._local_at = time.monotonic()

    def _setpoint_cb(self, msg: TrajectorySetpoint):
        self._setpoint = msg
        self._setpoint_at = time.monotonic()

    def _range_cb(self, msg: DistanceSensor):
        # TFmini is downward-facing. Ignore any other range sensor on the bus.
        if msg.orientation == DistanceSensor.ROTATION_DOWNWARD_FACING:
            self._range = msg
            self._range_at = time.monotonic()

    def _timesync_cb(self, msg: TimesyncStatus):
        self._timesync = msg
        self._timesync_at = time.monotonic()

    def _motor_cb(self, msg: ActuatorMotors):
        controls = msg.control
        if len(controls) >= 4:
            self._motor_m1 = float(controls[0])
            self._motor_m2 = float(controls[1])
            self._motor_m3 = float(controls[2])
            self._motor_m4 = float(controls[3])
            self._motor_at = time.monotonic()
            self._has_motor = True

    def _vehicle_status_cb(self, msg: VehicleStatus):
        self._arming_state = int(msg.arming_state)
        self._nav_state = int(msg.nav_state)
        self._vehicle_status_at = time.monotonic()

    def _rtab_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._rtab_x, self._rtab_y, self._rtab_z = float(p.x), float(p.y), float(p.z)
        self._rtab_source_stamp_ns = self._stamp_ns(msg.header.stamp)
        self._rtab_at = time.monotonic()

    def _laser_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._laser_x = float(p.x)
        self._laser_y = float(p.y)
        self._laser_z = float(p.z)
        self._laser_source_stamp_ns = self._stamp_ns(msg.header.stamp)
        self._laser_at = time.monotonic()

    def _localization_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        c = msg.pose.covariance
        self._localization_x = float(p.x)
        self._localization_y = float(p.y)
        self._localization_z = float(p.z)
        self._localization_cov_x = float(c[0])
        self._localization_cov_y = float(c[7])
        self._localization_source_stamp_ns = self._stamp_ns(msg.header.stamp)
        self._localization_at = time.monotonic()

    def _info_cb(self, msg):
        self._loop_closure_id = int(msg.loop_closure_id)
        self._proximity_detection_id = int(msg.proximity_detection_id)
        self._info_at = time.monotonic()

    @staticmethod
    def _stamp_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _age_ms(now_mono, received_at):
        return (now_mono - received_at) * 1000.0 if received_at > 0.0 else float('nan')

    def _source_age_ms(self, stamp_ns):
        if stamp_ns <= 0:
            return float('nan')
        return (self.get_clock().now().nanoseconds - stamp_ns) * 1e-6

    def _tick(self):
        now = time.monotonic()
        pos_fresh = self._has_pos and (now - self._pos_at) <= 1.0
        att_fresh = self._has_att and (now - self._att_at) <= 1.0
        vslam_fresh = self._has_vslam and (now - self._vslam_at) <= 1.0
        ev_fresh = self._has_ev and (now - self._ev_at) <= 1.0
        flags_fresh = self._has_flags and (now - self._flags_at) <= 2.0
        rtab_fresh = self._rtab_at > 0.0 and (now - self._rtab_at) <= 1.0
        laser_fresh = self._laser_at > 0.0 and (now - self._laser_at) <= 1.0
        localization_fresh = (
            self._localization_at > 0.0 and (now - self._localization_at) <= 2.0)
        local_fresh = self._local is not None and (now - self._local_at) <= 1.0
        setpoint_fresh = (
            self._setpoint is not None and (now - self._setpoint_at) <= 1.0)
        range_fresh = self._range is not None and (now - self._range_at) <= 1.0
        timesync_fresh = (
            self._timesync is not None and (now - self._timesync_at) <= 2.0)
        vehicle_status_fresh = (
            self._vehicle_status_at > 0.0
            and (now - self._vehicle_status_at) <= 2.0)

        if not (pos_fresh or att_fresh or vslam_fresh):
            return

        ts = time.time()
        px = self._pos_x if pos_fresh else float('nan')
        py = self._pos_y if pos_fresh else float('nan')
        pz = self._pos_z if pos_fresh else float('nan')

        if att_fresh:
            roll, pitch, yaw = self._roll, self._pitch, self._yaw
            roll_deg, pitch_deg, yaw_deg = (
                math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
        else:
            roll = pitch = yaw = float('nan')
            roll_deg = pitch_deg = yaw_deg = float('nan')

        if vslam_fresh:
            vx, vy, vz = self._vslam_x, self._vslam_y, self._vslam_z
            vyaw = math.degrees(self._vslam_yaw)
        else:
            vx = vy = vz = vyaw = float('nan')

        if ev_fresh:
            en, ee, ed, eq = self._ev_n, self._ev_e, self._ev_d, self._ev_quality
        else:
            en = ee = ed = eq = float('nan')

        if flags_fresh:
            f_pos, f_vel, f_hgt, f_yaw, f_rng = (
                self._cs_ev_pos, self._cs_ev_vel, self._cs_ev_hgt,
                self._cs_ev_yaw, self._cs_rng_hgt)
        else:
            f_pos = f_vel = f_hgt = f_yaw = f_rng = None
        reject_flags = (
            self._reject_hor_pos, self._reject_hor_vel, self._reject_ver_pos,
            self._reject_hagl, self._dead_reckoning,
        ) if flags_fresh else (None,) * 5

        local = self._local if local_fresh else None
        setpoint = self._setpoint if setpoint_fresh else None
        range_msg = self._range if range_fresh else None
        timesync = self._timesync if timesync_fresh else None
        if local is not None and setpoint is not None:
            sp_xy_error = math.hypot(
                float(local.x) - float(setpoint.position[0]),
                float(local.y) - float(setpoint.position[1]))
            sp_z_error = abs(float(local.z) - float(setpoint.position[2]))
        else:
            sp_xy_error = sp_z_error = float('nan')

        if pos_fresh and math.isfinite(px) and math.isfinite(py):
            horiz = math.hypot(px, py)
        else:
            horiz = float('nan')

        self._csv_writer.writerow([
            f'{ts:.6f}',
            _fmt(px), _fmt(py), _fmt(pz),
            _fmt(roll_deg, 2), _fmt(pitch_deg, 2), _fmt(yaw_deg, 2),
            _fmt(roll), _fmt(pitch), _fmt(yaw),
            _fmt(vx), _fmt(vy), _fmt(vz), _fmt(vyaw, 2),
            _fmt(en), _fmt(ee), _fmt(ed), _fmt(eq, 0),
            _fmt_bool(f_pos), _fmt_bool(f_vel), _fmt_bool(f_hgt),
            _fmt_bool(f_yaw), _fmt_bool(f_rng),
            _fmt(horiz),
            self._pose_source_stamp_ns if pos_fresh else '',
            _fmt(self._source_age_ms(self._pose_source_stamp_ns), 1) if pos_fresh else '',
            _fmt(self._age_ms(now, self._pos_at), 1) if pos_fresh else '',
            self._vslam_source_stamp_ns if vslam_fresh else '',
            _fmt(self._source_age_ms(self._vslam_source_stamp_ns), 1) if vslam_fresh else '',
            _fmt(self._age_ms(now, self._vslam_at), 1) if vslam_fresh else '',
            self._ev_timestamp_us if ev_fresh else '',
            self._ev_timestamp_sample_us if ev_fresh else '',
            _fmt(self._age_ms(now, self._ev_at), 1) if ev_fresh else '',
            self._flags_timestamp_us if flags_fresh else '',
            self._flags_timestamp_sample_us if flags_fresh else '',
            _fmt(self._age_ms(now, self._flags_at), 1) if flags_fresh else '',
            *(_fmt_bool(v) for v in reject_flags),
            _fmt(self._rtab_x) if rtab_fresh else '',
            _fmt(self._rtab_y) if rtab_fresh else '',
            _fmt(self._rtab_z) if rtab_fresh else '',
            self._rtab_source_stamp_ns if rtab_fresh else '',
            _fmt(self._source_age_ms(self._rtab_source_stamp_ns), 1) if rtab_fresh else '',
            _fmt(self._age_ms(now, self._rtab_at), 1) if rtab_fresh else '',
            _fmt(self._localization_x) if localization_fresh else '',
            _fmt(self._localization_y) if localization_fresh else '',
            _fmt(self._localization_z) if localization_fresh else '',
            _fmt(self._localization_cov_x) if localization_fresh else '',
            _fmt(self._localization_cov_y) if localization_fresh else '',
            self._localization_source_stamp_ns if localization_fresh else '',
            (_fmt(self._source_age_ms(self._localization_source_stamp_ns), 1)
             if localization_fresh else ''),
            _fmt(self._age_ms(now, self._localization_at), 1) if localization_fresh else '',
            self._loop_closure_id if self._info_at > 0.0 else '',
            self._proximity_detection_id if self._info_at > 0.0 else '',
            _fmt(self._motor_m1, 4) if self._has_motor else '',
            _fmt(self._motor_m2, 4) if self._has_motor else '',
            _fmt(self._motor_m3, 4) if self._has_motor else '',
            _fmt(self._motor_m4, 4) if self._has_motor else '',
            self._arming_state if vehicle_status_fresh else '',
            self._nav_state if vehicle_status_fresh else '',
            _fmt(self._vslam_vx) if vslam_fresh else '',
            _fmt(self._vslam_vy) if vslam_fresh else '',
            _fmt(self._vslam_vz_body) if vslam_fresh else '',
            _fmt(self._ev_vn) if ev_fresh else '',
            _fmt(self._ev_ve) if ev_fresh else '',
            _fmt(self._ev_vd) if ev_fresh else '',
            self._ev_reset_counter if ev_fresh else '',
            _fmt(self._ev_pos_var_xy, 5) if ev_fresh else '',
            _fmt(self._ev_vel_var_xy, 5) if ev_fresh else '',
            _fmt(float(local.x)) if local is not None else '',
            _fmt(float(local.y)) if local is not None else '',
            _fmt(float(local.z)) if local is not None else '',
            _fmt(float(local.vx)) if local is not None else '',
            _fmt(float(local.vy)) if local is not None else '',
            _fmt(float(local.vz)) if local is not None else '',
            _fmt_bool(bool(local.xy_valid)) if local is not None else '',
            _fmt_bool(bool(local.z_valid)) if local is not None else '',
            _fmt_bool(bool(local.dead_reckoning)) if local is not None else '',
            _fmt(float(local.eph)) if local is not None else '',
            _fmt(float(local.epv)) if local is not None else '',
            _fmt(float(local.dist_bottom)) if local is not None else '',
            _fmt_bool(bool(local.dist_bottom_valid)) if local is not None else '',
            int(local.xy_reset_counter) if local is not None else '',
            int(local.z_reset_counter) if local is not None else '',
            _fmt(float(setpoint.position[0])) if setpoint is not None else '',
            _fmt(float(setpoint.position[1])) if setpoint is not None else '',
            _fmt(float(setpoint.position[2])) if setpoint is not None else '',
            _fmt(float(setpoint.velocity[0])) if setpoint is not None else '',
            _fmt(float(setpoint.velocity[1])) if setpoint is not None else '',
            _fmt(float(setpoint.velocity[2])) if setpoint is not None else '',
            _fmt(float(setpoint.yaw)) if setpoint is not None else '',
            _fmt(sp_xy_error), _fmt(sp_z_error),
            _fmt(float(range_msg.current_distance)) if range_msg is not None else '',
            int(range_msg.signal_quality) if range_msg is not None else '',
            _fmt(float(range_msg.variance), 6) if range_msg is not None else '',
            int(timesync.estimated_offset) if timesync is not None else '',
            int(timesync.round_trip_time) if timesync is not None else '',
            int(timesync.source_protocol) if timesync is not None else '',
            _fmt(self._laser_x) if laser_fresh else '',
            _fmt(self._laser_y) if laser_fresh else '',
            _fmt(self._laser_z) if laser_fresh else '',
            self._laser_source_stamp_ns if laser_fresh else '',
            (_fmt(self._source_age_ms(self._laser_source_stamp_ns), 1)
             if laser_fresh else ''),
            _fmt(self._age_ms(now, self._laser_at), 1) if laser_fresh else '',
        ])

        bucket = int(now) // 5
        if bucket != self._last_flush_bucket:
            self._last_flush_bucket = bucket
            self._csv_file.flush()

    def close(self):
        if self._csv_file and not self._csv_file.closed:
            self._csv_file.flush()
            self._csv_file.close()
            self.get_logger().info(f'日志已保存: {self._csv_path}')

    def destroy_node(self):
        self.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='飞行位姿自动记录 (CSV 日志)')
    parser.add_argument(
        '--rate', type=float, default=10.0,
        help='记录频率 Hz (默认 10, 范围 1-50)',
    )
    parser.add_argument(
        '--dir', type=str, default=os.path.expanduser('~/ros2_ws/logs'),
        help='日志输出目录 (默认 ~/ros2_ws/logs)',
    )
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    node = PoseLogger(rate_hz=args.rate, log_dir=args.dir)

    def _sig_handler(signum, frame):
        node.get_logger().info(f'收到信号 {signum}，正在保存日志...')
        node.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
