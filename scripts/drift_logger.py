#!/usr/bin/env python3
"""
Drift Diagnostic Logger — records all SLAM odometry sources + PX4 EKF state to CSV.

Subscribes:
  /visual_slam/tracking/odometry   (nav_msgs/Odometry) — cuVSLAM VIO source
  /lidar/relay/odometry             (nav_msgs/Odometry) — lidar SLAM relay (if active)
  /rtabmap/relay/odometry           (nav_msgs/Odometry) — rtabmap relay (if active)
  /fmu/out/vehicle_local_position   — PX4 EKF local position
  /fmu/out/estimator_status_flags   — PX4 EKF innovation check flags

Logs timestamped CSV at 10 Hz to /tmp/drift_log_<date>_<time>.csv

Usage:
  ros2 run px4_interface drift_logger.py
  DRIFT_LOG_DIR=/home/cfly/ros2_ws/logs ros2 run px4_interface drift_logger.py
"""

import csv
import os
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
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition, EstimatorStatusFlags


class DriftLogger(Node):
    def __init__(self):
        super().__init__('drift_logger')

        log_dir = os.environ.get('DRIFT_LOG_DIR', '/tmp')
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(log_dir, f'drift_log_{timestamp}.csv')
        self._csv_file = open(self._csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'timestamp', 'elapsed_s',
            'vslam_x', 'vslam_y', 'vslam_z',
            'vslam_vx', 'vslam_vy', 'vslam_vz',
            'vslam_qw', 'vslam_qx', 'vslam_qy', 'vslam_qz',
            'lidar_x', 'lidar_y', 'lidar_z',
            'lidar_vx', 'lidar_vy', 'lidar_vz',
            'lidar_qw', 'lidar_qx', 'lidar_qy', 'lidar_qz',
            'rtabmap_x', 'rtabmap_y', 'rtabmap_z',
            'rtabmap_vx', 'rtabmap_vy', 'rtabmap_vz',
            'rtabmap_qw', 'rtabmap_qx', 'rtabmap_qy', 'rtabmap_qz',
            'px4_x', 'px4_y', 'px4_z',
            'px4_vx', 'px4_vy', 'px4_vz',
            'dist_bottom', 'dist_bottom_valid',
            'cs_ev_pos', 'cs_ev_vel', 'cs_ev_hgt', 'cs_ev_yaw',
            'cs_rng_hgt', 'cs_baro_hgt', 'cs_gps_hgt',
            'reject_hor_pos', 'reject_hor_vel', 'reject_ver_pos',
            'reject_hagl', 'cs_inertial_dead_reckoning',
            'vslam_stamp_ns', 'vslam_source_age_ms', 'vslam_rx_age_ms',
            'lidar_stamp_ns', 'lidar_source_age_ms', 'lidar_rx_age_ms',
            'rtabmap_stamp_ns', 'rtabmap_source_age_ms', 'rtabmap_rx_age_ms',
            'localization_x', 'localization_y', 'localization_z',
            'localization_cov_x', 'localization_cov_y',
            'localization_stamp_ns', 'localization_source_age_ms',
            'localization_rx_age_ms',
            'loop_closure_id', 'proximity_detection_id',
            'px4_timestamp_us', 'px4_timestamp_sample_us',
            'px4_rx_age_ms',
            'flags_timestamp_us', 'flags_timestamp_sample_us',
            'flags_rx_age_ms',
        ])
        self._csv_file.flush()

        self._start_time = time.time()

        # PX4 uXRCE-DDS output topics are BEST_EFFORT + transient-local in
        # flight. Default Reliable subscriptions silently miss them, which
        # would invalidate EKF-vs-VIO diagnosis.
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscriptions
        self._vslam_sub = self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry', self._vslam_cb,
            qos_sensor)
        self._lidar_sub = self.create_subscription(
            Odometry, '/lidar/relay/odometry', self._lidar_cb, qos_sensor)
        self._rtabmap_sub = self.create_subscription(
            Odometry, '/rtabmap/relay/odometry', self._rtabmap_cb, qos_sensor)
        self._localization_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/rtabmap/localization_pose',
            self._localization_cb, qos_sensor)
        try:
            from rtabmap_msgs.msg import Info as RtabmapInfo
            self._info_sub = self.create_subscription(
                RtabmapInfo, '/rtabmap/info', self._info_cb, qos_sensor)
        except Exception as exc:  # pragma: no cover - optional at import time
            self.get_logger().warn(
                f'rtabmap info logging disabled: {exc}')
            self._info_sub = None
        self._local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._local_pos_cb, qos_px4)
        self._flags_sub = self.create_subscription(
            EstimatorStatusFlags, '/fmu/out/estimator_status_flags',
            self._flags_cb, qos_px4)

        # State
        self._latest_vslam = None
        self._latest_lidar = None
        self._latest_rtabmap = None
        self._latest_localization = None
        self._latest_local_pos = None
        self._latest_flags = None
        self._loop_closure_id = 0
        self._proximity_detection_id = 0
        self._vslam_stamp = None
        self._lidar_stamp = None
        self._rtabmap_stamp = None
        self._localization_stamp = None
        self._local_pos_stamp = None
        self._flags_stamp = None

        # Log timer @ 10 Hz
        self._log_timer = self.create_timer(0.1, self._log_row)
        # Flush timer @ 1 Hz (keep CSV safe on crash)
        self._flush_timer = self.create_timer(1.0, self._flush_csv)

        self.get_logger().info(f'Drift Logger started → {self._csv_path}')

    def _vslam_cb(self, msg: Odometry):
        self._latest_vslam = msg
        self._vslam_stamp = self.get_clock().now()

    def _lidar_cb(self, msg: Odometry):
        self._latest_lidar = msg
        self._lidar_stamp = self.get_clock().now()

    def _rtabmap_cb(self, msg: Odometry):
        self._latest_rtabmap = msg
        self._rtabmap_stamp = self.get_clock().now()

    def _localization_cb(self, msg: PoseWithCovarianceStamped):
        self._latest_localization = msg
        self._localization_stamp = self.get_clock().now()

    def _info_cb(self, msg):
        self._loop_closure_id = int(msg.loop_closure_id)
        self._proximity_detection_id = int(msg.proximity_detection_id)

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self._latest_local_pos = msg
        self._local_pos_stamp = self.get_clock().now()

    def _flags_cb(self, msg: EstimatorStatusFlags):
        self._latest_flags = msg
        self._flags_stamp = self.get_clock().now()

    @staticmethod
    def _odom_xyzv(odom):
        """Extract (x,y,z, vx,vy,vz, qw,qx,qy,qz) from Odometry, or Nones."""
        if odom is None:
            return (None,) * 10
        p = odom.pose.pose.position
        o = odom.pose.pose.orientation
        t = odom.twist.twist.linear
        return (p.x, p.y, p.z, t.x, t.y, t.z, o.w, o.x, o.y, o.z)

    @staticmethod
    def _age_ms(stamp, now):
        if stamp is None:
            return None
        return (now - stamp).nanoseconds * 1e-6

    @staticmethod
    def _source_stamp_ns(msg):
        if msg is None:
            return None
        stamp = msg.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @classmethod
    def _source_age_ms(cls, msg, now):
        stamp_ns = cls._source_stamp_ns(msg)
        if not stamp_ns:
            return None
        return (now.nanoseconds - stamp_ns) * 1e-6

    def _log_row(self):
        now = self.get_clock().now()
        elapsed = time.time() - self._start_time
        vslam = self._odom_xyzv(self._latest_vslam)
        lidar = self._odom_xyzv(self._latest_lidar)
        rtabmap = self._odom_xyzv(self._latest_rtabmap)

        lp = self._latest_local_pos
        px4 = (None,) * 6
        dist_bottom = None
        dist_bottom_valid = None
        if lp is not None:
            px4 = (lp.x, lp.y, lp.z, lp.vx, lp.vy, lp.vz)
            dist_bottom = lp.dist_bottom
            dist_bottom_valid = lp.dist_bottom_valid

        flags = self._latest_flags
        cs_ev_pos = flags.cs_ev_pos if flags else None
        cs_ev_vel = flags.cs_ev_vel if flags else None
        cs_ev_hgt = flags.cs_ev_hgt if flags else None
        cs_ev_yaw = flags.cs_ev_yaw if flags else None
        cs_rng_hgt = flags.cs_rng_hgt if flags else None
        cs_baro_hgt = flags.cs_baro_hgt if flags else None
        cs_gps_hgt = flags.cs_gps_hgt if flags else None
        innovation_flags = (
            flags.reject_hor_pos, flags.reject_hor_vel, flags.reject_ver_pos,
            flags.reject_hagl, flags.cs_inertial_dead_reckoning,
        ) if flags else (None,) * 5

        source_messages = (
            (self._latest_vslam, self._vslam_stamp),
            (self._latest_lidar, self._lidar_stamp),
            (self._latest_rtabmap, self._rtabmap_stamp),
        )
        source_timing = []
        for msg, received_at in source_messages:
            stamp_ns = self._source_stamp_ns(msg)
            source_age = self._source_age_ms(msg, now)
            rx_age = self._age_ms(received_at, now)
            source_timing.extend((
                stamp_ns if stamp_ns is not None else '',
                f'{source_age:.1f}' if source_age is not None else '',
                f'{rx_age:.1f}' if rx_age is not None else '',
            ))

        localization = self._latest_localization
        localization_values = (None,) * 5
        if localization is not None:
            p = localization.pose.pose.position
            c = localization.pose.covariance
            localization_values = (p.x, p.y, p.z, c[0], c[7])
        localization_stamp_ns = self._source_stamp_ns(localization)
        localization_source_age = self._source_age_ms(localization, now)
        localization_rx_age = self._age_ms(self._localization_stamp, now)

        self._csv_writer.writerow([
            datetime.now().isoformat(), f'{elapsed:.3f}',
            *vslam, *lidar, *rtabmap, *px4,
            dist_bottom, dist_bottom_valid,
            cs_ev_pos, cs_ev_vel, cs_ev_hgt, cs_ev_yaw,
            cs_rng_hgt, cs_baro_hgt, cs_gps_hgt,
            *innovation_flags, *source_timing, *localization_values,
            localization_stamp_ns if localization_stamp_ns is not None else '',
            (f'{localization_source_age:.1f}'
             if localization_source_age is not None else ''),
            (f'{localization_rx_age:.1f}'
             if localization_rx_age is not None else ''),
            self._loop_closure_id, self._proximity_detection_id,
            lp.timestamp if lp else '', lp.timestamp_sample if lp else '',
            (f'{self._age_ms(self._local_pos_stamp, now):.1f}'
             if self._local_pos_stamp is not None else ''),
            flags.timestamp if flags else '',
            flags.timestamp_sample if flags else '',
            (f'{self._age_ms(self._flags_stamp, now):.1f}'
             if self._flags_stamp is not None else ''),
        ])

    def _flush_csv(self):
        self._csv_file.flush()

    def destroy_node(self):
        self._csv_file.close()
        self.get_logger().info(f'Drift log saved: {self._csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DriftLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
