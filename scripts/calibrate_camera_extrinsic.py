#!/usr/bin/env python3
"""
Camera Extrinsic Calibration Tool for D435i on UAV

Measures the pitch and roll of the D435i camera relative to the drone body
by reading the accelerometer when the drone is placed on a LEVEL SURFACE.

Principle:
  1. Place the drone on a level surface (base_link ≈ level).
  2. Subscribe to /camera/camera/imu and collect accelerometer samples.
  3. The averaged gravity vector in the IMU frame tells us the camera's tilt.
  4. Since the drone is level, this tilt IS the camera mounting pitch/roll.

Output:
  camera_pitch, camera_roll (radians) — plug these into
  launch/vslam_cuvslam.launch.py 与 launch/vslam_rtabmap.launch.py（两个方案共用同一组外参）

Frame conventions (for reference):
  - base_link:   FRD — x forward, y right, z down
  - camera_link: optical — z forward, x right, y down
  - camera_imu_optical_frame: same orientation as camera_link (approx)

  When the drone is level and camera is pointing forward horizontally:
    gravity reaction vector in IMU frame ≈ [0, +g, 0]  (y is down → +y is up-reaction)
    camera pitch (+down):  az > 0  (gravity reaction has forward component)
    camera roll  (+right): ax > 0  (gravity reaction has rightward component)

Usage:
  python3 calibrate_camera_extrinsic.py [--samples N] [--output]

  Run this while the SLAM pipeline is running (camera + IMU active).
  Keep the drone STATIONARY on a LEVEL SURFACE.
"""

import argparse
import math
import sys
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

# Match D435i IMU topic QoS: BEST_EFFORT + KEEP_LAST
IMU_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)


class CameraExtrinsicCalibrator(Node):
    def __init__(self, num_samples: int = 200):
        super().__init__('camera_extrinsic_calibrator')
        self.num_samples = num_samples
        self.samples: deque = deque(maxlen=num_samples)
        self.gyro_samples: deque = deque(maxlen=num_samples)

        self.imu_sub = self.create_subscription(
            Imu,
            '/camera/camera/imu',
            self.imu_callback,
            IMU_QOS,
        )

        # Status timer
        self.timer = self.create_timer(1.0, self.status_callback)

        self.get_logger().info(
            f'Collecting {num_samples} IMU samples...\n'
            '  Make sure the drone is on a LEVEL SURFACE and STATIONARY.\n'
            '  Do NOT touch or move the drone during calibration.'
        )

    def imu_callback(self, msg: Imu):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        self.samples.append((ax, ay, az))
        self.gyro_samples.append((
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ))

    def status_callback(self):
        n = len(self.samples)
        if n < self.num_samples:
            self.get_logger().info(
                f'Collecting... {n}/{self.num_samples} samples'
            )
            return

        # We have enough samples — compute result
        self.timer.cancel()
        self.compute_and_report()

    def compute_and_report(self):
        # Average accelerometer readings
        n = len(self.samples)
        sum_ax = sum(s[0] for s in self.samples)
        sum_ay = sum(s[1] for s in self.samples)
        sum_az = sum(s[2] for s in self.samples)
        avg_ax = sum_ax / n
        avg_ay = sum_ay / n
        avg_az = sum_az / n

        # Check stationarity: gyro std should be small
        gyro_magnitudes = [
            math.sqrt(gx * gx + gy * gy + gz * gz)
            for gx, gy, gz in self.gyro_samples
        ]
        mean_gyro = sum(gyro_magnitudes) / len(gyro_magnitudes)
        var_gyro = (
            sum((m - mean_gyro) ** 2 for m in gyro_magnitudes)
            / len(gyro_magnitudes)
        )
        std_gyro = math.sqrt(var_gyro)

        if std_gyro > 0.05:  # rad/s — significant motion
            self.get_logger().warn(
                f'Gyro std = {std_gyro:.4f} rad/s — drone may NOT be '
                f'stationary! Results will be unreliable.\n'
                'Re-run with the drone placed firmly on a level surface.'
            )

        # Gravity magnitude
        g_mag = math.sqrt(avg_ax * avg_ax + avg_ay * avg_ay + avg_az * avg_az)
        g_expected = 9.81
        g_error_pct = abs(g_mag - g_expected) / g_expected * 100

        if g_error_pct > 5.0:
            self.get_logger().warn(
                f'Measured |g| = {g_mag:.3f} m/s² ({g_error_pct:.1f}% off '
                f'expected {g_expected}) — IMU may need calibration or '
                f'drone was moving.'
            )

        # ── Compute pitch / roll ──
        #
        # In camera_imu_optical_frame (z fwd, x right, y down):
        #   Level camera:  [ax≈0, ay≈+g, az≈0]
        #
        # Pitch (camera tilting down, + nose-down):
        #   ay decreases, az increases positively
        #   pitch = atan2(az, ay)
        #
        # Roll (camera tilting right, + right-wing-down):
        #   ax increases positively, ay decreases
        #   roll = atan2(ax, ay)
        #
        # We use ay (the dominant component when level) as the denominator.

        pitch_rad = math.atan2(-avg_az, -avg_ay)
        roll_rad = math.atan2(avg_ax, -avg_ay)

        pitch_deg = math.degrees(pitch_rad)
        roll_deg = math.degrees(roll_rad)

        # ── Report ──
        print()
        print('=' * 60)
        print('  Camera Extrinsic Calibration Results')
        print('=' * 60)
        print()
        print(f'  Samples collected:   {n}')
        print(f'  Avg gravity |g|:     {g_mag:.4f} m/s²  '
              f'(expected ~{g_expected})')
        print(f'  Gyro std (motion):   {std_gyro:.4f} rad/s')
        print()
        print(f'  Raw accelerometer (avg):')
        print(f'    ax = {avg_ax:+.4f} m/s²')
        print(f'    ay = {avg_ay:+.4f} m/s²')
        print(f'    az = {avg_az:+.4f} m/s²')
        print()
        print(f'  Camera mounting pitch: {pitch_rad:+.4f} rad  '
              f'({pitch_deg:+.2f}°)')
        print(f'    (+ = nose-down / camera tilts downward)')
        print(f'    (- = nose-up   / camera tilts upward)')
        print()
        print(f'  Camera mounting roll:  {roll_rad:+.4f} rad  '
              f'({roll_deg:+.2f}°)')
        print(f'    (+ = right-wing-down)')
        print(f'    (- = left-wing-down)')
        print()
        print('  Copy these into launch/vslam_cuvslam.launch.py AND launch/vslam_rtabmap.launch.py:')
        print(f'    camera_pitch = {pitch_rad:.6f}')
        print(f'    camera_roll  = {roll_rad:.6f}')
        print()
        print('=' * 60)
        print()
        print('IMPORTANT: Verify by slightly tilting the drone and re-running.')
        print('  - Tilt the drone nose-down → pitch should be MORE positive')
        print('  - Tilt the drone right   → roll  should be MORE positive')
        print()

        self.get_logger().info('Calibration complete. Press Ctrl+C to exit.')
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description='Calibrate D435i camera extrinsic (pitch/roll) '
                    'from IMU accelerometer on a level surface.'
    )
    parser.add_argument(
        '--samples', type=int, default=200,
        help='Number of accelerometer samples to collect (default: 200)'
    )
    args = parser.parse_args()

    rclpy.init()
    node = CameraExtrinsicCalibrator(num_samples=args.samples)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
