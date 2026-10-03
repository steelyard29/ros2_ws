#!/usr/bin/env python3
"""
PX4 IMU → camera IMU relay.

Reads /fmu/out/sensor_combined (PX4 calibrated IMU, flight-controller grade)
and republishes as sensor_msgs/Imu on /camera/camera/imu_FC,
replacing the D435i internal consumer-grade IMU.

Coordinate transform: PX4 FRD → camera optical frame
  PX4 FRD:      x=fwd  y=right  z=down
  Camera opt:   z=fwd  x=right  y=down
  → cam.x=px4.y, cam.y=px4.z, cam.z=px4.x

The topic is /camera/camera/imu_FC (not /camera/camera/imu) so the original
D435i IMU remains available for comparison.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from px4_msgs.msg import SensorCombined

# Frame name used by Isaac VSLAM / stereo_odometry IMU input
IMU_FRAME = "camera_imu_optical_frame"

class Px4ImuRelay(Node):
    def __init__(self):
        super().__init__('px4_imu_relay')

        self._pub = self.create_publisher(
            Imu, '/camera/camera/imu_FC',
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=30))

        self._sub = self.create_subscription(
            SensorCombined, '/fmu/out/sensor_combined',
            self._cb, QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                  durability=DurabilityPolicy.VOLATILE,
                                  history=HistoryPolicy.KEEP_LAST, depth=5))

        self._count = 0
        self.get_logger().info('PX4 IMU relay: /fmu/out/sensor_combined → /camera/camera/imu_FC')

    def _cb(self, msg: SensorCombined):
        # PX4 FRD → camera optical frame transform
        gx = float(msg.gyro_rad[1])   # cam.x = px4.y (right)
        gy = float(msg.gyro_rad[2])   # cam.y = px4.z (down)
        gz = float(msg.gyro_rad[0])   # cam.z = px4.x (fwd)
        ax = float(msg.accelerometer_m_s2[1])
        ay = float(msg.accelerometer_m_s2[2])
        az = float(msg.accelerometer_m_s2[0])

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = IMU_FRAME
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        # PX4 IMU is well-calibrated; use tight covariance
        imu.angular_velocity_covariance[0] = 0.0001
        imu.angular_velocity_covariance[4] = 0.0001
        imu.angular_velocity_covariance[8] = 0.0001
        imu.linear_acceleration_covariance[0] = 0.01
        imu.linear_acceleration_covariance[4] = 0.01
        imu.linear_acceleration_covariance[8] = 0.01

        self._pub.publish(imu)
        self._count += 1
        if self._count % 100 == 0:
            self.get_logger().info(
                f'{self._count} msgs | '
                f'gyro=({gx:.4f},{gy:.4f},{gz:.4f}) '
                f'accel=({ax:.2f},{ay:.2f},{az:.2f})',
                throttle_duration_sec=5.0)

def main():
    rclpy.init()
    node = Px4ImuRelay()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
