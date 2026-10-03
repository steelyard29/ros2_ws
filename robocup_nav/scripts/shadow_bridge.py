#!/usr/bin/env python3
"""Preview only: no /fmu/in publisher, mode switch, arming or actuator API.

Horizontal body velocity -> ENU world -> NED preview. PX4 local-frame alignment
is NOT established by this conversion. The topic must not be relayed to PX4.
"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from px4_msgs.msg import TrajectorySetpoint
from std_msgs.msg import String
from nav_math import yaw, preview_ned


class ShadowBridge(Node):
    def __init__(self):
        super().__init__('robocup_shadow_bridge')
        self.declare_parameter('preview_height_m', 1.2)
        self.height = self.get_parameter('preview_height_m').value
        self.odom = None
        self.command = None
        self.odom_at = self.command_at = 0.0
        self.pub = self.create_publisher(TrajectorySetpoint, '/robocup/preview/trajectory_setpoint', 10)
        self.status = self.create_publisher(String, '/robocup/preview/status', 10)
        self.create_subscription(Odometry, '/robocup/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, '/robocup/nav/cmd_vel_raw', self.on_command, 10)
        self.create_timer(0.05, self.tick)

    def on_odom(self, msg):
        age = (self.get_clock().now().nanoseconds - (msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_footprint' or not -0.05 <= age <= 0.3:
            self.odom = None
            return
        self.odom, self.odom_at = msg, time.monotonic()

    def on_command(self, msg):
        self.command, self.command_at = msg, time.monotonic()

    def tick(self):
        now = time.monotonic()
        valid = self.odom is not None and self.command is not None and now-self.odom_at < 0.3 and now-self.command_at < 0.3
        if valid:
            q = self.odom.pose.pose.orientation
            c = self.command
            try:
                if abs(c.linear.x) > 0.251 or abs(c.linear.y) > 0.001 or abs(c.angular.z) > 0.351:
                    raise ValueError('outside baseline limits')
                velocity, rate, z = preview_ned(c.linear.x, c.linear.y, c.angular.z,
                                               yaw([q.x, q.y, q.z, q.w]), self.height)
            except ValueError:
                valid = False
        if valid:
            msg = TrajectorySetpoint()
            msg.timestamp = self.get_clock().now().nanoseconds//1000
            msg.position = [math.nan, math.nan, z]
            msg.velocity = velocity
            msg.acceleration = [math.nan]*3
            msg.jerk = [math.nan]*3
            msg.yaw = math.nan
            msg.yawspeed = rate
            self.pub.publish(msg)
        # No stale heartbeat or fake hover command; this is NOT a flight failsafe.
        self.status.publish(String(data='PREVIEW_ONLY' if valid else 'INHIBITED_STALE_OR_INVALID'))


if __name__ == '__main__':
    rclpy.init()
    node = ShadowBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
