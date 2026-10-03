#!/usr/bin/env python3
"""Project VIO odometry to yaw-only TF; guard nvblox shadow velocity output.

The upstream 3.2 nvblox Nav2 layer does not mark itself stale. Explicitly gate
on source depth, ESDF, odometry, tracker status and command freshness here.
No arming, Offboard heartbeats, flight or actuator publishers.
"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from nvblox_msgs.msg import DistanceMapSlice
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from nav_math import yaw, rotation
import numpy as np


class Guard(Node):
    def __init__(self):
        super().__init__('robocup_nvblox_guard')
        self.declare_parameter('center_z', 0.0)
        self.declare_parameter('relay_tracking',False)
        self.tracking_relay=self.tracking_pub=None
        if self.get_parameter('relay_tracking').value:
            from task_domain_io import TrackingStatusRelay
            self.tracking_relay=TrackingStatusRelay()
            self.tracking_pub=self.create_publisher(String,'/robocup/alignment/tracking',10)
        self.center = self.get_parameter('center_z').value
        self.seen = {}
        self.healthy = False
        self.in_band = False
        self.reset = False
        self.previous = None
        self.command = Twist()
        self.tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/robocup/odom', 10)
        self.vel_pub = self.create_publisher(Twist, '/robocup/nav/cmd_vel_raw', 10)
        self.status_pub = self.create_publisher(String, '/robocup/nvblox/readiness', 10)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.odom, qos_profile_sensor_data)
        self.create_subscription(VisualSlamStatus, '/visual_slam/status', self.status, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw',
                                 lambda m: self.stamp('depth', m.header.stamp), qos_profile_sensor_data)
        self.create_subscription(DistanceMapSlice, '/nvblox_node/static_map_slice', self.slice, 10)
        self.create_subscription(Twist, '/robocup/nvblox/cmd_vel_unchecked', self.cmd, 10)
        self.create_timer(0.05, self.tick)

    def stamp(self, key, stamp):
        age = (self.get_clock().now().nanoseconds - (stamp.sec*10**9+stamp.nanosec))/1e9
        if not -0.05 <= age <= 0.5:
            self.seen.pop(key, None)
            return False
        self.seen[key] = time.monotonic()
        return True

    def status(self, msg):
        self.healthy = int(msg.vo_state) == 1
        self.stamp('status', msg.header.stamp)
        if self.tracking_relay:
            payload=self.tracking_relay.encode(msg,self.get_publishers_info_by_topic('/visual_slam/status'))
            if payload is not None:self.tracking_pub.publish(String(data=payload))

    def slice(self, msg):
        self.seen.pop('slice', None)
        if msg.header.frame_id == 'odom' and msg.width*msg.height == len(msg.data) and len(msg.data) > 0:
            a = np.asarray(msg.data)
            if np.isfinite(a).all() and np.any(a != msg.unknown_value):
                self.stamp('slice', msg.header.stamp)

    def odom(self, msg):
        self.seen.pop('odom', None)
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link' or not self.stamp('odom', msg.header.stamp):
            return
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        try:
            heading = yaw([q.x, q.y, q.z, q.w])
            r = rotation([q.x, q.y, q.z, q.w])
        except ValueError:
            self.seen.pop('odom', None)
            return
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.seen.pop('odom', None)
            return
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        if not all(math.isfinite(x) for x in (v.x, v.y, v.z, w.x, w.y, w.z)):
            self.seen.pop('odom', None)
            return
        self.in_band = abs(p.z-self.center) < 0.08 and r[2, 2] > math.cos(math.radians(10))
        now = time.monotonic()
        if self.previous is not None:
            old, old_heading, t = self.previous
            delta_yaw = abs(math.atan2(math.sin(heading-old_heading), math.cos(heading-old_heading)))
            if now-t < 0.5 and (math.dist(old, (p.x, p.y, p.z)) > 0.5 or delta_yaw > 0.5):
                self.reset = True
        self.previous = ((p.x, p.y, p.z), heading, now)
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = 'base_footprint'
        out.pose.pose.position.x, out.pose.pose.position.y = p.x, p.y
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
        v = msg.twist.twist.linear
        world = r @ np.array([v.x, v.y, v.z])
        c, s = math.cos(heading), math.sin(heading)
        out.twist.twist.linear.x = float(c*world[0]+s*world[1])
        out.twist.twist.linear.y = float(-s*world[0]+c*world[1])
        w = msg.twist.twist.angular
        out.twist.twist.angular.z = float((r @ np.array([w.x, w.y, w.z]))[2])
        self.odom_pub.publish(out)
        tf = TransformStamped()
        tf.header, tf.child_frame_id = out.header, 'base_footprint'
        tf.transform.translation.x, tf.transform.translation.y = p.x, p.y
        tf.transform.rotation = out.pose.pose.orientation
        self.tf.sendTransform(tf)

    def cmd(self, msg):
        values = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
        if not all(math.isfinite(v) for v in values) or not (
            -0.001 <= msg.linear.x <= 0.251 and abs(msg.linear.y) < 0.001 and abs(msg.angular.z) <= 0.351
            and abs(msg.linear.z) < 0.001 and abs(msg.angular.x) < 0.001 and abs(msg.angular.y) < 0.001):
            self.seen.pop('command', None)
            return
        self.command = msg
        self.seen['command'] = time.monotonic()

    def tick(self):
        now = time.monotonic()
        missing = [k for k, limit in [('odom', .3), ('depth', .5), ('slice', .6), ('status', .3), ('command', .3)]
                   if now-self.seen.get(k, -1e20) > limit]
        ready = not missing and self.healthy and self.in_band and not self.reset
        self.vel_pub.publish(self.command if ready else Twist())
        label = 'SHADOW_READY' if ready else f'INHIBITED missing={missing} vo={self.healthy} band={self.in_band} reset={self.reset}'
        self.status_pub.publish(String(data=label))


if __name__ == '__main__':
    rclpy.init()
    node = Guard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
