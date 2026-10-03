#!/usr/bin/env python3
"""Yaw-only odometry plus depth points in the robot's occupied height slab.

Depth points retain their optical frame for correct ray origins in costmaps.
Missing depth does NOT create free space; stale observations invalidate layers.
"""
import math
from pathlib import Path
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from nav_math import rotation, yaw, depth_points

ROOT = Path(__file__).resolve().parents[1]


class GeometryBridge(Node):
    def __init__(self):
        super().__init__('robocup_geometry_bridge')
        self.declare_parameter('demo_geometry', False)
        platform = yaml.safe_load((ROOT/'config/platform.yaml').read_text())
        demo = self.get_parameter('demo_geometry').value
        if not demo and not platform['geometry_verified']:
            raise RuntimeError('Measured platform geometry required; demo_geometry is OFFLINE ONLY')
        if demo:
            self.above, self.below, self.radius = 0.25, 0.40, 0.50
        else:
            self.above = float(platform['above_center_m'])
            self.below = float(platform['below_center_with_payload_m'])
            self.radius = math.hypot(platform['length_m'], platform['width_m'])/2
            if not all(math.isfinite(x) and x > 0 for x in (self.above, self.below, self.radius)):
                raise RuntimeError('Invalid platform dimensions')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/robocup/odom', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/robocup/obstacles', 10)
        self.info = None
        self.last_depth = 0.0
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.on_depth, qos_profile_sensor_data)

    def fresh(self, stamp):
        age = (self.get_clock().now().nanoseconds - Time.from_msg(stamp).nanoseconds)/1e9
        return -0.05 <= age <= 0.3

    def on_info(self, msg):
        self.info = msg

    def on_odom(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link' or not self.fresh(msg.header.stamp):
            return
        q = msg.pose.pose.orientation
        try:
            r = rotation([q.x, q.y, q.z, q.w])
            heading = yaw([q.x, q.y, q.z, q.w])
        except ValueError:
            return
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = 'base_footprint'
        out.pose.pose.position.x = msg.pose.pose.position.x
        out.pose.pose.position.y = msg.pose.pose.position.y
        out.pose.pose.orientation.z = math.sin(heading/2)
        out.pose.pose.orientation.w = math.cos(heading/2)
        # nav_msgs/Odometry twist is in child frame. Rotate out roll/pitch.
        twist = msg.twist.twist
        linear = r @ np.array([twist.linear.x, twist.linear.y, twist.linear.z])
        c, s = math.cos(heading), math.sin(heading)
        out.twist.twist.linear.x = float(c*linear[0]+s*linear[1])
        out.twist.twist.linear.y = float(-s*linear[0]+c*linear[1])
        out.twist.twist.angular.z = float((r @ np.array([twist.angular.x, twist.angular.y, twist.angular.z]))[2])
        self.odom_pub.publish(out)
        tf = TransformStamped()
        tf.header = out.header
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = out.pose.pose.position.x
        tf.transform.translation.y = out.pose.pose.position.y
        tf.transform.rotation = out.pose.pose.orientation
        self.broadcaster.sendTransform(tf)

    def on_depth(self, msg):
        now = time.monotonic()
        info = self.info
        if now-self.last_depth < 0.2 or info is None or not self.fresh(msg.header.stamp):
            return
        if (msg.width, msg.height) != (info.width, info.height) or not self.fresh(info.header.stamp):
            return
        if info.header.frame_id != msg.header.frame_id:
            return
        self.last_depth = now
        dtype = {'16UC1': 'u2', '32FC1': 'f4'}.get(msg.encoding)
        if dtype is None:
            return
        try:
            dtype = np.dtype(('>' if msg.is_bigendian else '<')+dtype)
            arr = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                             strides=(msg.step, dtype.itemsize)).astype(np.float32)
            if msg.encoding == '16UC1':
                arr *= 0.001
            pts = depth_points(arr, info.k[0], info.k[4], info.k[2], info.k[5])
            stamp = Time.from_msg(msg.header.stamp)
            camera = self.buffer.lookup_transform('odom', msg.header.frame_id, stamp).transform
            body = self.buffer.lookup_transform('odom', 'base_link', stamp).transform
            q = camera.rotation
            r = rotation([q.x, q.y, q.z, q.w])
            world_z = (pts @ r.T)[:, 2]+camera.translation.z
            bq = body.rotation
            br = rotation([bq.x, bq.y, bq.z, bq.w])
            tilt_margin = self.radius*math.sqrt(max(0., 1.-br[2, 2]**2))+0.10
            valid = (world_z >= body.translation.z-self.below-tilt_margin) & (world_z <= body.translation.z+self.above+tilt_margin)
            # Preserve sensor origin, not base_footprint, for clearing rays.
            cloud = point_cloud2.create_cloud_xyz32(msg.header, pts[valid].tolist())
            self.cloud_pub.publish(cloud)
        except (ValueError, TransformException):
            return


if __name__ == '__main__':
    rclpy.init()
    node = GeometryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
