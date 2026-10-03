#!/usr/bin/env python3
"""Transform existing 360-degree scan to APF body FLU, no driver or PX4 output.

Only accepts planar, XY-coincident lidar mounts (operator: lidar x/y=0).
Uses actual TF including upside-down mounting; no guessed heading offset.
Unknown/missing rays stay NaN. Source timestamps are never refreshed.
"""
import copy
import math
import numpy as np
from nav_math import rotation


def transform_scan(m,tf):
    n=len(m.ranges)
    if (n<90 or not all(math.isfinite(v) for v in
            (m.angle_min,m.angle_max,m.angle_increment,m.range_min,m.range_max))
            or m.angle_increment<=0 or m.angle_max-m.angle_min<6.1
            or abs(m.angle_max-m.angle_min-(n-1)*m.angle_increment)>.02
            or m.range_min<0 or m.range_max<=m.range_min):raise ValueError('invalid full-circle scan')
    t,q=tf.translation,tf.rotation
    if not all(math.isfinite(v) for v in (t.x,t.y,t.z)) or math.hypot(t.x,t.y)>.001:
        raise ValueError('APF scan requires reviewed coincident XY sensor origin')
    quat=(q.x,q.y,q.z,q.w)
    if abs(np.linalg.norm(quat)-1)>.01:raise ValueError('invalid TF quaternion')
    r=rotation(quat)
    if math.hypot(r[2,0],r[2,1])>1e-3:raise ValueError('tilted lidar not planar')
    out=copy.deepcopy(m);out.header.frame_id='base_link'
    out.angle_min=-math.pi;out.angle_increment=2*math.pi/n
    out.angle_max=out.angle_min+(n-1)*out.angle_increment
    values=[math.nan]*n
    for i,d in enumerate(m.ranges):
        if not math.isfinite(d) or not m.range_min<=d<=m.range_max:continue
        theta=m.angle_min+i*m.angle_increment
        v=r@np.array([math.cos(theta),math.sin(theta),0.])
        a=math.atan2(v[1],v[0]);j=int(round((a+math.pi)/out.angle_increment))%n
        values[j]=min(values[j],d) if math.isfinite(values[j]) else float(d)
    out.ranges=values;out.intensities=[]
    return out


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data as qos
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer,TransformListener,TransformException
    rclpy.init()
    class Bridge(Node):
        def __init__(self):
            super().__init__('robocup_task_scan_bridge')
            self.tf=Buffer();self.listener=TransformListener(self.tf,self)
            self.pub=self.create_publisher(LaserScan,'/robocup/legacy_apf/scan',qos)
            self.create_subscription(LaserScan,'/scan',self.scan,qos)
        def scan(self,m):
            try:
                stamp=Time.from_msg(m.header.stamp)
                if not 0<=self.get_clock().now().nanoseconds-stamp.nanoseconds<=250_000_000:return
                if self.count_publishers('/scan')!=1:return
                tf=self.tf.lookup_transform('base_link',m.header.frame_id,stamp).transform
                self.pub.publish(transform_scan(m,tf))
            except (ValueError,TransformException):return
    node=Bridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
