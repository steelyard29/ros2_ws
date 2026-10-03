"""Domain176 sensor-only adapter for the resident transport; no runnable CLI.

Consumes raw cuVSLAM status, not an independent status relay. Preserves its
source stamp/state in the existing String wire schema and assigns its raw
publisher GID to that logical input. This is a mapping, not a ROS relay publisher.
No PX4 interface. Worker/service real routing is not enabled by this factory.
"""
import json
import math
import time

ODOM='/visual_slam/tracking/odometry'
RAW_STATUS='/visual_slam/status'
LOGICAL_STATUS='/robocup/alignment/tracking'
EXPECTED_XYZ=(.196,.025,-.05)


def validate_installation(transform):
    p,q=transform.translation,transform.rotation
    xyz=(p.x,p.y,p.z);quat=(q.x,q.y,q.z,q.w)
    if not all(math.isfinite(v) for v in xyz+quat):raise ValueError('nonfinite installation TF')
    # Configuration equality, NOT a micrometre physical calibration claim.
    if any(abs(a-b)>1e-6 for a,b in zip(xyz,EXPECTED_XYZ)):
        raise ValueError('installation translation differs from approved D435i offset')
    if any(abs(v)>1e-6 for v in quat[:3]) or abs(abs(quat[3])-1)>1e-6:
        raise ValueError('installation rotation differs from zero mounting angle')


def tracking_payload(msg):
    sec,nsec=int(msg.header.stamp.sec),int(msg.header.stamp.nanosec)
    if sec<0 or not 0<=nsec<10**9 or sec*10**9+nsec<=0:
        raise ValueError('invalid raw tracking stamp')
    return json.dumps(dict(stamp_ns=sec*10**9+nsec,vo_state=int(msg.vo_state)))


def create_sensor_node(sender,*,clock=time.monotonic):
    if sender.check.context.get_domain_id()!=176 or sender.check.port.isolated:
        raise ValueError('real VIO adapter requires domain176 sender')
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data as qos
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
    from tf2_ros import Buffer,TransformListener

    class Reader(Node):
        def __init__(self):
            super().__init__('resident_vio_sensor_reader',enable_rosout=False,
                             start_parameter_services=False,use_global_arguments=False)
            if self.context.get_domain_id()!=176:
                self.destroy_node();raise ValueError('real VIO reader domain mismatch')
            self.started=clock();self.verified=False
            self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
            self.create_subscription(Odometry,ODOM,self.pose,qos)
            self.create_subscription(VisualSlamStatus,RAW_STATUS,self.tracking,qos)
            self.create_timer(.1,self.audit)

        def audit(self):
            try:
                try:tr=self.buffer.lookup_transform('base_link','camera_link',Time()).transform
                except Exception:
                    if self.verified or clock()-self.started>=10:
                        raise RuntimeError('resident installation TF missing')
                    return
                validate_installation(tr)
                graph={logical:[list(end.endpoint_gid) for end in self.get_publishers_info_by_topic(raw)]
                       for logical,raw in ((ODOM,ODOM),(LOGICAL_STATUS,RAW_STATUS))}
                sender.audit(graph)
                self.verified=True
            except Exception as exc:
                self.verified=False;sender.stop(exc);raise

        def pose(self,msg):
            if not self.verified:return
            sender.receive(ODOM,msg)

        def tracking(self,msg):
            if not self.verified:return
            try:
                mapped=String();mapped.data=tracking_payload(msg)
                sender.receive(LOGICAL_STATUS,mapped)
            except Exception as exc:sender.stop(exc);raise
    return Reader()
