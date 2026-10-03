"""Pure geometry/provenance and mocked ROS source factory; no hardware access."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from resident_real_source import (validate_installation,tracking_payload,create_sensor_node,
                                 ODOM,RAW_STATUS,LOGICAL_STATUS)


def transform(w=1.):
    return NS(translation=NS(x=.196,y=.025,z=-.05),rotation=NS(x=0.,y=0.,z=0.,w=w))


class SourceTests(unittest.TestCase):
    def test_approved_transform_and_equivalent_quaternion(self):
        validate_installation(transform());validate_installation(transform(-1.))

    def test_wrong_offset_rotation_nan_rejected(self):
        for field,value in (('x',.18),('y',0.),('z',float('nan'))):
            tr=transform();setattr(tr.translation,field,value)
            with self.assertRaises(ValueError):validate_installation(tr)
        tr=transform();tr.rotation.x=.01
        with self.assertRaises(ValueError):validate_installation(tr)

    def test_raw_tracking_stamp_and_unhealthy_state_preserved(self):
        msg=NS(header=NS(stamp=NS(sec=123,nanosec=456)),vo_state=0)
        self.assertEqual(json.loads(tracking_payload(msg)),dict(stamp_ns=123000000456,vo_state=0))
        msg.header.stamp.nanosec=10**9
        with self.assertRaises(ValueError):tracking_payload(msg)

    def test_factory_provenance_no_ros_publication_and_tf_loss(self):
        state=NS(subs={},queries=[],tr=transform(),missing=False)
        class Node:
            def __init__(self,*args,**kwargs):self.context=NS(get_domain_id=lambda:176)
            def create_subscription(self,kind,topic,callback,qos):state.subs[topic]=callback
            def create_timer(self,*args):pass
            def get_publishers_info_by_topic(self,topic):
                state.queries.append(topic)
                return [NS(endpoint_gid=bytes([8 if topic==RAW_STATUS else 7]*24))]
            def create_publisher(self,*args):raise AssertionError('sensor adapter cannot publish ROS')
        class Buffer:
            def lookup_transform(self,parent,child,when):
                if state.missing:raise RuntimeError('TF absent')
                assert (parent,child)==('base_link','camera_link')
                return NS(transform=state.tr)
        sender=NS(check=NS(context=NS(get_domain_id=lambda:176),port=NS(isolated=False)),
                  audit=Mock(),receive=Mock(),stop=Mock())
        modules={'rclpy.node':NS(Node=Node),'rclpy.time':NS(Time=lambda:None),
            'rclpy.qos':NS(qos_profile_sensor_data=object()),'nav_msgs.msg':NS(Odometry=object),
            'std_msgs.msg':NS(String=NS),
            'isaac_ros_visual_slam_interfaces.msg':NS(VisualSlamStatus=object),
            'tf2_ros':NS(Buffer=Buffer,TransformListener=lambda *a:None)}
        with patch.dict(sys.modules,modules):
            node=create_sensor_node(sender)
            pose=object();node.pose(pose);sender.receive.assert_not_called()
            node.audit();self.assertEqual(set(state.subs),{ODOM,RAW_STATUS})
            graph=sender.audit.call_args[0][0]
            self.assertEqual(graph[LOGICAL_STATUS],[[8]*24])
            self.assertNotIn(LOGICAL_STATUS,state.queries)
            node.pose(pose);sender.receive.assert_called_with(ODOM,pose)
            node.tracking(NS(header=NS(stamp=NS(sec=1,nanosec=2)),vo_state=1))
            topic,mapped=sender.receive.call_args[0]
            self.assertEqual(topic,LOGICAL_STATUS)
            self.assertEqual(json.loads(mapped.data)['stamp_ns'],1000000002)
            state.missing=True
            with self.assertRaisesRegex(RuntimeError,'TF missing'):node.audit()
            self.assertFalse(node.verified);sender.stop.assert_called_once()

    def test_isolated_sender_rejected_before_ros_import(self):
        sender=NS(check=NS(context=NS(get_domain_id=lambda:183),port=NS(isolated=True)))
        with self.assertRaises(ValueError):create_sensor_node(sender)


if __name__=='__main__':unittest.main()
