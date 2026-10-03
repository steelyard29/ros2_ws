"""No ROS contexts or hardware: actual codec and typed message roundtrip."""
import base64
from pathlib import Path
import subprocess
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from task_perception_pipe import ReadonlyPipePort,depth_evidence,restore_depth,specifications
from task_domain_io import topic_sets
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rclpy.serialization import serialize_message
from unittest.mock import patch


class TaskPipeTests(unittest.TestCase):
    def make(self):
        port=ReadonlyPipePort('test',isolated=True,clock=lambda:10.)
        graph={t:[] for t in port.reads.keys()|port.writes}
        topic=next(t for t,k in port.reads.items() if k=='nav_msgs/msg/Odometry')
        graph[topic]=[dict(name='fixture',namespace='/',gid=[1]*24)]
        port.accept(dict(session='test',domain=183,seq=1,at=9.9,kind='graph',graph=graph))
        return port,topic,graph

    def test_allowlist_matches_existing_task_port(self):
        for isolated in (False,True):
            reads,writes,_=specifications(isolated)
            self.assertEqual((set(reads),writes),topic_sets(isolated))

    def test_typed_pose_preserves_source_and_twist(self):
        port,topic,_=self.make();seen=[]
        port.create_subscription(Odometry,topic,seen.append,None)
        msg=Odometry();msg.header.stamp.sec=123;msg.header.stamp.nanosec=456
        msg.header.frame_id='odom';msg.child_frame_id='base_link'
        msg.pose.pose.orientation.w=1.;msg.twist.twist.linear.x=.123
        packet=dict(session='test',domain=183,seq=2,at=9.95,kind='data',topic=topic,gid=[1]*24,
            cdr=base64.b64encode(serialize_message(msg)).decode())
        port.accept(packet)
        self.assertEqual(seen,[msg]);self.assertEqual(port.received[topic],1)
        with self.assertRaises(ValueError):port.accept(packet)
        rejection=dict(port.first_rejected)
        self.assertEqual(rejection['topic'],topic)
        self.assertEqual(rejection['seq'],2)
        with self.assertRaises(RuntimeError):port.accept(dict(packet,seq=3))
        self.assertEqual(port.first_rejected,rejection)

    def test_depth_metadata_not_fabricated_pixels(self):
        msg=Image();msg.width=640;msg.height=480;msg.header.stamp.sec=99
        msg.data=[0]*614400
        restored=restore_depth(depth_evidence(msg))
        self.assertEqual((restored.width,restored.height,len(restored.data)),(640,480,614400))
        self.assertEqual(restored.header.stamp.sec,99)
        self.assertIsInstance(restored.data,range)
        with self.assertRaises(ValueError):restore_depth(dict(depth_evidence(msg),data_length=-1))

    def test_stale_wrong_session_and_wrong_domain_rejected(self):
        for override in (dict(at=9.),dict(session='other'),dict(domain=176),dict(seq=9)):
            port,topic,graph=self.make()
            packet=dict(session='test',domain=183,seq=2,at=9.9,kind='graph',graph=graph)
            packet.update(override)
            with self.assertRaises(ValueError):port.accept(packet)

    def test_source_replacement_loss_and_goal_writer_rejected(self):
        for case in ('replace','lost','goal'):
            port,topic,graph=self.make()
            if case=='replace':graph[topic][0]['gid']=[2]*24
            elif case=='lost':graph[topic]=[]
            else:graph[next(iter(port.writes))]=[dict(name='other',namespace='/',gid=[2]*24)]
            with self.assertRaises(ValueError):port.accept(dict(session='test',domain=183,seq=2,
                at=9.95,kind='graph',graph=graph))

    def test_publish_and_foreign_subscribe_denied(self):
        port,_,_=self.make()
        with self.assertRaises(RuntimeError):port.create_publisher(None,'/fmu/in/vehicle_command',10)
        with self.assertRaises(ValueError):port.create_subscription(Odometry,'/foreign',lambda m:None,10)

    def test_startup_packets_not_delivered_or_replayed_into_active_task(self):
        port,topic,_=self.make();seen=[]
        port.discovery_only=True
        packet=dict(session='test',domain=183,seq=2,at=9.95,kind='data',topic=topic,gid=[1]*24,
                    cdr=base64.b64encode(serialize_message(Odometry())).decode())
        port.accept(packet)
        self.assertEqual(port.received,{})
        port.create_subscription(Odometry,topic,seen.append,None)
        port.discovery_only=False;port.activated_at=10.
        port.accept(dict(packet,seq=3))
        self.assertEqual(seen,[]);self.assertEqual(port.startup_discarded,2)
        port.accept(dict(packet,seq=4,at=10.))
        self.assertEqual(len(seen),1)

    def test_activation_requires_all_callbacks_and_fresh_graph(self):
        from task_perception_pipe import ContainerTaskInputs
        root=Path(__file__).resolve().parents[1]
        transport=ContainerTaskInputs(root/'evidence',20.,isolated=True)
        transport.proxy.discovery_only=True;transport.proxy.graph_at=9.9
        with patch('task_perception_pipe.time.monotonic',return_value=10.):
            with self.assertRaises(RuntimeError):transport.activate_inputs()
            transport.proxy.callbacks={t:[] for t in transport.proxy.reads}
            with self.assertRaises(RuntimeError):transport.activate_inputs()
            transport.proxy.callbacks={t:[(object,lambda m:None)] for t in transport.proxy.reads}
            transport.activate_inputs()
        self.assertFalse(transport.proxy.discovery_only)
        self.assertEqual(transport.proxy.activated_at,10.)

    def test_delayed_startup_discards_old_packets_but_active_age_still_rejected(self):
        port,topic,graph=self.make();port.discovery_only=True
        port.accept(dict(session='test',domain=183,seq=2,at=8.,kind='graph',graph=graph))
        port.accept(dict(session='test',domain=183,seq=3,at=8.1,kind='data',topic=topic,gid=[1]*24))
        self.assertEqual(port.startup_discarded,1)
        self.assertEqual(port.received,{})
        port.discovery_only=False
        with self.assertRaises(ValueError):
            port.accept(dict(session='test',domain=183,seq=4,at=8.2,kind='graph',graph=graph))

    def test_activation_refreshes_stale_graph_without_delivering_startup_data(self):
        from task_perception_pipe import ContainerTaskInputs
        transport=ContainerTaskInputs(Path(__file__).resolve().parents[1]/'evidence',20.,isolated=True)
        transport.proxy.discovery_only=True;transport.proxy.graph_at=8.
        transport.proxy.callbacks={t:[(object,lambda m:None)] for t in transport.proxy.reads}
        def refresh():
            self.assertTrue(transport.proxy.discovery_only)
            transport.proxy.graph_at=9.99
        with patch('task_perception_pipe.time.monotonic',return_value=10.),patch.object(transport,'pump',side_effect=refresh) as pump:
            transport.activate_inputs();pump.assert_called_once()
        self.assertEqual(transport.proxy.activated_at,10.)

    def test_integrated_fixture_default_does_not_run(self):
        script=Path(__file__).resolve().parents[1]/'tests/task_runtime_pipe_check.py'
        r=subprocess.run([sys.executable,str(script)],capture_output=True,text=True,timeout=3)
        self.assertEqual(r.returncode,0,r.stderr);self.assertIn('Describe only',r.stdout)

    def test_reader_cli_default_only_describes(self):
        script=Path(__file__).resolve().parents[1]/'scripts/task_perception_reader.py'
        result=subprocess.run([sys.executable,str(script)],capture_output=True,text=True,timeout=3)
        self.assertEqual(result.returncode,0,result.stderr);self.assertIn('Describe only',result.stdout)


if __name__=='__main__':unittest.main()
