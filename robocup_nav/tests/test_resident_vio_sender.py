"""Producer -> OS pipe -> receiver, no ROS/hardware or real CDR claims."""
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from resident_vio_sender import ResidentVioSender,create_node
from resident_vio_transport import ResidentVioPort,ResidentVioPipe


class SenderTests(unittest.TestCase):
    def setUp(self):
        self.now=100.;self.received=[]
        read,write=os.pipe();self.addCleanup(os.close,read);self.addCleanup(os.close,write)
        self.sender=ResidentVioSender(write,'s',domain=183,isolated=True,
            clock=lambda:self.now,encode=lambda msg:msg)
        self.receiver=ResidentVioPort('s',domain=183,isolated=True,
            clock=lambda:self.now,decode=lambda raw,kind:raw)
        self.pipe=ResidentVioPipe(read,self.receiver)
        self.graph={t:[[4]*24] for t in self.sender.check.port.reads}
        self.topic=sorted(self.graph)[0]
        for t in self.graph:self.receiver.port.create_subscription(bytes,t,self.received.append,None)

    def test_sender_receiver_preserves_payload_and_order_over_long_logical_session(self):
        for i in range(601):
            self.sender.audit(self.graph)
            self.sender.receive(self.topic,str(i).encode())
            self.sender.pump();self.pipe.pump();self.now+=.2
        self.assertEqual(self.received,[str(i).encode() for i in range(601)])
        self.assertEqual(self.sender.writer.stats['pending_bytes'],0)
        self.assertEqual(self.receiver.seq,1202)

    def test_source_replacement_discards_queued_payload(self):
        self.sender.audit(self.graph);self.sender.receive(self.topic,b'old')
        self.graph[self.topic]=[[5]*24]
        with self.assertRaises(RuntimeError):self.sender.audit(self.graph)
        self.assertFalse(self.sender.writer.queue)
        with self.assertRaises(RuntimeError):self.sender.pump()
        self.pipe.pump();self.assertEqual(self.received,[])

    def test_slow_consumer_is_fault_not_retimestamp_or_replay(self):
        self.sender.audit(self.graph);self.sender.receive(self.topic,b'old')
        self.now+=.501
        with self.assertRaisesRegex(TimeoutError,'queue stale'):self.sender.pump()
        self.assertFalse(self.sender.writer.queue)

    def test_oversized_payload_latches(self):
        self.sender.audit(self.graph)
        with self.assertRaises(BufferError):self.sender.receive(self.topic,b'x'*32768)
        self.assertIsNotNone(self.sender.fault)
        self.assertEqual(self.sender.writer.stats['pending_bytes'],0)

    def test_initial_discovery_does_not_forward_data_and_times_out(self):
        empty={t:[] for t in self.graph}
        self.sender.audit(empty);self.sender.receive(self.topic,b'no source')
        self.sender.pump();self.pipe.pump();self.assertEqual(self.received,[])
        self.now+=10
        with self.assertRaises(TimeoutError):self.sender.audit(empty)

    def test_real_factory_disabled_before_ros_import(self):
        with self.assertRaisesRegex(ValueError,'lifecycle not integrated'):create_node(self.sender)

    def test_actual_factory_only_subscribes_and_preserves_data(self):
        state=NS(domain=183,subscriptions={},timers=[])
        class Node:
            def __init__(self,*args,**kwargs):
                self.context=NS(get_domain_id=lambda:state.domain)
            def create_subscription(self,kind,topic,callback,qos):
                state.subscriptions[topic]=callback
            def create_timer(self,period,callback):state.timers.append((period,callback))
            def get_publishers_info_by_topic(self,topic):return [NS(endpoint_gid=bytes([4]*24))]
            def destroy_node(self):pass
            def create_publisher(self,*args,**kwargs):raise AssertionError('reader must not publish ROS')
        with patch.dict(sys.modules,{'rclpy.node':NS(Node=Node),
                'rclpy.qos':NS(qos_profile_sensor_data=object()),
                'nav_msgs.msg':NS(Odometry=bytes),'std_msgs.msg':NS(String=bytes)}):
            node=create_node(self.sender,isolated=True)
            self.assertEqual(set(state.subscriptions),set(self.graph))
            node.audit()
            state.subscriptions[self.topic](b'untouched source bytes')
            self.sender.pump();self.pipe.pump()
            self.assertEqual(self.received,[b'untouched source bytes'])
            state.domain=0
            with self.assertRaisesRegex(ValueError,'domain mismatch'):
                create_node(self.sender,isolated=True)


if __name__=='__main__':unittest.main()
