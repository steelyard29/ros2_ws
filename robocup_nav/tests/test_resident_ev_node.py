"""Real factories and callbacks, in-memory ROS endpoints only; no DDS/hardware."""
from contextlib import ExitStack
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from resident_ev_node import create_node
from flight_runtime import create_node as task_node
from flight_runtime_core import FlightRuntimeCore
from external_ev_evidence import ExternalEvEvidence


class NodeTests(unittest.TestCase):
    def setUp(self):
        stack=ExitStack();self.addCleanup(stack.close)
        s=self.s=NS(now=0.,domain=182,pubs=[],subs={},fail_send=False)
        class Node:
            def __init__(self,*a,**k):
                self.context=NS(get_domain_id=lambda:s.domain)
            def get_clock(self):return NS(now=lambda:NS(nanoseconds=int((100+s.now)*1e9)))
            def create_publisher(self,cls,topic,qos):
                def publish(msg):
                    if s.fail_send:raise RuntimeError('synthetic write failure')
                    for callback in s.subs.get(topic,[]):callback(msg)
                pub=NS(topic=topic,publish=publish);s.pubs.append(pub);return pub
            def destroy_publisher(self,pub):s.pubs.remove(pub)
            def create_subscription(self,cls,topic,callback,qos):
                s.subs.setdefault(topic,[]).append(callback);return NS()
            def create_timer(self,*a):return NS()
            def destroy_node(self):pass
            def get_publishers_info_by_topic(self,topic):
                if '/input/' in topic:return [NS(endpoint_gid=(1,))]
                return [NS(endpoint_gid=(9,)) for p in s.pubs if p.topic==topic]
            def count_publishers(self,topic):return len(self.get_publishers_info_by_topic(topic))
        names=('VehicleStatus VehicleLocalPosition EstimatorStatusFlags VehicleLandDetected '
               'ManualControlSetpoint ManualControlSwitches VehicleCommandAck FailsafeFlags '
               'VehicleOdometry OffboardControlMode TrajectorySetpoint VehicleCommand').split()
        stack.enter_context(patch.dict(sys.modules,{'rclpy':NS(),'rclpy.node':NS(Node=Node),
            'rclpy.qos':NS(qos_profile_sensor_data=object()),
            'nav_msgs.msg':NS(Odometry=object),'std_msgs.msg':NS(String=object),
            'px4_msgs.msg':NS(**{k:type(k,(),{}) for k in names})}))

    def step(self,node,armed=False):
        s=self.s;s.now+=1/30;stamp=int((100+s.now)*1e9);us=stamp//1000
        node.audit()
        for key,m in (
            ('status',NS(timestamp=us,arming_state=2 if armed else 1)),
            ('land',NS(timestamp=us,landed=not armed)),
            ('flags',NS(timestamp=us,cs_baro_hgt=True,cs_rng_hgt=False,cs_ev_hgt=True,cs_ev_vel=False))):
            # Route only to this resident's registered telemetry callbacks.
            self.s.subs[node.topics[key]][0](m)
        node.tracking(NS(data=json.dumps(dict(stamp_ns=stamp,vo_state=1))))
        node.pose(NS(header=NS(stamp=NS(sec=stamp//10**9,nanosec=stamp%10**9),frame_id='odom'),
            child_frame_id='base_link',pose=NS(pose=NS(position=NS(x=0.,y=0.,z=0.),
            orientation=NS(x=0.,y=0.,z=0.,w=1.)))))
        node.watchdog()

    def test_resident_to_actual_task_callback_single_ev_writer(self):
        resident=create_node('session',isolated=True,clock=lambda:self.s.now)
        core=FlightRuntimeCore(0.)
        core.bind_external_ev(ExternalEvEvidence('session',(9,)))
        task=task_node(core,isolated=True,clock=lambda:self.s.now)
        for _ in range(45):self.step(resident)
        self.assertGreater(resident.core.sent_count,0)
        self.assertEqual(core.external_ev.count,resident.core.sent_count)
        self.assertTrue(core.ev_receipt_recent(self.s.now))
        self.assertIsNone(resident.core.fault)
        self.assertEqual(sum(p.topic.endswith('/vehicle_visual_odometry') for p in self.s.pubs),1)
        self.assertNotIn('vehicle_visual_odometry',task.pubs)
        resident.close_output();self.s.now+=.31
        self.assertFalse(core.ev_receipt_recent(self.s.now))
        self.assertIsNotNone(core.fault)

    def test_publisher_continues_after_armed_and_landed(self):
        n=create_node('session',isolated=True,clock=lambda:self.s.now)
        for _ in range(45):self.step(n)
        before=n.core.sent_count
        for _ in range(15):self.step(n,True)
        for _ in range(15):self.step(n)
        self.assertGreater(n.core.sent_count,before);self.assertIsNone(n.core.fault)

    def test_send_exception_not_acknowledged(self):
        n=create_node('session',isolated=True,clock=lambda:self.s.now)
        self.s.fail_send=True
        with self.assertRaisesRegex(RuntimeError,'synthetic write'):
            for _ in range(45):self.step(n)
        self.assertEqual(n.core.sent_count,0);self.assertTrue(n.closed)

    def test_duplicate_ev_writer_stops(self):
        n=create_node('session',isolated=True,clock=lambda:self.s.now)
        n.create_publisher(object,n.output,None);n.audit()
        self.assertTrue(n.closed);self.assertIsNotNone(n.core.fault)

    def test_real_domain_or_mode_rejected_before_output(self):
        with self.assertRaises(ValueError):create_node('session')
        self.s.domain=0
        with self.assertRaises(ValueError):create_node('session',isolated=True)
        self.assertEqual(self.s.pubs,[])

    def split_port(self,domain=183):
        from resident_vio_input import ScopedVioInput
        port=NS(context=NS(get_domain_id=lambda:domain),subs={},queries=[],gid=(7,),broken=False)
        def subscribe(cls,topic,callback,qos):
            port.subs[topic]=callback
            return NS()
        def graph(topic):
            port.queries.append(topic)
            if port.broken:raise RuntimeError('pipe unavailable')
            return [NS(endpoint_gid=port.gid)]
        port.create_subscription=subscribe
        port.get_publishers_info_by_topic=graph
        return ScopedVioInput(port,isolated=True),port

    def test_split_subscriptions_and_source_audits_use_distinct_owners(self):
        scoped,port=self.split_port()
        n=create_node('split',isolated=True,clock=lambda:self.s.now,perception_node=scoped)
        self.assertEqual(set(port.subs),{n.topics['vio'],n.topics['tracking']})
        self.assertEqual(set(self.s.subs),{n.topics[k] for k in ('status','land','flags')})
        # Deliver vision exclusively through the external subscription callbacks.
        n.tracking=port.subs[n.topics['tracking']]
        n.pose=port.subs[n.topics['vio']]
        for _ in range(45):self.step(n)
        self.assertGreater(n.core.sent_count,0)
        self.assertEqual(n.core.sources['vio'],(7,))
        self.assertEqual(n.core.sources['status'],(1,))
        self.assertEqual(set(port.queries),set(port.subs))
        self.assertEqual(len(self.s.pubs),1)
        # Source replacement must not silently rebind even in the other domain.
        port.gid=(8,);n.audit()
        self.assertTrue(n.closed)
        self.assertIn('source generation changed',n.core.fault)

    def test_split_port_loss_closes_ev_and_cannot_auto_resume(self):
        scoped,port=self.split_port()
        n=create_node('split',isolated=True,clock=lambda:self.s.now,perception_node=scoped)
        for _ in range(45):self.step(n)
        sent=n.core.sent_count
        port.broken=True;n.audit()
        self.assertTrue(n.closed);self.assertEqual(self.s.pubs,[])
        port.broken=False
        for _ in range(15):self.step(n)
        self.assertEqual(n.core.sent_count,sent)

    def test_split_invalid_domains_rejected_before_endpoints(self):
        for domain in (0,176,182):
            scoped,_=self.split_port(domain)
            with self.assertRaisesRegex(ValueError,'distinct isolated domain'):
                create_node('split',isolated=True,perception_node=scoped)
            self.assertEqual(self.s.pubs,[]);self.assertEqual(self.s.subs,{})

    def test_unscoped_perception_rejected(self):
        with self.assertRaisesRegex(ValueError,'scoped perception'):
            create_node('split',isolated=True,perception_node=NS())
        self.assertEqual(self.s.pubs,[])

    def test_vio_port_has_no_writes_or_unrelated_reads(self):
        scoped,port=self.split_port()
        self.assertFalse(hasattr(scoped,'create_publisher'))
        for topic in ('/fmu/in/vehicle_visual_odometry','/robocup/task/navigation_goal',
                      '/robocup/runtime_test/input/vehicle_status_v1'):
            with self.assertRaises(ValueError):
                scoped.create_subscription(object,topic,lambda m:None,None)
            with self.assertRaises(ValueError):scoped.get_publishers_info_by_topic(topic)
        self.assertEqual(port.subs,{})

    def test_persistent_transport_dispatches_to_actual_resident_callbacks(self):
        import base64
        from resident_vio_transport import ResidentVioPort
        messages={}
        receiver=ResidentVioPort('transport',domain=183,isolated=True,
            clock=lambda:self.s.now,decode=lambda raw,kind:messages[raw])
        n=create_node('transport',isolated=True,clock=lambda:self.s.now,
                      perception_node=receiver.port)
        def packet(kind,**fields):
            return dict(session='transport',domain=183,seq=receiver.seq+1,
                        at=self.s.now,kind=kind,**fields)
        def deliver(key,m):
            raw=key.encode();messages[raw]=m
            receiver.accept(packet('data',topic=n.topics[key],gid=[7]*24,
                cdr=base64.b64encode(raw).decode()))
        # Registered callbacks are the original bound methods; test step routes
        # through the transport instead of directly invoking those callbacks.
        n.tracking=lambda m:deliver('tracking',m)
        n.pose=lambda m:deliver('vio',m)
        for _ in range(45):
            receiver.accept(packet('graph',graph={t:[[7]*24] for t in receiver.port.reads}))
            self.step(n)
        self.assertGreater(n.core.sent_count,0)
        self.assertEqual(receiver.received,90)
        self.assertIsNone(n.core.fault)
        self.assertEqual(len(self.s.pubs),1)
        self.s.now+=.501;n.audit()
        self.assertTrue(n.closed)
        self.assertIn('graph stale',receiver.fault)

    def test_bound_real_route_has_only_ev_output_and_owner_loss_closes_it(self):
        import tempfile
        from resident_ev_binding import ResidentEvOwner
        from resident_vio_transport import ResidentVioPort
        with tempfile.TemporaryDirectory() as tmp,patch('resident_ev_binding.LOCK_PATH',Path(tmp)/'lock'):
            owner=ResidentEvOwner('real');self.addCleanup(owner.close);owner.acquire()
            receiver=ResidentVioPort('real')
            self.s.domain=0
            node=create_node('real',owner=owner,perception_node=receiver.port,clock=lambda:self.s.now)
            self.assertEqual([p.topic for p in self.s.pubs],['/fmu/in/vehicle_visual_odometry'])
            self.assertEqual(set(self.s.subs),{'/fmu/out/vehicle_status_v1',
                '/fmu/out/vehicle_land_detected','/fmu/out/estimator_status_flags'})
            self.assertEqual(set(receiver.callbacks),receiver.port.reads)
            owner.close();node.audit()
            self.assertTrue(node.closed);self.assertEqual(self.s.pubs,[])


if __name__=='__main__':unittest.main()
