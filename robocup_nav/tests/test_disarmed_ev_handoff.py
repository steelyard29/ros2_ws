"""Exercise the actual Observer factory with a node-local fake DDS graph.

No ROS context, sockets, subprocesses, or physical outputs. This is a positive
control for node lifetime, not an emulation/proof of Fast DDS network recovery.
"""
import sys
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from disarmed_ev_session import create_node, wait_px4_graph


class HandoffTests(unittest.TestCase):
    def setUp(self):
        stack=ExitStack()
        self.addCleanup(stack.close)
        self.state = s = NS(now=0., delay=3., competing=False, nodes=[])

        class FakeNode:
            def __init__(self, *a, **kw):
                self.born=s.now; self.closed=False
                self.pubs=[]; self.subs=[]; s.nodes.append(self)
            def count_publishers(self, topic):
                if topic.startswith('/fmu/in/'):
                    return int(s.competing) + int(topic in self.pubs)
                return int(s.now-self.born >= s.delay)
            def get_topic_names_and_types(self):
                return [('/fmu/in/vehicle_visual_odometry', [])]
            def create_publisher(self, cls, topic, qos):
                self.pubs.append(topic); return topic
            def create_subscription(self, cls, topic, cb, qos):
                self.subs.append((topic, cb)); return topic
            def destroy_publisher(self, pub):self.pubs.remove(pub)
            def destroy_node(self):self.closed=True

        def spin(node, timeout_sec):s.now=round(s.now+timeout_sec, 8)
        ros=NS(create_node=FakeNode, spin_once=spin)
        msgs=NS(**{name:type(name, (), {}) for name in (
            'VehicleOdometry', 'VehicleStatus', 'VehicleLocalPosition',
            'EstimatorStatusFlags', 'VehicleLandDetected', 'ManualControlSetpoint')})
        stack.enter_context(patch.dict(sys.modules, {
            'rclpy':ros, 'rclpy.node':NS(Node=FakeNode),
            'rclpy.qos':NS(qos_profile_sensor_data=object()), 'px4_msgs.msg':msgs}))
        stack.enter_context(patch('socket.socket', side_effect=AssertionError('offline only')))

    def probe(self, node, deadline=10.):
        return wait_px4_graph(deadline, probe=node, clock=lambda:self.state.now)

    def test_retained_observer_keeps_discovery_without_preoutput_endpoints(self):
        n=create_node(None, defer_output=True)
        self.assertEqual(n.pubs, []); self.assertEqual(n.subs, [])
        r=self.probe(n)
        self.assertEqual(r['state'], 'ready'); self.assertFalse(n.closed)
        self.assertEqual(n.pubs, []); self.assertEqual(n.subs, [])
        self.assertFalse(r['publisher_created'])
        n.activate(NS())
        self.assertEqual(len(self.state.nodes), 1)
        self.assertEqual(len(n.pubs), 1); self.assertEqual(len(n.subs), 5)
        self.assertEqual(n.count_publishers('/fmu/out/vehicle_status_v1'), 1)
        n.close_output(); n.destroy_node()
        self.assertEqual(n.pubs, []); self.assertTrue(n.closed)

    def test_old_recreate_sequence_reproduces_lost_discovery(self):
        old=create_node(None, defer_output=True)
        self.assertEqual(self.probe(old)['state'], 'ready')
        old.destroy_node()
        new=create_node(None, defer_output=True)
        self.state.now += 2.1
        self.assertEqual(new.count_publishers('/fmu/out/vehicle_status_v1'), 0)
        self.assertEqual(old.count_publishers('/fmu/out/vehicle_status_v1'), 1)

    def test_timeout_retains_node_for_owner_cleanup_and_creates_no_endpoints(self):
        n=create_node(None, defer_output=True)
        self.assertEqual(self.probe(n, 2.)['state'], 'timeout')
        self.assertEqual(self.state.now, 2.)
        self.assertEqual(n.pubs, []); self.assertEqual(n.subs, [])
        self.assertFalse(n.closed)
        n.close_output(); n.destroy_node()
        self.assertTrue(n.closed)

    def test_conflict_never_activates_output(self):
        self.state.competing=True
        n=create_node(None, defer_output=True)
        self.assertEqual(self.probe(n)['state'], 'conflict')
        self.assertEqual(n.pubs, []); self.assertEqual(n.subs, [])

    def test_activation_single_use_even_after_output_closed(self):
        n=create_node(None, defer_output=True)
        with self.assertRaises(RuntimeError):n.activate(None)
        n.activate(NS()); n.close_output()
        with self.assertRaises(RuntimeError):n.activate(NS())
        self.assertEqual(n.pubs, []); self.assertEqual(len(n.subs), 5)

    def test_existing_immediate_factory_contract_preserved(self):
        n=create_node(NS())
        self.assertEqual(len(n.pubs), 1); self.assertEqual(len(n.subs), 5)

    def test_dispatch_uses_integer_nanoseconds_and_records_actual_pair(self):
        session=NS(packet=lambda *a:object(),fault=None,watchdog=lambda now:False)
        n=create_node(session)
        n.audit=lambda:None
        n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=1_000_000_000))
        n.paired=[dict(stamp=1_000_000,at=100.,xy_valid=True,z_valid=True,
                       armed=False,landed=True,reset_all=[1,2,3,4,5])]
        packet=dict(kind='pose',stamp_ns=1_000_000_000,
                    position=[0,0,0],quaternion=[0,0,0,1])
        with patch('disarmed_ev_session.time.monotonic',return_value=100.):
            n.dispatch(packet)
        self.assertEqual(n.pair_attempts,1)
        self.assertEqual(len(n.pose_pairs),1)
        self.assertEqual(n.pose_pairs[0]['difference_ms'],0.)
        self.assertEqual(packet['host_clock']['ros_ns'],1_000_000_000)
        self.assertEqual(n.output_count,0)


if __name__ == '__main__':unittest.main()
