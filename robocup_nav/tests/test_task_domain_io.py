from types import SimpleNamespace as NS
from pathlib import Path
import json
import sys
import unittest
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from task_domain_io import ScopedPerceptionNode,TrackingStatusRelay,topic_sets,PerceptionDomain


class DomainIoTests(unittest.TestCase):
    def test_all_px4_publish_and_subscribe_routes_rejected(self):
        for isolated in (False,True):
            node=Mock();port=ScopedPerceptionNode(node,isolated=isolated)
            for topic in ('/fmu/in/vehicle_visual_odometry','/fmu/in/vehicle_command',
                          '/fmu/out/vehicle_status_v1','/cmd_vel','/unlisted'):
                with self.assertRaises(ValueError):port.create_publisher(object,topic,10)
                with self.assertRaises(ValueError):port.create_subscription(object,topic,lambda _:None,10)
            node.create_publisher.assert_not_called();node.create_subscription.assert_not_called()

    def test_no_sensor_republication_and_exact_navigation_routes(self):
        for isolated in (False,True):
            node=Mock();port=ScopedPerceptionNode(node,isolated=isolated)
            reads,writes=topic_sets(isolated)
            for topic in reads:
                with self.assertRaises(ValueError):port.create_publisher(object,topic,10)
                port.create_subscription(object,topic,lambda _:None,10)
            self.assertEqual(len(writes),2)
            for topic in writes:port.create_publisher(object,topic,10)
            self.assertEqual(node.create_publisher.call_count,2)
            self.assertNotIn('/robocup/task_test/input/vio',reads)

    def test_isolated_domains_cannot_be_real_domains(self):
        for domain in (0,176,177,232):
            with self.assertRaises(ValueError):PerceptionDomain(domain_id=domain,isolated=True)

    def test_tracking_stamp_and_failure_are_not_rewritten(self):
        relay=TrackingStatusRelay()
        msg=NS(vo_state=2,header=NS(stamp=NS(sec=123,nanosec=456)))
        pubs=[NS(endpoint_gid=[1,2])]
        raw=relay.encode(msg,pubs)
        self.assertEqual(json.loads(raw),dict(vo_state=2,stamp_ns=123000000456))
        # Duplicate input stays duplicate for the existing downstream guard.
        self.assertEqual(relay.encode(msg,pubs),raw)

    def test_tracking_ownership_fault_latches(self):
        for changed in ([NS(endpoint_gid=[2])],[NS(endpoint_gid=[1]),NS(endpoint_gid=[2])]):
            relay=TrackingStatusRelay();msg=NS(vo_state=1,header=NS(stamp=NS(sec=1,nanosec=0)))
            self.assertIsNone(relay.encode(msg,[]));self.assertIsNone(relay.fault)
            self.assertIsNotNone(relay.encode(msg,[NS(endpoint_gid=[1])]))
            self.assertIsNone(relay.encode(msg,changed));self.assertIsNotNone(relay.fault)
            self.assertIsNone(relay.encode(msg,[NS(endpoint_gid=[1])]))


if __name__=='__main__':unittest.main()
