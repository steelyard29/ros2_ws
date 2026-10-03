from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from resident_ev_service import ResidentServiceLoop


class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.events=[]
        self.node=NS(closed=False,core=NS(fault=None),output='/fmu/in/vehicle_visual_odometry')
        def close():self.events.append('EV closed');self.node.closed=True
        self.node.close_output=close
        self.inputs=NS(pump=lambda:self.events.append('input'),stop=lambda why:self.events.append('reader stop'))
        self.loop=ResidentServiceLoop(self.node,self.inputs,lambda:self.events.append('PX4'))

    def test_normal_iteration_and_ordered_stop(self):
        self.loop.step();self.assertEqual(self.events,['PX4','input'])
        self.loop.stop('operator');self.assertEqual(self.events[-2:],['EV closed','reader stop'])
        with self.assertRaises(RuntimeError):self.loop.step()
        self.loop.stop('again');self.assertEqual(self.events.count('EV closed'),1)

    def test_input_failure_inhibits_before_cleanup(self):
        def fail():raise RuntimeError('pipe fault')
        self.inputs.pump=fail
        with self.assertRaisesRegex(RuntimeError,'pipe fault'):self.loop.step()
        self.assertEqual(self.events,['PX4','EV closed','reader stop'])

    def test_callback_fault_prevents_input_pump(self):
        self.node.core.fault='telemetry stale'
        with self.assertRaises(RuntimeError):self.loop.step()
        self.assertNotIn('input',self.events)

    def test_status_does_not_promote_dispatch_to_fusion_or_flight(self):
        self.node.core.health=lambda now:dict(recent_dispatch=True,sent_count=100)
        self.node.get_publishers_info_by_topic=lambda topic:[NS(endpoint_gid=bytes([1]*24))]
        status=self.loop.status()
        self.assertTrue(status['recent_dispatch'])
        self.assertFalse(status['fusion_verified']);self.assertFalse(status['flight_ready'])
        self.assertEqual(status['ev_publisher_gids'],[[1]*24])


if __name__=='__main__':unittest.main()
