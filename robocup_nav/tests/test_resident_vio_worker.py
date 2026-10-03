"""Exercise actual worker cleanup/receipt with mocked ROS; never starts nodes."""
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from resident_reader_lifecycle import ReaderLeaseOwner
from resident_vio_worker import run
from disarmed_ev_lifecycle import check_receipt


class WorkerTests(unittest.TestCase):
    def exercise(self,shutdown_error=False,sensor_only=False):
        with tempfile.TemporaryDirectory() as tmp:
            control=Path(tmp)/'control.json';receipt=Path(tmp)/'receipt.json'
            ReaderLeaseOwner(control,'s').renew()
            node=NS(destroy_node=Mock())
            sender=NS(stop=Mock(),pump=Mock(),writer=NS(stats={}))
            ros=NS(init=Mock(),spin_once=Mock(),try_shutdown=Mock(
                side_effect=RuntimeError('shutdown failed') if shutdown_error else None))
            factory=Mock(return_value=node)
            with patch.dict(os.environ,{},clear=False),patch.dict(sys.modules,{
                'rclpy':ros,'resident_real_source':NS(create_sensor_node=factory),
                'resident_vio_sender':NS(
                    ResidentVioSender=Mock(return_value=sender),create_node=factory)}),patch(
                'resident_vio_worker.service_loop',return_value='owner_requested_stop'):
                code=run('s',control,receipt,176 if sensor_only else 183,
                         None if sensor_only else time.monotonic()+20,sensor_only=sensor_only)
            if sensor_only:factory.assert_called_once_with(sender)
            node.destroy_node.assert_called_once();ros.try_shutdown.assert_called_once()
            sender.stop.assert_called_once_with('worker exit')
            data=json.loads(receipt.read_text())
            return code,data,check_receipt(receipt,'s',code,time.monotonic())

    def test_normal_cleanup_has_matching_close_receipt(self):
        code,data,checked=self.exercise()
        self.assertEqual(code,0);self.assertTrue(data['ros_closed'])
        self.assertTrue(checked['confirmed'])

    def test_shutdown_failure_not_reported_as_closed(self):
        code,data,checked=self.exercise(True)
        self.assertEqual(code,2);self.assertFalse(data['ros_closed'])
        self.assertFalse(checked['confirmed']);self.assertIn('cleanup_errors',data)

    def test_invalid_domain_rejected_without_receipt_or_ros(self):
        with tempfile.TemporaryDirectory() as tmp:
            receipt=Path(tmp)/'receipt.json'
            with self.assertRaises(ValueError):run('s',Path(tmp)/'control',receipt,0,time.monotonic()+20)
            self.assertFalse(receipt.exists())

    def test_sensor_only_routes_to_real_source_and_closes(self):
        code,data,checked=self.exercise(sensor_only=True)
        self.assertEqual(code,0);self.assertTrue(data['sensor_only'])
        self.assertEqual(data['domain'],176);self.assertTrue(checked['confirmed'])


if __name__=='__main__':unittest.main()
