"""No ROS contexts/network: actual runtime methods and original task supervisor."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import yaml
from test_task_readonly import InertNode
from test_task_handoff import TaskRig
from test_aux_switch_decoder import params
from flight_runtime import create_node
from flight_supervisor import FlightOutput
from task_flight_controller import TaskFlightCore
from task_perception_pipe import ContainerTaskInputs
from live_flight_runtime import pump_task_inputs

ROOT=Path(__file__).resolve().parents[1]


class IsolatedNode(InertNode):
    def __init__(self,*a,**kw):super().__init__(*a,domain=182,**kw)
    def create_publisher(self,*a,**kw):return NS(publish=Mock())


class TransportFailureTests(unittest.TestCase):
    def make(self):
        cfg=yaml.safe_load((ROOT/'config/roundtrip_task.yaml').read_text())
        core=TaskFlightCore(9.,params(),cfg)
        with patch('rclpy.node.Node',IsolatedNode),patch('socket.socket',side_effect=AssertionError):
            runtime=create_node(core,isolated=True,clock=lambda:10.)
        return core,runtime

    def test_preparation_failure_does_not_skip_executive_or_replay(self):
        core,runtime=self.make()
        runtime.audit=Mock()
        runtime.task_input.prepare=Mock(side_effect=RuntimeError('goal pipe lost'))
        runtime.candidate_flight_guard=None
        with patch.object(core,'tick',return_value=FlightOutput('ABORT')) as tick:
            runtime.tick();runtime.tick()
        self.assertEqual(tick.call_count,2)
        self.assertEqual(runtime.task_input.prepare.call_count,1)
        self.assertIn('goal pipe lost',runtime.task_transport_fault)
        self.assertTrue(core.ev_stopped)

    def test_pipe_eof_is_latched_and_never_pumped_again(self):
        core,runtime=self.make()
        transport=NS(pump=Mock(side_effect=EOFError('pipe closed')),quarantine=Mock())
        runtime.task_input.command=('old-goal',(.15,0.))
        pump_task_inputs(transport,runtime);pump_task_inputs(transport,runtime)
        self.assertEqual(transport.pump.call_count,1)
        self.assertIsNone(runtime.task_input.command)
        self.assertIn('pipe closed',core.stop_reason)
        self.assertFalse(core.closed)  # independent PX4 callbacks still permitted
        self.assertTrue(core.ev_stopped)

    def test_original_abort_requests_land_or_yields_to_operator(self):
        for changes,expected in (({},'ABORT'),({'manual_takeover':True},'HANDOVER'),
                                  ({'kill_active':True},'KILLED')):
            with self.subTest(expected=expected):
                core,runtime=self.make();rig=TaskRig();rig.cruise()
                core.controller=rig.c
                runtime.task_transport_failed('input loss')
                out,messages=rig.tick(sample_changes=changes)
                self.assertEqual(out.state,expected)
                self.assertFalse(out.stream)
                self.assertNotIn('trajectory_setpoint',messages)
                if expected=='ABORT':self.assertEqual(out.command,'LAND')
                else:self.assertFalse(messages)

    def test_quarantine_discards_pending_writer_without_wait_or_recovery(self):
        with tempfile.TemporaryDirectory() as tmp:
            transport=ContainerTaskInputs(Path(tmp),20.,isolated=True,navigation_writes=True)
            process=Mock();transport.process=process
            writer=Mock();transport.command_writer=writer
            transport.proxy.command_enqueue=writer.enqueue
            transport.quarantine('EOF');transport.quarantine('later')
            self.assertIsNone(transport.command_writer)
            self.assertIsNone(transport.proxy.command_enqueue)
            self.assertEqual(transport.quarantine_reason,'EOF')
            self.assertTrue(json.loads(transport.control.read_text())['stop'])
            process.wait.assert_not_called();writer.pump.assert_not_called()
            with self.assertRaisesRegex(RuntimeError,'no replay'):transport.pump()

    def test_stop_file_error_does_not_abort_px4_fault_handling(self):
        with tempfile.TemporaryDirectory() as tmp:
            transport=ContainerTaskInputs(Path(tmp),20.,isolated=True)
            transport.process=Mock()
            with patch('disarmed_ev_lifecycle.request_stop',side_effect=OSError('disk full')):
                transport.quarantine('EOF')
            self.assertEqual(transport.stop_request_error,'disk full')
            self.assertIsNone(transport.proxy.command_enqueue)


if __name__=='__main__':unittest.main()
