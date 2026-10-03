"""Actual runtime factory/callback with fake ROS endpoints; no ROS graph."""
from contextlib import ExitStack
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch, PropertyMock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from external_ev_evidence import ExternalEvEvidence
from flight_runtime_core import FlightRuntimeCore
from flight_runtime import create_node


def message(**changes):
    d=dict(timestamp=1_000_000,timestamp_sample=1_000_000,quality=100,
        pose_frame=2,velocity_frame=2,position=[0.,0.,0.],q=[1.,0.,0.,0.],
        velocity=[float('nan')]*3,reset_counter=0)
    d.update(changes);return NS(**d)


class RoutingTests(unittest.TestCase):
    def setUp(self):
        stack=ExitStack();self.addCleanup(stack.close)
        class Node:
            def __init__(self,*a,**kw):self.created=[];self.subscriptions=[]
            def create_publisher(self,cls,topic,qos):self.created.append(topic);return NS()
            def create_subscription(self,cls,topic,callback,qos):
                self.subscriptions.append((topic,callback));return NS()
            def create_timer(self,*a):return NS()
            def get_clock(self):return NS(now=lambda:NS(nanoseconds=1_000_000_000))
            def get_publishers_info_by_topic(self,topic):return [NS(endpoint_gid=(1,2))]
        classes=('VehicleStatus VehicleLocalPosition EstimatorStatusFlags VehicleLandDetected '
                 'ManualControlSetpoint ManualControlSwitches VehicleCommandAck FailsafeFlags '
                 'VehicleOdometry OffboardControlMode TrajectorySetpoint VehicleCommand').split()
        stack.enter_context(patch.dict(sys.modules,{'rclpy':NS(),'rclpy.node':NS(Node=Node),
            'rclpy.qos':NS(qos_profile_sensor_data=object()),
            'nav_msgs.msg':NS(Odometry=object),'std_msgs.msg':NS(String=object),
            'px4_msgs.msg':NS(**{k:type(k,(),{}) for k in classes})}))
        self.core=FlightRuntimeCore(0.)
        self.core.bind_external_ev(ExternalEvEvidence('fixture',(1,2)))

    def test_runtime_has_three_control_publishers_and_external_subscription(self):
        n=create_node(self.core,isolated=True,clock=lambda:1.)
        self.assertEqual(len(n.created),3)
        self.assertFalse(any(t.endswith('/vehicle_visual_odometry') for t in n.created))
        self.assertTrue(all(t.startswith('/robocup/runtime_test/') for t in n.created))
        callbacks=[cb for t,cb in n.subscriptions if t.endswith('/vehicle_visual_odometry')]
        self.assertEqual(len(callbacks),1)
        callbacks[0](message())
        self.assertEqual(self.core.last_ev_at,1.)
        self.assertEqual(n.input_counts['external_ev_received'],1)

    def test_bad_frame_or_publisher_never_renews(self):
        n=create_node(self.core,isolated=True,clock=lambda:1.)
        n.external_odometry(message(pose_frame=1))
        self.assertEqual(self.core.last_ev_at,-1.)
        self.assertIsNotNone(self.core.fault)

    def test_reset_counter_change_latches(self):
        e=self.core.external_ev
        self.assertTrue(e.receive_message(message(),(1,2),1_000_000_000,1.))
        self.assertFalse(e.receive_message(message(reset_counter=1),(1,2),1_000_000_000,1.))

    def test_graph_source_change_rejected(self):
        n=create_node(self.core,isolated=True,clock=lambda:1.)
        n.get_publishers_info_by_topic=lambda topic:[NS(endpoint_gid=(9,))]
        n.external_odometry(message())
        self.assertEqual(self.core.last_ev_at,-1.)
        self.assertIsNotNone(self.core.fault)

    def test_real_route_stays_disabled(self):
        with self.assertRaisesRegex(ValueError,'isolated validation only'):create_node(self.core)

    def test_bound_real_task_factory_routes_controls_only(self):
        # Exercise the actual production factory with fake ROS and fixture
        # authorization checks. This does NOT validate a real flight permit.
        from task_flight_controller import TaskFlightCore
        from task_flight_release import TaskFlightPermit
        from resident_status_binding import ResidentStatusBinding
        from test_aux_switch_decoder import params
        from tempfile import TemporaryDirectory
        import json
        core=TaskFlightCore(0.,params(),{})
        evidence=ExternalEvEvidence('fixture',(1,2));core.bind_external_ev(evidence)
        binding=object.__new__(ResidentStatusBinding);binding.evidence=evidence
        permit=object.__new__(TaskFlightPermit)
        for key,value in dict(height=.86,hover=3.,session_id='fixture',release_hash='fixture').items():
            object.__setattr__(permit,key,value)
        with TemporaryDirectory() as directory, ExitStack() as stack:
            root=Path(directory);markers=root/'evidence/live_release_consumed';markers.mkdir(parents=True)
            (markers/'fixture.json').write_text(json.dumps({'release_sha256':'fixture'}))
            stack.enter_context(patch('flight_release.ROOT',root))
            stack.enter_context(patch.object(TaskFlightPermit,'task_config',new_callable=PropertyMock,return_value={}))
            stack.enter_context(patch.object(TaskFlightPermit,'before_arm'))
            stack.enter_context(patch.object(ResidentStatusBinding,'check'))
            stack.enter_context(patch.dict(sys.modules,{'task_scene_input':NS(TaskSceneInput=lambda *a,**k:NS())}))
            n=create_node(core,exercise=True,live_permit=permit,resident_binding=binding,clock=lambda:1.)
            self.assertEqual(set(n.created),{'/fmu/in/offboard_control_mode',
                '/fmu/in/trajectory_setpoint','/fmu/in/vehicle_command'})
            n.external_odometry(message())
            self.assertEqual(core.last_ev_at,1.)
            core.controller.state='LAND'
            n.external_odometry(message(timestamp=1_010_000,timestamp_sample=1_010_000))
            self.assertEqual(evidence.count,2)
            core.close('fixture task exit')
            self.assertIsNone(evidence.fault)
            self.assertNotIn('vehicle_visual_odometry',n.pubs)
            with patch.object(ResidentStatusBinding,'verify_graph',side_effect=RuntimeError('source lost')):
                n.audit(1.)
            self.assertIn('resident binding lost',core.fault)


if __name__=='__main__':unittest.main()
