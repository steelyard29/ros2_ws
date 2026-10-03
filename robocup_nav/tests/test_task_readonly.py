"""Instantiate actual runtime/scene with generated messages and inert ROS Node.

No ROS Context or network is started. Explicit task publisher creation fails;
this does not model implicit rclpy metadata publishers such as parameter_events.
"""
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
import subprocess
from unittest.mock import patch
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from flight_runtime import create_node
from task_domain_io import ScopedPerceptionNode
from task_flight_controller import TaskFlightCore
from task_readonly_session import result_code, input_delivery_observed
from test_aux_switch_decoder import params
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class InertNode:
    def __init__(self,*args,domain=0,**kw):
        self.context=NS(get_domain_id=lambda:domain);self.subscriptions=[];self.writers={}
    def create_publisher(self,*args,**kw):raise AssertionError('readonly must not construct ANY publisher')
    def create_subscription(self,cls,topic,cb,qos):self.subscriptions.append((cls,topic,cb));return cb
    def create_timer(self,*args):return None
    def count_publishers(self,topic):return self.writers.get(topic,0)
    def get_publishers_info_by_topic(self,topic):return []
    def get_topic_names_and_types(self):return [(k,[]) for k in self.writers]
    def get_clock(self):return NS(now=lambda:NS(nanoseconds=10_000_000_000))
    def destroy_node(self):pass


class ReadonlyTests(unittest.TestCase):
    def test_cli_default_never_initializes_ros(self):
        script=Path(__file__).resolve().parents[1]/'scripts/task_readonly_session.py'
        result=subprocess.run([sys.executable,str(script)],capture_output=True,text=True,timeout=3)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(json.loads(result.stdout)['task_output_publishers_created'],0)

    def test_exception_cleanup_and_deadline_cannot_report_success(self):
        good=dict(observation_completed=True,contexts_closed=True,cleanup_errors=[],elapsed_s=20.)
        self.assertEqual(result_code(good),0)
        for change in (dict(error='analysis failed'),dict(observation_completed=False),
                       dict(contexts_closed=False),dict(cleanup_errors=['failed']),dict(elapsed_s=28.)):
            self.assertEqual(result_code(dict(good,**change)),1)

    def make(self):
        cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/roundtrip_task.yaml').read_text())
        core=TaskFlightCore(9.,params(),cfg);sensor=InertNode(domain=176)
        with patch('rclpy.node.Node',InertNode),patch('socket.socket',side_effect=AssertionError):
            node=create_node(core,task_preview=True,perception_node=ScopedPerceptionNode(sensor),clock=lambda:10.)
        return core,node,sensor

    def test_missing_each_input_or_tracking_never_claims_delivery(self):
        keys=('vehicle_status_v1','vehicle_local_position','estimator_status_flags',
            'vehicle_land_detected','manual_control_setpoint','vio_observed')
        node=NS(input_counts=dict.fromkeys(keys,1),task_input=NS(seen={'vio','map','depth'}))
        core=NS(ev=NS(guard=NS(tracking_stamp=1.)))
        self.assertTrue(input_delivery_observed(node,core))
        for stamp in (None,0.):
            core.ev.guard.tracking_stamp=stamp
            self.assertFalse(input_delivery_observed(node,core))
        core.ev.guard.tracking_stamp=1.
        for key in keys:
            node.input_counts[key]=0
            self.assertFalse(input_delivery_observed(node,core),key)
            node.input_counts[key]=1
        for key in ('vio','map','depth'):
            node.task_input.seen.remove(key)
            self.assertFalse(input_delivery_observed(node,core),key)
            node.task_input.seen.add(key)

    def test_armed_status_latches_fault_without_outputs(self):
        core,node,sensor=self.make()
        msg=NS(arming_state=2,nav_state=3,pre_flight_checks_pass=True,failsafe=False,
            failsafe_and_user_took_over=False,gcs_connection_lost=False)
        node.status(msg)
        self.assertTrue(node.armed_on_real_input)
        self.assertIn('read-only task preview observed real aircraft armed',core.fault)
        self.assertEqual(node.pubs,{})
        self.assertEqual(node.counts,{})

    def test_actual_constructor_creates_no_publishers_in_either_domain(self):
        core,node,sensor=self.make()
        self.assertEqual(node.pubs,{})
        self.assertIsNone(node.task_input.goal_pub);self.assertIsNone(node.task_input.status_pub)
        self.assertTrue(node.task_input.owns())
        r=node.report();self.assertTrue(r['task_read_only']);self.assertEqual(r['output_topics'],[])
        self.assertEqual(r['dds_domains'],dict(px4=0,perception=176))
        self.assertIn('alignment_reviewed',r['task_inputs']['configuration_blockers'])

    def test_actual_preview_accepts_container_readonly_port_without_outputs(self):
        from task_perception_pipe import ReadonlyPipePort
        cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/roundtrip_task.yaml').read_text())
        core=TaskFlightCore(9.,params(),cfg)
        remote=ReadonlyPipePort('offline',clock=lambda:10.)
        with patch('rclpy.node.Node',InertNode),patch('socket.socket',side_effect=AssertionError):
            node=create_node(core,task_preview=True,perception_node=ScopedPerceptionNode(remote),clock=lambda:10.)
        self.assertEqual(set(remote.callbacks),set(remote.reads))
        self.assertEqual(node.pubs,{})
        self.assertIsNone(node.task_input.goal_pub)
        self.assertEqual(node.report()['dds_domains'],dict(px4=0,perception=176))

    def test_tick_and_pose_never_invoke_executive_or_ev_generation(self):
        core,node,sensor=self.make()
        with patch.object(core,'tick',side_effect=AssertionError),patch.object(core,'pose',side_effect=AssertionError):
            node.tick();node.pose(Odometry())
        self.assertEqual(node.counts,{})
        self.assertFalse(core.ownership_ok)

    def test_real_candidate_callbacks_still_parse_vio_and_leave_config_unreviewed(self):
        core,node,sensor=self.make();msg=Odometry()
        msg.header.stamp.sec=10;msg.header.frame_id='odom';msg.child_frame_id='base_link';msg.pose.pose.orientation.w=1.
        for cls,topic,cb in sensor.subscriptions:
            if topic=='/visual_slam/tracking/odometry':cb(msg)
        node.tick()
        self.assertIsNotNone(node.task_input.pose_data)
        self.assertFalse(core.task_config['alignment_reviewed'])
        self.assertEqual(node.task_input.report()['accepted_source_stamps']['vio'],10_000_000_000)

    def test_competing_real_control_or_goal_writers_report_conflict(self):
        for where,topic in [('px4','/fmu/in/trajectory_setpoint'),('sensor','/robocup/task/navigation_goal')]:
            core,node,sensor=self.make()
            (node if where=='px4' else sensor).writers[topic]=1
            node.tick()
            self.assertIn('conflict',core.fault)
            self.assertEqual(node.counts,{})

    def test_preview_still_rejects_exercise_or_real_permit(self):
        core,_,_=self.make()
        for kwargs in (dict(exercise=True),dict(live_permit=object()),dict(isolated=True)):
            with self.assertRaises(ValueError):create_node(core,task_preview=True,**kwargs)


if __name__=='__main__':unittest.main()
