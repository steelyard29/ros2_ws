import json
from pathlib import Path
import subprocess
import sys
import unittest
import tempfile
import uuid
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from task_navigation_pipe import NavigationRequestGuard
from task_perception_pipe import ReadonlyPipePort
from std_msgs.msg import String
from task_pipe_authority import verify_grant,grant_for,SORTIE_SECONDS


class NavigationPipeTests(unittest.TestCase):
    topic='/robocup/task_test/navigation_goal'
    def packet(self,**kwargs):
        p=dict(session='s',seq=1,at=10.,topic=self.topic,
            data=json.dumps(dict(frame_id='odom',goal_id='s:OUTBOUND',position=[2.5,0.,.86],yaw=0.)))
        p.update(kwargs);return p

    def guard(self):return NavigationRequestGuard('s',{self.topic},lambda:10.)

    def test_valid_goal_and_duplicate_latches(self):
        g=self.guard();self.assertEqual(g.accept(self.packet())[0],self.topic)
        with self.assertRaises(ValueError):g.accept(self.packet())
        with self.assertRaises(ValueError):g.accept(self.packet(seq=2))

    def test_wrong_session_topic_stale_future_sequence_size_rejected(self):
        for changes in (dict(session='other'),dict(topic='/fmu/in/vehicle_command'),dict(at=9.7),
                        dict(at=10.1),dict(seq=3),dict(data='x'*65537)):
            with self.assertRaises(ValueError):self.guard().accept(self.packet(**changes))

    def test_changed_coordinates_cannot_reuse_goal_id(self):
        g=self.guard();g.accept(self.packet())
        d=json.loads(self.packet()['data']);d['position'][0]=3.
        with self.assertRaises(ValueError):g.accept(self.packet(seq=2,data=json.dumps(d)))

    def test_production_navigation_writes_rejected(self):
        with self.assertRaises(ValueError):ReadonlyPipePort('s',navigation_writes=True)
        script=Path(__file__).resolve().parents[1]/'scripts/task_perception_reader.py'
        r=subprocess.run([sys.executable,str(script),'--isolated-navigation-writes'],capture_output=True,text=True,timeout=3)
        self.assertNotEqual(r.returncode,0)
        self.assertIn('require isolated',r.stderr)

    def test_host_port_allows_only_string_navigation_topics(self):
        p=ReadonlyPipePort('s',isolated=True,navigation_writes=True,clock=lambda:10.)
        with self.assertRaises(ValueError):p.create_publisher(String,'/fmu/in/vehicle_command',10)
        pub=p.create_publisher(String,self.topic,10)
        with self.assertRaises(RuntimeError):pub.publish(String(data=self.packet()['data']))

    def authority_fixture(self,root):
        flight=str(uuid.uuid4())
        grant=dict(session='pipe',flight_session=flight,release_sha256='a'*64,deadline=210.,expires=1300.)
        marker=Path(root)/'evidence/live_release_consumed'/(flight+'.json')
        marker.parent.mkdir(parents=True)
        marker.write_text(json.dumps(dict(session_id=flight,release_sha256='a'*64)))
        return grant,marker

    def test_single_sortie_grant_binds_marker_session_deadline_expiry(self):
        with tempfile.TemporaryDirectory() as root:
            grant,marker=self.authority_fixture(root)
            self.assertEqual(verify_grant(grant,'pipe',210.,root,monotonic=10.,wall=1000.),grant)
            for changes in (dict(session='other'),dict(deadline=211.),dict(expires=1199.),
                            dict(release_sha256='b'*64),dict(flight_session='../escape')):
                with self.assertRaises((ValueError,OSError)):
                    verify_grant(dict(grant,**changes),'pipe',210.,root,monotonic=10.,wall=1000.)
            marker.unlink()
            with self.assertRaises(OSError):verify_grant(grant,'pipe',210.,root,monotonic=10.,wall=1000.)

    def test_no_unbounded_or_expired_reader_grant(self):
        with tempfile.TemporaryDirectory() as root:
            grant,_=self.authority_fixture(root)
            for deadline in (10.,9.,211.,float('nan'),float('inf')):
                with self.assertRaises(ValueError):
                    verify_grant(dict(grant,deadline=deadline),'pipe',deadline,root,monotonic=10.,wall=1000.)

    def test_plain_or_vertical_permit_cannot_grant_task_output(self):
        from flight_release import FlightPermit
        for permit in (None,object(),object.__new__(FlightPermit)):
            with self.assertRaises(ValueError):grant_for(permit,'pipe',210.)

    def test_exact_task_permit_still_checks_before_arm_and_consumed_marker(self):
        from task_flight_release import TaskFlightPermit
        with tempfile.TemporaryDirectory() as root:
            grant,marker=self.authority_fixture(root)
            permit=object.__new__(TaskFlightPermit)
            for k,v in dict(session_id=grant['flight_session'],release_hash='a'*64,expires=1300.).items():
                object.__setattr__(permit,k,v)
            with patch.object(TaskFlightPermit,'before_arm') as check,patch('flight_release.ROOT',Path(root)), \
                    patch('task_pipe_authority.time.monotonic',return_value=10.), \
                    patch('task_pipe_authority.time.time',return_value=1000.):
                port=ReadonlyPipePort('pipe',navigation_writes=True,live_permit=permit,deadline=210.)
                check.assert_called_once()
                self.assertEqual(port.context.get_domain_id(),176)
                self.assertEqual(port.live_grant,grant)
                with self.assertRaises(ValueError):port.create_publisher(String,'/fmu/in/vehicle_command',10)
                marker.unlink()
                with self.assertRaises(OSError):grant_for(permit,'pipe',210.)

    def test_real_reader_cli_cannot_start_without_grant_or_in_isolated_mode(self):
        import time
        script=Path(__file__).resolve().parents[1]/'scripts/task_perception_reader.py'
        with tempfile.TemporaryDirectory() as root:
            control=Path(root)/'control.json';control.write_text(json.dumps(dict(session='s',stop=False)))
            base=[sys.executable,str(script),'--authorized-task-navigation','--session','s',
                  '--deadline',str(time.monotonic()+100),'--control-file',str(control),
                  '--receipt-file',str(Path(root)/'receipt.json')]
            for extra in ([],['--isolated']):
                r=subprocess.run(base+extra,capture_output=True,text=True,timeout=3)
                self.assertNotEqual(r.returncode,0)
                self.assertFalse((Path(root)/'receipt.json').exists())

    def test_launch_budget_and_mode_do_not_leak_into_readonly(self):
        from task_perception_pipe import ContainerTaskInputs
        from types import SimpleNamespace as NS
        for live in (False,True):
            with self.subTest(live=live),tempfile.TemporaryDirectory() as root:
                deadline=210. if live else 30.
                grant=dict(test_only=True)
                with patch('task_pipe_authority.grant_for',return_value=grant), \
                        patch('task_perception_pipe.ROOT',Path(root)), \
                        patch('task_perception_pipe.time.monotonic',return_value=10.), \
                        patch('task_perception_pipe.subprocess.Popen') as launch, \
                        patch('task_perception_pipe.os.set_blocking'):
                    stream=Mock();stream.fileno.return_value=99
                    launch.return_value=NS(stdout=stream,stdin=stream)
                    t=ContainerTaskInputs(root,deadline,navigation_writes=live,
                                          live_permit=object() if live else None)
                    t.start(discovery_only=True)
                    args=launch.call_args.args[0]
                    self.assertIn('201' if live else '26',args)
                    self.assertEqual('--authorized-task-navigation' in args,live)
                    self.assertNotIn('--isolated-navigation-writes',args)
                    self.assertTrue(t.proxy.discovery_only)
                    self.assertEqual(json.loads(t.control.read_text())['task_authority'],grant if live else None)
                    t.log.close()
        with tempfile.TemporaryDirectory() as root,patch('task_perception_pipe.time.monotonic',return_value=10.):
            with self.assertRaises(ValueError):ContainerTaskInputs(root,210.).start()

    def test_production_runner_constructs_pipe_before_precheck_and_closes_on_failure(self):
        from types import SimpleNamespace as NS
        from live_flight_runtime import run
        permit=NS(session_id=str(uuid.uuid4()),release_hash='a'*64)
        pipe=Mock();pipe.report.return_value={}
        with tempfile.TemporaryDirectory() as root, \
                patch('live_flight_runtime.ROOT',Path(root)), \
                patch('rclpy.init'),patch('rclpy.try_shutdown'), \
                patch('task_perception_pipe.ContainerTaskInputs',return_value=pipe) as factory, \
                patch('live_flight_runtime.precheck',side_effect=ValueError('offline precheck stop')), \
                patch('builtins.print'),patch('socket.socket',side_effect=AssertionError):
            self.assertEqual(run(permit,{},task=True),1)
            self.assertIs(factory.call_args.kwargs['live_permit'],permit)
            self.assertTrue(factory.call_args.kwargs['navigation_writes'])
            pipe.start.assert_called_once_with(discovery_only=True)
            pipe.close.assert_called_once()
            pipe.activate_inputs.assert_not_called()


if __name__=='__main__':unittest.main()
