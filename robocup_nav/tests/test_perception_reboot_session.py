"""No Docker/ROS/hardware: exercise the host wrapper's ownership and budget."""
import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import perception_reboot_session as m


class RebootSessionTests(unittest.TestCase):
    def simulate(self,mode):
        calls=[];clock=types.SimpleNamespace(now=100.)
        before=dict(Id='exact-owned-container',State=dict(Running=False,Status='exited'),
            Config=dict(Entrypoint=['/usr/local/bin/scripts/deploy-entrypoint.sh'],Cmd=['sleep','infinity']),
            Mounts=[dict(Source=str(m.ROOT.parent),Destination='/workspaces/ros2_ws')])
        def output(command,**kw):
            calls.append(command)
            if command==['docker','inspect',m.CONTAINER]:return json.dumps([before])
            if mode=='inspect_error':raise subprocess.TimeoutExpired('inspect',1)
            return json.dumps(dict(Running=mode=='pass',Pid=12 if mode=='pass' else 0))
        def run(command,**kw):
            calls.append(command)
            if command[1]=='start':clock.now+=5
            else:clock.now+=1
            return types.SimpleNamespace(returncode=0)
        child=types.SimpleNamespace(returncode=None)
        def wait(timeout):
            self.assertLessEqual(timeout,77.)
            if mode=='timeout':
                clock.now+=timeout
                raise subprocess.TimeoutExpired('fixture',timeout)
            clock.now+=30;child.returncode=0 if mode=='pass' else 1
        child.wait=wait;child.poll=lambda:child.returncode
        child.kill=lambda:setattr(child,'returncode',-9)
        def spawn(command,**kw):calls.append(command);return child
        with tempfile.TemporaryDirectory() as directory,contextlib.ExitStack() as stack:
            root=Path(directory);(root/'evidence').mkdir()
            stack.enter_context(patch.object(m,'ROOT',root))
            stack.enter_context(patch.object(m,'validate_container'))
            stack.enter_context(patch.object(m.time,'monotonic',side_effect=lambda:clock.now))
            stack.enter_context(patch.object(m.subprocess,'check_output',side_effect=output))
            stack.enter_context(patch.object(m.subprocess,'run',side_effect=run))
            stack.enter_context(patch.object(m.subprocess,'Popen',side_effect=spawn))
            stack.enter_context(patch.object(sys,'argv',['recovery','--authorized-sensor-recovery']))
            with contextlib.redirect_stdout(io.StringIO()) as result:code=m.main()
            return code,json.loads(result.getvalue()),calls

    def test_pass_retains_and_includes_docker_time(self):
        code,r,calls=self.simulate('pass')
        self.assertEqual(code,0);self.assertEqual(r['elapsed_s'],35.)
        self.assertFalse(any(c[1] in ('stop','kill') for c in calls))
        cmd=next(c for c in calls if c[1]=='exec')
        self.assertIn('--session-deadline 190.0',cmd[-1])
        self.assertIn('--restore-support-sensors',cmd[-1])

    def test_failure_stops_exact_owned_container(self):
        code,r,calls=self.simulate('fail')
        self.assertEqual(code,1);self.assertTrue(r['container_stopped'])
        self.assertIn(['docker','stop','--time','1','exact-owned-container'],calls)
        self.assertEqual(sum(c[1]=='start' for c in calls),1)

    def test_timeout_no_retry_with_cleanup_before_90(self):
        code,r,calls=self.simulate('timeout')
        self.assertEqual(code,1);self.assertTrue(r['deadline_met']);self.assertTrue(r['container_stopped'])
        self.assertEqual(sum(c[1]=='start' for c in calls),1)

    def test_default_never_calls_docker(self):
        with patch.object(sys,'argv',['recovery']),patch.object(m.subprocess,'check_output',side_effect=AssertionError),contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(m.main(),0)

    def test_uncertain_final_state_also_stops_owned_container(self):
        code,r,calls=self.simulate('inspect_error')
        self.assertEqual(code,1)
        self.assertIn(['docker','stop','--time','1','exact-owned-container'],calls)


if __name__=='__main__':unittest.main()
