"""Exercise real rollout orchestration with inert children and a fake clock."""
from contextlib import ExitStack,redirect_stdout
import importlib.util
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch,Mock

ROOT=Path(__file__).resolve().parents[1]
NS=types.SimpleNamespace


def observation():
    metrics=dict(max_source_gap_s=.034,max_gap_s=.04,max_source_age_s=.02)
    return dict(passed=True,flight_ready=False,domain=176,passive_only=True,
        acceptance_window=dict(phase='PASSED',passed=True,fault=None,measure_started=2.,finished=14.,
                               required_measurement_s=12.,flight_ready=False),
        ground_reference=dict(position=[0.,0.,0.],max_displacement_m=.001,samples=60),
        camera_tf=[.196,.025,-.05,0.,0.,0.,1.],navigation_goal_publishers=0,
        qualified_source_identity={k:[1,2,3] for k in ('vio','depth','status')},
        streams={k:dict(metrics) for k in ('vio','depth','map')})


class TrialOfflineTests(unittest.TestCase):
    def run_trial(self,first=None,second=None,stubborn=False,conflict=False,support=False,deadline=None):
        spec=importlib.util.spec_from_file_location('trial_fixture',ROOT/'scripts/perception_transport_trial.py')
        m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
        clock=NS(now=100.);children=[];signals=[];runs=[]
        def spawn(*a,**kw):
            self.assertTrue(kw['start_new_session'])
            child=NS(pid=900000+len(children),alive=True)
            child.poll=lambda:None if child.alive else 0
            children.append(child);return child
        def killpg(pid,sig):
            self.assertIn(pid,[c.pid for c in children])
            signals.append((pid,sig))
            if not stubborn or sig==signal.SIGKILL:
                next(c for c in children if c.pid==pid).alive=False
        replies=[first if first is not None else observation(),second if second is not None else observation()]
        def run(command,**kwargs):
            self.assertIn('--qualified-window',command)
            self.assertLessEqual(kwargs['timeout'],33)
            runs.append(command);clock.now+=14
            reply=replies[len(runs)-1]
            if isinstance(reply,Exception):raise reply
            return NS(stdout=json.dumps(reply),stderr='fixture stderr',returncode=0)
        with tempfile.TemporaryDirectory(prefix='trial_offline_') as d,ExitStack() as stack:
            root=Path(d);(root/'evidence').mkdir()
            m.ROOT=root;m.time=NS(monotonic=lambda:clock.now,strftime=lambda _: 'fixture',
                                 sleep=lambda dt:setattr(clock,'now',clock.now+dt))
            stack.enter_context(patch.dict(os.environ))
            argv=['trial','--authorized-sensor-rollout','--expect-stopped']
            if support:argv+=['--restore-support-sensors']
            if deadline is not None:argv+=['--session-deadline',str(deadline)]
            stack.enter_context(patch.object(sys,'argv',argv))
            stack.enter_context(patch.object(m.Path,'exists',return_value=True))
            stack.enter_context(patch.object(m,'DeadlineAlarm',return_value=Mock()))
            stack.enter_context(patch.object(m,'verify_stopped',side_effect=RuntimeError('conflict') if conflict else None,return_value={}))
            stack.enter_context(patch.object(m,'tree',return_value={}))
            stack.enter_context(patch.object(m,'group_alive',side_effect=lambda pid:any(c.pid==pid and c.alive for c in children)))
            stack.enter_context(patch.object(m.subprocess,'Popen',side_effect=spawn))
            stack.enter_context(patch.object(m.subprocess,'run',side_effect=run))
            stack.enter_context(patch.object(m.os,'killpg',side_effect=killpg))
            stack.enter_context(patch.object(m.os,'kill',side_effect=AssertionError('no old PID signal')))
            stack.enter_context(patch('socket.socket',side_effect=AssertionError('no network')))
            with redirect_stdout(io.StringIO()) as output:code=m.main()
            report=json.loads(output.getvalue())
            logs={p.name:p.read_text() for p in (root/'evidence/fixture').glob('observer*')}
            return code,report,children,signals,runs,logs

    def test_success_retains_three_owned_launchers_no_signals(self):
        code,r,children,signals,runs,_=self.run_trial()
        self.assertEqual(code,0,r);self.assertTrue(r['passed']);self.assertFalse(r['flight_ready'])
        self.assertEqual(len(children),3);self.assertEqual(len(runs),2);self.assertEqual(signals,[])

    def test_reboot_restores_six_groups_under_host_deadline(self):
        code,r,children,signals,_,_=self.run_trial(support=True,deadline=185.)
        self.assertEqual(code,0,r);self.assertEqual(len(children),6);self.assertEqual(signals,[])
        self.assertEqual(r['session_deadline_monotonic'],185.)

    def test_failed_reboot_stops_support_too_without_retry(self):
        bad=observation();bad['passed']=False
        code,r,children,signals,runs,_=self.run_trial(first=bad,support=True,deadline=185.)
        self.assertEqual(code,1);self.assertEqual(len(children),4);self.assertEqual(len(runs),1)
        self.assertEqual(r['remaining_owned_groups'],[])

    def test_start_conflict_never_launches(self):
        code,r,children,signals,runs,_=self.run_trial(conflict=True)
        self.assertEqual(code,1);self.assertEqual(children,[]);self.assertEqual(runs,[]);self.assertEqual(signals,[])

    def test_first_failure_stops_only_perception_no_retry(self):
        bad=observation();bad['passed']=False
        code,r,children,signals,runs,_=self.run_trial(first=bad)
        self.assertEqual(code,1);self.assertEqual(len(children),1);self.assertEqual(len(runs),1)
        self.assertEqual(r['remaining_owned_groups'],[])

    def test_final_timeout_kills_stubborn_owned_groups_within_budget(self):
        error=subprocess.TimeoutExpired('observer',33,output='partial diagnostic',stderr='timeout stderr')
        code,r,children,signals,runs,logs=self.run_trial(second=error,stubborn=True)
        self.assertEqual(code,1);self.assertEqual(len(runs),2);self.assertEqual(len(children),3)
        self.assertEqual(r['remaining_owned_groups'],[]);self.assertTrue(r['deadline_met'])
        self.assertEqual({sig for _,sig in signals},{signal.SIGINT,signal.SIGTERM,signal.SIGKILL})
        self.assertIn('partial diagnostic',''.join(logs.values()))

    def test_missing_or_short_qualification_is_not_accepted(self):
        for change in ('missing','short','nan_ground','nan_tf'):
            with self.subTest(change=change):
                bad=observation()
                if change=='missing':bad.pop('acceptance_window')
                if change=='short':bad['acceptance_window']['finished']=3.
                if change=='nan_ground':bad['ground_reference']['max_displacement_m']=float('nan')
                if change=='nan_tf':bad['camera_tf'][0]=float('nan')
                code,r,children,_,runs,_=self.run_trial(first=bad)
                self.assertEqual(code,1,r)
                self.assertEqual(len(children),1);self.assertEqual(len(runs),1)

    def test_replaced_source_or_moved_ground_stops_full_chain(self):
        for change in ('source','ground'):
            with self.subTest(change=change):
                bad=observation()
                if change=='source':bad['qualified_source_identity']['vio']=[4,5,6]
                else:bad['ground_reference']['position'][0]=.1
                code,r,children,_,runs,_=self.run_trial(second=bad)
                self.assertEqual(code,1);self.assertEqual(len(children),3);self.assertEqual(len(runs),2)
                self.assertEqual(r['remaining_owned_groups'],[])

    def test_actual_observer_reports_accepted_by_real_rollout_code(self):
        from test_perception_observer_offline import ObserverOfflineTests
        _,first,_=ObserverOfflineTests().observe('startup_transient')
        _,second,_=ObserverOfflineTests().observe('healthy')
        code,r,_,_,_,_=self.run_trial(first=first,second=second)
        self.assertEqual(code,0,r)


if __name__=='__main__':unittest.main()
