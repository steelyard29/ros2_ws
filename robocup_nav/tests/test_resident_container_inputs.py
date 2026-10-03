"""Process lifecycle with fake Popen and real private pipes; never calls Docker."""
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from resident_container_inputs import ContainerResidentInputs,ROOT,container_path
from disarmed_ev_lifecycle import atomic_json


class ContainerTests(unittest.TestCase):
    def setUp(self):
        self.now=100.;self.calls=[]
        tmp=tempfile.TemporaryDirectory(dir=ROOT/'evidence');self.addCleanup(tmp.cleanup)
        read,write=os.pipe();self.write=write;self.addCleanup(os.close,write)
        self.process=NS(stdout=os.fdopen(read,'rb',buffering=0),code=None)
        self.process.poll=lambda:self.process.code
        self.addCleanup(self.process.stdout.close)
        def spawn(command,**kwargs):self.calls.append((command,kwargs));return self.process
        self.m=ContainerResidentInputs(Path(tmp.name)/'run',isolated=True,
            clock=lambda:self.now,popen=spawn)
        self.addCleanup(lambda:self.m.log.close() if self.m.log else None)

    def test_inert_construction_and_single_start(self):
        self.assertEqual(self.calls,[]);self.assertFalse(self.m.out.exists())
        self.m.start(120.)
        command=self.calls[0][0]
        self.assertIn('--run-isolated',command);self.assertIn('183',command)
        self.assertNotIn('--privileged',command)
        self.now+=.21;self.m.pump()
        self.assertEqual(json.loads(self.m.control.read_text())['seq'],2)
        with self.assertRaises(RuntimeError):self.m.start(120.)

    def test_stop_is_nonblocking_and_receipt_required(self):
        self.m.start(120.);self.m.stop()
        self.assertTrue(json.loads(self.m.control.read_text())['stop'])
        self.assertTrue(self.m.poll_close()['running'])
        self.process.code=0
        result=self.m.poll_close()
        self.assertFalse(result['confirmed']);self.assertFalse(result['running'])

    def test_matching_receipt_confirms_close(self):
        self.m.start(120.);self.m.stop();self.process.code=0
        atomic_json(self.m.receipt,dict(session=self.m.session,started_at=99.,closed_at=100.,
                                      ros_closed=True,exit_code=0))
        self.assertTrue(self.m.poll_close()['confirmed'])

    def test_client_exit_before_receipt_does_not_cache_false_failure(self):
        self.m.start(120.);self.m.stop();self.process.code=0
        self.assertTrue(self.m.poll_close()['awaiting_receipt'])
        self.now+=.04
        atomic_json(self.m.receipt,dict(session=self.m.session,started_at=99.,closed_at=100.,
                                      ros_closed=True,exit_code=0))
        self.assertTrue(self.m.poll_close()['confirmed'])

    def test_missing_receipt_wait_is_bounded(self):
        self.m.start(120.);self.m.stop();self.process.code=0
        self.assertTrue(self.m.poll_close()['awaiting_receipt'])
        self.now+=.51
        result=self.m.poll_close()
        self.assertFalse(result['confirmed'])
        self.assertFalse(result.get('awaiting_receipt',False))

    def test_eof_latches_and_no_restart(self):
        self.m.start(120.)
        # Process end is authoritative; no re-launch or old receipt reuse.
        self.process.code=1
        with self.assertRaisesRegex(RuntimeError,'process ended'):self.m.pump()
        self.assertTrue(self.m.stopping)
        with self.assertRaises(RuntimeError):self.m.start(120.)
        self.assertEqual(len(self.calls),1)

    def test_owner_stall_does_not_renew(self):
        self.m.start(120.);self.now+=1.1
        with self.assertRaisesRegex(TimeoutError,'loop stalled'):self.m.pump()
        self.assertTrue(json.loads(self.m.control.read_text())['stop'])

    def test_bad_paths_and_real_mode_rejected(self):
        with self.assertRaises(ValueError):container_path('/tmp/unrelated')
        with self.assertRaises(ValueError):ContainerResidentInputs(self.m.out)
        with self.assertRaises(ValueError):ContainerResidentInputs('/tmp/unrelated',isolated=True)
        self.assertEqual(self.calls,[])

    def test_spawn_failure_closes_log_and_cannot_retry(self):
        def fail(*args,**kwargs):raise OSError('synthetic spawn failure')
        self.m.popen=fail
        with self.assertRaises(OSError):self.m.start(120.)
        self.assertTrue(self.m.stopping);self.assertTrue(self.m.log.closed)
        self.assertTrue(json.loads(self.m.control.read_text())['stop'])
        self.assertFalse(self.m.poll_close()['process_started'])
        with self.assertRaises(RuntimeError):self.m.start(120.)

    def test_wrong_session_receipt_never_confirms(self):
        self.m.start(120.);self.m.stop();self.process.code=0
        atomic_json(self.m.receipt,dict(session='old',started_at=99.,closed_at=100.,
                                      ros_closed=True,exit_code=0))
        self.assertFalse(self.m.poll_close()['confirmed'])

    def test_sensor_only_command_has_no_bench_timer_or_ev_flag(self):
        self.m=ContainerResidentInputs(self.m.out,sensor_only=True,domain=176,
                                      clock=lambda:self.now,popen=self.m.popen)
        self.m.start()
        command=self.calls[0][0]
        self.assertIn('--lease-sensor-only',command);self.assertIn('176',command)
        self.assertNotIn('--deadline',command);self.assertNotIn('timeout',command)
        self.assertNotIn('--run-isolated',command)
        self.assertFalse(self.m.port.isolated)
        for _ in range(151):self.now+=.4;self.m.pump()
        self.assertFalse(self.m.stopping)
        self.m.stop();self.process.code=0;self.m.poll_close()

    def test_mixed_mode_and_sensor_bench_deadline_rejected(self):
        with self.assertRaises(ValueError):
            ContainerResidentInputs(self.m.out,isolated=True,sensor_only=True)
        self.m=ContainerResidentInputs(self.m.out,sensor_only=True,domain=176,
                                      clock=lambda:self.now,popen=self.m.popen)
        with self.assertRaises(ValueError):self.m.start(120.)
        self.assertEqual(self.calls,[])


if __name__=='__main__':unittest.main()
