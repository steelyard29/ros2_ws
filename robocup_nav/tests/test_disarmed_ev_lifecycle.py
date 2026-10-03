import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from disarmed_ev_lifecycle import discovery_state,atomic_json,request_stop,stop_requested,check_receipt

class Tests(unittest.TestCase):
    def classify(self,counts=None,outputs=None,elapsed=0.,ready=False,changed=None):
        return discovery_state({'ev':1} if outputs is None else outputs,'ev',
            {'status':1,'flags':1} if counts is None else counts,changed or [],elapsed,ready)
    def test_missing_is_wait_not_conflict(self):
        r=self.classify({'status':0,'flags':1},elapsed=1.9)
        self.assertEqual(r['state'],'discovering');self.assertEqual(r['missing'],['status'])
    def test_same_two_second_timeout_precise(self):
        r=self.classify({'status':0,'flags':1},elapsed=2.)
        self.assertEqual(r['state'],'discovery_timeout');self.assertFalse(r['duplicates'])
    def test_delayed_discovery_before_limit_ready(self):
        self.assertEqual(self.classify(elapsed=1.9)['state'],'ready')
    def test_duplicate_source_immediate(self):
        r=self.classify({'status':2,'flags':1})
        self.assertEqual(r['state'],'conflict');self.assertEqual(r['duplicates'],['status'])
    def test_extra_output_and_ev_duplicate_immediate(self):
        for outputs in ({'ev':1,'command':1},{'ev':2}):
            self.assertEqual(self.classify(outputs=outputs)['state'],'conflict')
    def test_loss_after_ready_immediate(self):
        self.assertEqual(self.classify({'status':0},ready=True)['state'],'source_lost')
    def test_generation_change_separate(self):
        self.assertEqual(self.classify(changed=['status'])['state'],'generation_changed')
    def test_own_ev_missing_blocks(self):
        self.assertEqual(self.classify(outputs={})['state'],'discovering')
    def test_stop_session_binding(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'control'
            atomic_json(p,dict(session='s',stop=False));self.assertFalse(stop_requested(p,'s'))
            request_stop(p,'s');self.assertTrue(stop_requested(p,'s'))
            with self.assertRaises(ValueError):stop_requested(p,'other')
    def test_receipt_requires_both_worker_ack_and_client_exit(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'ack';t=time.monotonic()
            atomic_json(p,dict(session='s',ros_closed=True,exit_code=0,started_at=t-1,closed_at=t))
            self.assertFalse(check_receipt(p,'s',None,t)['confirmed'])
            self.assertFalse(check_receipt(p,'other',0,t)['confirmed'])
            self.assertTrue(check_receipt(p,'s',0,t)['confirmed'])
            self.assertFalse(check_receipt(p,'s',1,t)['confirmed'])
    def test_receipt_missing_or_incomplete_never_success(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'ack';self.assertFalse(check_receipt(p,'s',0,time.monotonic())['confirmed'])
            p.write_text('{');self.assertFalse(check_receipt(p,'s',0,time.monotonic())['confirmed'])
            t=time.monotonic();atomic_json(p,dict(session='s',ros_closed=False,exit_code=0,started_at=t,closed_at=t))
            self.assertFalse(check_receipt(p,'s',0,t)['confirmed'])

if __name__=='__main__':unittest.main()
