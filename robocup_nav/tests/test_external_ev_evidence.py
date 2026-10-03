import sys
from pathlib import Path
import unittest
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from external_ev_evidence import ExternalEvEvidence
from flight_runtime_core import FlightRuntimeCore


class ExternalTests(unittest.TestCase):
    def evidence(self):return ExternalEvEvidence('session',(1,2))
    def recv(self,e,stamp=1_000_000,now=1.,gid=(1,2),quality=100):
        return e.receive('session',gid,stamp,stamp,quality,stamp*1000,now)
    def test_actual_receipt_required_not_local_candidate(self):
        c=FlightRuntimeCore(0.);e=self.evidence();c.bind_external_ev(e)
        c.ownership(1.,True);c.ev.on_pose=Mock(return_value=object())
        self.assertIsNone(c.pose(1,0,1.,[0,0,0],[0,0,0,1],True,1))
        self.assertEqual(c.last_ev_at,-1.)
        self.assertFalse(c.ev_receipt_recent(1.))
        self.assertTrue(c.external_ev_received('session',(1,2),1_000_000,1_000_000,100,1_000_000_000,1.))
        self.assertTrue(c.ev_receipt_recent(1.1))
    def test_duplicate_does_not_renew_then_silence_latches(self):
        e=self.evidence();self.assertTrue(self.recv(e))
        self.assertFalse(self.recv(e,now=1.2));self.assertEqual(e.last_at,1.)
        self.assertFalse(e.healthy(1.31));self.assertFalse(self.recv(e,1_100_000,1.32))
    def test_identity_change_latches(self):
        e=self.evidence();self.assertFalse(self.recv(e,gid=(9,)))
        self.assertFalse(self.recv(e))
    def test_invalid_quality_and_future_time(self):
        self.assertFalse(self.recv(self.evidence(),quality=49))
        e=self.evidence()
        self.assertFalse(e.receive('session',(1,2),3_000_000,3_000_000,100,1_000_000_000,1.))
    def test_source_gap_rejected(self):
        e=self.evidence();self.recv(e)
        self.assertFalse(self.recv(e,1_400_000,1.4))
    def test_cannot_rebind(self):
        c=FlightRuntimeCore(0.);c.bind_external_ev(self.evidence())
        with self.assertRaises(ValueError):c.bind_external_ev(self.evidence())
    def test_unintegrated_ros_route_rejects_before_ros_import(self):
        from flight_runtime import create_node
        c=FlightRuntimeCore(0.);c.bind_external_ev(self.evidence())
        with self.assertRaisesRegex(ValueError,'not integrated'):create_node(c)


if __name__=='__main__':unittest.main()
