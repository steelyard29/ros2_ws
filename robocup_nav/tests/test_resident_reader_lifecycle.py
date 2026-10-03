import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from resident_reader_lifecycle import ReaderLease,ReaderLeaseOwner,service_loop


class LeaseTests(unittest.TestCase):
    def setUp(self):
        self.now=100.;self.identity=dict(boot_id='boot',time_namespace='time:[1]')
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        self.path=Path(tmp.name)/'control.json'
        self.owner=ReaderLeaseOwner(self.path,'s',identity=self.identity,clock=lambda:self.now)
        self.lease=ReaderLease('s',self.identity,clock=lambda:self.now)
    def data(self):return json.loads(self.path.read_text())

    def test_continuous_renewal_has_no_bench_duration_limit(self):
        for _ in range(1001):
            self.owner.renew();self.assertTrue(self.lease.check(self.data()));self.now+=.2
        self.assertGreater(self.now,300.)

    def test_expiry_cannot_be_revived(self):
        self.owner.renew();self.assertTrue(self.lease.check(self.data()));self.now+=1.01
        self.assertFalse(self.lease.check(self.data()))
        self.owner.renew();self.assertFalse(self.lease.check(self.data()))

    def test_identity_and_replay_rejected(self):
        for field,value in (('session','other'),('clock_identity',{}),('seq',0),('renewed_at',101.)):
            lease=ReaderLease('s',self.identity,clock=lambda:self.now)
            self.owner.renew();data=self.data();data[field]=value
            self.assertFalse(lease.check(data))

    def test_expiry_during_spin_prevents_pump(self):
        self.owner.renew();pumps=[]
        def spin():self.now+=1.01
        reason=service_loop(self.lease,self.path,spin,lambda:pumps.append(1),clock=lambda:self.now)
        self.assertIn('expired',reason);self.assertEqual(pumps,[])

    def test_owner_stop_prevents_new_pump(self):
        self.owner.renew();pumps=[]
        reason=service_loop(self.lease,self.path,lambda:self.owner.renew(stop=True),
                            lambda:pumps.append(1),clock=lambda:self.now)
        self.assertEqual(reason,'owner_requested_stop');self.assertEqual(pumps,[])
        with self.assertRaises(RuntimeError):self.owner.renew()

    def test_missing_control_is_terminal(self):
        reason=service_loop(self.lease,self.path,lambda:None,lambda:None,clock=lambda:self.now)
        self.assertIn('unavailable',reason)

    def test_isolated_deadline_is_independent_of_live_lease(self):
        self.owner.renew();pumps=[]
        reason=service_loop(self.lease,self.path,lambda:None,lambda:pumps.append(1),
                            deadline=100.,clock=lambda:self.now)
        self.assertEqual(reason,'isolated_deadline');self.assertEqual(pumps,[])


if __name__=='__main__':unittest.main()
