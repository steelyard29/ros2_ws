import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from resident_ev_core import ResidentEvCore


class Rig:
    def __init__(self):
        self.c=ResidentEvCore(0.,'synthetic-session');self.t=0.
        self.ids={k:(i,) for i,k in enumerate(('vio','tracking','status','flags','land'),1)}
    def step(self,armed=False,send=True):
        self.t+=1/30;stamp=int((100+self.t)*1e9)
        c=self.c;c.ownership(self.ids,1,self.t)
        c.vehicle(2 if armed else 1,not armed,self.t)
        c.flags(True,False,self.t);c.tracking(1,stamp,self.t)
        ev=c.pose(stamp,stamp+10_000_000,self.t,[0,0,0],[0,0,0,1])
        if ev and send:c.dispatched(ev.timestamp_sample,self.t)
        c.watchdog(self.t)
        return ev


class ResidentTests(unittest.TestCase):
    def test_continuous_session_beyond_bench_limit_and_through_landing(self):
        r=Rig()
        for _ in range(60):r.step()
        for _ in range(1800):r.step(armed=True)
        for _ in range(60):r.step()
        self.assertGreater(r.t,60);self.assertIsNone(r.c.fault)
        self.assertTrue(r.c.health(r.t)['recent_dispatch'])
        self.assertFalse(r.c.health(r.t)['flight_ready'])
    def test_start_airborne_rejected(self):
        r=Rig();r.step(armed=True);self.assertIsNotNone(r.c.fault)
        self.assertIsNone(r.step())
    def test_generator_without_send_is_not_healthy(self):
        r=Rig()
        for _ in range(30):r.step(send=False)
        self.assertEqual(r.c.sent_count,0)
        self.assertFalse(r.c.health(r.t)['recent_dispatch'])
        r.step(armed=True);self.assertIsNotNone(r.c.fault)
    def test_source_change_latches(self):
        r=Rig()
        for _ in range(35):r.step()
        count=r.c.sent_count;r.ids['vio']=(99,);r.step()
        r.ids['vio']=(1,);r.step()
        self.assertEqual(r.c.sent_count,count);self.assertIsNotNone(r.c.fault)
    def test_silence_no_automatic_resume(self):
        r=Rig()
        for _ in range(35):r.step()
        r.c.watchdog(r.t+.31)
        self.assertIsNotNone(r.c.fault);self.assertIsNone(r.step())
    def test_future_sample_not_rewritten(self):
        r=Rig();r.step();stamp=int((100+r.t)*1e9)
        self.assertIsNone(r.c.pose(stamp+2_000_000_000,stamp,r.t,[0,0,0],[0,0,0,1]))
        self.assertIn('age',r.c.fault)
    def test_duplicate_owner_latches(self):
        r=Rig();self.assertFalse(r.c.ownership(r.ids,2,0.))
        self.assertFalse(r.c.ownership(r.ids,1,0.))


if __name__=='__main__':unittest.main()
