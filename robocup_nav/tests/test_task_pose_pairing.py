import sys
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from task_pose_pairing import closest_local


class PairTests(unittest.TestCase):
    def sample(self,at=10.,resets=(0,0,0,0,0),x=0.):
        return NS(local_at=at,reset_counters=resets,x=x,xy_valid=True,z_valid=True,armed=False,landed=True)

    def test_delayed_vio_selects_historical_not_latest_pose(self):
        old=self.sample(9.90,x=1.);latest=self.sample(10.,x=2.)
        pair=closest_local(9_900_000_000,[(9_900_000_000,old),(10_000_000_000,latest)],
                           10_000_000_000,10.,latest.reset_counters)
        self.assertIs(pair[1],old)
        self.assertEqual(pair[0],9_900_000_000)

    def test_source_age_receipt_age_reset_and_skew_not_relaxed(self):
        for stamp,sample in ((9_700_000_000,self.sample()),(9_900_000_000,self.sample(9.6)),
            (9_900_000_000,self.sample(resets=(1,0,0,0,0))),
            (9_960_000_001,self.sample()),(10_000_000_001,self.sample())):
            self.assertIsNone(closest_local(9_900_000_000,[(stamp,sample)],
                10_000_000_000,10.,(0,0,0,0,0)))

    def test_missing_history_does_not_substitute_latest_sample(self):
        self.assertIsNone(closest_local(10_000_000_000,[],10_000_000_000,10.,(0,)*5))

    def test_invalid_or_airborne_history_not_ground_alignment(self):
        for key,value in (('xy_valid',False),('z_valid',False),('armed',True),('landed',False)):
            sample=self.sample();setattr(sample,key,value)
            self.assertIsNone(closest_local(10_000_000_000,[(10_000_000_000,sample)],
                10_000_000_000,10.,(0,)*5))


if __name__=='__main__':unittest.main()
