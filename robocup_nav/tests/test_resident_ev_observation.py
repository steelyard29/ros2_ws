import sys
from pathlib import Path
from types import SimpleNamespace as N
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from resident_ev_observation import ResidentObservation


class ObservationTests(unittest.TestCase):
    def test_pair_uses_source_time_and_rejects_armed_or_old(self):
        o=ResidentObservation('test')
        local=N(timestamp=1_000_000,xy_valid=True,z_valid=True,x=0,y=0,z=0,heading=0,
            xy_reset_counter=0,z_reset_counter=0,heading_reset_counter=0,
            vxy_reset_counter=0,vz_reset_counter=0)
        pose=N(header=N(stamp=N(sec=1,nanosec=0)),pose=N(pose=N(
            position=N(x=0,y=0,z=0),orientation=N(x=0,y=0,z=0,w=1))))
        o.local_message(local,1.,False,True);o.pose(pose,1.,1_000_000_000)
        self.assertEqual(o.report()['matched_retained'],1)
        o.pose(pose,1.4,1_400_000_000)
        self.assertEqual(len(o.pairs),1)
        o.locals.clear();o.local_message(local,1.,True,True)
        o.pose(pose,1.,1_000_000_000);self.assertEqual(len(o.pairs),1)

    def test_fusion_is_measured_not_inferred_from_send(self):
        o=ResidentObservation('test')
        for i in range(3):
            m=N(timestamp=(i+1)*1_000_000,cs_ev_pos=True,cs_ev_yaw=True,
                cs_ev_hgt=i!=1,cs_ev_vel=False,cs_baro_hgt=True,cs_rng_hgt=False)
            o.flags_message(m,i+1.,(i+1)*1_000_000_000)
        r=o.report()
        self.assertEqual(r['fusion']['fully_fused_samples'],2)
        self.assertEqual(r['fusion']['longest_sampled_span_s'],0)
        self.assertFalse(r['flight_ready'])
