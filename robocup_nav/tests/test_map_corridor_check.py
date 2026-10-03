import sys,unittest,tempfile
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from map_corridor_check import audit


class MapTests(unittest.TestCase):
    def run_map(self,a,start=(1.2,2),goal=(3.5,2)):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'map.npz'
            np.savez(p,data=a,width=a.shape[1],height=a.shape[0],resolution=.05,origin=[0,0,0],unknown=1000.)
            return audit(p,start,goal)

    def test_open_and_sealed(self):
        a=np.ones((80,100))
        self.assertTrue(self.run_map(a)['path_found'])
        a[:,50]=0
        self.assertFalse(self.run_map(a)['path_found'])

    def test_unknown_start_not_cleared(self):
        a=np.ones((80,100));a[:,:30]=1000
        r=self.run_map(a)
        self.assertFalse(r['start_valid']);self.assertFalse(r['path_found'])

    def test_narrow_gap_not_centerline_only(self):
        a=np.ones((80,100));a[:,50]=0;a[35:46,50]=1
        self.assertFalse(self.run_map(a)['path_found'])


if __name__=='__main__':unittest.main()
