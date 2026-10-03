import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from stopping_space import stopping_space_clear


class StoppingTests(unittest.TestCase):
    def clear(self,a,velocity=(0,0),command=(0,0),position=(1,1),**kw):
        args=dict(body_radius=.3,uncertainty=.05,latency=.3,braking=.2)
        args.update(kw)
        return stopping_space_clear(a,.05,(0,0),position,velocity,command,**args)

    def test_free_and_edge(self):
        a=np.ones((40,40),dtype=bool)
        self.assertTrue(self.clear(a))
        self.assertFalse(self.clear(a,position=(.2,1)))

    def test_current_footprint_free_but_braking_blocked(self):
        a=np.ones((40,40),dtype=bool);a[:,28]=False
        self.assertTrue(self.clear(a))
        self.assertFalse(self.clear(a,command=(.15,0)))
        self.assertFalse(self.clear(a,velocity=(.15,0)))

    def test_bad_inputs_and_unknown(self):
        a=np.ones((40,40),dtype=bool)
        self.assertFalse(self.clear(a.astype(float)))
        self.assertFalse(self.clear(a,braking=0))
        self.assertFalse(self.clear(a,velocity=(float('nan'),0)))
        a[20,20]=False
        self.assertFalse(self.clear(a))


if __name__=='__main__':unittest.main()
