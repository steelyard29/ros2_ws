import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from stopping_space import stopping_space_clear,select_stopping_safe_velocity


class SelectionTests(unittest.TestCase):
    def test_narrow_gap_reduces_speed_not_clearance(self):
        free=np.ones((120,120),dtype=bool)
        free[22:38,44:60]=False;free[62:78,44:60]=False
        def check(v,current=(0.,0.)):
            return stopping_space_clear(free,.05,(0,0),(2.65,2.478),current,v,
                body_radius=.43,uncertainty=.05,latency=.3,braking=.2)
        self.assertFalse(check((.15,0)))
        v,scale=select_stopping_safe_velocity((.15,0),check)
        self.assertEqual(scale,.75);self.assertTrue(check(v))
        # Already-fast motion cannot be declared safe by reducing just demand.
        v,scale=select_stopping_safe_velocity((.15,0),lambda v:check(v,(.15,0)))
        self.assertEqual(v,(0.,0.));self.assertEqual(scale,0.)
        self.assertFalse(check(v,(.15,0)))

    def test_unknown_space_stays_inhibited(self):
        self.assertEqual(select_stopping_safe_velocity((.1,0),lambda v:False),((0.,0.),0.))
        with self.assertRaises(ValueError):select_stopping_safe_velocity((.2,0),lambda v:True)


if __name__=='__main__':unittest.main()
