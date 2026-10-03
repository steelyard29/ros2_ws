import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from landing_envelope import containment


class EnvelopeTests(unittest.TestCase):
    def call(self,error=(0,0),**kw):
        args=dict(target_error_bound=.01,localization_error_bound=.02,
                  descent_drift_bound=.02,geometry_verified=True)
        args.update(kw)
        return containment([[-.16,-.16],[-.16,.16],[.16,-.16],[.16,.16]],error,.6,**args)

    def test_center_error_budget_not_fixed_five_cm(self):
        self.assertTrue(self.call()['contained'])
        self.assertLess(self.call()['allowed_center_error_m'],.025)
        self.assertFalse(self.call((.05,0))['contained'])

    def test_unverified_or_uncertain_blocks(self):
        self.assertFalse(self.call(geometry_verified=False)['contained'])
        self.assertFalse(self.call(descent_drift_bound=.2)['contained'])
        self.assertFalse(self.call(target_error_bound=-1)['contained'])
        self.assertFalse(self.call(error=(float('nan'),0))['contained'])

    def test_degenerate_geometry_rejected(self):
        self.assertFalse(containment([[0,0]]*4,[0,0],.6,target_error_bound=0,
            localization_error_bound=0,descent_drift_bound=0,geometry_verified=True)['contained'])


if __name__=='__main__':unittest.main()
