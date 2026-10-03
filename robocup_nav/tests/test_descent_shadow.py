import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from descent_shadow import DescentObservation,DescentShadow


class DescentTests(unittest.TestCase):
    def sample(self,t,h=.6,**kw):
        values=dict(at=t,clearance_m=h,center_error_m=.01,xy_speed_mps=0,
             localization_ok=True,same_session=True,landing_column_clear=True,
             bounds_verified=True,h_visible=True,h_at=t,landing_anchor_verified=True)
        values.update(kw);return DescentObservation(**values)

    def test_synthetic_visible_then_terminal_then_touchdown(self):
        m=DescentShadow(.04);height=.6
        for i in range(600):
            t=i*.05
            landed=height<=0
            out=m.step(t,self.sample(t,height,h_visible=height>.20,landed=landed,armed=not landed),begin=i==0)
            height=max(0.,height+out['vertical_velocity_odom']*.05)
            self.assertFalse(out['flight_authorized']);self.assertIsNone(out['vehicle_command'])
            if out['state']=='DONE':break
        self.assertEqual(out['state'],'DONE')

    def test_loss_above_terminal_holds_and_cannot_commit_blind(self):
        m=DescentShadow(.04);m.step(0,self.sample(0),begin=True)
        self.assertEqual(m.step(.1,self.sample(.1,h_visible=False))['vertical_velocity_odom'],0)
        self.assertEqual(m.step(.2,self.sample(.2,.24,h_visible=False))['vertical_velocity_odom'],0)

    def test_reset_bounds_and_drift_abort(self):
        for fault in (dict(same_session=False),dict(bounds_verified=False),dict(center_error_m=.05),dict(landing_column_clear=False)):
            m=DescentShadow(.04);m.step(0,self.sample(0),begin=True)
            self.assertEqual(m.step(.1,self.sample(.1,**fault))['state'],'ABORT')
            self.assertEqual(m.step(.2,self.sample(.2))['state'],'ABORT')

    def test_touchdown_without_disarm_never_done(self):
        m=DescentShadow(.04);m.step(0,self.sample(0),begin=True)
        for i in range(1,51):out=m.step(i*.1,self.sample(i*.1,0,landed=True,armed=True))
        self.assertEqual(out['state'],'TOUCHDOWN')
        self.assertEqual(out['vertical_velocity_odom'],0)

    def test_no_assumed_entry_and_manual_override(self):
        m=DescentShadow(.04)
        self.assertEqual(m.step(0,self.sample(0,bounds_verified=False),begin=True)['state'],'WAIT')
        self.assertEqual(m.step(.1,self.sample(.1,manual_override=True))['state'],'HANDOVER')

    def test_anchor_requires_explicit_mode_and_independent_model(self):
        valid=dict(h_visible=False,anchor_at=0.,additional_anchor_error_m=.001,
                   anchor_drift_model_verified=True)
        for enabled,changes in ((False,{}),(True,dict(anchor_drift_model_verified=False)),
                (True,dict(additional_anchor_error_m=float('nan'))),
                (True,dict(additional_anchor_error_m=-.001)),
                (True,dict(additional_anchor_error_m=.04)),(True,dict(anchor_at=1.))):
            m=DescentShadow(.04,allow_verified_anchor=enabled)
            m.step(0,self.sample(0),begin=True)
            r=m.step(.1,self.sample(.1,**{**valid,**changes}))
            self.assertEqual(r['vertical_velocity_odom'],0,r)

    def test_verified_anchor_descends_then_holds_on_budget_expiry(self):
        m=DescentShadow(.04,allow_verified_anchor=True,max_anchor_age_s=.3)
        m.step(0,self.sample(0),begin=True)
        fields=dict(h_visible=False,anchor_at=0.,additional_anchor_error_m=.001,
                    anchor_drift_model_verified=True)
        self.assertLess(m.step(.1,self.sample(.1,**fields))['vertical_velocity_odom'],0)
        m.step(.2,self.sample(.2,**fields))
        self.assertEqual(m.step(.4,self.sample(.4,**fields))['vertical_velocity_odom'],0)

    def test_anchor_expiry_in_terminal_is_latched(self):
        m=DescentShadow(.04,allow_verified_anchor=True,max_anchor_age_s=.3)
        m.step(0,self.sample(0,.3),begin=True)
        fields=dict(h_visible=False,anchor_at=0.,additional_anchor_error_m=.001,
                    anchor_drift_model_verified=True)
        self.assertEqual(m.step(.1,self.sample(.1,.24,**fields))['state'],'TERMINAL_DESCEND')
        m.step(.2,self.sample(.2,.23,**fields))
        self.assertEqual(m.step(.4,self.sample(.4,.22,**fields))['state'],'ABORT')
        self.assertEqual(m.step(.5,self.sample(.5,.21))['state'],'ABORT')

    def test_anchor_cannot_substitute_initial_h_acquisition(self):
        m=DescentShadow(.04,allow_verified_anchor=True)
        r=m.step(0,self.sample(0,h_visible=False,anchor_at=0.,additional_anchor_error_m=0.,
                               anchor_drift_model_verified=True),begin=True)
        self.assertEqual(r['state'],'WAIT')


if __name__=='__main__':unittest.main()
