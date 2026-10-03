import math,sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_frame_contract import SessionAlignment,shadow_setpoint_fields


class FrameContractTests(unittest.TestCase):
    def make(self,oy=0,py=0):
        return SessionAlignment([1,2,3],oy,[4,5,6],py,'session',(0,)*5,verified=True)

    def test_flu_frd_and_arbitrary_origins(self):
        a=self.make()
        np.testing.assert_allclose(a.position([2,3,4],'session',(0,)*5),[5,4,5])
        np.testing.assert_allclose(a.velocity([.1,.2,.3],'session',(0,)*5),[.1,-.2,-.3])

    def test_heading_alignment_not_north_assumption(self):
        a=self.make(math.pi/2,math.pi/4)
        np.testing.assert_allclose(a.velocity([0,.1,0],'session',(0,)*5),[.1/math.sqrt(2),.1/math.sqrt(2),0],atol=1e-10)
        self.assertAlmostEqual(a.heading(math.pi/2,'session',(0,)*5),math.pi/4)

    def test_reset_latches(self):
        for session,resets in [('new',(0,)*5),('session',(0,0,1,0,0))]:
            a=self.make()
            with self.assertRaises(ValueError):a.position([0,0,0],session,resets)
            with self.assertRaises(ValueError):a.position([0,0,0],'session',(0,)*5)

    def test_shadow_field_limits_and_ownership(self):
        args=dict(now=1,pose_at=1,command_at=1,position_odom=[1,2,3.6],
                  velocity_odom=[.1,0,0],yaw_odom=0,vio_session='session',
                  reset_counters=(0,)*5,exclusive_writer_verified=True)
        out=shadow_setpoint_fields(self.make(),**args)
        self.assertAlmostEqual(out['position'][2],5.4)
        self.assertTrue(math.isnan(out['position'][0]));self.assertFalse(out['flight_authorized'])
        for bad in [dict(command_at=0),dict(exclusive_writer_verified=False),
                    dict(velocity_odom=[.2,0,0]),dict(velocity_odom=[0,0,.1])]:
            with self.assertRaises(ValueError):shadow_setpoint_fields(self.make(),**dict(args,**bad))


if __name__=='__main__':unittest.main()
