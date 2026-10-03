import sys,unittest,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from cruise_band import make_band
from navigation_landing_shadow import MissionShadow,Scene
from navigation_frame_contract import SessionAlignment
from navigation_shadow_pipeline import ShadowPipeline


class PipelineTests(unittest.TestCase):
    def make(self):
        # Explicit synthetic clearance callback; not real observed geometry.
        return ShadowPipeline(MissionShadow(make_band(),(-2,2,-2,2),lambda p,v,c:True),
            SessionAlignment([0,0,0],0,[4,5,6],0,'s',(0,)*5,verified=True),0.)

    def tick(self,p,t,scene=None,**kw):
        fields=dict(at=t,position=(0,0,.6),tilt_rad=0.,localization_ok=True,
                    airborne=True,map_at=t,footprint_known_free=True,navigation_at=t,
                    navigation_velocity_odom=(.1,.02))
        fields.update(scene or {})
        args=dict(timestamp_us=int((t+1)*1e6),vio_session='s',reset_counters=(0,)*5,
                  exclusive_writer_verified=True)
        args.update(kw)
        return p.step(t,Scene(**fields),**args)

    def test_navigation_altitude_not_current_pose_and_stale_command_stops(self):
        p=self.make();r=self.tick(p,0,dict(position=(0,0,.64)))
        sp=r['messages']['trajectory_setpoint']
        self.assertAlmostEqual(sp.position[2],5.4,places=5)
        self.assertAlmostEqual(sp.velocity[1],-.02,places=6)
        r=self.tick(p,.1,dict(navigation_at=-1),yaw_rate_odom=.2)
        sp=r['messages']['trajectory_setpoint']
        self.assertEqual(list(sp.velocity[:2]),[0,0]);self.assertEqual(sp.yaw,0.)

    def test_ground_takeover_and_localization_fault_do_not_generate_messages(self):
        for scene in (dict(airborne=False),dict(manual_override=True),dict(localization_ok=False)):
            self.assertFalse(self.tick(self.make(),0,scene)['messages'])

    def test_ownership_and_alignment_faults_latch(self):
        for fault in (dict(exclusive_writer_verified=False),dict(vio_session='new'),
                      dict(reset_counters=(0,0,0,0,1))):
            p=self.make();self.assertFalse(self.tick(p,0,**fault)['messages'])
            self.assertFalse(self.tick(p,.1)['messages'])

    def test_yaw_roundoff_clamped_but_real_overlimit_latches(self):
        for rate in (.20000000000000007,-.20000000000000007):
            p=self.make()
            self.assertTrue(self.tick(p,0,yaw_rate_odom=rate)['messages'])
            self.assertTrue(self.tick(p,.1,yaw_rate_odom=rate)['messages'])
            self.assertLessEqual(abs(p.yaw),.020000000001)
        for rate in (.200001,-.200001,float('nan'),float('inf')):
            p=self.make()
            self.assertEqual(self.tick(p,0,yaw_rate_odom=rate)['fault'],'invalid yaw rate')
            self.assertFalse(self.tick(p,.1)['messages'])

    def test_pending_land_never_sends_land_or_descends(self):
        p=self.make()
        for i in range(25):
            t=i*.1
            r=self.tick(p,t,dict(goal_reached=True,h_metric_verified=True,h_at=t,landing_boundary_verified=True))
        self.assertEqual(r['intention']['state'],'LAND_REQUEST')
        self.assertNotIn('vehicle_command',r['messages'])
        self.assertAlmostEqual(r['messages']['trajectory_setpoint'].position[2],5.4,places=5)
        self.assertFalse(r['flight_authorized'])


if __name__=='__main__':unittest.main()
