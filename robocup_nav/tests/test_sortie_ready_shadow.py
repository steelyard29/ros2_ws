"""Production readiness policy feeding synthetic full mission, without ROS."""
import sys
from pathlib import Path
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from sortie_ready_shadow import ReadySortieShadow
from test_sortie_shadow import Rig
from test_aux_switch_decoder import params


class Hook:
    def __init__(self, sortie):
        self.ready = ReadySortieShadow(sortie, 0., params())
        self.mode = None
        self.kill = False
        self.vision = True
        self.acknowledge = True

    def __getattr__(self,name):
        return getattr(self.ready.sortie,name)

    def step(self, now, sample, scene, landing, space, **kw):
        p = self.ready.policy
        p.ownership(now,sample.input_ownership_ok)
        p.gcs_status(True,now)
        p.status(2 if sample.armed else 1,now,nav_state=sample.nav_state,
                 preflight_ok=sample.preflight_ok,failsafe=sample.failsafe,
                 manual_takeover=sample.manual_takeover)
        p.local(now, **{k:getattr(sample,k) for k in ('x','y','z','heading','vx','vy','vz',
                  'xy_valid','z_valid','v_xy_valid','v_z_valid','reset_counters')})
        p.flags(now, **{k:getattr(sample,k) for k in ('ev_position','ev_yaw','ev_height',
                   'ev_velocity','baro_height','range_height')})
        p.land(now,sample.landed)
        stamp = kw['timestamp_us']
        slot = self.mode if self.mode is not None else (1 if p.ready_at is None else 6)
        p.aux_message(stamp,stamp-1000,now,0.,True,1,-1. if slot==1 else 1.,1. if self.kill else -1.)
        if self.vision:
            p.tracking(1,stamp*1000,now)
            p.pose(stamp*1000,0.,now,scene.position,(0.,0.,0.,1.),True,stamp*1000)
        result = self.ready.tick(now,scene,landing,space,timestamp_us=stamp,vio_session=kw['vio_session'])
        self.ready.sent(result)  # Unit fixture accepts batch; DDS writer tested separately.
        msg = result['messages'].get('vehicle_command')
        if msg and self.acknowledge:
            p.ack({176:'OFFBOARD',400:'ARM'}[int(msg.command)],0,stamp+1,now)
        return result


class ReadySortieTests(unittest.TestCase):
    def rig(self):
        r=Rig(); r.core=Hook(r.core)
        return r

    def test_full_mission_waits_for_ready_and_ends_without_rearm(self):
        r=self.rig()
        for _ in range(1400):
            result=r.tick()
            if result['state'] in r.core.TERMINAL:
                break
        self.assertEqual(result['state'],'DONE',r.core.ready.report())
        p=r.core.ready.policy
        self.assertGreaterEqual(p.ready_at,5.)
        self.assertEqual(p.accepted_commands,{'OFFBOARD','ARM'})
        self.assertGreater(p.ev.publish_count,30)
        self.assertFalse(r.tick()['messages'])

    def test_no_start_with_early_switch_or_unverified_column(self):
        for kind in ('early','column'):
            r=self.rig()
            if kind=='early':r.core.mode=6
            for _ in range(180):
                result=r.tick(space_ok=kind!='column')
                self.assertFalse(result['messages'])
            self.assertFalse(r.core.ready.policy.arm_requested)

    def test_no_mode_ack_never_arms(self):
        r=self.rig();r.core.acknowledge=False
        for _ in range(350):r.tick()
        self.assertFalse(r.core.ready.policy.arm_requested)
        self.assertEqual(r.core.state,'ABORT')

    def test_mission_vision_loss_and_takeover_are_latched(self):
        for fault in ('vision','takeover','kill'):
            r=self.rig()
            for _ in range(500):
                if r.tick()['state']=='NAVIGATE':break
            self.assertEqual(r.core.state,'NAVIGATE',r.core.ready.report())
            if fault=='vision':r.core.vision=False
            if fault=='takeover':r.core.mode=1
            if fault=='kill':r.core.kill=True
            for _ in range(10):result=r.tick()
            self.assertIn(result['state'],{'ABORT','HANDOVER','KILLED'},r.core.ready.report())
            r.core.vision=True;r.core.mode=6;r.core.kill=False
            for _ in range(10):self.assertFalse(r.tick()['messages'])

    def test_airborne_unexpected_disarm_is_not_contact_exception(self):
        r=self.rig()
        for _ in range(500):
            if r.tick()['state']=='NAVIGATE':break
        self.assertEqual(r.core.state,'NAVIGATE')
        r.armed=False
        result=r.tick()
        self.assertEqual(result['state'],'ABORT')
        self.assertFalse(result['messages'])
        r.armed=True
        self.assertFalse(r.tick()['messages'])

    def test_no_ready_without_scene_and_closed_policy_never_replays(self):
        r=self.rig()
        self.assertFalse(r.core.ready.ground_context_ok(0.))
        for _ in range(500):
            if r.tick()['state']=='NAVIGATE':break
        self.assertEqual(r.core.state,'NAVIGATE')
        r.core.ready.policy.close('test close')
        for _ in range(10):self.assertFalse(r.tick()['messages'])


if __name__=='__main__':unittest.main()
