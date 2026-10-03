"""Real serializers, pure ideal plant: not APF, SITL, hardware or flight evidence."""
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from flight_runtime import create_node
from flight_supervisor import FlightOutput
from task_flight_controller import TaskFlightSupervisor,TaskFlightCore,TaskOutput
from task_dispatch_contract import TaskDispatchContract,build_task_messages
from task_flight_release import validate_task_config
from navigation_frame_contract import SessionAlignment
from navigation_landing_shadow import Scene
from roundtrip_flight_test import RoundTripMission
from cruise_band import make_band
from test_flight_supervisor import healthy
from test_aux_switch_decoder import params


class TaskRig:
    def __init__(self, hover=1.):
        self.c=TaskFlightSupervisor(hover)
        self.g=TaskDispatchContract()
        self.now=0.;self.p=np.zeros(3);self.v=np.zeros(3)
        self.s=healthy();self.commands=set();self.trace=[]
        self.a=SessionAlignment((0,0,0),0,(2,-3,.7),.8,'unit-task',(0,)*5,verified=True)
        self.m=RoundTripMission(make_band(rise=.86,ground_clearance=.14),(-1,4,-2,2),
            lambda p,v,c:True,ground_origin=(0,0,0),yaw_odom=0,session_id='unit-task')

    def tick(self,scene_changes=None,sample_changes=None):
        self.now=round(self.now+.05,5);t=self.now
        target=self.m.planner_goal()['position']
        delta=np.array(target[:2])-self.p[:2];v=.6*delta
        norm=np.linalg.norm(v)
        if norm>.15:v*=.15/norm
        local=self.a.position(self.p,'unit-task',(0,)*5)
        lv=self.a.velocity(self.v,'unit-task',(0,)*5)
        self.s=replace(self.s,status_at=t,local_at=t,flags_at=t,land_at=t,rc_at=t,
            x=local[0],y=local[1],z=local[2],vx=lv[0],vy=lv[1],vz=lv[2])
        if sample_changes:self.s=replace(self.s,**sample_changes)
        sc=Scene(t,tuple(self.p),tuple(self.v[:2]),True,self.s.armed and not self.s.landed,
            map_at=t,footprint_known_free=True,navigation_velocity_odom=tuple(v),navigation_at=t,
            navigation_goal_id=self.m.goal_id,h_error_odom=tuple(-self.p[:2]),h_at=t,
            h_metric_verified=True,landing_boundary_verified=True,landed=self.s.landed,tilt_rad=0.)
        if scene_changes:sc=replace(sc,**scene_changes)
        self.c.context(self.m,self.a,sc,'unit-task',t)
        out=self.c.step(t,self.s,start=True,accepted_commands=self.commands)
        self.g.check(out,self.s,now=t,stamp=int((t+1)*1e6),state=self.c.state,
            origin=self.c.origin,height=self.c.height,switch_fresh=True,mode_slot=6,
            kill_switch=1 if self.s.kill_active else 3,exclusive=self.s.input_ownership_ok)
        messages=build_task_messages(out,int((t+1)*1e6))
        self.trace.append(out)
        if out.command:
            self.commands.add(out.command)
            if out.command=='OFFBOARD':self.s=replace(self.s,nav_state=14)
            if out.command=='ARM':self.s=replace(self.s,armed=True)
            if out.command=='LAND':self.s=replace(self.s,nav_state=18)
        old=self.p.copy()
        if out.stream:
            if isinstance(out,TaskOutput):
                self.p[:2]+=(self.a.rotation.T@np.array([*out.velocity_xy,0.]))[:2]*.05
            self.p[2]=.7-out.position[2]
        elif self.s.nav_state==18 and self.s.armed:
            self.p[2]=max(0.,self.p[2]-.2*.05)
        self.v=(self.p-old)/.05
        if self.p[2]>.01:self.s=replace(self.s,landed=False)
        elif self.s.nav_state==18:self.s=replace(self.s,landed=True,armed=False)
        return out,messages

    def cruise(self):
        for _ in range(400):
            out,_=self.tick()
            if out.state=='NAVIGATE':return
        raise AssertionError(self.c.reason)


class HandoffTests(unittest.TestCase):
    def test_three_second_hover_before_navigation(self):
        rig=TaskRig(hover=3.)
        first_hover=None
        for _ in range(500):
            output,_=rig.tick()
            if output.state=='HOVER' and first_hover is None:first_hover=rig.now
            if output.state=='NAVIGATE':break
        self.assertEqual(output.state,'NAVIGATE')
        self.assertIsNotNone(first_hover)
        self.assertGreaterEqual(rig.now-first_hover,3.-1e-9)

    def test_ground_no_map_no_h_takes_off_then_holds_original_position(self):
        r=TaskRig()
        missing=dict(footprint_known_free=False,map_at=-1.,navigation_at=-1.,
                     h_metric_verified=False,h_at=-1.)
        for _ in range(240):
            out,_=r.tick(scene_changes=missing)
            if r.c.map_wait_since is not None:break
        self.assertEqual(out.state,'HOVER')
        self.assertIn('ARM',r.commands)
        self.assertEqual(out.position[:2],(2,-3))
        self.assertIsNone(getattr(out,'velocity_xy',None))
        self.assertIsNone(r.c.handoff_at)
        for _ in range(20):
            out,_=r.tick(scene_changes=missing)
            self.assertEqual(out.state,'HOVER')
        # Fresh online map/path permits the handoff; H still NOT required.
        out,_=r.tick(scene_changes={'h_metric_verified':False,'h_at':-1.})
        self.assertEqual(out.state,'NAVIGATE')
        self.assertIsNotNone(out.velocity_xy)

    def test_unavailable_map_or_wrong_leg_times_out_to_original_land(self):
        for changes in ({'footprint_known_free':False},
                        {'navigation_goal_id':'stale-leg'}, {'map_at':-1.}):
            with self.subTest(changes=changes):
                r=TaskRig()
                for _ in range(600):
                    out,_=r.tick(scene_changes=changes)
                    if out.state=='LAND':break
                self.assertEqual(out.state,'LAND')
                self.assertIsNone(r.c.handoff_at)
                self.assertIn('not H landing',out.reason)
                self.assertGreaterEqual(r.now-r.c.map_wait_since,r.c.MAP_WAIT_SECONDS)
                self.assertFalse(any(isinstance(x,TaskOutput) for x in r.trace))

    def test_hover_map_wait_preserves_takeover_and_departure_guard(self):
        for changes,state in (({'manual_takeover':True},'HANDOVER'),({'x':2.31},'ABORT')):
            r=TaskRig()
            while r.c.map_wait_since is None:
                r.tick(scene_changes={'footprint_known_free':False})
            out,_=r.tick(scene_changes={'footprint_known_free':False},sample_changes=changes)
            self.assertEqual(out.state,state)

    def test_entire_original_takeoff_task_original_land(self):
        r=TaskRig()
        for _ in range(2400):
            out,m=r.tick()
            if out.state in r.c.TERMINAL:break
        self.assertEqual(out.state,'DONE',out)
        self.assertEqual(r.commands,{'OFFBOARD','ARM','LAND'})
        self.assertLess(np.linalg.norm(r.p[:2]),.05)
        self.assertEqual(r.m.leg,'RETURN')
        self.assertTrue({'TAKEOFF','HOVER','NAVIGATE','ALIGN_H','LAND','DONE'}<={x.state for x in r.trace})
        for x in r.trace:
            if x.state in ('TAKEOFF','HOVER'):
                self.assertEqual(x.position[:2],(2,-3));self.assertIsNone(getattr(x,'velocity_xy',None))
            if isinstance(x,TaskOutput):
                self.assertAlmostEqual(x.position[2],-.16);self.assertAlmostEqual(x.yaw,.8)

    def test_original_departure_guard_still_applies_during_takeoff(self):
        r=TaskRig()
        while r.c.state!='TAKEOFF':r.tick()
        out,_=r.tick(sample_changes={'x':2.31})
        self.assertEqual(out.state,'ABORT');self.assertIn('0.3',out.reason)

    def test_stale_old_leg_zero_not_motion(self):
        r=TaskRig();r.cruise()
        out,m=r.tick(scene_changes={'navigation_goal_id':'old','navigation_velocity_odom':(.15,0)})
        self.assertEqual(out.velocity_xy,(0.,0.));self.assertNotIn('vehicle_command',m)

    def test_takeover_kill_estimator_and_reset(self):
        for changes,state in (({'manual_takeover':True},'HANDOVER'),({'kill_active':True},'KILLED'),
                ({'ev_ok':False},'ABORT'),({'reset_counters':(1,0,0,0,0)},'ABORT')):
            with self.subTest(changes=changes):
                r=TaskRig();r.cruise();out,m=r.tick(sample_changes=changes)
                self.assertEqual(out.state,state)
                self.assertNotIn('trajectory_setpoint',m)
                if state in ('HANDOVER','KILLED'):self.assertFalse(m)

    def test_horizontal_mode_cannot_serialize_arm(self):
        with self.assertRaises(ValueError):
            build_task_messages(TaskOutput('NAVIGATE',True,(math.nan,math.nan,0),0.,'ARM','',(.1,0)),10)

    def test_no_navigation_without_hover_boundary(self):
        out=TaskOutput('NAVIGATE',True,(math.nan,math.nan,-.16),.8,None,'',(.1,0.))
        with self.assertRaisesRegex(ValueError,'stable-hover'):
            TaskDispatchContract().check(out,healthy(),now=1.,stamp=1,state='NAVIGATE',
                origin=(2,-3,.7,.8),height=.86,switch_fresh=True,mode_slot=6,kill_switch=3,exclusive=True)

    def test_deployment_defaults_refuse_release(self):
        cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/roundtrip_task.yaml').read_text())
        with self.assertRaises(ValueError):validate_task_config(cfg)

    def test_task_cannot_use_ordinary_live_route_or_exercised_preview(self):
        c=TaskFlightCore(0.,params(),{})
        with self.assertRaises(ValueError):create_node(c,exercise=True)
        with self.assertRaises(ValueError):create_node(c,exercise=True,task_preview=True)
        with self.assertRaises(ValueError):create_node(c,exercise=True,live_permit=object())
