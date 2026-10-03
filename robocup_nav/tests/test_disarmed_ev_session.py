"""No hardware/ROS nodes. Synthetic positive and rejection controls."""
import ast
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from disarmed_ev_contract import DisarmedEvSession


class Rig:
    def __init__(self):
        self.s=DisarmedEvSession(0,50,'test-session');self.now=0.;self.seq=0;self.outputs=[]
    def packet(self,kind,**kw):
        self.seq+=1
        d=dict(session='test-session',seq=self.seq,kind=kind,stamp_ns=int((100+self.now)*1e9))
        d.update(kw);return d
    def step(self,*,armed=False,nav=2,landed=True,rc=True,track=1,owner=True,position=(0,0,0)):
        self.now=round(self.now+.05,6);n=self.now;s=self.s;c=s.core;stamp=int((n+100)*1e6)
        c.ownership(n,owner)
        s.telemetry('status',stamp,0,n,lambda:c.status(2 if armed else 1,n,nav_state=nav,
            preflight_ok=True,failsafe=False))
        s.telemetry('flags',stamp,0,n,lambda:c.flags(n,ev_position=True,ev_yaw=True,
            ev_height=True,ev_velocity=False,baro_height=True,range_height=False))
        s.telemetry('land',stamp,0,n,lambda:c.land(n,landed))
        if rc:
            s.telemetry('rc',stamp,0,n,lambda:c.rc(n,True));c.switches(n,1,3)
        ns=int((n+100)*1e9)
        s.packet(self.packet('tracking',state=track),n,ns)
        ev=s.packet(self.packet('pose',frame='odom',child='base_link',position=position,
                    quaternion=[0,0,0,1]),n,ns)
        if ev is not None:
            s.dispatched();self.outputs.append(ev)
        return ev
    def ready(self):
        for _ in range(45):self.step()
        assert self.outputs, self.s.fault


class Tests(unittest.TestCase):
    def test_positive_ev_no_velocity_control(self):
        r=Rig();r.ready()
        self.assertEqual(r.s.core.controller.state,'WAIT')
        self.assertTrue(all(math.isnan(x) for x in r.outputs[-1].velocity))
        self.assertFalse(r.s.core.start_consumed)
    def test_arm_latches_and_disarm_does_not_resume(self):
        r=Rig();r.ready();count=len(r.outputs);r.step(armed=True)
        for _ in range(5):r.step()
        self.assertEqual(len(r.outputs),count);self.assertIsNotNone(r.s.fault)
    def test_missing_rc_never_outputs(self):
        r=Rig()
        for _ in range(205):r.step(rc=False)
        self.assertFalse(r.outputs);self.assertIsNotNone(r.s.fault)
    def test_mode_change_stops(self):
        r=Rig();r.ready();self.assertIsNone(r.step(nav=14));self.assertIsNotNone(r.s.fault)
    def test_not_landed_stops(self):
        r=Rig();r.ready();self.assertIsNone(r.step(landed=False));self.assertIsNotNone(r.s.fault)
    def test_tracking_failure_stops(self):
        r=Rig();r.ready();self.assertIsNone(r.step(track=2));self.assertIsNotNone(r.s.fault)
    def test_ownership_conflict_stops(self):
        r=Rig();r.ready();self.assertIsNone(r.step(owner=False));self.assertIsNotNone(r.s.fault)
    def test_jump_stops(self):
        r=Rig();r.ready();self.assertIsNone(r.step(position=(1,0,0)));self.assertIsNotNone(r.s.fault)
    def test_sensor_loss_latches(self):
        r=Rig();r.ready()
        self.assertTrue(r.s.watchdog(r.now+.35))
        later=r.now+1.01
        for name in ('status','flags','land','rc'):
            r.s.receipts[name]=later
        self.assertFalse(r.s.watchdog(later))
        self.assertIn('stale', r.s.fault)
    def test_sample_clock_after_ownership_refresh_keeps_exclusive_output(self):
        r=Rig();r.ready()
        audited=r.now+.01
        r.s.core.ownership(audited,True)
        self.assertFalse(r.s.core._owns(r.now))
        self.assertTrue(r.s.core._owns(audited))
    def test_fresh_pose_is_applied_before_idle_watchdog(self):
        r=Rig();r.ready();base=r.now
        r.now=round(base+.35,6)
        stamp=int((base+100+.04)*1e9);ns=stamp+20_000_000
        r.s.packet(r.packet('tracking',state=1,stamp_ns=stamp),r.now,ns)
        ev=r.s.packet(r.packet('pose',frame='odom',child='base_link',position=(0,0,0),
            quaternion=[0,0,0,1],stamp_ns=stamp),r.now,ns)
        self.assertIsNotNone(ev)
        self.assertIsNone(r.s.fault)
        self.assertTrue(r.s.watchdog(r.now))
    def test_source_age_and_session_rejected(self):
        for change in ({'stamp_ns':1},{'session':'old'},{'seq':999},{'stamp_ns':float('nan')},
                       {'frame':'camera_link'},{'kind':'command'}):
            r=Rig();r.ready();d=r.packet('pose',frame='odom',child='base_link',
                position=[0,0,0],quaternion=[0,0,0,1]);d.update(change)
            self.assertIsNone(r.s.packet(d,r.now,int((100+r.now)*1e9)))
            self.assertIsNotNone(r.s.fault)
    def test_duplicate_telemetry_never_renews(self):
        r=Rig();r.ready();before=r.s.receipts['status']
        self.assertFalse(r.s.telemetry('status',r.s.core.source_stamps['status'],0,r.now+.1,
                         lambda:self.fail('duplicate callback invoked')))
        self.assertEqual(before,r.s.receipts['status'])
    def test_old_telemetry_latches(self):
        r=Rig();r.ready()
        self.assertFalse(r.s.telemetry('status',999999999,.51,r.now,lambda:None))
        self.assertIsNotNone(r.s.fault)
    def test_deadline_no_resume(self):
        r=Rig();r.ready();self.assertFalse(r.s.watchdog(50))
        self.assertFalse(r.s.watchdog(r.now));self.assertIsNotNone(r.s.fault)
    def test_budget_bounds(self):
        for end in (0,51,float('inf'),float('nan')):
            with self.assertRaises(ValueError):DisarmedEvSession(0,end,'x')
    def test_describe_default_has_no_ros_import_or_devices(self):
        p=subprocess.run([sys.executable,str(ROOT/'scripts/disarmed_ev_session.py')],
                         capture_output=True,text=True,timeout=3)
        self.assertEqual(p.returncode,0,p.stderr)
        self.assertEqual(json.loads(p.stdout)['real_output_allowlist'],['/fmu/in/vehicle_visual_odometry'])
    def test_execute_without_confirmation_rejected_before_worker(self):
        p=subprocess.run([sys.executable,str(ROOT/'scripts/disarmed_ev_session.py'),'--execute-ev-only'],
                         capture_output=True,text=True,timeout=3)
        self.assertEqual(p.returncode,2)
    def test_single_output_api_and_reader_readonly(self):
        s=(ROOT/'scripts/disarmed_ev_session.py').read_text();t=ast.parse(s)
        publishers=[n for n in ast.walk(t) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
                    and n.func.attr=='create_publisher']
        self.assertEqual(len(publishers),1)
        self.assertEqual(publishers[0].args[0].id,'VehicleOdometry')
        for name in ('VehicleCommand','TrajectorySetpoint','OffboardControlMode','create_client','core.tick('):
            self.assertNotIn(name,s)
        reader=(ROOT/'scripts/disarmed_ev_sensor_reader.py').read_text()
        self.assertNotIn('create_publisher',reader);self.assertNotIn('px4_msgs',reader)

    def test_process_supervisor_kills_only_owned_hung_child(self):
        from disarmed_ev_session import supervise
        start=time.monotonic();pid=os.fork()
        if pid==0:
            os.setsid();time.sleep(10);os._exit(99)
        self.assertEqual(supervise(pid,start+.15),2)
        self.assertLess(time.monotonic()-start,1.)
        with self.assertRaises(ChildProcessError):os.waitpid(pid,os.WNOHANG)

    def test_process_supervisor_normal_exit(self):
        from disarmed_ev_session import supervise
        pid=os.fork()
        if pid==0:os.setsid();os._exit(0)
        self.assertEqual(supervise(pid,time.monotonic()+1),0)

    def test_safety_telemetry_stale_even_when_vision_refreshes(self):
        r=Rig();r.ready();r.s.receipts['status']=r.now-1.51
        self.assertFalse(r.s.watchdog(r.now));self.assertIn('telemetry',r.s.fault)

    def test_reader_failure_and_wrong_quaternion_latch(self):
        for kind,fields in [('fault',dict(reason='source restarted')),
                ('pose',dict(frame='odom',child='base_link',position=[0,0,0],quaternion=[0,0,0,0]))]:
            r=Rig();r.ready()
            self.assertIsNone(r.s.packet(r.packet(kind,**fields),r.now,int((100+r.now)*1e9)))
            self.assertIsNotNone(r.s.fault)


if __name__=='__main__':unittest.main()
