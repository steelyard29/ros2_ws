"""DDS telemetry/READY/AUX/EV -> ground sortie -> A*/APF -> H -> contact.

Uses real message schemas and production readiness policy, synthetic sensors,
perfect map/VIO and hypothetical dynamics. Never real PX4 SITL or flight.
"""
import hashlib
import json
import math
import time
from dataclasses import replace
import numpy as np
from sortie_h_closed_loop import SortieFixture, main, ROOT
from sortie_ready_shadow import ReadySortieShadow
from sortie_input_shadow import SortieInputShadow
from sortie_shadow import TakeoffSpace
from landing_sequence_shadow import LandingEvidence
from px4_msgs.msg import (VehicleStatus,VehicleLocalPosition,EstimatorStatusFlags,
    VehicleLandDetected,ManualControlSetpoint,VehicleCommandAck,VehicleOdometry)
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def fixture_aux_params():
    return {'RC_MAP_AUX1':6,'RC_MAP_AUX2':5,'RC_MAP_FLTMODE':6,'RC_MAP_KILL_SW':5,
        'RC_MAP_OFFB_SW':0,'RC_MAP_FLTM_BTN':0,'RC_KILLSWITCH_TH':.75,'COM_RC_IN_MODE':0,
        **{f'COM_FLTMODE{i}':2 if i<6 else 7 for i in range(1,7)}}


class ReadyFixture(SortieFixture):
    # Observe the added preflight warmup/READY phase as well as the mission.
    # This does not change SortieShadow's 120s active deadline or any freshness
    # limit. Retain the prior 85s incomplete result; no retroactive pass.
    test_observation_seconds=110.

    def source_hashes(self):
        result=super().source_hashes()
        for name in ('scripts/sortie_ready_shadow.py','scripts/sortie_input_shadow.py',
                     'scripts/live_flight_core.py','scripts/flight_runtime_core.py',
                     'scripts/flight_ev_adapter.py','scripts/aux_switch_decoder.py',
                     'tests/sortie_ready_h_closed_loop.py'):
            result[name]=hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
        return result

    def __init__(self,controller):
        super().__init__(controller)
        self.ready=ReadySortieShadow(self.sortie,time.monotonic(),fixture_aux_params())
        self.receiver=SortieInputShadow(self,self.ready,self.writer)
        classes=[(VehicleStatus,'vehicle_status_v1'),(VehicleLocalPosition,'vehicle_local_position'),
            (EstimatorStatusFlags,'estimator_status_flags'),(VehicleLandDetected,'vehicle_land_detected'),
            (ManualControlSetpoint,'manual_control_setpoint'),(VehicleCommandAck,'vehicle_command_ack'),
            (Odometry,'vio'),(String,'tracking')]
        self.input_pubs={name:self.create_publisher(cls,self.receiver.INPUT+name,10) for cls,name in classes}
        self.ev_received=0
        self.create_subscription(VehicleOdometry,self.receiver.EV,self.ev_message,10)
        self.telemetry_timer=self.create_timer(.02,self.publish_inputs)
        self.phase_reports=[]

    def ev_message(self,m):
        self.ev_received+=1

    def command_message(self,msg):
        super().command_message(msg)
        ack=VehicleCommandAck();ack.timestamp=self.get_clock().now().nanoseconds//1000
        ack.command=msg.command;ack.result=0;ack.target_system=1;ack.target_component=191
        self.input_pubs['vehicle_command_ack'].publish(ack)

    def publish_inputs(self):
        stamp=self.get_clock().now().nanoseconds;now=time.monotonic()
        a=self.pipeline.alignment
        local=a.local_origin+a.rotation@(np.array([*self.xy,self.z])-a.odom_origin)
        velocity=a.rotation@np.array([*self.v,self.vz])
        landed=bool(self.z<=self.floor_z+.002 and (not self.was_airborne or self.desired_z<=self.floor_z+.002))
        status=VehicleStatus();status.timestamp=stamp//1000
        status.arming_state=2 if self.armed else 1;status.nav_state=self.px4_mode
        status.pre_flight_checks_pass=True;status.gcs_connection_lost=False
        loc=VehicleLocalPosition();loc.timestamp=stamp//1000
        loc.x,loc.y,loc.z=map(float,local);loc.vx,loc.vy,loc.vz=map(float,velocity)
        loc.heading=0.;loc.xy_valid=loc.z_valid=loc.v_xy_valid=loc.v_z_valid=True
        flags=EstimatorStatusFlags();flags.timestamp=stamp//1000
        flags.cs_ev_pos=flags.cs_ev_yaw=flags.cs_ev_hgt=flags.cs_baro_hgt=True
        land=VehicleLandDetected();land.timestamp=stamp//1000;land.landed=landed
        rc=ManualControlSetpoint();rc.timestamp=stamp//1000;rc.timestamp_sample=stamp//1000-1000
        rc.valid=True;rc.data_source=1
        ready_at=self.ready.policy.ready_at
        rc.aux1=1. if ready_at is not None and now-ready_at>.3 else -1.;rc.aux2=-1.
        for name,m in [('vehicle_status_v1',status),('vehicle_local_position',loc),
                       ('estimator_status_flags',flags),('vehicle_land_detected',land),
                       ('manual_control_setpoint',rc)]:self.input_pubs[name].publish(m)
        tr=String();tr.data=json.dumps(dict(vo_state=1,stamp_ns=stamp))
        self.input_pubs['tracking'].publish(tr)
        od=Odometry();od.header.stamp.sec=stamp//10**9;od.header.stamp.nanosec=stamp%10**9
        od.header.frame_id='odom';od.child_frame_id='base_link'
        od.pose.pose.position.x=float(self.xy[0]);od.pose.pose.position.y=float(self.xy[1])
        od.pose.pose.position.z=float(self.z);od.pose.pose.orientation.w=1.
        self.input_pubs['vio'].publish(od)

    def control_tick(self):
        if self.navigation_sample is None or not hasattr(self,'receiver'):return
        msg,world_frame=self.navigation_sample
        self.receiver.audit()
        # Audit stamps must precede the executive snapshot; sampling now before
        # audit makes fresh ownership appear to come from the future.
        now=time.monotonic()
        c,s=math.cos(self.yaw),math.sin(self.yaw)
        world=(msg.linear.x,msg.linear.y) if world_frame else (c*msg.linear.x-s*msg.linear.y,s*msg.linear.x+c*msg.linear.y)
        sample=self.ready.policy.sample
        # Air/land state has one authoritative telemetry source. This avoids
        # mixing current plant truth with an older delivered land message.
        scene=replace(self.make_scene(now,world),landed=sample.landed,airborne=not sample.landed)
        evidence=LandingEvidence(now,self.ground_z,.14,geometry_verified=True,
            column_clear=True,bounds_verified=True,armed=sample.armed,
            anchor_drift_rate_bound_mps=.001,anchor_drift_model_verified=True)
        result=self.ready.tick(now,scene,evidence,TakeoffSpace(now,True),
            timestamp_us=self.get_clock().now().nanoseconds//1000,vio_session='synthetic')
        if self.writer.dispatch(result):self.ready.sent(result)
        else:self.ready.policy.close('sole writer fenced')
        self.landing_state=self.sortie.state;self.states.add(self.landing_state)
        if result['messages']:self.pipeline_messages+=1
        if self.sortie.state in ('ABORT','HANDOVER','KILLED') or self.writer.fault:
            self.pipeline_fault=self.sortie.reason or self.writer.fault
        if self.sortie.state in self.sortie.TERMINAL or not result['messages']:
            self.delayed.clear();self.desired_v[:]=0.;self.desired_z=self.z
        self.reasons[self.sortie.reason]+=1
        self.landing_samples.append(dict(at=now,state=self.landing_state,
            position=[*map(float,self.xy),self.z],reason=self.sortie.reason,
            h_valid=scene.h_metric_verified,h_age_s=now-self.h_at,
            h_stable_frames=self.stable,h_error_odom=list(scene.h_error_odom),
            speed_mps=float(np.linalg.norm(self.v)),
            align_stable_since=self.pipeline.mission.align.since,
            map_age_s=now-scene.map_at,footprint_free=scene.footprint_known_free))
        if not self.phase_reports or self.phase_reports[-1]['sortie_state']!=self.sortie.state:
            self.phase_reports.append(self.ready.report())

    def landing_report(self):
        r=super().landing_report()
        p=self.ready.policy
        complete=bool(r['sortie_validation_passed'] and p.ready_at is not None
            and p.accepted_commands=={'OFFBOARD','ARM'} and self.ev_received>30
            and self.receiver.owns() and all(self.receiver.counts[n]>0 for n in
                ('vehicle_status_v1','vehicle_local_position','estimator_status_flags',
                 'vehicle_land_detected','manual_control_setpoint','vehicle_command_ack','tracking','vio')))
        r.update(sortie_validation_passed=complete,production_ready_aux_runtime_used=False,
            production_ready_aux_policy_used=True,dedicated_isolated_input_adapter_used=True,
            production_ev_bridge_used=False,production_ev_policy_used=True,
            telemetry_is_synthetic=True,geometry_is_synthetic=True,
            ready_policy=self.ready.report(),received_telemetry=dict(self.receiver.counts),
            shadow_ev_published=self.receiver.ev_count,shadow_ev_received=self.ev_received,
            input_topics=self.receiver.inputs)
        return r

    def save_landing_evidence(self,out):
        super().save_landing_evidence(out)
        (out/'ready_phase_reports.json').write_text(json.dumps(self.phase_reports,indent=2))


if __name__=='__main__':raise SystemExit(main(ReadyFixture,full_landing=True))
