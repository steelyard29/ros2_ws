"""Full synthetic ground-start mission with one actual DDS candidate writer.

Real Nav2/APF and H detector; synthetic telemetry/ACK/landing and perfect VIO.
No real PX4 firmware, EV input, actuators, READY/AUX runtime or flight permit.
"""
import math
import time
import hashlib
import json
import numpy as np
from dataclasses import replace
from legacy_apf_h_closed_loop import HLandingFixture, main, ROOT
from flight_supervisor import FlightSample
from landing_sequence_shadow import LandingEvidence
from sortie_shadow import SortieShadow, SortieShadowWriter, TakeoffSpace
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


class SortieFixture(HLandingFixture):
    def source_hashes(self):
        names = ['scripts/sortie_shadow.py', 'scripts/flight_supervisor.py',
                 'scripts/flight_messages.py', 'scripts/flight_dispatch_contract.py',
                 'scripts/navigation_shadow_pipeline.py', 'scripts/navigation_landing_shadow.py',
                 'scripts/navigation_frame_contract.py', 'scripts/landing_sequence_shadow.py',
                 'scripts/descent_shadow.py', 'tests/sortie_h_closed_loop.py',
                 'tests/legacy_apf_h_closed_loop.py', 'tests/nvblox_closed_loop.py']
        return {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in names}

    def __init__(self, controller):
        super().__init__(controller)
        self.source_snapshot = self.source_hashes()
        self.z = self.floor_z
        self.desired_z = self.z
        self.sortie = SortieShadow(self.sequence, height=1.1, hover=3.)
        self.writer = SortieShadowWriter(self, self.sortie)
        self.armed = False
        self.px4_mode = 2
        self.accepted = set()
        self.was_airborne = False
        self.dds_setpoints = 0
        self.dds_modes = 0
        self.command_trace = []
        self.mode_stamp = None
        self.create_subscription(OffboardControlMode, self.writer.PREFIX+'offboard_control_mode', self.mode_message, 10)
        self.create_subscription(TrajectorySetpoint, self.writer.PREFIX+'trajectory_setpoint', self.setpoint_message, 10)
        self.create_subscription(VehicleCommand, self.writer.PREFIX+'vehicle_command', self.command_message, 10)

    def mode_message(self, msg):
        if msg.position:
            self.mode_stamp = msg.timestamp
            self.dds_modes += 1

    def command_message(self, msg):
        self.command_trace.append(dict(at=time.monotonic(), command=int(msg.command), param1=msg.param1))
        if msg.command == 176:
            self.px4_mode = 14
            self.accepted.add('OFFBOARD')
        elif msg.command == 400 and msg.param1 == 1.:
            self.armed = True
            self.accepted.add('ARM')
        else:
            self.pipeline_fault = 'unexpected command in synthetic nominal mission'

    def setpoint_message(self, msg):
        # Plant consumes the ROS-delivered message, never the returned Python
        # objects. Delivery/plant effects remain hypothetical and not PX4 SITL.
        a = self.pipeline.alignment
        local = a.local_origin+a.rotation@(np.array([*self.xy, self.z])-a.odom_origin)
        if all(math.isfinite(v) for v in msg.position):
            target = a.odom_origin+a.rotation.T@(np.array(msg.position)-a.local_origin)
            v = np.clip((target[:2]-self.xy)/.3, -.15, .15)
        else:
            target = a.odom_origin+a.rotation.T@(np.array([*local[:2], msg.position[2]])-a.local_origin)
            v = (a.rotation.T@np.array([*msg.velocity[:2], 0.]))[:2]
        self.delayed.append((time.monotonic()+.1, v, 0., float(target[2])))
        self.dds_setpoints += 1
        self.max_target_v = max(self.max_target_v, math.hypot(*v))

    def advance_vertical(self, dt):
        if not getattr(self, 'armed', False):
            self.vz = 0.
            return
        super().advance_vertical(dt)
        self.was_airborne |= self.z > self.floor_z+.1
        if (self.was_airborne and self.z <= self.floor_z+.002
                and self.desired_z <= self.floor_z+.002):
            # Hypothetical automatic disarm on simulated contact, not a command.
            self.armed = False

    def control_tick(self):
        if self.navigation_sample is None or not hasattr(self, 'writer'):
            return
        msg, world_frame = self.navigation_sample
        now = time.monotonic()
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        world = (msg.linear.x, msg.linear.y) if world_frame else (c*msg.linear.x-s*msg.linear.y, s*msg.linear.x+c*msg.linear.y)
        scene = self.make_scene(now, world)
        landed = self.z <= self.floor_z+.002 and not self.was_airborne or (
            self.was_airborne and self.z <= self.floor_z+.002 and self.desired_z <= self.floor_z+.002)
        scene = replace(scene, landed=bool(landed), airborne=not landed)
        a = self.pipeline.alignment
        local = a.local_origin+a.rotation@(np.array([*self.xy, self.z])-a.odom_origin)
        velocity = a.rotation@np.array([*self.v, self.vz])
        sample = FlightSample(status_at=now, local_at=now, flags_at=now, land_at=now, rc_at=now,
            armed=self.armed, landed=bool(landed), nav_state=self.px4_mode, preflight_ok=True,
            rc_valid=True, input_ownership_ok=self.writer.owns(), ev_ok=True,
            ev_position=True, ev_height=True, ev_yaw=True, baro_height=True,
            xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True,
            x=local[0], y=local[1], z=local[2], heading=0.,
            vx=velocity[0], vy=velocity[1], vz=velocity[2], reset_counters=(0,)*5)
        evidence = LandingEvidence(now, self.ground_z, .14, geometry_verified=True,
            column_clear=True, bounds_verified=True, armed=self.armed,
            anchor_drift_rate_bound_mps=.001, anchor_drift_model_verified=True)
        result = self.sortie.step(now, sample, scene, evidence, TakeoffSpace(now, True),
            timestamp_us=self.get_clock().now().nanoseconds//1000, vio_session='synthetic',
            start=True, accepted_commands=self.accepted, switch_fresh=True, mode_slot=6, kill_switch=3)
        self.writer.dispatch(result)
        self.landing_state = self.sortie.state
        self.states.add(self.landing_state)
        if result['messages']:
            self.pipeline_messages += 1
        if self.sortie.state in ('ABORT', 'HANDOVER', 'KILLED') or self.writer.fault:
            self.pipeline_fault = self.sortie.reason or self.writer.fault
        if self.sortie.state in self.sortie.TERMINAL or not result['messages']:
            # Offline plant fallback only, not a verified hardware response.
            self.delayed.clear()
            self.desired_v[:] = 0.
            self.desired_z = self.z
        self.reasons[self.sortie.reason] += 1
        self.landing_samples.append(dict(at=now, state=self.landing_state,
            position=[*map(float, self.xy), self.z], reason=self.sortie.reason))

    def landing_report(self):
        r = super().landing_report()
        transitions = []
        for at, state in self.sortie.history:
            if not transitions or transitions[-1][1] != state:
                transitions.append([at, state])
        states = {state for _, state in transitions}
        unchanged = self.source_snapshot == self.source_hashes()
        complete = (self.sortie.state == 'DONE' and self.dds_setpoints > 0 and self.dds_modes > 0
            and self.writer.owns() and self.writer.fault is None and unchanged
            and {'TAKEOFF', 'HOVER', 'MISSION_HANDOFF', 'NAVIGATE', 'ALIGN_H', 'DESCEND', 'TOUCHDOWN', 'DONE'} <= states
            and {r['command'] for r in self.command_trace} == {176, 400})
        r.update(ground_start_sortie=True, dds_candidate_output=True,
            dds_setpoints_received=self.dds_setpoints, dds_modes_received=self.dds_modes,
            unique_output_writer=self.writer.owns(), dispatch_fault=self.writer.fault,
            dispatch_counts=self.writer.counts, takeoff_handoff_at=self.sortie.handoff_at,
            sortie_transitions=transitions, synthetic_commands=self.command_trace,
            source_hashes=self.source_snapshot, source_unchanged_during_test=unchanged,
            sortie_validation_passed=bool(complete),
            physical_takeoff_column_verified=False, production_ready_aux_runtime_used=False,
            production_ev_bridge_used=False, synthetic_auto_disarm=True)
        return r

    def save_landing_evidence(self, out):
        super().save_landing_evidence(out)
        (out/'sortie_history.json').write_text(json.dumps(self.sortie.history))


if __name__ == '__main__':
    raise SystemExit(main(SortieFixture, full_landing=True))
