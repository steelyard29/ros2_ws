"""Pure full-sortie checks. Synthetic geometry/feedback is NOT flight evidence."""
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from cruise_band import make_band
from flight_supervisor import FlightSample, FlightSupervisor
from navigation_landing_shadow import Scene, MissionShadow
from navigation_shadow_pipeline import ShadowPipeline
from navigation_frame_contract import SessionAlignment
from landing_sequence_shadow import LandingSequenceShadow, LandingEvidence
from sortie_shadow import SortieShadow, TakeoffSpace, SortieShadowWriter


class Rig:
    def __init__(self):
        self.t = 0.
        self.p = np.array([0., 0., 0.])
        self.armed = False
        self.nav = 2
        self.accepted = set()
        self.velocity = np.zeros(3)
        self.was_airborne = False
        mission = MissionShadow(make_band(), (-2, 2, -2, 2), lambda p, v, c: True)
        alignment = SessionAlignment([0, 0, 0], 0, [2, -3, .7], .8, 'test', (0,)*5, verified=True)
        self.alignment = alignment
        pipeline = ShadowPipeline(mission, alignment, 0.)
        self.core = SortieShadow(LandingSequenceShadow(pipeline, .05), height=.6, hover=1.)
        self.trace = []

    def tick(self, sample_kw=None, scene_kw=None, space_ok=True, dt=.05, frame_kw=None):
        self.t += dt
        t = self.t
        # Plant truth must not reuse the controller's invalidation latch.
        pos = self.alignment.local_origin+self.alignment.rotation@(self.p-self.alignment.odom_origin)
        v = self.alignment.rotation@self.velocity
        landed = self.p[2] <= .002
        if self.was_airborne and landed and self.core.sequence.descent.state in ('TERMINAL_DESCEND', 'TOUCHDOWN'):
            self.armed = False
        self.was_airborne |= self.p[2] > .1
        sample = FlightSample(status_at=t, local_at=t, flags_at=t, land_at=t, rc_at=t,
            armed=self.armed, landed=landed, nav_state=self.nav, preflight_ok=True,
            rc_valid=True, input_ownership_ok=True, ev_ok=True, ev_position=True,
            ev_height=True, ev_yaw=True, baro_height=True, xy_valid=True, z_valid=True,
            v_xy_valid=True, v_z_valid=True, x=pos[0], y=pos[1], z=pos[2], heading=.8,
            vx=v[0], vy=v[1], vz=v[2], reset_counters=(0,)*5)
        scene = Scene(at=t, position=tuple(self.p), velocity_xy=tuple(self.velocity[:2]),
            localization_ok=True, airborne=not landed, landed=landed, map_at=t,
            footprint_known_free=True, navigation_velocity_odom=(.1, 0.), navigation_at=t,
            goal_reached=self.p[0] >= .7, h_error_odom=(.72-self.p[0], -self.p[1]),
            h_at=t, h_metric_verified=True, landing_boundary_verified=True, tilt_rad=0.)
        sample = replace(sample, **(sample_kw or {}))
        scene = replace(scene, **(scene_kw or {}))
        evidence = LandingEvidence(t, -.14, .14, geometry_verified=True, column_clear=True,
                                   bounds_verified=True, armed=sample.armed)
        frame = dict(timestamp_us=int((t+1)*1e6), vio_session='test', start=True,
                     accepted_commands=self.accepted, switch_fresh=True, mode_slot=6, kill_switch=3)
        frame.update(frame_kw or {})
        r = self.core.step(t, sample, scene, evidence, TakeoffSpace(t, space_ok), **frame)
        self.trace.append(r)
        messages = r['messages']
        if 'vehicle_command' in messages:
            code = messages['vehicle_command'].command
            if code == 176:
                self.nav = 14
                self.accepted.add('OFFBOARD')
            elif code == 400:
                self.armed = True
                self.accepted.add('ARM')
            else:
                raise AssertionError('nominal sortie must not request LAND/disarm')
        if 'trajectory_setpoint' in messages:
            sp = messages['trajectory_setpoint']
            local_target = np.array([pos[0], pos[1], sp.position[2]])
            target = self.alignment.odom_origin+self.alignment.rotation.T@(local_target-self.alignment.local_origin)
            if math.isfinite(sp.position[0]):
                self.velocity[:2] = 0.
            else:
                self.velocity[:2] = (self.alignment.rotation.T@np.array([*sp.velocity[:2], 0.]))[:2]
            self.velocity[2] = float(np.clip((target[2]-self.p[2])/.2, -.1, .2))
            self.p += self.velocity*dt
            if self.p[2] < .002 and target[2] <= .002:
                self.p[2] = 0.
        return r

    def navigate(self):
        for _ in range(350):
            r = self.tick()
            if r['state'] == 'NAVIGATE':
                return r
            assert r['state'] not in self.core.TERMINAL, r
        raise AssertionError('no mission handoff')


class SortieTests(unittest.TestCase):
    def test_full_ground_takeoff_navigation_h_landing(self):
        rig = Rig()
        for _ in range(1200):
            r = rig.tick()
            if r['state'] in rig.core.TERMINAL:
                break
        self.assertEqual(r['state'], 'DONE', r)
        self.assertLess(math.hypot(rig.p[0]-.72, rig.p[1]), .05)
        states = {state for _, state in rig.core.history}
        self.assertTrue({'STREAM', 'OFFBOARD', 'ARM', 'TAKEOFF', 'HOVER', 'MISSION_HANDOFF',
                         'NAVIGATE', 'ALIGN_H', 'DESCEND', 'TOUCHDOWN', 'DONE'} <= states, states)
        for out in rig.trace:
            if out['state'] in ('STREAM', 'OFFBOARD', 'ARM', 'TAKEOFF', 'HOVER'):
                sp = out['messages']['trajectory_setpoint']
                np.testing.assert_allclose(sp.position[:2], [2, -3], atol=1e-6)
            self.assertFalse(out['flight_authorized'])

    def test_no_takeoff_without_column_and_baseline_guard_unchanged(self):
        rig = Rig()
        for _ in range(80):
            r = rig.tick(space_ok=False)
            self.assertFalse(r['messages'])
        self.assertEqual(r['state'], 'WAIT')
        self.assertIsNone(rig.core.handoff_at)
        self.assertIs(type(FlightSupervisor()), FlightSupervisor)

    def test_takeoff_horizontal_departure_still_aborts(self):
        rig = Rig()
        while rig.core.state != 'TAKEOFF':
            rig.tick()
        rig.p[0] = .31
        out = rig.tick()
        self.assertEqual(out['state'], 'ABORT', out)
        self.assertIn('horizontal departure', out['reason'])

    def test_mission_health_faults_and_takeover_latch(self):
        cases = [({'manual_takeover': True}, {}, 'HANDOVER'),
                 ({'input_ownership_ok': False}, {}, 'HANDOVER'),
                 ({'nav_state': 2}, {}, 'HANDOVER'),
                 ({'kill_active': True}, {}, 'KILLED'),
                 ({'ev_ok': False}, {}, 'ABORT'),
                 ({'rc_at': 0}, {}, 'ABORT'),
                 ({'status_at': 0}, {}, 'ABORT'),
                 ({'reset_counters': (1, 0, 0, 0, 0)}, {}, 'ABORT'),
                 ({'x': 5.}, {}, 'ABORT'),
                 ({}, {'localization_ok': False}, 'ABORT')]
        for skw, ckw, expected in cases:
            with self.subTest(skw=skw, ckw=ckw):
                rig = Rig()
                rig.navigate()
                r = rig.tick(sample_kw=skw, scene_kw=ckw)
                self.assertEqual(r['state'], expected, r)
                self.assertFalse(r['messages'])
                # Keep the terminal state even when all inputs recover.
                self.assertFalse(rig.tick()['messages'])

    def test_map_stale_stops_horizontal_not_silently_descends(self):
        rig = Rig()
        rig.navigate()
        r = rig.tick(scene_kw={'map_at': 0})
        sp = r['messages']['trajectory_setpoint']
        self.assertEqual(list(sp.velocity[:2]), [0., 0.])
        self.assertAlmostEqual(sp.position[2], .1, places=5)

    def test_executive_gap_and_session_change_latch(self):
        for mode in ('gap', 'session'):
            rig = Rig()
            rig.navigate()
            r = rig.tick(dt=.4) if mode == 'gap' else rig.tick(frame_kw={'vio_session': 'new'})
            self.assertEqual(r['state'], 'ABORT', r)
            self.assertFalse(r['messages'])

    def test_wrong_ground_initialized_map_band_never_starts(self):
        rig = Rig()
        rig.core.sequence.navigation.mission.band = make_band(initial_z=.2)
        r = rig.tick()
        self.assertEqual(r['state'], 'ABORT')
        self.assertFalse(r['messages'])

    def test_dispatch_fences_replay_conflict_and_exception(self):
        class Publisher:
            def __init__(self):
                self.sent = []
                self.fail = False
            def publish(self, msg):
                if self.fail:
                    raise RuntimeError('test transport failure')
                self.sent.append(msg)
        class Node:
            conflict = False
            def create_publisher(self, cls, topic, depth):
                return Publisher()
            def count_publishers(self, topic):
                return 2 if self.conflict else 1
            def get_topic_names_and_types(self):
                return []
        for failure in ('replay', 'conflict', 'exception', 'speed', 'command'):
            with self.subTest(failure=failure), patch.dict('os.environ', {'ROS_DOMAIN_ID': '176'}):
                rig = Rig()
                result = rig.navigate()
                node = Node()
                writer = SortieShadowWriter(node, rig.core)
                if failure == 'replay':
                    self.assertTrue(writer.dispatch(result))
                elif failure == 'conflict':
                    node.conflict = True
                elif failure == 'exception':
                    writer.pubs['trajectory_setpoint'].fail = True
                elif failure == 'speed':
                    result['messages']['trajectory_setpoint'].velocity = [1., 0., math.nan]
                else:
                    from px4_msgs.msg import VehicleCommand
                    msg = VehicleCommand()
                    msg.timestamp = rig.core.last_stamp
                    msg.command = 400
                    msg.param1 = 1.
                    result['messages']['vehicle_command'] = msg
                self.assertFalse(writer.dispatch(result))
                counts = dict(writer.counts)
                self.assertFalse(writer.dispatch(rig.tick()))
                self.assertEqual(counts, writer.counts)
                self.assertEqual(rig.core.state, 'ABORT')


if __name__ == '__main__':
    unittest.main()
