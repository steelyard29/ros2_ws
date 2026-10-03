"""PX4 center 1 m above ground / 2.5 m forward / return / H-landing candidate.

Extends the existing A*/APF-to-H mission, NOT a stand-alone motor script.
Use with SortieShadow and LandingSequenceShadow for integration testing.
TaskFlightCore now supplies real inputs/dispatch under a separate reviewed
permit. That route is not flight-validated. This module itself cannot arm.
"""
import argparse
from dataclasses import replace
import json
import math
from navigation_landing_shadow import MissionShadow
from task_site_bounds import inside_bounds


class RoundTripMission(MissionShadow):
    DISTANCE = 2.5
    GROUND_CENTER_HEIGHT = .14  # Operator measured, unloaded.
    TARGET_CENTER_HEIGHT = 1.0  # Operator explicitly selected height reference.
    RISE = TARGET_CENTER_HEIGHT-GROUND_CENTER_HEIGHT
    ARRIVAL = .08
    SETTLE_SPEED = .05
    SETTLE_SECONDS = 2.0

    def __init__(self, band, bounds, stopping_check, *, ground_origin, yaw_odom,
                 session_id, landing_error_limit=.05):
        super().__init__(band, bounds, stopping_check, landing_error_limit)
        if (len(ground_origin) != 3 or not all(math.isfinite(v) for v in ground_origin)
                or not math.isfinite(yaw_odom) or not isinstance(session_id, str)
                or not session_id.strip()):
            raise ValueError('finite ground origin/heading and unique session required')
        if (not math.isclose(band.center-ground_origin[2], self.RISE, abs_tol=1e-6)
                or not math.isclose(band.center-band.floor,self.TARGET_CENTER_HEIGHT,abs_tol=1e-6)):
            raise ValueError('requires 0.86m rise and PX4 center 1m above the same ground plane')
        self.home = tuple(ground_origin[:2])
        self.endpoint = (self.home[0]+self.DISTANCE*math.cos(yaw_odom),
                         self.home[1]+self.DISTANCE*math.sin(yaw_odom))
        self.yaw = yaw_odom
        self.session_id = session_id
        self.leg = 'OUTBOUND'
        self.arrived_since = None
        self.return_settled = False
        self.transitions = []
        # Bounds are robot-center limits after collision-envelope accounting.
        if not all(inside_bounds(bounds,p) for p in (self.home,self.endpoint)):
            raise ValueError('home/endpoint outside reviewed robot-center bounds')

    @property
    def goal_id(self):
        return self.session_id+':'+self.leg

    def planner_goal(self):
        p = self.endpoint if self.leg == 'OUTBOUND' else self.home
        return dict(goal_id=self.goal_id, frame_id='odom', position=[*p,self.band.center],
                    yaw=self.yaw, flight_authorized=False)

    def output(self, velocity=(0.,0.), reason=''):
        out = super().output(velocity, reason)
        out.update(route_leg=self.leg, navigation_goal=self.planner_goal())
        return out

    def step(self, now, s):
        # A terminal mission never changes leg or revives after inputs recover.
        if self.state in ('ABORT','HANDOVER','DONE'):
            return self.output(reason='latched terminal state')
        try:
            p = self.planner_goal()['position']
            near = (len(s.position)==3 and all(math.isfinite(v) for v in s.position)
                    and math.dist(s.position[:2],p[:2]) <= self.ARRIVAL)
            slow = (len(s.velocity_xy)==2 and all(math.isfinite(v) for v in s.velocity_xy)
                    and math.hypot(*s.velocity_xy) < self.SETTLE_SPEED)
            correlated = s.navigation_goal_id == self.goal_id
            fresh_space = (0<=now-s.map_at<=.5 and s.footprint_known_free
                           and self.stopping_clear(s,(0.,0.)))
            settled = (near and slow and correlated and fresh_space
                       and 0<=now-s.navigation_at<=.25)
            # Never trust goal_reached alone, including after planner goal changes.
            # H alignment is allowed only after an independently settled return.
            landing_requested = bool(self.leg=='RETURN' and self.return_settled and settled)
            suppress = near or not correlated
            projected = replace(s, goal_reached=landing_requested,
                navigation_velocity_odom=(0.,0.) if suppress else s.navigation_velocity_odom,
                navigation_at=now if suppress else s.navigation_at)
            out = super().step(now, projected)
            if self.state != 'NAVIGATE':
                self.arrived_since = None
                return out
            if not settled or out['reason']:
                self.arrived_since = None
                self.return_settled = False
                if not correlated and not out['reason']:
                    return self.output(reason='waiting for planner command for current goal ID')
                return out
            if self.arrived_since is None:
                self.arrived_since = now
            if now-self.arrived_since < self.SETTLE_SECONDS:
                return self.output(reason='arrival dwell; H landing inhibited until return')
            if self.leg == 'OUTBOUND':
                self.leg = 'RETURN'
                self.arrived_since = None
                self.transitions.append(dict(at=now,event='outbound_settled_switch_to_home'))
                return self.output(reason='new return goal; discard previous planner command')
            self.return_settled = True
            return self.output(reason='return settled; H alignment begins on next valid sample')
        except (ValueError,TypeError,IndexError,ArithmeticError):
            self.state = 'ABORT'
            return self.output(reason='invalid roundtrip observation')


def describe():
    return dict(profile='roundtrip_1m_2p5m', rise_from_ground_start_body_pose_m=RoundTripMission.RISE,
        target_px4_center_above_ground_m=RoundTripMission.TARGET_CENTER_HEIGHT,
        ground_px4_center_above_ground_m=RoundTripMission.GROUND_CENTER_HEIGHT,
        forward_distance_m=2.5, heading_reference='captured takeoff heading, fixed throughout',
        legs=['TAKEOFF','OUTBOUND','SETTLE','RETURN','SETTLE','ALIGN_H','DESCEND','TOUCHDOWN'],
        max_horizontal_speed_mps=.15, arrival_radius_m=.08, settle_seconds=2.,
        requires=['fresh localization and PX4 telemetry','correlated A*/APF planner command',
                  'map-backed stopping space','H observation and landing evidence',
                  'real task adapter and operator-supervised commissioning'],
        real_flight_adapter_available=True, real_flight_adapter_validated=False,
        real_flight_entry='task_flight_runtime.py (separate task permit required)',flight_authorized=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--describe', action='store_true', help='print profile; never connects hardware')
    parser.parse_args()
    print(json.dumps(describe(), indent=2))
