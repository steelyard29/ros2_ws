"""Illustrative lagged XY plant: NOT PX4 SITL, actual DWB, or flight proof.

Known synthetic map, perfect metric H/attitude, no thrust/ground effect/wind.
Exercises shadow mission with a delayed velocity response and stopping audit.
The candidate adapter remains disconnected from the real flight controller.
"""
import math
import sys
import unittest
from collections import deque
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from cruise_band import make_band
from navigation_landing_shadow import MissionShadow,Scene
from stopping_space import stopping_space_clear


def rehearse(fault=None):
    dt=.05;res=.025
    rows,cols=np.mgrid[:160,:160]
    x=-1+(cols+.5)*res;y=-2+(rows+.5)*res
    # Two boxes separated by a 1.2 m corridor, synthetic observed free start.
    free=~((x>=.7)&(x<=1.5)&(np.abs(y)>=.6)&(np.abs(y)<=1.4))
    position=np.zeros(2);velocity=np.zeros(2)
    def geometry(p,v,c):
        return stopping_space_clear(free,res,(-1,-2),p,v,c,body_radius=.43,
                                   uncertainty=.05,latency=.3,braking=.2)
    mission=MissionShadow(make_band(),(-.5,2.6,-1.5,1.5),geometry)
    delay=deque([np.zeros(2),np.zeros(2)]) # 100 ms command transport delay
    states=set();collision=False;stopped_after_fault=False;min_clearance=math.inf
    for i in range(1800):
        t=i*dt;broken=fault is not None and t>=5
        goal=position[0]>=1.85
        s=Scene(at=t,position=(*position,.6),tilt_rad=0.,velocity_xy=tuple(velocity),
            localization_ok=not (broken and fault=='localization'),airborne=True,
            map_at=t-1 if broken and fault=='map' else t,footprint_known_free=True,
            navigation_at=t,navigation_velocity_odom=(.15,0),goal_reached=goal,
            h_error_odom=tuple(np.array([2.,0.])-position),h_at=t,h_metric_verified=True,
            landing_boundary_verified=True,manual_override=broken and fault=='override')
        out=mission.step(t,s);states.add(out['state'])
        desired=np.asarray(out['velocity_odom_xy'])
        if broken and np.linalg.norm(desired)==0:stopped_after_fault=True
        delay.append(desired);delayed=delay.popleft()
        # First-order velocity tracking, acceleration bounded at assumed .2 m/s2.
        acceleration=(delayed-velocity)/.3
        norm=np.linalg.norm(acceleration)
        if norm>.2:acceleration*=.2/norm
        velocity+=acceleration*dt;position+=velocity*dt
        # Independent geometric collision check, square body axis-aligned.
        for cy in (-1.,1.):
            dx=max(abs(position[0]-1.1)-.4,0)
            dy=max(abs(position[1]-cy)-.4,0)
            collision |= dx<.3 and dy<.3
            min_clearance=min(min_clearance,math.hypot(dx,dy)-.3)
        if out['state']=='LAND_REQUEST':break
        if broken and t>=8:break
    return dict(final_state=out['state'],states=states,position=position,
                velocity=velocity,collision=collision,minimum_clearance=min_clearance,
                stopped_after_fault=stopped_after_fault,flight_authorized=out['flight_authorized'])


class LagTests(unittest.TestCase):
    def test_synthetic_corridor_to_h_with_velocity_lag(self):
        r=rehearse()
        self.assertEqual(r['final_state'],'LAND_REQUEST',r)
        self.assertFalse(r['collision'])
        self.assertLess(np.linalg.norm(r['position']-[2,0]),.05)
        self.assertLess(np.linalg.norm(r['velocity']),.05)
        self.assertFalse(r['flight_authorized'])

    def test_stale_map_stops_and_pose_loss_latches(self):
        for fault,state in [('map','NAVIGATE'),('localization','ABORT'),('override','HANDOVER')]:
            r=rehearse(fault)
            self.assertEqual(r['final_state'],state,r)
            self.assertTrue(r['stopped_after_fault'])
            self.assertLess(np.linalg.norm(r['velocity']),.001)
            self.assertFalse(r['collision'])


if __name__=='__main__':unittest.main()
