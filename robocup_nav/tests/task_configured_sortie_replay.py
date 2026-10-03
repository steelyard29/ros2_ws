"""Offline exact task supervisor + site geometry + projected synthetic H.

Ideal goal-seeking velocity, perfect pose/dynamics; NOT A*/APF, SITL, real
camera accuracy, or flight clearance. Does not initialize ROS or publishers.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import cv2
import numpy as np
import yaml
from test_task_handoff import TaskRig
from roundtrip_flight_test import RoundTripMission
from task_site_bounds import resolve_bounds,inside_bounds
from landing_projection import target_on_ground
from navigation_frame_contract import SessionAlignment

ROOT=Path(__file__).resolve().parents[1]


def replay(yaw):
    cfg=yaml.safe_load((ROOT/'config/roundtrip_task.yaml').read_text())
    camera=cfg['camera'];r=TaskRig(hover=3.)
    r.a=SessionAlignment((0,0,0),yaw,(2,-3,.7),.8,'unit-task',(0,)*5,verified=True)
    bounds=resolve_bounds(cfg,(0.,0.),yaw)
    r.m=RoundTripMission(r.m.band,bounds,lambda *a:True,
        ground_origin=(0.,0.,0.),yaw_odom=yaw,session_id='unit-task')
    c,s=math.cos(yaw),math.sin(yaw)
    world_from_body=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
    body_from_optical=np.array(camera['body_from_optical'],dtype=float)
    offset=np.array(camera['offset_flu_m']);K=np.array(camera['k'])
    distortion=np.array(camera['distortion'])
    h_world=np.array([0.,0.,r.m.band.floor])
    max_forward=0.;max_height=.14;projection_error=0.;invalid_h=valid_h=0
    for _ in range(2400):
        origin=r.p+world_from_body@offset
        optical=body_from_optical.T@world_from_body.T@(h_world-origin)
        pixel,_=cv2.projectPoints(optical.reshape(1,3),np.zeros(3),np.zeros(3),K,distortion)
        pixel=pixel.reshape(2)
        w,h=camera['image_size']
        visible=bool(optical[2]>0 and 0<=pixel[0]<w and 0<=pixel[1]<h)
        changes=dict(h_metric_verified=False,h_at=-1.)
        if visible:
            # These axes are perfect synthetic truth, NOT a production review.
            target=target_on_ground(pixel,K,distortion,(w,h),(w,h),r.p,world_from_body,
                body_from_optical,offset,r.m.band.floor,calibration_verified=True,axes_verified=True)
            projection_error=max(projection_error,float(np.linalg.norm(target-h_world)))
            changes=dict(h_metric_verified=True,h_at=r.now+.05,
                         h_error_odom=tuple(target[:2]-r.p[:2]))
            valid_h+=1
        else:invalid_h+=1
        out,_=r.tick(scene_changes=changes)
        if not inside_bounds(bounds,tuple(r.p[:2])):raise AssertionError('left exact site boundary')
        max_forward=max(max_forward,c*r.p[0]+s*r.p[1])
        max_height=max(max_height,r.p[2]+.14)
        if r.c.state in r.c.TERMINAL:break
    states=list(dict.fromkeys(x.state for x in r.trace))
    passed=bool(out.state=='DONE' and not r.s.armed and r.s.landed and r.m.leg=='RETURN'
        and {'NAVIGATE','ALIGN_H','LAND','DONE'}<=set(states)
        and 2.4<max_forward<=2.5 and math.hypot(*r.p[:2])<.05
        and abs(max_height-1.)<1e-6 and projection_error<1e-4)
    return dict(passed=passed,yaw_rad=yaw,state=out.state,reason=out.reason,
        simulated_seconds=r.now,states=states,commands=sorted(r.commands),
        max_forward_m=float(max_forward),max_px4_height_m=float(max_height),
        final_position_m=r.p.tolist(),landed=r.s.landed,armed=r.s.armed,
        simulated_touchdown_threshold_m=.01,
        synthetic_projection_error_m=projection_error,
        visible_h_samples=valid_h,out_of_image_h_samples=invalid_h,
        production_axes_reviewed=camera['axes_reviewed'])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-offline',action='store_true');a=p.parse_args()
    if not a.run_offline:
        print('Describe only: ideal offline task + site + H projection, no hardware.');return 0
    start=time.monotonic();cases=[replay(y) for y in (0.,math.pi/4,math.pi/2)]
    report=dict(passed=all(x['passed'] for x in cases),cases=cases,
        hardware=False,actual_astar_apf=False,px4_sitl=False,flight_ready=False,
        assumptions=['ideal goal-seeking velocities','free map','perfect VIO and dynamics',
                     'synthetic H pixels from same nominal camera model','ideal PX4 LAND response'],
        elapsed_s=time.monotonic()-start)
    files=['tests/task_configured_sortie_replay.py','tests/test_task_handoff.py',
           'scripts/task_flight_controller.py','scripts/task_dispatch_contract.py',
           'scripts/roundtrip_flight_test.py','scripts/navigation_landing_shadow.py',
           'scripts/task_site_bounds.py','scripts/landing_projection.py','config/roundtrip_task.yaml']
    report['source_sha256']={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in files}
    out=ROOT/'evidence'/time.strftime('task_configured_replay_%Y%m%d_%H%M%S');out.mkdir()
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(evidence=str(out),**report),indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
