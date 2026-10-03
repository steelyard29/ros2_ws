"""Postprocess an existing sensor session; never starts ROS or hardware."""
import argparse
import json
from pathlib import Path
import math
import numpy as np
from scipy.ndimage import distance_transform_edt
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from map_corridor_check import audit


def main():
    p=argparse.ArgumentParser();p.add_argument('evidence',type=Path);args=p.parse_args()
    out=args.evidence;report=json.loads((out/'report.json').read_text())
    timing=json.loads((out/'stream_timing.json').read_text())
    z=np.load(out/'esdf_slice.npz');a=z['data'].reshape(int(z['height']),int(z['width']))
    res=float(z['resolution']);origin=z['origin'];unknown=float(z['unknown'])
    known=np.isfinite(a)&(a!=unknown);free=known&(a>0);obstacle=known&(a<=0)
    start=np.array(report['initial_position'][:2]);goal=np.array(report['candidate_goal']['position'][:2])
    rows,cols=np.indices(a.shape);xs=origin[0]+(cols+.5)*res;ys=origin[1]+(rows+.5)*res
    def sample(point):
        col=int(math.floor((point[0]-origin[0])/res));row=int(math.floor((point[1]-origin[1])/res))
        if not(0<=row<a.shape[0] and 0<=col<a.shape[1]):return 'outside map'
        return 'unknown' if not known[row,col] else 'free' if free[row,col] else 'obstacle'
    touched=(np.maximum(np.abs(xs-start[0])-res/2,0)**2+
             np.maximum(np.abs(ys-start[1])-res/2,0)**2)<=.48**2
    distance=distance_transform_edt(np.pad(free,1,constant_values=False))[1:-1,1:-1]*res
    allowed=free&(distance>.48+res/math.sqrt(2))
    good=np.argwhere(allowed)
    nearest=None
    if len(good):
        points=np.stack((origin[0]+(good[:,1]+.5)*res,origin[1]+(good[:,0]+.5)*res),axis=1)
        k=np.argmin(np.linalg.norm(points-start,axis=1));nearest=points[k].tolist()
    stats={}
    # Uniform startup exclusion is diagnostic only, not a lowered flight gate.
    steady_start=timing['vio'][0][0]+10
    for key,rs in timing.items():
        valid=np.array([r for r in rs if r[1]>0 and r[0]>=steady_start])
        if len(valid)<2:continue
        gaps=np.diff(valid[:,0]);stamp_gaps=np.diff(valid[:,1])/1e9
        stats[key]=dict(count=len(valid),receive_gap_max_s=float(gaps.max()),
            source_gap_max_s=float(stamp_gaps.max()),age_p95_max_s=np.quantile(valid[:,2],[.95,1]).tolist(),
            source_zero_stamp_count=sum(r[1]==0 for r in rs),receive_gaps_over_250ms=int(np.sum(gaps>.25)))
    result=dict(source=str(out/'esdf_slice.npz'),hardware_started=False,
        start_cell=sample(start),goal_cell=sample(goal),shape=list(a.shape),resolution_m=res,
        origin=origin.tolist(),start_envelope_radius_m=.48,
        start_envelope_cells=dict(total=int(touched.sum()),free=int((touched&free).sum()),
            unknown=int((touched&~known).sum()),obstacle=int((touched&obstacle).sum())),
        start_envelope_extends_outside_map=bool(start[0]-.48<origin[0] or start[1]-.48<origin[1]
            or start[0]+.48>origin[0]+a.shape[1]*res or start[1]+.48>origin[1]+a.shape[0]*res),
        nearest_static_clear_center=nearest,
        route_samples=[dict(distance_m=float(d),cell=sample(start+d*(goal-start)/2.5)) for d in np.arange(0,2.51,.25)],
        steady_after_vio_start_10s=stats,flight_ready=False,
        note='Known 2D projected cells alone do not prove every voxel in the body-height column observed.')
    result['snapshot_actual_start_route']=audit(out/'esdf_slice.npz',start.tolist(),goal.tolist())
    if nearest is not None:
        result['snapshot_observed_segment_only']=audit(out/'esdf_slice.npz',nearest,goal.tolist())
    (out/'snapshot_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
