"""Conservative snapshot A* audit. Unknown is blocked, no ROS or control.

Circular envelope covers all yaw angles; success only describes this static 2D
slice and does not authorize entry from an unobserved start or certify 3D safety.
"""
import argparse
import heapq
import json
import math
import numpy as np
from scipy.ndimage import distance_transform_edt


def audit(path,start,goal,length=.6,width=.6,margin=.05):
    z=np.load(path)
    a=z['data'].reshape(int(z['height']),int(z['width']))
    resolution=float(z['resolution']);origin=z['origin'];unknown=float(z['unknown'])
    known=np.isfinite(a)&(a!=unknown)
    free=known&(a>0)
    # Padding ensures the map border is never treated as unbounded free space.
    distance=distance_transform_edt(np.pad(free,1,constant_values=False))[1:-1,1:-1]*resolution
    radius=.5*math.hypot(length,width)+margin+resolution/math.sqrt(2)
    allowed=free&(distance>radius)
    def cell(p):return (int(math.floor((p[1]-origin[1])/resolution)),int(math.floor((p[0]-origin[0])/resolution)))
    def valid(p):return 0<=p[0]<a.shape[0] and 0<=p[1]<a.shape[1] and bool(allowed[p])
    s,g=cell(start),cell(goal)
    report={'flight_validated':False,'start_m':start,'goal_m':goal,'clearance_radius_m':radius,
            'start_valid':valid(s),'goal_valid':valid(g),'path_found':False}
    if not valid(s) or not valid(g):return report
    queue=[(0,s)];cost={s:0};parent={}
    while queue:
        _,p=heapq.heappop(queue)
        if p==g:
            cells=[p]
            while p!=s:p=parent[p];cells.append(p)
            cells.reverse()
            report.update(path_found=True,path_m=[[float(origin[0]+(c[1]+.5)*resolution),
                         float(origin[1]+(c[0]+.5)*resolution)] for c in cells])
            return report
        for dy,dx in ((1,0),(-1,0),(0,1),(0,-1)):
            q=(p[0]+dy,p[1]+dx)
            if not valid(q):continue
            new=cost[p]+1
            if new<cost.get(q,math.inf):
                cost[q]=new;parent[q]=p
                heapq.heappush(queue,(new+abs(q[0]-g[0])+abs(q[1]-g[1]),q))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('slice');p.add_argument('--start',nargs=2,type=float,default=[0,0]);p.add_argument('--goal',nargs=2,type=float,required=True)
    args=p.parse_args();print(json.dumps(audit(args.slice,args.start,args.goal),indent=2))
