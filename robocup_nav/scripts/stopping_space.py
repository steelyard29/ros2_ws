"""Conservative local 2D stopping-space audit, NOT flight certification.

Requires a known-free full-height slice, not just positive raw ESDF values.
Uses an all-direction stopping disk to cover current/command direction changes.
Latency and achievable braking MUST be measured before real flight use.
"""
import math
import numpy as np


def select_stopping_safe_velocity(command_xy,check):
    """Choose a SLOWER candidate without changing the stopping-space criterion.

    `check` must include actual current velocity, whole body, uncertainty, data
    freshness and measured braking. Zero return means inhibit, NOT proof that
    the vehicle can stop in its current space. Caller retains final safety gate.
    """
    from navigation_limits import horizontal_velocity
    v=horizontal_velocity(command_xy)
    for scale in (1.,.75,.5,.25,.125):
        candidate=tuple(x*scale for x in v)
        if check(candidate):return candidate,scale
    return (0.,0.),0.


def stopping_space_clear(free, resolution, origin_xy, position_xy,
                         velocity_xy, command_xy, *, body_radius,
                         uncertainty, latency, braking):
    vectors=(origin_xy,position_xy,velocity_xy,command_xy)
    if any(len(v)!=2 or not all(math.isfinite(x) for x in v) for v in vectors):
        return False
    scalars=(resolution,body_radius,uncertainty,latency,braking)
    if not all(math.isfinite(x) for x in scalars):return False
    if min(resolution,body_radius,braking)<=0 or min(uncertainty,latency)<0:return False
    a=np.asarray(free)
    if a.ndim!=2 or a.dtype!=np.bool_ or not a.size:return False
    speed=max(math.hypot(*velocity_xy),math.hypot(*command_xy))
    radius=body_radius+uncertainty+speed*latency+speed*speed/(2*braking)
    x,y=position_xy;ox,oy=origin_xy
    # Reject any portion of the envelope beyond the observed map.
    if x-radius<=ox or y-radius<=oy or x+radius>=ox+a.shape[1]*resolution or y+radius>=oy+a.shape[0]*resolution:
        return False
    lo_x=int(math.floor((x-radius-ox)/resolution));hi_x=int(math.floor((x+radius-ox)/resolution))
    lo_y=int(math.floor((y-radius-oy)/resolution));hi_y=int(math.floor((y+radius-oy)/resolution))
    rows,cols=np.mgrid[lo_y:hi_y+1,lo_x:hi_x+1]
    # Exact distance to each closed cell square: touching is blocked.
    dx=np.maximum(np.abs(ox+(cols+.5)*resolution-x)-resolution/2,0)
    dy=np.maximum(np.abs(oy+(rows+.5)*resolution-y)-resolution/2,0)
    touched=dx*dx+dy*dy<=radius*radius
    return bool(np.all(a[lo_y:hi_y+1,lo_x:hi_x+1][touched]))
