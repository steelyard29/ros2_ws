"""Exact takeoff-aligned rectangle; no outer-AABB expansion or ROS access."""
from dataclasses import dataclass
import math


def valid_limits(values):
    if (not isinstance(values,(list,tuple)) or len(values)!=4
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in values)
            or values[0]>=values[1] or values[2]>=values[3]):
        raise ValueError('finite ordered robot-center bounds required')
    return tuple(values)


@dataclass(frozen=True)
class TakeoffRectangle:
    origin: tuple
    yaw: float
    limits: tuple

    def __post_init__(self):
        if (len(self.origin)!=2 or not all(math.isfinite(x) for x in self.origin)
                or not math.isfinite(self.yaw)):
            raise ValueError('finite takeoff XY and heading required')
        object.__setattr__(self,'origin',tuple(self.origin))
        object.__setattr__(self,'limits',valid_limits(self.limits))

    def local(self,point):
        x,y=point[0]-self.origin[0],point[1]-self.origin[1]
        c,s=math.cos(self.yaw),math.sin(self.yaw)
        return (c*x+s*y,-s*x+c*y)


def validate_bounds(bounds):
    if type(bounds) is not TakeoffRectangle:valid_limits(bounds)
    return bounds


def inside_bounds(bounds,point,extra_margin=0.):
    try:
        if (len(point)!=2 or not all(math.isfinite(v) for v in point)
                or not math.isfinite(extra_margin) or extra_margin<0):return False
        validate_bounds(bounds)
        x,y=bounds.local(point) if type(bounds) is TakeoffRectangle else point
        a,b,c,d=bounds.limits if type(bounds) is TakeoffRectangle else bounds
        return a+extra_margin<x<b-extra_margin and c+extra_margin<y<d-extra_margin
    except (ValueError,TypeError,IndexError):return False


def resolve_bounds(config,origin,yaw):
    """Explicit legacy odom limits or a complete operator physical-site survey.

    Physical boundaries are inset by body radius + position uncertainty exactly
    once. Additional stopping excursion is checked separately at runtime.
    """
    if config.get('bounds_odom') is not None:return valid_limits(config['bounds_odom'])
    survey=config.get('site_measurement',{})
    if survey.get('reference')!='takeoff_px4_center_with_takeoff_heading':
        raise ValueError('takeoff-frame site measurement required')
    distances=survey.get('distances_to_physical_boundary_m',{})
    values=[distances.get(k) for k in ('forward','backward','left','right')]
    if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in values):
        raise ValueError('four positive measured site distances required')
    body=config['stopping']['body_radius'];uncertainty=config['stopping']['uncertainty']
    if any(type(v) not in (int,float) or not math.isfinite(v) or v<0 for v in (body,uncertainty)) or body<=0:
        raise ValueError('finite positive body radius and uncertainty required')
    inset=body+uncertainty;forward,backward,left,right=values
    return TakeoffRectangle(tuple(origin),yaw,(-backward+inset,forward-inset,-right+inset,left-inset))
