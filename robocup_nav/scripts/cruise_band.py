"""Ground-start, fixed cruise envelope; no vehicle outputs.

ground_clearance is ground-to-base_link at initialization, not lidar range.
Navigation must remain inhibited during ascent/descent. Changing band requires
a new mapper session because nvblox 3.2 mapper parameters are initialized once.
"""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class CruiseBand:
    center: float
    lower: float
    upper: float
    floor: float
    tolerance: float = .08

    def contains_vehicle(self, z, tilt_rad):
        return (math.isfinite(z) and math.isfinite(tilt_rad)
                and abs(z-self.center) < self.tolerance and abs(tilt_rad) < math.radians(10))


def make_band(initial_z=0., rise=.6, ground_clearance=.15,
              above=.115, below=.15, length=.60, width=.60, margin=.05):
    values = (initial_z,rise,ground_clearance,above,below,length,width,margin)
    if not all(math.isfinite(v) for v in values) or min(values[1:]) <= 0:
        raise ValueError('finite positive measured geometry required')
    if not .2 <= rise <= 1.3:
        raise ValueError('cruise rise outside reviewed 0.2..1.3 m envelope')
    tilt = .5*math.hypot(length,width)*math.sin(math.radians(10))
    center = initial_z+rise
    extra = tilt+margin+.08
    band = CruiseBand(center,center-below-extra,center+above+extra,initial_z-ground_clearance)
    if band.lower <= band.floor+.05:
        raise ValueError('cruise collision band intersects ground clearance')
    return band
