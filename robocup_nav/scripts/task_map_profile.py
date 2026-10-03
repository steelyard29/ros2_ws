"""Single ground/cruise profile and evidence from nvblox's actual ESDF integrator.

Versioned source: nvblox_node::publishDebugVisualizations and heightLimitToMarker.
Markers contain min/max heights read from the integrator, not parameter echoes.
This proves height consistency, NOT observed-free volume or flight readiness.
"""
import math
from cruise_band import make_band

VOXEL_SIZE=.05
BOOTSTRAP_MARGIN=VOXEL_SIZE/2  # Conservative map expansion, not measured VIO accuracy.


def task_band(initial_z):
    return make_band(initial_z=initial_z,rise=.86,ground_clearance=.14)


def mapper_profile(initial_z):
    b=task_band(initial_z)
    return dict(center_z=b.center,band_below=b.center-b.lower+BOOTSTRAP_MARGIN,
                band_above=b.upper-b.center+BOOTSTRAP_MARGIN,bounds_rate_hz=5.)


class MapBandEvidence:
    def __init__(self):
        self.bounds={};self.pending={};self.center=None;self.slice_at=-1.;self.reason='no map/bounds evidence'

    def marker(self,m,source_at):
        kind=m.ns
        if kind not in ('bottom_height_limit','top_height_limit'):return False
        stamp=int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
        z=[float(p.z) for p in m.points]
        if (m.header.frame_id!='odom' or m.type!=11 or m.action!=0 or m.id!=0
                or stamp<=0 or len(z)!=6 or not all(math.isfinite(v) for v in z)
                or max(z)-min(z)>1e-6 or not math.isfinite(source_at)):
            self.bounds.pop(kind,None);self.pending.pop(kind,None)
            return False
        self.pending[kind]=(stamp,source_at,z[0])
        # Bottom and top are distinct DDS samples. Retain the last COMPLETE
        # fresh pair while the next pair is arriving; never pair two generations.
        if len(self.pending)==2 and len({x[0] for x in self.pending.values()})==1:
            self.bounds=dict(self.pending)
        return True

    def slice(self,center,source_at):
        self.center=float(center);self.slice_at=source_at

    def check(self,band,now):
        self.reason='map/bounds stale, missing or mismatched pair'
        bottom=self.bounds.get('bottom_height_limit');top=self.bounds.get('top_height_limit')
        if (not math.isfinite(now) or self.center is None or not math.isfinite(self.center)
                or not 0<=now-self.slice_at<=.5 or bottom is None or top is None
                or bottom[0]!=top[0] or not all(0<=now-x[1]<=.75 for x in (bottom,top))):return False
        # Half a voxel of bootstrap offset is tolerated ONLY when the entire
        # required body band remains contained in the actual integrated band.
        if (abs(self.center-band.center)>BOOTSTRAP_MARGIN+1e-6
                or bottom[2]>band.lower+1e-6 or top[2]<band.upper-1e-6
                or not band.floor+.05<bottom[2]<top[2]):
            self.reason='actual mapper height band does not cover current ground/cruise geometry'
            return False
        self.reason='actual integrator bounds and map center match task';return True
