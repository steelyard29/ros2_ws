"""Passive same-session measurements. Never grants flight/control authority."""
from collections import deque
from types import SimpleNamespace
from task_pose_pairing import closest_local
from disarmed_ev_fusion_evidence import summarize


class ResidentObservation:
    def __init__(self,session):
        self.session=session;self.locals=deque(maxlen=128)
        self.flags=deque(maxlen=400);self.pairs=deque(maxlen=2000)
        self.attempts=0;self.local_count=0;self.flag_count=0

    def flags_message(self,m,now,now_ns):
        self.flag_count+=1
        self.flags.append(dict(at=now,stamp_us=int(m.timestamp),
            source_age_s=(now_ns-int(m.timestamp)*1000)/1e9,
            **{key:bool(getattr(m,field)) for key,field in (
                ('ev_position','cs_ev_pos'),('ev_yaw','cs_ev_yaw'),
                ('ev_height','cs_ev_hgt'),('ev_velocity','cs_ev_vel'),
                ('baro_height','cs_baro_hgt'),('range_height','cs_rng_hgt'))}))

    def local_message(self,m,now,armed,landed):
        self.local_count+=1
        self.locals.append((int(m.timestamp)*1000,SimpleNamespace(
            local_at=now,xy_valid=bool(m.xy_valid),z_valid=bool(m.z_valid),
            armed=armed,landed=landed,
            reset_counters=tuple(int(getattr(m,k)) for k in (
                'xy_reset_counter','z_reset_counter','heading_reset_counter',
                'vxy_reset_counter','vz_reset_counter')),
            position=[float(m.x),float(m.y),float(m.z)],heading=float(m.heading))))

    def pose(self,m,now,now_ns):
        self.attempts+=1
        if not self.locals:return
        stamp=int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
        pair=closest_local(stamp,self.locals,now_ns,now,self.locals[-1][1].reset_counters)
        if pair:
            p,q=m.pose.pose.position,m.pose.pose.orientation
            self.pairs.append(dict(at=now,vio_stamp_ns=stamp,px4_stamp_ns=pair[0],
                difference_ms=abs(stamp-pair[0])/1e6,vio_position=[p.x,p.y,p.z],
                vio_quaternion=[q.x,q.y,q.z,q.w],px4_position=pair[1].position,
                px4_heading=pair[1].heading,reset_counters=list(pair[1].reset_counters)))

    def report(self):
        return dict(session=self.session,flight_ready=False,
            scope='stationary source-timestamp pairing; not dynamic alignment or flight release',
            local_count=self.local_count,fusion_samples=list(self.flags),
            fusion=summarize(list(self.flags),truncated=self.flag_count>400),
            attempts=self.attempts,matched_retained=len(self.pairs),pairs=list(self.pairs))
