"""Task-side external EV receipt evidence. Receipt is NOT PX4 EKF fusion.

Runtime must bind the expected publisher identity/session before use. No ROS,
publication, clock repair, synthetic freshness, or automatic rebind.
"""
import math


class ExternalEvEvidence:
    def __init__(self,session,publisher_gid):
        if not session or not publisher_gid:raise ValueError('session and publisher required')
        self.session=session;self.publisher_gid=tuple(publisher_gid)
        self.fault=None;self.last_at=None;self.last_sample=None;self.count=0
        self.reset_counter=None

    def stop(self,reason):
        self.fault=self.fault or reason
        return False

    def receive(self,session,gid,stamp_us,sample_us,quality,now_ns,now):
        if self.fault:return False
        if session!=self.session or tuple(gid)!=self.publisher_gid:
            return self.stop('external EV session/publisher changed')
        if (type(stamp_us) is not int or type(sample_us) is not int
                or min(stamp_us,sample_us)<=0 or not math.isfinite(now)
                or not -.05<=(now_ns-stamp_us*1000)/1e9<=.25
                or not -.05<=(now_ns-sample_us*1000)/1e9<=.25
                or not -.05<=(stamp_us-sample_us)/1e6<=.25
                or not 50<=quality<=100):
            return self.stop('external EV invalid quality/time')
        if self.last_sample is not None:
            if sample_us<self.last_sample:return self.stop('external EV source reset')
            if sample_us==self.last_sample:return False
            if (sample_us-self.last_sample)/1e6>.3:
                return self.stop('external EV source gap')
        self.last_sample=sample_us;self.last_at=now;self.count+=1
        return True

    def healthy(self,now):
        if self.fault or self.last_at is None:return False
        if not 0<=now-self.last_at<=.3:return self.stop('external EV receipt stale')
        return True

    def receive_message(self,msg,gid,now_ns,now):
        try:
            if (msg.pose_frame!=2 or msg.velocity_frame!=2
                    or len(msg.position)!=3 or len(msg.q)!=4
                    or not all(math.isfinite(v) for v in (*msg.position,*msg.q))
                    or abs(sum(v*v for v in msg.q)-1.)>.01
                    or not all(math.isnan(v) for v in msg.velocity)
                    or len(msg.velocity)!=3):
                return self.stop('external EV frame/payload mismatch')
            reset=int(msg.reset_counter)
            if self.reset_counter is not None and reset!=self.reset_counter:
                return self.stop('external EV reset counter changed')
            accepted=self.receive(self.session,gid,int(msg.timestamp),int(msg.timestamp_sample),
                                  int(msg.quality),now_ns,now)
            if accepted:self.reset_counter=reset
            return accepted
        except (AttributeError,TypeError,ValueError,OverflowError):
            return self.stop('external EV malformed message')
