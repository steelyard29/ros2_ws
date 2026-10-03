"""ROS-free, fail-closed session gate. A fault requires a NEW reviewed session.

Thresholds are bench defaults, not validated airborne performance limits.
No reset/rearm method: visual recovery alone must never resume publication.
"""
import math


class EvSessionGuard:
    def __init__(self, started, warmup=30):
        self.started = started
        self.warmup = warmup
        self.count = 0
        self.fault = None
        self.tracking_at = None
        self.tracking_stamp = None
        self.pose_at = None
        self.pose_stamp = None

    def trip(self, reason):
        self.fault = self.fault or reason
        return False

    def tracking(self, state, stamp, now):
        if self.fault:
            return False
        if state != 1:
            return self.trip('Tracking failed; new session required')
        if stamp <= 0 or (self.tracking_stamp is not None and stamp <= self.tracking_stamp):
            return self.trip('Tracking timestamp reset/duplicate')
        self.tracking_at, self.tracking_stamp = now, stamp
        return True

    def watchdog(self, now, idle_s=0.3):
        if self.fault:
            return False
        if not math.isfinite(now) or not math.isfinite(idle_s) or idle_s <= 0:
            return self.trip('Invalid monotonic clock')
        # Host silence only. A sample already in hand is judged by source interval.
        for label, receipt in [('Tracking', self.tracking_at), ('VIO', self.pose_at)]:
            if receipt is not None and now - receipt > idle_s:
                return self.trip(label + ' stale; new session required')
        if now - self.started > 10 and (self.tracking_at is None or self.pose_at is None):
            return self.trip('Startup data timeout')
        return True

    def pose(self, stamp, age_ms, now, frame_ok=True):
        """Accept a sample already in hand from its source interval.

        The idle watchdog still treats 0.3 s without any sample as a stopped
        stream. A host scheduling gap must not discard a source-continuous pose.
        """
        if self.fault:
            return False
        if not math.isfinite(now):
            return self.trip('Invalid monotonic clock')
        if not frame_ok or not math.isfinite(age_ms) or not -50 <= age_ms <= 400:
            return self.trip('Invalid VIO frame or sample age')
        if stamp <= 0 or (self.pose_stamp is not None and stamp <= self.pose_stamp):
            return self.trip('VIO timestamp reset/duplicate')
        if self.pose_stamp is not None and (stamp - self.pose_stamp) / 1e9 > 0.3:
            return self.trip('VIO stale; new session required')
        if self.tracking_stamp is None:
            return False
        if not -50 <= (stamp - self.tracking_stamp) / 1e6 <= 300:
            return self.trip('Tracking/VIO timestamps disagree')
        self.pose_at, self.pose_stamp = now, stamp
        self.count += 1
        return self.count >= self.warmup
