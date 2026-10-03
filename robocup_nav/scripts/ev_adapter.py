"""ROS-free EV publish adapter. Faults latch; there is no resume.

Used by `ev_bridge.py`. Fault injection and recorded-session replay live in
tests; this module has no device or ROS side effects.
"""
from __future__ import annotations

from typing import Optional, Sequence

from ev_odometry import (
    ARMING_STATE_ARMED,
    MIN_HEALTHY_PREVIEWS,
    build_ev_odometry,
    discontinuity,
    inhibit_reason,
    motion_step,
    session_convert,
    should_publish_ev,
)
from ev_session_guard import EvSessionGuard

TELEMETRY_STALE_S = 2.0
TRACKING_RECEIPT_S = 0.3


class EvAdapter:
    def __init__(self, started: float, allow_ev_hgt: bool = False, warmup: int = 30):
        self.allow_ev_hgt = bool(allow_ev_hgt)
        self.guard = EvSessionGuard(started, warmup=warmup)
        self.frame = None
        self.previous = None
        self.tracking = None
        self.fault = None
        self.inhibit = False
        self.preview_count = 0
        self.publish_count = 0
        self.disarmed = False
        self.vehicle_at = None
        self.flags_at = None
        self.quality_counts = {}

    def trip(self, reason: Optional[str]):
        if reason and self.fault is None:
            self.fault = reason
        self.inhibit = True
        return None

    def on_tracking(self, vo_state: int, stamp_ns: int, now: float) -> bool:
        if self.inhibit:
            return False
        if not self.guard.tracking(int(vo_state), int(stamp_ns), now):
            self.trip(self.guard.fault)
            return False
        self.tracking = ({'vo_state': int(vo_state), 'stamp_ns': int(stamp_ns)}, now)
        return True

    def on_vehicle(self, arming_state: int, now: float) -> bool:
        if self.inhibit:
            return False
        self.vehicle_at = now
        self.disarmed = int(arming_state) == 1
        if int(arming_state) == ARMING_STATE_ARMED:
            self.trip(inhibit_reason(armed=True))
            return False
        return True

    def on_flags(self, cs_ev_hgt: bool, cs_ev_vel: bool, now: float) -> bool:
        if self.inhibit:
            return False
        self.flags_at = now
        reason = inhibit_reason(cs_ev_hgt=bool(cs_ev_hgt), cs_ev_vel=bool(cs_ev_vel),
                                allow_ev_hgt=self.allow_ev_hgt)
        if reason:
            self.trip(reason)
            return False
        return True

    def extra_inputs(self, topics) -> bool:
        reason = inhibit_reason(extra_inputs=topics)
        if reason:
            self.trip(reason)
            return False
        return True

    def watchdog(self, now: float, idle_s: float = 0.3) -> bool:
        if self.inhibit:
            return False
        if not self.guard.watchdog(now, idle_s):
            self.trip(self.guard.fault)
            return False
        if self.publish_count and (self.vehicle_at is None or self.flags_at is None
                or now - self.vehicle_at > TELEMETRY_STALE_S
                or now - self.flags_at > TELEMETRY_STALE_S):
            self.trip('PX4 safety telemetry stale')
            return False
        return True

    def on_pose(self, stamp_ns: int, age_ms: float, now: float, position_xyz: Sequence[float],
                quat_xyzw: Sequence[float], frame_ok: bool = True,
                now_ns: Optional[int] = None):
        if self.inhibit:
            return None
        ready = self.guard.pose(int(stamp_ns), age_ms, now, frame_ok)
        if self.guard.fault:
            return self.trip(self.guard.fault)
        tr = self.tracking
        if tr is None or tr[0].get('vo_state') != 1:
            return None
        if now - tr[1] > TRACKING_RECEIPT_S:
            return None
        if not -50 <= (int(stamp_ns) - tr[0]['stamp_ns']) / 1e6 <= 300:
            return None
        try:
            self.frame, validated, matrix, position, orientation = session_convert(
                self.frame, position_xyz, quat_xyzw)
            dt, disp, ang = motion_step(self.previous, int(stamp_ns), validated, matrix)
            jump = discontinuity(self.previous, int(stamp_ns), validated, matrix)
            if jump == 'timestamp moved backwards or duplicated':
                return None
            if jump:
                return self.trip(jump)
            self.previous = (int(stamp_ns), validated, matrix)
        except ValueError:
            return self.trip('Invalid VIO pose')
        self.preview_count += 1
        if not ready or self.preview_count < MIN_HEALTHY_PREVIEWS:
            return None
        if (not self.vehicle_ready() or self.vehicle_at is None or self.flags_at is None
                or now - self.vehicle_at > TELEMETRY_STALE_S
                or now - self.flags_at > TELEMETRY_STALE_S):
            return None
        clock_ns = int(stamp_ns if now_ns is None else now_ns)
        ev = build_ev_odometry(position, orientation, clock_ns, int(stamp_ns),
                               vo_state=int(tr[0]['vo_state']),
                               position_step_m=disp, angle_step_rad=ang, dt_s=dt)
        self.quality_counts[str(ev.quality)] = self.quality_counts.get(str(ev.quality), 0) + 1
        if not should_publish_ev(ev.quality):
            return self.trip('Rejected EV quality; new session required')
        self.publish_count += 1
        return ev

    def vehicle_ready(self) -> bool:
        """Bench policy. Flight candidate overrides this without weakening bench."""
        return self.disarmed
