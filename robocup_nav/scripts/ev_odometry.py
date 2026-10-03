"""Prop-off EV input contract for PX4 `/fmu/in/vehicle_visual_odometry`.

Matches the bench that passed left 70 cm, yaw isolation, and up 45 cm:
visual-only cuVSLAM, local FRD pose, NaN velocity, ROS-clock timestamps,
no range Z, no timesync offset, no yaw-to-PX4 heading fit. Quality follows
cuVSLAM `vo_state` and motion step (not an unconditional 100). Pose
variances stay a conservative floor and inflate when quality drops.
EKF2_EV_POS stays 0 because VIO already publishes odom→base_link.

This module has no ROS or device side effects. It is not flight approval.
Do not use src/px4_interface vslam_odom_bridge (range-as-Z, timesync offset,
align_yaw_to_px4).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Mapping, Optional, Sequence

import numpy as np

from alignment_math import LocalFRD

POSE_FRAME_FRD = 2
VELOCITY_FRAME_UNKNOWN = 0
VELOCITY_FRAME_FRD = 2
ARMING_STATE_ARMED = 2
EV_TOPIC = '/fmu/in/vehicle_visual_odometry'
ALLOWED_IN_PREFIX = '/fmu/in/'
POSITION_VARIANCE = (0.01, 0.01, 0.04)
ORIENTATION_VARIANCE = (0.01, 0.01, 0.02)
# Match live EKF2_EV_QMIN=50; this module does not write PX4 parameters.
EV_QMIN = 50
QUALITY_HEALTHY = 100
QUALITY_DEGRADED = 70
QUALITY_REJECT = EV_QMIN - 1
QUALITY_LOST = 0
QUALITY = QUALITY_HEALTHY
MAX_STEP_M = 0.3
MAX_STEP_RAD = 0.5
MAX_STEP_DT_S = 0.5
MIN_HEALTHY_PREVIEWS = 30
VO_STATE_SUCCESS = 1


@dataclass(frozen=True)
class EvVisualOdometry:
    timestamp: int
    timestamp_sample: int
    position: tuple[float, float, float]
    q: tuple[float, float, float, float]
    pose_frame: int = POSE_FRAME_FRD
    velocity_frame: int = VELOCITY_FRAME_FRD
    velocity: tuple[float, float, float] = (math.nan, math.nan, math.nan)
    angular_velocity: tuple[float, float, float] = (math.nan, math.nan, math.nan)
    position_variance: tuple[float, float, float] = POSITION_VARIANCE
    orientation_variance: tuple[float, float, float] = ORIENTATION_VARIANCE
    velocity_variance: tuple[float, float, float] = (math.nan, math.nan, math.nan)
    reset_counter: int = 0
    quality: int = QUALITY


def ev_timestamps_us(now_ns: int, sample_ns: int, timesync_offset_us: int = 0) -> tuple[int, int]:
    """ROS-clock microseconds. SYNCT=1; ignore any timesync offset."""
    del timesync_offset_us
    return int(now_ns) // 1000, int(sample_ns) // 1000


def ev_position_frd(position_frd: Sequence[float], dist_bottom: Optional[float] = None) -> tuple[float, float, float]:
    """Keep VIO height. Range is a separate EKF source and must not replace Z."""
    del dist_bottom
    x, y, z = (float(v) for v in position_frd)
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise ValueError('invalid FRD position')
    return x, y, z


def motion_step(previous: Optional[tuple], stamp_ns: int, position, rotation_matrix):
    """Return (dt_s or None, displacement_m, angle_rad). None dt means first sample."""
    if previous is None:
        return None, 0.0, 0.0
    old_stamp, old_position, old_rotation = previous
    dt = (stamp_ns - old_stamp) / 1e9
    angle = math.acos(float(np.clip((np.trace(old_rotation.T @ rotation_matrix) - 1) / 2, -1, 1)))
    displacement = float(np.linalg.norm(np.asarray(position) - np.asarray(old_position)))
    return dt, displacement, angle


def discontinuity(previous: Optional[tuple], stamp_ns: int, position, rotation_matrix) -> Optional[str]:
    """Latch on a >0.3 m or >0.5 rad step within 0.5 s. None means accept."""
    dt, displacement, angle = motion_step(previous, stamp_ns, position, rotation_matrix)
    if dt is None:
        return None
    if dt <= 0:
        return 'timestamp moved backwards or duplicated'
    if dt > MAX_STEP_DT_S:
        return None
    if displacement > MAX_STEP_M or angle > MAX_STEP_RAD:
        return 'VIO discontinuity; new session required'
    return None


def ev_quality(*, vo_state: int, position_step_m: float = 0.0, angle_step_rad: float = 0.0,
               dt_s: Optional[float] = None) -> int:
    """Map cuVSLAM tracking to PX4 EV quality 0..100. vo_state 1 is Success."""
    if int(vo_state) != VO_STATE_SUCCESS:
        return QUALITY_LOST
    if dt_s is not None and (not math.isfinite(dt_s) or dt_s <= 0.0 or dt_s > MAX_STEP_DT_S):
        return QUALITY_REJECT
    if position_step_m > 0.5 * MAX_STEP_M or angle_step_rad > 0.5 * MAX_STEP_RAD:
        return QUALITY_REJECT
    if position_step_m > 0.25 * MAX_STEP_M or angle_step_rad > 0.25 * MAX_STEP_RAD:
        return QUALITY_DEGRADED
    return QUALITY_HEALTHY


def ev_variances(quality: int) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Conservative pose-variance floor; inflate when tracking quality drops."""
    if quality >= QUALITY_HEALTHY:
        scale = 1.0
    elif quality >= QUALITY_DEGRADED:
        scale = 4.0
    else:
        scale = 16.0
    position = tuple(v * scale for v in POSITION_VARIANCE)
    orientation = tuple(v * scale for v in ORIENTATION_VARIANCE)
    return position, orientation


def should_publish_ev(quality: int) -> bool:
    """Do not send samples PX4 would reject with EKF2_EV_QMIN=50."""
    return int(quality) >= EV_QMIN


def inhibit_reason(*, armed: bool = False, cs_ev_hgt: bool = False, cs_ev_vel: bool = False,
                   allow_ev_hgt: bool = False, vio_fault: Optional[str] = None,
                   extra_inputs: Iterable[str] = ()) -> Optional[str]:
    extras = [topic for topic in extra_inputs if topic]
    if extras:
        return 'Existing flight-input publisher: ' + ', '.join(extras)
    if armed:
        return 'PX4 reports armed; stopping injection'
    if cs_ev_hgt and not allow_ev_hgt:
        return 'EV height fusion active; stopping injection'
    if cs_ev_vel:
        return 'EV velocity fusion active; stopping injection'
    if vio_fault:
        return vio_fault
    return None


def extra_flight_inputs(publisher_counts: Mapping[str, int], own_ev_publishers: int = 1) -> list[str]:
    extras = []
    for topic, count in publisher_counts.items():
        if not topic.startswith(ALLOWED_IN_PREFIX) or count <= 0:
            continue
        if topic == EV_TOPIC and count <= own_ev_publishers:
            continue
        extras.append(topic)
    return extras


def build_ev_odometry(position_frd, q_wxyz, now_ns: int, sample_ns: int,
                      vo_state: int, timesync_offset_us: int = 0,
                      dist_bottom: Optional[float] = None,
                      position_step_m: float = 0.0, angle_step_rad: float = 0.0,
                      dt_s: Optional[float] = None) -> EvVisualOdometry:
    timestamp, timestamp_sample = ev_timestamps_us(now_ns, sample_ns, timesync_offset_us)
    position = ev_position_frd(position_frd, dist_bottom)
    q = tuple(float(v) for v in q_wxyz)
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError('invalid quaternion')
    quality = ev_quality(vo_state=vo_state, position_step_m=position_step_m,
                         angle_step_rad=angle_step_rad, dt_s=dt_s)
    position_variance, orientation_variance = ev_variances(quality)
    return EvVisualOdometry(timestamp=timestamp, timestamp_sample=timestamp_sample,
                            position=position, q=q, quality=quality,
                            position_variance=position_variance,
                            orientation_variance=orientation_variance)


def apply_to_vehicle_odometry(msg, ev: EvVisualOdometry) -> None:
    msg.timestamp = ev.timestamp
    msg.timestamp_sample = ev.timestamp_sample
    msg.pose_frame = ev.pose_frame
    msg.position = list(ev.position)
    msg.q = list(ev.q)
    msg.velocity_frame = ev.velocity_frame
    msg.velocity = list(ev.velocity)
    msg.angular_velocity = list(ev.angular_velocity)
    msg.position_variance = list(ev.position_variance)
    msg.orientation_variance = list(ev.orientation_variance)
    msg.velocity_variance = list(ev.velocity_variance)
    msg.reset_counter = ev.reset_counter
    msg.quality = ev.quality


def session_convert(frame: Optional[LocalFRD], position_xyz, quaternion_xyzw):
    """Create the session FRD frame on first valid pose, then convert."""
    p = [float(v) for v in position_xyz]
    q = [float(v) for v in quaternion_xyzw]
    validated, matrix = LocalFRD.validate(p, q)
    if frame is None:
        frame = LocalFRD(p, q)
    position, orientation = frame.convert(p, q)
    return frame, validated, matrix, position, orientation
