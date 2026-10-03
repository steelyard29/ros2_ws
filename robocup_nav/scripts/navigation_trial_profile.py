"""Offline-reviewed SHADOW trial profile. Does not authorize flight."""
import math


def projected_width(length, width, yaw_rad):
    if not all(math.isfinite(v) for v in (length, width, yaw_rad)) or min(length, width) <= 0:
        raise ValueError('finite positive measured envelope required')
    return abs(length * math.sin(yaw_rad)) + abs(width * math.cos(yaw_rad))


def required_clearance(speed, latency, braking_accel, uncertainty):
    if not all(math.isfinite(v) for v in (speed, latency, braking_accel, uncertainty)):
        raise ValueError('nonfinite stopping model')
    if min(speed, latency, uncertainty) < 0 or braking_accel <= 0:
        raise ValueError('invalid stopping model')
    return speed * latency + speed * speed / (2 * braking_accel) + uncertainty


def apply_trial_profile(config):
    """Mutates only supplied launch config, not the validated flight controller."""
    p = config['controller_server']['ros__parameters']['FollowPath']
    p.update(max_vel_x=0.15, max_speed_xy=0.15, max_vel_y=0.0,
             min_vel_y=0.0, max_vel_theta=0.20, acc_lim_x=0.20,
             decel_lim_x=-0.20, sim_time=2.0)
    # These are planning limits; actual vehicle response must be measured.
    return config
