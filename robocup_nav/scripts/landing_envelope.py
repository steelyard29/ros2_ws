"""Metric landing-circle containment budget, no flight command or authorization.

Motor centers are used for the scoring geometry, NOT obstacle clearance.
Prop guards/full body still determine physical collision clearance separately.
All error bounds must be independently established, not chosen to make a fit.
"""
import math
import numpy as np


def containment(motor_xy,center_error_xy,ring_inner_diameter,*,
                target_error_bound,localization_error_bound,descent_drift_bound,
                geometry_verified=False):
    refused={'contained':False,'flight_authorized':False,'reason':'unverified/invalid geometry'}
    if not geometry_verified:return refused
    try:
        motors=np.asarray(motor_xy,float);offset=np.asarray(center_error_xy,float)
        bounds=np.asarray([target_error_bound,localization_error_bound,descent_drift_bound],float)
        if (motors.shape!=(4,2) or offset.shape!=(2,) or bounds.shape!=(3,)
            or not all(np.isfinite(v).all() for v in (motors,offset,bounds))
            or (bounds<0).any() or not math.isfinite(ring_inner_diameter) or ring_inner_diameter<=0):return refused
        if len(np.unique(motors,axis=0))!=4 or np.linalg.matrix_rank(motors-motors.mean(axis=0))<2:return refused
        # Conservative circle about the measured body/PX4 reference covers all
        # four centers, even if that reference is not the geometric center.
        radius=float(np.max(np.linalg.norm(motors,axis=1)))
        margin=ring_inner_diameter/2-radius-float(bounds.sum())
        error=float(np.linalg.norm(offset))
        return dict(contained=margin>0 and error<margin,flight_authorized=False,
                    motor_envelope_radius_m=radius,allowed_center_error_m=max(0.,margin),
                    current_center_error_m=error,remaining_margin_m=margin-error,
                    reason='scoring geometry only; not descent or obstacle approval')
    except (ValueError,TypeError,OverflowError):return refused
