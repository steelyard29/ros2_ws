"""Calibrated image ray/ground-plane intersection, pure geometry, no commands.

Inputs must describe the actual image mode and verified optical-to-body axes.
Camera offset alone is insufficient. Ground plane must be in the same odom
session. Caller must gate pose/target freshness and localization resets.
"""
import math
import numpy as np
import cv2


def nominal_landing_clearance(geometry):
    """Confirmed concentric motor-circle/ring dimensions, NOT an error budget."""
    if not isinstance(geometry,dict) or geometry.get('motor_circle_center_matches_px4') is not True:
        raise ValueError('motor circle center relative to PX4 is not confirmed')
    ring=geometry.get('ring_inner_diameter_m');motor=geometry.get('motor_diagonal_m')
    if (any(type(v) not in (int,float) or not math.isfinite(v) for v in (ring,motor))
            or not 0<motor<ring):
        raise ValueError('positive ring clearance required')
    return (ring-motor)/2


def rotation_matrix(value):
    r=np.asarray(value,dtype=float)
    if r.shape!=(3,3) or not np.isfinite(r).all() or not np.allclose(r.T@r,np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(r),1,atol=1e-5):
        raise ValueError('proper finite rotation required')
    return r


def target_on_ground(pixel,k,distortion,image_size,calibration_size,
                     body_position,odom_from_body,body_from_optical,
                     camera_offset,ground_z,*,calibration_verified=False,
                     axes_verified=False):
    if not calibration_verified or not axes_verified:
        raise ValueError('calibration and optical axes must be independently verified')
    if tuple(image_size)!=tuple(calibration_size):
        raise ValueError('image mode differs from calibration')
    K=np.asarray(k,dtype=float);d=np.asarray(distortion,dtype=float)
    p=np.asarray(pixel,dtype=float);body=np.asarray(body_position,dtype=float);offset=np.asarray(camera_offset,dtype=float)
    if (K.shape!=(3,3) or p.shape!=(2,) or body.shape!=(3,) or offset.shape!=(3,)
            or not all(np.isfinite(x).all() for x in (K,d,p,body,offset)) or not math.isfinite(ground_z)
            or K[0,0]<=0 or K[1,1]<=0 or not np.allclose(K[2],[0,0,1])):
        raise ValueError('invalid projection input')
    if not 0<=p[0]<image_size[0] or not 0<=p[1]<image_size[1]:
        raise ValueError('pixel outside image')
    R=rotation_matrix(odom_from_body);C=rotation_matrix(body_from_optical)
    uv=cv2.undistortPoints(p.reshape(1,1,2),K,d).reshape(2)
    direction=R@C@np.array([*uv,1.])
    origin=body+R@offset
    if origin[2]<=ground_z+.05 or direction[2]>=-.2:
        raise ValueError('ground plane/ray geometry unsafe')
    t=(ground_z-origin[2])/direction[2]
    point=origin+t*direction
    if np.linalg.norm(point[:2]-body[:2])>3:
        raise ValueError('ground intersection outside local landing envelope')
    return point


class AlignmentPreview:
    """Preview velocity only. Not a flight controller or descent authorization."""
    def __init__(self,tolerance=.05):
        if not math.isfinite(tolerance) or not 0<tolerance<=.05:
            raise ValueError('reviewed alignment tolerance within 0..0.05m required')
        self.tolerance=tolerance
        self.since=None
        self.last=None

    def step(self,now,target_at,odom_at,error_xy,velocity_xy,*,boundary_verified=False):
        def invalid():
            self.since=None
            self.last=None
            return {'velocity_xy':[0.,0.],'aligned':False,'descent_allowed':False}
        try:
            error=np.asarray(error_xy,dtype=float)
            velocity=np.asarray(velocity_xy,dtype=float)
            clocks=np.asarray([now,target_at,odom_at],dtype=float)
        except (ValueError,TypeError,OverflowError):
            return invalid()
        if (error.shape!=(2,) or velocity.shape!=(2,) or clocks.shape!=(3,)
                or not all(np.isfinite(a).all() for a in (error,velocity,clocks))):
            return invalid()
        now,target_at,odom_at=map(float,clocks)
        if self.last is not None and not 0<now-self.last<=.3:
            self.since=None
        self.last=now
        if not 0<=now-target_at<=.25 or not 0<=now-odom_at<=.25:
            self.since=None
            return {'velocity_xy':[0.,0.],'aligned':False,'descent_allowed':False}
        if np.linalg.norm(error)>2:
            self.since=None
            return {'velocity_xy':[0.,0.],'aligned':False,'descent_allowed':False}
        v=.3*error;speed=np.linalg.norm(v)
        if speed>.1:v*=.1/speed
        aligned=np.linalg.norm(error)<self.tolerance and np.linalg.norm(velocity)<.05
        if not aligned:self.since=None
        elif self.since is None:self.since=now
        settled=bool(aligned and now-self.since>=2)
        return {'velocity_xy':v.tolist(),'aligned':settled,
                'boundary_verified':bool(boundary_verified),
                'descent_allowed':False}  # Real descent has separate state/authority.
