"""Bound detector pixel workload, return coordinates in ORIGINAL image pixels.

No physical H size, range estimate or flight permission. Subpixel scaling uses
pixel-center convention; camera calibration must retain original image size.
"""
import cv2
import numpy as np
from h_target_geometry import candidates


def fast_candidates(image,max_dimension=640):
    if image is None or image.ndim!=3 or not isinstance(max_dimension,int) or max_dimension<320:
        raise ValueError('BGR image and max dimension >=320 required')
    h,w=image.shape[:2];scale=min(1.,max_dimension/max(h,w))
    if scale==1:return candidates(image)
    rw,rh=max(1,round(w*scale)),max(1,round(h*scale))
    small=cv2.resize(image,(rw,rh),interpolation=cv2.INTER_AREA)
    out=candidates(small);sx,sy=w/rw,h/rh
    for item in out:
        u,v=item['center_px'];item['center_px']=[(u+.5)*sx-.5,(v+.5)*sy-.5]
        x,y,bw,bh=item['bbox_px'];item['bbox_px']=[x*sx,y*sy,bw*sx,bh*sy]
    return out
