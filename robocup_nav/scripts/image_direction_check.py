"""Compare a stationary camera's images of a translated paper target.

No calibration writes. A match alone does not establish body axes: the operator
must identify the physical translation, keep the aircraft fixed, and repeat
on the second axis. Pixel motion is not a metric calibration.
"""
import argparse
import json
import math
import cv2
import numpy as np


def compare(before,after):
    out=dict(translation_accepted=False,optical_axes_verified=False,
             flight_authorized=False)
    def reject(reason):return dict(out,reason=reason)
    if before is None or after is None:return reject('missing image')
    if before.shape!=after.shape or before.dtype!=np.uint8 or after.dtype!=np.uint8:
        return reject('image modes differ or invalid encoding')
    if before.ndim==3 and before.shape[2]==3:
        before=cv2.cvtColor(before,cv2.COLOR_BGR2GRAY)
        after=cv2.cvtColor(after,cv2.COLOR_BGR2GRAY)
    if before.ndim!=2:return reject('expected grayscale or BGR image')
    points=cv2.goodFeaturesToTrack(before,300,.02,10,blockSize=7)
    if points is None or len(points)<12:return reject('insufficient features')
    opts=dict(winSize=(31,31),maxLevel=4,
              criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,40,.01))
    moved,status,_=cv2.calcOpticalFlowPyrLK(before,after,points,None,**opts)
    if moved is None or status is None:return reject('forward flow failed')
    back,back_status,_=cv2.calcOpticalFlowPyrLK(after,before,moved,None,**opts)
    if back is None or back_status is None:return reject('backward flow failed')
    p=points.reshape(-1,2);q=moved.reshape(-1,2)
    h,w=before.shape
    keep=(status.ravel()==1)&(back_status.ravel()==1)
    keep &= np.linalg.norm(back.reshape(-1,2)-p,axis=1)<1.
    keep &= np.isfinite(q).all(axis=1)&(q[:,0]>=0)&(q[:,0]<w)&(q[:,1]>=0)&(q[:,1]<h)
    p,q=p[keep],q[keep]
    if len(p)<12:return reject('insufficient consistent matches')
    matrix,inliers=cv2.estimateAffinePartial2D(p,q,method=cv2.RANSAC,ransacReprojThreshold=2.)
    if matrix is None or inliers is None:return reject('motion fit failed')
    use=inliers.ravel().astype(bool);count=int(use.sum())
    if count<12 or count/len(p)<.8:return reject('inconsistent scene motion')
    p,q=p[use],q[use]
    scale=math.hypot(matrix[0,0],matrix[1,0])
    angle=math.degrees(math.atan2(matrix[1,0],matrix[0,0]))
    displacement=np.median(q-p,axis=0)
    residual=float(np.quantile(np.linalg.norm(q-p-displacement,axis=1),.9))
    out.update(matches=count,scale=scale,rotation_deg=angle,
               displacement_px=displacement.tolist(),translation_residual_p90_px=residual)
    if abs(scale-1)>.04 or abs(angle)>3 or residual>3:
        return reject('rotation/scale/nontranslation exceeds limit')
    if np.ptp(p[:,0])<w*.2 or np.ptp(p[:,1])<h*.2:
        return reject('features too spatially concentrated')
    length=float(np.linalg.norm(displacement))
    if length<5 or length>min(h,w)*.35:return reject('movement too small or too large')
    dx,dy=displacement
    # Require a dominant axis, otherwise report that the mounting is oblique.
    if max(abs(dx),abs(dy))<2*min(abs(dx),abs(dy)):
        return reject('oblique movement; no cardinal-axis conclusion')
    side=('right' if dx>0 else 'left') if abs(dx)>abs(dy) else ('down' if dy>0 else 'up')
    return dict(out,translation_accepted=True,image_direction=side,
                reason='image displacement only; requires physical direction confirmation')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('before');parser.add_argument('after')
    args=parser.parse_args()
    print(json.dumps(compare(cv2.imread(args.before),cv2.imread(args.after)),indent=2))
