"""Saved image -> board-relative PnP evidence, never writes camera extrinsics.

A partial checkerboard identifies a plane and grid spacing, NOT its physical
origin/heading relative to PX4. Both planar pose solutions are reported. Low
reprojection error alone is not independent metric/extrinsic verification.
"""
import argparse
import json
import math
from pathlib import Path
import time
import cv2
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image',type=Path,required=True)
    parser.add_argument('--square-mm',type=float,required=True)
    parser.add_argument('--board-thickness-mm',type=float,required=True)
    parser.add_argument('--columns',type=int,default=4)
    parser.add_argument('--rows',type=int,default=3)
    args=parser.parse_args()
    if not 1<=args.square_mm<=100 or not 0<=args.board_thickness_mm<=100:
        parser.error('finite plausible measured dimensions required')
    image=cv2.imread(str(args.image));cv2.setNumThreads(1)
    if image is None:raise ValueError('saved image missing')
    cal=yaml.safe_load((ROOT/'config/downward_camera_info.yaml').read_text())
    if image.shape[1::-1]!=(cal['image_width'],cal['image_height']):
        raise ValueError('image resolution does not match calibration')
    K=np.array(cal['camera_matrix']['data']).reshape(3,3)
    D=np.array(cal['distortion_coefficients']['data'])
    found,corners=cv2.findChessboardCornersSB(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),
        (args.columns,args.rows),flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:raise ValueError('requested corner grid not found; do not guess points')
    objects=np.zeros((args.columns*args.rows,3),np.float64)
    objects[:,:2]=np.mgrid[:args.columns,:args.rows].T.reshape(-1,2)*args.square_mm/1000.
    ok,rvecs,tvecs,_=cv2.solvePnPGeneric(objects,corners,K,D,flags=cv2.SOLVEPNP_IPPE)
    if not ok:raise ValueError('PnP did not return a solution')
    solutions=[]
    for rvec,tvec in zip(rvecs,tvecs):
        initial,_=cv2.projectPoints(objects,rvec,tvec,K,D)
        initial_rms=float(np.sqrt(np.mean(np.sum((initial.reshape(-1,2)-corners.reshape(-1,2))**2,axis=1))))
        rvec,tvec=cv2.solvePnPRefineLM(objects,corners,K,D,rvec.copy(),tvec.copy())
        R,_=cv2.Rodrigues(rvec);t=tvec.reshape(3)
        projected,_=cv2.projectPoints(objects,rvec,tvec,K,D)
        errors=np.linalg.norm(projected.reshape(-1,2)-corners.reshape(-1,2),axis=1)
        n=R[:,2];height=abs(float(n@t))
        solutions.append(dict(camera_from_board_rotation=R.tolist(),translation_camera_m=t.tolist(),
            initial_ippe_rms_px=initial_rms,lm_refined=True,
            all_points_in_front=bool(((R@objects.T).T+t)[:,2].min()>0),
            reprojection_rms_px=float(np.sqrt(np.mean(errors**2))),reprojection_max_px=float(errors.max()),
            optical_axis_to_board_normal_deg=math.degrees(math.acos(min(1.,abs(float(n[2]))))),
            camera_to_board_normal_distance_m=height,
            camera_to_ground_if_board_horizontal_m=height+args.board_thickness_mm/1000.))
    solutions.sort(key=lambda s:s['reprojection_rms_px'])
    report=dict(image=str(args.image.resolve()),square_mm=args.square_mm,
        board_thickness_mm=args.board_thickness_mm,inner_corner_pattern=[args.columns,args.rows],
        corners_px=corners.reshape(-1,2).tolist(),solutions=solutions,
        configured_unloaded_camera_ground_distance_m=.14-.055,
        partial_board=True,board_heading_relative_to_body_verified=False,
        board_normal_relative_to_body_verified=False,metric_extrinsics_verified=False,
        flight_authorized=False,configuration_written=False)
    out=ROOT/'evidence'/time.strftime('chessboard_plane_%Y%m%d_%H%M%S');out.mkdir(exist_ok=False)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(out);print(json.dumps(report,indent=2))


if __name__=='__main__':main()
