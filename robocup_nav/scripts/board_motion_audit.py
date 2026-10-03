"""Offline continuous feature audit of an authorized board translation capture.

Tracks original corners without reseeding (periodic-board ambiguity), rejects
frame gaps/jumps/FB failures. Projection uses the declared nominal plane and
PROVISIONAL axes, never updates flight configuration or certifies full extrinsics.
"""
import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


def audit(capture, start_index=49):
    cv2.setNumThreads(1)
    rows = json.loads((capture/'frames.json').read_text())
    cal = yaml.safe_load((ROOT/'config/downward_camera_info.yaml').read_text())
    K = np.array(cal['camera_matrix']['data']).reshape(3, 3)
    D = np.array(cal['distortion_coefficients']['data'])
    first = cv2.imread(str(capture/rows[start_index]['file']), cv2.IMREAD_GRAYSCALE)
    if first is None or first.shape != (720, 1280):
        raise ValueError('calibrated baseline image unavailable')
    mask = np.zeros_like(first)
    mask[25:695, 230:1250] = 255  # Board interior, excludes printed left border.
    p = cv2.goodFeaturesToTrack(first, maxCorners=120, qualityLevel=.04,
                               minDistance=25, mask=mask, blockSize=7)
    if p is None or len(p) < 8:
        raise ValueError('too few baseline features')
    initial = p.copy()
    before = first
    previous = rows[start_index]
    history = []
    failures = []
    def plane_points(points):
        rays = cv2.undistortPoints(points, K, D).reshape(-1, 2)
        # Physical lens nominal 85mm above ground, board top 10mm above ground.
        # Provisional image-up=forward, image-right=starboard. Not accepted TF.
        return np.column_stack((-rays[:, 1], -rays[:, 0]))*.075
    for index in range(start_index+1, len(rows)):
        row = rows[index]
        gap = row['at']-previous['at']
        if not 0 < gap <= .25:
            failures.append(dict(frame=index, reason='frame gap', gap_s=gap))
            break
        current = cv2.imread(str(capture/row['file']), cv2.IMREAD_GRAYSCALE)
        q, status, err = cv2.calcOpticalFlowPyrLK(before, current, p, None,
            winSize=(31,31), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT, 30, .01))
        back, back_status, _ = cv2.calcOpticalFlowPyrLK(current, before, q, None,
            winSize=(31,31), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT, 30, .01))
        fb = np.linalg.norm(back-p, axis=2).ravel()
        delta = np.linalg.norm(q-p, axis=2).ravel()
        xy = q.reshape(-1, 2)
        valid = ((status.ravel() == 1) & (back_status.ravel() == 1) & (fb < .7)
                 & (err.ravel() < 40.) & (delta < 60.) & np.isfinite(xy).all(axis=1)
                 & (xy[:,0] > 5) & (xy[:,0] < 1275) & (xy[:,1] > 5) & (xy[:,1] < 715))
        p, initial = q[valid], initial[valid]
        if len(p) < 8:
            failures.append(dict(frame=index, reason='lost original track support', surviving=len(p)))
            break
        shifts = plane_points(p)-plane_points(initial)
        median = np.median(shifts, axis=0)
        spread = np.median(np.linalg.norm(shifts-median, axis=1))
        history.append(dict(frame=index, at=row['at'], relative_s=row['relative_s'],
            tracked_points=len(p), forward_left_m=median.tolist(),
            median_point_scatter_m=float(spread), max_fb_px=float(fb[valid].max()),
            max_step_px=float(delta[valid].max())))
        before, previous = current, row
    end = history[-1] if history else None
    return dict(capture=str(capture), baseline_frame=start_index,
        initial_points=int(len(cv2.goodFeaturesToTrack(first, maxCorners=120, qualityLevel=.04,
            minDistance=25, mask=mask, blockSize=7))),
        tracked_frame_count=len(history), capture_frame_count=len(rows), final=end,
        failures=failures, expected_operator_forward_m=.034,
        nominal_plane_distance_m=.075, optical_axes_provisional=True,
        operator_motion_completion_confirmed=False, metric_extrinsics_verified=False,
        flight_authorized=False, full_capture_tracked=not failures and len(history)==len(rows)-start_index-1), history


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture', type=Path, required=True)
    args = p.parse_args()
    report, history = audit(args.capture)
    out = ROOT/'evidence'/time.strftime('board_motion_%Y%m%d_%H%M%S')
    out.mkdir(exist_ok=False)
    (out/'report.json').write_text(json.dumps(report, indent=2))
    (out/'track_history.json').write_text(json.dumps(history))
    print(out)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
