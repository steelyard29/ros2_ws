"""Saved endpoint images: match unique printed f/g labels, NOT repeated cells.

Patch coordinates refer to the visually inspected 2026-09-30 baseline only.
Reports correlation alternatives and nominal-plane projection; no accepted TF.
"""
import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


def consistent_estimate(matches):
    """Diagnostic only: never average contradictory label correspondences."""
    if len(matches) < 2:
        return None
    shifts = np.array([m['pixel_translation'] for m in matches])
    # 10 px is a diagnostic correspondence tolerance, not flight accuracy.
    if np.max(np.linalg.norm(shifts-shifts[0], axis=1)) > 10:
        return None
    if any(m['score'] < .85 or m['score']-m['alternative_score'] < .10 for m in matches):
        return None
    return np.median([m['nominal_forward_left_m'] for m in matches], axis=0)


def inspect_cell(first, last, K, D, square_m):
    if not np.isfinite(square_m) or not .001 <= square_m <= .1:
        raise ValueError('explicit measured square edge length required')
    # Same square beside printed g; manually identified in both original images.
    # These are diagnostic correspondences, not an automatic calibration.
    seeds = [np.array([[344,558],[478,560],[342,695],[477,697]],np.float32),
             np.array([[456,185],[588,186],[454,318],[587,319]],np.float32)]
    refined = []
    for img, seed in zip((first,last), seeds):
        p = cv2.cornerSubPix(img, seed.reshape(-1,1,2).copy(), (7,7), (-1,-1),
                            (cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,40,.001)).reshape(-1,2)
        if np.max(np.linalg.norm(p-seed,axis=1)) > 10:
            raise ValueError('manual seed refinement exceeded diagnostic limit')
        response = cv2.cornerMinEigenVal(img, 5)
        strength = [float(response[round(float(y)),round(float(x))]) for x,y in p]
        if min(strength) < .001:
            raise ValueError('weak manually seeded corner')
        refined.append(p)
    rays = [cv2.undistortPoints(p.reshape(-1,1,2),K,D).reshape(-1,2) for p in refined]
    changes = (rays[1]-rays[0])*.075
    motion = np.column_stack((-changes[:,1],-changes[:,0]))
    edges = [(0,1),(0,2),(1,3),(2,3)]
    distances = [[float(square_m/np.linalg.norm(r[j]-r[i])) for i,j in edges] for r in rays]
    return dict(correspondence='manually inspected square beside unique g label',
        refined_pixels=[p.tolist() for p in refined],
        pixel_translation=(refined[1]-refined[0]).tolist(),
        nominal_forward_left_m=motion.mean(axis=0).tolist(),
        corner_scatter_m=np.ptp(motion,axis=0).tolist(),
        measured_square_m=square_m,
        frontoparallel_edge_implied_distance_m=distances,
        caveat='75mm plane and body axes provisional; edge scale uses explicit measured square and frontoparallel board',
        metric_acceptance=False)


def match_patch(first, last, rect):
    x, y, w, h = rect
    patch = first[y:y+h, x:x+w]
    scores = cv2.matchTemplate(last, patch, cv2.TM_CCOEFF_NORMED)
    _, best, _, point = cv2.minMaxLoc(scores)
    other = scores.copy()
    px, py = point
    other[max(0, py-h//2):py+h//2+1, max(0, px-w//2):px+w//2+1] = -1
    _, runner_up, _, other_point = cv2.minMaxLoc(other)
    return dict(score=best, alternative_score=runner_up, alternative_location=list(other_point),
        source_rect=rect, endpoint_top_left=list(point),
        pixel_translation=[px-x, py-y], source_point=[x+w/2, y+h/2],
        endpoint_point=[px+w/2, py+h/2])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--endpoint', type=Path, required=True)
    parser.add_argument('--square-mm', type=float, required=True)
    parser.add_argument('--operator-forward-mm', type=float, required=True)
    args = parser.parse_args()
    if (not np.isfinite(args.square_mm) or not 1 <= args.square_mm <= 100
            or not np.isfinite(args.operator_forward_mm) or not 0 < args.operator_forward_mm <= 1000):
        parser.error('finite, explicit measured square and displacement required')
    cv2.setNumThreads(1)
    first = cv2.imread(str(args.baseline), cv2.IMREAD_GRAYSCALE)
    last = cv2.imread(str(args.endpoint), cv2.IMREAD_GRAYSCALE)
    if first is None or last is None or first.shape != (720,1280) or last.shape != first.shape:
        raise ValueError('original calibrated-size images required')
    cal = yaml.safe_load((ROOT/'config/downward_camera_info.yaml').read_text())
    K = np.array(cal['camera_matrix']['data']).reshape(3,3)
    D = np.array(cal['distortion_coefficients']['data'])
    matches = []
    for label, rect in [('f', [148,443,57,85]), ('g', [148,584,57,82])]:
        m = match_patch(first, last, rect)
        points = np.array([m['source_point'], m['endpoint_point']], dtype=np.float64).reshape(-1,1,2)
        rays = cv2.undistortPoints(points,K,D).reshape(-1,2)
        change = (rays[1]-rays[0])*.075
        m.update(label=label, nominal_forward_left_m=[float(-change[1]), float(-change[0])])
        matches.append(m)
    estimate = consistent_estimate(matches)
    report = dict(baseline=str(args.baseline), endpoint=str(args.endpoint), matches=matches,
        label_correspondence_consistent=estimate is not None,
        nominal_forward_left_m=None if estimate is None else estimate.tolist(),
        nominal_norm_m=None if estimate is None else float(np.linalg.norm(estimate)),
        manually_inspected_cell=inspect_cell(first,last,K,D,args.square_mm/1000.),
        reported_operator_forward_m=args.operator_forward_mm/1000., expected_plane_distance_m=.075,
        operator_motion_completion_confirmed=True, no_continuous_movement_record=True,
        optical_axes_provisional=True, independent_metric_acceptance=False,
        metric_extrinsics_verified=False, flight_authorized=False)
    out = ROOT/'evidence'/time.strftime('board_endpoint_%Y%m%d_%H%M%S')
    out.mkdir(exist_ok=False)
    (out/'report.json').write_text(json.dumps(report,indent=2))
    print(out)
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
