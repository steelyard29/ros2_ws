"""Colour-independent H contour candidates. Offline/preview only, NOT landing permission.

Both grayscale polarities and multiple thresholds accommodate outlined/filled H.
No metric pose, ring containment, temporal tracking or flight outputs are claimed.
"""
import argparse
import json
import cv2
import numpy as np


def candidates(image):
    if image is None or image.ndim != 3:
        raise ValueError('BGR image required')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    height, width = gray.shape
    found = []
    for threshold in (40, 70, 100, 130, 160, 190, 220):
        for polarity in (cv2.THRESH_BINARY, cv2.THRESH_BINARY_INV):
            _, mask = cv2.threshold(gray, threshold, 255, polarity)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7,7), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                x,y,w,h = cv2.boundingRect(contour)
                if area < width*height*.005 or min(w,h) < 20:
                    continue
                if x <= 1 or y <= 1 or x+w >= width-1 or y+h >= height-1:
                    continue
                if not .35 < w/h < 2.8:
                    continue
                # Filled *contour* supports both outlined and solid print. Test
                # bars AND empty notches, rather than Hu moments alone: nearly
                # symmetric H moments were unstable and rectangles can score well.
                rect = cv2.minAreaRect(contour)
                rw,rh = rect[1]
                if min(rw,rh) <= 0 or max(rw,rh)/min(rw,rh) > 2.8:
                    continue
                corners = cv2.boxPoints(rect).astype(np.float32)
                transform = cv2.getPerspectiveTransform(corners,np.array(
                    [[0,0],[99,0],[99,99],[0,99]],dtype=np.float32))
                filled = np.zeros_like(gray)
                cv2.drawContours(filled,[contour],-1,255,-1)
                patch = cv2.warpPerspective(filled,transform,(100,100),flags=cv2.INTER_NEAREST)>0
                scores=[]
                for a in (patch,np.rot90(patch)):
                    left=float(a[:,2:10].mean());right=float(a[:,90:98].mean())
                    cross=float(a[44:56,:].mean())
                    top=float(a[5:28,38:62].mean());bottom=float(a[72:95,38:62].mean())
                    if min(left,right)>.75 and cross>.65 and max(top,bottom)<.15:
                        scores.append(min(left,right,cross)*(1-max(top,bottom)))
                if not scores:
                    continue
                score = 1-max(scores)
                moments = cv2.moments(contour)
                found.append({'center_px': [moments['m10']/moments['m00'], moments['m01']/moments['m00']],
                              'bbox_px': [x,y,w,h], 'shape_distance': score,
                              'threshold': threshold, 'polarity': int(polarity)})
    # Merge repeated threshold detections; retain spatially distinct ambiguities.
    unique = []
    for item in sorted(found, key=lambda r:r['shape_distance']):
        x,y,w,h = item['bbox_px']
        if any(abs(x-a['bbox_px'][0]) < .15*w and abs(y-a['bbox_px'][1]) < .15*h
               and abs(w-a['bbox_px'][2]) < .2*w and abs(h-a['bbox_px'][3]) < .2*h for a in unique):
            continue
        unique.append(item)
    return unique


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('image')
    args = parser.parse_args()
    result = candidates(cv2.imread(args.image))
    print(json.dumps({'candidates': result, 'flight_validated': False,
                      'metric_pose_valid': False, 'landing_boundary_valid': False}, indent=2))
