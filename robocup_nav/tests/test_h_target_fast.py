import sys
import unittest
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from h_target_geometry import candidates
from h_target_fast import fast_candidates
from test_h_target_geometry import target


class FastTests(unittest.TestCase):
    def test_original_pixel_centers_and_polarities(self):
        for a in (target(),255-target(),target((190,90,20),(255,255,255))):
            image=cv2.resize(a,(1280,960))
            full=candidates(image);small=fast_candidates(image)
            self.assertEqual(len(full),1);self.assertEqual(len(small),1)
            self.assertLess(np.linalg.norm(np.array(full[0]['center_px'])-small[0]['center_px']),2.)

    def test_noninteger_resize_rotations(self):
        for angle in (15,45,90):
            a=cv2.warpAffine(target(),cv2.getRotationMatrix2D((320,240),angle,1),
                            (640,480),borderValue=(255,255,255))
            image=cv2.resize(a,(1182,886))
            full=candidates(image);small=fast_candidates(image)
            self.assertTrue(full);self.assertTrue(small)
            self.assertLess(np.linalg.norm(np.array(full[0]['center_px'])-small[0]['center_px']),3.)

    def test_negative_and_invalid(self):
        a=np.full((720,1280,3),255,np.uint8)
        cv2.rectangle(a,(400,250),(800,600),(0,0,0),-1)
        self.assertFalse(fast_candidates(a))
        for im,dim in ((None,640),(a,10)):
            with self.assertRaises(ValueError):fast_candidates(im,dim)

    def test_jpeg_transport_center(self):
        for quality in (80,95):
            image=cv2.resize(target(),(1280,960))
            ok,encoded=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,quality])
            self.assertTrue(ok)
            decoded=cv2.imdecode(encoded,cv2.IMREAD_COLOR)
            a,b=fast_candidates(image),fast_candidates(decoded)
            self.assertEqual(len(b),1)
            self.assertLess(np.linalg.norm(np.array(a[0]['center_px'])-b[0]['center_px']),2.)


if __name__=='__main__':unittest.main()
