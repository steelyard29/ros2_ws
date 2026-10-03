import sys,unittest
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from image_direction_check import compare


class DirectionTests(unittest.TestCase):
    def setUp(self):
        rng=np.random.default_rng(19)
        self.a=np.zeros((480,640),np.uint8)
        for x,y,r in rng.integers([30,30,3],[610,450,9],size=(180,3)):
            cv2.circle(self.a,(int(x),int(y)),int(r),int(rng.integers(80,255)),-1)

    def test_cardinal_translations(self):
        for dx,dy,side in ((20,0,'right'),(-20,0,'left'),(0,20,'down'),(0,-20,'up')):
            b=cv2.warpAffine(self.a,np.float32([[1,0,dx],[0,1,dy]]),(640,480))
            out=compare(self.a,b)
            self.assertTrue(out['translation_accepted'],out)
            self.assertEqual(out['image_direction'],side)
            self.assertFalse(out['optical_axes_verified'])
            self.assertFalse(out['flight_authorized'])

    def test_stationary_blank_and_rotation_rejected(self):
        rotated=cv2.warpAffine(self.a,cv2.getRotationMatrix2D((320,240),10,1),(640,480))
        for a,b in ((self.a,self.a),(np.zeros_like(self.a),np.zeros_like(self.a)),
                    (self.a,rotated),(self.a,None)):
            self.assertFalse(compare(a,b)['translation_accepted'])

    def test_diagonal_is_not_a_cardinal_axis(self):
        b=cv2.warpAffine(self.a,np.float32([[1,0,20],[0,1,20]]),(640,480))
        self.assertFalse(compare(self.a,b)['translation_accepted'])


if __name__=='__main__':unittest.main()
