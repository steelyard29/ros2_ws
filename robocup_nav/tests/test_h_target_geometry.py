import sys
import unittest
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from h_target_geometry import candidates


def target(background=(255,255,255), ink=(0,0,0)):
    im=np.full((480,640,3),background,np.uint8)
    for a,b in [((220,120),(260,360)),((380,120),(420,360)),((220,220),(420,260))]:
        cv2.rectangle(im,a,b,ink,-1)
    return im


class HTests(unittest.TestCase):
    def test_polarities_and_blue_background(self):
        for im in [target(),255-target(),target((190,90,20),(255,255,255))]:
            self.assertTrue(candidates(im))

    def test_rotations(self):
        for angle in [15,45,90]:
            im=cv2.warpAffine(target(),cv2.getRotationMatrix2D((320,240),angle,1),(640,480),borderValue=(255,255,255))
            self.assertTrue(candidates(im),angle)

    def test_negatives(self):
        for kind in ['blank','rectangle','cross','ring']:
            im=np.full((480,640,3),255,np.uint8)
            if kind=='rectangle':cv2.rectangle(im,(180,100),(440,380),(0,0,0),-1)
            if kind=='cross':
                cv2.rectangle(im,(300,100),(340,380),(0,0,0),-1)
                cv2.rectangle(im,(180,220),(460,260),(0,0,0),-1)
            if kind=='ring':cv2.circle(im,(320,240),150,(0,0,0),15)
            self.assertFalse(candidates(im),kind)

    def test_real_outline_fixture(self):
        p=Path(__file__).resolve().parents[1]/'evidence/downward_20260929_113856/image.png'
        if not p.exists():self.skipTest('local recorded fixture absent')
        out=candidates(cv2.imread(str(p)))
        self.assertTrue(out)
        self.assertLess(abs(out[0]['center_px'][0]-315),25)
        self.assertLess(abs(out[0]['center_px'][1]-240),30)


if __name__=='__main__':unittest.main()
