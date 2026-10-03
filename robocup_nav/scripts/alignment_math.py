"""Fixed local FRD preview from gravity-aligned ROS odom / body FLU.

No PX4 north alignment or range substitution. Origin is fixed for a session.
"""
import numpy as np
from scipy.spatial.transform import Rotation


class LocalFRD:
    def __init__(self, position, quaternion_xyzw):
        p, r = self.validate(position, quaternion_xyzw)
        heading = np.arctan2(r[1, 0], r[0, 0])
        self.origin = p.copy()
        self.flip = np.diag([1., -1., -1.])
        self.world = self.flip @ Rotation.from_euler('z', -heading).as_matrix()

    @staticmethod
    def validate(position, quaternion):
        p, q = np.asarray(position, dtype=float), np.asarray(quaternion, dtype=float)
        if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all():
            raise ValueError('invalid pose')
        if abs(np.linalg.norm(q)-1.) > .01:
            raise ValueError('quaternion not normalized')
        return p, Rotation.from_quat(q).as_matrix()

    def convert(self, position, quaternion_xyzw):
        p, r = self.validate(position, quaternion_xyzw)
        q = Rotation.from_matrix(self.world @ r @ self.flip).as_quat()
        return self.world @ (p-self.origin), q[[3, 0, 1, 2]]


def rpy_degrees(q_wxyz):
    q = np.asarray(q_wxyz, dtype=float)
    return Rotation.from_quat(q[[1, 2, 3, 0]]).as_euler('xyz', degrees=True)
