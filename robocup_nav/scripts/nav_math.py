"""Pure geometry and preview conversion; no ROS or device side effects."""
import math
import numpy as np


def rotation(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.isfinite(q).all() or norm < 1e-8:
        raise ValueError('invalid quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def yaw(q):
    r = rotation(q)
    return math.atan2(r[1, 0], r[0, 0])


def preview_ned(vx, vy, yaw_rate, heading, height):
    if not all(math.isfinite(v) for v in (vx, vy, yaw_rate, heading, height)) or height <= 0:
        raise ValueError('invalid command')
    c, s = math.cos(heading), math.sin(heading)
    east, north = c*vx-s*vy, s*vx+c*vy
    return [north, east, 0.0], -yaw_rate, -height


def depth_points(image, fx, fy, cx, cy, stride=8):
    if fx <= 0 or fy <= 0:
        raise ValueError('invalid camera focal length')
    v, u = np.mgrid[0:image.shape[0]:stride, 0:image.shape[1]:stride]
    z = image[::stride, ::stride]
    valid = np.isfinite(z) & (z >= 0.25) & (z <= 4.0)
    z, u, v = z[valid], u[valid], v[valid]
    return np.column_stack(((u-cx)*z/fx, (v-cy)*z/fy, z)).astype(np.float32)
