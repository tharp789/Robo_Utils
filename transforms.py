from scipy.spatial.transform import Rotation as R
import numpy as np

def quat2rot(q):
    r = R.from_quat(q)
    return r.as_matrix()

def rot2quat(R):
    r = R.from_matrix(R)
    return r.as_quat()

def quat2euler(q):
    r = R.from_quat(q)
    return r.as_euler('xyz')

def euler2quat(e):
    r = R.from_euler('xyz', e)
    return r.as_quat()

def rot2euler(R):
    r = R.from_matrix(R)
    return r.as_euler('xyz')

def euler2rot(e):
    r = R.from_euler('xyz', e)
    return r.as_matrix()

def make_transform_rm(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def make_transform_q(q, t):
    R = quat2rot(q)
    return make_transform_rm(R, t)

