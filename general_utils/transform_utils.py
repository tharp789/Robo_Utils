from scipy.spatial.transform import Rotation as Rot
import numpy as np

def quat2rot(q):
    r = Rot.from_quat(q)
    return r.as_matrix()

def rot2quat(R):
    r = Rot.from_matrix(R)
    return r.as_quat()

def quat2euler(q):
    r = Rot.from_quat(q)
    return r.as_euler('xyz')

def euler2quat(e):
    r = Rot.from_euler('xyz', e)
    return r.as_quat()

def rot2euler(R):
    r = Rot.from_matrix(R)
    return r.as_euler('xyz')

def euler2rot(e):
    r = Rot.from_euler('xyz', e)
    return r.as_matrix()

def make_transform_rotmatrix(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def make_transform_q(quat, pos):
    '''
    Make a 4x4 transformation matrix from a quaternion and a position
    
    Args:
    quat: 4 element list, quaternion
    pos: 3 element list, position
    
    Returns:
    4x4 numpy array, transformation matrix
    '''
    
    R = quat2rot(quat)
    return make_transform_rotmatrix(R, pos)


