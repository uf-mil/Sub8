from __future__ import division
import numpy as np
from tf import transformations
import math
from geometry_msgs.msg import Quaternion


def make_rotation(vector_a, vector_b):
    '''Determine a 3D rotation that rotates A onto B
        In other words, we want a matrix R that aligns a with b

        >> R = make_rotation(a, b)
        >> p = R.dot(a)
        >> np.cross(p, a)
        >>>  array([0.0, 0.0, 0.0])

        [1] Calculate Rotation Matrix to align Vector A to Vector B in 3d?
            http://math.stackexchange.com/questions/180418
        [2] N. Ho, Finding Optimal Rotation...Between Corresponding 3D Points
            http://nghiaho.com/?page_id=671
    '''
    unit_a = normalize(vector_a)
    unit_b = normalize(vector_b)

    v = np.cross(unit_a, unit_b)
    s = np.linalg.norm(v)

    c = np.dot(unit_a, unit_b)

    skew_cross = skew_symmetric_cross(v)
    skew_squared = np.linalg.matrix_power(skew_cross, 2)

    if np.isclose(c, 1.0, atol=1e-4):
        R = np.eye(3)
        return R
    elif np.isclose(c, -1.0, atol=1e-4):
        R = np.eye(3)
        R[2, 2] *= -1
        return R

    normalization = (1 - c) / (s ** 2)

    R = np.eye(3) + skew_cross + (skew_squared * normalization)

    # Address the reflection case
    if np.linalg.det(R) < 0:
        R[:, 3] *= -1

    return R


def skew_symmetric_cross(a):
    '''Return the skew symmetric matrix representation of a vector
        [1] https://en.wikipedia.org/wiki/Cross_product#Skew-symmetric_matrix
    '''
    assert len(a) == 3, "a must be in R3"
    skew_symm = np.array([
        [0.,     -a[2], +a[1]],
        [+a[2],     0., -a[0]],
        [-a[1],  +a[0],    0.],
    ], dtype=np.float32)
    return skew_symm


def normalize(vector):
    return vector / np.linalg.norm(vector)


def compose_transformation(R, t):
    '''Compose a transformation from a rotation matrix and a translation matrix'''
    transformation = np.zeros((4, 4))
    transformation[:3, :3] = R
    transformation[3, :3] = t
    transformation[3, 3] = 1.0
    return transformation


def quat_to_rotvec(q):
    ''' Turn a quaternion into a axis rotation vector [x,y,z] '''
    if q[3] < 0:
        q = -q
    q = transformations.unit_vector(q)
    angle = math.acos(q[3])*2
    axis = normalize(q[0:3])
    return axis * angle

def quat_to_np(q):
    ''' Turn a quaternion into a numpy array '''
    array = np.zeros((4))
    array[0] = q.x
    array[1] = q.y
    array[2] = q.z
    array[3] = q.w
    return array

def np_to_quat(array):
    ''' Turn a numpy array into a ROS quaternion '''
    quat = Quaternion()
    quat.x = array[0]
    quat.y = array[1]
    quat.z = array[2]
    quat.w = array[3]
    return quat

def rotvec_to_quat(rotvec):
    ''' Turn a axis rotation vector into a ROS quaternion '''
    return transformations.quaternion_about_axis(np.linalg.norm(rotvec), rotvec)

