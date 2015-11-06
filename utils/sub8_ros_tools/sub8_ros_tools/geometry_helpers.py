from __future__ import division
import numpy as np
from tf import transformations
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

def quat_to_euler(q):
    ''' Approximate a quaternion as a euler rotation vector
            [1]: http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/
            [2]:http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
    '''

    quat = np.array(([q.x, q.y, q.z, q.w]))
    quat = normalize(quat)
    x,y,z,w = quat[0], quat[1], quat[2], quat[3]

    ''' singularity avoidance
        values close to 90 degrees would render math.atan2(0,0) as 0
        in the x,y,z calculations leading to innacurate returns. 
        this avoids those innacuracies
        0.499 it around 87 degrees, adjust value accordingly 
    '''
    verify = x * y + z * w

    if (verify > .499): # if singularity at north pole
        yaw = 2 * np.arctan2(x,w)
        pitch = np.pi/2
        roll = 0
        final = np.array(([roll, pitch, yaw]))
        return final
   
    if (verify < -.499): # if singularity at south pole
        yaw = -2 * np.arctan2(x,w)
        pitch = - np.pi/2
        roll = 0
        final = np.array(([roll, pitch, yaw]))
        return final
    
    # proofs of formula can be found at the links above 
    roll = np.arctan2(2*x*w - 2*y*z , 1 - 2*x*x - 2*z*z)
    pitch = np.arctan2(2*y*w-2*x*z , 1 - 2*y*y - 2*z*z)
    yaw = np.arcsin(2*(x * y + z * w))

    final = np.array(([roll, pitch, yaw]))
    return final

def euler_to_quat(rotvec):
    ''' convert a euler rotation vector into a ROS quaternion 
            [1]: http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/
    '''

    # proof of formula can be found at the links above 
    roll = rotvec[0]
    pitch = rotvec[1]
    yaw = rotvec[2]
    c1 = np.cos(pitch/2)
    c2 = np.cos(yaw/2)
    c3 = np.cos(roll/2)
    s1 = np.sin(pitch/2)
    s2 = np.sin(yaw/2)
    s3 = np.sin(roll/2)
    w =c1*c2*c3 - s1*s2*s3;
    x =c1*c2*s3 + s1*s2*c3;
    y =s1*c2*c3 + c1*s2*s3;
    z =c1*s2*c3 - s1*c2*s3;
    # return formatted as a ROS quaternion
    return Quaternion(x,y,z,w)




