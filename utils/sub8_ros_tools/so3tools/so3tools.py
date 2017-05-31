"""Functions for manipulating SO3 quantities (rigid body rotations)."""

# --- SUMMARY
# This library is effectively an extension of Christoph Gohlke's
# transformations.py. It contains extra functions specifically
# for handling SO3 quantities.
# Version 1.1

# --- HIGHLIGHTS
# The functions are generally representation independent; you can pass
# in a quaternion or rotation matrix or whatever valid SO3 quantity you
# want and high level operations are carried out correctly regardless.

# SO3 quantity typing is not object-oriented. This means you can keep your
# quantities as simple arrays; you don't have to make all sorts of wacky
# class function calls to work on a state with SO3 quantities. For example,
# you could fit your favorite quaternion q into a state vector as easy as
# [blah,q] and all numpy array operations will still be usable.

# Conversions to and from axles (Euler vectors) are available. An axle
# is the product of the minimum angle of rotation (radians, 0 to pi)
# and axis of rotation (unit 3-vector). The set of all axles is the
# tangent space of the SO3 manifold; it is a lie algebra. When expressed
# as a skew-symmetric matrix, the matrix exponential of the axle is equal
# to its associated rotation matrix. Note: it is a mathematical truth
# that "axle" is a cooler name than "Euler vector" because it cleverly
# combines the words "axis" and "angle."

# "Addition and subtraction" for SO3 quantities is also available.
# For example, minus(a, b) will return an SO3 quantity representing
# the minimal rotation from orientation b to orientation a. These
# are simply abstractions of composition and inverse composition.

# --- USAGE
# Inputted array quantities should be numpy ndarrays, but may also be lists or tuples.
# Outputted array quantities will be numpy ndarrays.
# Inputted angles must be in radians, any value.
# Outputted angles will be in radians, between 0 and pi.
# Quaternions are ordered as [w,xi,yj,zk] where w is the real part.

# --- REFERENCE
# https://github.com/uf-mil/software-common/blob/master/uf_common/src/uf_common/orientation_helpers.py
# https://github.com/krishauser/RobotSim/blob/master/Python/robot/py
# https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation
# http://arxiv.org/pdf/1107.1119.pdf

# --- AUTHOR
# Jason Nezvadovitz

################################################# IMPORTS

# standard
from __future__ import division
# 3rd party
import numpy as np, numpy.linalg as npl
import transformations as trns

################################################# MAIN

def rep(x):
    """Returns a string naming the SO3 representation of the given
    array x by examining its shape and testing for orthonormality.
    If the quantity is not an axle, unit quaternion, rotation matrix,
    or homogeneous transformation matrix, the string 'notSO3' is returned.

    Output formats: 'axle', 'quaternion', 'rotmat', 'tfmat', or 'notSO3'.

    Note that this function is also useful for checking (as one example) if
    a quaternion is remaining unit-norm during operations that accrue error.

    >>> M = trns.random_rotation_matrix()
    >>> print(rep(M))
    tfmat
    >>> print(rep(M[:3,:3]))
    rotmat
    >>> M[-1,-1] = 2  # breaks homogeneity
    >>> print(rep(M))
    notSO3
    >>> print(rep([5,1,0]))  # arbitrary vector
    notSO3

    """
    x = np.array(x)
    shape = x.shape
    if shape == (3,):
        if npl.norm(x)>=0 and npl.norm(x)<=np.pi:
            return 'axle'  # norm of a proper axle is always between 0 and pi (inclusive)
    elif shape == (4,):
        if np.isclose(npl.norm(x), 1):
            return 'quaternion'  # here, quaternion refers specifically to unit quaternions (versors)
    elif shape == (3,3):
        if np.isclose(npl.det(x), 1) and np.allclose(x.dot(x.T), np.eye(3)):
            return 'rotmat'  # a proper orthonormal matrix must have a determinant of +1 and an inverse equal to its transpose
    elif shape == (4,4):
        if np.isclose(npl.det(x[:3,:3]), 1) and np.allclose(x[:3,:3].dot(x[:3,:3].T), np.eye(3)) and np.allclose(x[3,:], [0,0,0,1]):
            return 'tfmat'  # both orthonormality and homogeneity (bottom row of [0,0,0,1]) are required
    return 'notSO3'


def apply(x, P, t=None):
    """Applies the rotation given by the SO3 quantity x to a set of
    points P, and returns the newly transformed set of points. The points
    must be in 3-space and fitted into P as columns:

        [x1 x2 x3 ...]
    P = [y1 y2 y3 ...]
        [z1 z2 z3 ...]

    If t is set to a 3 element vector, then a translation by that specific vector
    will also be applied. Even if you pass in an affine transform (tfmat), only
    the rotation part will be carried out. However, if you intend to pass in
    a tfmat, do  apply(x, P, 'tfmat')  instead of  apply(x, P, x[:3,3])  and
    the extraction of x[:3,3] will be inherent.

    >>> P = [[1,0,1],
    ...      [0,1,1],
    ...      [0,0,1]]
    >>> r = [0,0,np.pi/2]  # rotation of 90 deg about +z axis
    >>> P2 = apply(r,P)
    >>> print(np.round(P2,2))
    [[ 0. -1. -1.]
     [ 1.  0.  1.]
     [ 0.  0.  1.]]
    >>> P3 = apply(r,P,[1,2,3])
    >>> print(np.round(P3,2))
    [[ 1.  0.  0.]
     [ 3.  2.  3.]
     [ 3.  3.  4.]]
    >>> M = affine(matrix_from_axle(r), [1,2,3])
    >>> P4 = apply(M,P)
    >>> P5 = apply(M,P,'tfmat')
    >>> print(np.allclose(P2,P4))
    True
    >>> print(np.allclose(P3,P5))
    True
    >>> A = np.array([[-1 , -0.7 ,  1],
    ...               [ 4 ,   2  ,  4],
    ...               [ 5 ,  0.3 , -8]])  # some constant matrix in body frame
    >>> q = trns.random_quaternion()  # quaternion representing body_to_world conversion
    >>> A_world_ex1 = trns.quaternion_matrix(q)[:3,:3].dot(A).dot(trns.quaternion_matrix(q)[:3,:3].T)
    >>> A_world_ex2 = apply(q, apply(q, A.T).T)  # note that apply(q,A.T).T == A*inv(q)
    >>> print(np.allclose(A_world_ex1, A_world_ex2))
    True

    """
    # Check shape of P:
    P = np.array(P)
    shape = P.shape
    if shape[0] == 3:
        # Check representation of x:
        xrep = rep(x)
        # Get rotation matrix if not rotmat:
        if xrep == 'axle':
            x = matrix_from_axle(x)
        elif xrep == 'quaternion':
            x = trns.quaternion_matrix(x)[:3,:3]
        elif xrep == 'tfmat':
            if t == 'tfmat':
                t = x[:3,3]
            x = x[:3,:3]
        # Apply rotation matrix to each point:
        newP = x.dot(P)
        # Apply translation as desired:
        if t is not None:
            # Allow the user to have used a 1D array for a single point:
            if shape == (3,):
                newP = newP + t
            else:
                newP = newP + np.array([t]).T
        # Finally:
        return newP
    else:
        raise ValueError("P must be 3 by n.")


def plus(x1, x2):
    """Returns the SO3 quantity that represents first rotating
    by x1 and then by x2 (i.e. the composition of x2 on x1, call
    it "x1+x2"). The output will be in the same representation as the
    inputs, but the inputs must be in the same representation as each other.
    Note that in the case of using tfmats, the resulting tfmat's translation
    will, as expected, be the sum of the two constituent translations.

    >>> R1 = trns.random_rotation_matrix()[:3,:3]
    >>> R2 = trns.random_rotation_matrix()[:3,:3]
    >>> R3 = R2.dot(R1)  # rotating vector v by R3 is R2*R1*v
    >>> R = plus(R1,R2)
    >>> print(np.allclose(R,R3))
    True
    >>> M1 = affine(R1, [4,4,4])
    >>> M2 = affine(R2, [5,5,5])
    >>> M3 = affine(R2.dot(R1), [9,9,9])
    >>> M = plus(M1,M2)
    >>> print(np.allclose(M,M3))
    True
    >>> print(np.allclose(M[:3,:3],R))
    True
    >>> q1 = trns.quaternion_from_matrix(R1)
    >>> q2 = trns.quaternion_from_matrix(R2)
    >>> q3 = trns.quaternion_multiply(q2,q1)
    >>> q = plus(q1,q2)  # rotating vector v by q3 is imag(q2*q1*qv)
    >>> print(np.allclose(q,q3))
    True
    >>> print(np.allclose(trns.quaternion_matrix(q)[:3,:3],R))
    True
    >>> R1plusR2plusR3 = reduce(plus,[R1,R2,R3])  # using plus in succession

    """
    xrep = rep(x1)
    if xrep == 'quaternion':
        return trns.unit_vector(trns.quaternion_multiply(x2,x1))
    elif xrep == 'rotmat':
        return x2.dot(x1)
    elif xrep == 'tfmat':
        return affine(x2[:3,:3].dot(x1[:3,:3]), x1[:3,3]+x2[:3,3])
    elif xrep == 'axle':
        # Adding axles is only valid for small perturbations...
        # Perform operation on actual SO3 manifold instead:
        x1 = matrix_from_axle(x1)
        x2 = matrix_from_axle(x2)
        return get_axle(x2.dot(x1))
    else:
        raise ValueError('One or both values to "add" are not SO3.')


def minus(x1, x2):
    """Returns the SO3 quantity representing the minimal rotation from
    orientation x1 to orientation x2 (i.e. the inverse composition of x2 on x1,
    call it "x2-x1"). The output will be in the same representation as the
    inputs, but the inputs must be in the same representation as each other.
    Note that in the case of using tfmats, the resulting tfmat's translation
    will, as expected, be the translation of x2 minus the translation of x1.
    That is, the translation from the x1 origin to the x2 origin.

    >>> A = trns.rotation_matrix(.2,[0,1,0])  # rotation of 0.2 rad about +y axis
    >>> B = trns.rotation_matrix(-.2,[0,1,0])  # rotation of -0.2 rad about +y axis
    >>> AtoB = minus(A,B)  # this is "B - A", the rotation from A to B
    >>> angle, axis = angle_axis(AtoB)  # should be a rotation of (-0.2)-(0.2) = -0.4 rad about +y axis
    >>> print(angle, axis)
    (0.39999999999999997, array([-0., -1., -0.]))
    >>> # The above is 0.4 rad about -y axis, which is equivalent to the expected -0.4 rad about the +y axis.
    >>> # As usual, the angle is always returned between 0 and pi, with the accordingly correct axis.
    >>> # The rotations need not be about the same axis of course:
    >>> NtoA = trns.random_quaternion()  # some rotation from frame N to frame A
    >>> NtoB = trns.random_quaternion()  # some rotation from frame N to frame B
    >>> AtoB = minus(NtoA, NtoB)  # We say "AtoB = NtoB - NtoA"
    >>> NtoAtoB = plus(NtoA, AtoB)  # NtoAtoB == NtoB and we say "NtoAtoB = NtoA + AtoB"
    >>> print(np.allclose(NtoAtoB, NtoB))
    True
    >>> # Evidently, plus and minus are inverse operations.
    >>> # "x1 + (x2 - x1) = x2"  reads as "(N to x1) plus (x1 to x2) = (N to x2)"

    """
    xrep = rep(x1)
    if xrep == 'quaternion':
        return trns.quaternion_multiply(x2, trns.quaternion_inverse(x1))
    elif xrep == 'rotmat':
        return x2.dot(x1.T)
    elif xrep == 'tfmat':
        return affine(x2[:3,:3].dot(x1.T[:3,:3]), x2[:3,3]-x1[:3,3])
    elif xrep == 'axle':
        # Subtracting axles is only valid for small perturbations...
        # Perform operation on actual SO3 manifold instead:
        x1 = matrix_from_axle(x1)
        x2 = matrix_from_axle(x2)
        return get_axle(x2.dot(x1.T))
    else:
        raise ValueError('One or both values to "subtract" are not SO3.')


def error(current,target):
    """Returns a vector representing the difference in orientation between target
    and current. Picture the SO3 manifold as some surface, and now put two points
    on it; one called target and one called current. Draw a curve along the manifold
    from current to target. That curve is so3.minus(current,target). At the curve's
    midpoint, draw the tangent vector to the curve. Make that tangent vector's
    magnitude equal to the arclength of the curve. This vector is the axle. If current
    and target are very close to each other, the axle becomes equivalent to the curve.
    However, even for long distances, the axle is still a good vector representation
    of error from current to target. It tells you instantaneously which way to rotate
    to get from current to target, and tells you approximately by how much. As current
    gets closer to target, these approximations become exact, hence why axles work.

    >>> current = trns.random_quaternion()
    >>> axle_current = get_axle(current)
    >>>
    >>> current_to_target_small = trns.quaternion_about_axis(0.001, [1,-2,3])  # a small rotation of 0.001 rad about some axis
    >>> current_to_target_large = trns.quaternion_about_axis(3, [1,-2,3])  # a large rotation of 3 rad about the same axis
    >>>
    >>> target_near = plus(current, current_to_target_small)  # current + small change = target_near
    >>> axle_target_near = get_axle(target_near)  # axle form of target_near
    >>>
    >>> target_far = plus(current, current_to_target_large)  # current + large change = target_far
    >>> axle_target_far = get_axle(target_far)  # axle form of target_far
    >>>
    >>> err_near = error(current, target_near)  # axle form of current_to_target_small
    >>> err_far = error(current, target_far)  # axle form of current_to_target_large
    >>>
    >>> # In the limit, the error becomes exactly the difference between the two axles:
    >>> print(np.allclose(err_near, axle_target_near - axle_current, atol=1e-02))
    True
    >>> print(np.allclose(err_far, axle_target_far - axle_current, atol=1e-02))
    False

    """
    return get_axle(minus(current,target))


def slerp(x1, x2, fraction):
    """Spherical Linear intERPolation. Returns an SO3 quantity representing an orientation
    between x1 and x2. The fraction of the path from x1 to x2 is the fraction input, and it
    must be between 0 and 1. The output will be in the same representation as the inputs, but
    the inputs must be in the same representation as each other. In the case of using tfmats, 
    the resulting tfmat's translation will point to a location along the line from the x1
    origin to the x2 origin, with that point of course dictated by fraction.

    >>> x1 = affine(trns.random_rotation_matrix()[:3,:3], [1,1,1])
    >>> x2 = affine(trns.random_rotation_matrix()[:3,:3], [2,2,2])
    >>> nogo = slerp(x1, x2, 0)
    >>> print(np.allclose(nogo, x1))
    True
    >>> allgo = slerp(x1, x2, 1)
    >>> print(np.allclose(allgo, x2))
    True
    >>> fourth_between = slerp(x1, x2, 0.25)  # orientation 1/4th the way from orientation x1 to orientation x2
    >>> threefourth_between = slerp(x1, x2, 0.75)  # orientation 3/4th the way from orientation x1 to orientation x2
    >>> fourth_rot = minus(threefourth_between, x2)  # a fourth the rotation from x1 to x2
    >>> print(np.allclose(fourth_between, plus(x1, fourth_rot)))  # x1 + (1-0.75)*(x2-x1)
    True
    >>> print(fourth_between[:3,3])
    [ 1.25  1.25  1.25]

    """
    xrep = rep(x1)
    if xrep == 'quaternion':
        r12 = get_axle(trns.quaternion_multiply(x2, trns.quaternion_inverse(x1)))
        x12 = quaternion_from_axle(fraction*r12)
    elif xrep == 'rotmat':
        r12 = get_axle(x2.dot(x1.T))
        x12 = matrix_from_axle(fraction*r12)
    elif xrep == 'tfmat':
        r12 = get_axle(x2[:3,:3].dot(x1.T[:3,:3]))
        x12 = affine(matrix_from_axle(fraction*r12), fraction*(x2[:3,3]-x1[:3,3]))
    elif xrep == 'axle':
        R1 = matrix_from_axle(x1)
        R2 = matrix_from_axle(x2)
        x12 = fraction*get_axle(R2.dot(R1.T))
    else:
        raise ValueError('One or both values to slerp between are not SO3.')
    return plus(x1, x12)


def get_axle(x):
    """Returns the axle equivalent to the given SO3 quantity x.

    >>> q = trns.random_quaternion()
    >>> r = get_axle(q)
    >>> M = trns.rotation_matrix(npl.norm(r), r/npl.norm(r))
    >>> print(trns.is_same_transform(M, trns.quaternion_matrix(q)))
    True

    """
    # sorry Mario, but the function you are looking for is in another castle
    angle, axis = angle_axis(x)
    return angle*axis


def angle_axis(x):
    """Returns the extracted angle and axis from a valid SO3 quantity x.
    This is equivalent to separating the axle form of x into its
    magnitude and direction. The angle will always be between 0 and pi,
    inclusive, and the axis will always be a unit vector.

    >>> yourAngle, yourAxis = -1337, [-2,0,7]
    >>> yourMat = trns.rotation_matrix(yourAngle, yourAxis)
    >>> myAngle, myAxis = angle_axis(yourMat)
    >>> print(np.isclose(myAngle, np.mod(yourAngle, 2*np.pi)))
    True
    >>> print(np.allclose(myAxis, trns.unit_vector(yourAxis)))
    True
    >>> myAngle2, myAxis2 = angle_axis(trns.quaternion_from_matrix(yourMat))
    >>> print(np.allclose(myAngle, myAngle2), np.allclose(myAxis, myAxis2))
    (True, True)

    """
    xrep = rep(x)
    # In the matrix case, transformations.py already has a nice implementation:
    if xrep in ['rotmat', 'tfmat']:
        if xrep == 'rotmat':
            x = affine(x)
        angle, axis, point = trns.rotation_from_matrix(x)
        # But to be consistent with axles, we only use positive angles:
        if angle<0:
            angle, axis = -angle, -axis
        # And we map the identity rotation to an axis of [0,0,0]:
        elif np.isclose(angle, 0):
            axis = np.array([0,0,0])
    # In the quaternion case, we carry out the following routine:
    elif xrep == 'quaternion':
        # Renormalize for accuracy:
        x = trns.unit_vector(x)
        # If "amount of rotation" is negative, flip quaternion:
        if x[0] < 0:
            x = -x
        # Extract axis:
        imMax = max(map(abs, x[1:]))
        if not np.isclose(imMax, 0):
            axis = trns.unit_vector(x[1:] / (imMax*npl.norm(x[1:])))
        else:
            axis = np.array([0,0,0])
        # Extract angle:
        angle = 2*np.arccos(x[0])
    # In the axle case, the routine is trivial:
    elif xrep == 'axle':
        angle = npl.norm(x)
        if not np.isclose(angle, 0):
            axis = x/angle
        else:
            axis = np.array([0,0,0])
    # If not SO3:
    else:
        raise ValueError('Quantity to extract angle and axis from must be on SO3')
    # Finally:
    return (angle, axis)


def quaternion_from_axle(r):
    """Returns the quaternion equivalent to the given axle r.

    >>> myAxle = np.pi * np.array([0,1,0])
    >>> q = quaternion_from_axle(myAxle)
    >>> M1 = trns.quaternion_matrix(q)
    >>> M2 = trns.rotation_matrix(np.pi, [0,1,0])
    >>> print(trns.is_same_transform(M1, M2))
    True

    """
    angle = np.mod(npl.norm(r), 2*np.pi)
    if not np.isclose(angle, 0):
        return trns.quaternion_about_axis(angle, r/angle)
    else:
        return np.array([1,0,0,0])  # unit real number is identity quaternion


def matrix_from_axle(r):
    """Returns the rotation matrix equivalent to the given axle r.

    >>> angle, axis = 45*(np.pi/180), trns.unit_vector([0,0,1])
    >>> myAxle = angle*axis
    >>> myMat = matrix_from_axle(myAxle)
    >>> print(np.round(myMat, decimals=3))
    [[ 0.707 -0.707  0.   ]
     [ 0.707  0.707  0.   ]
     [ 0.     0.     1.   ]]

    """
    angle = np.mod(npl.norm(r), 2*np.pi)
    if not np.isclose(angle, 0):
        axlemat = crossmat(r/angle)
        return np.eye(3) + np.sin(angle)*axlemat + (1-np.cos(angle))*axlemat.dot(axlemat)
    else:
        return np.eye(3)


def affine(x, t=None):
    """Returns the affine space form of a given array x. If x is
    a vector, it appends a new element with value 1 (creating a
    homogeneous coordinate). If x is a square matrix, it appends
    the row [0,0,0...] and the column [t,1].T representing a
    translation of vector t. If t is None (default) then the
    translation is set to the zero vector.
    See: https://en.wikipedia.org/wiki/Affine_transformation#Augmented_matrix

    >>> v = np.array([-4,5,2])
    >>> print(affine(v))
    [-4  5  2  1]
    >>> R = np.array([[np.cos(1.5),-np.sin(1.5)],[np.sin(1.5),np.cos(1.5)]])
    >>> print(affine(R, [3,-4]))
    [[ 0.0707372  -0.99749499  3.        ]
     [ 0.99749499  0.0707372  -4.        ]
     [ 0.          0.          1.        ]]
    >>> M = trns.random_rotation_matrix()  # fyi this function creates a tfmat
    >>> print(np.allclose(M, affine(M[:3,:3])))
    True

    """
    x = np.array(x)
    shape = x.shape
    dim = len(shape)
    if dim == 1:
        return np.append(x,1)
    elif dim == 2 and shape[0] == shape[1]:
        cast = np.zeros((shape[0]+1, shape[1]+1))
        cast[-1,-1] = 1
        cast[:shape[0],:shape[1]] = x
        if t is not None:
            cast[:-1,-1] = t
        return cast
    else:
        raise ValueError('Array to make affine must be a row or a square.')


def crossmat(v):
    """Returns the skew-symmetric matrix form of a three element vector.
    See: https://en.wikipedia.org/wiki/Cross_product#Conversion_to_matrix_multiplication

    >>> a = [1,2,-3]
    >>> b = [4,-5,6]
    >>> print(np.cross(a,b) == crossmat(a).dot(b))
    [ True  True  True]

    """
    return np.array( [[  0   , -v[2]  ,  v[1] ],
                      [ v[2] ,   0    , -v[0] ],
                      [-v[1] ,  v[0]  ,   0   ]] )


def crossvec(W, check=False):
    """Returns the vector form of the given skew-symmetric matrix W.
    This is exactly the inverse operation of crossmat(v). Be aware that
    no checking is done to make sure the given matrix is skew-symmetric
    unless check is set to True.

    >>> v = trns.random_vector(3)
    >>> u = crossvec(crossmat(v))
    >>> print(np.allclose(u,v))
    True

    """
    v = np.array([W[2,1], W[0,2], W[1,0]])
    if check == True and not np.allclose(W, crossmat(v)):
        raise ValueError('Cross-product matrix must be skew-symmetric.')
    return v


def random_axle():
    """Returns a randomly generated valid axle.

    >>> r = random_axle()
    >>> print(rep(r))
    axle

    """
    angle = np.pi*np.random.rand(1)
    axis = trns.unit_vector(np.random.rand(3))
    for i in range(3):
        coinflip = np.random.rand(1)
        if coinflip>0.5:
            axis[i] = -axis[i]
    return angle*axis


# Module unit test:
if __name__ == "__main__":
    import doctest
    doctest.testmod()
