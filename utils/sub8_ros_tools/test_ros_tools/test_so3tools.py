#!/usr/bin/env python
from __future__ import division
import unittest
import numpy as np, numpy.linalg as npl
import transformations as trns
import so3tools as so3


class TestSo3Tools(unittest.TestCase):
    def setUp(self):
        pass

    def test_rep(self):
        M = trns.random_rotation_matrix()
        self.assertEqual(so3.rep(M), 'tfmat')
        self.assertEqual(so3.rep(M[:3,:3]), 'rotmat')
        self.assertEqual(so3.rep(so3.get_axle(M)), 'axle')
        self.assertEqual(so3.rep([5,1,0]), 'notSO3')
        M[-1,-1] = 2
        self.assertEqual(so3.rep(M), 'notSO3')

    def test_apply(self):
        P = [[1,0,1],
             [0,1,1],
             [0,0,1]]
        r = [0,0,np.pi/2]
        P2 = np.round(so3.apply(r,P),2)
        np.testing.assert_array_equal(P2, [[0,-1,-1],[1,0,1],[0,0,1]])
        P3 = np.round(so3.apply(r,P,[1,2,3]),2)
        np.testing.assert_array_equal(P3, [[1,0,0],[3,2,3],[3,3,4]])
        M = so3.affine(so3.matrix_from_axle(r), [1,2,3])
        P4 = so3.apply(M,P)
        P5 = so3.apply(M,P,'tfmat')
        self.assertTrue(np.allclose(P2,P4))
        self.assertTrue(np.allclose(P3,P5))
        A = np.array([[-1,-0.7,1],[4,2,4],[5,0.3,-8]])
        q = trns.random_quaternion()
        A_world_ex1 = trns.quaternion_matrix(q)[:3,:3].dot(A).dot(trns.quaternion_matrix(q)[:3,:3].T)
        A_world_ex2 = so3.apply(q, so3.apply(q, A.T).T)  # note that apply(q,A.T).T == A*inv(q)
        self.assertTrue(np.allclose(A_world_ex1, A_world_ex2))

    def test_plus(self):
        R1 = trns.random_rotation_matrix()[:3,:3]
        R2 = trns.random_rotation_matrix()[:3,:3]
        R3 = R2.dot(R1)
        R = so3.plus(R1,R2)
        self.assertTrue(np.allclose(R,R3))
        M1 = so3.affine(R1, [4,4,4])
        M2 = so3.affine(R2, [5,5,5])
        M3 = so3.affine(R2.dot(R1), [9,9,9])
        M = so3.plus(M1,M2)
        self.assertTrue(np.allclose(M,M3))
        self.assertTrue(np.allclose(M[:3,:3],R))
        q1 = trns.quaternion_from_matrix(R1)
        q2 = trns.quaternion_from_matrix(R2)
        q3 = trns.quaternion_multiply(q2,q1)
        q = so3.plus(q1,q2)
        self.assertTrue(np.allclose(q,q3))
        self.assertTrue(np.allclose(trns.quaternion_matrix(q)[:3,:3],R))

    def test_minus(self):
        A = trns.rotation_matrix(.2,[0,1,0])
        B = trns.rotation_matrix(-.2,[0,1,0])
        AtoB = so3.minus(A,B)
        angle, axis = so3.angle_axis(AtoB)
        np.testing.assert_array_equal(angle, 0.39999999999999997)
        np.testing.assert_array_equal(axis, [-0,-1,-0])
        NtoA = trns.random_quaternion()
        NtoB = trns.random_quaternion()
        AtoB = so3.minus(NtoA, NtoB)
        NtoAtoB = so3.plus(NtoA, AtoB)
        self.assertTrue(np.allclose(NtoAtoB, NtoB))

    def test_error(self):
        current = trns.random_quaternion()
        axle_current = so3.get_axle(current)
        current_to_target_small = trns.quaternion_about_axis(0.001, [1,-2,3])  # a small rotation of 0.001 rad about some axis
        current_to_target_large = trns.quaternion_about_axis(3, [1,-2,3])  # a large rotation of 3 rad about the same axis
        target_near = so3.plus(current, current_to_target_small)  # current + small change = target_near
        axle_target_near = so3.get_axle(target_near)  # axle form of target_near
        target_far = so3.plus(current, current_to_target_large)  # current + large change = target_far
        axle_target_far = so3.get_axle(target_far)  # axle form of target_far
        err_near = so3.error(current, target_near)  # axle form of current_to_target_small
        err_far = so3.error(current, target_far)  # axle form of current_to_target_large
        self.assertTrue(np.allclose(err_near, axle_target_near - axle_current, atol=1e-02))
        self.assertFalse(np.allclose(err_far, axle_target_far - axle_current, atol=1e-02))

    def test_slerp(self):
        x1 = so3.affine(trns.random_rotation_matrix()[:3,:3], [1,1,1])
        x2 = so3.affine(trns.random_rotation_matrix()[:3,:3], [2,2,2])
        nogo = so3.slerp(x1, x2, 0)
        self.assertTrue(np.allclose(nogo, x1))
        allgo = so3.slerp(x1, x2, 1)
        self.assertTrue(np.allclose(allgo, x2))
        fourth_between = so3.slerp(x1, x2, 0.25)  # orientation 1/4th the way from orientation x1 to orientation x2
        threefourth_between = so3.slerp(x1, x2, 0.75)  # orientation 3/4th the way from orientation x1 to orientation x2
        fourth_rot = so3.minus(threefourth_between, x2)  # a fourth the rotation from x1 to x2
        self.assertTrue(np.allclose(fourth_between, so3.plus(x1, fourth_rot)))  # x1 + (1-0.75)*(x2-x1)
        np.testing.assert_array_equal(fourth_between[:3,3], [1.25,1.25,1.25])

    def test_get_axle(self):
        q = trns.random_quaternion()
        r = so3.get_axle(q)
        M = trns.rotation_matrix(npl.norm(r), r/npl.norm(r))
        self.assertTrue(trns.is_same_transform(M, trns.quaternion_matrix(q)))

    def test_angle_axis(self):
        yourAngle, yourAxis = -1337, [-2,0,7]
        yourMat = trns.rotation_matrix(yourAngle, yourAxis)
        myAngle, myAxis = so3.angle_axis(yourMat)
        self.assertTrue(np.isclose(myAngle, np.mod(yourAngle, 2*np.pi)))
        self.assertTrue(np.allclose(myAxis, trns.unit_vector(yourAxis)))
        myAngle2, myAxis2 = so3.angle_axis(trns.quaternion_from_matrix(yourMat))
        self.assertTrue(np.allclose(myAngle, myAngle2))
        self.assertTrue(np.allclose(myAxis, myAxis2))

    def test_quaternion_from_axle(self):
        myAxle = np.pi * np.array([0,1,0])
        q = so3.quaternion_from_axle(myAxle)
        M1 = trns.quaternion_matrix(q)
        M2 = trns.rotation_matrix(np.pi, [0,1,0])
        self.assertTrue(trns.is_same_transform(M1, M2))

    def test_matrix_from_axle(self):
        angle, axis = 45*(np.pi/180), trns.unit_vector([0,0,1])
        myAxle = angle*axis
        myMat = so3.matrix_from_axle(myAxle)
        np.testing.assert_array_equal(np.round(myMat, decimals=3), [[0.707,-0.707,0],[0.707,0.707,0],[0,0,1]])

    def test_affine(self):
        v = np.array([-4,5,2])
        np.testing.assert_array_equal(so3.affine(v), [-4,5,2,1])
        R = np.array([[np.cos(1.5),-np.sin(1.5)],[np.sin(1.5),np.cos(1.5)]])
        np.testing.assert_array_almost_equal(so3.affine(R, [3,-4]), [[0.0707372,-0.99749499,3.],[0.99749499,0.0707372,-4.],[0.,0.,1.]])
        M = trns.random_rotation_matrix()
        np.testing.assert_array_almost_equal(M, so3.affine(M[:3,:3]))

    def test_crossmat(self):
        a = [1,2,-3]
        b = [4,-5,6]
        np.testing.assert_array_equal(np.cross(a,b) == so3.crossmat(a).dot(b), [True,True,True])

    def test_crossvec(self):
        v = trns.random_vector(3)
        u = so3.crossvec(so3.crossmat(v))
        self.assertTrue(np.allclose(u,v))

    def test_random_axle(self):
        r = so3.random_axle()
        self.assertTrue(so3.rep(r) == 'axle')


if __name__ == '__main__':
    unittest.main()
