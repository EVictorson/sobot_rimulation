# Depends: sympy, mpmath

import numpy as np
import sympy as sp

class DH:
    def __init__(self):
        self.DH_trans = []
        self.rot_matrices = []
        self.T

    # call init after adding all links
    def init(self):
        self.num_links = len(self.DH_trans)

    # add DH matrix composed of
    # a: link length
    # d: link offset
    # alpha: link twist
    # theta: joint angle
    def add(self, a, d, alpha, theta):
        sp.var('a d alpha theta A1 A2 R')
        # Translate frame i-1 by d, along axis z(i-1), and rotate it by theta about z(i-1)
        # This sequence aligns the current frame with frame i'.
        A1 = sp.Matrix([np.cos(theta), -1*np.sin(theta), 0, 0],
                    [np.sin(theta), np.cos(theta), 0, 0],
                    [0, 0, 1, d],
                    [0, 0, 0, 1])

        R = sp.Matrix([A1[0,0], A1[0,1], A1[0,2],
                        [A1[1,0], A1[1,1], A1[1,2],
                        [A1[2,0], A1[2,1], A1[2,2]])

        # Translate the frame aligned with frame i' by alpha along axis xi',
        # and rotate it by alpha about axis xi'.
        # This sequence aligns the current frame with frame i.
        A2 = sp.Matrix([1, 0, 0, alpha],
                    [0, np.cos(alpha), -1*np.sin(alpha), 0],
                    [0, np.sin(alpha), np.cos(alpha), 0],
                    [0, 0, 0, 1])

        # Multiply the matrices to get the homogeneous transformation from
        # Frame i-1 to Frame i
        A1_2 = A1.multiply(A2)

        # append the transformation matrix
        self.DH_trans.append(A1_2)
        self.rot_matrices.append(R)

    def get_int_dh_mat(self, start_link, end_link):
        num = end_link - start_link
        i = start_link

        A0_N = self.DH_trans[start_link]
        while(i < end_link):
            A0_N = A0_N.multiply(self.DH_trans[i+1])
            i = i+1

        return A0_N

    def get_int_rot_mat(self, start_link, end_link):
        num = end_link - start_link
        i = start_link

        R0_N = self.rot_matrices[start_link]
        while(i < end_link):
            R0_N = A0_N.multiply(self.rot_matrices[i+1])
            i = i+1

        return R0_N


    def direct_kinematics(self):
        An = self.DH_trans[0]
        for i in range(len(self.DH_trans)-1):
            An = An.multiply(self.DH_trans[i+1])

        self.T = An


# For now this class assumes all revolute joints
class Jacobian(DH):
    self.Jp = []
    self.Jo = []

    # get z(i-1) vector by performing R(0->1)R(1->2)... [0, 0, 1]^T
    def get_z_vect(self, end_link):
        R = self.get_int_rot_mat(0, end_link)
        z0 = sp.Matrix([[0],[0],[1]])
        z_i = R.multiply(z0)

        return z_i

    # get position of origin of link i by performing A(0->1)A(1->2)...[0,0,0,1]^T
    # this method will also be used to get pos(link-1)
    def get_pos_li(self, end_link):
        A = self.get_int_dh_mat(0, end_link)
        p0 = sp.Matrix([[0], [0], [0], [1]])

        p_li = A.multiply(p0)

        return p_li

    # linear jacobian for link i
    # Jpi = zi-1 x (pli - pli-1)
    def get_JP_i(self, ref_link, i):
        li_sub = ref_link-i
        zi_sub_1 = self.get_z_vect(li_sub_1)
        pli = self.get_pos_li(ref_link)
        pli_sub_1 = self.get_pos_li(li_sub_1)
        Jp_i = zi_sub_1.multiply((pli - pli_sub_1))

        return Jp_i

    # populate entire linear jacobian matrix
    def get_Jp(self):
        JP = []
        for i in range(self.num_links):
            for j in range(self.num_links):
                JPi = self.get_JP_i(i,j)
                JP.append(JPi)
            self.JP.append(JP)

    # angular link jacobian
    def get_JO_i(self, ref_link, i):
        if(ref_link < i):
            return [0 0 0]
        else:
            zi_sub_1 = self.get_z_vect(self, ref_link, i)

            return zi_sub_1

    # populate entire angular jacobian matrix
    def get_Jo(self):
        JO = []
        for i in range(self.num_links):
            for j in range(self.num_links):
                Joi = self.get_JO_i(i,j)
                JO.append(Joi)
            self.JO.append(JO)

    # create partitioned linear and angular jacobian Matrix
    def construct_link_jacobian(self):
        # do all the things for the links

    def construct_motor_jacobian(self):
        # do all the things for the motors

    # Construct the nxn symmetric Inertia Matrix
    # where n is number of generalized coordinates
    # need Jp, Jo, motor masses, link masses, Rotation matrices,
    # link inertia matrices, motor inertia matrices
    def construct_inertia_matrix(self):


    def get_general_christoffel_symbol(self,i,j,k):
