# Trajectory generation given q0 and qf

# take q0, qf, and tf,

import numpy as np

class TrajectoryGen:
    def __init__(self, q0, q0dot, qf, qfdot, tf, rate):
        self.q0 = q0
        self.qf = qf
        self.tf = tf
        self.q0dot = q0dot
        self.qfdot = qfdot
        self.rate = rate
        self.A = []
        self.q = []

    def return_trajectory(self):
        return self.q



class CubicPoly(TrajectoryGen):

    def calc_coeffs(self):
        # for each joint calculate the coefficients for the cubic polynomial
        for i in range(len(q0)):
            q0 = self.q0[i]
            q0dot = self.q0dot[i]
            qf = self.qf[i]
            qfdot = self.qfdot[i]
            t0 = 0
            tf = self.tf
            a = np.array([[1, t0, t0**2, t0**3], [0, 1, 2*t0, 3*t0**2],
                [1, tf, tf**2, tf**3], [0, 1, 2*tf, 3*tf**2]])
            b = np.array([q0, q0dot, qf, qfdot])

            x = np.linalg.solve(a, b)
            xout = x.tolist()

            # append coefficient vector to list of coefficient vectors
            self.A.append(xout)


    def calc_trajectory(self):
        num_points = tf * rate
        A = self.A

        # for each joint calculate list of positions
        for joint in range(len(self.q0)):
            q = []
            for i in range(num_points):
                t = i / rate
                q.append(A[joint,3] * t**3 + A[joint,2] * t**2 + A[joint,1]*t + A[joint,0])

            # appen list of joint position i to q
            self.q.append(q)


class QuinticPoly(TrajectoryGen):
    def __init__(self, q0, q0dot, q0ddot, qf, qfdot, qfddot, tf, rate):
        self.q0 = q0
        self.qf = qf
        self.tf = tf
        self.q0dot = q0dot
        self.qfdot = qfdot
        self.q0ddot = q0ddot
        self.qfddot = qfddot
        self.rate = rate
        self.A = []
        self.q = []


    def calc_coeffs(self):
        for i in range(len(q0)):
            q0 = self.q0[i]
            q0dot = self.q0dot[i]
            q0ddot = self.q0ddot[i]
            qf = self.qf[i]
            qfdot = self.qfdot[i]
            qfddot = self.qfddot[i]
            t0 = 0
            tf = self.tf

            a = np.array([[1, t0, t0**2, t0**3, t0**4, t0**5],
                [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
                [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
                [1, tf, tf**2, tf**3, tf**4, tf**5],
                [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]])

            b = np.array([q0, q0dot, q0ddot, qf, qfdot, qfddot])

            x = np.linalg.solve(a, b)
            xout = x.tolist()

            # append coefficient vector to list of coefficient vectors
            self.A.append(xout)

    def calc_trajectory(self):
        num_points = tf * rate
        A = self.A

        # for each joint calculate list of positions
        for joint in range(len(self.q0)):
            q = []
            for i in range(num_points):
                t = i / rate
                q.append(A[joint,5] * t**5 + A[joint,4] * t**4 + A [joint,3] * t**3 + A[joint,2] * t**2 + A[joint,1] * t + A[joint,0])

            # appen list of joint position i to q
            self.q.append(q)
