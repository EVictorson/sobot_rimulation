#!/usr/bin/env python
""" Simple class to generate robot manipulator joint space trajectories.

This class only performs direct kinematics.  If a task space goal is desired
you must first convert your task space goal to a joint space goal via the associated
inverse kinematics module.
Methods currently employed are cubic polynomial and quintic polynomial.
The CubicPoly method takes in angular position and angular velocity.
The QuinticPoly method takes in angular position, velocity and acceleration.

  Typical usage example:

    q0 = [0.52, 0.52]
    q0dot = [0,0]
    qf = [2.09, -2.62]
    qfdot = [1, 1]
    traj_rate = 20
    tf = 3

    TGen = CubicPoly(q0,q0dot,qf,qfdot,tf,traj_rate)
    TGen.run()
"""

import numpy as np
import matplotlib.pyplot as plt

class TrajectoryGen:
    """ Trajectory generation parent class

    Attributes:
        q0: state position initial conditions
        q0dot: state velocity initial conditions
        qf: state position final conditions
        qfdot: state velocity final conditions
        tf: final time
        rate: rate at which generator runs
        A: list of polynomial coefficients
        q: list of states at each sampling period
    """

    def __init__(self, q0, q0dot, qf, qfdot, tf, rate):
        """ Init with above attributes. """
        self.q0 = q0
        self.qf = qf
        self.tf = tf
        self.q0dot = q0dot
        self.qfdot = qfdot
        self.rate = rate
        self.A = []
        self.q = []
        self.qdot = []
        self.qddot = []

    def return_trajectory(self):
        """ Return generated trajectory. """
        return self.q

    def run(self):
        """ Run everything. """
        self.calc_coeffs()
        self.calc_trajectory()

    def get_traj_length(self):
        return len(self.q)


class CubicPoly(TrajectoryGen):
    """ Trajectory generation child class to perform cubic polynomial generation."""
    def calc_coeffs(self):
        """ Solve for cubic polynomial coefficients for each joint."""
        for i in range(len(self.q0)):
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
        """ Generate trajectory given polynomial coefficients and sampling rate."""
        num_points = self.tf * self.rate
        A = self.A

        # for each joint calculate list of positions
        for joint in range(len(self.q0)):
            q = []
            qdot = []
            qddot = []
            for i in range(num_points):
                t = (float(i) / self.rate)
                q.append(A[joint][3] * t**3 + A[joint][2] * t**2 + A[joint][1]*t + A[joint][0])
                qdot.append(3*A[joint][3] * t**2 + 2*A[joint][2] * t + A[joint][1])
                qddot.append(6*A[joint][3] * t + 2*A[joint][2])

            # append list of joint position i to q
            self.q.append(q)
            self.qdot.append(qdot)
            self.qddot.append(qddot)




class QuinticPoly(TrajectoryGen):
    """ Trajectory generation child class to perform cubic polynomial generation.

    Attributes:
        q0: state position initial conditions
        q0dot: state velocity initial conditions
        q0ddot: state acceleration initial conditions
        qf: state position final conditions
        qfdot: state velocity final conditions
        qfddot: state acceleration final conditions
        tf: final time
        rate: rate at which generator runs
        A: list of polynomial coefficients
        q: list of states at each sampling period
        """
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
        self.qdot = []
        self.qddot = []


    def calc_coeffs(self):
        """ Solve for quintic polynomial coefficients for each joint."""
        for i in range(len(self.q0)):
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
        """ Generate trajectory given polynomial coefficients and sampling rate."""
        num_points = self.tf * self.rate
        A = self.A

        # for each joint calculate list of positions
        for joint in range(len(self.q0)):
            q = []
            qdot = []
            qddot = []
            for i in range(num_points):
                t = float(i) / self.rate
                q.append(A[joint][5] * t**5 + A[joint][4] * t**4 + A[joint][3] * t**3 + A[joint][2] * t**2 + A[joint][1] * t + A[joint][0])
                qdot.append(5*A[joint][5] * t**4 + 4*A[joint][4] * t**3 + 3*A[joint][3] * t**2 + 2*A[joint][2] * t + A[joint][1])
                qddot.append(20*A[joint][5] * t**3 + 12*A[joint][4] * t**2 + 6*A[joint][3] * t + 2*A[joint][2])

            # appen list of joint position i to q
            self.q.append(q)
            self.qdot.append(qdot)
            self.qddot.append(qddot)


# if main do test
if __name__ == '__main__':
    # trajectory parameters
    q0 = [0.52, 0.52]
    q0dot = [0,0]
    qf = [2.09, -2.62]
    qfdot = [1, 1]
    traj_rate = 20
    tf = 3

    TGen = CubicPoly(q0,q0dot,qf,qfdot,tf,traj_rate)
    TGen.run()
    traj = TGen.q
    traj_dot = TGen.qdot
    traj_ddot = TGen.qddot

    t = np.linspace(0, tf, traj_rate*tf)
    theta1 = traj[0]
    theta2 = traj[1]
    omega1 = traj_dot[0]
    omega2 = traj_dot[1]
    alpha1 = traj_ddot[0]
    alpha2 = traj_ddot[1]


    print(len(t))
    print(len(theta1))


    f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True, sharey=False)
    ax1.plot(t,theta1, color='b', label='Theta1')
    ax1.plot(t,theta2, color='r', label='Theta2')
    ax1.grid()
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Angular Position (rad)')
    ax1.set_title('Joint Position vs Time')
    ax1.legend(loc='upper left')

    ax2.plot(t,omega1, color='b', label='Omega1')
    ax2.plot(t,omega2, color='r', label='Omega2')
    ax2.grid()
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angular Velocity (rad/s)')
    ax2.set_title('Joint Velocity vs Time')
    ax2.legend(loc='upper left')

    ax3.plot(t,alpha1, color='b', label='Alpha1')
    ax3.plot(t,alpha2, color='r', label='Alpha2')
    ax3.grid()
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angular Acceleration (rad/s/s)')
    ax3.set_title('Joint Acceleration vs Time')
    ax3.legend(loc='upper left')

    plt.show()
