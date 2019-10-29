#!/usr/bin/env python3

import numpy as np

"""
model for trajectory of a ball is:
x_ddot = 0, y_ddot = -g, where (x,y) is position of ball
in order to analyze the numerical solution we must break these
second order ODEs into two sets of first order ODEs
x_dot = vx
vx_dot = 0
y_dot = vx
vy_dot = -g

with initial conditions
x(0) = 0
vx(0) = v0 cos(theta)
y(0) = y0
vy(0) = v0 sin(theta)

"""

class BallModel:
    def __init__(self):
        # 2nd order system states: x, xdot, y, ydot
        self.g = 9.807
        self.v0 = 5
        self.theta = 80*np.pi/180
        # initial conditions
        self.U0 = [0, self.v0*np.cos(self.theta), 0, self.v0*np.sin(self.theta)]
        print("Model Constructed!")

    def f(self, u, t):
        x, vx, y, vy = u
        return [vx, 0, vy, -1*self.g]


class DPModel:
        def __init__(self):
            # Pendulum rod lengths (m), bob masses (kg).
            self.L1 = 0.5
            self.L2 = 0.5
            self.m1 = 10
            self.m2 = 10
            # The gravitational acceleration (m/s/s).
            self.g = 9.807
            # Initial conditions: theta1, dtheta1/dt, theta2, dtheta2/dt.
            self.U0 = [3*np.pi/7, 0, 3*np.pi/4, 0]
            print("Model Constructed!")


        def f(self, u, t):
            #print('function called!')
            theta1, z1, theta2, z2 = u
            L1 = self.L1
            L2 = self.L2
            m1 = self.m1
            m2 = self.m2
            g = self.g

            #print('U = ', u)
            #print('theta1 = ', theta1)
            #print('theta2 = ', theta2)


            c, s = np.cos(theta1-theta2), np.sin(theta1-theta2)

            theta1dot = z1
            z1dot = (m2*g*np.sin(theta2)*c - m2*s*(L1*z1**2*c + L2*z2**2) -
                     (m1+m2)*g*np.sin(theta1)) / (L1 / (m1 + m2*s**2))

            theta2dot = z2
            z2dot = ((m1+m2)*(L1*z1**2*s - g*np.sin(theta2) + g*np.sin(theta1)*c) +
                     m2*L2*z2**2*s*c) / (L2 / (m1 + m2*s**2))

            #print('theta1dot = ', theta1dot)
            #print('theta2dot = ', theta2dot)
            #print('z1dot = ', z1dot)
            #print('z2dot = ', z2dot)


            return [theta1dot, z1dot, theta2dot, z2dot]

class PlanarArmModel:
        def __init__(self):
            # Pendulum rod lengths (m), bob masses (kg).
            self.L1 = 0.5
            self.L2 = 0.5
            self.m1 = 10
            self.m2 = 10
            # The gravitational acceleration (m/s/s).
            self.g = 9.807
            # Initial conditions: theta1, dtheta1/dt, theta2, dtheta2/dt.
            self.U0 = [3*np.pi/7, 0, 3*np.pi/4, 0]
            print("Model Constructed!")


        def f(self, u, t):
            #print('function called!')
            theta1, z1, theta2, z2 = u
            L1 = self.L1
            L2 = self.L2
            m1 = self.m1
            m2 = self.m2
            g = self.g

            #print('U = ', u)
            #print('theta1 = ', theta1)
            #print('theta2 = ', theta2)


            c, s = np.cos(theta1-theta2), np.sin(theta1-theta2)

            theta1dot = z1
            z1dot = (m2*g*np.sin(theta2)*c - m2*s*(L1*z1**2*c + L2*z2**2) -
                     (m1+m2)*g*np.sin(theta1)) / (L1 / (m1 + m2*s**2))

            theta2dot = z2
            z2dot = ((m1+m2)*(L1*z1**2*s - g*np.sin(theta2) + g*np.sin(theta1)*c) +
                     m2*L2*z2**2*s*c) / (L2 / (m1 + m2*s**2))

            #print('theta1dot = ', theta1dot)
            #print('theta2dot = ', theta2dot)
            #print('z1dot = ', z1dot)
            #print('z2dot = ', z2dot)


            return [theta1dot, z1dot, theta2dot, z2dot]
