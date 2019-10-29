# class with robot model parameters

import ODESolver
import numpy as np

class Model:
    def __init__(self, dt):
        self.M = []
        self.C = []
        self.F = []
        self.G = []
        self.q = []
        self.qdot = []
        self.qddot = []
        self.dt = dt

    def init(self):
        self.solver = ODESolver.RungeKutta4(self.f)

    def retrieve_M(self):
        return self.M

    def retrieve_C(self):
        return self.C

    def retrieve_F(self):
        return self.F

    def retrieve_G(self):
        return self.G

    # apply u to system model and use ODESolver to propogate state forward
    # one timestep
    def propogate_state(self, u):


    # callback function for ODE solver (system of 2 first order ODEs)
    def f(self, u, t):
        # only implemented in inhereted classes


class DoublePendulum(Model):
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

        self.M = val
        self.C = val
        self.F = val
        self.G = val

    def f(self, u, t):


class TriplePendulum(Model):
    def __init__(self):
        self.M = val
        self.C = val
        self.F = val
        self.G = val

    def f(self, u, t):

class Puma560(Model):
    def __init__(self):
        self.M = val
        self.C = val
        self.F = val
        self.G = val

    def f(self, u, t):

class StanfordManipulator(Model):
    def __init__(self):
        self.M = val
        self.C = val
        self.F = val
        self.G = val

    def f(self, u, t):
