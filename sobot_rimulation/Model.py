# class with robot model parameters

import ODESolver
import numpy as np

class Model:
    def __init__(self, dt):
        self.B = []
        self.C = []
        self.F = []
        self.G = []
        self.q = []
        self.qdot = []
        self.qddot = []
        self.dt = dt

    def init(self):
        self.solver = ODESolver.RungeKutta4(self.f)

    def retrieve_B(self):
        return self.B

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

class TwoLinkPlanarArm(Model):
      def __init__(self):
        # parameters of the dynamic model
        self.L1 = 0.5    # (m)
        self.L2 = 0.5    # (m)
        self.a1 = 1      # (m)
        self.a2 = 1      # (m)
        self.ML1 = 50    # (kg)
        self.ML2 = 50    # (kg)
        self.IL1 = 10    # (kg m^2)
        self.IL2 = 10    # (kg m^2)
        self.kr1 = 100   # (unitless)
        self.kr2 = 100   # (unitless)
        self.Mm1 = 5     # (kg)
        self.Mm2 = 5     # (kg)
        self.Im1 = 0.01  # (kg m^2)
        self.Im2 = 0.01  # (kg m^2)
        self.Fm1 = 0.01  # (N m s / rad)
        self.Fm2 = 0.01  # (N m s / rad)
        self.Ra1 = 10    # (ohm)
        self.Ra2 = 10    # (ohm)
        self.kt1 = 2     # (N m / A)
        self.kt2 = 2     # (N m / A)
        self.kv1 = 2     # (V s / rad)
        self.kv2 = 2     # (V s / rad)
        self.g = 9.807   # (m/s/s)


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
