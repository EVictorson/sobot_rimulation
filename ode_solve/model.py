#!/usr/bin/env python3

import numpy as np

class Model:

    def __init__(self):
        self.g = 9.807
        self.v0 = 5
        self.theta = 80*np.pi/180
        self.U0 = [0, self.v0*np.cos(self.theta), 0, self.v0*np.sin(self.theta)]


    def f(u, t):
        x, vx, y, vy = u
        return [vx, 0, vy, -1*self.g]
