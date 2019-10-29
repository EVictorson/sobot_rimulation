#!/usr/bin/env python3

import sys
import numpy as np
from scipy.integrate import odeint, ode
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import ODESolver
import Model


print("Starting!")

class DoublePenSim:

  def __init__(self):
    # Pendulum rod leModel.DPModel()ngths (m), bob masses (kg).
    self.g = 9.807
    self.q1 = []
    self.q2 = []
    self.q1dot = []
    self.q2dot = []
    self.x1 = []
    self.y1 = []
    self.x2 = []
    self.y2 = []
    self.state = []

  def init(self):
    model = Model.DPModel()
    self.model = model
    self.L1 = model.L1
    self.L2 = model.L2
    self.m1 = model.m1
    self.m2 = model.m2

  def deriv(self, y, t, L1, L2, m1, m2):
    """Return the first derivatives of y = theta1, z1, theta2, z2."""
    theta1, z1, theta2, z2 = y

    L1 = self.L1
    L2 = self.L2
    m1 = self.m1
    m2 = self.m2
    g = self.g

    c, s = np.cos(theta1-theta2), np.sin(theta1-theta2)

    theta1dot = z1
    z1dot = (m2*g*np.sin(theta2)*c - m2*s*(L1*z1**2*c + L2*z2**2) -
             (m1+m2)*g*np.sin(theta1)) / (L1 / (m1 + m2*s**2))

    theta2dot = z2
    z2dot = ((m1+m2)*(L1*z1**2*s - g*np.sin(theta2) + g*np.sin(theta1)*c) +
             m2*L2*z2**2*s*c) / (L2 / (m1 + m2*s**2))


    return theta1dot, z1dot, theta2dot, z2dot

  def calc_E(self, y):
    """Return the total energy of the system."""

    L1 = self.L1
    L2 = self.L2
    m1 = self.m1
    m2 = self.m2
    g = self.g

    th1, th1d, th2, th2d = y.T
    V = -(m1+m2)*L1*g*np.cos(th1) - m2*L2*g*np.cos(th2)
    T = 0.5*m1*(L1*th1d)**2 + 0.5*m2*((L1*th1d)**2 + (L2*th2d)**2 +
            2*L1*L2*th1d*th2d*np.cos(th1-th2))
    return T + V

  def make_plot(self, i):
    L1 = self.L1
    L2 = self.L2
    m1 = self.m1
    m2 = self.m2
    g = self.g
    r = self.r

    # Plot and save an image of the double pendulum configuration for time
    # point i.
    # The pendulum rods.
    self.ax.plot([0, self.x1[i], self.x2[i]], [0, self.y1[i], self.y2[i]], lw=2, c='k')

    # Circles representing the anchor point of rod 1, and bobs 1 and 2.
    c0 = Circle((0, 0), self.r/2, fc='k', zorder=10)
    c1 = Circle((self.x1[i], self.y1[i]), self.r, fc='b', ec='b', zorder=10)
    c2 = Circle((self.x2[i], self.y2[i]), self.r, fc='r', ec='r', zorder=10)
    self.ax.add_patch(c0)
    self.ax.add_patch(c1)
    self.ax.add_patch(c2)

    # The trail will be divided into ns segments and plotted as a fading line.
    ns = 20
    s = self.max_trail // ns

    for j in range(ns):
      imin = i - (ns-j)*s
      if imin < 0:
        continue
      imax = imin + s + 1
      # The fading looks better if we square the fractional length along the
      # trail.
      alpha = (j/ns)**2
      self.ax.plot(self.x2[imin:imax], self.y2[imin:imax], c='r', solid_capstyle='butt',
                lw=2, alpha=alpha)

    # Centre the image on the fixed anchor point, and ensure the axes are equal
    self.ax.set_xlim(-L1-L2-r, L1+L2+r)
    self.ax.set_ylim(-L1-L2-r, L1+L2+r)
    self.ax.set_aspect('equal', adjustable='box')
    plt.axis('off')
    plt.savefig('frames/_img{:04d}.png'.format(i//self.di), dpi=72)
    plt.cla()


  def plot_states(self):
    x = range(len(self.x1))
    plt.plot(x, self.q1, 'r', x, self.q2, 'b')
    plt.xlabel('Sample Num')
    plt.ylabel('Angular Position (rad)')
    plt.grid()
    plt.show()

  def run(self):
    DPModel = Model.DPModel()
    self.U0 = self.model.U0

    #L1 = self.L1
    #L2 = self.L2
    #m1 = self.m1
    #m2 = self.m2
    tmax, dt = 30, 0.01
    tsize = int(tmax/dt)
    #solver = ODESolver.ODE(self.model.f, dt)
    #solver.set_initial_condition(self.model.U0)

    integrator = ode(self.model.f).set_integrator('lsoda')


# calculate single step numerical solution
    for i in range(int(tmax/dt)):
      integrator.set_initial_value(self.model.U0, 0)

      if(integrator.successful()):
        u = integrator.integrate(integrator.t+dt)
        self.state.append(u)
        self.U0 = u



      print('solver single step called')
      #new_states = solver.solve_single_step(dt)
      #new_states = solver.solve_single_step_ode(dt)
      # TODO: Add control function here

    #for state in solver.state:
    for state in self.state:
      #q1 = theta1, q2 = theta2
      q1 = state[0]
      q1dot = state[1]
      q2 = state[2]
      q2dot = state[3]
      self.q1.append(q1)
      self.q1dot.append(q1dot)
      self.q2.append(q2)
      self.q2dot.append(q2dot)

      # Convert to Cartesian coordinates of the two bob positions.
      x1 = self.model.L1 * np.sin(q1)
      y1 = -1*self.model.L1 * np.cos(q1)
      x2 = x1 + self.model.L2 * np.sin(q2)
      y2 = y1 - self.model.L2 * np.cos(q2)
      #print(x1)

      self.x1.append(x1)
      self.y1.append(y1)
      self.x2.append(x2)
      self.y2.append(y2)

    # Plotted bob circle radius

    self.plot_states()

    self.r = 0.05
    # Plot a trail of the m2 bob's position for the last trail_secs seconds.
    trail_secs = 1
    # This corresponds to max_trail time points.
    self.max_trail = int(trail_secs / dt)
    # Make an image every di time points, corresponding to a frame rate of fps
    # frames per second.
    # Frame rate, s-1
    fps = 100
    self.di = int(1/fps/dt)
    self.fig = plt.figure(figsize=(8.3333, 6.25), dpi=72)
    self.ax = self.fig.add_subplot(111)

    for i in range(0, tsize, self.di):
      print(i // self.di, '/', tsize // self.di)
      self.make_plot(i)


if __name__ == "__main__":
  DPS = DoublePenSim()
  DPS.init()
  DPS.run()
