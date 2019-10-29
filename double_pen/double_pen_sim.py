#!/usr/bin/env python3
# code used to simulate double pendulum
# adapted from https://scipython.com/blog/the-double-pendulum/



import sys
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

print("Starting!")

class DoublePenSim:

  def __init__(self):
    # Pendulum rod lengths (m), bob masses (kg).
    self.L1 = 1
    self.L2 = 1
    self.m1 = 1
    self.m2 = 1
    # The gravitational acceleration (m/s/s).
    self.g = 9.807

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

    print('y = ', y)
    print('theta1 = ', theta1)
    print('theta2 = ', theta2)
    print('theta1dot = ', theta1dot)
    print('theta2dot = ', theta2dot)
    print('z1dot = ', z1dot)
    print('z2dot = ', z2dot)

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



  def run(self):

    L1 = self.L1
    L2 = self.L2
    m1 = self.m1
    m2 = self.m2

    # Maximum time, time point spacings and the time grid (all in s).
    tmax, dt = 30, 0.01
    t = np.arange(0, tmax+dt, dt)
    # Initial conditions: theta1, dtheta1/dt, theta2, dtheta2/dt.
    y0 = np.array([3*np.pi/7, 0, 3*np.pi/4, 0])

    # Do the numerical integration of the equations of motion
    y = odeint(self.deriv, y0, t, args=(L1, L2, m1, m2))

    # Check that the calculation conserves total energy to within some tolerance.
    EDRIFT = 5
    # Total energy from the initial conditions
    E = self.calc_E(y0)
    #if np.max(np.sum(np.abs(self.calc_E(y) - E))) > EDRIFT:
    #  sys.exit('Maximum energy drift of {} exceeded.'.format(EDRIFT))

    # Unpack z and theta as a function of time
    theta1, theta2 = y[:,0], y[:,2]

    # Convert to Cartesian coordinates of the two bob positions.
    self.x1 = L1 * np.sin(theta1)
    self.y1 = -L1 * np.cos(theta1)
    self.x2 = self.x1 + L2 * np.sin(theta2)
    self.y2 = self.y1 - L2 * np.cos(theta2)

    # Plotted bob circle radius
    self.r = 0.05
    # Plot a trail of the m2 bob's position for the last trail_secs seconds.
    trail_secs = 1
    # This corresponds to max_trail time points.
    self.max_trail = int(trail_secs / dt)


    # Make an image every di time points, corresponding to a frame rate of fps
    # frames per second.
    # Frame rate, s-1
    fps = 10
    self.di = int(1/fps/dt)
    self.fig = plt.figure(figsize=(8.3333, 6.25), dpi=72)
    self.ax = self.fig.add_subplot(111)

    for i in range(0, t.size, self.di):
      print(i // self.di, '/', t.size // self.di)
      self.make_plot(i)




if __name__ == "__main__":
  DPS = DoublePenSim()
  DPS.run()
