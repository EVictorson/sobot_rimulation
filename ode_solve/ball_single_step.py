#!/usr/bin/env python3

import ODESolver
import matplotlib.pyplot as plt
import numpy as np
import Model

x = []
xdot = []
y = []
ydot = []
exact = []

# simulation params
T_final = 1
dt = 0.01
T_curr = 0
T = 0
T0 = 0

BallModel = Model.Model()
solver = ODESolver.ForwardEuler(BallModel.f)
solver.set_initial_condition(BallModel.U0)

def terminate(u, t, step_no):
    y = u[:,2]                   # all the y coordinates
    return y[step_no] < 0

# calculate single step numerical solution
for i in range(int(T_final/dt)):
    solver.solve_single_step(dt)
    #print(solver.state[-1])


for state in solver.state:
    x.append(state[0])
    xdot.append(state[1])
    y.append(state[2])
    ydot.append(state[3])
    exact.append(state[0]*np.tan(BallModel.theta) -
    BallModel.g*state[0]**2/(2*BallModel.v0**2)*1/(np.cos(BallModel.theta))**2 + 0)

plt.plot(x, y, 'r')
plt.plot(x, exact, 'b')
plt.legend(('numerical', 'exact'))
plt.title('Thrown Ball ODE Solution dt=%g' % dt)
plt.grid()
plt.xlabel('Horizontal Position (m)')
plt.ylabel('Vertical Position (m)')
plt.savefig('tmp_ball_single_step.pdf')
plt.show()
