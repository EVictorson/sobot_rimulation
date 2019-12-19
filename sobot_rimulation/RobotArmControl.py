# Perform Dynamic Inversion Control

import RobotDynamics
import sys
import ODESolver
import numpy as np
import TrajectoryGen

class RobotArmControl:

    def __init__(self):
        self.q_des = np.asarray([1000, 1000])
        self.qdot_des = np.asarray([1000, 1000])
        self.qddot_des = np.asarray([1000, 1000])
        self.control_rate = 1000
        self.apply_control = True
        self.curr_step = -1
        # trajectory parameters
        self.q0 = [0.52, 0.52]
        self.q0dot = [0,0]
        self.qf = [2.09, -2.62]
        self.qfdot = [1, 1]
        self.traj_rate = 20
        self.tf = 3
        self.U0 = [0.52, 0, 0.52, 0]
        self.state = []

    # pass in model class
    def init(self):
        q0 = self.q0
        q0dot = self.q0dot
        qf = self.qf
        qfdot = self.qfdot
        tf = self.tf
        traj_rate = self.traj_rate
        dt = 1/1000
        self.Model = RobotDynamics.RobotDynamics()
        self.ODESolver = ODESolver.ForwardEuler(self.deriv, dt)
        self.ODESolver.set_initial_condition(self.U0)
        self.TGen = TrajectoryGen.CubicPoly(q0,q0dot,qf,qfdot,tf,traj_rate)
        self.TGen.run()
        #self.q_des = self.TGen.q
        #self.qdot_des = self.TGen.qdot
        #self.qddot_des = self.TGen.qddot

    def increment_traj_point(self):
        self.curr_step = self.curr_step + 1
    #def tick(self):
        # only implemented in inhereted classes


    #def calculate_control_effort(self):
        # only implemented in inhereted classes

    # sample current system state
    def sample_states(self):
        if(self.curr_step == 0):
            print(self.curr_step)
            self.q = np.asarray(self.q0)
            self.qdot = np.asarray(self.q0dot)
        else:
            self.q = self.Model.q
            self.qdot = self.Model.qdot
            #self.qddot = self.Model.qddot

    def sample_trajectory(self):
        q = np.asarray(self.TGen.q)
        qd = np.asarray(self.TGen.qdot)
        qdd = np.asarray(self.TGen.qddot)
        #b = self.TGen.q[:][self.curr_step]
        #c = self.TGen.q[0]
        #self.q_des = np.asarray(self.TGen.q[self.curr_step][:])
        #self.qdot_des = np.asarray(self.TGen.qdot[self.curr_step])
        #self.qddot_des = np.asarray(self.TGen.qddot[self.curr_step])

        self.q_des = q[:,self.curr_step]
        self.qdot_des = qd[:,self.curr_step]
        self.qddot_des = qdd[:,self.curr_step]

        #print(self.TGen.q)
        #print(self.q_des)
        #print(self.qdot_des)
        #print(a[:,self.curr_step])
        #print(b)
        #print(c)

    # update Inertia, Coriolis, Friction, and Gravity matrices given system states
    def update_system_matrices(self):
        self.B = self.Model.get_B()
        self.C = self.Model.get_C()
        # for now, assume no friction
        #self.F = self.Model.get_F()
        self.G = self.Model.get_G()

    # return control effort
    def publish_control_effort(self):
        self.T = self.u
        return self.u


    # SEEMS TO BE BARFING HERE BECAUSE SOMETHING ABOUT CREATING ARRAY WITH LIST
    # THATS NOT SHAPED LIKE AN ARRAY
    def deriv(self, u, t):
        theta1, z1, theta2, z2 = u
        # will this make it 1 timestep behind?
        self.state.append([theta1, z1, theta2, z2])
        self.Model.q = np.asarray([theta1, theta2])
        self.Model.qdot = np.asarray([z1, z2])

        B = self.B
        C = self.C
        G = self.G
        T = self.T

        if(not self.apply_control):
            T = np.array([0,0])

        #B_inv = 1 / (B[0,0]*B[1,1] - B[1,0]*B[0,1])
        ########### issue seems to be here ###############

        z = np.array([[z1], [z2]])
        theta = np.array([[theta1], [theta2]])
        #z = np.array([[z1], [z2]])
        #theta = np.array([[theta1], [theta2]])
        ##################################################
        # Governing system of nonlinear differential equations
        thetadot = z

        print('T = ')
        print(T)
        print('C = ')
        print(C)
        print('B = ')
        print(B)
        print('G = ')
        print(G)
        print('theta_dot')
        print(thetadot)
        print('matmul(c,thetadot)')
        print(np.matmul(C,thetadot) )

        print('T - np.matmul(C,thetadot)- G')
        print(T - np.matmul(C,thetadot)- G)





        zdot = np.matmul(np.linalg.inv(B),(T - np.matmul(C,thetadot) - G))
        print('z = ')
        print(z)
        print('theta = ')
        print(theta)
        print('zdot = ')
        print(zdot)
        print('thetadot = ')
        print(thetadot)
        print('\n')
        print('\n')


        theta1dot = thetadot[0]
        theta2dot = thetadot[1]
        z1dot = zdot[0]
        z2dot = zdot[1]

        print('theta1dot, z1dot, theta2dot, z2dot')
        print([theta1dot, z1dot, theta2dot, z2dot])

        return [theta1dot, z1dot, theta2dot, z2dot]

    def propogate_states(self):
        self.ODESolver.solve_single_step(1/self.control_rate)


# PD control treating each joint as independent
class IndependentJointPIDControl(RobotArmControl):
    def __init__(self):
        self.q_des = np.array([1000, 1000])
        self.qdot_des = np.array([1000, 1000])
        self.qddot_des = np.array([1000, 1000])
        self.Kp = val
        self.Kd = val
        self.Ki = val

    #def tick(self):


    def calculate_control_effort(self):
        q_error = q_des - q
        qdot_error = qdot_des - qdot
        Kd = self.Kd * qdot_error
        Kp = self.Kp * q_error

# Feedback Linearization with PD control`
class DynamicInversionPDControl(RobotArmControl):
    def __init__(self):
        self.q_des = np.array([1000, 1000])
        self.qdot_des = np.array([1000, 1000])
        self.qddot_des = np.array([1000, 1000])
        self.control_rate = 1000
        self.apply_control = False
        self.Kp = np.array([[25, 0],[0, 25]])
        self.Kd = np.array([[5, 0],[0, 5]])
        self.q = np.array([0,0])
        self.qdot = np.array([0,0])
        self.curr_step = -1
        # trajectory parameters
        self.q0 = [0.52, 0.52]
        self.q0dot = [0,0]
        self.qf = [2.09, -2.62]
        self.qfdot = [1, 1]
        self.traj_rate = 20
        self.tf = 3
        self.q = np.array([0.52, 0.52])
        self.qdot = np.array([0, 0])
        self.U0 = [0.52, 0, 0.52, 0]
        self.state = []
        self.B = np.array([[1,2],[4,1]])
        self.C = np.ones((2,2))
        self.G = np.ones((2,1))
        self.T = np.ones((2,1))


    def calculate_control_effort(self):
        print(self.curr_step)
        print(self.qdot_des)
        print(self.qdot)
        qdot_error = np.subtract(self.qdot_des, self.qdot)
        q_error = np.transpose(self.q_des - self.q)

        print('qdot_des = ')
        print(self.qdot_des)
        print('qdot = ')
        print(self.qdot)
        print('qdot_error = ')
        print(qdot_error)

        Kd = np.matmul(self.Kd, qdot_error)
        Kp = np.matmul(self.Kp, q_error)

        # intermediate control input y
        y = (np.transpose(self.qddot_des) + Kd + Kp)
        print('kd = ')
        print(Kd)
        print('Kp = ')
        print(Kp)


        print('c*qdot')
        print(np.matmul(self.C, self.qdot))
        print('g')
        print(self.G)

        self.u = np.matmul(self.B, y) + np.matmul(self.C, self.qdot) + self.G
        print('u = ')
        print(self.u)
        print('y = ')
        print(y)

    def tick(self):
        if(self.q_des[0]== 1000):
            self.sample_trajectory()
        self.sample_states()
        self.update_system_matrices()
        self.calculate_control_effort()
        self.publish_control_effort()
        self.propogate_states()

class GravityCompensationPDControl(RobotArmControl):
    def __init__(self):
        self.Kp = 0
        self.Kd = 0


# Feedback linearization with PD Control and Lyapunov Redesign
class DILyapunovPDControl(RobotArmControl):
    def __init__(self):
        self.Kp = 0
        self.Kd = 0

# Feedback linearization with PD control and adaptive Lyapunov Redesign
# adaptive in parametric uncertainty
class DIAdaptiveLyapunovPDControl(RobotArmControl):
    def __init__(self):
        self.Kp = 0
        self.Kd = 0

# variable structure sliding mode controller
class SlidingModeControl(RobotArmControl):
    def __init__(self):
        self.Kp = 0
        self.Kd = 0

# Feedback linearization with adaptive model estimation and PD control
class AdaptiveDynamicInversionPDControl(RobotArmControl):
    def __init__(self):
        self.Kp = 0
        self.Kd = 0





    #def tick(self):

#class LyapunovAdaptiveControl(RobotArmControl):
    #def __init__(self):

    #def tick(self):
