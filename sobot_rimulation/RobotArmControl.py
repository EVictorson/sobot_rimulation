# Perform Dynamic Inversion Control

import RobotDynamics
import ODESolver

class RobotArmControl:

    def __init__(self):
        self.q_des = 0
        self.qdot_des = 0
        self.qddot_des = 0
        self.control_rate = 1000
        self.apply_control = True

    # pass in model class
    def init(self, Model, ODESOlver):
        self.model = RobotDynamics.RobotDynamics()
        self.ODESolver = ODESolver.RungeKutta4(self.deriv)

    #def tick(self):
        # only implemented in inhereted classes


    #def calculate_control_effort(self):
        # only implemented in inhereted classes

    # sample current system state
    def sample_states(self):
        self.q = self.model.q
        self.qdot = self.model.qdot
        self.qddot = self.model.qddot

    def sample_trajectory(self, traj):
        self.q_des = traj[0]
        self.qdot_des = traj[1]
        self.qddot_des = traj[2]


    # update Inertia, Coriolis, Friction, and Gravity matrices given system states
    def update_system_matrices(self):
        self.B = self.Model.get_B(self.q)
        self.C = self.Model.get_C(self.q, self.qdot)
        self.F = self.Model.get_F(self.qdot)
        self.G = self.Model.get_G(self.q)

    # return control effort
    def publish_control_effort(self):
        self.T = self.u
        return self.u

    def deriv(self, u, t):
        theta1, z1, theta2, z2 = u
        self.state.append[theta1, z1, theta2, z2]

        B = self.B
        C = self.C
        G = self.G
        T = self.T

        B_inv = 1 / (B[0,0]*B[1,1] - B[1,0]*B[0,1])

        # Governing system of nonlinear differential equations
        thetadot = z
        zdot = np.linalg.inv(B)*(T - C*thetadot - G)

        theta1dot = thetadot[0]
        theta2dot = thetadot[1]
        z1dot = zdot[0]
        z2dot = zdot[1]

        return theta1dot, z1dot, theta2dot, z2dot

    def propogate_states(self):
        self.ODESolver.solve_single_step(1/self.control_rate)



class IndependentJointPIDControl(RobotArmControl):
    def __init__(self):
        self.q_des = 0
        self.qdot_des = 0
        self.qddot_des = 0
        self.Kp = val
        self.Kd = val
        self.Ki = val

    #def tick(self):


    def calculate_control_effort(self):
        q_error = q_des - q
        qdot_error = qdot_des - qdot
        Kd = self.Kd * qdot_error
        Kp = self.Kp * q_error


class DynamicInversionPDControl(RobotArmControl):
    def __init__(self):
        self.Kp = [[25, 0],[0, 25]]
        self.Kd = [[5, 0][0, 5]]

    def calculate_control_effort(self):
        qdot_error = qdot_des - qdot
        q_error = q_des - q
        Kd = self.Kv * qdot_error
        Kp = self.Kp * q_error

        # intermediate control input y
        y = (qddot_des + Kd + Kp)

        self.u = self.M * y + self.C * q_dot + self.G

    def tick(self):
        if(not self.q_des):
            self.sample_trajectory()
        self.sample_states()
        self.update_system_matrices()
        self.calculate_control_effort()
        self.publish_control_effort()

class GravityCompensationPDControl(RobotArmControl):
    def __init__(self):
        self.Kp = val
        self.Kd = val

    #def tick(self):

#class LyapunovAdaptiveControl(RobotArmControl):
    #def __init__(self):

    #def tick(self):
