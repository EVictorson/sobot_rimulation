# Perform Dynamic Inversion Control

import Model


class RobotArmControl:

    def __init__(self):
        self.q_des = 0
        self.qdot_des = 0
        self.qddot_des = 0

    # pass in model class
    def init(self, Model):
        self.model = Model

    def tick(self):
        # only implemented in inhereted classes


    def calculate_control_effort(self):
        # only implemented in inhereted classes

    # sample current system state
    def sample_states(self):
        self.q = self.model.q
        self.qdot = self.model.qdot
        self.qddot = self.model.qddot

    def sample_trajectory(self, traj):
        self.q_des = traj.q_des
        self.qdot_des = traj.qdot_des
        self.qddot_des = traj.qddot_des


    # update Inertia, Coriolis, Friction, and Gravity matrices given system states
    def update_system_matrices(self):
        self.M = self.Model.retrieve_M(self.q)
        self.C = self.Model.retrieve_C(self.q, self.qdot)
        self.F = self.Model.retrieve_F(self.qdot)
        self.G = self.Model.retrieve_G(self.q)

    # return control effort
    def publish_control_effort(self):
        return self.u


class IndependentJointPIDControl(RobotArmControl):
    def __init__(self):
        self.Kp = val
        self.Kd = val
        self.Ki = val

    def tick(self):


    def calculate_control_effort(self):
        q_error = q_des - q
        qdot_error = qdot_des - qdot
        Kd = self.Kd * qdot_error
        Kp = self.Kp * q_error


class DynamicInversionPDControl(RobotArmControl):
    def __init__(self):
        self.Kp = val
        self.Kd = val

    def calculate_control_effort(self):
        qdot_error = qdot_des - qdot
        q_error = q_des - q
        Kd = self.Kv * qdot_error
        Kp = self.Kp * q_error

        self.u = self.M * (qddot_des + Kv + Kp) + self.C * q_dot + F + G

    def tick(self):
        if(not self.q_des):
            self.sample_trajectory()
        self.sample_states()
        self.update_system_matrices()
        self.calculate_control_effort()
        return self.publish_control_effort()

class GravityCompensationPDControl(RobotArmControl):
    def __init__(self):
        self.Kp = val
        self.Kd = val
    def tick(self):

class LyapunovAdaptiveControl(RobotArmControl):
    def __init__(self):

    def tick(self):
