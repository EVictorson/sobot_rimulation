#!/usr/bin/env python
# Run Dynamic System Simulation and Control

# Script to perform high level control of robot manipulator

import RobotDynamics
import numpy as np
import TrajectoryGen
import RobotArmControl
from datetime import datetime

class Simulate:
    def __init__(self):
        self.control_rate = 1000
        self.t_last_traj = 0
        self.t_last_control = 0
        self.run_flag = True
        self.curr_traj_point = 0

        # trajectory parameters
        self.q0 = [0.52, 0.52]
        self.q0dot = [0,0]
        self.qf = [2.09, -2.62]
        self.qfdot = [1, 1]
        self.traj_rate = 20
        self.tf = 3

    def run(self, TGen):
        TGen.run()
        self.q_des = TGen.q
        self.qdot_des = TGen.qdot
        self.qddot_des = TGen.qddot

    def micros(self):
        dt = datetime.now()
        return dt.microsecond

    def get_next_trajectory_point(self):
        self.curr_traj_point = self.curr_traj_point + 1

        self.curr_q_des = self.q_des[self.curr_traj_point]
        self.curr_qdot_des = self.qdot_des[self.curr_traj_point]
        self.curr_qddot_des = self.qddot_des[self.curr_traj_point]


if __name__ == '__main__':
    control_rate = 1000
    t_last_traj = 0
    t_last_control = 0
    run_flag = True
    curr_traj_point = 0

    # trajectory parameters
    q0 = [0.52, 0.52]
    q0dot = [0,0]
    qf = [2.09, -2.62]
    qfdot = [1, 1]
    traj_rate = 20
    tf = 3

    TGen = TrajectoryGen.CubicPoly(q0,q0dot,qf,qfdot,tf,traj_rate)
    #RDynamics = RobotDynamics.RobotDynamics()
    RControl = RobotArmControl.DynamicInversionPDControl()
    RControl.init()

    sim = Simulate()
    #sim.run(TGen)

    num_traj_points = RControl.TGen.get_traj_length


    while(run_flag):
        t_now = sim.micros()          # grab current time in nanoseconds

        delta_t_traj = (t_now - t_last_traj)
        delta_t_control = (t_now - t_last_control)

        # grab next reference trajectory via point
        if(delta_t_traj > 1/traj_rate):
            #grab_next_trajectory_point   # grab the next trajectory point from the list (q, q_dot, q_ddot)
            #sim.get_next_trajectory_point()
            RControl.increment_traj_point()
            RControl.sample_trajectory()

            t_last_traj = t_now

        # update control effort and apply it to the system
        if(delta_t_control > 1/control_rate):
            RControl.tick()

            #apply_control_effort        # inject system model with control torques

            #propogate_state             # propogate system model given control torques forward one time step (1/control_rate)

            t_last_control = t_now

        # if the time since the last trajectory via in the list is loaded is larger than 5 seceonds, exit simulation
        if(curr_traj_point == num_traj_points and delta_t_traj > 5000000):
            run_flag = False
