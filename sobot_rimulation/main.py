# Run Dynamic System Simulation and Control

# Script to perform high level control of robot manipulator



if __name__ == '__main__':
    traj_rate = 20
    control_rate = 1000
    t_last = 0
    run_flag = True
    num_traj_points = len(trajectory)
    curr_traj_point = 0

    while(run_flag):
        t_now = grab_curr_time          # grab current time in nanoseconds

        delta_t_traj = (t_now - t_last_traj)
        delta_t_control = (t_now - t_last_cont)

        # grab next reference trajectory via point
        if(delta_t_traj > 1/traj_rate):
            grab_next_trajectory_point   # grab the next trajectory point from the list (q, q_dot, q_ddot)
            t_last_traj = t_now
            curr_traj_point = curr_traj_point + 1

        # update control effort and apply it to the system
        if(delta_t_cont > 1/control_rate):
            update_system_matrices      # update Inertia, Coriolis, Gravity matrices with current system states

            calculate_control_effort    # calculate control effort from trajectory point (q, q_dot, q_ddot) and system matrices

            apply_control_effort        # inject system model with control torques

            propogate_state             # propogate system model given control torques forward one time step (1/control_rate)

            t_last_control = t_now

        # if the time since the last trajectory via in the list is loaded is larger than 5 seceonds, exit simulation
        if(curr_traj_point == num_traj_points and delta_t_traj > 5000000):
            run_flag = False
