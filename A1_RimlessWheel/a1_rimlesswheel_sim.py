"""
Rimless wheel dynamics simulation instructions:
This code simulates the dynamics of a rimless wheel with a user-given number of spokes, incline slope angle, and
spoke length. To run the simulation, first, define those variables mentioned. Then, define the intial condition
(intial_state) of the system, which is the angle of the stance leg to vertical and the angular velocity.
Using too small of an angular velocity will result in no spoke switching, shown by the absence
of sharp spikes. After defining the initial conditions, set the simulation time as needed and then
simply run the code, outputting the time trajectory, state trajectory, and steady state behavior of the system.
"""

#import necessary libraries
import numpy as np
import matplotlib.pyplot as plt

#dynamics of spokes (treat as inverted pendulum)
def spoke_dynamics(theta, theta_dot, length, gravity):
    #eom for inverted pendulum
    theta_ddot = (gravity / length) * np.sin(theta)
    return np.array([theta_dot, theta_ddot])

def singlesim(spoke_number, gamma, length, initial_state, sim_time):

    #pre-defined variables
    alpha = (2*np.pi/spoke_number)/2 #angle between adjacent spokes in radians
    gravity = 9.81 #acceleration due to gravity in m/s^2

    #initialize variables
    state_traj = np.zeros((2, 1))
    state_traj[:, 0] = initial_state
    switch_angular_velocity = []  # tracks angular velocity at switchs over time
    steady_state_behavior = None #initialize steady state behavior variable
    step = 0
    t = 0
    time_traj = [t]

    #simulation loop including switch between spokes
    while t < sim_time:
        #better time efficiency by switching timestep
        if abs(state_traj[0,step] - (alpha+gamma)) < 0.05: #within .05 rads of switch
            timestep = 1e-5 #smaller timestep for more accurate switch
        else:
            timestep = 1e-2 #larger timestep for faster simulation

        #change dynamics if stance spoke switch
        if state_traj[0, step] >= alpha + gamma: #switch spoke condition
            state_traj[0, step] -= 2 * alpha  # make angle -alpha + gamma
            state_traj[1, step] *= np.cos(2 * alpha)  # change angular velocity
            switch_angular_velocity.append(state_traj[1, step])  # store angular velocity after switch
            if len(switch_angular_velocity) > 1 and np.isclose(switch_angular_velocity[-1], switch_angular_velocity[-2], rtol=0.01):
                        steady_state_behavior = "Limit Cycle"
                        #break #save time by stopping if steady state reached

        #update state using explicit euler method
        state_dot = spoke_dynamics(state_traj[0, step], state_traj[1, step], length, gravity)
        new_state = state_traj[:, step] + timestep * state_dot
        state_traj = np.column_stack((state_traj, new_state))

        #go to next time step
        t += timestep
        step += 1
        time_traj.append(t)

    #other cases for not limit cycle
    if steady_state_behavior != "Limit Cycle":
        if len(switch_angular_velocity) > 1:
            steady_state_behavior = "Not yet steady state" #assume less than 2 switch = stalled
        else:
            steady_state_behavior = "Stalled"

    return time_traj, state_traj, steady_state_behavior, switch_angular_velocity

