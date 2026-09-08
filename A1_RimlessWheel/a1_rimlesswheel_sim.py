"""
Rimless wheel dynamics simulation instructions:
This code simulates the dynamics of a rimless wheel with a user-given number of spokes, incline slope angle, and
spoke length. To run the simulation, first, define those variables mentioned. Then, define the intial condition
(intial_state) of the system, which is the angle of the stance leg to vertical and the angular velocity.
Using too small of an angular velocity will result in no spoke switching, shown by the absence
of sharp spikes. After defining the initial conditions, set the simulation time as needed and then
simply run the code, outputting a plot of the stance leg angle to vertical vs time.
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

    #state variables and simulation conditions
    timestep = 1e-5
    n_timesteps = int(sim_time / timestep) + 1
    time_traj = np.arange(n_timesteps) * timestep
    state_traj = np.zeros((2, n_timesteps))
    state_traj[:, 0] = initial_state

    touchdown_count = 0  # counter for number of spoke switches
    #simulation loop including switch between spokes
    for step, t in enumerate(time_traj[:-1]):
        #change dynamics if stance spoke switch
        if state_traj[0, step] >= alpha + gamma: #switch spoke condition
            state_traj[0, step] -= 2 * alpha  # make angle -alpha + gamma
            state_traj[1, step] *= np.cos(2 * alpha)  # change angular velocity
            touchdown_count += 1
        state_dot = spoke_dynamics(state_traj[0, step], state_traj[1, step], length, gravity)
        state_traj[:, step + 1] = state_traj[:, step] + timestep * state_dot  # explicit euler
        if touchdown_count > (sim_time/2):
            steady_state_behavior = "Limit Cycle"
        else:
            steady_state_behavior = "Stalled"
    return time_traj, state_traj, steady_state_behavior

#code in case of plotting trajectories
"""
#simulation parameters
spoke_number = 8 #number of spokes N
gamma = np.pi/16 #angle of slope in radians
alpha = (2*np.pi/spoke_number)/2 #angle between adjacent spokes in radians
length = 1 #length of the spokes in meters
gravity = 9.81 #acceleration due to gravity in m/s^2
initial_state = np.array([-alpha + gamma, 1]) #Angle, Angular velocity
sim_time = 10.0

#run function for plotting
time_traj, state_traj, steady_state_behavior = singlesim(spoke_number, gamma, length, initial_state, sim_time)

#plotting angle of stance leg to vertical vs time
plt.close('all')
plt.figure()
plt.plot(time_traj, state_traj[0, :], label="Angle (radians)")
plt.xlabel("Time (s)")
plt.ylabel("Angle (radians)")
plt.title("Stance Leg Angle to Vertical vs Time")
plt.legend()
plt.tight_layout()
plt.show()
"""