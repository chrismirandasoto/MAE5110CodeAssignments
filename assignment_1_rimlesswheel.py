#Rimless wheel dynamics

#import necessary libraries
import numpy as np
import matplotlib.pyplot as plt

#define variables
spoke_number = 8 #number of spokes N
gamma = np.pi/16 #angle of slope in radians
alpha = (2*np.pi/spoke_number)/2 #angle between adjacent spokes in radians
length = 1 #length of the spokes in meters
gravity = 9.81 #acceleration due to gravity in m/s^2

#state variables and simulation conditions
initial_state = np.array([-alpha + gamma, 0])
timestep = 1e-5
sim_time = 5.0
n_timesteps = int(sim_time / timestep) + 1
time_traj = np.arange(n_timesteps) * timestep
state_traj = np.zeros((2, n_timesteps))
state_traj[:, 0] = initial_state

#dynamics of spokes (treat as inverted pendulum)
def spoke_dynamics(theta, theta_dot, length, gravity):
    #eom for inverted pendulum
    theta_ddot = (gravity / length) * np.sin(theta)
    return np.array([theta_dot, theta_ddot])

#simulation loop including switch between spokes
for step, t in enumerate(time_traj[:-1]):
    #change dynamics if stance spoke switch
    if state_traj[0, step] >= alpha + gamma:
        state_traj[0, step] -= 2 * alpha  # make angle -alpha + gamma
        state_traj[1, step] *= np.cos(2 * alpha)  # change angular velocity
    state_dot = spoke_dynamics(state_traj[0, step], state_traj[1, step], length, gravity)
    state_traj[:, step + 1] = state_traj[:, step] + timestep * state_dot  # explicit euler

plt.close('all')
plt.figure()
plt.plot(time_traj, state_traj[0, :], label="Angle (radians)")
plt.xlabel("Time (s)")
plt.ylabel("Angle (radians)")
plt.title("Stance Leg Angle to Vertical vs Time")
plt.legend()
plt.tight_layout()
plt.show()
