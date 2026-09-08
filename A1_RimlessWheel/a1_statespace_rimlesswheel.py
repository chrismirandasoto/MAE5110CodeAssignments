"""
State space
"""
#import necessary libraries
import numpy as np
import matplotlib.pyplot as plt
from a1_rimlesswheel_sim import singlesim

#variables for rimless wheel simulation
#time_traj, state_traj, steady_state_behavior = singlesim(spoke_number, gamma, length, initial_state, sim_time)
spoke_number = 8 #number of spokes N
gamma = np.pi/16 #angle of slope in radians
alpha = (2*np.pi/spoke_number)/2 #angle between adjacent spokes in radians
length = 1 #length of the spokes in meters
sim_time = 3.0

#Creating state space for rimless wheel initial conditions
state_space_range = 5
theta_range = np.linspace(-alpha + gamma, alpha + gamma, state_space_range)
thetadot_range = np.linspace(-1, 5, state_space_range)

#make empty lists to store when limit cycle or stall
limitcycle_theta = []
limitcycle_thetadot = []
stalled_theta = []
stalled_thetadot = []

for theta_point in theta_range:
    for thetadot_point in thetadot_range:
        initial_state = np.array([theta_point, thetadot_point])
        time_traj, state_traj, steady_state_behavior = singlesim(spoke_number, gamma, length, initial_state, sim_time)
        if steady_state_behavior == "Limit Cycle":
            limitcycle_theta.append(theta_point)
            limitcycle_thetadot.append(thetadot_point)
        else:
            stalled_theta.append(theta_point)
            stalled_thetadot.append(thetadot_point)

plt.close('all')
plt.figure()
plt.plot(limitcycle_theta, limitcycle_thetadot, 'go', label="Limit Cycle")
plt.plot(stalled_theta, stalled_thetadot, 'ro', label="Stalled")
plt.xlabel("Initial Angle (radians)")
plt.ylabel("Initial Angular Velocity (rad/s)")
plt.title("Rimless Wheel State Space")
#plt.legend()
plt.tight_layout()
plt.show()