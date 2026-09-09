"""
Code for plotting trajectories.
"""
import numpy as np
import matplotlib.pyplot as plt
from a1_rimlesswheel_sim import singlesim

#simulation parameters
spoke_number = 8 #number of spokes N
gamma = np.pi/16 #angle of slope in radians
alpha = (2*np.pi/spoke_number)/2 #angle between adjacent spokes in radians
length = 1 #length of the spokes in meters
gravity = 9.81 #acceleration due to gravity in m/s^2
initial_state = np.array([.5, -1]) #Angle, Angular velocity
sim_time = 10.0

#run function for plotting
time_traj, state_traj, notused1, notused2 = singlesim(spoke_number, gamma, length, initial_state, sim_time)

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
