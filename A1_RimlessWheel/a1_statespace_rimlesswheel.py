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
sim_time = 10

"""
Part 1: Region of Attraction for rimless wheel
"""
"""
#Creating state space for rimless wheel initial conditions
state_space_range = 50
theta_range = np.linspace(-alpha + gamma, alpha + gamma, state_space_range)
thetadot_range = np.linspace(-1, 5, state_space_range)

#make empty lists to store when limit cycle or stall
limitcycle_theta = []
limitcycle_thetadot = []
stalled_theta = []
stalled_thetadot = []
notsteady_theta = []
notsteady_thetadot = []

for theta_point in theta_range:
    for thetadot_point in thetadot_range:
        initial_state = np.array([theta_point, thetadot_point])
        time_traj, state_traj, steady_state_behavior, switch_angular_velocity = singlesim(spoke_number, gamma, length, initial_state, sim_time)
        if steady_state_behavior == "Limit Cycle":
            limitcycle_theta.append(theta_point)
            limitcycle_thetadot.append(thetadot_point)
        elif steady_state_behavior == "Stalled":
            stalled_theta.append(theta_point)
            stalled_thetadot.append(thetadot_point)
        else:
            notsteady_theta.append(theta_point)
            notsteady_thetadot.append(thetadot_point)

# #velocities sanity check
# test_initial_state = np.array([0, 2]) #showed green
# test_time_traj, test_state_traj, test_steady_state_behavior, test_switch_angular_velocity = singlesim(spoke_number, gamma, length, test_initial_state, sim_time)
# print("Switch velocities [0,2]:", test_switch_angular_velocity)

#plotting state space code
plt.close('all')
plt.figure()
plt.plot(limitcycle_theta, limitcycle_thetadot, 'go', label="Limit Cycle")
plt.plot(stalled_theta, stalled_thetadot, 'ro', label="Stalled")
plt.plot(notsteady_theta, notsteady_thetadot, 'bo', label="Not yet steady state")
plt.xlabel("Initial Angle (radians)")
plt.ylabel("Initial Angular Velocity (rad/s)")
plt.title("Rimless Wheel State Space")
plt.legend()
plt.tight_layout()
plt.show()

#extra sanity check for stalled points
#print("Points at which stalling occurred:")
#print(np.array([stalled_theta, stalled_thetadot]).T)

"""

"""
Part 2: Poincare section
"""
# Poincare return map

#chose limit cycle initial condition area
poincare_initial_state = np.array([0, 1])

time_traj, state_traj, steady_state_behavior, switch_angular_velocity = singlesim(spoke_number, gamma, length, poincare_initial_state, 10)
thetadot_n = switch_angular_velocity[:-1] #start at the first switch and go to the second to last switch
thetadot_next = switch_angular_velocity[1:] #start at the second switch and go to the last switch

#plotting return map
plt.figure()
plt.scatter(thetadot_n, thetadot_next, label="Return map")

# line with slope 1
xvals = np.linspace(min(thetadot_n), max(thetadot_n), 100)
yvals = xvals
plt.plot(xvals, yvals, 'r-', label="Line")

#labels
plt.xlabel("theta_dot_n")
plt.ylabel("theta_dot_n+1")
plt.title("Poincare Return Map")
plt.legend()
plt.show()

#find exact fixed point values
for n in range(len(thetadot_n)):
    if np.isclose(thetadot_next[n], thetadot_n[n], rtol=0.0001) == True:
        print("Fixed point found at theta_dot =", thetadot_n[n])
print("done")