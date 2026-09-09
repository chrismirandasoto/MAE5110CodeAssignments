#import necessary libraries
import numpy as np
import matplotlib.pyplot as plt
from a1_rimlesswheel_sim import singlesim

"""
Part 1: Region of Attraction for rimless wheel
"""
#default variables for rimless wheel simulation
#time_traj, state_traj, steady_state_behavior = singlesim(spoke_number, gamma, length, initial_state, sim_time)
spoke_number = 8 #number of spokes N
gamma = np.pi/16 #angle of slope in radians
alpha = (2*np.pi/spoke_number)/2 #angle between adjacent spokes in radians
length = 1 #length of the spokes in meters
sim_time = 5


def RegionOfAttraction(gamma, spoke_number):
    #Creating state space for rimless wheel initial conditions
    state_space_range = 20
    theta_range = np.linspace(-alpha + gamma, alpha + gamma, state_space_range)
    thetadot_range = np.linspace(-3, 3, state_space_range)

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
            notused1, notused2, steady_state_behavior, notused3 = singlesim(spoke_number, gamma, length, initial_state, sim_time)
            if steady_state_behavior == "Limit Cycle":
                limitcycle_theta.append(theta_point)
                limitcycle_thetadot.append(thetadot_point)
            elif steady_state_behavior == "Stalled":
                stalled_theta.append(theta_point)
                stalled_thetadot.append(thetadot_point)
            else:
                notsteady_theta.append(theta_point)
                notsteady_thetadot.append(thetadot_point)
    return limitcycle_theta, limitcycle_thetadot, stalled_theta, stalled_thetadot, notsteady_theta, notsteady_thetadot


#plotting state space code
limitcycle_theta, limitcycle_thetadot, stalled_theta, stalled_thetadot, notsteady_theta, notsteady_thetadot = RegionOfAttraction(gamma, spoke_number)

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

#velocities sanity check
test_initial_state = np.array([0, 2]) #showed green
test_time_traj, test_state_traj, test_steady_state_behavior, test_switch_angular_velocity = singlesim(spoke_number, gamma, length, test_initial_state, sim_time)
print("Switch velocities [0,2]:", test_switch_angular_velocity)

#extra sanity check for stalled points
print("Points at which stalling occurred:")
print(np.array([stalled_theta, stalled_thetadot]).T)

"""
Part 2: Poincare section
"""
# Poincare return map

#chose limit cycle initial condition area
poincare_initial_state = np.array([1, 2])

plt.figure()
time_traj, state_traj, steady_state_behavior, switch_angular_velocity = singlesim(spoke_number, gamma, length, poincare_initial_state, 10)
if len(switch_angular_velocity) > 1: # can't do poincare with less than 2 values
    thetadot_n = switch_angular_velocity[:-1] #start at the first switch and go to the second to last switch
    thetadot_next = switch_angular_velocity[1:] #start at the second switch and go to the last switch
    #plotting return map
    plt.scatter(thetadot_n, thetadot_next, label="Return map")

# line with slope 1
if len(switch_angular_velocity) < 2:
    xvals = np.linspace(0, 2, 100) #random slope 1 line if no points
else:
    xvals = np.linspace(min(thetadot_n), max(thetadot_n), 100) #identity line
yvals = xvals
plt.plot(xvals, yvals, 'r-', label="Identity Line")

#labels
plt.xlabel("theta_dot_n")
plt.ylabel("theta_dot_n+1")
plt.title("Poincare Return Map")
plt.legend()
plt.show()

# #find exact fixed point values
# for n in range(len(thetadot_n)):
#     if np.isclose(thetadot_next[n], thetadot_n[n], rtol=0.0001) == True:
#         print("Fixed point found at theta_dot =", thetadot_n[n])

"""
Part 3: Floquet Multipier
"""
#function to find floquet multiplier
def floquet_finder(gamma, spoke_number):
    perturbation = 0.05 # magnitude of perturbation away from limit cycle
    alpha = (2*np.pi/spoke_number)/2 #angle between adjacent spokes in radians
    poincare_angle = gamma - alpha # angle at which poincare section is taken
    # get value for limit cycle
    baseline_initial = np.array([poincare_angle, 2])  # any starting velocity works
    notused1, notused2, notused3, baseline_angular_velocities = singlesim(spoke_number, gamma, length, baseline_initial, 10)
    #check if limit cycle never reached
    if len(baseline_angular_velocities) == 0:
        print(f"Limit cycle not reached at gamma={gamma:.4f}, spoke_number={spoke_number}")
        return np.nan
    limit_cycle_fixed = baseline_angular_velocities[-1]  # steady state limit cycle value
    floquet_init_under = limit_cycle_fixed - perturbation
    floquet_init_over =  limit_cycle_fixed + perturbation

    #run simulation for each perturbed initial condition
    notused1, notused2, notused3, angular_velocity_under = singlesim(spoke_number,
    gamma, length, np.array([poincare_angle, floquet_init_under]), 10)

    notused1, notused2, notused3, angular_velocity_over = singlesim(spoke_number,
    gamma, length, np.array([poincare_angle, floquet_init_over]), 10)

    if len(angular_velocity_under) == 0 or len(angular_velocity_over) == 0:
        #print(f"Limit cycle not reached at gamma={gamma:.4f}, spoke_number={spoke_number}")
        return np.nan
    #look at first value
    next_under = angular_velocity_under[0]
    next_over = angular_velocity_over[0]

    #floquet multipier calculation
    Floquet = (next_over - next_under) / (2*perturbation)
    #print("Floquet Multiplier Value:", Floquet)

    return Floquet

"""
Part 4: Inclination and Spoke Number Sweep
"""
#define values for sweeping
inclination_values = np.linspace(0, np.pi/4, 10) # sweep inclination values
spokenumber_values = np.linspace(6, 12, 7) # sweep spoke number values

#ROA Sweeps
#inclination sweep for ROA
ROA_inclination = []
for inclination_angle in inclination_values:
    #call ROA function per inclination angle
    limitcycle_theta, limitcycle_thetadot, stalled_theta, stalled_thetadot, notsteady_theta, notsteady_thetadot = RegionOfAttraction(inclination_angle, 8)
    # find how many total points there are by adding lengths of each behavior list
    total_points = len(limitcycle_theta) + len(stalled_theta) + len(notsteady_theta)
    limitcycle_points = len(limitcycle_theta) # just limit cycle points
    ROA_ratio= limitcycle_points/total_points
    ROA_inclination.append(ROA_ratio)

plt.close('all')
plt.figure()
plt.plot(inclination_values, ROA_inclination, label = "RoA Change with Inclination")
plt.xlabel("Inclination Angle (radians)")
plt.ylabel("RoA Ratio")
plt.title("RoA Ratio with Inclination")
plt.legend()
plt.tight_layout()
plt.show()

#spoke number sweep for ROA
ROA_spokenumber = []
for spoke_number in spokenumber_values:
    #call ROA function per spoke number
    limitcycle_theta, limitcycle_thetadot, stalled_theta, stalled_thetadot, notsteady_theta, notsteady_thetadot = RegionOfAttraction(np.pi/16, spoke_number)
    # find how many total points there are by adding lengths of each behavior list
    total_points = len(limitcycle_theta) + len(stalled_theta) + len(notsteady_theta)
    limitcycle_points = len(limitcycle_theta) # just limit cycle points
    ROA_ratio= limitcycle_points/total_points
    ROA_spokenumber.append(ROA_ratio)

plt.close('all')
plt.figure()
plt.plot(spokenumber_values, ROA_spokenumber, label = "RoA Change with Spoke Number")
plt.xlabel("Spoke Number")
plt.ylabel("RoA Ratio")
plt.title("RoA Ratio with Spoke Number")
plt.legend()
plt.tight_layout()
plt.show()

#FLOQUET SWEEPS
#inclination sweep for floquet
floquet_multipliers_inclination = []
for inclination_angle in inclination_values:
    floquet_multipliers_inclination.append(floquet_finder(inclination_angle, 8))

plt.close('all')
plt.figure()
plt.plot(inclination_values, floquet_multipliers_inclination, label = "Floquet Change with Inclination")
plt.xlabel("Inclination Angle (radians)")
plt.ylabel("Floquet Multiplier")
plt.title("Floquet Change with Inclination")
plt.legend()
plt.tight_layout()
plt.show()

#spoke number sweep for floquet
floquet_multipliers_spokenumber = []
for spokenumber in spokenumber_values:
    floquet_multipliers_spokenumber.append(floquet_finder(np.pi/16, spokenumber))

plt.close('all')
plt.figure()
plt.plot(spokenumber_values, floquet_multipliers_spokenumber, label = "Floquet Change with Spoke Number")
plt.xlabel("Spoke Number")
plt.ylabel("Floquet Multiplier")
plt.title("Floquet Change with Spoke Number")
plt.legend()
plt.tight_layout()
plt.show()





