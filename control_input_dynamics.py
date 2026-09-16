# Inverted pendulum walker control input code
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from models import inverted_pendulum_walker as model

#rk integrator
def rk4_step(state, params, dt):
    k1 = model.dynamics(0, state, params)
    k2 = model.dynamics(0, state + dt / 2 * k1, params)
    k3 = model.dynamics(0, state + dt / 2 * k2, params)
    k4 = model.dynamics(0, state + dt * k3, params)
    return state + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

#function for finding torque that counteracts current motion
def calculate_torque(state, params):
    mass, gravity, length = params["mass"], params["gravity"], params["length"]
    theta = state[0]
    angular_velocity = state[1]
    #torque bounds
    torque_min = -0.1 * mass * gravity * length
    torque_max = 0.05 * mass * gravity * length
    #PD control variables
    Kp, Kd = 28.48 , 10.67 #used AI to find good weights
    #want stability so need correction term and energy loss term
    desired_accel = -Kp * theta - Kd * angular_velocity
    #cancel gravity, then add the desired acceleration times inertia
    ankle_torque = -mass * gravity * length * np.sin(theta) + mass * length**2 * desired_accel
    return np.clip(ankle_torque, torque_min, torque_max)

#to make RoA, need code to know if walker stabilized in upright
def simulate_if_stabilized(state, params, time_step=0.005, sim_time=5.0, tol=1e-3):
    for step in range(int(sim_time / time_step)):
        params["ankle_torque"] = calculate_torque(state, params)
        state = rk4_step(state, params, time_step)
        theta, angular_velocity = state[0], state[1]
        if theta > params["incline"] + params["angle_of_attack"] or theta < params["incline"] - params["angle_of_attack"]: #past touchdown angle on either side
            return False
        #theta and angular velocity both very very close to zero continually
        if np.abs(theta) < tol and np.abs(angular_velocity) < tol:
            return True
    return False #assume stuck if neither condition triggered

#Sim and Plot RoA code
params = model.generate_params()
#grid of starting states from back wall to front wall
theta_back = params["incline"] - np.pi/7
theta_front = params["incline"] + np.pi/7
resolution = 40 #40 default, higher number = higher resolution
thetas = np.linspace(theta_back, theta_front, resolution)
angular_velocities = np.linspace(-1.5, 1.5, resolution)
#try every starting state and record true or false
RoA_space = np.zeros((len(thetas), len(angular_velocities)), dtype=bool)
for i, th0 in enumerate(thetas):
    for j, thdot0 in enumerate(angular_velocities):
        RoA_space[i, j] = simulate_if_stabilized(np.array([th0, thdot0]), params)
params["ankle_torque"] = 0.0  #reset so the walker sim starts unpowered
#plot
plt.pcolormesh(thetas, angular_velocities, RoA_space.T, shading="nearest", cmap="Blues")
plt.xlabel(r"$\theta$ [rad]")
plt.ylabel(r"$\dot{\theta}$ [rad/s]")
plt.title("Region of attraction of the ankle controller")
plt.show()

#checks if a point and its 3x3 square is within RoA
def check_if_in_RoA(state, RoA_space, thetas, angular_velocities):
    theta, angular_velocity = state[0], state[1]
    delta_theta = thetas[1] - thetas[0] #distance between theta points
    delta_velocity = angular_velocities[1] - angular_velocities[0] #distance between velocity points
    #assign state to closest point on RoA space
    closest_theta_index = int(round((theta - thetas[0]) / delta_theta))
    closest_velocity_index = int(round((angular_velocity - angular_velocities[0]) / delta_velocity))
    #3x3 block needs a neighbor on every side so nearest point can't be on the edge
    if not (1 <= closest_theta_index <= len(thetas) - 2): # first and last index has no neighbor
        return False
    if not (1 <= closest_velocity_index <= len(angular_velocities) - 2):
        return False
    #all 9 points in the 3x3 block must be in the RoA
    return bool(RoA_space[closest_theta_index - 1:closest_theta_index + 2,
                        closest_velocity_index - 1:closest_velocity_index + 2].all())



