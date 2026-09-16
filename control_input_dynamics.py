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
theta_back = params["incline"] - params["angle_of_attack"]
theta_front = params["incline"] + params["angle_of_attack"]
thetas = np.linspace(theta_back, theta_front, 40)
angular_velocities = np.linspace(-1.5, 1.5, 40)
#try every starting state and record true or false
roa = np.zeros((len(angular_velocities), len(thetas)), dtype=bool)
for i, w0 in enumerate(angular_velocities):
    for j, th0 in enumerate(thetas):
        roa[i, j] = simulate_if_stabilized(np.array([th0, w0]), params)
params["ankle_torque"] = 0.0  #reset so the walker sim starts unpowered
#plot
plt.pcolormesh(thetas, angular_velocities, roa, shading="nearest", cmap="Greens")
plt.xlabel(r"$\theta$ [rad]")
plt.ylabel(r"$\dot\theta$ [rad/s]")
plt.title("Region of attraction of the ankle controller")
plt.show()



