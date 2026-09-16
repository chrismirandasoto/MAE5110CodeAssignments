# Inverted pendulum walker control input code
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch


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

#Function to create RoA
def create_RoA(params, resolution = 40):
    #grid of starting states from max back wall to max front wall
    theta_back = params["incline"] - np.pi/7
    theta_front = params["incline"] + np.pi/7
    thetas = np.linspace(theta_back, theta_front, resolution)
    angular_velocities = np.linspace(-1.5, 1.5, resolution)
    #try every starting state and record true or false
    RoA_space = np.zeros((len(thetas), len(angular_velocities)), dtype=bool)
    for i, th0 in enumerate(thetas):
        for j, thdot0 in enumerate(angular_velocities):
            RoA_space[i, j] = simulate_if_stabilized(np.array([th0, thdot0]), params)

    return thetas, angular_velocities, RoA_space.T #transpose matrix so Theta = x, Angular Vel = y

#test plot code for RoA
params = model.generate_params()
state = [0,1]
thetas, angular_velocities, RoA_space = create_RoA(params)
plt.pcolormesh(thetas, angular_velocities, RoA_space, shading="nearest", cmap="Blues")
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

# #simulate the walker and turn on control once in RoA
# def simulate_walker_with_control(state, params, time_step=0.005, sim_time=5.0):
#     theta0, angular_velocity0 = state[0], state[1]
#     state_traj = [np.array([theta0, angular_velocity0])]
#     time_traj = [0]
#     thetas, angular_velocities, RoA_space = create_RoA(params)
#     for step in range(int(sim_time / time_step)):
#         if check_if_in_RoA(state, RoA_space, thetas, angular_velocities):
#             params["ankle_torque"] = calculate_torque(state, params)
#         else:
#             params["ankle_torque"] = 0
#         new_state = rk4_step(state, params, time_step)
#         if model.event_guard(state, new_state, params):
#             new_state = model.event_dynamics(state, params)
#         state_traj.append(new_state)
#         time_traj.append(step * time_step)
#         state = new_state

#     return time_traj, state_traj

#simulate the walker and turn on control once in RoA
def simulate_walker_with_control(state, params, RoA_space, thetas, angular_velocities,
                                time_step=0.005, sim_time=20.0, tol=1e-3):
    state = np.array(state, dtype=float)
    state_traj = [state.copy()]
    time_traj = [0.0]
    controller_on = False
    reset_angle = params["incline"] - params["angle_of_attack"] #back wall of this stance
    outcome = "timeout"
    switch_states = [] #state right before each spoke switch
    switch_times = [] #time of each spoke switch

    for step in range(int(sim_time / time_step)):
        #check if in RoA unless already in and controller is on
        if not controller_on and check_if_in_RoA(state, RoA_space, thetas, angular_velocities):
            controller_on = True
        #recalculate torque per step
        if controller_on:
            params["ankle_torque"] = calculate_torque(state, params)
        else:
            params["ankle_torque"] = 0.0
        new_state = rk4_step(state, params, time_step) #new state with rk
        #can only stall if controller not on and not in RoA
        if not controller_on:
            if model.event_guard(state, new_state, params): #switches spoke forward
                switch_states.append(new_state.copy()) #record switches
                switch_times.append((step + 1) * time_step)
                new_state = model.event_dynamics(new_state, params)
                reset_angle = params["incline"] - params["angle_of_attack"]
            elif new_state[0] < reset_angle: #rolled back past the stance foot
                outcome = "fell back"
                break
        elif np.abs(new_state[0]) < tol and np.abs(new_state[1]) < tol: #settled upright
            outcome = "standing"
            state_traj.append(new_state.copy())
            time_traj.append((step + 1) * time_step)
            break
        #add new state and time to lists
        state = new_state
        state_traj.append(state.copy())
        time_traj.append((step + 1) * time_step)

    params["ankle_torque"] = 0.0 #reset for the next run
    return np.array(time_traj), np.array(state_traj), outcome, np.array(switch_times), np.array(switch_states)

#test simulate walker code
params = model.generate_params()
thetas, angular_velocities, RoA_space = create_RoA(params) #build once

start_state = [0.0, 2.0]
time_traj, state_traj, outcome, switch_times, switch_states = simulate_walker_with_control(
    start_state, params, RoA_space, thetas, angular_velocities)
print(outcome)
print(f"switched spokes {len(switch_times)} times")

fig, ax = plt.subplots(figsize=(9, 6))

#RoA background (drawn under everything else)
ax.pcolormesh(thetas, angular_velocities, RoA_space.T, shading="nearest",
            cmap="Blues", vmin=0, vmax=2, zorder=0) #vmax=2 keeps the band a light blue

#stance walls for the alpha used in this run
theta_reset = params["incline"] - params["angle_of_attack"]
theta_strike = params["incline"] + params["angle_of_attack"]
ax.axvline(theta_reset, color="tab:blue", ls="--", lw=1, label=r"reset wall $\gamma-\alpha$")
ax.axvline(theta_strike, color="tab:red", ls="--", lw=1, label=r"heelstrike wall $\gamma+\alpha$")

#trajectory: split at each impact so the jump isn't drawn as a normal path
jump_indices = np.where(np.abs(np.diff(state_traj[:, 0])) > 0.2)[0] #theta jumps back at each impact
for k, segment in enumerate(np.split(state_traj, jump_indices + 1)):
    ax.plot(segment[:, 0], segment[:, 1], lw=1.4, color="dimgray", zorder=2,
            label="stance" if k == 0 else None)
#impact jumps: from the heelstrike state to the reset state, dotted
for k, idx in enumerate(jump_indices):
    after = state_traj[idx + 1]
    ax.plot([switch_states[k, 0], after[0]], [switch_states[k, 1], after[1]],
            ls=":", lw=1, color="purple", zorder=2, label="impact jump" if k == 0 else None)

#start point: first row of the trajectory
ax.plot(*state_traj[0], "o", color="black", markersize=6, label="start", zorder=3)

#spoke switches: mark each heelstrike and number it
if len(switch_states) > 0:
    ax.plot(switch_states[:, 0], switch_states[:, 1], "^", color="purple",
            markersize=8, label="spoke switch", zorder=3)
    for n, (theta_hit, velocity_hit) in enumerate(switch_states, start=1):
        ax.annotate(str(n), (theta_hit, velocity_hit), textcoords="offset points",
                    xytext=(-12, 4), color="purple", fontsize=10)

#end point: last row, marker depends on the outcome
end_markers = {
    "standing": dict(marker="o", color="green", markersize=9, label="stabilized"),
    "fell back": dict(marker="X", color="red", markersize=11, label="failed"),
    "timeout": dict(marker="s", color="orange", markersize=8, label="timeout"),
}
ax.plot(*state_traj[-1], linestyle="none", zorder=4, **end_markers[outcome])

#axes, grid, labels
ax.axhline(0, color="black", lw=0.6)
ax.axvline(0, color="black", lw=0.6)
ax.grid(True, which="major", color="gray", alpha=0.35, lw=0.6)
ax.minorticks_on()
ax.grid(True, which="minor", color="gray", alpha=0.15, lw=0.4)
ax.set_axisbelow(False) #draw the grid on top of the RoA shading
ax.set_xlim(thetas[0], thetas[-1])
ax.set_xlabel(r"$\theta$ [rad]")
ax.set_ylabel(r"$\dot{\theta}$ [rad/s]")
ax.set_title(rf"Walker from $(\theta_0, \dot\theta_0) = ({start_state[0]:g}, {start_state[1]:g})$: "
            f"{outcome} after {len(switch_times)} spoke switches")
handles, labels = ax.get_legend_handles_labels()
handles.append(Patch(color=plt.cm.Blues(0.5))) #same blue as the band (True = 1 on a 0 to 2 scale)
labels.append("region of attraction")
ax.legend(handles, labels, loc="center left", bbox_to_anchor=(1.02, 0.5), frameon=False)
fig.tight_layout()
plt.show()
