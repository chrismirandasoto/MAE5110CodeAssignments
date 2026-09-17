from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

from models import inverted_pendulum_walker as model

from integrators import rk4

# Fixed controls for this visualization example.
params = {
    "gravity": 9.81,  # m/s^2
    "length": 1.0,  # m
    "mass": 1.0,  # kg
    "incline": 0.06,  # rad
    "angle_of_attack": np.pi / 8,  # rad
    "ankle_torque": 0.0,  # N m
}

def compute_ankle_torque(state, params):
    theta = state[0]
    angular_velocity = state[1]

    mass = params["mass"]
    gravity = params["gravity"]
    length = params["length"]

    position_gain = 4.0
    velocity_gain = 4.0

    torque = (
        -mass * gravity * length * np.sin(theta)
        -mass * length**2
        * (position_gain * theta + velocity_gain * angular_velocity)
    )

    min_torque = -0.1 * mass * gravity * length
    max_torque = 0.05 * mass * gravity * length

    return np.clip(torque, min_torque, max_torque)

def reaches_standing(initial_state, params, timestep=1e-3, sim_time=5.0):
    state = initial_state.copy()
    test_params = params.copy()

    angle_tolerance = 0.005
    velocity_tolerance = 0.005

    n_steps = int(sim_time / timestep)

    for step in range(n_steps):
        test_params["ankle_torque"] = compute_ankle_torque(state, test_params)
        state = rk4(
            model.dynamics,
            step * timestep,
            state,
            timestep,
            test_params,
        )

        # walker has clearly fallen over
        if abs(state[0]) > np.pi / 2:
            return False

        if (
            abs(state[0]) < angle_tolerance
            and abs(state[1]) < velocity_tolerance
        ):
            return True

    return False


def compute_roa(params, grid_size=61):
    angle_values = np.linspace(-0.25, 0.25, grid_size)
    velocity_values = np.linspace(-1.5, 1.5, grid_size)

    roa = np.zeros((grid_size, grid_size), dtype=bool)

    for velocity_index, angular_velocity in enumerate(velocity_values):
        for angle_index, theta in enumerate(angle_values):
            initial_state = np.array([theta, angular_velocity])

            roa[velocity_index, angle_index] = reaches_standing(
                initial_state,
                params,
            )

    return angle_values, velocity_values, roa


initial_state = np.array([0.03, 0.0])

timestep = 1e-4
sim_time = 3.0
desired_number_of_steps = 3

n_timesteps = round(sim_time / timestep) + 1
time_traj = np.arange(n_timesteps) * timestep
state_traj = np.zeros((2, n_timesteps))
state_traj[:, 0] = initial_state
completed_steps = 0

# Simulation loop. Replace this Euler step with your own integrator as needed.
for step, t in enumerate(time_traj[:-1]):
    state = state_traj[:, step]

    params["ankle_torque"] = compute_ankle_torque(state, params)
    next_state = rk4(model.dynamics, t, state, timestep, params)

    if model.event_guard(state, next_state, params):
        next_state = model.event_dynamics(next_state, params)
        completed_steps += 1

    state_traj[:, step + 1] = next_state
    if completed_steps == desired_number_of_steps:
        break

time_traj = time_traj[: step + 2]
state_traj = state_traj[:, : step + 2]

fig, ax = plt.subplots(figsize=(8, 5), layout="constrained")


def draw_frame(index):
    # The massless swing leg is repositioned instantaneously at each impact.
    model.visualize(state_traj[:, index], params, ax=ax)
    ax.set_title(f"t = {time_traj[index]:.2f} s")


# Simulate at a small timestep, but render only 25 frames per second.
fps = 25
frame_stride = round(1 / (fps * timestep))
frame_indices = list(range(0, time_traj.size, frame_stride))
if frame_indices[-1] != time_traj.size - 1:
    frame_indices.append(time_traj.size - 1)

animation = FuncAnimation(
    fig, draw_frame, frames=frame_indices, interval=1000 / fps, repeat=False
)
output = Path("output/assignment_2")
output.mkdir(parents=True, exist_ok=True)
animation.save(output / "walker.gif", writer=PillowWriter(fps=fps))

# To save an MP4 instead, install FFmpeg and use:
# animation.save(output / "walker.mp4", writer="ffmpeg", fps=fps)
print(f"Saved {output / 'walker.gif'} ({completed_steps} footstrikes).")

#roa plots
angle_values, velocity_values, roa = compute_roa(params)

plt.figure(figsize=(7, 5))
plt.imshow(
    roa,
    origin="lower",
    extent=[
        angle_values[0],
        angle_values[-1],
        velocity_values[0],
        velocity_values[-1],
    ],
    aspect="auto",
)
plt.xlabel(r"$\theta$ (rad)")
plt.ylabel(r"$\dot{\theta}$ (rad/s)")
plt.title("Ankle Controller Region of Attraction")
plt.show()
