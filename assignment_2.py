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

def simulate_poincare_step(initial_velocity, alpha, params, timestep=1e-3):
    test_params = params.copy()
    test_params["angle_of_attack"] = alpha
    test_params["ankle_torque"] = 0.0

    state = np.array([0.0, initial_velocity])
    time = 0.0
    impact_occurred = False

    max_time = 5.0
    max_steps = int(max_time / timestep)

    for _ in range(max_steps):
        next_state = rk4(
            model.dynamics,
            time,
            state,
            timestep,
            test_params,
        )

        if not impact_occurred and model.event_guard(
            state,
            next_state,
            test_params,
        ):
            forward_contact = (
                test_params["incline"]
                + test_params["angle_of_attack"]
            )

            # if it hits the backward guard, there is no forward return
            if next_state[0] < forward_contact:
                return np.nan

            next_state = model.event_dynamics(
                next_state,
                test_params,
            )
            impact_occurred = True

        elif impact_occurred:
            # next crossing of the theta = 0 Poincare section
            if state[0] < 0.0 and next_state[0] >= 0.0:
                fraction = -state[0] / (next_state[0] - state[0])

                return (
                    state[1]
                    + fraction * (next_state[1] - state[1])
                )

            # reversed direction before getting back to the section
            if model.event_guard(state, next_state, test_params):
                return np.nan

        state = next_state
        time += timestep

    return np.nan

def build_step_lookup(params, state_count=61, action_count=31):
    max_velocity = np.sqrt(2.0 * params["gravity"] / params["length"])

    velocity_values = np.linspace(0.0, max_velocity, state_count)
    alpha_values = np.linspace(np.pi / 8, np.pi / 7, action_count)

    next_velocity_table = np.full(
        (state_count, action_count),
        np.nan,
    )

    for state_index, angular_velocity in enumerate(velocity_values):
        for action_index, alpha in enumerate(alpha_values):
            next_velocity_table[state_index, action_index] = (
                simulate_poincare_step(
                    angular_velocity,
                    alpha,
                    params,
                )
            )

    return velocity_values, alpha_values, next_velocity_table


def find_section_roa(velocity_values, params):
    in_roa = np.zeros(len(velocity_values), dtype=bool)

    for index, angular_velocity in enumerate(velocity_values):
        state = np.array([0.0, angular_velocity])
        in_roa[index] = reaches_standing(state, params)

    return in_roa


def compute_steps_to_stand(
    velocity_values,
    alpha_values,
    next_velocity_table,
    section_roa,
    params,
    max_steps=20,
):
    steps_to_stand = np.full(len(velocity_values), -1, dtype=int)
    best_alpha = np.full(len(velocity_values), np.nan)

    # already inside ankle-controller RoA
    steps_to_stand[section_roa] = 0

    # first find states that land directly in the RoA
    for state_index in range(len(velocity_values)):
        if steps_to_stand[state_index] == 0:
            continue

        for action_index, alpha in enumerate(alpha_values):
            next_velocity = next_velocity_table[state_index, action_index]

            if not np.isfinite(next_velocity):
                continue

            next_state = np.array([0.0, next_velocity])

            if reaches_standing(next_state, params):
                steps_to_stand[state_index] = 1
                best_alpha[state_index] = alpha
                break

    # then work backward through the lookup table
    for step_count in range(2, max_steps + 1):
        found_new_state = False

        for state_index in range(len(velocity_values)):
            if steps_to_stand[state_index] >= 0:
                continue

            for action_index, alpha in enumerate(alpha_values):
                next_velocity = next_velocity_table[state_index, action_index]

                if not np.isfinite(next_velocity):
                    continue

                nearest_index = np.argmin(
                    np.abs(velocity_values - next_velocity)
                )

                if (
                    steps_to_stand[nearest_index] >= 0
                    and steps_to_stand[nearest_index] < step_count
                ):
                    steps_to_stand[state_index] = step_count
                    best_alpha[state_index] = alpha
                    found_new_state = True
                    break

        if not found_new_state:
            break

    return steps_to_stand, best_alpha

def compute_max_steps_to_stand(
    velocity_values,
    alpha_values,
    next_velocity_table,
    section_roa,
    params,
):
    max_steps = np.full(len(velocity_values), np.nan)
    max_alpha = np.full(len(velocity_values), np.nan)

    # already in the ankle RoA
    max_steps[section_roa] = 0.0

    visiting = set()

    def solve_state(state_index):
        if not np.isnan(max_steps[state_index]):
            return max_steps[state_index]

        # if we find a cycle, walking can potentially continue indefinitely
        if state_index in visiting:
            return np.inf

        visiting.add(state_index)

        best_steps = -1.0
        best_action = np.nan

        for action_index, alpha in enumerate(alpha_values):
            next_velocity = next_velocity_table[state_index, action_index]

            if not np.isfinite(next_velocity):
                continue

            next_state = np.array([0.0, next_velocity])

            # this step lands directly in the ankle-controller RoA
            if reaches_standing(next_state, params):
                candidate_steps = 1.0

            else:
                nearest_index = np.argmin(
                    np.abs(velocity_values - next_velocity)
                )

                downstream_steps = solve_state(nearest_index)

                if downstream_steps < 0:
                    continue

                if np.isinf(downstream_steps):
                    candidate_steps = np.inf
                else:
                    candidate_steps = 1.0 + downstream_steps

            if candidate_steps > best_steps:
                best_steps = candidate_steps
                best_action = alpha

        visiting.remove(state_index)

        max_steps[state_index] = best_steps
        max_alpha[state_index] = best_action

        return best_steps

    for state_index in range(len(velocity_values)):
        solve_state(state_index)

    return max_steps, max_alpha


def is_state_in_roa(state, angle_values, velocity_values, roa):
    theta = state[0]
    angular_velocity = state[1]

    if (
        theta < angle_values[0]
        or theta > angle_values[-1]
        or angular_velocity < velocity_values[0]
        or angular_velocity > velocity_values[-1]
    ):
        return False

    angle_index = np.argmin(np.abs(angle_values - theta))
    velocity_index = np.argmin(
        np.abs(velocity_values - angular_velocity)
    )

    return roa[velocity_index, angle_index]


def select_step_alpha(angular_velocity, velocity_values, best_alpha):
    state_index = np.argmin(
        np.abs(velocity_values - angular_velocity)
    )

    return best_alpha[state_index]


def simulate_controlled_recovery(
    initial_velocity,
    params,
    step_velocities,
    alpha_policy,
    roa_angles,
    roa_velocities,
    roa,
    timestep=1e-3,
    sim_time=10.0,
):
    test_params = params.copy()

    state = np.array([0.0, initial_velocity])
    time = 0.0

    time_history = [time]
    state_history = [state.copy()]

    footstrikes = 0
    balancing = is_state_in_roa(
        state,
        roa_angles,
        roa_velocities,
        roa,
    )

    if not balancing:
        alpha = select_step_alpha(
            state[1],
            step_velocities,
            alpha_policy,
        )

        if not np.isfinite(alpha):
            return (
                np.array(time_history),
                np.array(state_history).T,
                footstrikes,
                False,
            )

        test_params["angle_of_attack"] = alpha

    max_steps = int(sim_time / timestep)

    for _ in range(max_steps):
        if balancing:
            test_params["ankle_torque"] = compute_ankle_torque(
                state,
                test_params,
            )
        else:
            test_params["ankle_torque"] = 0.0

        next_state = rk4(
            model.dynamics,
            time,
            state,
            timestep,
            test_params,
        )

        if not balancing:
            # enter standing controller as soon as we reach its RoA
            if is_state_in_roa(
                next_state,
                roa_angles,
                roa_velocities,
                roa,
            ):
                balancing = True

            elif model.event_guard(state, next_state, test_params):
                next_state = model.event_dynamics(
                    next_state,
                    test_params,
                )
                footstrikes += 1

            # choose a new alpha at the next Poincare crossing
            if (
                not balancing
                and state[0] < 0.0
                and next_state[0] >= 0.0
            ):
                alpha = select_step_alpha(
                    next_state[1],
                    step_velocities,
                    alpha_policy,
                )

                if not np.isfinite(alpha):
                    break

                test_params["angle_of_attack"] = alpha

        state = next_state
        time += timestep

        time_history.append(time)
        state_history.append(state.copy())

        if (
            balancing
            and abs(state[0]) < 0.005
            and abs(state[1]) < 0.005
        ):
            return (
                np.array(time_history),
                np.array(state_history).T,
                footstrikes,
                True,
            )

    return (
        np.array(time_history),
        np.array(state_history).T,
        footstrikes,
        False,
    )

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

#poincare test state-action lookup table
step_velocities, step_alphas, next_velocity_table = build_step_lookup(params)

valid_entries = np.count_nonzero(np.isfinite(next_velocity_table))
total_entries = next_velocity_table.size

print("\nStep lookup table")
print(f"velocity states: {len(step_velocities)}")
print(f"alpha actions: {len(step_alphas)}")
print(f"valid transitions: {valid_entries}/{total_entries}")

plt.figure(figsize=(7, 5))
plt.imshow(
    next_velocity_table,
    origin="lower",
    extent=[
        np.rad2deg(step_alphas[0]),
        np.rad2deg(step_alphas[-1]),
        step_velocities[0],
        step_velocities[-1],
    ],
    aspect="auto",
)
plt.xlabel(r"$\alpha$ (deg)")
plt.ylabel(r"$\dot{\theta}_k$ (rad/s)")
plt.title(r"Next Poincaré Velocity $\dot{\theta}_{k+1}$")
plt.colorbar(label=r"$\dot{\theta}_{k+1}$ (rad/s)")

section_roa = find_section_roa(step_velocities, params)

roa_velocities = step_velocities[section_roa]

print("\nRoA on Poincare section")

if len(roa_velocities) > 0:
    print(
        f"recoverable velocity range at theta = 0: "
        f"{roa_velocities[0]:.3f} to {roa_velocities[-1]:.3f} rad/s"
    )
else:
    print("no grid states on the section are inside the RoA")


steps_to_stand, best_alpha = compute_steps_to_stand(
    step_velocities,
    step_alphas,
    next_velocity_table,
    section_roa,
    params,
)

max_steps_to_stand, max_alpha = compute_max_steps_to_stand(
    step_velocities,
    step_alphas,
    next_velocity_table,
    section_roa,
    params,
)

print("\nSteps to stand")

for step_count in range(np.max(steps_to_stand) + 1):
    count = np.count_nonzero(steps_to_stand == step_count)
    print(f"{step_count} steps: {count} states")

unreachable_count = np.count_nonzero(steps_to_stand < 0)
print(f"unreachable: {unreachable_count} states")


plt.figure(figsize=(7, 5))
plt.step(
    step_velocities,
    steps_to_stand,
    where="mid",
)
plt.xlabel(r"Initial $\dot{\theta}$ (rad/s)")
plt.ylabel("Steps to stand")
plt.title("Steps Required to Reach Standing")
plt.grid()

three_step_indices = np.where(steps_to_stand == 3)[0]

test_index = three_step_indices[len(three_step_indices) // 2]
recovery_velocity = step_velocities[test_index]

(
    recovery_time,
    recovery_states,
    recovery_footstrikes,
    recovery_success,
) = simulate_controlled_recovery(
    recovery_velocity,
    params,
    step_velocities,
    best_alpha,
    angle_values,
    velocity_values,
    roa,
)

print("\nControlled recovery")
print(f"initial velocity: {recovery_velocity:.3f} rad/s")
print(f"predicted steps: {steps_to_stand[test_index]}")
print(f"actual footstrikes: {recovery_footstrikes}")
print(f"reached standing: {recovery_success}")

plt.figure(figsize=(7, 5))
plt.plot(
    recovery_states[0],
    recovery_states[1],
)
plt.scatter([0.0], [0.0], marker="x")
plt.xlabel(r"$\theta$ (rad)")
plt.ylabel(r"$\dot{\theta}$ (rad/s)")
plt.title(
    f"Controlled Recovery from "
    f"$\\dot{{\\theta}}_0={recovery_velocity:.2f}$ rad/s"
)
plt.grid()

maximum_steps = max_steps_to_stand[test_index]

print("\nMaximum walking recovery")

if np.isinf(maximum_steps):
    print("walker can remain outside the RoA indefinitely")
else:
    print(f"maximum predicted steps: {int(maximum_steps)}")

(
    max_recovery_time,
    max_recovery_states,
    max_recovery_footstrikes,
    max_recovery_success,
) = simulate_controlled_recovery(
    recovery_velocity,
    params,
    step_velocities,
    max_alpha,
    angle_values,
    velocity_values,
    roa,
)

print(f"actual maximum footstrikes: {max_recovery_footstrikes}")
print(f"eventually reached standing: {max_recovery_success}")

plt.figure(figsize=(7, 5))
plt.plot(
    max_recovery_states[0],
    max_recovery_states[1],
)
plt.scatter([0.0], [0.0], marker="x")
plt.xlabel(r"$\theta$ (rad)")
plt.ylabel(r"$\dot{\theta}$ (rad/s)")
plt.title(
    f"Maximum-Step Recovery from "
    f"$\\dot{{\\theta}}_0={recovery_velocity:.2f}$ rad/s"
)
plt.grid()

plt.show()
