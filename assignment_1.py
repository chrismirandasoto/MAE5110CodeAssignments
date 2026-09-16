import numpy as np
import matplotlib.pyplot as plt

from models import rimless_wheel as model
from integrators import rk4


params = model.generate_params()

# baseline case just downhill of vertical
initial_state = np.array([0.1, 0.0])

timestep = 1e-3
sim_time = 5.0

n_timesteps = int(sim_time / timestep) + 1
time_traj = np.arange(n_timesteps) * timestep

state_traj = np.zeros((2, n_timesteps))
state_traj[:, 0] = initial_state


def find_contact_state(t, state, timestep, params, contact_direction):
    # find impact inside one timestep
    half_spoke_angle = np.pi / params["num_spokes"]
    contact_angle = params["slope"] + contact_direction * half_spoke_angle

    # use bisection to refine the contact time
    lower_time = 0.0
    upper_time = timestep

    for _ in range(10):
        middle_time = 0.5 * (lower_time + upper_time)
        middle_state = rk4(model.dynamics, t, state, middle_time, params)

        if contact_direction == 1:
            contact_reached = middle_state[0] >= contact_angle
        else:
            contact_reached = middle_state[0] <= contact_angle

        if contact_reached:
            upper_time = middle_time
        else:
            lower_time = middle_time

    contact_time = upper_time
    contact_state = rk4(model.dynamics, t, state, contact_time, params)

    # put angle exactly at contact
    contact_state[0] = contact_angle

    return contact_state, contact_time


def simulate_one_step(initial_angular_velocity, params, timestep):
    # simulate from one impact to the next
    half_spoke_angle = np.pi / params["num_spokes"]

    # start right after impact
    state = np.array([params["slope"] - half_spoke_angle, initial_angular_velocity, ])

    time = 0.0
    max_time = 5.0
    max_steps = int(max_time / timestep)

    for _ in range(max_steps):
        next_state = rk4(model.dynamics, time, state, timestep, params)
        contact_direction = model.detect_contact(next_state, params)

        if contact_direction == 1:
            contact_state, _ = find_contact_state(time, state, timestep, params, contact_direction)
            post_impact_state = model.reset_state(contact_state, params, contact_direction)
            return post_impact_state[1]

        # backward impact means no forward return map point
        if contact_direction == -1:
            return np.nan

        state = next_state
        time += timestep

    return np.nan


def classify_trajectory(initial_state, params, timestep):
    # classify one starting state
    # 0 unresolved  1 rolling  2 upright  3 resting
    if np.all(np.abs(initial_state) < 1e-12):
        return 2

    state = initial_state.copy()
    time = 0.0

    max_time = 15.0
    max_steps = int(max_time / timestep)

    post_impact_velocities = []
    contact_directions = []

    for _ in range(max_steps):
        next_state = rk4(model.dynamics, time, state, timestep, params)
        contact_direction = model.detect_contact(next_state, params)

        if contact_direction != 0:
            contact_state, contact_time = find_contact_state(time, state, timestep, params, contact_direction)
            reset_state = model.reset_state(contact_state, params, contact_direction)

            post_impact_velocities.append(reset_state[1])
            contact_directions.append(contact_direction)

            # check the last few impacts for steady behavior
            if len(post_impact_velocities) >= 4:
                recent_velocities = post_impact_velocities[-4:]
                recent_directions = contact_directions[-4:]

                all_forward = all(direction == 1 for direction in recent_directions)
                velocity_range = (max(recent_velocities) - min(recent_velocities))

                if all_forward and velocity_range < 1e-3:
                    return 1

                directions_alternate = all(recent_directions[index] != recent_directions[index - 1] for index in range(1, len(recent_directions)))
                small_impact_velocity = max(abs(velocity) for velocity in recent_velocities) < 0.05

                if directions_alternate and small_impact_velocity:
                    return 3

            # finish the rest of the timestep after impact
            remaining_time = timestep - contact_time
            next_state = rk4(model.dynamics, time + contact_time, reset_state, remaining_time, params, )

        state = next_state
        time += timestep

    return 0


def estimate_roa_fractions(params, timestep, grid_size=25):
    # estimate rolling and resting regions
    half_spoke_angle = np.pi / params["num_spokes"]

    angle_values = np.linspace(params["slope"] - half_spoke_angle, params["slope"] + half_spoke_angle, grid_size, )
    velocity_values = np.linspace(-4.0, 4.0, grid_size)

    rolling_count = 0
    resting_count = 0
    total_count = grid_size * grid_size

    for initial_velocity in velocity_values:
        for initial_angle in angle_values:
            initial_state = np.array([initial_angle, initial_velocity])
            attractor = classify_trajectory(initial_state, params, timestep)

            if attractor == 1:
                rolling_count += 1
            if attractor == 3:
                resting_count += 1

    rolling_fraction = rolling_count / total_count
    resting_fraction = resting_count / total_count

    return rolling_fraction, resting_fraction


def estimate_fixed_point_and_floquet(params, timestep):
    # find fixed point and floquet multiplier
    test_velocities = np.linspace(0.1, 4.0, 150)
    next_velocities = np.zeros_like(test_velocities)

    for index, velocity in enumerate(test_velocities):
        next_velocities[index] = simulate_one_step(velocity, params, timestep)

    valid_points = ~np.isnan(next_velocities)
    valid_velocities = test_velocities[valid_points]
    valid_next_velocities = next_velocities[valid_points]

    if len(valid_velocities) < 2:
        return np.nan, np.nan

    return_map_error = valid_next_velocities - valid_velocities
    fixed_point_velocity = np.nan

    # look for where return map crosses identity
    for index in range(len(return_map_error) - 1):
        error_1 = return_map_error[index]
        error_2 = return_map_error[index + 1]

        if error_1 * error_2 <= 0:
            velocity_1 = valid_velocities[index]
            velocity_2 = valid_velocities[index + 1]

            fixed_point_velocity = (velocity_1 - error_1 * (velocity_2 - velocity_1) / (error_2 - error_1))
            break

    if np.isnan(fixed_point_velocity):
        return np.nan, np.nan

    # local slope around the fixed point
    velocity_perturbation = 0.01
    lower_velocity = fixed_point_velocity - velocity_perturbation
    upper_velocity = fixed_point_velocity + velocity_perturbation

    lower_next_velocity = simulate_one_step(lower_velocity, params, timestep)
    upper_next_velocity = simulate_one_step(upper_velocity, params, timestep)

    if np.isnan(lower_next_velocity) or np.isnan(upper_next_velocity):
        return fixed_point_velocity, np.nan

    floquet_multiplier = ((upper_next_velocity - lower_next_velocity) / (upper_velocity - lower_velocity))

    return fixed_point_velocity, floquet_multiplier


# baseline trajectory
for step, t in enumerate(time_traj[:-1]):
    current_state = state_traj[:, step]
    next_state = rk4(model.dynamics, t, current_state, timestep, params)

    contact_direction = model.detect_contact(next_state, params)

    if contact_direction != 0:
        contact_state, contact_time = find_contact_state(t, current_state, timestep, params, contact_direction)
        reset_state = model.reset_state(contact_state, params, contact_direction)

        remaining_time = timestep - contact_time
        next_state = rk4(model.dynamics, t + contact_time, reset_state, remaining_time, params, )

    state_traj[:, step + 1] = next_state


plt.figure()
plt.plot(time_traj, state_traj[0])
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.title("Rimless wheel angle")
plt.tight_layout()

plt.figure()
plt.plot(time_traj, state_traj[1])
plt.xlabel("Time (s)")
plt.ylabel("Angular velocity (rad/s)")
plt.title("Rimless wheel angular velocity")
plt.tight_layout()


# region of attraction
half_spoke_angle = np.pi / params["num_spokes"]
minimum_angle = params["slope"] - half_spoke_angle
maximum_angle = params["slope"] + half_spoke_angle

angle_values = np.linspace(minimum_angle, maximum_angle, 41)
velocity_values = np.linspace(-4.0, 4.0, 41)

attractor_map = np.zeros((len(velocity_values), len(angle_values)), dtype=int, )

# checked against 2.5e-3 with same RoA result
# impact timing still refined with bisection
roa_timestep = 5e-3

for velocity_index, initial_velocity in enumerate(velocity_values):
    for angle_index, initial_angle in enumerate(angle_values):
        initial_grid_state = np.array([initial_angle, initial_velocity, ])
        attractor_map[velocity_index, angle_index] = classify_trajectory(initial_grid_state, params, roa_timestep)

angle_grid, velocity_grid = np.meshgrid(angle_values, velocity_values)

plt.figure()

for attractor, label in [(0, "Unresolved"), (1, "Rolling limit cycle"), (2, "Upright equilibrium"), (3, "Resting"), ]:
    points = attractor_map == attractor
    plt.scatter(angle_grid[points], velocity_grid[points], marker="s", s=25, label=label, )

plt.xlabel("Initial angle (rad)")
plt.ylabel("Initial angular velocity (rad/s)")
plt.title("Rimless wheel regions of attraction")
plt.legend()
plt.tight_layout()

rolling_fraction = np.mean(attractor_map == 1)
resting_fraction = np.mean(attractor_map == 3)

print(f"Rolling fraction of sampled state space: {rolling_fraction:.3f}")
print(f"Resting fraction of sampled state space: {resting_fraction:.3f}")


# one step return map
initial_velocities = np.linspace(0.5, 2.5, 100)
next_velocities = np.zeros_like(initial_velocities)

for index, initial_velocity in enumerate(initial_velocities):
    next_velocities[index] = simulate_one_step(initial_velocity, params, timestep)

valid_points = ~np.isnan(next_velocities)
valid_initial_velocities = initial_velocities[valid_points]
valid_next_velocities = next_velocities[valid_points]

return_map_error = (valid_next_velocities - valid_initial_velocities)
fixed_point_velocity = None

for index in range(len(return_map_error) - 1):
    error_1 = return_map_error[index]
    error_2 = return_map_error[index + 1]

    if error_1 * error_2 <= 0:
        velocity_1 = valid_initial_velocities[index]
        velocity_2 = valid_initial_velocities[index + 1]

        fixed_point_velocity = (velocity_1 - error_1 * (velocity_2 - velocity_1) / (error_2 - error_1))
        break

if fixed_point_velocity is not None:
    print(f"Fixed point velocity: {fixed_point_velocity:.4f} rad/s")

    velocity_perturbation = 0.01
    lower_velocity = fixed_point_velocity - velocity_perturbation
    upper_velocity = fixed_point_velocity + velocity_perturbation

    lower_next_velocity = simulate_one_step(lower_velocity, params, timestep)
    upper_next_velocity = simulate_one_step(upper_velocity, params, timestep)

    floquet_multiplier = ((upper_next_velocity - lower_next_velocity) / (upper_velocity - lower_velocity))
    print(f"Floquet multiplier: {floquet_multiplier:.4f}")

plt.figure()
plt.plot(initial_velocities, next_velocities, label="Return map")
plt.plot(initial_velocities, initial_velocities, "--", label="Identity line", )

if fixed_point_velocity is not None:
    plt.plot(fixed_point_velocity, fixed_point_velocity, "o", label="Fixed point", )

plt.xlabel("Current post-impact velocity (rad/s)")
plt.ylabel("Next post-impact velocity (rad/s)")
plt.title("Rimless wheel return map")
plt.legend()
plt.tight_layout()


# slope sweep
slope_degrees = np.linspace(5.0, 25.0, 9)

rolling_fractions = []
resting_fractions = []
fixed_point_velocities = []
floquet_multipliers = []

sweep_roa_timestep = 5e-3
sweep_grid_size = 25

for slope_degrees_current in slope_degrees:
    sweep_params = params.copy()
    sweep_params["slope"] = np.deg2rad(slope_degrees_current)

    rolling_fraction, resting_fraction = estimate_roa_fractions(sweep_params, sweep_roa_timestep, sweep_grid_size, )
    fixed_point_velocity, floquet_multiplier = (estimate_fixed_point_and_floquet(sweep_params, timestep))

    rolling_fractions.append(rolling_fraction)
    resting_fractions.append(resting_fraction)
    fixed_point_velocities.append(fixed_point_velocity)
    floquet_multipliers.append(floquet_multiplier)

    print(f"Slope: {slope_degrees_current:.1f} deg, " f"rolling: {rolling_fraction:.3f}, " f"resting: {resting_fraction:.3f}, " f"fixed point: {fixed_point_velocity:.3f}, " f"Floquet: {floquet_multiplier:.3f}")

plt.figure()
plt.plot(slope_degrees, rolling_fractions, "o-", label="Rolling")
plt.plot(slope_degrees, resting_fractions, "o-", label="Resting")
plt.xlabel("Slope inclination (deg)")
plt.ylabel("Fraction of sampled state space")
plt.title("Effect of slope on regions of attraction")
plt.legend()
plt.tight_layout()

plt.figure()
plt.plot(slope_degrees, floquet_multipliers, "o-")
plt.xlabel("Slope inclination (deg)")
plt.ylabel("Floquet multiplier")
plt.title("Effect of slope on rolling stability")
plt.ylim(0.0, 0.5)
plt.tight_layout()


# spoke count sweep
spoke_counts = np.arange(6, 13)

spoke_rolling_fractions = []
spoke_resting_fractions = []
spoke_fixed_point_velocities = []
spoke_floquet_multipliers = []

for spoke_count in spoke_counts:
    sweep_params = params.copy()
    sweep_params["num_spokes"] = int(spoke_count)

    rolling_fraction, resting_fraction = estimate_roa_fractions(sweep_params, sweep_roa_timestep, sweep_grid_size, )
    fixed_point_velocity, floquet_multiplier = (estimate_fixed_point_and_floquet(sweep_params, timestep))

    spoke_rolling_fractions.append(rolling_fraction)
    spoke_resting_fractions.append(resting_fraction)
    spoke_fixed_point_velocities.append(fixed_point_velocity)
    spoke_floquet_multipliers.append(floquet_multiplier)

    print(f"Spokes: {spoke_count}, " f"rolling: {rolling_fraction:.3f}, " f"resting: {resting_fraction:.3f}, " f"fixed point: {fixed_point_velocity:.3f}, " f"Floquet: {floquet_multiplier:.3f}")

plt.figure()
plt.plot(spoke_counts, spoke_rolling_fractions, "o-", label="Rolling", )
plt.plot(spoke_counts, spoke_resting_fractions, "o-", label="Resting", )
plt.xlabel("Number of spokes")
plt.ylabel("Fraction of sampled state space")
plt.title("Effect of spoke count on regions of attraction")
plt.xticks(spoke_counts)
plt.legend()
plt.tight_layout()

plt.figure()
plt.plot(spoke_counts, spoke_floquet_multipliers, "o-")
plt.xlabel("Number of spokes")
plt.ylabel("Floquet multiplier")
plt.title("Effect of spoke count on rolling stability")
plt.xticks(spoke_counts)
plt.ylim(0.0, 1.0)
plt.tight_layout()

plt.show()
