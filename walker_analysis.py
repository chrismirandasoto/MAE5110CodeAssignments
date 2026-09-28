import numpy as np

from controllers import (
    compute_ankle_torque,
    has_reached_roa,
    nearest_index,
    select_alpha,
)
from integrators import rk4
from models import inverted_pendulum_walker as model


SIM_DT = 1e-3
STAND_TOL = 0.005


def reaches_standing(initial_state, params, dt=SIM_DT, sim_time=5.0):
    state = initial_state.copy()
    test_params = params.copy()

    for step in range(int(sim_time / dt)):
        test_params["ankle_torque"] = compute_ankle_torque(state, test_params)
        state = rk4(model.dynamics, step * dt, state, dt, test_params)

        if abs(state[0]) > np.pi / 2:
            return False

        if abs(state[0]) < STAND_TOL and abs(state[1]) < STAND_TOL:
            return True

    return False


def compute_roa(params, grid_size=61):
    angles = np.linspace(-0.25, 0.25, grid_size)
    velocities = np.linspace(-1.5, 1.5, grid_size)
    roa = np.zeros((grid_size, grid_size), dtype=bool)

    for i, omega in enumerate(velocities):
        for j, theta in enumerate(angles):
            roa[i, j] = reaches_standing(np.array([theta, omega]), params)

    return angles, velocities, roa


def crossed_poincare(state, next_state):
    return state[0] < 0.0 <= next_state[0]


def interpolate_section_velocity(state, next_state):
    fraction = -state[0] / (next_state[0] - state[0])
    return state[1] + fraction * (next_state[1] - state[1])


def simulate_poincare_step(
    initial_velocity,
    alpha,
    params,
    roa_angles,
    roa_velocities,
    roa,
    dt=SIM_DT,
):
    test_params = params.copy()
    test_params["angle_of_attack"] = alpha
    test_params["ankle_torque"] = 0.0

    state = np.array([0.0, initial_velocity])
    impact_occurred = False

    if has_reached_roa(state, roa_angles, roa_velocities, roa):
        return np.nan, 0, True

    for step in range(int(5.0 / dt)):
        next_state = rk4(model.dynamics, step * dt, state, dt, test_params)

        if model.event_guard(state, next_state, test_params):
            if impact_occurred:
                return np.nan, 1, False

            forward_contact = test_params["incline"] + alpha

            if next_state[0] < forward_contact:
                return np.nan, 0, False

            next_state = model.event_dynamics(next_state, test_params)
            impact_occurred = True

        if has_reached_roa(next_state, roa_angles, roa_velocities, roa):
            return np.nan, int(impact_occurred), True

        if impact_occurred and crossed_poincare(state, next_state):
            omega_next = interpolate_section_velocity(state, next_state)
            return omega_next, 1, False

        state = next_state

    return np.nan, 0, False


def build_step_lookup(
    params,
    roa_angles,
    roa_velocities,
    roa,
    state_count,
    action_count=31,
):
    max_velocity = np.sqrt(2 * params["gravity"] / params["length"])

    velocities = np.linspace(0.0, max_velocity, state_count)
    alphas = np.linspace(np.pi / 8, np.pi / 7, action_count)

    next_velocity = np.full((state_count, action_count), np.nan)
    reaches_roa = np.zeros((state_count, action_count), dtype=bool)
    footstrikes = np.zeros((state_count, action_count), dtype=int)

    for i, omega in enumerate(velocities):
        for j, alpha in enumerate(alphas):
            next_velocity[i, j], footstrikes[i, j], reaches_roa[i, j] = (
                simulate_poincare_step(
                    omega,
                    alpha,
                    params,
                    roa_angles,
                    roa_velocities,
                    roa,
                )
            )

    return velocities, alphas, next_velocity, reaches_roa, footstrikes


def find_section_roa(step_velocities, roa_angles, roa_velocities, roa):
    return np.array(
        [
            has_reached_roa(
                np.array([0.0, omega]),
                roa_angles,
                roa_velocities,
                roa,
            )
            for omega in step_velocities
        ]
    )


def simulate_recovery(
    initial_velocity,
    params,
    step_velocities,
    alpha_policy,
    roa_angles,
    roa_velocities,
    roa,
    dt=SIM_DT,
    sim_time=10.0,
):
    test_params = params.copy()

    state = np.array([0.0, initial_velocity])
    history = [state.copy()]

    footstrikes = 0
    balancing = has_reached_roa(state, roa_angles, roa_velocities, roa)

    if not balancing:
        alpha = select_alpha(state[1], step_velocities, alpha_policy)

        if not np.isfinite(alpha):
            return np.array(history).T, footstrikes, False

        test_params["angle_of_attack"] = alpha

    for step in range(int(sim_time / dt)):
        if balancing:
            test_params["ankle_torque"] = compute_ankle_torque(state, test_params)
        else:
            test_params["ankle_torque"] = 0.0

        next_state = rk4(model.dynamics, step * dt, state, dt, test_params)

        if not balancing:
            if model.event_guard(state, next_state, test_params):
                forward_contact = (
                    test_params["incline"] + test_params["angle_of_attack"]
                )

                if next_state[0] < forward_contact:
                    return np.array(history).T, footstrikes, False

                next_state = model.event_dynamics(next_state, test_params)
                footstrikes += 1

            if has_reached_roa(next_state, roa_angles, roa_velocities, roa):
                balancing = True

            elif crossed_poincare(state, next_state):
                omega = interpolate_section_velocity(state, next_state)
                alpha = select_alpha(omega, step_velocities, alpha_policy)

                if not np.isfinite(alpha):
                    return np.array(history).T, footstrikes, False

                test_params["angle_of_attack"] = alpha

        state = next_state
        history.append(state.copy())

        if abs(state[0]) > np.pi / 2:
            return np.array(history).T, footstrikes, False

        if balancing and abs(state[0]) < STAND_TOL and abs(state[1]) < STAND_TOL:
            return np.array(history).T, footstrikes, True

    return np.array(history).T, footstrikes, False


def find_roa_velocity_limit(angles, velocities, roa):
    recoverable = [
        omega
        for omega in velocities[velocities >= 0.0]
        if has_reached_roa(np.array([0.0, omega]), angles, velocities, roa)
    ]

    return recoverable[-1] if recoverable else 0.0


def choose_state_count(params, roa_velocity_limit, error_fraction=0.1):
    max_velocity = np.sqrt(2 * params["gravity"] / params["length"])
    max_error = error_fraction * roa_velocity_limit

    if max_error <= 0:
        raise ValueError("RoA velocity limit must be positive")

    return int(np.ceil(max_velocity / (2 * max_error))) + 1


def print_grid_check(params, roa_velocity_limit, state_count):
    max_velocity = np.sqrt(2 * params["gravity"] / params["length"])
    error_limit = 0.1 * roa_velocity_limit

    print("\nGrid resolution test")
    print(f"maximum allowed nearest-state error: {error_limit:.4f} rad/s")

    for n in [state_count - 2, state_count - 1, state_count, state_count + 1]:
        spacing = max_velocity / (n - 1)
        max_error = spacing / 2.0
        status = "PASS" if max_error <= error_limit else "FAIL"

        print(
            f"{n:3d} states: spacing = {spacing:.4f} rad/s, "
            f"max error = {max_error:.4f} rad/s, {status}"
        )


def find_test_state(
    steps_to_stand,
    step_velocities,
    min_policy,
    params,
    roa_angles,
    roa_velocities,
    roa,
):
    candidates = np.where(steps_to_stand == 3)[0]

    for i in candidates:
        states, footstrikes, success = simulate_recovery(
            step_velocities[i],
            params,
            step_velocities,
            min_policy,
            roa_angles,
            roa_velocities,
            roa,
        )

        if success and footstrikes == 3:
            return i, states, footstrikes

    raise RuntimeError("No 3-step state matched the full simulation")