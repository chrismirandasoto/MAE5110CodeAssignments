import numpy as np


def compute_ankle_torque(state, params):
    theta, omega = state
    m, g, length = params["mass"], params["gravity"], params["length"]
    kp, kd = 4.0, 4.0

    torque = -m * g * length * np.sin(theta)
    torque -= m * length**2 * (kp * theta + kd * omega)

    return np.clip(torque, -0.1 * m * g * length, 0.05 * m * g * length)


def get_bracket(values, value):
    upper = np.searchsorted(values, value)

    if upper < len(values) and np.isclose(values[upper], value):
        return upper, upper

    if upper > 0 and np.isclose(values[upper - 1], value):
        return upper - 1, upper - 1

    return upper - 1, upper


def has_reached_roa(state, angles, velocities, roa):
    theta, omega = state

    if not angles[0] <= theta <= angles[-1]:
        return False

    if not velocities[0] <= omega <= velocities[-1]:
        return False

    a0, a1 = get_bracket(angles, theta)
    v0, v1 = get_bracket(velocities, omega)

    return np.all(roa[v0 : v1 + 1, a0 : a1 + 1])


def nearest_index(values, value):
    return np.argmin(np.abs(values - value))


def select_alpha(omega, velocities, policy):
    return policy[nearest_index(velocities, omega)]


def compute_min_step_policy(
    velocities,
    alphas,
    next_velocity,
    reaches_roa,
    footstrikes,
    section_roa,
):
    cost = np.full(len(velocities), np.inf)
    policy = np.full(len(velocities), np.nan)
    cost[section_roa] = 0.0

    for _ in range(len(velocities)):
        old_cost = cost.copy()
        changed = False

        for i in range(len(velocities)):
            if section_roa[i]:
                continue

            for j, alpha in enumerate(alphas):
                if reaches_roa[i, j]:
                    candidate = float(footstrikes[i, j])
                else:
                    omega_next = next_velocity[i, j]

                    if not np.isfinite(omega_next):
                        continue

                    next_i = nearest_index(velocities, omega_next)

                    if not np.isfinite(old_cost[next_i]):
                        continue

                    candidate = footstrikes[i, j] + old_cost[next_i]

                if candidate < cost[i]:
                    cost[i] = candidate
                    policy[i] = alpha
                    changed = True

        if not changed:
            break

    steps = np.full(len(velocities), -1, dtype=int)
    reachable = np.isfinite(cost)
    steps[reachable] = cost[reachable].astype(int)

    return steps, policy


def compute_max_step_policy(
    velocities,
    alphas,
    next_velocity,
    reaches_roa,
    footstrikes,
    section_roa,
):
    max_steps = np.full(len(velocities), np.nan)
    policy = np.full(len(velocities), np.nan)
    max_steps[section_roa] = 0.0

    visiting = set()

    def solve(i):
        if not np.isnan(max_steps[i]):
            return max_steps[i]

        if i in visiting:
            return np.inf

        visiting.add(i)

        best_steps = -1.0
        best_alpha = np.nan

        for j, alpha in enumerate(alphas):
            if reaches_roa[i, j]:
                candidate = float(footstrikes[i, j])
            else:
                omega_next = next_velocity[i, j]

                if not np.isfinite(omega_next):
                    continue

                next_i = nearest_index(velocities, omega_next)
                downstream = solve(next_i)

                if downstream < 0:
                    continue

                if np.isinf(downstream):
                    candidate = np.inf
                else:
                    candidate = footstrikes[i, j] + downstream

            if candidate > best_steps:
                best_steps = candidate
                best_alpha = alpha

        visiting.remove(i)

        max_steps[i] = best_steps
        policy[i] = best_alpha

        return best_steps

    for i in range(len(velocities)):
        solve(i)

    return max_steps, policy