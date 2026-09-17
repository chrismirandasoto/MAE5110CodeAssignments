# Inverted pendulum walker control input code
import time

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

def calculate_torque(state, params):
    mass, gravity, length = params["mass"], params["gravity"], params["length"]
    theta = state[0]
    angular_velocity = state[1]
    torque_min = -0.1 * mass * gravity * length
    torque_max = 0.05 * mass * gravity * length
    Kp, Kd = 28.48 , 10.67
    desired_accel = -Kp * theta - Kd * angular_velocity
    ankle_torque = -mass * gravity * length * np.sin(theta) + mass * length**2 * desired_accel
    return np.clip(ankle_torque, torque_min, torque_max)

def simulate_if_stabilized(state, params, time_step=0.005, sim_time=5.0, tol=1e-3):
    for step in range(int(sim_time / time_step)):
        params["ankle_torque"] = calculate_torque(state, params)
        state = rk4_step(state, params, time_step)
        theta, angular_velocity = state[0], state[1]
        if theta > params["incline"] + np.pi / 7 or theta < params["incline"] - np.pi / 7:
            return False
        if np.abs(theta) < tol and np.abs(angular_velocity) < tol:
            return True
    return False

def create_RoA(params, resolution = 80):
    theta_back = params["incline"] - np.pi/7
    theta_front = params["incline"] + np.pi/7
    thetas = np.linspace(theta_back, theta_front, resolution)
    angular_velocities = np.linspace(-1.5, 1.5, resolution)
    RoA_space = np.zeros((len(thetas), len(angular_velocities)), dtype=bool)
    for i, th0 in enumerate(thetas):
        for j, thdot0 in enumerate(angular_velocities):
            RoA_space[i, j] = simulate_if_stabilized(np.array([th0, thdot0]), params)
    params["ankle_torque"] = 0
    return thetas, angular_velocities, RoA_space #RoA_space[theta_index, velocity_index]

def check_if_in_RoA(state, RoA_space, thetas, angular_velocities):
    theta, angular_velocity = state[0], state[1]
    delta_theta = thetas[1] - thetas[0]
    delta_velocity = angular_velocities[1] - angular_velocities[0]
    closest_theta_index = int(round((theta - thetas[0]) / delta_theta))
    closest_velocity_index = int(round((angular_velocity - angular_velocities[0]) / delta_velocity))
    if not (1 <= closest_theta_index <= len(thetas) - 2):
        return False
    if not (1 <= closest_velocity_index <= len(angular_velocities) - 2):
        return False
    return bool(RoA_space[closest_theta_index - 1:closest_theta_index + 2,
                        closest_velocity_index - 1:closest_velocity_index + 2].all())

#look up the policy's alpha for a section speed
def choose_alpha(angular_velocity, velocity_values, best_alpha, current_alpha):
    delta = velocity_values[1] - velocity_values[0]
    row = int(round((angular_velocity - velocity_values[0]) / delta))
    if 0 <= row < len(velocity_values) and not np.isnan(best_alpha[row]):
        return best_alpha[row]
    return current_alpha #no table entry: keep the current alpha

def simulate_walker_with_control(state, params, RoA_space, thetas, angular_velocities,
        velocity_values=None, best_alpha=None, time_step=0.005, sim_time=20.0, tol=1e-3):
    params = dict(params) #copy: alpha and torque change during the run
    state = np.array(state, dtype=float)
    state_traj = [state.copy()]
    time_traj = [0.0]
    controller_on = False
    outcome = "timeout"
    switch_states = []
    switch_times = []
    alphas_used = [] #alpha of each step
    use_policy = best_alpha is not None
    #starting on the section: pick the first alpha before walking
    if use_policy and state[0] == 0.0 and state[1] > 0:
        params["angle_of_attack"] = choose_alpha(state[1], velocity_values, best_alpha, params["angle_of_attack"])
    reset_angle = params["incline"] - params["angle_of_attack"]

    for step in range(int(sim_time / time_step)):
        if not controller_on and check_if_in_RoA(state, RoA_space, thetas, angular_velocities):
            controller_on = True
        if controller_on:
            params["ankle_torque"] = calculate_torque(state, params)
        else:
            params["ankle_torque"] = 0.0
        new_state = rk4_step(state, params, time_step)
        if not controller_on:
            if model.event_guard(state, new_state, params):
                switch_states.append(new_state.copy())
                switch_times.append((step + 1) * time_step)
                alphas_used.append(params["angle_of_attack"])
                new_state = model.event_dynamics(new_state, params)
                reset_angle = params["incline"] - params["angle_of_attack"]
            elif new_state[0] < reset_angle:
                outcome = "fell back"
                break
            elif use_policy and state[0] < 0 <= new_state[0] and new_state[1] > 0:
                params["angle_of_attack"] = choose_alpha(new_state[1], velocity_values, best_alpha, params["angle_of_attack"])
        elif np.abs(new_state[0]) < tol and np.abs(new_state[1]) < tol:
            outcome = "standing"
            state_traj.append(new_state.copy())
            time_traj.append((step + 1) * time_step)
            break
        state = new_state
        state_traj.append(state.copy())
        time_traj.append((step + 1) * time_step)

    return (np.array(time_traj), np.array(state_traj), outcome,
            np.array(switch_times), np.array(switch_states), np.array(alphas_used))

def find_next_poincare_crossing(thetadot_k, alpha, params, RoA_space, thetas, angular_velocities,
            time_step=0.005, sim_time=5.0):
    params = dict(params)
    params["angle_of_attack"] = alpha
    params["ankle_torque"] = 0.0
    state = np.array([0.0, thetadot_k])
    reset_angle = params["incline"] - alpha
    switch_happened = False
    for step in range(int(sim_time / time_step)):
        new_state = rk4_step(state, params, time_step)
        if check_if_in_RoA(new_state, RoA_space, thetas, angular_velocities):
            return "standing", np.nan
        if model.event_guard(state, new_state, params):
            new_state = model.event_dynamics(new_state, params)
            switch_happened = True
        elif new_state[0] < reset_angle:
            return "fell back", np.nan
        elif switch_happened and state[0] < 0 <= new_state[0]:
            return "next step", new_state[1]
        state = new_state
    return "timeout", np.nan

def create_lookup_table(params, RoA_space, thetas, angular_velocities, n_velocities=90, n_alphas=15):
    gravity, length = params["gravity"], params["length"]
    froude_angular_velocity = np.sqrt(2 * gravity / length)
    alpha_values = np.linspace(np.pi / 8, np.pi / 7, n_alphas)
    angular_velocity_values = np.linspace(0, froude_angular_velocity, n_velocities)
    status_table = np.empty((len(angular_velocity_values), len(alpha_values)), dtype=object)
    next_velocity_table = np.full((len(angular_velocity_values), len(alpha_values)), np.nan)
    for j, alpha in enumerate(alpha_values):
        for i, angular_velocity in enumerate(angular_velocity_values):
            status, thetadot_k1 = find_next_poincare_crossing(angular_velocity, alpha, params,
                RoA_space, thetas, angular_velocities, time_step=0.005, sim_time=5.0)
            status_table[i, j] = status
            next_velocity_table[i, j] = thetadot_k1
    return angular_velocity_values, alpha_values, status_table, next_velocity_table

#back out how many steps each speed needs to reach the RoA, and which alpha to use
def backward_induction(velocity_values, alpha_values, status_table, next_velocity_table,
        RoA_space, thetas, angular_velocities):
    n = len(velocity_values)
    steps_to_stand = np.full(n, np.inf) #inf = no known way to stand yet
    best_alpha = np.full(n, np.nan)
    delta_velocity = velocity_values[1] - velocity_values[0]

    def neighbors(i): #row i and the rows right next to it
        return range(max(i - 1, 0), min(i + 2, n))

    def steps_if_chosen(r, j):
        """total steps if the walker is on row r and uses alpha j (with what is solved so far)"""
        if status_table[r, j] == "standing":
            return 1
        if status_table[r, j] != "next step":
            return np.inf #fell back or timed out
        landing = int(round((next_velocity_table[r, j] - velocity_values[0]) / delta_velocity))
        if not 1 <= landing <= n - 2:
            return np.inf #landed off the table
        return 1 + steps_to_stand[landing - 1:landing + 2].max() #worst of landing row and its neighbors

    #0 steps: this row and its neighbors are already in the RoA on the section
    in_roa = [check_if_in_RoA([0.0, w], RoA_space, thetas, angular_velocities) for w in velocity_values]
    for i in range(n):
        if all(in_roa[r] for r in neighbors(i)):
            steps_to_stand[i] = 0

    #1 step, 2 steps, 3 steps, ...
    k = 0
    while True:
        k += 1
        newly_solved = []
        for i in range(n):
            if steps_to_stand[i] != np.inf:
                continue
            #alphas that reach standing in k steps from this row AND its neighbors
            working = [j for j in range(len(alpha_values))
                    if max(steps_if_chosen(r, j) for r in neighbors(i)) <= k]
            if working:
                newly_solved.append((i, alpha_values[working[len(working) // 2]])) #middle choice
        if not newly_solved and k > 1:
            break
        for i, alpha in newly_solved: #assign after the sweep so a round can't build on itself
            steps_to_stand[i] = k
            best_alpha[i] = alpha

    return steps_to_stand, best_alpha


#one plot: colored regions of initial speeds by how many steps they need
def plot_steps_to_standstill(velocity_values, steps_to_stand, best_alpha):
    finite = steps_to_stand[np.isfinite(steps_to_stand)]
    max_steps = int(finite.max())
    colors = plt.cm.viridis(np.linspace(0.0, 0.9, max_steps + 1)) #one color per step count
    delta = velocity_values[1] - velocity_values[0]

    fig, ax = plt.subplots(figsize=(10, 5))

    #colored band for every grid speed (each band is one row wide)
    for w, n_steps in zip(velocity_values, steps_to_stand):
        color = "lightgray" if np.isinf(n_steps) else colors[int(n_steps)]
        ax.axvspan(w - delta / 2, w + delta / 2, color=color, alpha=0.35, lw=0)

    #the alpha the policy uses at each speed, same colors
    for n_steps in range(max_steps + 1):
        rows = steps_to_stand == n_steps
        ax.plot(velocity_values[rows], best_alpha[rows], "o", color=colors[n_steps],
                markersize=5, markeredgecolor="black", markeredgewidth=0.4)

    #label each region with its step count and speed range
    for n_steps in range(max_steps + 1):
        speeds = velocity_values[steps_to_stand == n_steps]
        if len(speeds) == 0:
            continue
        ax.text((speeds.min() + speeds.max()) / 2, np.pi / 7 + 0.004,
                f"{n_steps} step{'s' if n_steps != 1 else ''}\n{speeds.min():.2f}–{speeds.max():.2f}",
                ha="center", va="bottom", fontsize=9)

    handles = [Patch(color=colors[n], alpha=0.6, label=f"{n} steps") for n in range(max_steps + 1)]
    if np.isinf(steps_to_stand).any():
        handles.append(Patch(color="lightgray", label="no way to stand"))
    ax.legend(handles=handles, loc="lower right")

    ax.set_xlim(velocity_values[0] - delta / 2, velocity_values[-1] + delta / 2)
    ax.set_ylim(np.pi / 8 - 0.005, np.pi / 7 + 0.016)
    ax.set_xlabel(r"initial speed on the section $\dot{\theta}_0$ [rad/s]")
    ax.set_ylabel(r"first stride chosen by the policy, $\alpha$ [rad]")
    ax.set_title("Steps needed to reach standstill from each initial speed")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig

#build the table and policy at one resolution
def make_policy(params, RoA_space, thetas, angular_velocities, n_velocities, n_alphas):
    velocity_values, alpha_values, status_table, next_velocity_table = create_lookup_table(
        params, RoA_space, thetas, angular_velocities, n_velocities, n_alphas)
    steps_to_stand, best_alpha = backward_induction(
        velocity_values, alpha_values, status_table, next_velocity_table,
        RoA_space, thetas, angular_velocities)
    return velocity_values, steps_to_stand, best_alpha

#run the full walker with the policy from each start speed; steps taken (inf = failed)
def policy_step_counts(start_speeds, params, RoA_space, thetas, angular_velocities,
        velocity_values, best_alpha):
    counts = []
    for w0 in start_speeds:
        _, _, outcome, switch_times, _, _ = simulate_walker_with_control(
            [0.0, w0], params, RoA_space, thetas, angular_velocities, velocity_values, best_alpha)
        counts.append(len(switch_times) if outcome == "standing" else np.inf)
    return np.array(counts)

#grid resolution study: find the coarsest table that still works
def resolution_study(params, RoA_space, thetas, angular_velocities, row_options,
    n_alphas=15, reference_rows=300, n_tests=150, max_extra_percent=5.0):
    froude = np.sqrt(2 * params["gravity"] / params["length"])
    rng = np.random.default_rng(0) #fixed seed: same test speeds every run
    test_speeds = np.sort(rng.uniform(0.0, froude, n_tests)) #almost never on a grid row

    #reference: a very fine table
    ref_velocities, _, ref_alpha = make_policy(params, RoA_space, thetas, angular_velocities,
        reference_rows, n_alphas)
    ref_steps = policy_step_counts(test_speeds, params, RoA_space, thetas, angular_velocities,
        ref_velocities, ref_alpha)

    print(f"reference ({reference_rows} rows): worst case {ref_steps.max():.0f} steps")
    print(" rows | failed | broken promises | worst steps | extra-step starts | time [s] | passes")
    for n_velocities in row_options:
        t0 = time.time()
        velocity_values, steps_to_stand, best_alpha = make_policy(
            params, RoA_space, thetas, angular_velocities, n_velocities, n_alphas)
        build_time = time.time() - t0
        actual = policy_step_counts(test_speeds, params, RoA_space, thetas, angular_velocities,
                                    velocity_values, best_alpha)
        #what the table promised for each test speed (its nearest row)
        delta = velocity_values[1] - velocity_values[0]
        rows = np.round((test_speeds - velocity_values[0]) / delta).astype(int)
        predicted = steps_to_stand[rows]

        failed = int(np.isinf(actual).sum())
        broken = int(np.sum(actual > predicted))
        worst = actual.max()
        extra = 100 * np.mean(actual > ref_steps)
        passes = failed == 0 and broken == 0 and worst <= ref_steps.max() and extra <= max_extra_percent
        print(f" {n_velocities:4d} | {failed:6d} | {broken:15d} | {worst:11.0f} | "
            f"{extra:16.1f}% | {build_time:8.1f} | {passes}")

def plot_RoA(thetas, angular_velocities, RoA_space):
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.pcolormesh(thetas, angular_velocities, RoA_space.T, shading="nearest", cmap="Blues")
    ax.set(xlabel=r"$\theta$ [rad]", ylabel=r"$\dot{\theta}$ [rad/s]",
    title="Region of attraction of the ankle controller")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig

def plot_walker_run(start_state, params, RoA_space, thetas, angular_velocities,
                    velocity_values=None, best_alpha=None):
    time_traj, state_traj, outcome, switch_times, switch_states, alphas_used = simulate_walker_with_control(
        start_state, params, RoA_space, thetas, angular_velocities, velocity_values, best_alpha)
    print(f"start ({start_state[0]:g}, {start_state[1]:.2f}): {outcome} after {len(switch_times)} spoke switches, alphas {np.round(alphas_used, 3)}")

    fig, ax = plt.subplots(figsize=(9, 6))
    ax.pcolormesh(thetas, angular_velocities, RoA_space.T, shading="nearest",
        cmap="Blues", vmin=0, vmax=2, zorder=0)
    if best_alpha is None: #fixed alpha: the walls don't move
        theta_reset = params["incline"] - params["angle_of_attack"]
        theta_strike = params["incline"] + params["angle_of_attack"]
        ax.axvline(theta_reset, color="tab:blue", ls="--", lw=1, label=r"reset wall $\gamma-\alpha$")
        ax.axvline(theta_strike, color="tab:red", ls="--", lw=1, label=r"heelstrike wall $\gamma+\alpha$")
    else: #policy: mark the Poincare section instead
        ax.plot([0, 0], [0, 1.05 * state_traj[:, 1].max()], color="tab:green", lw=3, alpha=0.5,
                label=r"Poincaré section $\theta=0$")

    jump_indices = np.where(np.abs(np.diff(state_traj[:, 0])) > 0.2)[0]
    for k, segment in enumerate(np.split(state_traj, jump_indices + 1)):
        ax.plot(segment[:, 0], segment[:, 1], lw=1.4, color="dimgray", zorder=2,
                label="stance" if k == 0 else None)
    for k, idx in enumerate(jump_indices):
        after = state_traj[idx + 1]
        ax.plot([switch_states[k, 0], after[0]], [switch_states[k, 1], after[1]],
                ls=":", lw=1, color="purple", zorder=2, label="impact jump" if k == 0 else None)

    ax.plot(*state_traj[0], "o", color="black", markersize=6, label="start", zorder=3)
    if len(switch_states) > 0:
        ax.plot(switch_states[:, 0], switch_states[:, 1], "^", color="purple",
                markersize=8, label="spoke switch", zorder=3)
        for n, ((theta_hit, velocity_hit), alpha) in enumerate(zip(switch_states, alphas_used), start=1):
            ax.annotate(rf"{n}: $\alpha={alpha:.3f}$", (theta_hit, velocity_hit), textcoords="offset points",
                        xytext=(-8, 4), ha="right", color="purple", fontsize=9)

    end_markers = {
        "standing": dict(marker="o", color="green", markersize=9, label="stabilized"),
        "fell back": dict(marker="X", color="red", markersize=11, label="failed"),
        "timeout": dict(marker="s", color="orange", markersize=8, label="timeout"),
    }
    ax.plot(*state_traj[-1], linestyle="none", zorder=4, **end_markers[outcome])

    ax.axhline(0, color="black", lw=0.6)
    ax.axvline(0, color="black", lw=0.6)
    ax.grid(True, which="major", color="gray", alpha=0.35, lw=0.6)
    ax.minorticks_on()
    ax.grid(True, which="minor", color="gray", alpha=0.15, lw=0.4)
    ax.set_axisbelow(False)
    ax.set_xlim(thetas[0], thetas[-1])
    ax.set_xlabel(r"$\theta$ [rad]")
    ax.set_ylabel(r"$\dot{\theta}$ [rad/s]")
    policy_text = "with policy" if best_alpha is not None else "fixed $\\alpha$"
    ax.set_title(rf"Walker from $(\theta_0, \dot\theta_0) = ({start_state[0]:g}, {start_state[1]:.2f})$, "
        f"{policy_text}: {outcome} after {len(switch_times)} steps")
    handles, labels = ax.get_legend_handles_labels()
    handles.append(Patch(color=plt.cm.Blues(0.5)))
    labels.append("region of attraction")
    ax.legend(handles, labels, loc="center left", bbox_to_anchor=(1.02, 0.5), frameon=False)
    fig.tight_layout()
    return fig


if __name__ == "__main__":
    RUN_RESOLUTION_STUDY = True #slow; set False once you've chosen the grid

    params = model.generate_params()
    thetas, angular_velocities, RoA_space = create_RoA(params) #build once, reuse everywhere
    plot_RoA(thetas, angular_velocities, RoA_space)

    #event guard test with fixed alpha
    plot_walker_run([0.0, 2.0], params, RoA_space, thetas, angular_velocities)

    #lookup table and policy
    velocity_values, alpha_values, status_table, next_velocity_table = create_lookup_table(
        params, RoA_space, thetas, angular_velocities, n_velocities=90, n_alphas=15)
    steps_to_stand, best_alpha = backward_induction(
        velocity_values, alpha_values, status_table, next_velocity_table,
        RoA_space, thetas, angular_velocities)
    plot_steps_to_standstill(velocity_values, steps_to_stand, best_alpha)

    #a start that needs 3 steps, walked with the policy
    three_step_speeds = velocity_values[steps_to_stand == 3]
    plot_walker_run([0.0, three_step_speeds[len(three_step_speeds) // 2]], params, RoA_space,
                    thetas, angular_velocities, velocity_values, best_alpha)

    if RUN_RESOLUTION_STUDY:
        resolution_study(params, RoA_space, thetas, angular_velocities,
            row_options=[30, 60, 80, 90, 100, 150])

    plt.show()