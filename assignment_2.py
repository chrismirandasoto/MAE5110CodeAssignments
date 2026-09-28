from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

from controllers import (
    compute_ankle_torque,
    compute_max_step_policy,
    compute_min_step_policy,
)
from integrators import rk4
from models import inverted_pendulum_walker as model
from walker_analysis import (
    build_step_lookup,
    choose_state_count,
    compute_roa,
    find_roa_velocity_limit,
    find_section_roa,
    find_test_state,
    print_grid_check,
    simulate_recovery,
)


def plot_roa(angles, velocities, roa):
    plt.figure(figsize=(7, 5))

    plt.imshow(
        roa,
        origin="lower",
        extent=[angles[0], angles[-1], velocities[0], velocities[-1]],
        aspect="auto",
    )

    plt.xlabel(r"$\theta$ (rad)")
    plt.ylabel(r"$\dot{\theta}$ (rad/s)")
    plt.title("Ankle Controller Region of Attraction")


def plot_lookup(velocities, alphas, next_velocity):
    plt.figure(figsize=(7, 5))

    plt.imshow(
        next_velocity,
        origin="lower",
        extent=[
            np.rad2deg(alphas[0]),
            np.rad2deg(alphas[-1]),
            velocities[0],
            velocities[-1],
        ],
        aspect="auto",
    )

    plt.xlabel(r"$\alpha$ (deg)")
    plt.ylabel(r"$\dot{\theta}_k$ (rad/s)")
    plt.title(r"Next Poincaré Velocity $\dot{\theta}_{k+1}$")
    plt.colorbar(label=r"$\dot{\theta}_{k+1}$ (rad/s)")


def plot_steps_to_stand(velocities, steps):
    plot_steps = steps.astype(float)
    plot_steps[steps < 0] = np.nan

    plt.figure(figsize=(7, 5))
    plt.step(velocities, plot_steps, where="mid")

    plt.xlabel(r"Initial $\dot{\theta}$ (rad/s)")
    plt.ylabel("Steps to stand")
    plt.title("Steps Required to Reach Standing")
    plt.grid()


def plot_recovery(states, initial_velocity, title):
    plt.figure(figsize=(7, 5))
    plt.plot(states[0], states[1])
    plt.scatter([0.0], [0.0], marker="x")

    plt.xlabel(r"$\theta$ (rad)")
    plt.ylabel(r"$\dot{\theta}$ (rad/s)")
    plt.title(f"{title} from $\\dot{{\\theta}}_0={initial_velocity:.2f}$ rad/s")
    plt.grid()


def simulate_balance(params, dt=1e-4, sim_time=3.0):
    test_params = params.copy()
    state = np.array([0.03, 0.0])

    times = np.arange(round(sim_time / dt) + 1) * dt
    states = np.zeros((2, len(times)))
    states[:, 0] = state

    for i, time in enumerate(times[:-1]):
        test_params["ankle_torque"] = compute_ankle_torque(state, test_params)
        state = rk4(model.dynamics, time, state, dt, test_params)
        states[:, i + 1] = state

    return times, states, dt


def save_balance_animation(times, states, dt, params, output):
    fig, ax = plt.subplots(figsize=(8, 5), layout="constrained")

    fps = 25
    stride = round(1 / (fps * dt))
    frames = list(range(0, len(times), stride))

    if frames[-1] != len(times) - 1:
        frames.append(len(times) - 1)

    def draw_frame(i):
        frame_params = params.copy()
        frame_params["ankle_torque"] = compute_ankle_torque(
            states[:, i],
            frame_params,
        )

        model.visualize(states[:, i], frame_params, ax=ax)
        ax.set_title(f"t = {times[i]:.2f} s")

    animation = FuncAnimation(
        fig,
        draw_frame,
        frames=frames,
        interval=1000 / fps,
        repeat=False,
    )

    animation.save(output / "walker.gif", writer=PillowWriter(fps=fps))


def main():
    params = {
        "gravity": 9.81,
        "length": 1.0,
        "mass": 1.0,
        "incline": 0.06,
        "angle_of_attack": np.pi / 8,
        "ankle_torque": 0.0,
    }

    output = Path("output/assignment_2")
    output.mkdir(parents=True, exist_ok=True)

    # standing controller animation
    times, balance_states, balance_dt = simulate_balance(params)
    save_balance_animation(times, balance_states, balance_dt, params, output)

    print(f"Saved {output / 'walker.gif'}")

    # ankle controller region of attraction
    roa_angles, roa_velocities, roa = compute_roa(params)
    plot_roa(roa_angles, roa_velocities, roa)
    plt.savefig(output / "roa.png", dpi=200, bbox_inches="tight")

    roa_limit = find_roa_velocity_limit(roa_angles, roa_velocities, roa)
    state_count = choose_state_count(params, roa_limit)

    print(f"\nRoA velocity limit at theta = 0: {roa_limit:.3f} rad/s")
    print_grid_check(params, roa_limit, state_count)
    print(f"\nSelected velocity grid: {state_count} states")
    
    # Poincare state-action lookup
    (
        step_velocities,
        step_alphas,
        next_velocity,
        reaches_roa,
        footstrikes,
    ) = build_step_lookup(
        params,
        roa_angles,
        roa_velocities,
        roa,
        state_count,
    )

    valid = np.count_nonzero(np.isfinite(next_velocity))

    print("\nStep lookup table")
    print(f"velocity states: {len(step_velocities)}")
    print(f"alpha actions: {len(step_alphas)}")
    print(f"valid transitions: {valid}/{next_velocity.size}")

    plot_lookup(step_velocities, step_alphas, next_velocity)
    plt.savefig(output / "poincare_lookup.png", dpi=200, bbox_inches="tight")

    section_roa = find_section_roa(
        step_velocities,
        roa_angles,
        roa_velocities,
        roa,
    )

    # minimum and maximum step policies
    steps_to_stand, min_policy = compute_min_step_policy(
        step_velocities,
        step_alphas,
        next_velocity,
        reaches_roa,
        footstrikes,
        section_roa,
    )

    max_steps, max_policy = compute_max_step_policy(
        step_velocities,
        step_alphas,
        next_velocity,
        reaches_roa,
        footstrikes,
        section_roa,
    )

    print("\nSteps to stand")

    for n in range(np.max(steps_to_stand) + 1):
        count = np.count_nonzero(steps_to_stand == n)
        print(f"{n} steps: {count} states")

    print(f"unreachable: {np.count_nonzero(steps_to_stand < 0)} states")

    plot_steps_to_stand(step_velocities, steps_to_stand)
    plt.savefig(output / "steps_to_stand.png", dpi=200, bbox_inches="tight")

    # choose a clean 3-step example
    test_index, min_states, min_footstrikes = find_test_state(
        steps_to_stand,
        step_velocities,
        min_policy,
        params,
        roa_angles,
        roa_velocities,
        roa,
    )

    recovery_velocity = step_velocities[test_index]

    print("\nMinimum-step recovery")
    print(f"initial velocity: {recovery_velocity:.3f} rad/s")
    print(f"predicted steps: {steps_to_stand[test_index]}")
    print(f"actual footstrikes: {min_footstrikes}")
    print("reached standing: True")

    plot_recovery(min_states, recovery_velocity, "Minimum-Step Recovery")
    plt.savefig(output / "minimum_recovery.png", dpi=200, bbox_inches="tight")

    # maximum number of steps from the same initial condition
    max_states, max_footstrikes, max_success = simulate_recovery(
        recovery_velocity,
        params,
        step_velocities,
        max_policy,
        roa_angles,
        roa_velocities,
        roa,
    )

    print("\nMaximum-step recovery")
    print(f"predicted max steps: {max_steps[test_index]}")
    print(f"actual footstrikes: {max_footstrikes}")
    print(f"reached standing: {max_success}")

    plot_recovery(max_states, recovery_velocity, "Maximum-Step Recovery")
    plt.savefig(output / "maximum_recovery.png", dpi=200, bbox_inches="tight")

    plt.show()


if __name__ == "__main__":
    main()