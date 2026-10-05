"""Sanity checks for the pendulum model.

With no damping and no applied torque the pendulum is conservative, so the
total energy must stay constant along a trajectory. Any drift points at a
mistake in the dynamics, the energy bookkeeping, or the integrator.
"""

import numpy as np

from integrators import rk4
from models import pendulum as model

TIMESTEP = 1e-3
STEP_COUNT = 100


def simulate_trajectory(params, initial_state):
    """Roll the pendulum out with RK4 and return the (2, STEP_COUNT + 1) states."""
    state_traj = np.zeros((2, STEP_COUNT + 1))
    state_traj[:, 0] = initial_state
    state = np.asarray(initial_state, dtype=float)
    for step in range(STEP_COUNT):
        state = rk4(model.dynamics, step * TIMESTEP, state, TIMESTEP, params)
        state_traj[:, step + 1] = state

    return state_traj


def simulate_conservative_pendulum():
    """Roll out the undamped, untorqued pendulum and return its trajectory."""
    params = model.generate_params()
    params["damping_coeff"] = 0.0  # no dissipation
    params["torque"] = 0.0  # no actuation

    # Start away from both equilibrium points using an offset
    initial_state = np.array([np.pi / 4, 0.0])

    state_traj = simulate_trajectory(params, initial_state)

    return params, state_traj


def test_zero_damping_and_torque_conserves_energy():
    params, state_traj = simulate_conservative_pendulum()

    kinetic_energy, potential_energy = model.calculate_energy(state_traj, params)
    total_energy = kinetic_energy + potential_energy
    energy_change = np.diff(total_energy)

    # Filter out pendulums at equilibrium that naturally conserve energy
    assert not np.isclose(state_traj[0, -1], state_traj[0, 0])
    assert np.any(np.abs(state_traj[1]) > 0.0)

    # atol is far above RK4's ~1e-15 round-off at this timestep, but far below
    # any physically meaningful gain or loss of energy.
    assert np.all(np.isclose(energy_change, 0.0, atol=1e-9)), (
        f"energy drifted by {total_energy[-1] - total_energy[0]:.6e} J over "
        f"{STEP_COUNT} steps"
    )


def test_damping_opposes_motion_and_decays_at_the_right_rate():
    """Damping must always fight the current motion, with the right magnitude."""
    params = model.generate_params()
    params["torque"] = 0.0
    # Be sure to avoid setting mass or length to a value of 1, otherwise
    # an error in the m * L^2 expression would not arise
    params["mass"] = 2.0
    params["length"] = 1.5
    inertia = params["mass"] * params["length"] ** 2

    # At the upright angle the gravity term drops out (sin 0 == 0) and the
    # torque is zero, so the damping term is the only one left and can be read
    # straight off the acceleration.
    for angular_velocity in (3.0, -3.0, 0.5):
        derivative = model.dynamics(0.0, np.array([0.0, angular_velocity]), params)
        expected_acceleration = -params["damping_coeff"] * angular_velocity / inertia

        assert np.isclose(derivative[1], expected_acceleration)
        # "Opposes the motion" means the signs disagree, no matter the way it spins.
        assert derivative[1] * angular_velocity < 0.0

    # No gravity leaves a pure first-order decay with the closed-form solution 
    # omega(t) = omega_0 * exp(-c * t / (m * L^2)). Checking against it shows the
    # the damping coefficient's scale.
    decay_params = {**params, "gravity": 0.0, "damping_coeff": 0.4}
    initial_state = np.array([0.3, 2.0])
    state_traj = simulate_trajectory(decay_params, initial_state)

    elapsed = np.arange(STEP_COUNT + 1) * TIMESTEP
    expected_velocity = initial_state[1] * np.exp(
        -decay_params["damping_coeff"] * elapsed / inertia
    )

    # Guard against an invalid pass: the velocity has to have actually decayed.
    assert np.abs(state_traj[1, -1]) < np.abs(initial_state[1])

    assert np.all(np.isclose(state_traj[1], expected_velocity, atol=1e-9)), (
        f"velocity departs from exponential decay by at most "
        f"{np.abs(state_traj[1] - expected_velocity).max():.6e} rad/s"
    )


def test_torque_accelerates_the_pendulum_and_can_hold_it_against_gravity():
    """Torque must enter as tau / (m * L^2), with the sign that lifts the angle."""
    params = model.generate_params()
    params["damping_coeff"] = 0.0
    # Same as previous test, avoid m or l = 1
    params["mass"] = 2.0
    params["length"] = 1.5
    mass, length, gravity = params["mass"], params["length"], params["gravity"]
    inertia = mass * length**2

    # Upright and at rest, gravity and damping both contribute nothing, so the
    # acceleration is the torque term alone.
    for torque in (2.0, -2.0):
        derivative = model.dynamics(
            0.0, np.array([0.0, 0.0]), {**params, "torque": torque}
        )

        assert np.isclose(derivative[1], torque / inertia)
        # A positive torque should drive the angle positive.
        assert np.sign(derivative[1]) == np.sign(torque)

    # Gravity disabled results in a constant angular acceleration from torque
    driven_params = {**params, "gravity": 0.0, "torque": 3.0}
    initial_state = np.array([0.3, 0.0])
    state_traj = simulate_trajectory(driven_params, initial_state)

    elapsed = np.arange(STEP_COUNT + 1) * TIMESTEP
    acceleration = driven_params["torque"] / inertia
    expected_angle = initial_state[0] + 0.5 * acceleration * elapsed**2
    expected_velocity = acceleration * elapsed

    assert np.all(np.isclose(state_traj[0], expected_angle, atol=1e-9))
    assert np.all(np.isclose(state_traj[1], expected_velocity, atol=1e-9))

    # With gravity back on, a torque that exactly cancels the gravitational
    # moment at hold_angle must leave the pendulum sitting there.
    hold_angle = np.pi / 4
    holding_torque = -mass * gravity * length * np.sin(hold_angle)
    held_traj = simulate_trajectory({**params, "torque": holding_torque}, [hold_angle, 0.0])

    assert np.all(np.isclose(held_traj[0], hold_angle, atol=1e-9)), (
        f"angle moved by {np.abs(held_traj[0] - hold_angle).max():.6e} rad under "
        f"a holding torque of {holding_torque:.4f} N m"
    )
    assert np.all(np.isclose(held_traj[1], 0.0, atol=1e-9))
