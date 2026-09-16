import numpy as np


def generate_params():
    return {
        "gravity": 9.81,
        "spoke_length": 1.0,
        "num_spokes": 6,
        "slope": np.deg2rad(15.0),
    }


def dynamics(t, state, params):
    gravity = params["gravity"]
    spoke_length = params["spoke_length"]

    angle = state[0]
    angular_velocity = state[1]

    angular_acceleration = (gravity / spoke_length) * np.sin(angle)

    return np.array([
        angular_velocity,
        angular_acceleration,
    ])


def detect_contact(state, params):
    num_spokes = params["num_spokes"]
    slope = params["slope"]

    half_spoke_angle = np.pi / num_spokes

    forward_contact = slope + half_spoke_angle
    backward_contact = slope - half_spoke_angle

    if state[0] >= forward_contact:
        return 1

    if state[0] <= backward_contact:
        return -1

    return 0


def reset_state(state, params, contact_direction):
    num_spokes = params["num_spokes"]

    half_spoke_angle = np.pi / num_spokes

    angle = state[0]
    angular_velocity = state[1]

    new_angle = (
        angle
        - contact_direction * 2.0 * half_spoke_angle
    )

    new_angular_velocity = (
        angular_velocity
        * np.cos(2.0 * half_spoke_angle)
    )

    return np.array([
        new_angle,
        new_angular_velocity,
    ])

def calculate_energy(state, params):
    gravity = params["gravity"]
    spoke_length = params["spoke_length"]

    angle = state[0]
    angular_velocity = state[1]

    kinetic_energy = 0.5 * spoke_length**2 * angular_velocity**2

    potential_energy = gravity * spoke_length * np.cos(angle)

    return kinetic_energy, potential_energy