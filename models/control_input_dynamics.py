# Inverted pendulum walker control input code
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

from models import inverted_pendulum_walker as model

def apply_control_torque(state, params):
    min_torque = -0.1 * params["mass"] * params["gravity"] * params["length"]
    max_torque = 0.05 * params["mass"] * params["gravity"] * params["length"]
    control_input = Kp * (state[0] - desired_state[0]) + Kd * (state[1] - desired_state[1])
    return np.clip(control_input, min_torque, max_torque)

