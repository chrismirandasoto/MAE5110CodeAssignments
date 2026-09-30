"""
Pendulum sanity check code (by cm2233)
"""
import inspect

import numpy as np
import pytest

from models import pendulum #module being tested
from integrators import rk4

#simulate and return the states and total energy at each step for tests
def simulate(params, state, number_of_steps=100, time_step=1e-3):
    current_time = 0.0
    states = np.zeros((number_of_steps, len(state)))
    energy_over_time = np.zeros(number_of_steps)
    for step in range(number_of_steps):
        state = rk4(pendulum.dynamics, current_time, state, time_step, params)
        current_time += time_step
        kinetic, potential = pendulum.calculate_energy(state, params)
        states[step] = state
        energy_over_time[step] = kinetic + potential
    return states, energy_over_time

#create function to test pendulum model by checking if energy is constant all throughout
def test_pendulum():
    #import general params
    params = pendulum.generate_params()
    #set torque and damp coeff to 0 for sanity checking
    params["torque"] = 0
    params["damping_coeff"] = 0
    state = np.array([1.0, 1.0]) # non-equilbrium IC
    _, energy_over_time = simulate(params, state)
    for step in range(1, len(energy_over_time)):
        assert np.isclose(energy_over_time[step], energy_over_time[0], rtol=1e-6), \
            f"energy drifted at step {step}"

#test if damping is implemented correctly by seeing if energy consistenly decreases
def test_pendulum_damping():
    #import general params
    params = pendulum.generate_params()
    #set torque to zero and damp coeff to a non zero value for sanity checking
    params["torque"] = 0
    params["damping_coeff"] = 1.0 #fails if 0, passes if positive
    state = np.array([1.0, 1.0]) # non-equilbrium IC
    _, energy_over_time = simulate(params, state, number_of_steps=100, time_step=1e-1) #only works with large enough timestep
    for step in range(1, len(energy_over_time)):
        assert energy_over_time[step] < energy_over_time[step - 1]

#test if torque is implemented correctly by seeing if torque added energy
def test_pendulum_torque():
    #import general params
    params = pendulum.generate_params()
    #set torque to nonzero value and damp coeff to 0 for sanity checking
    params["torque"] = 0.5 # fails if 0, passes if positive
    params["damping_coeff"] = 0
    state = np.array([0.0, 0.0]) # equilbrium IC
    _, energy_over_time = simulate(params, state, number_of_steps=100, time_step=1e-1)
    assert energy_over_time[-1] > energy_over_time[0], \
        "torque did not add energy to the system"



