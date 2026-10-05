import numpy as np
import matplotlib.pyplot as plt

from models import bouncingball as model
from integrators import explicit_euler as integrator
#from integrators import rk4 as integrator

# Basic simulation of the bouncing ball

params = {
    "gravity": 9.81,  # gravity m/s^2)
    "mass": 0.2,  # point mass at end of rod (kg)
    "restitution_coeff": .8  #unitless
}

# some set-up
initial_state = np.array([5 , 0.0])
timestep = 1e-5
sim_time = 5
n_timesteps = int(sim_time / timestep) + 1
time_traj = np.arange(n_timesteps) * timestep
state_traj = np.zeros((2, n_timesteps))
state_traj[:, 0] = initial_state

# simulation loop
for step, t in enumerate(time_traj[:-1]):
    if state_traj[0, step] <= 0 and state_traj[1, step] < 0:
                state_traj[1, step] = -params["restitution_coeff"] * state_traj[1, step]
                state_traj[0, step] = 0
    state_traj[:, step + 1] = integrator(model.dynamics, t, state_traj[:, step], timestep, params)


# sanity check the energies
kinetic_energy, potential_energy = model.calculate_energy(state_traj, params)
plt.figure()
plt.plot(time_traj, potential_energy, label="Potential energy")
plt.plot(time_traj, kinetic_energy, label="Kinetic energy")
plt.plot(time_traj, potential_energy + kinetic_energy, label="Total energy")
plt.xlabel("Time (s)")
plt.ylabel("Energy (J)")
plt.title("Bouncing ball energy")
plt.legend()
plt.tight_layout()
plt.show()

