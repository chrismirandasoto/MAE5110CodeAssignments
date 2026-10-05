import numpy as np
import matplotlib.pyplot as plt
import timeit

from models import pendulum as model
from integrators import explicit_euler as integrator
#from integrators import rk4 as integrator

# Basic simulation of the pendulum

params = {
    "gravity": 9.81,  # gravity m/s^2)
    "length": 1,  # rod length (m)
    "mass": 0.2,  # point mass at end of rod (kg)
    "damping_coeff": 0.0,  # damping coefficient (kg*m^2/s)
    "torque": 0.0,  # torque (Nm)
}


# some set-up
initial_state = np.array([np.pi / 4, 0.0])
timestep = 1e-5
def sweep_largest():
    timestep = 1e-5
    Stable = True
    while Stable == True:
        sim_time = 5.0
        n_timesteps = int(sim_time / timestep) + 1
        time_traj = np.arange(n_timesteps) * timestep
        state_traj = np.zeros((2, n_timesteps))
        state_traj[:, 0] = initial_state

        # simulation loop
        for step, t in enumerate(time_traj[:-1]):
            state_traj[:, step + 1] = integrator(model.dynamics, t, state_traj[:, step], timestep, params)

        # sanity check the energies
        kinetic_energy, potential_energy = model.calculate_energy(state_traj, params)
        if np.isclose(potential_energy[0] + kinetic_energy[0], potential_energy[-1] + kinetic_energy[-1], rtol=.001) == False:
            Stable = False
            plt.figure()
            plt.plot(time_traj, potential_energy, label="Potential energy")
            plt.plot(time_traj, kinetic_energy, label="Kinetic energy")
            plt.plot(time_traj, potential_energy + kinetic_energy, label="Total energy")
            plt.xlabel("Time (s)")
            plt.ylabel("Energy (J)")
            plt.title("Pendulum energy")
            plt.legend()
            plt.tight_layout()
            plt.show()
        else:
            timestep += 1e-5
    return timestep

def sweep_fixed():
    timestep = 1e-5
    sim_time = 5.0
    n_timesteps = int(sim_time / timestep) + 1
    time_traj = np.arange(n_timesteps) * timestep
    state_traj = np.zeros((2, n_timesteps))
    state_traj[:, 0] = initial_state

    # simulation loop
    for step, t in enumerate(time_traj[:-1]):
        state_traj[:, step + 1] = integrator(model.dynamics, t, state_traj[:, step], timestep, params)

    # sanity check the energies
    kinetic_energy, potential_energy = model.calculate_energy(state_traj, params)

    plt.figure()
    plt.plot(time_traj, potential_energy, label="Potential energy")
    plt.plot(time_traj, kinetic_energy, label="Kinetic energy")
    plt.plot(time_traj, potential_energy + kinetic_energy, label="Total energy")
    plt.xlabel("Time (s)")
    plt.ylabel("Energy (J)")
    plt.title("Pendulum energy")
    plt.legend()
    plt.tight_layout()
    plt.show()

    return timestep

#result = sweep_largest()
#print(f"Largest stable timestep: {result}")

elapsed = timeit.timeit(sweep_fixed, number=1)
print(f"Sweep took {elapsed:.4f} seconds")



# TODO: make a phase portrait plot
