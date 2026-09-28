import numpy as np
import matplotlib.pyplot as plt
import timeit

from models import pendulum as model
from integrators import explicit_euler, rk4

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

#TIME SWEEP

sim_time = 5.0

def simulate(timestep, integrator):
    n_timesteps = int(sim_time / timestep) + 1
    time_traj = np.arange(n_timesteps) * timestep

    state_traj = np.zeros((2, n_timesteps))
    state_traj[:, 0] = initial_state

    for step, t in enumerate(time_traj[:-1]):
        state_traj[:, step + 1] = integrator(
            model.dynamics,
            t,
            state_traj[:, step],
            timestep,
            params,
        )

    return time_traj, state_traj


# Coarse timestep sweep
timesteps = [
    1e-5,
    2e-5,
    3e-5,
    4e-5,
    5e-5,
    6e-5,
    7e-5,
    8e-5,
    9e-5,
    1e-4,
]

max_acceptable_error = 0.002
largest_stable_timestep = None

for timestep in timesteps:
    time_traj, state_traj = simulate(timestep, explicit_euler)

    kinetic_energy, potential_energy = model.calculate_energy(
        state_traj, params
    )

    total_energy = kinetic_energy + potential_energy

    max_energy_error = np.max(
    np.abs(total_energy - total_energy[0])
    )

    print(
    f"dt = {timestep:.2e} s, "
    f"max energy error = {max_energy_error * 1000:.3f} mJ"
    )

    if max_energy_error < max_acceptable_error:
        largest_stable_timestep = timestep

print(
    "Largest stable timestep from coarse sweep:",
    largest_stable_timestep,
)


# Run baseline case for plotting
timestep = 1e-5


time_traj, state_traj = simulate(timestep, rk4)

kinetic_energy, potential_energy = model.calculate_energy(
    state_traj, params
)

# Timing comparison

euler_max_dt = 0.0002
rk4_max_dt = 0.17

# Compare both integrators using the same timestep
same_dt = 0.0002

euler_same_dt_time = timeit.timeit(
    lambda: simulate(same_dt, explicit_euler),
    number=3,
) / 3

rk4_same_dt_time = timeit.timeit(
    lambda: simulate(same_dt, rk4),
    number=3,
) / 3

print("\nTiming with same timestep:")
print(f"Euler, dt = {same_dt}: {euler_same_dt_time:.6f} s")
print(f"RK4,   dt = {same_dt}: {rk4_same_dt_time:.6f} s")


# Compare each integrator using its largest accurate timestep

euler_best_time = timeit.timeit(
    lambda: simulate(euler_max_dt, explicit_euler),
    number=3,
) / 3

rk4_best_time = timeit.timeit(
    lambda: simulate(rk4_max_dt, rk4),
    number=3,
) / 3

print("\nTiming with largest accurate timestep:")
print(
    f"Euler, dt = {euler_max_dt}: "
    f"{euler_best_time:.6f} s"
)
print(
    f"RK4,   dt = {rk4_max_dt}: "
    f"{rk4_best_time:.6f} s"
)


#PLOTTING

timestep = 1e-5
sim_time = 5.0

n_timesteps = int(sim_time / timestep) + 1
time_traj = np.arange(n_timesteps) * timestep
state_traj = np.zeros((2, n_timesteps))
state_traj[:, 0] = initial_state

# simulation loop
for step, t in enumerate(time_traj[:-1]):
    state_traj[:, step + 1] = state_traj[:, step] + timestep * model.dynamics(
        t, state_traj[:, step], params
    )

# sanity check the energies: since there is no actuation, and no damping, total energy should stay
# constant. If we turn on the damping coefficient, it should slowly bleed out energy until it comes to
# a stand-still.

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

# TODO: make a phase portrait plot
