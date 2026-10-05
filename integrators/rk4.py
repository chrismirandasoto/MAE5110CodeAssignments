def rk4(dyn, t, state, timestep, params):
    k1 = dyn(t, state, params)
    k2 = dyn(t + timestep / 2, state + k1 * timestep / 2, params)
    k3 = dyn(t + timestep / 2, state + k2 * timestep / 2, params)
    k4 = dyn(t + timestep, state + timestep * k3, params)
    new_state = state + (timestep / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

    return new_state


## simulation loop
##   for step, t in enumerate(time_traj[:-1]):
##      state_traj[:, step + 1] = state_traj[:, step] + timestep * model.dynamics(
##           t, state_traj[:, step], params)
