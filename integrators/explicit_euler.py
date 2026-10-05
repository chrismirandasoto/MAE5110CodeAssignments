def explicit_euler(dyn, t, state, timestep, params):
    new_state = state + (timestep * dyn(t, state, params))

    return new_state


## simulation loop
##   for step, t in enumerate(time_traj[:-1]):
##      state_traj[:, step + 1] = state_traj[:, step] + timestep * model.dynamics(
##           t, state_traj[:, step], params)
