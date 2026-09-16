"""
Visualization tool to show phase portrait orbits at different initial conditions
"""

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp
from models import inverted_pendulum_walker as model

params = {
    "gravity": 9.81,  # m/s^2
    "length": 1.0,  # m
    "mass": 1.0,  # kg
    "incline": 0.06,  # rad
    "angle_of_attack": np.pi / 8,  # rad
    "ankle_torque": 0.0,  # N m
}

def trace_orbit(theta0, theta_dot0, params, t_max=5.0):
    """Integrate one stance phase until it hits either wall (or times out)."""
    gamma, alpha = params["incline"], params["angle_of_attack"]
    left, right = gamma - alpha, gamma + alpha

    def hit_right(t, x, *args):
        return x[0] - right
    hit_right.terminal, hit_right.direction = True, 1

    def hit_left(t, x, *args):
        return x[0] - left
    hit_left.terminal, hit_left.direction = True, -1

    sol = solve_ivp(model.dynamics, (0, t_max), [theta0, theta_dot0], args=(params,),
                    events=[hit_right, hit_left], max_step=0.005, rtol=1e-9, atol=1e-9)
    outcome = "step" if sol.t_events[0].size else "fall back"
    return sol.y[0], sol.y[1], outcome


def section_crossing(th, thd):
    """Angular velocity where an orbit crosses the Poincare section theta = 0 moving forward.
    Returns None if the orbit never crosses it."""
    for i in range(len(th) - 1):
        if th[i] < 0.0 <= th[i + 1] and thd[i + 1] > 0:
            #linear interpolation between the two samples on either side of theta = 0
            frac = (0.0 - th[i]) / (th[i + 1] - th[i])
            return thd[i] + frac * (thd[i + 1] - thd[i])
    return None


def add_arrow(ax, x, y, color):
    """Small arrow at the middle of a traced orbit to show the direction of flow."""
    i = len(x) // 2
    if i + 1 < len(x):
        ax.annotate("", xy=(x[i + 1], y[i + 1]), xytext=(x[i], y[i]),
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=1.5, mutation_scale=14))


def plot_phase_portrait(params, start_speeds=(0.2, 0.5, 0.8, 1.0, 1.3, 1,4, 1.5, 1.6, 2, 2.5, 3)):
    g, L = params["gravity"], params["length"]
    gamma, alpha = params["incline"], params["angle_of_attack"]
    left, right = gamma - alpha, gamma + alpha

    fig, ax = plt.subplots(figsize=(9, 6))
    th_lim = (left - 0.25, right + 0.25)
    thd_lim = (-1.5, 3.0)

    # background: faint energy contours of the unforced pendulum (every orbit is a level set)
    TH, THD = np.meshgrid(np.linspace(*th_lim, 400), np.linspace(*thd_lim, 400))
    E = 0.5 * THD**2 + (g / L) * np.cos(TH)
    ax.contour(TH, THD, E, levels=30, colors="lightgray", linewidths=0.6, zorder=0)

    # separatrix: orbits that just barely reach upright (theta = 0) with zero speed
    th_s = np.linspace(*th_lim, 400)
    sep = 2 * np.sqrt(g / L) * np.abs(np.sin(th_s / 2))
    ax.plot(th_s, sep, "k--", lw=1, label="separatrix")
    ax.plot(th_s, -sep, "k--", lw=1)

    # shade the region outside the stance phase and draw the walls
    ax.axvspan(th_lim[0], left, color="gray", alpha=0.15)
    ax.axvspan(right, th_lim[1], color="gray", alpha=0.15)
    ax.axvline(left, color="tab:blue", lw=2.5, label=r"$\gamma-\alpha$ (reset)")
    ax.axvline(right, color="tab:red", lw=2.5, label=r"$\gamma+\alpha$ (spoke switch)")

    # Poincare section: theta = 0, crossed moving forward (theta_dot > 0 only)
    ax.plot([0, 0], [0, thd_lim[1]], color="tab:green", lw=4, alpha=0.6, zorder=1,
            label=r"Poincaré section $\theta=0,\ \dot\theta>0$")

    # traced orbits starting just after impact at theta = gamma - alpha
    colors = plt.cm.viridis(np.linspace(0, 0.9, len(start_speeds)))
    for w0, c in zip(start_speeds, colors):
        th, thd, outcome = trace_orbit(left, w0, params)
        ls = "-" if outcome == "step" else ":"
        ax.plot(th, thd, color=c, lw=2, ls=ls,
                label=rf"$\dot\theta_0={w0}$ ({outcome})")
        add_arrow(ax, th, thd, c)

        # mark where this orbit crosses the section: that speed is theta_dot_k
        w_k = section_crossing(th, thd)
        if w_k is not None:
            ax.plot(0, w_k, "o", color=c, markersize=8, markeredgecolor="black", zorder=4)
            ax.annotate(rf"$\dot\theta_k={w_k:.2f}$", (0, w_k), textcoords="offset points",
                        xytext=(-8, 6), ha="right", fontsize=9)

    ax.axhline(0, color="k", lw=0.5)
    ax.axvline(0, color="k", lw=0.5)
    ax.set(xlim=th_lim, ylim=thd_lim, xlabel=r"$\theta$ [rad]",
        ylabel=r"$\dot\theta$ [rad/s]",
        title=rf"Stance phase portrait, $\gamma={gamma}$, $\alpha={alpha:.3f}$")
    ax.legend(loc="upper left", fontsize=8, framealpha=0.9)
    fig.tight_layout()
    return fig, ax


if __name__ == "__main__":
    fig, ax = plot_phase_portrait(params)
    plt.show()