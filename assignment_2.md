# Assignment 2 Report

## Sketches

Here are my sketches for the inverted pendulum walker. I used these to work through the geometry, what $\alpha$ is doing, some different walking configurations, and the state-space / Poincaré section setup before finalizing the controller.

![Assignment 2 sketches](output/assignment_2/assignment2sketch.PNG)

The main idea from the sketches is that the walker evolves continuously during each stance, while the ankle torque $\tau$ acts as a continuous control input. The landing angle $\alpha$ is chosen once per step and changes where the next touchdown happens. Because of this, the touchdown condition itself changes with $\alpha$.

---

## Region of Attraction for the Ankle Controller

For the continuous-time balancing controller, I used feedback linearization around the upright equilibrium.

The continuous dynamics are

$$
\ddot{\theta}
=
\frac{g}{\ell}\sin\theta
+
\frac{\tau}{m\ell^2}.
$$

I used the control law

$$
\tau
=
-mg\ell\sin\theta
-
m\ell^2
\left(
k_p\theta+k_d\dot{\theta}
\right)
$$

with

$$
k_p=4,\qquad k_d=4.
$$

The first term cancels the nonlinear gravitational term, while the remaining terms stabilize the upright equilibrium. The torque command was also clipped to the required bounds

$$
\tau \in [-0.1mg\ell,\ 0.05mg\ell].
$$

I estimated the region of attraction by simulating the ankle controller from a grid of different initial values of $\theta$ and $\dot{\theta}$. States that converged to the upright equilibrium were classified as being inside the RoA.

![Ankle controller region of attraction](output/assignment_2/roa.png)

At $\theta=0$, the positive velocity portion of the estimated RoA extends to approximately

$$
\dot{\theta}=0.300\text{ rad/s}.
$$

Outside of this region, the ankle controller is left off. Once the walking trajectory enters the RoA, the ankle controller takes over and brings the walker to standing.

---

## Choice of Poincaré Section

For the rimless wheel, touchdown was a convenient Poincaré section because the touchdown angle was fixed. In this model, however, the touchdown angle depends on the control input $\alpha$.

The forward touchdown condition is

$$
\theta_{TD}=\gamma+\alpha.
$$

Since $\alpha$ changes from step to step, using touchdown as the section would mean the section itself also moves.

Instead, I chose

$$
\boxed{\theta=0,\qquad \dot{\theta}>0}
$$

as the Poincaré section.

This gives a section that is fixed regardless of the value of $\alpha$. It is also transverse to the relevant walking trajectories because the walker crosses it with positive angular velocity.

This lets the step-to-step system use only

$$
\dot{\theta}_k
$$

as its state, while

$$
\alpha_k
$$

is the control input.

For each combination of $\dot{\theta}_k$ and $\alpha_k$, I simulated the walker forward until it either reached the standing-controller RoA or crossed $\theta=0$ again. This gives the next state $\dot{\theta}_{k+1}$.

![Next Poincaré velocity map](output/assignment_2/poincare_lookup.png)

---

## Grid Resolution

The Poincaré velocity grid covers the required range

$$
0\leq\dot{\theta}\leq\sqrt{\frac{2g}{\ell}},
$$

which for this model is approximately

$$
0\leq\dot{\theta}\leq4.43\text{ rad/s}.
$$

The simulated next velocity normally does not land exactly on a grid point, so I use the closest velocity state for the next lookup.

To choose the grid resolution, I limited the maximum nearest-state error to 10% of the standing controller's capture range on the Poincaré section.

The measured RoA velocity limit was approximately

$$
0.300\text{ rad/s},
$$

so the maximum allowed nearest-state error was

$$
e_{\max}=0.0300\text{ rad/s}.
$$

For an evenly spaced velocity grid, the worst-case nearest-neighbor error is

$$
e_{\max}=\frac{\Delta\dot{\theta}}{2}.
$$

The grid sizes around the cutoff gave:

| Velocity states | Spacing (rad/s) | Maximum error (rad/s) | Result |
| ---: | ---: | ---: | --- |
| 73 | 0.0615 | 0.0308 | Fail |
| 74 | 0.0607 | 0.0303 | Fail |
| 75 | 0.0599 | 0.0299 | Pass |
| 76 | 0.0591 | 0.0295 | Pass |

I therefore used **75 velocity states**, since it was the coarsest grid that satisfied the criterion. I used 31 evenly spaced values of $\alpha$ over the allowed range

$$
\alpha\in\left[\frac{\pi}{8},\frac{\pi}{7}\right].
$$

The final lookup table had:

- velocity states: **75**
- alpha actions: **31**
- valid transitions: **1610 / 2325**

---

## Steps to Standstill

Starting from states that were already inside the RoA, I worked backwards through the lookup table to determine which states could reach the RoA in one step, two steps, three steps, and so on.

The final grid gave:

| Steps to stand | Number of states |
| ---: | ---: |
| 0 | 6 |
| 1 | 22 |
| 2 | 20 |
| 3 | 27 |
| Unreachable | 0 |

The number of steps required as a function of initial angular velocity is shown below.

![Steps required to reach standing](output/assignment_2/steps_to_stand.png)

All of the states in the selected grid were eventually able to reach the standing-controller RoA.

---

## Minimum-Step Recovery

I selected an initial condition that required at least three steps:

$$
\dot{\theta}_0=2.933\text{ rad/s}.
$$

The minimum-step policy predicted that the walker would require three footstrikes before reaching the standing-controller RoA.

The full simulation gave:

- predicted footstrikes: **3**
- actual footstrikes: **3**
- reached standing: **True**

![Minimum-step recovery](output/assignment_2/minimum_recovery.png)

The smooth parts of the trajectory are the continuous inverted-pendulum dynamics during stance, while the jumps come from the impact resets. Once the state enters the RoA, the ankle controller brings the system to the upright equilibrium near $(0,0)$.

---

## Maximum-Step Recovery

For the same initial condition,

$$
\dot{\theta}_0=2.933\text{ rad/s},
$$

I also found a policy that lets the walker continue walking for as many steps as possible before eventually entering the RoA.

The result was:

- predicted maximum footstrikes: **4**
- actual footstrikes: **4**
- eventually reached standing: **True**

![Maximum-step recovery](output/assignment_2/maximum_recovery.png)

This shows that changing $\alpha$ can change how quickly the walker loses enough energy to enter the balancing controller's RoA. The minimum-step policy reaches it after three footstrikes, while another valid control sequence keeps the walker going for four footstrikes from the same initial condition.

