
CONTENTS OF MARKDOWN FILE
1. an explanation of your sanity checks, including what you expected and what happened; (5 pts)
2. a state-space plot showing the RoA of every stable attractor, including fixed points and limit cycles; (5 pts)
3. your one-dimensional return-map plot, with its fixed point and the identity line clearly marked; and (5 pts)
4. visualization and discussion of how the slope and number of spokes affects the RoA and local convergence (10 pts)"

ChrisMirandaSoto Explanation:

1. Throughout my code, I conducted several sanity checks at each stage. First, I wrote my code for
the spoke dynamics and made a diagram in my notes of what it should look like. If my code was correct,
I expected that the state trajectory should show an asymmetric plot where the data oscilates between
-alpha + gamma and alpha + gamma, increasing greatly, then sharpily dropping making a sawtooth wave
when spoke switch happened. Also, at initial conditions where I expected it to stall, I expected
to see pendulum behavior where the leg never switches and just oscilates. Other sanity checks include making my
RoA and checking specific initial conditions that were shown to become limit cycles by running them individually
and checking the angular velocity values after spoke switches to see if they truly converged, as well as conceptually evaluating whether my RoA vs and Floquet vs plots made sense, which they did.

2. 

