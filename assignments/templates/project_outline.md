---
title: "[Project title]"
author: "[Your name and netID, or all teammates' names and netIDs]"
date: ""
documentclass: article
fontsize: 11pt
papersize: letter
geometry: margin=0.85in
pagestyle: empty
header-includes:
  - |
    ```{=latex}
    \usepackage{graphicx}
    \usepackage{titlesec}
    \titleformat{\section}{\normalfont\large\bfseries}{}{0pt}{}
    \titlespacing*{\section}{0pt}{1em}{0.35em}
    \setlength{\parindent}{0pt}
    \setlength{\parskip}{0.5em}
    \raggedbottom
    \makeatletter
    \renewcommand{\maketitle}{%
      \begingroup\centering
      {\Large\bfseries\@title\par}
      \vspace{0.35em}
      {\normalsize\@author\par}
      \endgroup\vspace{0.6em}}
    \makeatother
    % Start each example on a new PDF page without raw LaTeX in the Markdown body.
    \newif\ifoutlinefirstsection
    \outlinefirstsectiontrue
    \let\outlinesection\section
    \renewcommand{\section}{%
      \ifoutlinefirstsection\outlinefirstsectionfalse\else\clearpage\fi
      \outlinesection}
    ```
---

<!--
Copy this template for each proposal and replace the title, names, and prompts.
The headings are a starting point; adapt them if another structure communicates
your idea more clearly. This is an exploration, so say what you are still unsure about.
Include related work where it helps explain your idea.

Build from the folder containing your copy:
  pandoc project_outline.md --from=gfm+raw_attribute+smart --pdf-engine=pdflatex -o project_outline.pdf

The matching LaTeX template is generated with:
  pandoc project_outline.md --from=gfm+raw_attribute+smart --standalone -o project_outline.tex
-->

# *[MAE4110 Performance-driven example]*

## What do you want to achieve?

Get the robot to backflip on command.
A minimum test will be successful sim-to-real transfer of a policy that can perform a backflip (full 360 rotation) from standing.
Stretch goals will be to perform back-flips while moving, or to do other acrobatics (e.g. barrel-rolls, etc.).

## How will you show it?

To assess performance, we will look at how "clean" the landing is:

- how quickly rotational momentum is dissipated and if it has oscillations
- time from touch-down to stable standing
- as well as number of foot-repositioning (lifting the foot after it has touched-down).
- difference of the above between simulation and hardware experiments

# *[MAE5110 Contribution-driven example]*

## Context

Current state-of-the-art RL control for legged robots learn policies that output the joint position targets for a PD controller, implemented as part of the environment, of the form

$$
\tau = K_p (q_{\text{target}} - q) + K_d (\dot{q}_{\text{target}} - \dot{q}),
$$

where $\tau$ is the joint torque, $K_p$ and $K_d$ are feedback gains, and $q$ and $\dot{q}$ are the joint positions and velocities, respectively.
However, the policies typically only output the joint position targets, and leave the velocity target $`\dot{q}_{\text{target}} = 0`$.
This is often justified by the ambiguity introduced by an action space including both terms, since a desired torque $\tau$ can be achieved by different combinations of $q_{\text{target}}$ and $\dot{q}_{\text{target}}$.

<!-- Despite this, compared to established practices for tuning $K_p$ and $K_d$, the feedback gains are comparatively "soft", and it has been empirically observed that policies often learn to output desired positions $q_{\text{target}}$ that are well outside the kinematic limits of the robot.
This can lead to policies with lower performance, or require additional reward shaping to avoid dangerous motions that slam into hardstops. -->
<!-- This all suggests that the action space is somehow a blend of an impedance controller and a position controller. -->

## Contribution

We show that the an action space including both $q_{\text{target}}$ and $\dot{q}_{\text{target}}$ is ambiguous under typical reward shaping, but that eliminating this ambiguity results in policies that violate kinematic constraints less often, and are better able to deal with behaviors that require both high velocity and abrupt changes in velocity, such as coming to a quick stop from running.

## Experiment

We will establish the presence of ambiguity by comparing three cases:

1. action-space $\mathcal{A}: [q_{\text{target}}]$ (baseline)
2. action-space $\mathcal{A}: [q_{\text{target}}, \dot{q}_{\text{target}}]$.
3. action-space $\mathcal{A}: [q_{\text{target}}]$ with $\dot{q}_{\text{target}}$ computed from a history of $q_{\text{target}}$.

For each, we will compare the variance in policy output to variance in torque output, and in overall reward-output.

We will then benchmark custom rewards that separately penalize $q_{\text{target}}$ and $\dot{q}_{\text{target}}$ to disambiguate the action space, and validate them in terms of variance ratio (see above), and also in terms of frequency of the policy outputting kinematically infeasible commands during walking, running, and transitions.
<!--
Optional: put supporting figures on a second page. Each top-level heading starts
a new PDF page. To add a figures page, uncomment the following lines and replace
the path and caption:

# Supporting figures

![Caption explaining how the figure helps communicate your idea.](figure.png)
-->
