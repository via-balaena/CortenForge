# Impulse method failure modes

Impulse-based contact resolves collisions as discrete velocity changes: at the moment two bodies touch, the solver applies an impulse sized to satisfy a velocity-level non-penetration condition plus a friction-cone condition, typically by solving a linear complementarity problem (LCP) or its projected Gauss-Seidel (PGS) approximation. This is the dominant formulation in rigid-body engines (ODE, Bullet, PhysX rigid-body, Havok) and has been ported to soft bodies in several frameworks. The [parent Ch 00](../00-why-ipc.md) claimed impulse-based contact is structurally wrong for `sim-soft` because its solution is a piecewise function of the input state, and piecewise functions have jumps in their derivatives at the piece boundaries. This sub-leaf names the four specific failure modes that follow from that.

## LCP and PGS in outline

A velocity-level contact LCP for a pair of bodies with relative normal velocity $v_n^-$ before impulse and $v_n^+$ after solves for the impulse magnitude $p \ge 0$ satisfying:

$$ v_n^+ = v_n^- + p / m_\text{eff}, \quad p \ge 0, \quad v_n^+ \ge 0, \quad p \cdot v_n^+ = 0 $$

— the complementarity condition says either the contact is active ($p > 0$, $v_n^+ = 0$) or it is inactive ($p = 0$, $v_n^+ > 0$). For a system of $N$ contact pairs with friction, the coupled LCP is $N$-dimensional (normal-only) or $3N$-dimensional (normal plus two tangential friction directions per pair). PGS is a fixed-point iteration that cycles through contacts one at a time, each iteration updating its own $p$ given the current estimates of the others.

## Failure 1 — Gradient jumps at the active-set boundary

The LCP solution is a piecewise-linear function of the input: within a fixed active set (which contacts are "on"), the impulses are a linear solve; across a change of active set, the linear system's size and structure change. The derivative of the output (post-impulse velocity, or subsequent position) with respect to any design parameter has a finite jump at every active-set transition. This is the same structural failure as penalty's Hessian discontinuity, restated at velocity rather than force level: the [Part 6 Ch 02 adjoint](../../60-differentiability/02-implicit-function.md) requires a single-valued derivative at the solution, and under impulse contact the derivative is single-valued only within an active set, not across the active-set transitions that optimization steps routinely cross.

## Failure 2 — Coefficient of restitution picks winners

Impulse-based contact at the velocity level does not fully determine post-collision behaviour without a Newton-impact law — the coefficient of restitution $e \in [0, 1]$ governs how much of the pre-impact normal velocity is reversed on rebound:

$$ v_n^+ = -e \, v_n^- $$

$e$ is not a property of the continuum — it is a scalar averaged over a physical phenomenon (finite-time deformation-and-recovery) that the impulse formulation has elected not to resolve. For rigid bodies in contact over milliseconds this averaging is defensible; for soft bodies where the deformation *is* the physics, collapsing it to a restitution scalar throws away the primary output. The canonical problem's [pressure-uniformity reward](../../10-physical/01-reward/00-pressure-uniformity.md) reads the deformation-and-pressure field directly; any simulator that reduces that field to $e$ at the contact instant produces a reward that is insensitive to the design parameters that actually matter.

## Failure 3 — Fragile under stacked and chained contact

PGS convergence on coupled LCPs is geometric with a rate that depends on the matrix structure — for a contact chain of length $L$ (e.g., $L$ stacked objects or a stacked sleeve pair pressed between two probes), the PGS iteration count for a given residual tolerance grows with $L$, and for dense all-pairs contact graphs it can degrade further. Production rigid-body engines ship a PGS iteration-count knob, typically 4–20, and trade convergence against frame cost; below the convergence threshold, stacked contact exhibits visible jitter, gradual settling, or drift. The [canonical problem's multi-layer regime](../04-multi-layer.md) is exactly this stacked-contact case, and it is precisely where impulse-based engines fail most visibly. Increasing the iteration count recovers convergence but loses the real-time budget; dropping it keeps the budget but admits drift.

## Failure 4 — Velocity-level resolution is wrong for the FEM state

FEM-based soft-body simulation represents state as nodal positions and (sometimes) velocities; the equilibrium equations are written in terms of positions through [backward-Euler's residual](../../50-time-integration/00-backward-euler.md). Impulse-based contact resolves at the velocity level and then integrates positions from the updated velocities, which introduces a per-step impulse–position split that FEM does not share with rigid-body dynamics. Under large timesteps, position drift accumulates because the velocity correction satisfies non-penetration at the end of the impulse and not across the whole step interior. Under small timesteps, the per-step impulse magnitude shrinks but the per-step active-set jumps (Failure 1) accumulate — halving $\Delta t$ doubles the number of active-set transitions the forward integrator passes through, so reducing the step size multiplies the discontinuity count rather than smoothing it out. There is no step size at which impulse-based contact is correct for soft bodies — only step sizes at which one or the other failure mode is less visible.

## Why "just use LCP, everything works in rigid body" is misleading

Rigid-body dynamics tolerates impulse contact because rigid bodies have no finite-element-level stress to corrupt and no continuous deformation to resolve. The active-set discontinuities produce jumps in rigid-body linear and angular velocity but do not corrupt a stress tensor that doesn't exist. For soft bodies with FEM stress per element (the primary `sim-soft` output consumed by the [reward function](../../10-physical/01-reward.md) and by the [stress-based refinement criterion in Part 7 Ch 03](../../70-sdf-pipeline/03-adaptive-refine.md)), the same active-set discontinuity becomes a stress-field discontinuity at every contact-set change. What is an acceptable jitter at the rigid-body scale is an uninterpretable reward-signal contamination at the FEM scale.

## What this sub-leaf commits the book to

- **`sim-soft` does not ship an impulse-based contact implementation.** The four failure modes above are cumulative — any one of them would disqualify impulse contact; the combination is decisive. The [Part 4 Ch 01 energy-based formulation](../01-ipc-internals/03-energy.md) resolves contact at the force/energy level, not the velocity level, and is the only formulation `sim-soft` ships.
- **Restitution $e$ is not a `sim-soft` contact parameter.** Contact recovery is emergent from the elastic, viscoelastic, and inertial energies integrated through backward-Euler; it is not a free scalar tunable on the contact pair. [Viscoelastic dissipation (Part 2 Ch 07)](../../20-materials/07-viscoelastic.md) is where energy leaves the bulk; the contact layer is conservative.
- **The LCP-is-standard-for-rigid-bodies precedent does not transfer.** When later chapters reference rigid-body engines as `sim-soft` partners (notably [`sim-mjcf`](../../110-crate/03-build-order.md) rigid contact bodies for the probe in the canonical problem), the rigid-body side uses whatever contact law MuJoCo provides; the soft-body side uses IPC; the inter-crate contact interface between them is one-way (soft reacts to rigid's prescribed kinematics) rather than symmetric.
