# Standard Coulomb and its limits

[Part 1 Ch 02's missing-friction failure](../../10-physical/02-what-goes-wrong/03-no-friction.md) named the physical phenomenon: silicone-on-probe at $\mu_c \sim 0.5$–$1.0$ has a stick regime that a frictionless or ad-hoc-Coulomb formulation does not reproduce, and the canonical problem has no steady state without it. This sub-leaf is the mathematical statement of that physics — what the two regimes of Coulomb friction actually say, and where that statement becomes a non-differentiable set-valued map the Newton solver cannot consume directly. The [next sub-leaf](01-smoothed.md) replaces the set-valued transition with a smooth function; understanding what it is replacing, and why the replacement is non-trivial, is the purpose of this sub-leaf.

## The two-regime law

Let $n$ be the normal force magnitude at a contact pair, $t$ the tangential force vector, and $\dot u_T$ the relative tangential velocity at the contact point. The Coulomb law is a pair of conditional statements:

$$ \text{stick: } \dot u_T = 0 \text{ and } \|t\| \le \mu_c |n| $$

$$ \text{slip: } \dot u_T \ne 0 \text{ and } t = -\mu_c |n| \, \dot u_T / \|\dot u_T\| $$

In stick, the tangential velocity is zero and the tangential force is free to take any value inside a friction cone of radius $\mu_c |n|$ — whatever is needed to hold the velocity at zero against the applied tangent load. In slip, the tangential velocity is non-zero and the tangential force is determined: it points opposite the velocity and has magnitude exactly $\mu_c |n|$.

The coefficient $\mu_c$ is a per-contact-pair material property; $\mu_c \approx 0.5$ is representative of silicone-on-smooth-metal, and the [material database](../../appendices/02-material-db.md) fixes the value per pair. Dry friction with static and kinetic coefficients can be handled by using a larger $\mu_s$ in the stick regime and $\mu_c < \mu_s$ in slip, but the same regime structure applies; `sim-soft` ships with $\mu_s = \mu_c$ as the default and treats the distinction as a later-layer refinement.

## Why the transition is not a function

At $\dot u_T = 0$, the slip-regime direction $\dot u_T / \|\dot u_T\|$ is undefined (0/0). The law is not "the force is discontinuous at $\dot u_T = 0$" — it is "the force is a set-valued map at $\dot u_T = 0$". Every vector in the closed ball of radius $\mu_c|n|$ is a valid value of $t$ at the stick point; the law selects which one is physically realized by whatever dynamics is surrounding the contact. There is no single-valued $t(\dot u_T)$ function that covers both regimes.

A common simplification writes $t = -\mu_c |n| \, \text{sign}(\dot u_T)$ unconditionally. This collapses the stick regime into the single point $\dot u_T = 0$ with $t = 0$, and the tangential force jumps discontinuously as $\dot u_T$ crosses through zero — from $-\mu_c|n|$ to $+\mu_c|n|$ (signs per component). This is the "ad-hoc Coulomb without a stick regime" failure from Part 1 Ch 02: the sleeve under axial load below $\mu_c|n|$ has no stick equilibrium, so the solver chatters across $\dot u_T = 0$ and never settles.

## Why Newton cannot consume it directly

The [backward-Euler Newton loop](../../50-time-integration/00-backward-euler.md) iterates on a Hessian $H$ derived from the total potential's second derivative. At a stick point, the physical stiffness along the tangent direction is infinite (any displacement from the stick configuration is instantly resisted up to the friction-cone bound); at a slip point, the physical stiffness along $\dot u_T$ is zero (the force is constant at $\mu_c|n|$ regardless of velocity magnitude). The Hessian contribution from friction therefore has a jump of infinite magnitude across the stick–slip boundary.

A Newton iterate in the stick regime would assemble $H$ with an infinite tangent-direction stiffness (in practice, a large regularizer), take a step that lands in the slip regime, re-assemble $H$ with zero tangent-direction stiffness, take a step back across the boundary, and oscillate. Line search cannot rescue this because the two Hessians describe two different local optimization problems; no step size along either direction can reconcile them. The same Hessian-discontinuity pattern as the [penalty failure](../00-why-ipc/00-penalty.md), in a different coordinate.

This is the decisive reason Coulomb cannot be used directly. It is not "Coulomb is the wrong physics" — it is precisely the physics the [missing-friction failure](../../10-physical/02-what-goes-wrong/03-no-friction.md) demands — but as written the law is set-valued at one point, and every iterative solver that needs a Jacobian has no defined Jacobian at that point. The fix is not to remove the stick regime (that was the ad-hoc Coulomb failure); it is to replace the set-valued point with a smoothed approximation whose Jacobian is well-defined everywhere.

## What this sub-leaf commits the book to

- **The physics target is two-regime Coulomb, not a one-regime approximation.** `sim-soft` preserves both stick and slip behaviour; the [next sub-leaf](01-smoothed.md)'s smoothing is an approximation to this two-regime law, not a replacement that collapses the regimes.
- **The set-valued stick point is the well-defined mathematical obstacle.** Failure diagnoses that blame Coulomb for being "discontinuous" are imprecise — the force is set-valued at one point, not discontinuous. The replacement strategy follows from the set-valued structure: a smooth single-valued map that agrees with slip asymptotically and resolves the stick set by making the transition zone smooth but finite-width.
- **Per-contact-pair friction coefficient, not per-material.** $\mu_c$ is a property of the pair (pair-material × pair-material), not of either body alone. The [Part 4 Ch 04 multi-layer](../04-multi-layer.md) sub-leaves on sliding interfaces and the [canonical-problem sleeve-probe interface](../../10-physical/00-canonical/00-formulation.md) both read $\mu_c$ from per-pair metadata on the `SoftScene`.
