# Popping and snap-through

A probe separating from a cavity should produce a contact force that decays smoothly toward zero as the gap opens; the pressure field diminishes continuously across the contact band. A simulator in which the probe separation instead produces a discontinuous force drop, a visible snap-through of the rim, or an oscillation triggered by contact state changes is producing a non-physical contact response. The visual signature is a character's hand clipping through a surface and then popping out. The scientific signature is a contact formulation that admits overlap energy and releases it non-smoothly, or a formulation whose total potential energy is not continuous across the contact/no-contact boundary. Same defect, two framings.

## What fails, and where it shows up

Three off-the-shelf solver classes reproduce this failure on the [canonical problem](../00-canonical/00-formulation.md).

**Penalty contact.** The contact force is proportional to overlap depth — $F_\text{pen} = \kappa_\text{pen} \max(0, -d)$ for gap $d$ and stiffness $\kappa_\text{pen}$. Energy is stored proportional to overlap squared. When the bodies separate, the stored energy is released over the timesteps during which overlap reduces to zero; depending on the stiffness–mass–timestep triple, this looks like a ringing oscillation at contact, or a single-step pop as the overlap discretely jumps out. Neither is physical.

**SDF-fallback contact.** Contact is resolved by projecting penetrating points back to the nearest surface of the partner body in a single step. The displacement-correction step releases all accumulated elastic strain energy at once — visible as a snap-through animation artefact. At the next step the bodies are no longer penetrating, which often means they now have a gap too large to be in contact, triggering a new oscillation cycle.

**Impulse-based contact.** Momentum-transfer at contact events — common in rigid-body engines ported to soft bodies — resolves a contact as a finite impulse applied over one step. The energy transferred is not continuous across the contact boundary; small velocity changes at the contact instant produce finite energy changes. Under bodies moving slowly into and out of contact, the impulse ping-pongs and snap-through artefacts emerge.

## Which reward terms it corrupts

- **Contact coverage.** Spurious loss-of-contact during separation oscillations reduces the contact indicator across multiple steps; the [coverage term](../01-reward/01-coverage.md) reads transient minima as the steady-state coverage, biasing the reward downward at precisely the configurations where the solver is unstable.
- **Pressure uniformity.** Instantaneous pressure spikes at the pop event are localized where the penalty or impulse discharge concentrates. The [uniformity term](../01-reward/00-pressure-uniformity.md) reports non-physical spread around the pop event.
- **Effective stiffness.** Erratic transmitted force doesn't correlate cleanly with held displacement. The [stiffness-bound barrier](../01-reward/03-stiffness-bounds.md) fires and clears depending on when the reward is sampled.
- **Gradient path.** Non-smooth contact forces are not differentiable; the [IFT adjoint](../../60-differentiability/02-implicit-function.md) requires $\partial r/\partial x$ to exist at the converged $x^\ast$, which fails at pop transitions. Gradient-based optimization becomes unavailable. This is in addition to the reward-corruption, not instead of.

## How sim-soft fixes it

One commitment carries the whole failure.

**[IPC barrier with adaptive stiffness and tolerance](../../40-contact/01-ipc-internals/00-barrier.md).** The contact potential $b(d) = -(d - \hat d)^2 \log(d/\hat d)$ is $C^2$, vanishes at $d = \hat d$, and diverges at $d = 0$. Non-penetration is guaranteed because any configuration with $d \le 0$ has infinite energy and is inaccessible to the Newton solver. Contact release is smooth by construction — the barrier decays toward zero smoothly as the gap grows through the tolerance band $[0, \hat d]$. No energy is stored in overlap because no overlap exists.

The barrier's $C^2$ property means the total potential energy $U(x) = \Psi_\text{elastic}(x) + b(d(x)) + \Psi_\text{inertia}(x)$ is smooth in $x$ wherever the solver visits. [Newton-on-total-potential-energy](../../50-time-integration/00-backward-euler.md) converges without discontinuity at contact transitions; the adjoint for [Part 6 Ch 02 IFT](../../60-differentiability/02-implicit-function.md) is well-defined at $x^\ast$; the reward is differentiable.

Adaptive stiffness ([Part 4 Ch 01](../../40-contact/01-ipc-internals/01-adaptive-kappa.md)) and adaptive tolerance ([Part 4 Ch 05](../../40-contact/05-real-time/01-barrier-width.md)) together keep the barrier tight enough to resolve fine contact features and loose enough not to dominate the Newton condition number, without manual tuning per scene.

## Why the obvious cheap fix doesn't work

**"Turn penalty stiffness up and timestep down until it stops popping.** Increasing $\kappa_\text{pen}$ amplifies the discontinuity at separation and worsens the pop. Reducing $\Delta t$ eventually sub-samples the pop enough to average it out visually, but adds compute cost without changing the underlying defect — the energy storage in overlap is still there, just resolved at finer granularity. At infinite stiffness and infinitesimal timestep, penalty converges to IPC; at any finite stiffness it does not.

**"Add a velocity-dependent damper at contact transitions.** Damping tangential or normal velocity at separation absorbs the impulse but introduces a dissipation mechanism that is not physical — the real silicone damping comes from [viscoelasticity](../../20-materials/07-viscoelastic.md) in the bulk, not from contact. A velocity-damper hides the pop but corrupts the force-decay curve, biasing [sim-to-real](../../100-optimization/05-sim-to-real.md) calibration.

**"Live with the artefact; it's small at high resolution."** The artefact is not small at the reward's scale. A single pop in the contact-pressure field produces a spike that dominates the uniformity variance for that step. The reward sampled at a popping step is unusable even for a coarse gradient-free optimizer.
