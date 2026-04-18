# Dissipative heating

The [Ch 08 parent's Claim 2](../08-thermal-coupling.md) closes the thermal-coupling loop: viscoelastic hysteresis converts mechanical work to heat at every timestep, and that heat must flow back into [`sim-thermostat`](../../110-crate/02-coupling/01-thermostat.md)'s temperature field for the model to stay physically self-consistent over long horizons. This leaf writes the per-element dissipation rate, the handoff to `sim-thermostat`, and the opt-in for one-way thermal in cold-regime scenes.

## Per-mode dissipation rate

The mechanical work converted to heat per unit reference volume per unit time, for a viscoelastic material with Prony hereditary stresses $\{Q_i\}$ from the [Prony sibling](../07-viscoelastic/00-prony.md), is the sum of per-mode dissipations:

$$ \dot q = \sum_{i=1}^{N} \dot q_i, \qquad \dot q_i = \frac{Q_i : Q_i}{G_i\, \tau_i} $$

with $G_i = g_i\, \mu_\text{base}$ the over-stress shear modulus for mode $i$ and $\tau_i$ its relaxation time. The product $G_i\, \tau_i$ is the effective dashpot viscosity for mode $i$; the formula is the standard tensor generalization of "dissipation = stress² / viscosity" for each Maxwell branch.

The expression is non-negative term-by-term ($Q_i : Q_i \geq 0$, $G_i > 0$, $\tau_i > 0$), so dissipative heating is always a non-negative source in `sim-thermostat`'s temperature equation, consistent with the second-law-of-thermodynamics requirement that mechanical work converts to heat in the dissipative direction only.

For the [Oldroyd-B variant](../07-viscoelastic/01-oldroyd.md), the same per-mode formula applies — the over-stress tensor $Q_i$ has a different time evolution but is still the tensor whose dissipative contribution is $(Q_i : Q_i) / (G_i\, \tau_i)$, with corrections from the upper-convected-derivative source term named in the validity-domain handoff below.

## Handoff to `sim-thermostat`

The handoff happens at each Newton-converged timestep. After the per-element converged $\{Q_i^{n+1, *}\}$ are committed to the per-element history block ([Prony sibling](../07-viscoelastic/00-prony.md)), the [`Thermal<M>` decorator](../00-trait-hierarchy/01-composition.md) computes $\dot q$ per element from the formula above and reports it to `sim-thermostat` via the [coupling boundary](../../110-crate/02-coupling/01-thermostat.md). The reported per-element heat source enters `sim-thermostat`'s temperature update at the next thermostat substep; the temperature change feeds back through the [temperature-dependent modulus sibling](00-modulus-T.md) and the [thermal expansion sibling](01-expansion.md) at the next mechanical timestep, closing the bidirectional loop.

The reporting is per-element, not per-Gauss-point: dissipative heat is a per-volume scalar that distributes uniformly across the element. For Tet10 with multiple Gauss points, the per-Gauss-point dissipations are summed (weighted by Gauss weights) to produce the per-element source.

## One-way thermal opt-in

Per the [Ch 08 parent's Claim 2](../08-thermal-coupling.md), some scenes do not need the bidirectional coupling: short transient runs where dissipative heating is provably small relative to the simulation horizon, cold-regime simulations where ambient cooling dominates, regression tests where temperature should be held fixed for reproducibility. These cases opt out of the heating handoff via a construction-time choice on the decorator.

When `Thermal<M>` is constructed in bidirectional mode (the default), the decorator queries $T$ from `sim-thermostat` and reports $\dot q$ back at each converged step. When constructed in one-way mode, the decorator queries $T$ but skips the heating-out call: the thermostat sees no input from this decorator, and the temperature field evolves only from boundary conditions and other heat sources. The struct shape locked by the [Ch 00 composition leaf](../00-trait-hierarchy/01-composition.md) is unchanged — the bidirectional/one-way choice is an impl-detail (a constructor variant, a private field, or a thermostat handle that ignores writes) that does not leak into the trait surface.

This is the right setting for the [canonical problem](../../10-physical/00-canonical.md)'s short-transient analyses where the heating is below the thermostat's resolution, and the default for [regression-test scenes](../../110-crate/04-testing/01-regression.md) that need deterministic temperature trajectories.

## Validity-domain implications

The dissipation formula assumes Prony's small-strain-linearity validity range — at very large strain the Maxwell-branch dissipation model loses accuracy, and the [Oldroyd-B variant](../07-viscoelastic/01-oldroyd.md)'s dissipation involves additional terms from the upper-convected-derivative source that are not in the simple per-mode form above. For the Prony default, the dissipation rate inherits the small-strain validity ceiling of the Prony parametrization itself; above that, the bidirectional coupling switches to the Oldroyd-B dissipation form, and the noisy-gradient flag from the Weissenberg-number guard propagates through to the heating handoff.

## What this sub-leaf commits the book to

- **Per-mode dissipation $\dot q_i = (Q_i : Q_i) / (G_i\, \tau_i)$.** Standard "stress² / viscosity" tensor generalization; non-negative term-by-term, second-law-consistent.
- **Per-element handoff at Newton convergence.** $\dot q$ is summed over Gauss points (Tet10) or evaluated at the single point (Tet4); reported once per element per converged step via [`sim-thermostat`'s coupling boundary](../../110-crate/02-coupling/01-thermostat.md).
- **Bidirectional coupling is the default; one-way thermal opt-in via a construction-time choice.** Short-transient and regression-test scenes can opt out without modifying the [Ch 00-locked struct shape](../00-trait-hierarchy/01-composition.md).
- **The handoff lives on `Thermal<M>`, not inside the viscoelastic decorator.** Per [Ch 00 composition](../00-trait-hierarchy/01-composition.md): viscoelasticity owns the per-element history; thermal coupling owns the cross-system handoff. The two compose via the locked outside-in stacking.
- **Oldroyd-B dissipation requires UCD-source corrections.** The simple per-mode form is Prony-only; large-deformation regimes use the Oldroyd-B-specific dissipation form (out of this leaf's scope, named in the validity-domain handoff).
