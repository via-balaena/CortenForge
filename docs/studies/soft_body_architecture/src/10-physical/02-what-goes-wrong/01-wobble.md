# Wobble and unphysical jelly feel

Real silicone has internal friction — when you push on a silicone part and let go, oscillations decay within a small number of cycles and the part settles. A simulator whose silicone keeps oscillating after the load is removed, or whose steady state never settles because oscillations never decay, is producing a material that does not exist. The visual signature is a sleeve that jiggles after the probe is held still. The scientific signature is a constitutive model with no viscoelastic spectrum and/or an integrator that admits non-physical energy retention. Same defect, two framings.

## What fails, and where it shows up

Three off-the-shelf solver classes reproduce this failure on the [canonical problem](../00-canonical/00-formulation.md).

**Position-based dynamics and mass-spring networks.** The constitutive law — such as it is — is energy-conservative. A PBD constraint projects positions without dissipating kinetic energy, and a mass-spring network's internal oscillation modes have no dissipation at their natural frequencies. The result is a sleeve whose post-load vibrations persist indefinitely, attenuated only by the numerical damping inadvertently introduced by the integrator.

**Under-damped explicit integrators.** Even with a viscous term in the constitutive law, symplectic or semi-implicit integrators (leapfrog, RK4, symplectic Euler) at a timestep tuned for stability but not accuracy amplify the natural frequencies rather than damping them. The wobble signature looks like genuine material oscillation but is numerical.

**Elastic-only FEM with no viscoelasticity.** Linear-elastic or hyperelastic without a Prony series has zero damping. Under transient loading the sleeve rings at its natural frequencies forever — or until the integrator's numerical damping incidentally absorbs the energy. Either case, the decay rate is not the silicone's decay rate.

## Which reward terms it corrupts

- **Time-to-equilibrium.** The [reward terms](../01-reward.md) are defined on the steady-state solution. If no steady state exists, every reward reading is a transient sample that depends on when the reward callback ran.
- **Uniformity, coverage, peak, stiffness — all four.** All four read fields that are oscillating if the solver hasn't settled; the optimizer sees noisy rewards that confound gradient estimation ([Part 6 Ch 02 IFT](../../60-differentiability/02-implicit-function.md) assumes a unique $x^\ast$).
- **Sim-to-real agreement.** A printed silicone part's dynamic response — force-decay curve after a pulsed load — is set by its viscoelastic spectrum. A simulator with wrong decay rate can't be calibrated to measured physical response without changing a parameter it doesn't have, so the [sim-to-real correction](../../100-optimization/05-sim-to-real.md) has no handle.

## How sim-soft fixes it

Two commitments together.

- **[Prony-series viscoelasticity calibrated to DMA data](../../20-materials/07-viscoelastic.md).** The constitutive law carries a discrete relaxation spectrum $\{(\tau_i, g_i)\}$ fit to storage-and-loss-modulus data from dynamic mechanical analysis on each base silicone. For Ecoflex grades this is measured and lives in the [viscoelastic-spectrum leaf of Part 1 Ch 04](../04-material-data/00-ecoflex/02-viscoelastic.md); the solver reads the Prony coefficients from the `MaterialField`.
- **[Backward Euler implicit time integration](../../50-time-integration/00-backward-euler.md).** Unconditionally stable for any timestep; energy dissipation in the integrator is controlled by the discretization, not by the solver blowing up at too large a timestep. Under a transient load, the sleeve damps at the physical Prony-series rate rather than at a numerical-damping-driven rate.

The combination is load-bearing. Prony viscoelasticity without implicit integration is still subject to integrator-induced numerical damping on top of the physical damping; implicit integration without viscoelasticity has no physical damping in the constitutive law at all. Both together — real damping from the material, implicit integration to preserve it — reproduce the measured decay rate.

## Why the obvious cheap fix doesn't work

**"Just add Rayleigh damping (mass- and stiffness-proportional).** Rayleigh damping is a mathematical convenience: $C = \alpha_M M + \alpha_K K$ with two scalar dials. It damps every mode at a rate set by the scalars rather than by a frequency-dependent spectrum — the damping curve has the Rayleigh formula's $\alpha_M \omega + \alpha_K / \omega$ shape, which is fixed by two parameters and cannot match silicone's measured loss-tangent curve across the operating frequency decades. Rayleigh makes the wobble go away but over-damps fast transients and under-damps slow ones relative to a Prony-fit to the DMA data. Prony series is the spectrum-shaped fix.

**"Just lower the explicit timestep.** Reducing $\Delta t$ on an explicit integrator reduces the numerical-damping contribution but does not add physical damping. Enough $\Delta t$ reduction makes an elastic-only FEM ring forever at its natural frequencies instead of gradually decaying to numerical damping. The fix is the constitutive law, not the integrator alone.

**"Call the final positions the steady state at a fixed wall-clock time budget."** If the wobble hasn't decayed by then, the reward is sampling a transient. Optimizer gradients read through a transient are biased; the optimization converges to the wrong optimum. The fix requires that a steady state exists and that the solver reaches it.
