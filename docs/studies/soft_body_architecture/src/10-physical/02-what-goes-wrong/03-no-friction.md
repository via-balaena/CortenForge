# Missing friction

A silicone cavity pressing against a probe has real friction — silicone-on-smooth-metal runs at $\mu_c \sim 0.5$–$1.0$ depending on surface finish and contamination. A simulator in which the cavity slides off the probe under any axial load, or in which axial force is transmitted only through normal reaction (never through tangential stick), is producing a contact law that does not match the physical material. The visual signature is the cavity sliding off the probe like it is greased. The scientific signature is a frictionless or stick-free contact formulation. Same defect, two framings.

## What fails, and where it shows up

Two off-the-shelf patterns reproduce this failure on the [canonical problem](../00-canonical/00-formulation.md).

**Frictionless contact.** The contact law enforces non-penetration along the outward surface normal but applies zero tangential constraint. Any component of load along the cavity axis produces unresisted sliding motion — the cavity translates axially and leaves the probe. Common in real-time engines optimizing the common case, where friction is hand-tuned at the animation layer rather than at the physics layer.

**Ad-hoc Coulomb without a stick regime.** Tangential force is set to $\mu_c \times F_\text{normal}$, signed by the tangential velocity. At zero tangential velocity the sign is undefined, so the force switches discontinuously as the tangential velocity crosses through zero. The contact is always in kinetic friction; there is no static-friction stick state. Under axial load below $\mu_c \times F_\text{normal}$ the expected behavior is that the cavity sticks and does not slide; under the discontinuous-Coulomb law, the solver chatters around zero tangential velocity and never settles.

## Which reward terms it corrupts

- **Every reward term.** The canonical problem's force-control load mode has no steady state under frictionless contact: any force-balance configuration requires friction to hold. Under discontinuous Coulomb, the solver chatters and the reward callback reads transient values. Both failure paths make reward evaluation non-reproducible at the same design $\theta$.
- **Effective stiffness specifically.** Under force control, the effective stiffness is $k_\text{eff} = F_\text{ax}/\delta$. Without friction, axial force is transmitted only if the probe is still advancing (friction was from kinetic friction at advance). Once held static, force decays to zero and $k_\text{eff}$ drops to zero. The [stiffness-bound barrier](../01-reward/03-stiffness-bounds.md) fires incorrectly on every candidate.
- **Coverage and uniformity during transient slide.** If the cavity is in the process of sliding off the probe at the moment the reward is read, the contact footprint is a narrow transient band rather than the intended full coverage. Both reward terms read confounded values.

## How sim-soft fixes it

**[Smoothed Coulomb friction integrated with IPC](../../40-contact/02-friction.md).** The tangential force law replaces the sign-of-tangential-velocity discontinuity with a smooth transition: tangential force follows a sigmoid in tangential velocity, saturating at $\mu_c \times F_\text{normal}$ at large velocity and passing through zero at zero velocity with a finite slope. Static-friction stick emerges naturally as the finite-slope regime around zero velocity.

The smoothing is integrated with the IPC barrier ([Part 4 Ch 02](../../40-contact/02-friction.md)) so the total potential energy contribution from friction is a smooth function of $x$, which preserves the [$C^2$ total-potential-energy property](../../50-time-integration/00-backward-euler.md) the Newton loop requires. The friction coefficient $\mu_c$ is a per-contact-pair property read from the `SoftScene`'s metadata; the silicone-on-probe default is set from the [material database](../../appendices/02-material-db.md) to match measured silicone-on-metal values.

Under the smoothed law, the force-control steady state exists and is unique in the stick regime. The solver settles. The reward is reproducible.

## Why the obvious cheap fix doesn't work

**"Post-process the trajectory to stick when $\|F_\text{tangent}\| < \mu_c F_\text{normal}$.** A conditional stick-switch in post-processing is discontinuous. The solver inside the Newton loop does not know about the switch and will oscillate around the transition; the post-processing makes the output look clean but leaves the inner solver pathological. Gradient-based optimization through a conditional-switch output is not differentiable. The smoothing needs to be at the force-law level, not at the output-cleanup level.

**"Use a very high friction coefficient so the cavity never slides.** Driving $\mu_c$ well above the physical value makes stick artificially dominant and prevents slide, but at the cost of wrong reward values — a cavity designed to run at $\mu_c = 0.5$ does not optimize the same as one running at $\mu_c = 2.0$. [Sim-to-real](../../100-optimization/05-sim-to-real.md) cannot calibrate out a wrong friction value because the failure mode (slide vs. stick) is binary in the physical behaviour that the $\mu_c$ choice would pretend to control.

**"Skip force-control load mode; use only displacement control.** Displacement control under frictionless contact produces a confusing reward surface: the cavity may stay engaged at the held displacement for artificial reasons (nothing is pulling it off), but the transmitted axial force is zero because no stick exists. The stiffness-bound barrier fires on every candidate without exception. The conclusion "every design is bad" is not actionable.
