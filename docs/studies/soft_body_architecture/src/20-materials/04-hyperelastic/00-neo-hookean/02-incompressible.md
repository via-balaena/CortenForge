# Near-incompressible form

Silicone's Poisson ratio at rest is $\nu \approx 0.499$. In the Lamé pair this means $\lambda / \mu \approx 500$: the volumetric term dominates the deviatoric by three orders of magnitude, and the mesh locks under bending. The [energy-and-stress sibling](00-energy.md)'s closed-form $\psi(F) = \tfrac{\mu}{2}(I_1 - 3) - \mu \ln J + \tfrac{\lambda}{2}(\ln J)^2$ does not cure this on its own — it inherits the locking failure from the underlying Hooke-law form. This leaf writes the volumetric/deviatoric split that sets up [Ch 05's locking treatments](../../05-incompressibility.md) to consume, and states what neo-Hookean carries on its side of the split versus what Ch 05 carries on the other.

## Volumetric/deviatoric split

Factor $F$ into its volume-preserving (isochoric) part $\bar F$ and a pure-volumetric scaling:

$$ F \;=\; \bar F \cdot J^{1/3}\, I \qquad\text{with}\qquad \bar F = J^{-1/3}\, F,\ \det \bar F = 1 $$

The first invariant of the isochoric part is $\bar I_1 = \mathrm{tr}(\bar F^T \bar F) = J^{-2/3}\, I_1$ ([Part 2 Ch 01 invariants](../../01-strain-measures/03-invariants.md)).

The compressible-NH energy from the [energy-and-stress sibling](00-energy.md) does not decompose cleanly into $\psi_\text{dev}(\bar F) + \psi_\text{vol}(J)$ — the $(I_1 - 3)$ term couples $\bar F$ and $J$ through $I_1 = J^{2/3}\, \bar I_1$, and the cross-coupling is what breaks the near-incompressibility treatment in the $\lambda / \mu \to \infty$ limit. For near-incompressibility, `sim-soft` uses the *modified* neo-Hookean form, which is deviatoric-plus-volumetric by construction:

$$ \psi_\text{modified}(F) \;=\; \tfrac{\mu}{2}\,(\bar I_1 - 3) \;+\; U(J) $$

The deviatoric term depends only on $\bar F$ (since $\bar I_1$ is a function of $\bar F$ alone); the volumetric term $U(J)$ depends only on $J$. Common choices for $U(J)$ are $\tfrac{\lambda}{2}(\ln J)^2$ (matching the compressible form's volumetric barrier) or $\tfrac{\kappa}{2}(J - 1)^2$ with $\kappa$ a bulk modulus; the handoff to [Ch 05](../../05-incompressibility.md) treats either choice.

Modified-NH and compressible-NH agree to within $O(\|F - I\|^3)$ at small strain, so the [gradcheck small-strain reduction](../../../110-crate/04-testing/03-gradcheck.md) passes either form against linear elasticity at 0.1% strain. They diverge at finite strain — the modified form's clean decomposition comes at the cost of different stress predictions at moderate stretch. Which form an impl uses is declared on the struct; the book's default for standalone low-Poisson use is the compressible form (simpler closed-form tangent), and the near-incompressibility decorator wraps the modified form.

[Ch 05's mixed u-p formulation](../../05-incompressibility/01-mixed-up.md) and [F-bar method](../../05-incompressibility/02-f-bar.md) both operate on the modified form. Mixed u-p replaces $U(J)$ with a Lagrange-multiplier constraint $p\,(J - 1)$, adding $p$ as an independent unknown enforcing $J \to 1$. F-bar substitutes an element-averaged $\bar F$ for the per-Gauss-point $\bar F$ in the deviatoric term, which suppresses high-frequency locking modes without changing the volumetric treatment.

## Why this does not cure locking inside `NeoHookean`

`sim-soft` does not bake the locking cure into the `NeoHookean` impl itself, for two reasons.

**The locking fix is orthogonal to the constitutive law.** Mixed u-p and F-bar both apply to any hyperelastic law, not just neo-Hookean; baking the fix into `NeoHookean` would duplicate it across [Mooney-Rivlin](../01-mooney-rivlin.md) and [Ogden](../02-ogden.md) and force three parallel code paths that diverge as soon as the laws pick different deviatoric forms. Keeping the fix in Ch 05's machinery means one locking-fix decorator wraps each base law, not a per-law variant.

**The decorator pattern matches viscoelastic and thermal.** Ch 00 already commits to composition: [`Viscoelastic<M>`](../../00-trait-hierarchy/01-composition.md) and [`Thermal<M>`](../../00-trait-hierarchy/01-composition.md) wrap a base `M: Material`. A locking-fix decorator — whether mixed-u-p-flavored or F-bar-flavored — is the fourth composite feature. The [validity-domain descriptor](../../00-trait-hierarchy/02-validity.md)'s Poisson upper bound widens from $\nu < 0.45$ (standalone NH) to $\nu < 0.499$ (NH wrapped in the mixed-u-p decorator from Ch 05) when the composition applies.

## What `NeoHookean` provides for the Ch 05 handoff

`NeoHookean` exposes three evaluation hooks its locking-fix wrapper consumes:

- **Deviatoric energy and stress at fixed $J$.** The wrapper needs $\psi_\text{dev}(\bar F) = \tfrac{\mu}{2}(\bar I_1 - 3)$ and the corresponding Piola contribution $\mu\,(\bar F - \bar F^{-T})$, evaluated on the element's actual $\bar F$ with $J$ held at whatever value the wrapper pins it to (1 for mixed u-p, the element-average for F-bar).
- **Volumetric energy and pressure-like scalar.** The wrapper needs $\psi_\text{vol}(J)$ and its scalar derivative $\mathrm{d}\psi_\text{vol}/\mathrm{d}J$ (the scalar pressure), which mixed u-p equates to its Lagrange multiplier $p$.
- **Tangent blocks split by $(\bar F, J)$.** The [tangent sibling](01-tangent.md)'s $\mathbb{C}$ restricted to pure-deviatoric and pure-volumetric modes; Ch 05's locking-fix decorator re-assembles the element tangent from these blocks under the mixed-u-p or F-bar constraint.

These hooks are evaluation-only — no additional state on `NeoHookean`. They live as additional methods on the impl, documented in the [implementation sub-leaf](03-impl.md).

## What this sub-leaf commits the book to

- **The volumetric/deviatoric split is NH's contribution to the locking fix, not the fix itself.** The closed-form $\psi$ rearranges into deviatoric-plus-volumetric; the actual locking cure lives in [Ch 05's mixed u-p or F-bar decorator](../../05-incompressibility.md), which consumes the split.
- **Standalone `NeoHookean` declares $\nu < 0.45$ on its validity domain.** Going above $0.45$ requires the locking-fix decorator from Ch 05; the widened Poisson upper bound ($0.499$) is a property of the composition, not of standalone NH.
- **No per-law variants of the locking fix.** Mixed u-p and F-bar are constitutive-law-agnostic; the same decorators wrap Mooney-Rivlin and Ogden, and the neo-Hookean-specific evaluation hooks above have parallels on the other hyperelastic impls.
- **Near-incompressibility is a composition, not an intrinsic NH feature.** The [Ch 00 composition rules](../../00-trait-hierarchy/01-composition.md) pattern — decorator wraps base — governs here too; the locking fix is one more decorator.
