# Fiber-reinforced models

The [Ch 06 parent](../06-anisotropic.md) commits to anisotropy as a decorator over the isotropic hyperelastic family from [Ch 04](../04-hyperelastic.md), not as a new constitutive family. This leaf writes the general fiber-reinforced form — the structural template that [HGO](01-hgo.md) and other specific laws fill in — with the additive energy split, the $I_4$ kinematic input, and the per-element data flow that the [`Anisotropic<M, R>` decorator](../00-trait-hierarchy/01-composition.md) implements.

## Additive energy split

A fiber-reinforced hyperelastic energy decomposes additively over an isotropic base:

$$ \psi(F, a) \;=\; \psi_\text{iso}(\bar I_1, \bar I_2, J) \;+\; \psi_\text{fiber}(I_4) $$

where $\psi_\text{iso}$ is the base law's isotropic energy (neo-Hookean, Mooney-Rivlin, Ogden), $\psi_\text{fiber}$ is a function of the [fiber invariant $I_4 = a \cdot C\, a$](../01-strain-measures/03-invariants.md), and $a$ is the unit fiber direction in the reference configuration. The split is what makes the decorator pattern pay: the base impl evaluates $\psi_\text{iso}$ unchanged, and `Anisotropic<M, R>` adds $\psi_\text{fiber}$ on top.

Three structural requirements pin down what $\psi_\text{fiber}$ can look like:

1. **Vanishes at the reference state.** At $I_4 = 1$ (no fiber stretch), $\psi_\text{fiber} = 0$ so the undeformed configuration is energy-free. Standard forms factor a $(I_4 - 1)^2$ multiplier (or a higher even power) to enforce this directly.
2. **Stress-free reference, no corrector needed.** The fiber stress contribution $P_\text{fiber} = \partial \psi_\text{fiber} / \partial F$ vanishes at $I_4 = 1$ as well — any energy form that vanishes at-least-quadratically at $I_4 = 1$ has $\psi'_\text{fiber}(1) = 0$, so the stress vanishes structurally without a $-c \ln J$-style additive corrector of the kind [NH](../04-hyperelastic/00-neo-hookean/00-energy.md), [MR](../04-hyperelastic/01-mooney-rivlin/00-two-param.md), and [Ogden](../04-hyperelastic/02-ogden/00-n-term.md) require on their volumetric terms.
3. **Convex in $I_4$, with stiffening growth.** Real fibers stiffen under stretch — collagen, glass, carbon, and embedded textile reinforcements all show super-linear stress at increasing stretch. $\psi_\text{fiber}(I_4)$ is convex in $I_4$ with growth that exceeds the isotropic base's at large $I_4$.

The first Piola contribution from the fiber term is:

$$ P_\text{fiber} \;=\; 2\, \psi'_\text{fiber}(I_4)\, F\, a\, a^T $$

where $\psi'_\text{fiber}(I_4) = \partial \psi_\text{fiber} / \partial I_4$. The $F\, a\, a^T$ rank-one tensor encodes the directionality: fiber stress acts along the deformed fiber direction $F a$ alone.

## Tension-only fibers

Most fiber-reinforced impls treat fibers as load-bearing in tension only. Under compression ($I_4 < 1$), fibers buckle out of plane and carry negligible stress; the energy is set to zero for $I_4 < 1$:

$$ \psi_\text{fiber}(I_4) = \begin{cases} \text{the active form} & I_4 \geq 1 \\ 0 & I_4 < 1 \end{cases} $$

The discontinuity in $\partial^2 \psi / \partial I_4^2$ at $I_4 = 1$ is a known nuisance for the [Newton tangent](../../50-time-integration/00-backward-euler.md): the stiffness drops from finite-positive to zero across the threshold, and Newton iterations near the boundary can chatter. `sim-soft`'s impls smooth the switch over a narrow band around $I_4 = 1$ with a $C^2$ bridging polynomial, restoring a continuous tangent without distorting the qualitative tension-only behaviour.

## Two-fiber families

Materials with two distinct fiber families — woven textile sleeves, helically reinforced silicone tubes, biological vessel walls — carry two fiber directions $a^{(1)}, a^{(2)}$ at each material point with two independent invariants:

$$ I_4^{(1)} = a^{(1)} \cdot C\, a^{(1)}, \qquad I_4^{(2)} = a^{(2)} \cdot C\, a^{(2)} $$

The energy split adds two fiber terms:

$$ \psi(F, a^{(1)}, a^{(2)}) = \psi_\text{iso}(\bar I_1, \bar I_2, J) + \psi_\text{fiber}(I_4^{(1)}) + \psi_\text{fiber}(I_4^{(2)}) $$

The cross-invariant $I_8 = a^{(1)} \cdot C\, a^{(2)}$ that captures shear coupling between fiber families is not part of the standard form. For the canonical problem's typical reinforcement patterns (single-direction sleeve, $\pm 45°$ braid), the no-coupling form is sufficient; biomechanics applications that require the coupling are an out-of-scope extension.

The resolver `R: FiberDirection` returns either a single unit vector or a pair; the energy function and stress evaluation branch on the resolver type at compile time, so single-fiber and two-fiber materials share the decorator without runtime branching.

## Per-element data and trait surface

The fiber direction $a$ is a per-element value sampled from a [spatial fiber direction field](02-direction-fields.md) at element construction time and stored alongside the per-element material descriptor in [`element/`](../../110-crate/00-module-layout/01-element.md). The `Anisotropic<M, R>` decorator does not store $a$ on its own state — the resolver `R` is a function from reference-frame position to unit vector, evaluated once per element when the mesh is built or warm-restarted from a [re-mesh](../../70-sdf-pipeline/04-live-remesh.md).

The per-element block size is unchanged from the isotropic case — $12 \times 12$ for [Tet4](../../30-discretization/00-element-choice/00-tet4.md), $30 \times 30$ for [Tet10](../../30-discretization/00-element-choice/01-tet10.md). Anisotropy modifies the constitutive evaluation per Gauss point, not the solver topology.

## Validity-domain widening

`Anisotropic<M, R>` widens the [composed `ValidityDomain`](../00-trait-hierarchy/02-validity.md)'s anisotropy descriptor from "isotropic only" to "single-direction" or "two-fiber-family" depending on the resolver. Other validity slots — Poisson cap from the base, strain-rate range, temperature range — are inherited unchanged. Stacking anisotropy with other decorators (incompressibility, viscoelasticity, thermal coupling) follows the [Ch 00 composition rules](../00-trait-hierarchy/01-composition.md); anisotropy itself does not relax or tighten the base's other validity slots.

## What this sub-leaf commits the book to

- **Fiber-reinforced energy is an additive term over the isotropic base.** $\psi = \psi_\text{iso} + \psi_\text{fiber}(I_4)$, with `Anisotropic<M, R>` the decorator wrapping any `M: Material`. The base's evaluation is untouched; the decorator adds $\psi_\text{fiber}$ on top of the base's `first_piola` and `tangent` outputs.
- **Stress-free reference is structural, not corrective.** Any energy form that vanishes at-least-quadratically at $I_4 = 1$ produces a stress that vanishes there too, no $-c \ln J$-style additive corrector required.
- **Tension-only with smoothed switch.** Fibers carry load for $I_4 \geq 1$; a narrow band around $I_4 = 1$ uses a $C^2$ bridging polynomial to keep the Newton tangent continuous.
- **Two-fiber families compose by adding two $I_4^{(i)}$ terms.** No cross-fiber-family $I_8$ coupling in the standard form. The resolver `R` carries either one direction or two; compile-time branching selects the form.
- **Per-element block size is unchanged.** Anisotropy lives in the constitutive evaluation, not the solver topology — Tet4 and Tet10 blocks both carry over from the isotropic case.
