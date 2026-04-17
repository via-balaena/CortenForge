# Two-parameter form

The [Mooney-Rivlin branch parent](../01-mooney-rivlin.md) named the $(C_{10}, C_{01})$ two-parameter closed-form law and its place between [neo-Hookean](../00-neo-hookean.md) and [Ogden](../02-ogden.md). This leaf writes the compressible energy density $\psi(F)$ down, derives the first Piola stress $P = \partial \psi / \partial F$ term by term, names the tangent structure $\partial P / \partial F$, and states the evaluation path `sim-soft`'s `impl Material for MooneyRivlin` takes. The [calibration sibling](01-calibration.md) and [tradeoffs sibling](02-tradeoffs.md) cover the $(C_{10}, C_{01})$ fit and when the extra parameter is worth paying for.

## Energy density

$$ \psi(F) \;=\; C_{10}\,(I_1 - 3) \;+\; C_{01}\,(I_2 - 3) \;-\; (2C_{10} + 4C_{01}) \ln J \;+\; \tfrac{\lambda}{2}\,(\ln J)^2 $$

with:

- $I_1 = \mathrm{tr}(F^T F)$, $I_2 = \tfrac{1}{2}\,(I_1^2 - \mathrm{tr}((F^T F)^2))$ — first and second invariants of $C = F^T F$ ([Part 2 Ch 01 invariants](../../01-strain-measures/03-invariants.md))
- $J = \det F$ — volume ratio ([Part 2 Ch 01 deformation gradient](../../01-strain-measures/00-F.md))
- $C_{10}, C_{01}$ — Mooney-Rivlin coefficients, both with dimensions of pressure
- $\lambda$ — volumetric modulus for the standalone compressible form, carried over from the Lamé pair so that the small-strain bulk modulus matches [Ch 02 linear](../../02-linear.md)

The form is a standard compressible Mooney-Rivlin construction: two invariant-dependent deviatoric-looking terms plus a $-(2C_{10} + 4C_{01}) \ln J$ corrector plus a $(\ln J)^2$ volumetric barrier. The $-(2C_{10} + 4C_{01}) \ln J$ piece exists only to cancel the stress at the reference configuration — its Piola contribution at $F = I$ is $-(2C_{10} + 4C_{01})\, I$, which exactly offsets the $2C_{10}\,I$ and $4C_{01}\,I$ contributions from the $I_1$ and $I_2$ terms (the derivation below makes this explicit). The $(\ln J)^2$ barrier is the volumetric term, same shape as [neo-Hookean's](../00-neo-hookean/00-energy.md) and same divergence as $J \to 0^+$ or $J \to \infty$.

Small-strain reduction: at $F = I + \epsilon\,G$ with $\epsilon \ll 1$, $\psi$ reduces to the isotropic Lamé form $\mu\,\varepsilon : \varepsilon + \tfrac{\lambda}{2}\,(\mathrm{tr}\,\varepsilon)^2$ with $\mu = 2(C_{10} + C_{01})$. This is the linkage [Ch 02 linear](../../02-linear.md) leans on for the [gradcheck small-strain reduction](../../../110-crate/04-testing/03-gradcheck.md) — the impl matches linear elasticity to six digits at 0.1% strain provided $C_{10} + C_{01}$ matches $\mu/2$ for the target material.

Rotation invariance is structural. Under $F \mapsto RF$ with $R \in SO(3)$: $I_1$ is unchanged because $R^T R = I$, $I_2$ is unchanged for the same reason, and $J$ is unchanged because $\det R = 1$. Every term of $\psi(F)$ is therefore invariant, and the [fictitious-rotation-stress failure mode](../../02-linear.md) Ch 02 predicts cannot arise.

## First Piola stress

Differentiating $\psi$ term by term:

$$ P(F) \;=\; 2C_{10}\, F \;+\; 2C_{01}\,\left(I_1\, F - F\, C\right) \;-\; (2C_{10} + 4C_{01})\, F^{-T} \;+\; \lambda\,(\ln J)\, F^{-T} $$

using $\partial I_1/\partial F = 2F$, $\partial I_2/\partial F = 2(I_1\, F - F\, C)$ with $C = F^T F$, $\partial \ln J / \partial F = F^{-T}$, and $\partial (\ln J)^2 / \partial F = 2(\ln J)\, F^{-T}$. The second identity is the one the neo-Hookean derivation does not use — it comes from writing $I_2 = \tfrac{1}{2}(I_1^2 - \mathrm{tr}(C^2))$ and differentiating the $\mathrm{tr}(C^2)$ piece against $F$.

At $F = I$: $F = I$, $I_1 = 3$, $C = I$, $F^{-T} = I$, $\ln J = 0$, so

$$ P(I) \;=\; 2C_{10}\, I \;+\; 2C_{01}(3I - I) \;-\; (2C_{10} + 4C_{01})\, I \;+\; 0 \;=\; 0 $$

which is the reason the $-(2C_{10} + 4C_{01}) \ln J$ corrector is in the energy. Remove it and the reference configuration becomes pre-stressed, which would break the [Phase B gradcheck's](../../../110-crate/04-testing/03-gradcheck.md) zero-residual assertion at $F = I$.

Cauchy stress is derived from $P$ via $\sigma = P F^T / J$; the base [`Material` trait surface](../../00-trait-hierarchy/00-trait-surface.md) returns $P$ only, and downstream callers convert if they need Cauchy ([Part 9 Ch 04 thermal visualization](../../../90-visual/04-thermal-viz.md)).

## Tangent structure

Differentiating $P(F)$ a second time gives the fourth-order tangent $\mathbb{C}_{ijkl} = \partial P_{ij} / \partial F_{kl}$, which neo-Hookean's [tangent sub-leaf](../00-neo-hookean/01-tangent.md) wrote out as a sum of three index-indexed tensor products. The Mooney-Rivlin tangent carries the same three contributions from the $I_1$ and $\ln J$ derivatives plus two extra contributions from the $I_2$ piece:

- $2C_{10}\,\delta_{ik}\,\delta_{jl}$ — the shear-like identity-on-$F$ block, analogous to the $\mu\,\delta_{ik}\,\delta_{jl}$ block in neo-Hookean
- A pair of index-contracted products from $\partial (I_1 F - F C) / \partial F$ — one involving $F$ itself, one involving $F\, F^T$, both scaled by $2C_{01}$ and computed once per Gauss point
- The two $F^{-T}$-swapped and $F^{-T}$-direct blocks from the $\ln J$ and $(\ln J)^2$ derivatives, structurally identical to neo-Hookean's with coefficients $(2C_{10} + 4C_{01} - \lambda \ln J)$ and $\lambda$ respectively

The tangent is symmetric by hyperelastic construction (major symmetry $\mathbb{C}_{ijkl} = \mathbb{C}_{klij}$), same argument as the neo-Hookean tangent: $P$ derives from a scalar energy, Clairaut. The Newton system's per-element stiffness block stays a symmetric $12\times 12$ (Tet4, 1 Gauss point) or $30\times 30$ (Tet10, 4 Gauss points) — identical sparse-assembly shape to neo-Hookean ([Part 3 Ch 00 element choice](../../../30-discretization/00-element-choice.md)), so the [factor-on-tape Cholesky path](../../../50-time-integration/00-backward-euler.md) carries over without change.

## Evaluation path in `sim-soft`

Per Gauss point, `impl Material for MooneyRivlin` evaluates:

- $I_1 = \mathrm{tr}(F^T F) = \|F\|_F^2$ — Frobenius norm squared, same as neo-Hookean, no $C$ materialization
- $I_2 = \tfrac{1}{2}(I_1^2 - \|F^T F\|_F^2)$ — one extra Frobenius-norm-squared over $C$'s entries; $C = F^T F$ is *not* materialized as a $3\times 3$, only its Frobenius norm squared is needed
- $F\, C = F\, F^T\, F$ — one triple product, needed for $P$'s $I_2$ contribution; caches to the `EvaluationScratch` so the tangent re-uses it
- $F^{-T}$, $\ln J$ — shared with neo-Hookean, same caching pattern

The net cost over neo-Hookean is one extra Frobenius-norm-squared evaluation plus one $3\times 3$ triple product per Gauss point. Small-strain path uses the same cancellation-safe $I_1 - 3 = 2\,\mathrm{tr}(\varepsilon) + \|\nabla u\|_F^2$ rearrangement as neo-Hookean plus an analogous rearrangement for $I_2 - 3$.

## Near-incompressibility and the Ch 05 handoff

Silicone sits at $\nu \approx 0.499$, and the compressible form above locks under bending the same way neo-Hookean does — the $\lambda / \mu$ ratio dominates, tets lose the bending mode, and the [volume-loss failure](../../../10-physical/02-what-goes-wrong/00-volume-loss.md) appears unchanged. The cure is the same decorator pattern as [NH's incompressible sibling](../00-neo-hookean/02-incompressible.md): the *modified* Mooney-Rivlin form, written in isochoric invariants,

$$ \psi_\text{modified}(F) \;=\; C_{10}\,(\bar I_1 - 3) \;+\; C_{01}\,(\bar I_2 - 3) \;+\; U(J) $$

with $\bar I_1 = J^{-2/3} I_1$ and $\bar I_2 = J^{-4/3} I_2$ (per [Ch 01 invariants](../../01-strain-measures/03-invariants.md)), is deviatoric-plus-volumetric by construction — $\bar I_1$ and $\bar I_2$ depend only on $\bar F$, $U(J)$ depends only on $J$. Modified-MR and compressible-MR agree at $O(\|F - I\|^3)$ at small strain and diverge at finite strain, same story as neo-Hookean. Which form the impl uses is declared on the struct; standalone low-Poisson use takes the compressible form (simpler tangent), the near-incompressibility decorator from [Ch 05](../../05-incompressibility.md) wraps the modified form.

Standalone `MooneyRivlin` declares $\nu < 0.45$ on its [`ValidityDomain`](../../00-trait-hierarchy/02-validity.md), matching neo-Hookean's standalone bound. The [mixed u-p](../../05-incompressibility/01-mixed-up.md) and [F-bar](../../05-incompressibility/02-f-bar.md) decorators widen the upper bound to $\nu < 0.499$ when they wrap this impl; the decorators are constitutive-law-agnostic and pair the same way with neo-Hookean, Mooney-Rivlin, and Ogden.

## What this sub-leaf commits the book to

- **Mooney-Rivlin's evaluation path is $(I_1, I_2, J)$, not principal stretches.** The impl never materializes $C$ as a $3\times 3$ — only its trace and the Frobenius norm squared of its entries — and never takes an eigen-decomposition. [Ogden](../02-ogden.md) takes the eigen-decomposition route; Mooney-Rivlin does not.
- **The compressible form carries a $-(2C_{10} + 4C_{01}) \ln J$ stress-free-reference corrector.** Without it the reference configuration is pre-stressed and the Phase B gradcheck fails. This is the same structural role neo-Hookean's $-\mu \ln J$ plays, and the corrector's coefficient is $(2C_{10} + 4C_{01})$ by exact analogy.
- **Tangent structure has two extra blocks over neo-Hookean.** Both come from the $I_2$ derivative chain rule. Per-element stiffness block size is unchanged — $12 \times 12$ for Tet4, $30 \times 30$ for Tet10 — so sparse-assembly shape and the [factor-on-tape path](../../../50-time-integration/00-backward-euler.md) carry over without change.
- **Standalone `MooneyRivlin` caps at $\nu < 0.45$.** Higher Poisson ratios require wrapping in the Ch 05 mixed-u-p or F-bar decorator; the $(E, \nu)$ constructor asserts on $\nu \geq 0.45$ rather than silently constructing an instance outside its declared regime, same pattern as [neo-Hookean's constructor](../00-neo-hookean/03-impl.md).
