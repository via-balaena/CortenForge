# N-term form

The [Ogden branch parent](../02-ogden.md) named the principal-stretch formulation and the $N = 2$ or $3$ practical range. This leaf writes the $N$-term energy down, derives the first Piola stress and tangent structure through principal-stretch derivatives, and states the per-Gauss-point evaluation path — the eigen-decomposition, the `pow` calls, and the cached quantities the [stability sibling](02-stability.md) inherits as inputs to its degeneracy treatment. The [why-Ogden sibling](01-why-ogden.md) takes the closed-form energy and argues why its function shapes are structurally out of reach for [Mooney-Rivlin](../01-mooney-rivlin.md)'s invariant-polynomial form.

## Energy density

$$ \psi(F) \;=\; \sum_{i=1}^{N} \frac{\mu_i}{\alpha_i}\,\left(\lambda_1^{\alpha_i} + \lambda_2^{\alpha_i} + \lambda_3^{\alpha_i} - 3\right) \;+\; U_\text{vol}(J) $$

with:

- $\lambda_1, \lambda_2, \lambda_3$ — principal stretches, the eigenvalues of the right stretch tensor $U$ from the polar decomposition $F = R\, U$ ([Part 2 Ch 01 deformation gradient](../../01-strain-measures/00-F.md))
- $(\mu_i, \alpha_i)_{i=1}^{N}$ — Ogden parameters, $N$ pairs of shear-coefficient $\mu_i$ (pressure units) and exponent $\alpha_i$ (dimensionless, typically non-integer and possibly negative)
- $U_\text{vol}(J)$ — volumetric term; the same role as neo-Hookean's $\tfrac{\lambda}{2}(\ln J)^2$ and Mooney-Rivlin's $-(2C_{10} + 4C_{01})\ln J + \tfrac{\lambda}{2}(\ln J)^2$ — a strictly convex function of $J$ with minimum at $J = 1$ plus a reference-configuration offset that cancels the deviatoric term's stress at $F = I$

$N = 2$ or $3$ is the practical range named in the parent: $N = 2$ covers strain-hardening silicones through most of their operating regime, $N = 3$ extends through the elongation-at-break inflection. The impl supports general $N$; calibrated defaults ship only for $N \in \{2, 3\}$.

At $\lambda_1 = \lambda_2 = \lambda_3 = 1$ each bracketed factor $\lambda_j^{\alpha_i} - 1$ vanishes, so the "$-3$" makes $\psi(I) = 0$ by construction. The deviatoric term's first-Piola contribution at $F = I$ is $\sum_i \mu_i\, I$ per principal direction, not zero — so $U_\text{vol}$ must carry a stress-free-reference corrector, typically $-(\Sigma_i \mu_i) \ln J$, in direct parallel to neo-Hookean's $-\mu \ln J$ and Mooney-Rivlin's $-(2C_{10} + 4C_{01}) \ln J$. The corrector plus a $(\ln J)^2$-class volumetric barrier is the standard combined $U_\text{vol}$.

Small-strain reduction: the effective small-strain shear modulus is $\mu = \tfrac{1}{2}\sum_i \mu_i\, \alpha_i$ (Ogden's sum-rule). Calibration picks $(\mu_i, \alpha_i)_i$ so that $\tfrac{1}{2}\sum_i \mu_i \alpha_i$ matches the measured linear-elastic $\mu$ of the target material, plus additional constraints from the finite-strain data — the [gradcheck small-strain reduction](../../../110-crate/04-testing/03-gradcheck.md) asserts six-digit agreement with linear elasticity at 0.1% strain exactly when the sum rule holds.

Rotation invariance is structural. Principal stretches are eigenvalues of $U$, which is the rotation-invariant factor in $F = RU$; under $F \mapsto R'F$ the rotation $R$ becomes $R'R$ and $U$ is unchanged. Every Ogden term is therefore rotation-invariant without a separate argument.

## First Piola stress through principal stretches

For an isotropic hyperelastic energy written in the principal stretches, the first Piola stress in the principal frame has diagonal components equal to the partial derivatives $\partial \psi / \partial \lambda_j$:

$$ P_{jj}^\text{principal} \;=\; \frac{\partial \psi}{\partial \lambda_j} \;=\; \sum_{i=1}^{N} \mu_i\, \lambda_j^{\alpha_i - 1} \;+\; \frac{\partial U_\text{vol}}{\partial J}\, \frac{\partial J}{\partial \lambda_j} $$

The full $3\times 3$ first Piola is reconstructed by rotating back: if $F = R\, U = R\, N\, \mathrm{diag}(\lambda_j)\, N^T$ with $N$ the orthonormal eigenvectors of $U$, then

$$ P(F) \;=\; R\, N\, \mathrm{diag}(P_{jj}^\text{principal})\, N^T $$

The per-Gauss-point cost is the spectral decomposition of $U$ (one $3\times 3$ symmetric eigenvalue problem), $N$ non-integer `pow` evaluations per principal stretch, and one rotation back to the reference frame. This is structurally different from neo-Hookean's and Mooney-Rivlin's $(I_1, I_2, J)$ evaluation path: Ogden actually needs the eigen-decomposition, no invariant-only route exists (per the [why-Ogden sibling](01-why-ogden.md)).

## Tangent structure

Differentiating $P(F)$ a second time gives the fourth-order tangent $\mathbb{C}_{ijkl} = \partial P_{ij}/\partial F_{kl}$. In the principal frame, the tangent has:

- **Diagonal-diagonal blocks** $\partial P_{jj}^\text{principal} / \partial \lambda_k$ — the second derivatives $\partial^2 \psi / \partial \lambda_j\, \partial \lambda_k$, which for Ogden are closed-form: $\sum_i \mu_i (\alpha_i - 1)\, \lambda_j^{\alpha_i - 2}$ on the diagonal and $\partial^2 U_\text{vol}/\partial \lambda_j \partial \lambda_k$ for the volumetric coupling.
- **Off-diagonal (shear) blocks** involving derivatives of the eigenvectors $N$ themselves with respect to $F$. These are the terms that become ill-conditioned at repeated-eigenvalue configurations ($\lambda_1 \approx \lambda_2$ or the isotropic point), and are what the [stability sibling](02-stability.md) treats.

The tangent is symmetric (major symmetry $\mathbb{C}_{ijkl} = \mathbb{C}_{klij}$) by hyperelastic construction — $P$ derives from a scalar energy, Clairaut. Per-element stiffness block size is the same $12\times 12$ (Tet4) or $30\times 30$ (Tet10) as the other hyperelastic laws; the per-Gauss-point arithmetic cost is where Ogden pays.

## Evaluation path in `sim-soft`

Per Gauss point, `impl Material for Ogden`:

- Computes $U = \sqrt{F^T F}$ via symmetric eigen-decomposition of $C = F^T F$ — one $3\times 3$ symmetric eigenvalue problem. Principal stretches are the square roots of $C$'s eigenvalues; eigenvectors $N$ are shared with $C$.
- Evaluates $\lambda_j^{\alpha_i}$ for each term $i$ and principal direction $j$. For non-integer $\alpha_i$ this is a `f64::powf`; for integer $\alpha_i$ the impl branches to integer-power inlining (`powi` or hand-unrolled multiplications).
- Caches $\lambda_j^{\alpha_i - 1}$ and $\lambda_j^{\alpha_i - 2}$ on the `EvaluationScratch` so the first-Piola and tangent calls share the `pow` evaluations.
- Computes $J = \det F$ directly (not as $\lambda_1 \lambda_2 \lambda_3$) to avoid the compounded `pow` error on $J$, and evaluates $U_\text{vol}(J)$ and its derivatives through the standard scalar closed forms.
- Reconstructs the full $3\times 3$ $P$ and tangent via the rotation back, or equivalently stores the principal-frame quantities and pulls back via $N$ at assembly time.

The eigen-decomposition is the single largest per-Gauss-point cost difference vs the other hyperelastic laws. The [tradeoffs sub-leaf in the Mooney-Rivlin branch](../01-mooney-rivlin/02-tradeoffs.md) named this as the regime that argues against Ogden for moderate-stretch scenes: the arithmetic pays for the extra regime coverage only when the stretch range actually uses it.

## Near-incompressibility and the Ch 05 handoff

The principal-stretch form decomposes cleanly into isochoric + volumetric. Write $\lambda_j = J^{1/3}\, \bar\lambda_j$ with $\bar\lambda_1 \bar\lambda_2 \bar\lambda_3 = 1$; the modified Ogden energy is

$$ \psi_\text{modified}(F) \;=\; \sum_{i=1}^{N} \frac{\mu_i}{\alpha_i}\,\left(\bar\lambda_1^{\alpha_i} + \bar\lambda_2^{\alpha_i} + \bar\lambda_3^{\alpha_i} - 3\right) \;+\; U(J) $$

The first sum depends only on $\bar\lambda_j$ (equivalently only on $\bar F$), and $U(J)$ depends only on $J$. This split is structurally cleaner than Mooney-Rivlin's (which has to carry the $I_1, I_2$ coupling of $\bar F$ with $J$ through the non-trivial $\bar I_1 = J^{-2/3} I_1$ and $\bar I_2 = J^{-4/3} I_2$ rearrangements) — principal stretches already factor through $J^{1/3}$.

Standalone compressible Ogden and modified Ogden agree at $O(\|F - I\|^3)$ at small strain and diverge at finite strain, same structural story as neo-Hookean and Mooney-Rivlin. Standalone use at low Poisson takes the compressible form, near-incompressibility wraps the modified form via the [Ch 05 decorator](../../05-incompressibility.md).

Standalone `Ogden` declares $\nu < 0.45$ on its [`ValidityDomain`](../../00-trait-hierarchy/02-validity.md), matching neo-Hookean's and Mooney-Rivlin's standalone bounds. The [mixed u-p](../../05-incompressibility/01-mixed-up.md) and [F-bar](../../05-incompressibility/02-f-bar.md) decorators widen it to $\nu < 0.499$ via the same constitutive-law-agnostic pattern. `InversionHandling::RequireOrientation` carries over too — standalone Ogden assumes IPC prevents inversion and panics on non-positive $\det F$.

## What this sub-leaf commits the book to

- **Ogden's evaluation path is principal stretches, not invariants.** The per-Gauss-point eigen-decomposition is not optional and does not have an equivalent invariant-only shortcut; the [why-Ogden sibling](01-why-ogden.md) argues this formally.
- **The small-strain shear-modulus sum rule is $\mu = \tfrac{1}{2}\sum_i \mu_i\, \alpha_i$.** Calibration imposes this as a hard constraint alongside the finite-strain residual minimization, so the gradcheck small-strain reduction test holds by construction rather than by fit quality.
- **The principal-stretch split into isochoric + volumetric is structurally clean.** $\bar\lambda_j = J^{-1/3}\,\lambda_j$ with $\prod \bar\lambda_j = 1$, and the modified-Ogden form is deviatoric-plus-volumetric by construction without the invariant-rearrangement acrobatics Mooney-Rivlin's modified form carries.
- **Per-element stiffness block size is unchanged — $12\times 12$ (Tet4) or $30\times 30$ (Tet10).** The cost difference vs the invariant-based laws is per-Gauss-point arithmetic (the eigen-decomposition plus $N$ `pow` calls per principal direction), not solver topology. The [factor-on-tape Cholesky path](../../../50-time-integration/00-backward-euler.md) carries over without change.
- **Standalone `Ogden` caps at $\nu < 0.45$ with `RequireOrientation`.** The Ch 05 decorator widens the Poisson upper bound to $\nu < 0.499$; IPC prevents element inversion on the Phase D onward canonical-problem solves.
