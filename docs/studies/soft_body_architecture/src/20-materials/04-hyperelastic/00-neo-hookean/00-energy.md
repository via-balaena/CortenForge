# Energy density and stress

The [neo-Hookean branch parent](../00-neo-hookean.md) named the two-Lamé-parameter closed-form law; this leaf writes the energy density $\psi(F)$ down, derives the first Piola stress $P = \partial \psi / \partial F$ by differentiation, and names the identities the derivation turns on. The [tangent sibling](01-tangent.md) takes the next derivative to get $\partial P / \partial F$.

## Energy density

$$ \psi(F) \;=\; \tfrac{\mu}{2}\,(I_1 - 3) \;-\; \mu \ln J \;+\; \tfrac{\lambda}{2}\,(\ln J)^2 $$

with:

- $I_1 = \mathrm{tr}(F^T F)$ — first invariant of the right Cauchy-Green tensor ([Part 2 Ch 01 invariants](../../01-strain-measures/03-invariants.md))
- $J = \det F$ — volume ratio ([Part 2 Ch 01 deformation gradient](../../01-strain-measures/00-F.md))
- $\mu, \lambda$ — Lamé parameters, convertible from $(E, \nu)$ via $\mu = E / 2(1+\nu)$ and $\lambda = E\nu / [(1+\nu)(1-2\nu)]$ ([Part 2 Ch 02 linear](../../02-linear.md))

The three terms split responsibilities. The $\tfrac{\mu}{2}(I_1 - 3)$ term grows with any departure from the identity and is zero at $F = I$ (since $I_1(I) = 3$). The $-\mu \ln J$ term pairs with the first: together, their derivatives with respect to $F$ combine to $\mu\,(F - F^{-T})$, which vanishes at $F = I$ (stress-free reference configuration) and reduces to the shear-coefficient $\mu$ of linear elasticity in the small-strain limit. Neither term alone can produce that structure — $-\mu \ln J$ is what makes the first-Piola formula come out right, not a volumetric double-count corrector. The $\tfrac{\lambda}{2}(\ln J)^2$ term is the volumetric barrier — it grows as $(\ln J)^2$, diverging as $J \to 0^+$ (element collapse) and as $J \to \infty$ (extreme dilation). [The bounded-energy pathology of Saint Venant–Kirchhoff](../../01-strain-measures/02-E.md) is exactly what this term prevents.

Rotation invariance is structural. Under $F \mapsto R\, F$ with $R \in SO(3)$: $I_1$ is unchanged because $R^T R = I$, and $J$ is unchanged because $\det R = 1$. Therefore $\psi(R F) = \psi(F)$ for every $R \in SO(3)$. The fictitious-rotation-stress failure that [Ch 02 linear](../../02-linear.md) predicts and [Ch 03 corotational](../../03-corotational.md) works around cannot occur for this energy; rotation-invariance is a consequence of the $(I_1, J)$ parameterization, not an implementation property.

## First Piola stress

The first Piola-Kirchhoff stress is the derivative of $\psi$ with respect to $F$:

$$ P(F) \;=\; \frac{\partial \psi}{\partial F} \;=\; \mu\,(F - F^{-T}) \;+\; \lambda\,(\ln J)\, F^{-T} $$

The derivation uses two tensor identities:

$$ \frac{\partial I_1}{\partial F} \;=\; 2\, F \qquad\text{and}\qquad \frac{\partial \ln J}{\partial F} \;=\; F^{-T} $$

The first follows from $I_1 = \mathrm{tr}(F^T F) = F : F$. The second follows from Jacobi's formula $\partial \det F / \partial F = (\det F)\, F^{-T}$ and the chain rule on $\ln J$. Applying them term-by-term:

$$ \frac{\partial}{\partial F}\left[\tfrac{\mu}{2}\,(I_1 - 3)\right] \;=\; \mu\, F $$

$$ \frac{\partial}{\partial F}\left[-\mu \ln J\right] \;=\; -\mu\, F^{-T} $$

$$ \frac{\partial}{\partial F}\left[\tfrac{\lambda}{2}\,(\ln J)^2\right] \;=\; \lambda\,(\ln J)\, F^{-T} $$

Summing gives $P = \mu\,(F - F^{-T}) + \lambda\,(\ln J)\, F^{-T}$. At $F = I$ the stress is zero ($F^{-T} = I$, $\ln J = 0$, so $P = \mu\,(I - I) + 0 = 0$) — a stress-free reference configuration is built in.

## Evaluation path in `sim-soft`

Per Gauss point, `sim-soft`'s `impl Material for NeoHookean` ([implementation sub-leaf](03-impl.md)) computes $I_1$ and $J$ directly from $F$ rather than forming the full $C = F^T F$ tensor ([Part 2 Ch 01 — C](../../01-strain-measures/01-C.md)):

- $I_1 = \mathrm{tr}(F^T F) = \sum_{ij} F_{ij}^2$ — Frobenius norm squared of $F$, no matrix multiplication
- $J = \det F$ — one $3\times 3$ determinant
- $F^{-T}$ — one $3\times 3$ matrix inverse transpose, cached because both $P$ and the [tangent](01-tangent.md) need it

At small strain ($F \approx I$), the $I_1 - 3$ and $\ln J$ offsets are catastrophic-cancellation-prone; [Part 2 Ch 01 invariants](../../01-strain-measures/03-invariants.md) named the cancellation-safe expressions `sim-soft` uses in place of direct subtractions — $I_1 - 3 = 2\,\mathrm{tr}(\varepsilon) + \|\nabla u\|_F^2$, $\ln J$ via `log1p` when $J - 1$ is small.

## What this sub-leaf commits the book to

- **Neo-Hookean's evaluation path is $(I_1, J)$, not $(I_1, I_2, I_3)$ or principal stretches.** The impl never forms $I_2$ or the full $C$ tensor; [Mooney-Rivlin](../01-mooney-rivlin.md) adds $I_2$ and [Ogden](../02-ogden.md) adds a principal-stretch eigen-decomposition, neo-Hookean does not.
- **Rotation invariance is structural.** Any downstream claim about neo-Hookean passing the [rotation-invariance gradcheck](../../../110-crate/04-testing/03-gradcheck.md) is a consequence of the energy being a function of $(I_1, J)$ alone, not a test the implementation has to be engineered around.
- **The $\tfrac{\lambda}{2}(\ln J)^2$ barrier is load-bearing for IPC safety.** An element approaching inversion pays unbounded energy; the [Part 4 IPC barrier](../../../40-contact/01-ipc-internals/00-barrier.md) and this volumetric term together make the canonical problem's IPC-guarded basin strictly bounded in $\det F$.
- **First-Piola form is the assembly input.** Downstream ([Part 6 Ch 01 FEM assembly VJP](../../../60-differentiability/01-custom-vjps/01-fem-assembly.md), [Part 5 Ch 00 Newton residual](../../../50-time-integration/00-backward-euler.md)) consumes $P = \mu\,(F - F^{-T}) + \lambda\,(\ln J)\, F^{-T}$ directly; no Cauchy or second-Piola conversion on the hot path.
