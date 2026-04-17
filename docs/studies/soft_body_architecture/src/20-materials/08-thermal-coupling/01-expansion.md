# Thermal expansion

The [Ch 08 parent's Section 1](../08-thermal-coupling.md) commits to a multiplicative split $F = F_e\, F_\theta$ for thermal expansion, with $F_\theta = (1 + \alpha (T - T_0))\, I$ for isotropic expansion. This leaf writes the kinematic split, the Piola-stress conversion the [`Thermal<M>` decorator](../00-trait-hierarchy/01-composition.md) performs around the wrapped base call, the volumetric-strain interpretation, and the $\partial P / \partial T$ contribution from the expansion side.

## Multiplicative versus additive split

Additive thermal strain $\varepsilon_\text{therm} = \alpha\, (T - T_0)\, I$ is correct only at small strain — at finite mechanical deformation, additive strains do not compose multiplicatively with the deformation gradient, and the resulting stress is wrong by terms of order $\|F - I\|^2$. The multiplicative split is the standard finite-deformation generalization:

$$ F = F_e\, F_\theta, \qquad F_\theta = \beta(T)\, I, \qquad \beta(T) = 1 + \alpha\, (T - T_0) $$

with $\alpha$ the thermal expansion coefficient ([appendix entry](../../appendices/03-notation.md), distinguished from Ogden's $\alpha_i$ by the absence of a per-mode subscript). $F_e$ is the elastic part — the deformation the wrapped base material sees — and $F_\theta$ is the thermal-expansion part. At $T = T_0$, $\beta = 1$ and $F_\theta = I$, so $F_e = F$ and the decorator collapses to the base material directly. At $T > T_0$, $F_\theta$ describes a uniform isotropic expansion: every length scales by $\beta$, every volume by $\beta^3$.

The decorator computes $F_e = F\, F_\theta^{-1} = F / \beta$ before calling `base.first_piola(F_e)`, then applies the inverse map to convert the result back to the first Piola in the original $F$ frame:

$$ P = P_e\, F_\theta^{-T} = \frac{1}{\beta}\, P_e $$

For isotropic $F_\theta = \beta\, I$, $F_\theta^{-T} = (1/\beta)\, I$, so the conversion is a scalar rescaling — much simpler than the general anisotropic case. The base material's evaluation is unchanged; the decorator handles the kinematic-split bookkeeping at the trait-surface boundary.

## Volumetric interpretation

The volume ratio $J = \det F$ decomposes as $J = J_e\, J_\theta$ with $J_e = \det F_e$ and $J_\theta = \det F_\theta = \beta^3$. The thermal contribution is:

$$ J_\theta - 1 = \beta^3 - 1 \approx 3\, \alpha\, (T - T_0) $$

for small $\alpha\, (T - T_0)$ — recovering the small-strain "volumetric thermal strain is $3\alpha\, \Delta T$" textbook result as the linearization of the multiplicative form. The mechanical $J_e$ is what the wrapped base material's volumetric energy term operates on (the [neo-Hookean $(\lambda/2)(\ln J)^2$](../04-hyperelastic/00-neo-hookean/00-energy.md), the [Mooney-Rivlin volumetric corrector](../04-hyperelastic/01-mooney-rivlin/00-two-param.md), the [Ogden $-(\sum \mu_i)\ln J$ corrector](../04-hyperelastic/02-ogden/00-n-term.md)) — the base sees only the mechanical part, which is correct: the thermal expansion does not generate volumetric stress, it just changes the equilibrium configuration.

## Per-Gauss-point evaluation

At every Newton iteration of every step, the decorator queries the temperature at each Gauss point ([temperature-dependent modulus sibling](00-modulus-T.md)), computes $\beta(T_g) = 1 + \alpha\, (T_g - T_0)$ for the per-Gauss-point thermal stretch factor, evaluates $F_e^{(g)} = F^{(g)} / \beta(T_g)$ as the elastic part the base material sees, calls `base.first_piola(F_e^{(g)})` with the temperature-adjusted Lamé pair (also per [Ch 00 composition](../00-trait-hierarchy/01-composition.md)), and rescales the returned $P_e^{(g)}$ by $1/\beta(T_g)$ to produce the per-Gauss-point first Piola.

Per-Gauss-point evaluation handles spatially varying temperature: a temperature gradient across an element produces different $\beta$ values at different Gauss points, so the resulting Piola stress is a non-uniform combination. For Tet4 (one Gauss point) the temperature is element-uniform; for Tet10 (multiple Gauss points) the temperature gradient within the element is resolved.

## $\partial P / \partial T$ contribution from expansion

When the [bidirectional coupling](02-dissipation.md) is live, the expansion side contributes to the Newton tangent's $\partial P / \partial T$ alongside the [modulus side](00-modulus-T.md). The expansion-side derivative comes from the chain through $\beta$:

$$ \frac{d \beta}{d T} = \alpha, \qquad \frac{\partial F_e}{\partial T} = -\frac{F\, \alpha}{\beta^2}, \qquad \frac{\partial P}{\partial T}\bigg|_\text{exp} = -\frac{\alpha}{\beta^2}\, P_e + \frac{1}{\beta}\, \frac{\partial P_e}{\partial F_e}\, \frac{\partial F_e}{\partial T} $$

The first term comes from the rescaling $1/\beta$ derivative; the second from the chain through $F_e$. Both terms are negative for stretched material at $T > T_0$ (consistent with "warmer is softer"). The full $\partial P / \partial T$ is the sum of the [modulus-side contribution](00-modulus-T.md) and this expansion-side contribution; both flow into the Newton tangent and are gradcheck-verified at [Phase B](../../110-crate/03-build-order.md).

## Validity and the anisotropic-expansion soft-defer

The isotropic-expansion form $F_\theta = \beta\, I$ covers the dominant case — silicones, rubbers, and most polymeric soft materials are essentially isotropic in their thermal response. Anisotropic thermal expansion (some fiber-reinforced composites with directional CTE mismatch, biological vessels with anatomy-dependent expansion) requires a tensor $F_\theta$ that is not proportional to $I$; the decorator's `expansion: f64` field would extend to a per-direction tensor in that regime, which is out of the Phase A–I scope and soft-deferred to a post-Phase-I extension.

The temperature range in which the linear $\beta(T)$ approximation holds is the same as the [modulus curve's validity band](00-modulus-T.md); both validity slots widen together when the user supplies a non-linear curve.

## What this sub-leaf commits the book to

- **Multiplicative split $F = F_e\, F_\theta$ with $F_\theta = \beta(T)\, I$ and $\beta(T) = 1 + \alpha\, (T - T_0)$.** Standard finite-deformation generalization of additive thermal strain, recovering the small-strain $3\alpha\, \Delta T$ volumetric result as the linearization.
- **Decorator computes $F_e = F/\beta$, calls base, rescales by $1/\beta$.** The base material's evaluation is unchanged; the decorator handles the kinematic-split bookkeeping at the trait-surface boundary. For isotropic expansion the conversion is a scalar rescaling.
- **The base sees only the mechanical $J_e$.** Thermal expansion does not generate volumetric stress; it changes the equilibrium configuration, and the wrapped base law's volumetric corrector operates on the mechanical part only.
- **Per-Gauss-point evaluation handles spatial temperature gradients.** Tet10's per-Gauss-point sampling resolves within-element temperature variation; Tet4 uses the element-uniform value.
- **Expansion-side $\partial P / \partial T$ adds to the modulus-side contribution.** Both flow through the Newton tangent; gradcheck-verified at Phase B.
- **Anisotropic thermal expansion is soft-deferred to post-Phase-I.** The default isotropic form covers silicones and most soft-material applications; tensor-valued $F_\theta$ extension is named but not in the Phase A–I scope.
