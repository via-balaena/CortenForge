# Why it dominates for Ecoflex at high strain

The [N-term sibling](00-n-term.md) wrote down the principal-stretch energy; the [Ogden branch parent](../02-ogden.md) claimed Ogden reaches function shapes the invariant-polynomial laws cannot. This leaf discharges that claim — first by demonstrating algebraically that the non-integer-exponent Ogden term is not a rational function of $(I_1, I_2, I_3)$, and then by naming the two Ecoflex-at-high-strain features — strain hardening through elongation-at-break and tension-vs-compression asymmetry — that the extra freedom captures. The [stability sibling](02-stability.md) handles what goes numerically wrong when the principal-stretch representation touches the isotropic point.

## The function-shape gap

Mooney-Rivlin's energy is a polynomial in the isotropic invariants $I_1, I_2, I_3$ of $C = F^T F$, and any extension within the invariant-polynomial family — higher-order Yeoh, generalized polynomial forms — stays inside the field $\mathbb{Q}(I_1, I_2, I_3)$ of rational functions of the three invariants. Ogden with non-integer $\alpha_i$ is structurally outside that field.

**Algebraic demonstration.** Fix two principal stretches $(\lambda_2, \lambda_3)$ at any generic values and vary the third, $\lambda_1$, as a real parameter. With $\lambda_2, \lambda_3$ fixed, each invariant becomes a polynomial in the single variable $u = \lambda_1^2$:

$$ I_1 \;=\; u \;+\; (\lambda_2^2 + \lambda_3^2) $$

$$ I_2 \;=\; u\,(\lambda_2^2 + \lambda_3^2) \;+\; \lambda_2^2\, \lambda_3^2 $$

$$ I_3 \;=\; u\, \lambda_2^2\, \lambda_3^2 $$

All three are degree-one in $u$. Any rational function $R(I_1, I_2, I_3)$ therefore reduces to a rational function $R\!\!\text{ }\!(u)$ of the single variable $u$ — a quotient of two polynomials in $u$ with real coefficients.

The Ogden single-term contribution to $\psi$ is $(\mu_i / \alpha_i)(\lambda_1^{\alpha_i} + \lambda_2^{\alpha_i} + \lambda_3^{\alpha_i} - 3)$. With $(\lambda_2, \lambda_3)$ fixed, the $\lambda_1$-dependence is $\lambda_1^{\alpha_i} = u^{\alpha_i / 2}$, a constant term aside. For non-integer $\alpha_i$, $u^{\alpha_i / 2}$ is not a rational function of $u$ — no polynomial identity $p(u)\, u^{\alpha_i/2} = q(u)$ with $p, q$ real polynomials has a solution, because the leading behavior as $u \to \infty$ on the left grows as $u^{\deg p + \alpha_i/2}$ and on the right as $u^{\deg q}$, and the two exponents cannot match ($\alpha_i / 2 \notin \mathbb{Q}$ in the irrational-exponent case; $\deg p + \alpha_i / 2 \notin \mathbb{Z}$ in the non-integer-rational case).

Therefore the Ogden term (restricted to variable $\lambda_1$ with the other two stretches fixed) cannot equal any rational function of $u$, and a fortiori cannot equal any rational function of $(I_1, I_2, I_3)$. The argument holds for every generic $(\lambda_2, \lambda_3)$, so the Ogden function as a function of the three principal stretches is structurally outside the invariant-polynomial family. The Mooney-Rivlin ring of two-parameter polynomials in $(I_1, I_2)$ is a further restriction inside $\mathbb{Q}(I_1, I_2, I_3)$; whatever Ogden reaches that invariant-rational-functions cannot, Mooney-Rivlin cannot reach either.

For integer $\alpha_i$ the story splits by parity. Even integer $\alpha_i$ reduces $\lambda_j^{\alpha_i} = (\lambda_j^2)^{\alpha_i / 2}$ to a polynomial in the squared stretches and hence to a polynomial in the invariants by Newton's identities; odd integer $\alpha_i$ does not, because the reduction requires extracting $\lambda_j$ itself from $\lambda_j^2$ (a square-root operation). The practical Ogden calibrations for silicone carry $\alpha_i$ that are either non-integer (common) or odd-integer (occasional), so the function-shape gap is the generic case.

## Strain hardening through elongation-at-break

Ecoflex 00-30 and the softer 00-10 grade exhibit a characteristic stress–strain curve shape under tensile loading: a roughly linear small-strain region, a moderate-stretch regime where stress rises sublinearly with stretch, and then a strain-hardening inflection near the elongation-at-break where the slope increases again before the material fails. The overall shape is neither a monotone-increasing polynomial nor a saturating exponential — it has an inflection that's the physical signature of polymer chain extensibility approaching its limit.

Neo-Hookean's $(I_1 - 3)$ term is monotone in stretch and has no inflection. Mooney-Rivlin's addition of $(I_2 - 3)$ can bend the curve but its polynomial structure in the invariants limits the achievable shapes — the curve saturates rather than re-hardens at high stretch. An $N = 2$ Ogden fit with one large-positive $\alpha_1$ and one negative $\alpha_2$ produces a term that scales with high-positive power at tension (capturing the hardening inflection) while retaining linear-elastic behavior at small strain (via the sum rule $\mu = \tfrac{1}{2}\sum_i \mu_i \alpha_i$). $N = 3$ extends the envelope further.

This is the specific regime the [Ogden branch parent](../02-ogden.md) named: above ≈150% stretch, both neo-Hookean and Mooney-Rivlin saturate against published Ecoflex data, while Ogden tracks through the strain-hardening inflection at $N = 2$ and through the pre-failure region at $N = 3$.

## Tension-vs-compression asymmetry

Ecoflex under contact-driven loading sees tension at the sleeve rim and compression at the indentation zone in the same simulation, and the measured material response is not symmetric about $\lambda = 1$. A rounded probe pressed into a compliant cavity produces tensile principal stretches on one side of the contact zone and compressive ones on the other — the [rim-deformation failure mode](../../../10-physical/02-what-goes-wrong/04-rim.md) is where this asymmetry becomes reward-visible.

Ogden with mixed-sign $\alpha_i$ handles this directly. A term with $\alpha_i > 0$ dominates at $\lambda_j \gg 1$ (tensile stretching), and a term with $\alpha_i < 0$ dominates at $\lambda_j \ll 1$ (compressive stretching, where $\lambda_j^{\alpha_i} = \lambda_j^{-|\alpha_i|}$ grows as $\lambda_j \to 0^+$). The two terms' coefficients are fit independently, so the tensile and compressive stiffnesses are set by different parameters of the same Ogden instance. The [stability sibling](02-stability.md) names the $\lambda \to 0$ divergence as a numerical concern and the IPC barrier as what keeps $\det F$ bounded away from zero in practice.

Mooney-Rivlin's $(C_{10}, C_{01})$ polynomial in $(I_1, I_2)$ couples tension and compression through the same two coefficients. Fit $(C_{10}, C_{01})$ to match tensile data and the compressive response is determined — there is no independent compressive tuning. When the measured tensile and compressive stiffnesses disagree on what $(C_{10}, C_{01})$ should be, the fit splits the difference and residuals grow on both sides. The Ogden structural freedom to decouple the two regimes via separate $(\mu_i, \alpha_i)$ pairs is what captures the asymmetry cleanly.

## When the gap actually matters

The function-shape gap and the asymmetry advantage are both real, but they only move the reward in the stretch regimes where the other laws are drifting. The [Mooney-Rivlin tradeoffs sub-leaf](../01-mooney-rivlin/02-tradeoffs.md) stated this from the other side: below ≈50% stretch the three laws agree within measurement noise; between 50% and 150%, Mooney-Rivlin earns its extra parameter; above ≈150% Ogden is the only law tracking. Adding the asymmetry axis: in contact scenarios where the principal stretches cross below $\lambda \approx 0.7$ and above $\lambda \approx 1.5$ on the same element over time, Ogden's independent tension/compression tuning is what lets a single impl serve the whole range.

Where the stretch never crosses 150% and never goes deeply compressive, the function-shape gap is irrelevant and the eigen-decomposition cost is overhead. The [N-term sibling's](00-n-term.md) evaluation-path observation — eigen-decomposition plus $N$ `pow` calls per principal direction per Gauss point — is the honest price of the extra coverage; the decision of whether to pay it is the [tradeoffs sub-leaf's](../01-mooney-rivlin/02-tradeoffs.md) three-axis test.

## What this sub-leaf commits the book to

- **Ogden is structurally out of reach for invariant-polynomial laws.** Non-integer $\alpha_i$ produces a term that is not a rational function of $(I_1, I_2, I_3)$; the transcendence argument on a single-stretch restriction makes this algebraic rather than empirical. Mooney-Rivlin, Yeoh, and any generalized-polynomial hyperelastic form inherits this limitation.
- **The strain-hardening inflection is where Ogden earns the cost.** $N = 2$ captures the inflection at moderate-strain onset; $N = 3$ extends through the pre-failure regime. Ecoflex 00-30 at the elongation-at-break end of its operating range is the canonical case for $N = 3$ in [Part 1 Ch 04](../../../10-physical/04-material-data/00-ecoflex.md)'s material database.
- **Tension-vs-compression asymmetry is a mixed-sign $\alpha_i$ feature.** Positive exponents set tensile stiffness, negative exponents set compressive stiffness, and the sum rule $\mu = \tfrac{1}{2}\sum_i \mu_i \alpha_i$ ties the two to the linear-elastic limit. A single Ogden instance handles both directions; a single Mooney-Rivlin instance couples them through shared coefficients.
- **The gap only matters in regimes that use it.** Below 50% stretch or in purely tensile-or-compressive scenes, Ogden's extra expressive power is overhead. The [tradeoffs sub-leaf in the MR branch](../01-mooney-rivlin/02-tradeoffs.md) makes the per-scene decision; this sub-leaf makes the "Ogden actually has the expressive power" part of that decision rigorous.
