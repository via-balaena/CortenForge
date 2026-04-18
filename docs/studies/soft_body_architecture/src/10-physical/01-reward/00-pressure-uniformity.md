# Pressure uniformity

The [parent chapter](../01-reward.md) named four reward terms the [composition sub-chapter](04-composition.md) combines into a scalar objective. This leaf pins down the first: pressure uniformity. A cavity that loads the probe with high pressure in one band and low pressure in another is strictly worse than a cavity that loads evenly, even at the same total transmitted force. The uniformity term penalizes the former.

## What it measures

Nonuniform contact pressure over the actively-contacted region of the cavity's inner surface. In prose: a ring of pressure higher than the surrounding band scores worse than a pressure distribution that is constant over the contacted area. The former indents the probe in a ring pattern, preferentially fatigues the cavity material on the ring, and propagates a bias into any downstream application that depends on pressure distribution (seal leakage, gripper slip, sheath insertion force).

## Functional form

Let $\Gamma \subset \partial\Omega_c$ denote the intended contact surface — a designated subset of the cavity's inner surface, from the `SoftScene`'s boundary metadata. At each point $x \in \Gamma$, let $p(x)$ be the contact-pressure scalar — the outward-normal component of the IPC barrier's per-pair traction, summed over pairs touching $x$.

Define a smooth active-contact weight:

$$ w(p) \;=\; \operatorname{logistic}\!\big(\beta_w (p - p_\text{th})\big) $$

with $\operatorname{logistic}(u) = 1/(1+e^{-u})$, pressure threshold $p_\text{th}$ at a small fraction of the material's tensile strength, and sharpness $\beta_w$ chosen so $w$ saturates within $\pm p_\text{th}/2$ of the threshold. (The unsubscripted $\sigma$ is reserved for Cauchy stress across Parts 2–6 per [`appendices/03-notation.md`](../../appendices/03-notation.md), so this leaf names the logistic by word.) The uniformity cost is the weighted coefficient of variation, squared:

$$ \bar p \;=\; \frac{\int_\Gamma w(p)\, p \, dA}{\int_\Gamma w(p) \, dA} \;,\qquad J_\text{unif}(\theta) \;=\; \frac{1}{\bar p^2}\,\frac{\int_\Gamma w(p)\,(p - \bar p)^2 \, dA}{\int_\Gamma w(p) \, dA} $$

Lower is better; the global optimum is $J_\text{unif} = 0$ at perfectly uniform pressure over the contacted region. The reward contribution is $R_\text{unif} = -J_\text{unif}$, passed into [composition](04-composition.md) with its weight.

## Why this form

Three design choices matter.

**Coefficient of variation, not raw variance.** Nondimensional by construction, so the reward weight in composition does not have to be retuned when the operating pressure changes (scaling the cavity, swapping the material). A cavity designed for 50 kPa mean pressure with 5 kPa spread and a cavity designed for 200 kPa mean pressure with 20 kPa spread score identically; the uniformity term measures the shape of the pressure distribution, not its magnitude. This keeps the term composable with the dimensional-analysis framing in [Part 1 Ch 00's nondimensional groups](../00-canonical/00-formulation.md).

**Squared, not standard deviation.** $\sqrt{x}$ is non-smooth at $x = 0$ — its derivative diverges as $x \to 0^+$. Squared CoV is smooth everywhere, including at the global optimum, which is exactly where the optimizer will spend the most iterations. Smoothness at the optimum is a prerequisite for [gradient-augmented BayesOpt](../../100-optimization/02-bayesopt.md) to not blow up on near-converged candidates.

**Smooth active-contact weight $w(p)$, not a hard cutoff.** Non-contacted regions of the intended surface ($p \approx 0$) should not contribute to the uniformity calculation — if they did, the uniformity term would fire on coverage failures and double-count the [coverage term](01-coverage.md). A hard cutoff ($w = \mathbf{1}[p > p_\text{th}]$) is non-differentiable in $\theta$ at the contact boundary; the sigmoidal cutoff preserves differentiability while still approximately separating contacted and non-contacted regions.

## What the solver exports

The term reads two fields that the solver already exports at every converged step:

- **Per-surface-element contact pressure.** The IPC barrier's per-pair normal traction contribution, summed over pairs touching the element, divided by the element area. Available after any converged primal solve.
- **Intended-contact-surface tag.** Per-surface-element boolean from the `SoftScene`'s boundary metadata designating which elements are on $\Gamma$.

No additional solver state is required. Integration against $w(p)$ is done in the reward-evaluation callback after the primal solve converges; per-element contributions are accumulated via a reduction over the surface-element list.

## Gradient path

$J_\text{unif}$ is smooth in $\theta$ through the chain:

$$ \frac{\partial J_\text{unif}}{\partial \theta} \;=\; \int_\Gamma \frac{\partial J_\text{unif}}{\partial p}\,\frac{\partial p}{\partial x^\ast}\,\frac{\partial x^\ast}{\partial \theta} \, dA $$

Outer integral evaluated numerically at surface-element centroids; $\partial p/\partial x^\ast$ from the IPC barrier's VJP ([Part 6 Ch 01 contact-barrier VJP](../../60-differentiability/01-custom-vjps/02-contact-barrier.md)); $\partial x^\ast/\partial \theta$ from the IFT adjoint ([Part 6 Ch 02](../../60-differentiability/02-implicit-function.md)). The factor-on-tape pattern ([Part 5 Ch 00](../../50-time-integration/00-backward-euler.md)) amortizes the adjoint factorization across all four reward terms; this term contributes one RHS to the backward solve.

## Alternatives considered

**Standard deviation rather than CoV.** Preserves a dimensional kPa-of-pressure quantity — interpretable. Rejected because the reward weight would need retuning every time the operating pressure changes, which defeats the scale-invariance the canonical problem's nondimensional framing is built on.

**Max minus min range.** Interpretable, cheap to compute. Rejected because max and min are non-differentiable in the pressure distribution; optimizer would see discontinuous gradients at configurations where a new element takes the role of max or min pressure.

**Entropy of the pressure distribution.** Information-theoretic spread — a flat distribution scores maximum entropy. Rejected because entropy is scale-sensitive on the log scale, weighting low-pressure and high-pressure distributions asymmetrically even at equal spread, which does not match the intended "shape, not magnitude" measurement.

**L⁴ or L⁸ norm deviation.** Would emphasize tail spikes. Rejected because [peak pressure bounds](02-peak-bounds.md) is a separate term with its own barrier; keeping them separated lets composition tune peak weighting independently of uniformity weighting.
