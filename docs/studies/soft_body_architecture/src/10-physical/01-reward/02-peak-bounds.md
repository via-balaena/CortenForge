# Peak pressure bounds

Uniformity and coverage score the distribution of pressure; neither by itself prevents a local pressure spike that damages the cavity material. This leaf pins down the peak-pressure barrier — a smooth penalty that diverges as the peak contact pressure approaches a material-dependent damage threshold, and vanishes when the peak is comfortably below. The barrier machinery mirrors [IPC's contact barrier](../../40-contact/01-ipc-internals/00-barrier.md), which this book commits to under [Part 1 Ch 01 Claim 2](../01-reward.md) ("hard constraints encoded as smooth barriers in the same shape as IPC's contact potential").

## What it measures

A smoothed maximum of the contact pressure over $\Gamma$, penalized as it approaches the material's peak-pressure ceiling $p_\text{max}$. The ceiling is a material property (set from tensile strength, per the [material-database conventions](../../appendices/02-material-db.md)), not a free reward-design parameter; the composition weight on this term scales how much the optimizer fears the ceiling relative to the other reward terms.

## Functional form

Let $p(x)$ be the contact pressure on $\Gamma$, as in [uniformity](00-pressure-uniformity.md). The smoothed maximum is a high-order $L^q$ norm:

$$ \hat p_\text{peak}(\theta) \;=\; \left(\frac{1}{|\Gamma|} \int_\Gamma p(x)^q \, dA\right)^{1/q} $$

with $q$ large enough to approximate the true maximum to a few percent (typical $q \in [8, 16]$). $\hat p_\text{peak} \to \max_{x \in \Gamma} p(x)$ as $q \to \infty$; finite $q$ smooths the max's corner.

The barrier is an inverted IPC form applied to the margin $m = p_\text{max} - \hat p_\text{peak}$. Define a tolerance $\hat m$ — the margin below the ceiling at which the barrier starts to engage — and set:

$$ B_\text{peak}(\hat p_\text{peak}) \;=\; \begin{cases} -(m - \hat m)^2 \,\log(m / \hat m) & \text{if } 0 < m < \hat m \\ 0 & \text{if } m \ge \hat m \\ +\infty & \text{if } m \le 0 \end{cases} $$

The functional form matches the [IPC barrier $b(d)$](../../40-contact/01-ipc-internals/00-barrier.md) with margin $m$ playing the role of gap $d$ and tolerance $\hat m$ playing the role of barrier width $\hat d$. Vanishes at $m = \hat m$ with a $C^2$ transition, grows toward the ceiling at $m \to 0^+$, and strictly diverges at the ceiling.

The reward contribution is $R_\text{peak} = -B_\text{peak}$, passed into [composition](04-composition.md) with its weight.

## Why this form

Four choices matter.

**Smoothed $L^q$ maximum, not true $\max$.** The true $\max_x p(x)$ is non-differentiable at configurations where two points compete for the top pressure; gradient-based optimization would see a non-smooth reward. $L^q$ for large $q$ is smooth everywhere in the pressure field, converges to the true max, and lets the VJP flow through the integral cleanly.

**IPC-shape barrier, not quadratic.** A quadratic penalty $\alpha (\hat p_\text{peak} - (p_\text{max} - \hat m))^2$ activated at a threshold would be $C^0$ (not $C^2$), producing a discontinuous gradient at activation. The IPC $-(m-\hat m)^2 \log(m/\hat m)$ form is $C^2$ and grows smoothly from zero, which Newton-on-reward composition can minimize without the inner-optimization jitter a quadratic would induce.

**Margin $\hat m$ set proportional to the ceiling, not absolutely.** Material-by-material, a silicone's ceiling is set from its tensile strength (Ecoflex 00-50 at ≈2170 kPa per the [Ecoflex mechanical leaf](../04-material-data/00-ecoflex/00-mechanical.md); Dragon Skin grades similarly). Setting $\hat m$ at a fraction of the ceiling (typical: 10–25% of $p_\text{max}$) keeps the barrier's active region scale-consistent across materials without per-material retuning.

**Ceiling from tensile strength, not ultimate compressive strength.** Soft-body failures under contact are tensile-stress-limited rather than compressive; the contact-pressure spike correlates with through-thickness tensile stress on the outer surface of the cavity wall. Using tensile strength as the ceiling tracks the actual failure mode. The choice is documented per-material in the [material database](../../appendices/02-material-db.md).

## What the solver exports

The term reads one field: per-surface-element contact pressure on $\Gamma$ (same as uniformity and coverage). The ceiling $p_\text{max}$ is a `Material`-level property, read once at scene setup from the `MaterialField`'s validity metadata (the [Ecoflex mechanical leaf](../04-material-data/00-ecoflex/00-mechanical.md) stores it in `Material::validity()`).

## Gradient path

The barrier is $C^2$ in $\hat p_\text{peak}$ and $\hat p_\text{peak}$ is smooth in $p$ (derivative of the $L^q$ norm), so the VJP flows cleanly through:

$$ \frac{\partial B_\text{peak}}{\partial \theta} \;=\; \frac{\partial B_\text{peak}}{\partial \hat p_\text{peak}}\,\frac{\partial \hat p_\text{peak}}{\partial p}\,\frac{\partial p}{\partial x^\ast}\,\frac{\partial x^\ast}{\partial \theta} $$

with $\partial \hat p_\text{peak}/\partial p = \hat p_\text{peak}^{1-q} p^{q-1}/|\Gamma|$ per pointwise contribution. One RHS to the shared IFT backward solve.

## Alternatives considered

**Hard indicator — reject any candidate with peak above $p_\text{max}$.** Zero cost to compute. Rejected because indicator rejection is non-differentiable and dangerous for BayesOpt-style optimizers that use gradient information near the boundary; a single over-threshold sample on a rough surface rejects an otherwise near-optimal candidate.

**Log-sum-exp instead of $L^q$ for the smoothed max.** $\hat p = \beta^{-1} \log \int_\Gamma \exp(\beta p) \, dA$. Equivalent to $L^q$ for large $q = \beta$; numerically comparable. $L^q$ preferred here because the book's material database exposes tensile-strength values as magnitudes rather than log-magnitudes, and $L^q$ reads the threshold in the same units.

**Quadratic penalty below threshold + quadratic penalty above.** $C^1$ at the threshold but $C^0$ at the peak, gradient jump at crossover. Rejected because Newton-on-composition reward sees the jump as a stiff region and stalls.

**Linear barrier in the margin.** $B = \max(0, \hat m - m)$. Non-smooth at $m = \hat m$. Rejected on the same smoothness argument that rules out the hard indicator.
