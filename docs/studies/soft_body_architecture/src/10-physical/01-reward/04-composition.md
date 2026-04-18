# Multi-objective composition and weighting

The four reward terms — [pressure uniformity](00-pressure-uniformity.md), [contact coverage](01-coverage.md), [peak-pressure bounds](02-peak-bounds.md), and [effective-stiffness bounds](03-stiffness-bounds.md) — each measure a different failure mode. Composing them into a single scalar the optimizer can minimize is a design choice with real consequences: the weighting implicitly picks which failure modes the optimizer can trade off against which others. This leaf pins down the composition rule.

## Three composition modes

`sim-soft`'s reward framework supports three modes. Each mode reads the same four per-term scalars and produces a different optimizer-facing signal.

### 1. Fixed-weight linear composition

The default. The scalar reward is a fixed-weight sum:

$$ R(\theta) \;=\; w_u \, R_\text{unif} + w_c \, R_\text{cov} + w_p \, R_\text{peak} + w_k \, R_\text{stiff} $$

with per-term reward contributions as defined in each sibling leaf. All four weights are non-negative; scaling them uniformly rescales the reward without changing the optimum, so a canonical normalization fixes $\sum_i w_i = 1$ or holds one weight fixed.

The canonical-problem baseline is the equal-weight configuration $w_u = w_c = w_p = w_k = 1/4$, per the [Part 12 Ch 06 optimization milestone](../../120-roadmap/06-optimization.md). Domain users override per [Part 1 Ch 00 related-problems map](../00-canonical/01-related-problems.md) — a seal weights peak-pressure more, a compression sleeve weights stiffness more.

### 2. Pareto front

The optimizer produces a front over the four per-term rewards rather than a single scalar. A pairwise-dominance ordering decides feasibility:

$$ \theta_1 \succeq \theta_2 \iff R_i(\theta_1) \ge R_i(\theta_2) \text{ for all } i \in \{u, c, p, k\} $$

and the front is the set of $\theta$ that no other $\theta'$ dominates. The domain user picks an operating point on the front post-hoc.

Useful when the relative weighting is genuinely unknown — i.e., when the application has competing requirements and the weighting is itself a design decision. `sim-opt` ([Part 10](../../100-optimization/00-forward.md)) can expose a Pareto-front sampler that feeds the [BayesOpt acquisition](../../100-optimization/02-bayesopt.md).

### 3. Preference-learning alternative

Instead of fixing the weights up front, [Part 10 Ch 03 preference learning](../../100-optimization/03-preference.md) infers them from pairwise ratings of candidate cavities. The domain user is shown two candidates at a time and picks the preferred one; a Bradley–Terry-model GP fits an implicit reward function consistent with the ratings, and the acquisition function switches to the GP-inferred reward once enough ratings have accumulated.

Useful for wearable and medical applications where the best-conformity criterion is the domain user's subjective judgment (fit quality of a sheath or mask, feel of a gripper grip). Defers the weight-specification step past the reward-definition step.

## Choosing a mode

The mode is a scene-setup-time decision, not a runtime branch. The three modes compose differently:

| Mode | When to use | Optimizer interface |
|---|---|---|
| Fixed weights | Domain user knows the relative priorities | Single scalar reward; any scalar optimizer works |
| Pareto front | Relative priorities unknown or under study | Vector reward; specialized Pareto-aware optimizer |
| Preference learning | Priorities only expressible via human comparison | Single scalar reward via GP-inferred weights |

The canonical-problem runs documented in [Part 12 Ch 06](../../120-roadmap/06-optimization.md) default to fixed weights. The [sim-to-real milestone](../../100-optimization/05-sim-to-real.md) preserves the composition mode — sim-to-real correction adjusts the individual reward terms, not the composition rule.

## What makes the composition smooth

The [parent chapter's Claim 2](../01-reward.md) committed each individual reward term to be smooth in $\theta$; a weighted sum of smooth functions is smooth, so the fixed-weight composition is smooth by construction. The Pareto front is not a smooth scalar but a vector-valued reward; smoothness is preserved per-coordinate, which is what multi-objective optimizers consume.

Preference-learning mode builds the reward from a GP fit, which is smooth by the GP kernel's construction ([Part 10 Ch 03 preference](../../100-optimization/03-preference.md)).

The shared factor-on-tape across all four reward terms ([Part 5 Ch 00](../../50-time-integration/00-backward-euler.md)) pays off most under fixed weights — one backward solve handles the composed scalar, not four independent backward solves.

## Why not a more complex composition

Three richer composition rules were considered and rejected at Pass 1.

**Product composition** ($R = \prod_i R_i^{w_i}$). Monotone in each term, naturally multiplicatively normalized. Rejected because a single zero-valued term collapses the product to zero and makes the gradient in every other direction vanish as well — any direction's derivative has a factor of the zero term and evaluates to zero, so the optimizer loses all directional information at configurations where any one reward is zero. Early-iteration optimization — where some terms are typically near zero — would be ill-conditioned.

**Min-over-terms composition** ($R = \min_i w_i R_i$). Emphasizes the worst-performing term. Rejected because $\min$ is non-differentiable exactly where the optimizer needs to distinguish which term is limiting, which defeats the point of a gradient-aware reward.

**Learned composition** (neural-network reward head). Generic, flexible. Rejected because the book's reward framework is intentionally opaque to its optimizer — the optimizer does not need to know how the reward is composed, only that the reward is smooth and differentiable. A learned head would need its own calibration data and would bury the specific conformity-failure-mode decomposition the four-term structure makes explicit.

## Interaction with sim-to-real

The [sim-to-real correction](../../100-optimization/05-sim-to-real.md) ingests measured-print data and produces per-term residual adjustments to the sim-side reward. Composition mode is preserved: fixed-weight composition continues to use fixed weights on the corrected reward terms; Pareto-front mode continues to build a front on the corrected vector; preference-learning mode continues to fit the GP on the corrected scalar. Sim-to-real does not change the composition rule.

The four-term decomposition makes per-term correction sensible — each physical-print measurement channel (force curve, pressure-field rubbing, diffusion profile, stiffness-from-force-curve) corresponds cleanly to one reward term, so residual GPs fit per-term without reward-composition entanglement.
