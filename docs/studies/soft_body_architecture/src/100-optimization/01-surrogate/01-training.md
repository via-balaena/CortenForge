# Training on sim evaluations

The surrogate is fit to samples emitted by the forward map of [Ch 00](../00-forward.md). Its training loop has two properties unusual relative to a generic MLP regression: (1) most samples arrive with exact gradient labels, not just value labels, and (2) a minority of samples arrive with a known noise variance on the gradient. This leaf names the loss function, its noise-weighted variant, and the retrain schedule.

## Sobolev-style loss: match both $R$ and $\nabla_\theta R$

The forward map exports $(\theta, R, \nabla_\theta R)$ triples on ≈95% of samples (the exact-gradient case from [Ch 00's 95/5 split](../00-forward.md)). The training loss combines a value term and a gradient term:

$$
\mathcal{L}_\text{exact}(\phi) = \frac{1}{|\mathcal D_\text{exact}|} \sum_{(\theta, R, g) \in \mathcal D_\text{exact}} \Big[
    (\widehat R_\phi(\theta) - R)^2 + \alpha \, \lVert \nabla_\theta \widehat R_\phi(\theta) - g \rVert_2^2
\Big]
$$

where $\phi$ are the network parameters, $g = \nabla_\theta R$ is the exact gradient pulled from the autograd tape on the forward pass, and $\alpha$ is a balancing weight. This is the Sobolev-training pattern of [Czarnecki, Osindero, Jaderberg, Świrszcz & Pascanu 2017](https://arxiv.org/abs/1706.04859), which adds first-derivative supervision alongside value supervision. Wu et al. 2017's gradient-enhanced GPs (the GP analogue, used on [Ch 02's BayesOpt surrogate path](../02-bayesopt.md)) share the same structural insight.

The balancing weight $\alpha$ is a fixed hyperparameter placing the value term and gradient term on comparable numerical footing — the two have different numerical magnitudes on typical training data (the gradient's scale depends on the reward's local Lipschitz behavior, the value's on the reward's global range), so the gradient term dominates or vanishes if left uncalibrated. A single $\alpha$ fixed at training-start suffices; dynamic rebalancing is available but not the default.

The supervision advantage is substantial: each exact-gradient sample contributes $d + 1$ signals (one from the value, $d$ from the gradient components) rather than one. At moderate dimension this turns the effective information rate per real evaluation into a multiple of the value-only rate — which is what makes gradient-enhanced surrogates practical at the evaluation-budget scales Part 10 is designed for.

## Noise-weighted loss: samples flagged `GradientEstimate::Noisy`

The ≈5% topology-crossing samples arrive with `GradientEstimate::Noisy { variance }` — the [Part 6 Ch 05 FD wrapper](../../60-differentiability/05-diff-meshing.md) produces a gradient estimate with a known variance, not an exact value. These samples should still contribute — the value label is still exact — but the gradient term should be down-weighted by the reported noise.

Inverse-variance weighting on the gradient term:

$$
\mathcal{L}_\text{noisy}(\phi) = \frac{1}{|\mathcal D_\text{noisy}|} \sum_{(\theta, R, g, \sigma^2) \in \mathcal D_\text{noisy}} \Big[
    (\widehat R_\phi(\theta) - R)^2 + \alpha \cdot \frac{\lVert \nabla_\theta \widehat R_\phi(\theta) - g \rVert_2^2}{\sigma^2 + \epsilon}
\Big]
$$

where $\sigma^2$ is the variance reported on the `GradientEstimate::Noisy` variant and $\epsilon > 0$ prevents division-by-zero for accidentally-small variances. The total loss sums over both subsets:

$$
\mathcal{L}_\text{total}(\phi) = \mathcal{L}_\text{exact}(\phi) + \mathcal{L}_\text{noisy}(\phi)
$$

The value term is never down-weighted — $R$ is always exact on a completed forward-map evaluation, regardless of edit class; it is only the gradient that may be noisy. A sample whose reported $\sigma^2$ is very large contributes to training essentially through its value label alone, which is the right behavior: if the gradient carries no usable information, the gradient supervision should vanish without discarding the value.

## Training an ensemble member

Each of the $M$ ensemble members from [§00](00-architecture.md) is trained independently on the same training set with a different random initialization seed. There is no cross-member coupling in the loss; diversity in test-time predictions comes from initialization diversity propagating through training, not from a diversity-encouraging regularizer and not from per-member bootstrap resamples. Per-member bootstrapping is an optional variant available in the literature but is not the default recommended in [Lakshminarayanan et al. 2017](https://arxiv.org/abs/1612.01474).

```rust
use sim_ml_chassis::Tensor;
use sim_soft::GradientEstimate;

pub struct TrainingSample {
    pub theta: Tensor<f64>,
    pub reward: f64,
    pub gradient: Tensor<f64>,
    pub gradient_estimate: GradientEstimate,   // Exact or Noisy { variance }
}

pub struct SobolevLoss {
    pub alpha: f64,          // fixed at training-start
    pub epsilon: f64,        // variance-denominator floor
}
```

## Retrain schedule: tied to evaluation-budget milestones

The surrogate is retrained periodically rather than updated incrementally, for two reasons. First, gradient-matching training is sensitive to sample distribution, and fine-tuning a stale surrogate on a few new samples biases predictions toward the old-sample region. Second, the training loop needs the full sample set resident at once for the $\alpha$ balancing calibration (§ Sobolev-style loss, above) to remain well-conditioned.

The retrain cadence is tied to evaluation-budget milestones rather than to wall-clock or to sample-count thresholds:

- **Initial training** after the Sobol or Latin-hypercube seed batch has filled the training set — the seed-batch size is determined by [Ch 02's seeding discussion](../02-bayesopt.md).
- **Milestone retrain** each time the training-set size doubles ($n_0, 2 n_0, 4 n_0, \dots$). This is cheap early (small sets train quickly) and rare later (doublings are infrequent once the set is large), so the retrain cost amortizes naturally against evaluation-budget growth without needing a separate schedule knob.
- **Trigger retrain** if the ensemble's calibration diagnostic from [§02 uncertainty](02-uncertainty.md) crosses its flag threshold on the most recent batch of predictions — indicating the surrogate has drifted from the current sample distribution faster than the doubling schedule accommodates.

Between retrains, the surrogate is queried but not updated. Incremental online updates on a per-sample basis are rejected for the two reasons above.

## What this sub-leaf commits the book to

- **Training loss is Sobolev-style** — value term plus gradient term with a fixed $\alpha$ balancing weight — following Czarnecki et al. 2017 (Wu et al. 2017 for the GP analogue).
- **Noisy-gradient samples have their gradient term inverse-variance-weighted; the value term is never down-weighted.** The `GradientEstimate::Noisy { variance }` flag from Part 6 Ch 05 is what the loss consumes.
- **Ensemble members are trained with independent init seeds on the same (non-bootstrapped) training set.** Diversity comes from initialization and loss-landscape stochasticity.
- **Retrain cadence is tied to training-set-size doublings plus a calibration-error trigger.** No incremental per-sample updates; each retrain is a batch operation over the full current sample set.
