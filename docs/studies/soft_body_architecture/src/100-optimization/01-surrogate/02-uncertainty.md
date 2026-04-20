# Uncertainty quantification

The surrogate is consumed by [Ch 02 BayesOpt](../02-bayesopt.md) and [Ch 04 active learning](../04-active-learning.md), both of which integrate against the surrogate's *posterior variance* — acquisition functions like Expected Improvement, UCB, and BALD read $\sigma^2$ as load-bearing input, not as a monitoring statistic. A surrogate that produces good point predictions and bad uncertainty breaks every acquisition function downstream. This leaf names the primary uncertainty estimator (deep ensembles), the cheaper fallback (MC-dropout), and the calibration diagnostic that gates retraining and switches between the two.

## Deep ensembles: the default estimator

Given $M$ members from [§00](00-architecture.md) trained via [§01](01-training.md), the ensemble predicts $\mu(\theta)$ and $\sigma^2(\theta)$:

$$
\mu(\theta) = \frac{1}{M} \sum_{m=1}^M \widehat R_{\phi_m}(\theta), \qquad
\sigma^2(\theta) = \frac{1}{M} \sum_{m=1}^M \big(\widehat R_{\phi_m}(\theta) - \mu(\theta)\big)^2.
$$

The predictive distribution is approximated as Gaussian with mean $\mu$ and variance $\sigma^2$. On training-distribution points, $\sigma^2$ is small — members agree because training pulled them toward the same data. On out-of-distribution points, $\sigma^2$ grows — members' predictions diverge because nothing pulled them toward a shared answer in that region. That divergence is what acquisition functions need: points with high $\sigma^2$ are points where visiting reduces the surrogate's disagreement.

[Lakshminarayanan, Pritzel & Blundell 2017](https://arxiv.org/abs/1612.01474) demonstrated empirically that deep ensembles produce better-calibrated uncertainty than MC-dropout on a range of regression and classification benchmarks. That result motivates ensembles as the default; it is not a universal theorem, and ensembles can still miscalibrate — the calibration diagnostic below exists because the default is a *good baseline*, not a certified-correct one.

## MC-dropout: the cheap fallback

If the $M$-fold training cost of an ensemble is too expensive for the project's compute budget, the fallback is MC-dropout (Gal & Ghahramani 2016): train a single network with dropout active, and at inference time run $K$ stochastic forward passes with dropout still enabled. Take the sample mean and sample variance across the $K$ passes as the predictive distribution.

Training cost is $1/M$ of the ensemble (one network vs $M$). Inference cost is comparable when $K \approx M$ (possibly slightly higher for dropout due to the stochastic-mask overhead on each pass), so savings are primarily in training.

The trade-off is uncertainty quality: Lakshminarayanan et al.'s benchmarks reported deep ensembles outperforming MC-dropout on out-of-distribution test points — MC-dropout's predictive variance grew less sharply on OOD inputs than the ensemble's, which is exactly the regime the acquisition function depends on for exploration. A MC-dropout surrogate biases the BayesOpt feedback loop toward premature exploitation: the optimizer stops exploring before the ensemble-based estimate would have said the surrogate has run out of information.

MC-dropout is wired behind a configuration switch as a budget-constrained fallback, not the default.

## Calibration diagnostic

Regardless of which estimator is in use, the surrogate's uncertainty is checked against held-out real-evaluation data via a reliability diagram, following the approach [Guo, Pleiss, Sun & Weinberger 2017](https://arxiv.org/abs/1706.04599) established for classification calibration and here adapted to the regression setting.

On a held-out set, the surrogate emits $(\mu_i, \sigma_i^2)$ predictions. Under the Gaussian predictive approximation, the standardized residuals $z_i = (R_i - \mu_i) / \sigma_i$ should be approximately standard-normal if uncertainty is calibrated. Two diagnostics:

- **Interval-coverage reliability:** for each predicted coverage level $p \in \{50\%, 68\%, 90\%, 95\%\}$, measure the empirical fraction of held-out points falling inside the predicted interval $\mu_i \pm q_p \sigma_i$ (where $q_p$ is the standard-normal quantile for level $p$). Plot predicted coverage (x) vs empirical coverage (y); a calibrated surrogate lies on the diagonal. Under-coverage (empirical below predicted) indicates overconfidence; over-coverage, underconfidence.
- **Empirical-CDF vs standard-normal:** plot the empirical CDF of the $z_i$ against $\Phi(z)$. Systematic shape deviations surface asymmetries or tail misbehavior that interval-coverage averages over.

If the diagnostic reports systematic miscalibration, a post-hoc correction rescales the variance by a single scalar $\tau$ fit on the held-out set — the regression analogue of Guo et al.'s classification temperature scaling. Replace $\sigma_i$ with $\tau \sigma_i$ and re-check. If no single $\tau$ brings the diagram onto the diagonal, scalar post-hoc scaling is insufficient, and the calibration-failure flag triggers [§01](01-training.md) retraining on the current sample set.

```rust
use sim_ml_chassis::Tensor;

pub struct CalibrationReport {
    /// (predicted coverage, empirical coverage) pairs at standard levels.
    pub interval_coverage: Vec<(f64, f64)>,
    /// supremum of |F̂(z) - Φ(z)| on the held-out residuals.
    pub ecdf_max_deviation: f64,
    /// Fitted scalar rescaling of σ; 1.0 if already calibrated.
    pub temperature: f64,
    /// Diagnostic pass/fail — false triggers retraining per §01.
    pub calibrated: bool,
}
```

## What this sub-leaf commits the book to

- **Deep ensembles are the default uncertainty estimator** — mean + empirical variance across $M$ members, approximated as Gaussian for the downstream acquisition functions.
- **MC-dropout is the fallback, wired behind a configuration switch.** Single network with $K$ stochastic forward passes; cheaper in training but known to under-cover at the OOD boundary.
- **Calibration is diagnosed via interval-coverage reliability and empirical-CDF comparison** against standard-normal quantiles, adapting the Guo et al. 2017 reliability-diagram framework from classification to regression.
- **Post-hoc temperature scaling with a single $\tau$** corrects uniform miscalibration; if scalar $\tau$ fails, [§01](01-training.md) retraining is triggered.
- **`CalibrationReport` is what the retrain-scheduler's trigger from §01 consumes.**
