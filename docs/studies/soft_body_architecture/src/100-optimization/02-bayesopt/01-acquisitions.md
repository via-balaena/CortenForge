# Acquisition functions — EI, UCB, Thompson

The parent spine's Claim 3 is that `sim-soft`'s BayesOpt acquisitions tolerate the `GradientEstimate::Noisy { variance }` flag from [Part 7 Ch 04](../../70-sdf-pipeline/04-live-remesh.md) — the noise flag is an API contract, not a diagnostic. This leaf names the three acquisition functions `sim-soft` ships (Expected Improvement as the default, GP-UCB for explicit uncertainty-penalty control, Thompson sampling for parallel batch selection), derives each under the [§00 gradient-enhanced GP](00-gp.md) posterior, and names the inner-loop optimizer that finds the acquisition maximum.

## Expected improvement — the default

[Jones, Schonlau & Welch 1998](https://doi.org/10.1023/A:1008306431147) introduced Expected Improvement (EI) as the acquisition for kriging-based response-surface optimization. EI at $\theta$ is the expected amount by which $R(\theta)$ improves on the best incumbent value $R^\ast = \max_i R_i$:

$$
\alpha_\text{EI}(\theta) = \mathbb E\big[\max(R(\theta) - R^\ast,\ 0)\big],
$$

where the expectation is over the GP posterior at $\theta$. Under the Gaussian posterior $R(\theta) \sim \mathcal N(\mu(\theta), \sigma^2(\theta))$ from [§00](00-gp.md), EI admits a closed form:

$$
\alpha_\text{EI}(\theta) = (\mu(\theta) - R^\ast)\, \Phi(z) + \sigma(\theta)\, \phi(z),
\qquad z = \frac{\mu(\theta) - R^\ast}{\sigma(\theta)},
$$

with $\Phi, \phi$ the standard-normal CDF and PDF. The closed form is what makes EI the default — the acquisition is smooth in $\theta$, has an analytic gradient through the GP's $\mu(\theta)$ and $\sigma(\theta)$, and does not require quadrature.

EI balances exploration and exploitation automatically: $(\mu - R^\ast)\,\Phi(z)$ rewards points with high posterior mean relative to the incumbent; $\sigma \phi(z)$ rewards points with high posterior uncertainty near the incumbent value. Far from the incumbent at low uncertainty, both terms vanish; near the incumbent at high uncertainty, the exploration term dominates. The balance is automatic — no exploration-coefficient knob to tune.

The parent's Section 3 commits to EI consuming the augmented posterior: when a training sample flagged `GradientEstimate::Noisy { variance }` contributes to the GP fit, the posterior $\sigma^2$ at that sample's location inherits the noise variance, so $\sigma(\theta)$ remains large there and EI prefers to re-evaluate rather than treating the sample as exact.

## GP-UCB — explicit exploration-coefficient control

The Upper Confidence Bound acquisition

$$
\alpha_\text{UCB}(\theta) = \mu(\theta) + \beta_t\, \sigma(\theta)
$$

(matching the [parent spine's §3](../02-bayesopt.md) form) was analyzed for GP surrogates by [Srinivas, Krause, Kakade & Seeger 2010](https://arxiv.org/abs/0912.3995). The paper originally framed it as an information-gain-based approach; the community has since standardized on the name GP-UCB. Srinivas et al. prove that a scheduled $\beta_t$ yields sublinear cumulative regret for a broad class of GP kernels, with the regret rate controlled by the kernel's maximum information gain rather than by the input dimensionality directly.

GP-UCB is the acquisition `sim-soft` reaches for when EI's automatic balance is not the balance the project wants. Two situations motivate it:

- **Explicit noise-penalty control.** When the sample set contains high-variance `Noisy` samples near a candidate region, EI's exploration term grows there by construction, but the magnitude is set by the local $\sigma(\theta)$. A larger $\beta_t$ makes GP-UCB more conservative around noisy regions; a smaller $\beta_t$ makes it more aggressive. The knob is explicit, not buried in the posterior.
- **Parallel batching via fantasy updates.** Batch-UCB variants select $b$ candidate points sequentially, each optimizing UCB against a posterior updated with placeholder *fantasy* observations at the previously-selected candidates. The fantasy-update pattern needs the exploration coefficient as a separable scalar, which EI's form does not factor out.

The parent spine's Section 3 noise-inflated UCB variant is the one `Noisy`-flagged samples feed into: the GP posterior's $\sigma(\theta)$ at noisy sample locations already reflects the reported variance, so no additional logic is needed at the acquisition level.

## Thompson sampling — parallel batch selection

Thompson sampling selects $\theta$ by drawing a random function $\widetilde R$ from the GP posterior and optimizing $\widetilde R$ deterministically. For parallel batch selection — choosing $b$ candidates to evaluate simultaneously across multi-process forward-map runners — each worker draws its own $\widetilde R_i$. The draws differ because of the GP posterior's variance, so the workers spread across the design space without needing explicit cross-worker coordination:

1. Each worker draws a function $\widetilde R_i$ from the current GP posterior (sampled on a dense grid, or via a spectral approximation at higher $d$).
2. Each worker runs the inner-loop acquisition optimizer on $\widetilde R_i$ deterministically — the maximizer of that draw is the worker's candidate.
3. When the real evaluations return, the GP posterior is updated with the new samples; the next batch's workers draw from the updated posterior.

Thompson sampling does not explicitly penalize uncertainty — it balances exploration and exploitation only in expectation over the random draw. EI and GP-UCB are preferred for sequential evaluation; Thompson earns its place specifically when parallelism is on the table and the forward-map runners are the bottleneck.

## Inner-loop acquisition optimization

Whichever acquisition is in use, the outer BayesOpt loop needs to solve

$$
\theta_{t+1} = \arg\max_\theta \alpha(\theta)
$$

over the design space. The maximizer is typically non-unique: EI and GP-UCB can peak in several disjoint pockets when the posterior variance is high in many regions, especially in the early stages of the loop. `sim-soft` uses quasi-Newton L-BFGS with multi-start initialization:

- Draw $N$ candidate starting points from the design space via Sobol or Latin hypercube sampling.
- Run L-BFGS from each starting point using the acquisition function's analytic gradient. The gradient is available because the GP's $\mu(\theta)$ and $\sigma(\theta)$ are smooth in $\theta$ and EI / UCB are closed-form expressions in $\mu, \sigma$. Thompson sampling is the exception — each worker's $\widetilde R_i$ is a sampled-function realization rather than a closed-form object; the inner optimizer for Thompson takes the argmax over the grid the sample was drawn on, optionally with single-start local refinement around that grid maximum.
- Return the best maximizer across starts.

The analytic gradient matters: the inner-loop issues many back-substitutions against the cached Cholesky factor from [§00](00-gp.md) — each back-substitution is quadratic-cost, and L-BFGS needing a gradient at every iteration is affordable exactly because the cache is in place. Without the cached factor, multi-start L-BFGS would dominate the BayesOpt loop's wall time.

```rust
use sim_ml_chassis::Tensor;

pub enum Acquisition {
    ExpectedImprovement,
    UpperConfidenceBound { beta: f64 },    // beta_t — coefficient on sigma per spine's form
    Thompson,
}

pub struct AcquisitionOptimizer {
    pub n_starts: usize,                   // multi-start count
    pub lbfgs_max_iter: usize,             // inner L-BFGS step limit per start
    pub acquisition: Acquisition,
}

impl AcquisitionOptimizer {
    pub fn select_next(
        &self,
        gp: &GradientEnhancedGP,
        bounds: &DesignBounds,
    ) -> Tensor<f64> { /* ... */ }
}
```

The `n_starts` and `lbfgs_max_iter` knobs are tuned against the per-evaluation forward-map wall time from [Ch 00](../00-forward.md)'s cost table: the more expensive a single real evaluation is, the more inner-loop budget the acquisition optimizer can afford to spend before it becomes the bottleneck. Since the topology-crossing path can run to $\sim 1.5$ s per evaluation while parameter-only edits run $\leq 70$ ms, the tuning ratio is not a single constant — it tracks the per-`EditClass` evaluation cost the outer loop has already committed to.

## What this sub-leaf commits the book to

- **Expected Improvement (Jones et al. 1998) is the default acquisition.** Closed-form under the Gaussian posterior from [§00](00-gp.md); no exploration coefficient to tune.
- **GP-UCB (Srinivas et al. 2010) is the acquisition for explicit exploration-coefficient control** — for noise-penalty tuning or for parallel batching via fantasy updates. The original analysis is in information-gain terms; the community name GP-UCB is post-hoc.
- **Thompson sampling is the parallel-batch acquisition.** Each worker draws a posterior function and maximizes it; diversity comes from the random draws, not from explicit coordination.
- **Inner-loop acquisition optimization is multi-start L-BFGS on the acquisition's analytic gradient** for EI / UCB, and grid-argmax plus optional single-start local refinement for Thompson; all three paths read from the cached Cholesky factor in [§00](00-gp.md).
- **`Acquisition` enum names EI, UCB, Thompson; `AcquisitionOptimizer` carries the multi-start and L-BFGS-iteration knobs.** The knobs tune against the per-`EditClass` forward-map wall time from [Ch 00](../00-forward.md).
