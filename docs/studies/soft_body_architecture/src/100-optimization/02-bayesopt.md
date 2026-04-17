# Bayesian optimization

`sim-soft`'s Phase D–G commitment is **Bayesian optimization over a gradient-enhanced Gaussian-process prior**, with acquisition functions that tolerate the `GradientEstimate::Noisy` flag from [Part 7 Ch 04](../70-sdf-pipeline/04-live-remesh.md). BayesOpt is the primary optimizer; every alternative evaluated below either under-uses the gradient information the IFT hands us, over-samples the forward map, or fails to coexist with the sim-to-real loop that [Ch 05](05-sim-to-real.md) closes. This is a positive-commitment chapter in the shape of [Part 4 Ch 00 ("why IPC")](../40-contact/00-why-ipc.md) and [Part 5 Ch 00 ("backward Euler on total potential energy")](../50-time-integration/00-backward-euler.md) — one scheme, reasons given, alternatives named and rejected.

| Section | What it covers |
|---|---|
| [Gaussian processes](02-bayesopt/00-gp.md) | GP prior with Matérn-5/2 kernel; ARD length-scales per design-space axis; gradient-enhanced GP ingesting $(\theta_i, R_i, \nabla_\theta R_i)$ triples; posterior update cost $O(n^3)$ in sample count dominates at $n \gtrsim 2000$ |
| [Acquisition functions — EI, UCB, Thompson](02-bayesopt/01-acquisitions.md) | Expected improvement (Jones et al. 1998) as the default; GP-UCB (Srinivas et al. 2010) when variance inflation on noisy samples matters; Thompson sampling for parallel batch selection; acquisition optimization via L-BFGS with multi-start |
| [High-dimensional BayesOpt](02-bayesopt/02-high-dim.md) | Additive kernel decomposition for $d \gtrsim 20$; trust-region BayesOpt (TuRBO, Eriksson et al. 2019) for $d \gtrsim 50$; projections to latent coordinates learned from the first ~200 evaluations as the Phase H scaling path |

Five claims this chapter commits to.

## 1. BayesOpt is the primary optimizer because the expense asymmetry demands it

The forward-map cost table in [Ch 00](00-forward.md) shows per-evaluation wall times of 50 ms to 1.5 s. The design space is $d \in [10, 50]$ for the canonical problem. A BayesOpt loop with 500–2000 samples lands a converged design; gradient descent with line search on a non-convex multi-modal landscape needs multi-start to find anything close to the global optimum, and each random restart is a fresh convergence trajectory of 50–200 evaluations. The sample-efficiency gap is roughly an order of magnitude on canonical-scale problems, and every evaluation is 50–500 ms of solver time.

BayesOpt also consumes gradients natively via the gradient-enhanced GP formulation (Solak et al. 2003, Wu et al. 2017). Most BayesOpt production deployments do not do this because most production problems do not hand the optimizer exact gradients; `sim-soft` does, for 95% of steps, and leaving the gradients on the floor is a category error.

The Shahriari et al. 2016 review ([Taking the Human Out of the Loop](https://ieeexplore.ieee.org/document/7352306)) is the textbook reference for the method's breadth; Frazier 2018's tutorial ([A Tutorial on Bayesian Optimization](https://arxiv.org/abs/1807.02811)) is the textbook reference for the decisions this chapter makes.

## 2. The GP prior is gradient-enhanced

The forward map gives $(\theta_i, R_i, \nabla_\theta R_i)$ on every smooth-gradient sample. A gradient-enhanced GP ingests all three, modeling the joint distribution over values and partials as a single Gaussian via the derivative-kernel identity:

$$ \operatorname{Cov}\!\left(\frac{\partial R}{\partial \theta_a}, \frac{\partial R}{\partial \theta_b}\right) = \frac{\partial^2 k(\theta, \theta')}{\partial \theta_a\, \partial \theta_b'} $$

for a twice-differentiable kernel $k$. The Matérn-5/2 kernel is twice differentiable by construction, so the gradient-augmented kernel matrix is well-defined. Including $d$ gradient components per sample adds $d$ supervision rows per sample; the kernel matrix grows from $n \times n$ to $n(d+1) \times n(d+1)$, so the $O(n^3)$ inversion cost becomes $O(n^3 (d+1)^3)$. For $n = 500$, $d = 20$, that is a $\sim 10{,}500 \times 10{,}500$ Cholesky — at the edge of dense-tractability and a natural fit for block-structured Cholesky on faer.

The payoff: gradient-augmented GPs converge to the optimum in 2–5× fewer scalar-reward evaluations than gradient-free GPs on smooth problems (Wu et al. 2017 reports these ratios on synthetic testbeds; the ratio depends on the smoothness of the target). For the canonical problem where the forward-map gradient is close-to-free relative to the forward-map evaluation, this is strictly a gain.

```rust
use faer::sparse::linalg::solvers::Cholesky;
use sim_ml_chassis::Tensor;

pub struct GradientEnhancedGP {
    pub samples: Vec<GpSample>,             // (theta, r, grad_r, noise_estimate)
    pub length_scales: Tensor<f64>,         // ARD, per-dimension
    pub signal_variance: f64,
    pub factor: Option<Cholesky<f64>>,      // Cholesky of augmented kernel matrix
}

pub struct GpSample {
    pub theta: Tensor<f64>,
    pub r: f64,
    pub grad_r: Option<Tensor<f64>>,        // None on noisy samples if variance > threshold
    pub grad_noise_var: Option<Tensor<f64>>,// per-dimension variance for down-weighting
}
```

The Cholesky factor of the augmented kernel matrix is cached on the GP struct exactly as the Newton-Hessian factor is cached on the tape in [Part 5 Ch 00](../50-time-integration/00-backward-euler.md) — same pattern, different object. Acquisition optimization re-applies the factor many times; every inner step is a back-substitution, not a re-factor.

## 3. Acquisition functions tolerate noisy gradients

The `GradientEstimate::Noisy { variance }` flag from [Part 7 Ch 04](../70-sdf-pipeline/04-live-remesh.md) rides through to acquisition selection. Three adaptations:

- **UCB with variance-inflated noisy samples.** The classical GP-UCB acquisition is $\alpha_{\text{UCB}}(\theta) = \mu(\theta) + \beta_t \sigma(\theta)$ for a scheduled exploration coefficient $\beta_t$. When the posterior $\sigma(\theta)$ is computed including noisy-sample variance contributions, the same acquisition is correctly conservative around samples that crossed a topology boundary. No additional logic is needed; the noise is absorbed into the posterior uncertainty at the sample points.
- **Expected improvement with posterior truncation.** EI computed under the augmented posterior integrates over samples-plus-noise; for large reported variance, the sample's influence on the posterior mean shrinks but its influence on the posterior variance remains. EI therefore prefers to re-evaluate near high-variance samples, which is exactly the right behavior at a topology boundary where the variance is an honest signal that the optimizer does not trust the value.
- **Down-weighted training in info-criteria.** For Ch 04's info-theoretic acquisitions, noisy samples are down-weighted in the information-gain computation by a factor of $1 / (1 + \text{var}/\sigma_\text{signal}^2)$. A sample with infinite reported variance contributes nothing to the posterior update; a sample with zero variance contributes fully. Samples in between interpolate.

No acquisition function is picked ignorant of the noise flag; the flag is an API contract, not a diagnostic.

## 4. Scaling past $d \approx 20$ needs additive or trust-region structure

Classical GP-BayesOpt is known to underperform at high dimension: kernel length-scales become hard to identify, the acquisition function becomes flat, and the sample cost scales poorly in $d$. [The high-dim sub-chapter](02-bayesopt/02-high-dim.md) walks the three approaches `sim-soft` commits to:

- **Additive kernel decomposition.** Assume $R(\theta) = \sum_g R_g(\theta_g)$ for a partition of $\theta$ into groups, each of dimension ≤10. The posterior factors across groups; the acquisition inherits the structure. This works when the canonical design space actually decomposes — SDF parameters don't interact strongly with material-field parameters — which on our problem is mostly true at the group level.
- **Trust-region BayesOpt (TuRBO).** Maintain a local trust region around the best-seen point, run BayesOpt inside it, expand or contract based on progress. Scales to $d \sim 100$ with strong empirical performance on non-convex problems (Eriksson et al. 2019). Phase H scope.
- **Latent-space BayesOpt.** Learn a low-dimensional latent representation of $\theta$ from the first ~200 samples (e.g., via a variational autoencoder on $(\theta, R)$ pairs), then run BayesOpt in the latent space. High risk / high reward; research-frontier; listed in [Part 12 Ch 07](../120-roadmap/07-open-questions.md) as an open question, not a Phase A–I deliverable.

The commitment: additive + trust-region land in Phase G alongside the SDF bridge. Latent-space methods are research.

## 5. BayesOpt composes cleanly with the sim-to-real loop

The sim-to-real loop in [Ch 05](05-sim-to-real.md) models the residual $r(\theta) = R_\text{real}(\theta) - R_\text{sim}(\theta)$ as its own GP over design space, trained on the small number of physically-printed samples. The sim-side BayesOpt runs on $R_\text{sim}(\theta)$; the real-side surrogate corrects it to $\widehat R_\text{real}(\theta) = R_\text{sim}(\theta) + r_{\text{GP}}(\theta)$. The outer acquisition (for choosing the next *print*, not the next sim evaluation) runs on the corrected GP.

The two GPs coexist because they live at different scales: the sim GP has $n_\text{sim} \sim 10^3$ samples, the real GP has $n_\text{real} \sim 10^1$–$10^2$ samples. The composition is one posterior-mean addition and one variance addition per evaluation — cheap. A non-GP surrogate (e.g., a neural net) would not admit this kind of composition without re-training the entire surrogate on the combined dataset.

The `sim-ml-chassis` crate exposes both surrogates as `Surrogate` trait objects; the acquisition function reads from whichever one matches the current question. This clean composition is part of why the book's commitment is "GP first, neural surrogate only where GP scales break" — the GP is the part that the rest of the platform composes with. Neural surrogates are opaque at composition.

## What this commits downstream

- **[Ch 03 preference learning](03-preference.md)** uses a *preference GP* — the same GP machinery as this chapter, but with a pairwise-comparison likelihood in place of the Gaussian likelihood on scalar $R$. The kernel and hyperparameter infrastructure is shared.
- **[Ch 04 active learning](04-active-learning.md)** reads the GP posterior's epistemic uncertainty for information-gain acquisitions. The noise-down-weighted training from §3 is what keeps info-gain sensible on noisy samples.
- **[Ch 05 sim-to-real](05-sim-to-real.md)** reuses the GP library to model the sim-real residual (§5 above).
- **[Ch 06 full loop](06-full-loop.md)** runs BayesOpt as the inner optimizer at the sim scale and as the outer optimizer at the print scale — the two loops are structurally the same loop at different budgets.
- **[Part 11 Ch 02 coupling boundaries](../110-crate/02-coupling.md)** have `Surrogate` trait objects in the `sim-ml-chassis` boundary; GP and neural surrogates both implement the trait, and downstream consumers are backend-agnostic.

## Alternatives rejected

**Gradient descent / L-BFGS only.** Direct gradient descent is sample-efficient per iteration on smooth convex problems but catastrophically sample-inefficient on the canonical problem's multi-modal landscape — the optimizer converges to whichever basin it started in, and multi-start with $\sim 10$ restarts pays 10× the wall time for coverage that BayesOpt gets from its posterior variance. Rejected as the primary optimizer; retained as the *inner* acquisition-optimization step (optimize EI or UCB w.r.t. $\theta$ via L-BFGS, multi-start).

**CMA-ES and other evolutionary methods.** CMA-ES is robust, derivative-free, and well-understood at moderate dimension. It is also sample-inefficient — thousands of evaluations for the same problems BayesOpt solves in hundreds. The `sim-opt` crate already ships SA and PT; CMA-ES would extend it naturally as a fallback for design spaces where the gradient is unusably noisy. Shipped as an option in the `sim-opt` menu, not as the primary optimizer. See `sim-opt`'s crate-level spec for the existing derivative-free baseline.

**RL from scratch.** Learning a policy $\pi(\theta | \text{history})$ via RL on the design task has theoretical appeal and is where some of the differentiable-physics literature points ([DiffTaichi](../00-context/02-sota/04-difftaichi.md), Genesis). For `sim-soft`'s actual numbers — hundreds to thousands of evaluations at seconds each — RL's sample cost is 2–4 orders of magnitude too high. RL is the right tool when the design problem is *itself* a sequential decision-making problem; ours is not. Rejected.

**Hyperband / BOHB.** Designed for hyperparameter tuning where cheap partial evaluations are informative. `sim-soft`'s forward map does not admit a cheap-partial-evaluation regime — a partially-converged Newton solve is not a cheap noisy estimate of the converged $R$, it is a biased wrong answer. Rejected; the mechanism does not apply here.

**Pure surrogate-only optimization.** Fit a neural surrogate, run gradient descent on the surrogate, ignore the real forward map after initial training. Rejected because the surrogate can be arbitrarily wrong at regions it has not sampled; the peak-pressure-barrier heavy tail from [Ch 00's landscape section](00-forward.md) is exactly the kind of feature a surrogate under-fits. Surrogates amortize inner-loop acquisition; they never replace the ground-truth evaluation in the outer loop.
