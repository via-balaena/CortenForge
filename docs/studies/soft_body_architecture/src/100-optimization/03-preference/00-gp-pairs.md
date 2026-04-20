# GP over pairwise ratings

The parent spine's Claim 3 is that preference learning runs its own GP inference path — separate kernel machinery and hyperparameters from the BayesOpt GP of [Ch 02 §00](../02-bayesopt/00-gp.md) — and plugs into the composite reward at the weight-slot composition point, not at the inference layer. This leaf names why a GP surrogate is the right choice for preference data at the budget a designer can produce in a single rating session, how a probit likelihood over pairs reshapes the inference from the closed-form case of Ch 02, and what the Laplace approximation buys in exchange for losing closed-form conjugacy.

## Latent utility over pairs — why a GP prior

The designer does not supply absolute scalar ratings for every design; they supply *pairwise* comparisons ("A feels more compliant than B") between rated realizations. The standard modelling move is to posit a latent utility function $f$ over an input space $\mathcal W$ — the space of weight vectors in the composite reward per [Ch 00](../00-forward.md)'s $\theta$ breakdown — and declare the observed preference $w_A \succ w_B$ a noisy observation of the sign of $f(w_A) - f(w_B)$. Under that framing, the inference problem is to recover the posterior over $f$ given the pairs — and a Gaussian-process prior on $f$ is the natural structured prior: it gives a full function-space posterior from a typical rating-session budget of $10^1$–$10^2$ pairs without forcing a parametric form on $f$ in advance.

The GP prior on $f$ is specified by a kernel $k(w, w')$ in $\mathcal W$. The kernel-family choice can be the same Matérn-5/2 ARD form the BayesOpt GP in [Ch 02 §00](../02-bayesopt/00-gp.md) uses — the prior expectation on designer preferences is that the subjective utility varies smoothly enough in weight space to admit a $C^2$-sample-path prior, so the kernel family Ch 02 §00 justifies for the physical reward is a reasonable default here too — but the hyperparameters ($\sigma_f^2$, $\{\ell_a\}$, nugget) are fit against the preference data's own marginal likelihood, separately from the BayesOpt GP's. The two GPs share a kernel *family*, not a *factorization* and not a joint hyperparameter fit.

## The probit likelihood — Chu & Ghahramani 2005's formulation

[Chu & Ghahramani 2005 (ICML)](https://icml.cc/Conferences/2005/proceedings/papers/018_Preference_ChuGhahramani.pdf) is the canonical paper on preference learning with GP priors. Their likelihood model for a single pair $(w_A, w_B)$ with observed preference $w_A \succ w_B$ uses the standard-normal CDF $\Phi$:

$$
P(w_A \succ w_B \mid f) = \Phi\!\left(\frac{f(w_A) - f(w_B)}{\sqrt{2}\, \sigma_n}\right),
$$

where $\sigma_n$ is a noise-scale parameter absorbing calibration noise in the designer's judgment. The **probit** form — as opposed to the **sigmoid/logit** form that Bradley–Terry-style models in [§01](01-bradley-terry.md) use — arises naturally from the latent-utility-plus-Gaussian-noise story: the rating noise is modelled as additive Gaussian on $f$, and integrating over the noise turns the "sign of difference exceeds noise" condition into $\Phi(\cdot)$.

Per Claim 1 of the parent spine, preference data is the only honest way to learn the subjective components of $w$; the probit + GP-prior combination lets that data feed a full posterior over $f$ — and hence a predictive posterior over what preference ordering a yet-unseen pair would elicit — rather than just a point estimate of $\hat w$.

## Posterior inference via Laplace approximation

Unlike the Gaussian-likelihood case of [Ch 02 §00](../02-bayesopt/00-gp.md), the preference posterior $p(f \mid \text{pairs})$ is *not* Gaussian in closed form: the probit likelihood $\Phi((\cdot)/\sqrt 2\, \sigma_n)$ is non-conjugate to the Gaussian prior on $f$. Chu & Ghahramani use a **Laplace approximation** — fitting a Gaussian to the posterior at its mode — as the primary inference path. The Laplace step is:

1. Solve $\hat f = \arg\max_f \log p(f \mid \text{pairs})$ via Newton iterations on the stacked function-value vector at training points.
2. Evaluate the Hessian $H = -\nabla^2 \log p(f \mid \text{pairs})\big|_{\hat f}$ at the mode.
3. Report the approximate Gaussian posterior with mean $\hat f$ and covariance $H^{-1}$ at the training points; extend to test points via the GP's conditional rule.

Each Newton iteration factorizes a Hessian-structure matrix that combines the kernel matrix $K$ over the distinct training inputs with a symmetric contribution $\Lambda$ assembled from the per-pair second derivatives of the probit log-likelihood — each rated pair contributes a rank-2 update coupling its two training points, so $\Lambda$ has off-diagonal entries where single-item-likelihood GPC Laplace has a pure diagonal. At convergence, the factor is cached for posterior-mean and posterior-variance evaluation — the same factor-once, back-substitute-many pattern as the BayesOpt GP, but on a different matrix with different inner-loop cost. Expectation propagation is a known alternative inference scheme for non-Gaussian-likelihood GPs; `sim-soft` commits to Laplace as the default and leaves EP as a knob if the pair count grows past the point where the mode-centered Gaussian misfits the posterior's tails.

## Scaling regime and cost

The dominant per-refit cost is the $O(n_\text{train}^3)$ Cholesky factor inside the Newton loop, where $n_\text{train}$ is the number of distinct training designs — not the number of pairs, since multiple pairs share the function values at the designs they reference. A typical design-session budget of $10^1$–$10^2$ rated designs is well within what a dense Cholesky on a single machine absorbs without effort; the Newton loop converges in a handful of iterations for the probit likelihood.

The parent spine's "scales to $n_\text{prefs} \sim 10^3$" gestures at the regime beyond what a single-session designer produces — aggregating preferences across multiple designers or multiple sessions. The Chu & Ghahramani paper does not commit to a specific tractability threshold at $10^3$; the number in the spine is an order-of-magnitude upper bound consistent with single-machine dense Cholesky on the training-design space, not a result cited from the paper. Past $n_\text{train} \gtrsim 10^3$ the story is the same as for the BayesOpt GP — sparse-GP approximations take over — but that regime is outside Phase-I's scope and outside this sub-leaf's commitments.

## The struct, with probit-likelihood Laplace inference

```rust
use faer::sparse::linalg::solvers::Llt;
use sim_ml_chassis::Tensor;

pub struct PreferenceGP {
    pub training_inputs: Vec<Tensor<f64>>,       // one w per distinct rated design
    pub pairs: Vec<PreferencePair>,              // observed preferences
    pub length_scales: Tensor<f64>,              // ARD, per weight-axis
    pub signal_variance: f64,                    // sigma_f^2
    pub noise_scale: f64,                        // sigma_n in the probit likelihood
    pub nugget: f64,                             // diagonal jitter for conditioning
    pub factor: Option<Llt<f64>>,                // Cholesky factor of the Laplace Hessian matrix
    pub laplace_mode: Option<Tensor<f64>>,       // hat-f at training inputs
}

pub struct PreferencePair {
    pub preferred_idx: usize,                    // index into training_inputs
    pub dispreferred_idx: usize,
}

pub struct GpPrediction {
    pub mean: f64,
    pub variance: f64,
}

impl PreferenceGP {
    /// Refit the Laplace approximation given current (training_inputs, pairs).
    /// Newton loop on log p(f | pairs); caches the Laplace-Hessian factor and the mode.
    pub fn fit(&mut self) { /* ... */ }

    /// Posterior mean and variance of the latent utility at a new w.
    pub fn predict(&self, w: &Tensor<f64>) -> GpPrediction { /* ... */ }
}
```

The `noise_scale` $\sigma_n$ parameter is tuned alongside $(\sigma_f^2, \{\ell_a\})$ by maximizing the Laplace-approximated marginal likelihood. Low $\sigma_n$ treats the designer's preferences as near-deterministic; high $\sigma_n$ softens the likelihood toward a more transitivity-violation-tolerant posterior. [§01](01-bradley-terry.md) is the companion sub-leaf for the multi-designer regime where global softening via a scalar-width parameter is the natural route; this sub-leaf's $\sigma_n$ is the single-designer analogue, fit per rating session.

The `fit` method is called at the start of every designer rating session: new pairs are folded in, the Laplace mode is refit, and the factor is cached for the acquisition loop of [§02](02-dueling.md) to consume. The refit cost is a handful of Newton iterations times a dense Cholesky on an $n_\text{train} \times n_\text{train}$ matrix — for $n_\text{train} \lesssim 10^2$, well under a second.

## Composition with the BayesOpt GP

Per the spine's Claim 3, the composition point with the BayesOpt GP of [Ch 02 §00](../02-bayesopt/00-gp.md) is *not* the inference layer but the `Surrogate::call` trait method at the `sim-ml-chassis` boundary. The preference GP's posterior-mean evaluation yields $\hat w = \arg\max_w\, \mathbb E[f(w)]$ — the weight vector the designer's latent utility peaks over, under the current Laplace approximation. That $\hat w$ slots into the composite reward's weight vector, and the BayesOpt GP then runs over the remaining $\theta$-components with $w = \hat w$ held fixed for the duration of one outer BayesOpt iteration. A full-posterior propagation (marginalizing $w$ out of the BayesOpt problem) is available in principle; Phase-I's default is the MAP-style $\hat w$ substitution for implementation simplicity.

The upshot is two separate GP objects — `PreferenceGP` and the `GradientEnhancedGP` of [Ch 02 §00](../02-bayesopt/00-gp.md) — with independent kernels, hyperparameters, and inference paths, coordinating only at the `Surrogate` trait boundary once per outer BayesOpt iteration.

## What this sub-leaf commits the book to

- **Preference GP: Gaussian process over latent utility $f$ in weight-vector space $\mathcal W$.** Matérn-5/2 ARD kernel family selected independently from the BayesOpt GP; hyperparameters fit on the preference data's own marginal likelihood; no shared factorization.
- **Probit likelihood over pairs per Chu & Ghahramani 2005 (ICML).** $P(w_A \succ w_B \mid f) = \Phi((f(w_A) - f(w_B))/\sqrt 2\, \sigma_n)$; the probit form follows from modelling designer-judgment noise as additive Gaussian on $f$.
- **Laplace approximation for posterior inference.** Newton loop on the log-posterior, Cholesky factor of the Laplace-Hessian matrix cached at the mode, EP left as a fallback knob if the mode-centered Gaussian misfits.
- **Session-scale fit cost: well under a second for $n_\text{train} \lesssim 10^2$.** Dense Cholesky per Newton iteration; handfuls of Newton iterations per refit; one refit per rating session.
- **Composition with BayesOpt at the `Surrogate` trait boundary.** Preference GP's $\hat w$ slots into the composite reward; BayesOpt GP runs over the remaining $\theta$-components with $\hat w$ held fixed per outer iteration.
- **Post-Phase-I integration.** The preference GP's real-designer data source is the physical-print rating loop of [Ch 06](../06-full-loop.md), which is post-Phase-I; the sub-leaf defines the machinery, but the integrated loop lands with the full rating infrastructure.
