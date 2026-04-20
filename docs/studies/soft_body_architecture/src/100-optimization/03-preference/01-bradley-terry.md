# Bradley-Terry model

The Bradley–Terry framework is the classical statistical model for paired-comparison data. Preference learning in `sim-soft` uses a GP-prior-plus-probit-likelihood formulation in [§00](00-gp-pairs.md); the Bradley–Terry path of this sub-leaf is the alternative the book exposes for the multi-designer tournament setting, where the primary signal is not a single well-calibrated designer's rating session but aggregated pairs from several designers with heterogeneous calibrations.

## The original Bradley & Terry 1952 model

[Bradley & Terry 1952](https://doi.org/10.1093/biomet/39.3-4.324) — "Rank Analysis of Incomplete Block Designs: I. The Method of Paired Comparisons", Biometrika 39(3-4):324–345 — introduces the parametric model

$$
P(i \succ j) = \frac{\pi_i}{\pi_i + \pi_j}
$$

for a set of items $\{1, \dots, N\}$ with positive strength parameters $\pi_i > 0$. The parameters are identified up to a shared multiplicative scale; a common reparameterization $\pi_i = \exp(s_i)$ turns the pairwise probability into a sigmoid of the latent-strength difference:

$$
P(i \succ j) = \frac{1}{1 + \exp\!\big(-(s_i - s_j)\big)} = \sigma(s_i - s_j).
$$

This is the **sigmoid / logit** form that distinguishes Bradley–Terry from the **probit** form of Chu & Ghahramani 2005 used in [§00](00-gp-pairs.md). The two likelihoods are numerically close (both monotonic in the latent-strength difference, both symmetric about zero, both bounded in $[0,1]$) but arise from different noise models — Gaussian additive noise on the latent for probit, logistic additive noise for logit — and have different tail behaviour that becomes relevant when a large fraction of pairs sit near the decision boundary or when calibration varies across raters.

The original 1952 paper treats the $\pi_i$ as independent parameters fit by maximum likelihood over an observed pair-count matrix. With $N$ items and $\binom{N}{2}$ possible pair types, the model is parametrized by $N{-}1$ degrees of freedom after fixing the scale; the paper's examples are small-$N$ ranking problems where the finite-parameter MLE is the natural fit.

## GP prior over Bradley–Terry strengths

The design-space extension that `sim-soft` commits to places a GP prior on the latent strengths: $s : \mathcal W \to \mathbb R$ with $s \sim \mathcal{GP}(0, k(w, w'))$ over the weight-vector space $\mathcal W$ per [Ch 00](../00-forward.md). Under this prior the observed preference likelihood for a pair $(w_A, w_B)$ becomes

$$
P(w_A \succ w_B \mid s) = \sigma\!\big(s(w_A) - s(w_B)\big),
$$

the logit analogue of Chu & Ghahramani's probit likelihood. The GP prior turns the $N$-parameter problem of the original Bradley–Terry into a function-space problem — the strength function is defined over the continuous weight space, not just the finite set of items rated, so predictions extend to unrated $w$ via the GP conditional rule.

The kernel-family and ARD-length-scale choices follow the same logic as [§00](00-gp-pairs.md): Matérn-5/2 ARD matches the $C^2$ smoothness expected of the subjective strength function over the weight space. Hyperparameters are fit independently per data source; the design choice that distinguishes Bradley–Terry-GP from the §00 probit-GP is the *likelihood shape*, not the *prior*. The GP-extension of Bradley–Terry is the modern augmentation, not part of Bradley & Terry's own 1952 contribution.

## Inference with the sigmoid likelihood

The sigmoid likelihood is non-conjugate to the Gaussian prior, as the probit is — so the inference path is again Laplace approximation at the mode of the log-posterior. The Newton loop's inner step uses the sigmoid's first and second derivatives:

$$
\frac{\partial \log \sigma(x)}{\partial x} = 1 - \sigma(x), \qquad
\frac{\partial^2 \log \sigma(x)}{\partial x^2} = -\sigma(x)\,(1 - \sigma(x)).
$$

The per-pair second-derivative magnitude $\sigma(\cdot)(1-\sigma(\cdot))$ takes values in $(0, 1/4]$, bounded above — a convenient property when the rating matrix spans a wide range of preference strengths across multiple designers, since each pair's contribution to the Laplace-Hessian matrix is bounded regardless of how highly-separated the pair's latent-utility difference is. The iteration matrix itself has the same pair-coupled structure as the probit case in [§00](00-gp-pairs.md): each pair contributes a rank-2 update coupling its two training points, and the Cholesky factor is cached at convergence for predictive evaluation.

## Softening for transitivity violations

Multi-designer rating data typically violates strict transitivity: designer $D_1$ says $A \succ B$, designer $D_2$ says $B \succ C$, designer $D_3$ says $C \succ A$. A strict Bradley–Terry fit on the aggregated pair count fits a consistent latent-strength ordering to data that is not, in aggregate, consistently ordered — the MLE still exists but the data likelihood at the fit is low and the posterior uncertainty is correspondingly wide. The softening strategy `sim-soft` commits to is to **inflate the noise-scale** via a width parameter $\tau$ in the likelihood:

$$
P(w_A \succ w_B \mid s) = \sigma\!\left(\frac{s(w_A) - s(w_B)}{\tau}\right).
$$

Large $\tau$ flattens the likelihood toward $0.5$ for moderate strength differences — the model still fits the dominant-preference signal but does not treat intransitive triads as hard contradictions. $\tau$ is fit from the marginal likelihood alongside the kernel hyperparameters; a high fitted $\tau$ is a diagnostic signal that the aggregated data carries heterogeneous calibrations rather than a single consistent utility.

An alternative softening route is to keep $\tau$ fixed and inflate the **kernel nugget** instead, absorbing designer-level variability into a random-effects term via per-designer residuals. The nugget path is more expressive but requires a richer data model (designer-ID features per pair); Phase-I's commitment is to the scalar-$\tau$ path for implementation simplicity, with the random-effects extension flagged as a post-Phase-I refinement if the tournament data warrants it.

## When to pick Bradley–Terry over the probit path of §00

The probit formulation of [§00](00-gp-pairs.md) is the default for single-designer rating sessions — one well-calibrated rater, Gaussian-noise judgment model, relatively few intransitive triads. The Bradley–Terry formulation of this section is the tournament-setting alternative — multiple designers, calibration heterogeneity, intransitive aggregate data, larger pair count overall. The two formulations share the GP prior and the Matérn-5/2 ARD kernel family; they differ as follows.

| Property | Probit (§00) | Bradley–Terry (§01) |
|---|---|---|
| Likelihood | $\Phi\big((f_A - f_B)/\sqrt 2\, \sigma_n\big)$ | $\sigma\big((s_A - s_B)/\tau\big)$ |
| Noise model | Gaussian-additive on latent | Logistic-additive on latent |
| Typical pair count | $10^1$–$10^2$ (one session) | aggregated over sessions/designers |
| Softening parameter | Per-pair $\sigma_n$ | Scalar $\tau$ (or per-designer random effects) |
| Primary regime | One designer, one session | Multi-designer tournament |

For Phase-I's synthetic demo (no designer rating yet), the distinction is academic; for the post-Phase-I physical-print loop with real designer input, the session pattern determines which path `sim-soft` instantiates.

## The struct

```rust
use faer::sparse::linalg::solvers::Llt;
use sim_ml_chassis::Tensor;

pub struct BradleyTerryGP {
    pub training_inputs: Vec<Tensor<f64>>,       // one w per distinct rated design
    pub pairs: Vec<TournamentPair>,              // pairs with optional designer-ID
    pub length_scales: Tensor<f64>,              // ARD, per weight-axis
    pub signal_variance: f64,
    pub softening_tau: f64,                      // tau in the softened sigmoid likelihood
    pub nugget: f64,
    pub factor: Option<Llt<f64>>,                // Cholesky factor of the Laplace Hessian matrix
    pub laplace_mode: Option<Tensor<f64>>,       // hat-s at training inputs
}

pub struct TournamentPair {
    pub preferred_idx: usize,                    // index into training_inputs
    pub dispreferred_idx: usize,
    pub designer_id: Option<u32>,                // None collapses to single-pool; else rater class
}

impl BradleyTerryGP {
    /// Newton loop on the softened-sigmoid log-posterior; caches factor and mode.
    pub fn fit(&mut self) { /* ... */ }

    /// Posterior mean and variance of the latent strength at a new w.
    pub fn predict(&self, w: &Tensor<f64>) -> GpPrediction { /* ... */ }
}
```

The `designer_id` field is the hook for the random-effects extension; scalar-$\tau$ softening (the Phase-I default) ignores the field, and the random-effects extension partitions pairs by it to fit per-designer residuals.

## What this sub-leaf commits the book to

- **Bradley–Terry-GP formulation: sigmoid/logit likelihood over pairs with a GP prior on latent strengths $s$.** Distinct from §00's probit likelihood; same Matérn-5/2 ARD kernel family selected independently.
- **Faithful attribution.** Bradley & Terry 1952's own contribution is the $N$-parameter MLE model $P(i \succ j) = \pi_i / (\pi_i + \pi_j)$ for a finite set of items; the exponential reparameterization $\pi_i = \exp(s_i)$ and the GP extension over a continuous weight space are modern augmentations the book commits to, not the 1952 paper's.
- **Transitivity-violation softening via scalar $\tau$.** Inflates the likelihood width to tolerate intransitive aggregate data; per-designer random-effects extension deferred to post-Phase-I.
- **Laplace inference with logit-specific Hessian.** Per-pair second-derivative magnitude $\sigma(\cdot)(1-\sigma(\cdot)) \in (0, 1/4]$ — bounded contribution per pair to the Laplace-Hessian matrix, which helps conditioning when the rating matrix spans a wide range of preference strengths.
- **Regime: multi-designer tournament with heterogeneous calibration.** Single-designer session is §00's regime; tournament aggregation is this section's. Both real-data paths are post-Phase-I.
