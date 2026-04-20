# Information-theoretic criteria

The parent spine frames active learning as picking $\theta$ that maximally reduces the posterior entropy of the reward-surface GP. This leaf names three information-theoretic acquisitions — max-value entropy search (MES), predictive entropy search (PES), BALD — derives each as a mutual-information quantity against a different latent, and commits to which one `sim-soft` ships at the physical-print layer.

## The shared identity

Every acquisition in this family reduces to a single form: mutual information between the candidate query's output $y(\theta)$ and some latent quantity $z$ of interest, expressed as the predictive entropy minus the expected conditional entropy.

$$
\alpha_\text{IG}(\theta) = \mathrm{I}\big[y(\theta);\, z \,\big|\, \mathcal D\big]
= \mathrm{H}\big[p(y(\theta)\mid \mathcal D)\big]
- \mathbb E_{z \mid \mathcal D}\!\big[\mathrm{H}[p(y(\theta) \mid z, \mathcal D)]\big],
$$

where $\mathcal D$ is the current GP training set of real-measurement samples. The three acquisitions differ only in the choice of $z$: the global maximum value $y^\ast = \max_\theta f(\theta)$ (MES), the maximizer location $x^\ast = \arg\max_\theta f(\theta)$ (PES), or the underlying function realization $f$ itself (BALD). Each choice encodes a different question the designer is asking the next physical print to answer.

## Max-value entropy search — default for the print layer

[Wang & Jegelka 2017](https://arxiv.org/abs/1703.01968) introduced max-value entropy search (MES) as the acquisition whose latent $z = y^\ast$ is the global-optimum *value*:

$$
\alpha_\text{MES}(\theta)
= \mathrm{H}\big[p(y(\theta)\mid \mathcal D)\big]
- \mathbb E_{y^\ast \mid \mathcal D}\!\big[\mathrm{H}[p(y(\theta)\mid y^\ast, \mathcal D)]\big].
$$

Under the Gaussian posterior from [Ch 02 §00](../02-bayesopt/00-gp.md), the unconditional entropy $\mathrm{H}[p(y(\theta)\mid \mathcal D)]$ has the closed form $\tfrac{1}{2}\log(2\pi e\, \sigma^2(\theta))$. The conditional entropy under the $y \le y^\ast$ truncation is the entropy of a truncated normal, also closed-form. The expectation over $y^\ast$ is the only quadrature in the acquisition, and MES approximates it by Monte Carlo: draw a modest number of posterior max-value samples $y^\ast_{(1)}, \ldots, y^\ast_{(M)}$ and average.

The Monte Carlo draws are what make MES practical at the print layer. Sampling $y^\ast$ only requires evaluating the posterior maximum, not locating it — the Gumbel approximation of Wang & Jegelka fits a Gumbel distribution to the posterior maximum and draws $y^\ast$ samples analytically, avoiding an inner-loop optimization per draw. The whole acquisition stays cheap relative to the physical print wait.

## Predictive entropy search — same family, harder latent

[Hernández-Lobato, Hoffman & Ghahramani 2014](https://arxiv.org/abs/1406.2541) introduced predictive entropy search (PES) with the complementary latent $z = x^\ast$:

$$
\alpha_\text{PES}(\theta)
= \mathrm{H}\big[p(y(\theta)\mid \mathcal D)\big]
- \mathbb E_{x^\ast \mid \mathcal D}\!\big[\mathrm{H}[p(y(\theta)\mid x^\ast, \mathcal D)]\big].
$$

PES asks "where is the optimum?" where MES asks "what is the optimum?" The distinction is load-bearing when the cost of action depends on $x^\ast$ specifically — for a designer who needs to *know the design* rather than know its performance, PES's latent is the one that actually answers the question. For `sim-soft` at the physical-print layer, both latents route through the same downstream decision (pick the next design to print), so the cheaper-to-compute of the two is the defensible default.

PES is more expensive than MES per evaluation because sampling $x^\ast$ requires drawing a posterior function realization and *optimizing* that draw to locate its maximizer. Each $x^\ast$ sample carries an inner-loop cost of the same order as the outer BayesOpt's inner-loop acquisition optimization in [Ch 02 §01](../02-bayesopt/01-acquisitions.md). `sim-soft` does not ship PES as a first-class acquisition; MES delivers the core active-learning gains at a fraction of the compute cost.

## BALD — framework translation from classification to regression

[Houlsby, Huszár, Ghahramani & Lengyel 2011](https://arxiv.org/abs/1112.5745) introduced BALD (Bayesian Active Learning by Disagreement) with the latent $z = f$, the underlying function realization itself. The paper frames BALD for *classification* — GP classifiers with Bernoulli likelihoods and discrete output entropies — and derives the acquisition as the expected KL divergence between the full posterior predictive and the predictive conditioned on a single function draw.

Applying BALD to a reward-surface *regression* GP, which is the relevant setting at the physical-print layer, is an algebraic translation not derived in the original paper. The mutual-information identity still applies:

$$
\alpha_\text{BALD}(\theta)
= \mathrm{H}\big[p(y(\theta)\mid \mathcal D)\big]
- \mathbb E_{f \mid \mathcal D}\!\big[\mathrm{H}[p(y(\theta)\mid f, \mathcal D)]\big],
$$

and under Gaussian likelihoods the conditional entropy $\mathrm{H}[p(y(\theta)\mid f, \mathcal D)] = \tfrac{1}{2}\log(2\pi e\,\sigma^2_\text{noise})$ collapses to a constant — the only uncertainty left after conditioning on $f$ is the observation-noise variance. With $\sigma^2(\theta)$ as the GP predictive variance over the noisy observation (the convention established in [Ch 02 §00](../02-bayesopt/00-gp.md)), the acquisition reduces to

$$
\alpha_\text{BALD}(\theta) = \tfrac{1}{2}\log\!\left(\frac{\sigma^2(\theta)}{\sigma^2_\text{noise}}\right),
$$

the log signal-to-noise ratio of the GP posterior at $\theta$. This regression form is well-known in the GP-active-learning literature but is not what Houlsby et al. 2011 derives; the citation is for the classification-BALD acquisition and the identity from which the regression form follows, not for the regression form itself. The sub-leaf commits to this translation explicitly rather than silently extending the paper's framing.

For a homoscedastic GP, BALD-in-regression-form is monotone in $\sigma^2(\theta)$ and reduces to pure-variance-maximizing active learning. That is not a failure — it is the correct acquisition when the only latent worth resolving is the function realization itself and the observation noise is constant. Under the `GradientEstimate::Noisy { variance }` flag from [Ch 02 §01](../02-bayesopt/01-acquisitions.md), the noise variance becomes $\theta$-dependent and BALD's regression form generalizes to the heteroscedastic $\tfrac{1}{2}\log(\sigma^2(\theta)/\sigma^2_\text{noise}(\theta))$.

## What `sim-soft` ships

MES as the default, BALD as the simpler fallback for regression-only settings, PES absent from the initial surface. The three appear in the `Acquisition` enum as an extension of the [Ch 02 §01](../02-bayesopt/01-acquisitions.md) names:

```rust
use sim_ml_chassis::Tensor;

pub enum Acquisition {
    ExpectedImprovement,
    UpperConfidenceBound { beta: f64 },
    Thompson,
    MaxValueEntropySearch { n_ystar_samples: usize },
    BALDRegression,
}
```

`n_ystar_samples` controls the Monte Carlo sample count for $y^\ast$ in MES; the Gumbel-fit approximation keeps each sample's cost a small multiple of a GP posterior evaluation. `BALDRegression` names the regression adaptation rather than the classification-BALD of the original paper — the enum variant encodes the framework translation.

PES is omitted from the shipped enum because its implementation cost is disproportionate to its benefit for the physical-print use case. The `Acquisition` enum leaves room for a `PredictiveEntropySearch` variant to land in a follow-up if the designer's question changes to one where the maximizer location is the bottleneck signal.

## What this sub-leaf commits the book to

- **Information-theoretic active learning is mutual information between $y(\theta)$ and a latent $z$**, computed as predictive entropy minus expected conditional entropy. MES, PES, BALD differ only in $z$.
- **MES (Wang & Jegelka 2017) uses $z = y^\ast$**, has a closed-form conditional entropy under truncation, and approximates the outer expectation via a small Monte Carlo draw of posterior maxima. Default active-learning acquisition at the physical-print layer.
- **PES (Hernández-Lobato et al. 2014) uses $z = x^\ast$** and asks "where is the optimum?" Named for completeness; not shipped because $x^\ast$ sampling requires an inner optimization per draw, and the physical-print use case routes both latents through the same decision.
- **BALD (Houlsby et al. 2011) uses $z = f$.** The paper frames BALD for GP classification; the regression adaptation collapses the acquisition to the log signal-to-noise ratio of the GP posterior. The citation is for the classification BALD and the mutual-information identity; the regression form is algebraic from that identity, noted explicitly rather than silently extended.
- **`Acquisition` enum gains `MaxValueEntropySearch { n_ystar_samples }` and `BALDRegression` variants** that compose with the existing `ExpectedImprovement`, `UpperConfidenceBound`, `Thompson` from [Ch 02 §01](../02-bayesopt/01-acquisitions.md). `PredictiveEntropySearch` is not in the initial surface.
