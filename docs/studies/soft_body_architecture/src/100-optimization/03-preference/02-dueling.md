# Dueling bandits

The [§00](00-gp-pairs.md) and [§01](01-bradley-terry.md) sub-leaves cover the *inference* side of preference learning: given a batch of rated pairs, fit a posterior over the latent utility. This sub-leaf covers the *acquisition* side — which pair should the designer rate next? — and the online-decision formulation that answers it.

## The dueling bandit problem

The framing is due to **Yue, Broder, Kleinberg & Joachims**, introduced at [COLT 2009](https://www.cs.cornell.edu/people/tj/publications/yue_etal_09a.pdf) and extended in the journal version [Yue et al. 2012, J. Computer & System Sciences 78(5):1538–1556](https://doi.org/10.1016/j.jcss.2011.12.028). Both papers are titled "The K-armed Dueling Bandits Problem"; the JCSS 2012 paper extends the COLT 2009 original with refined analysis and proofs. The dueling formulation is: instead of a standard multi-armed bandit where each arm pull yields a scalar reward, the learner selects a *pair* of arms at each round and observes only a noisy preference outcome between them. This matches the structure of `sim-soft`'s rating loop per [Ch 03's Claim 1](../03-preference.md) — absolute scalar feedback is not available, pairwise preference is.

The core algorithmic contribution is the **Interleaved Filter** algorithm family (IF1 and IF2 in the published analysis), which progressively narrows the candidate-arm pool by eliminating arms that lose enough head-to-head duels with statistically high confidence. The theoretical guarantees are cumulative-regret bounds under stochastic-transitivity assumptions on the preference matrix; that class of assumption is the analogue of the "well-calibrated single designer" regime of [§00](00-gp-pairs.md), not the multi-designer tournament regime of [§01](01-bradley-terry.md).

## From K-armed to continuous design space

The original dueling-bandit formulation treats the arms as a finite discrete set — the classical $K$-armed framing, where $K$ is typically in the tens. `sim-soft`'s design-axis space is continuous and high-dimensional; the adaptation the book commits to is a two-stage pattern:

1. **Candidate generation.** At each round, produce a finite pool of candidate $w$-vectors from the BayesOpt acquisition optimizer's top-$K$ outputs, from a space-filling draw over $\mathcal W$, or from previously-rated neighbours. Pool size $K \in [10, 50]$ per round.
2. **Pair selection within the pool.** Apply a dueling-bandit-style pair-selection rule to the pool; ask the designer to rate the selected pair.

The Interleaved Filter style applied directly at stage 2 gives the paper's regret bounds — at the cost of requiring repeated re-rating of promising arms to build up statistical confidence. The alternative `sim-soft` commits to as the default — selecting the pair that maximizes expected posterior-shift over the preference-GP's posterior — is cheaper per round but lacks the dueling-bandit paper's cumulative-regret guarantees. The Interleaved Filter fallback is exposed as a knob for settings where the regret guarantee is load-bearing.

## KL-based pair selection

The default acquisition rule — a KL-divergence-based pair-selection policy — picks the pair whose rating outcome is expected to move the preference-GP's posterior the most, measured in KL divergence:

$$
(w_A, w_B)^\ast = \arg\max_{(w_A, w_B)\in \text{pool}^2}\ \mathbb E_{y \sim p(y \mid \mathcal D)}\!\left[\operatorname{KL}\!\Big(p\big(f \mid \mathcal D \cup \{(w_A, w_B, y)\}\big)\ \big\|\ p(f \mid \mathcal D)\Big)\right]
$$

where $y \in \{w_A \succ w_B,\ w_B \succ w_A\}$ is the unobserved rating outcome and the outer expectation is taken under the predictive distribution of $y$ given the current posterior. The rule naturally balances exploring high-variance regions of $\mathcal W$ against resolving close contests between promising $w$-candidates: a pair of two distant, unknown $w$'s scores high on the exploration term; a pair of two close-in-utility $w$'s scores high on the contest-resolution term.

Computing the expected KL exactly is intractable over the full function-space posterior. The practical path evaluates the expression under the Laplace approximation from [§00](00-gp-pairs.md): the posterior pre-rating is Gaussian at the training points with mean $\hat f$ and covariance $H^{-1}$; the posterior post-rating is refit by one Newton iteration from the pre-rating Laplace mode, conditioned on each possible outcome $y$; the KL between two Gaussians at the training points is then closed-form. At pool sizes $K \in [10, 50]$, the full $\binom{K}{2}$ pair enumeration is cheap — at most $\sim 10^3$ evaluations of the one-Newton-step Laplace update, all warm-starting from the cached Laplace factor.

## Cost-aware variant — when designer time is the constraint

The dueling-bandit literature's native cost model is uniform: each duel has unit cost. In `sim-soft`'s rating loop, the cost is not uniform — pairs that involve *previously-rated* designs are cheaper to rate (the designer already has a mental model) than pairs involving two fresh designs. The **cost-aware variant** the book commits to re-weights the KL acquisition:

$$
\text{score}(w_A, w_B) = \frac{\text{expected KL gain}(w_A, w_B)}{\text{expected designer-time cost}(w_A, w_B)},
$$

where the designer-time cost is modelled as a simple linear function of the number of fresh designs in the pair — zero, one, or two — plus a flat anchor cost for any rating event. Pairs with one fresh design and one previously-rated anchor are the sweet spot: they deliver new information while preserving the designer's calibration against a known reference.

This cost model is deliberately blunt; more elaborate cost structures (per-designer time curves, fatigue effects, session-length penalties) are post-Phase-I refinements. The scalar fresh-design-count cost is enough to avoid the degenerate acquisition regime where the KL rule prefers pairs of two fresh designs because they maximize posterior shift but also maximize designer cognitive load.

## What this sub-leaf does *not* commit to

Per the spine, the preference sub-system is post-Phase-I — the physical-print rating loop is the data source, and that loop is outside the Phase A–I roadmap. This sub-leaf defines the online-acquisition machinery — pair-selection rule, cost-weighting, the Interleaved Filter fallback for discrete-pool settings — but does not commit `sim-soft` to running live dueling-bandit rounds on a real designer before Phase I closes. Synthetic-rating experiments, where a ground-truth utility function plays the role of an oracle designer, are fair game within Phase I for internal validation of the acquisition policy; real-designer rounds ride with the [Ch 06](../06-full-loop.md) integrated loop post-Phase-I.

## The struct

```rust
use sim_ml_chassis::Tensor;

pub struct DuelingBanditPolicy {
    pub preference_gp: PreferenceGP,             // from §00
    pub candidate_pool_size: usize,              // K; typically 10-50
    pub cost_weights: CostWeights,
}

pub struct CostWeights {
    pub per_fresh_design: f64,                   // incremental cost per fresh design in pair
    pub anchor_base: f64,                        // flat cost for any rating event
}

pub struct PairProposal {
    pub candidate_a: Tensor<f64>,
    pub candidate_b: Tensor<f64>,
    pub expected_kl_gain: f64,
    pub expected_cost: f64,
    pub score: f64,                              // expected_kl_gain / expected_cost
}

impl DuelingBanditPolicy {
    /// Propose the next pair for the designer to rate, given the current GP posterior
    /// and a candidate pool. Ranked by cost-aware KL score; Interleaved Filter exposed
    /// as a fallback path for regret-bounded settings.
    pub fn propose_next_pair(&self, pool: &[Tensor<f64>]) -> PairProposal { /* ... */ }
}
```

## What this sub-leaf commits the book to

- **Dueling-bandit framing for online preference acquisition.** Attribution to Yue, Broder, Kleinberg & Joachims, COLT 2009 original and JCSS 2012 journal extension; the "K-armed Dueling Bandits" framework and its Interleaved Filter algorithms are the named prior art.
- **Two-stage adaptation to continuous design space.** Candidate-pool generation from the BayesOpt acquisition optimizer (pool size $K \in [10, 50]$) plus pair-selection within the pool; Interleaved Filter applies at the pair-selection stage if regret bounds are needed.
- **KL-based pair selection as the default rule.** Expected posterior-shift maximization under the [§00](00-gp-pairs.md) Laplace approximation; closed-form two-Gaussian KL at the training points after one-Newton-iteration conditional refit per candidate outcome; cheap at pool sizes $K \in [10, 50]$.
- **Cost-aware re-weighting via fresh-design count.** Scalar cost model — anchor-plus-fresh count — to avoid degenerate cognitive-load acquisition; richer cost models deferred to post-Phase-I.
- **Interleaved Filter as regret-bounded fallback.** Default is KL acquisition; Interleaved Filter exposed for settings where the paper's cumulative-regret guarantee is load-bearing.
- **Phase-I deferral.** Synthetic-oracle acquisition experiments within Phase I; real-designer rounds ride with [Ch 06](../06-full-loop.md)'s post-Phase-I integrated loop.
