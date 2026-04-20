# Cost-aware active learning

The parent spine's Claim 2 is that cost-aware information gain — maximize $\mathrm{IG}(\theta) / c(\theta)$ rather than $\mathrm{IG}(\theta)$ alone — is load-bearing rather than a refinement. This leaf specifies the cost model the acquisition divides by, the batch structure that amortizes fixed print costs across 4–8 designs, and the API surface that composes cost-aware active learning with the [§00 info-criteria](00-info-criteria.md) acquisitions.

## Cost-weighted information gain

The cost-aware acquisition is

$$
\alpha_\text{cost}(\theta) = \frac{\mathrm{IG}(\theta)}{c(\theta)},
$$

where $\mathrm{IG}(\theta)$ is any of the three mutual-information quantities from [§00](00-info-criteria.md) and $c(\theta)$ is the wall-clock-plus-material cost of evaluating the reward at $\theta$. The division-by-cost pattern is the cost-weighted extension used in practical BayesOpt deployments where evaluation cost varies by orders of magnitude across the design space; [Snoek, Larochelle & Adams 2012](https://arxiv.org/abs/1206.2944) is the canonical reference for the pattern in the hyperparameter-tuning setting, where training-time budget per configuration varies the same way print time varies here.

The division is not the only defensible cost-normalization — multi-fidelity formulations that model the acquisition over a $(\theta, \text{fidelity})$ joint space ([Kandasamy et al. 2017](https://arxiv.org/abs/1703.06240)) generalize the pattern when the cost axis is continuous and the evaluation outcomes differ only in noise level. `sim-soft` uses the simpler quotient form because the physical-print layer has discrete fidelity levels (sim vs print) rather than a continuum, and the print fidelity is the only one where cost matters at the information-gain timescale.

## The cost model

$c(\theta)$ in this chapter is not a single number. It tracks two asymmetries — a within-sim asymmetry that the inner loop already handles, and a sim-versus-print asymmetry that is the active-learning layer's reason to exist.

### Within-sim cost variation

Sim evaluations inherit the [Ch 00](../00-forward.md) cost table indexed by `EditClass`: parameter-only edits warm-start from the previous solution and run at the bottom end of the sim cost range, while topology-crossing edits fall back to a cold solve at the top end. [Ch 02 §01's acquisition optimizer](../02-bayesopt/01-acquisitions.md) already tunes its inner-loop budget against this per-`EditClass` cost; the active-learning layer reuses the same cost table without amending it.

### Sim-versus-print cost asymmetry

The dominant asymmetry is between sim evaluations and physical prints. The parent spine commits to roughly millisecond-scale sim evaluations and hour-scale prints — four to five orders of magnitude of cost separation, consistent with [Ch 06's budget table](../06-full-loop.md). The acquisition's cost model has two corresponding cases:

$$
c(\theta) =
\begin{cases}
c_\text{sim}(\theta, \mathrm{EditClass}) & \text{sim evaluation, from Ch 00's table} \\
c_\text{print,batched}(\theta) & \text{physical print, amortized per batch}
\end{cases}
$$

The sim case is cheap enough that the cost-normalized acquisition is almost always maximized by a sim evaluation unless the sim posterior has locally saturated the available information. This is the design intent from the parent spine's Claim 2: the cost-aware form automatically routes queries to the cheap channel until there is no cheap information left to extract, then spends a print slot on the remaining uncertainty.

## Batch structure for physical prints

A single print job carries fixed setup cost (printer and measurement-bench preparation, post-processing) that the parent spine amortizes across 4–8 designs per batch. That amortization is why prints are selected in batches rather than one at a time. The batch-level acquisition is

$$
\alpha_\text{batch}(\Theta) = \frac{\mathrm{IG}(\Theta)}{c_\text{setup} + \sum_{\theta_i \in \Theta} c_\text{marginal}(\theta_i)},
$$

where $\Theta = \{\theta_1, \ldots, \theta_K\}$ is a batch of $K$ candidate designs, $c_\text{setup}$ is the once-per-batch fixed cost, and $c_\text{marginal}$ is the per-design incremental cost (print + measurement). $\alpha_\text{batch}$ has a sweet spot in $K$: at small $K$ the fixed setup cost dominates the denominator and adding designs improves the ratio; at large $K$ the marginal cost dominates and the joint IG's diminishing returns drag the ratio down. The parent spine's 4–8 per batch is the regime where the two effects are in balance.

Two structural properties of the batch acquisition matter:

- **Joint IG, not summed IG.** The candidate designs $\theta_1, \ldots, \theta_K$ are correlated under the GP posterior, so the batch's information gain is not $\sum_i \mathrm{IG}(\theta_i)$. The joint form is the entropy of the predictive distribution over $(y_1, \ldots, y_K)$ minus the expected conditional entropy given the latent. For the BALD-regression case this reduces to a closed-form log-determinant difference of covariance matrices; for MES or PES the outer expectation over the latent is evaluated by Monte Carlo sampling of joint maxima or maximizer locations. In every case the summed form over-counts information at correlated designs and collapses the batch onto near-duplicates.
- **Diversity promotion is automatic via the joint form.** Because correlated designs contribute less joint IG than decorrelated ones, the batch acquisition intrinsically spreads across the design space. [Ch 06's shortlist](../06-full-loop.md) cites determinantal sampling and clustering as explicit diversity mechanisms; those are complementary to the joint-IG structure and serve as tractable approximations when direct batch-IG optimization over $K$ joint dimensions is too expensive.

The joint maximization over $\Theta$ is combinatorial in principle, but the greedy approximation — select $\theta_1$ to maximize individual cost-aware IG, then $\theta_2$ to maximize the incremental IG given $\theta_1$ already selected, and so on — has the standard submodularity-based approximation guarantee and is what `sim-soft` ships.

## API surface

```rust
use sim_ml_chassis::Tensor;
use sim_soft::EditClass;

pub struct CostModel {
    pub sim_cost_fn: fn(&Tensor<f64>, EditClass) -> f64,   // ms, from Ch 00's table
    pub print_setup_cost: f64,                             // fixed per-batch overhead
    pub print_marginal_cost_fn: fn(&Tensor<f64>) -> f64,   // per-design print + measurement
}

pub struct CostAwareAcquisition {
    pub base: Acquisition,                                  // from Ch 02 §01 / §00 this chapter
    pub cost_model: CostModel,
}

impl CostAwareAcquisition {
    pub fn select_next_sim(
        &self,
        gp: &GradientEnhancedGP,
        bounds: &DesignBounds,
    ) -> Tensor<f64> { /* ... */ }

    pub fn select_next_print_batch(
        &self,
        gp: &GradientEnhancedGP,
        bounds: &DesignBounds,
        batch_size: usize,                                  // 4..=8 per parent spine
    ) -> Vec<Tensor<f64>> { /* greedy joint-IG maximization */ }
}
```

`select_next_sim` and `select_next_print_batch` are separate entry points because the outer loop calls them at different cadences: sim at each inner-loop iteration, print at each outer-loop iteration. The cost model is shared — a single `CostModel` carries both the within-sim per-`EditClass` table and the print setup + marginal decomposition — but the two entry points apply different normalizations and different optimizers (multi-start L-BFGS for the single-query sim case from [Ch 02 §01](../02-bayesopt/01-acquisitions.md), greedy batch for the print case).

## Where the residual GP enters

The posterior the acquisitions operate against is not the inner-loop BayesOpt GP over sim evaluations — it is the sim-to-real residual GP over physical-measurement samples. [Ch 05](../05-sim-to-real.md) develops the residual-GP construction and the correction path that turns a sim-evaluated reward into a real-world-predicted reward; this chapter assumes the residual GP as a given and computes acquisitions against its posterior. The forward reference is load-bearing in the direction "Ch 04 depends on Ch 05's residual GP," not the other direction.

## What this sub-leaf commits the book to

- **Cost-aware active learning divides information gain by evaluation cost**: $\alpha_\text{cost}(\theta) = \mathrm{IG}(\theta)/c(\theta)$. Cost-weighted EI in [Snoek et al. 2012](https://arxiv.org/abs/1206.2944) is the reference pattern; multi-fidelity BayesOpt ([Kandasamy et al. 2017](https://arxiv.org/abs/1703.06240)) generalizes when the cost axis is continuous.
- **The cost model tracks two asymmetries**: within-sim per-`EditClass` variation (inherited from [Ch 00](../00-forward.md)'s cost table) and the sim-versus-print four-to-five-orders-of-magnitude gap (from [Ch 06](../06-full-loop.md)'s budget).
- **Physical-print batch acquisition amortizes fixed setup cost** across the 4–8-designs-per-batch batch size committed by the parent spine, using joint information gain rather than summed per-design IG (closed-form log-determinant ratio in the BALD-regression case; Monte Carlo over joint latent samples for MES/PES).
- **Batch selection uses greedy maximization** of the joint cost-aware acquisition; diversity falls out of the joint-IG structure automatically, with determinantal sampling or clustering from [Ch 06](../06-full-loop.md) as tractable approximations.
- **`CostAwareAcquisition` wraps any `Acquisition`** from [Ch 02 §01](../02-bayesopt/01-acquisitions.md) / [§00](00-info-criteria.md) with a `CostModel` that exposes `sim_cost_fn`, `print_setup_cost`, and `print_marginal_cost_fn`. Two entry points: `select_next_sim` for inner-loop sim queries, `select_next_print_batch` for outer-loop print batches.
- **The residual GP from [Ch 05](../05-sim-to-real.md) is the posterior** the acquisitions operate against. Ch 04 is a consumer of that posterior, not its definition.
