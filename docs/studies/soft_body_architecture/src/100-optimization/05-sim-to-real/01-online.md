# Online updating

The parent spine commits to three update cadences against the residual GP: incremental per-print posterior updates, periodic hyperparameter re-fits with change-detection against the prior fit, and designer-initiated batch re-calibration when the printer or resin changes. This leaf names the Cholesky-factor machinery that makes the per-print update cheap enough to sit inside [Ch 06's budget](../06-full-loop.md), the marginal-likelihood comparison that stands in for a formal drift test, the age-weighted ingestion rule that keeps old training data informative at a reduced contribution, and the `MeasurementReport` boundary that keeps `sim-opt` on one side and the physical world on the other.

## Incremental per-print update via bordered Cholesky

Each measured print appends a single row and column to each component residual GP's training data. The augmented kernel matrix grows from $n \times n$ to $(n+1) \times (n+1)$:

$$
K_{n+1} =
\begin{pmatrix}
K_n            & k(\Theta_n, \theta_{n+1}) \\
k(\theta_{n+1}, \Theta_n) & k(\theta_{n+1}, \theta_{n+1}) + \sigma^2_\text{noise}
\end{pmatrix},
$$

and the Cholesky factor extends by the bordered-Cholesky formula:

$$
L_{n+1} =
\begin{pmatrix}
L_n             & 0 \\
\ell_{n+1}^\top & d_{n+1}
\end{pmatrix},
\qquad L_n\, \ell_{n+1} = k(\Theta_n, \theta_{n+1}),
\qquad d_{n+1} = \sqrt{k(\theta_{n+1}, \theta_{n+1}) + \sigma^2_\text{noise} - \ell_{n+1}^\top \ell_{n+1}}.
$$

The triangular solve for $\ell_{n+1}$ is $O(n^2)$ and the scalar $d_{n+1}$ is $O(n)$, so the incremental update is $O(n^2)$ per print rather than the $O(n^3)$ a full re-factorization would cost. This is the textbook bordered-Cholesky extension of a GP with one new observation; [Rasmussen & Williams 2006 ch. 2](https://gaussianprocess.org/gpml/) develops the Cholesky-based GP inference pattern from which the bordered extension follows directly.

The parent spine names $n_\text{real} \lesssim 100$ as the regime where the incremental update stays well-conditioned. Past that threshold, accumulated floating-point error in the factor — each incremental update loses a few digits of precision relative to a ground-truth factorization of the current $K$ — starts to degrade the posterior's calibration. `sim-soft`'s policy is to re-factor from scratch every $\sim 20$ prints once $n_\text{real}$ crosses the threshold; the $O(n^3)$ re-factorization is cheap at $n \lesssim 200$ ($< 1$ s at single-machine scale, well inside [Ch 06's residual-GP-update budget](../06-full-loop.md#budget-scales)) and restores the factor to ground truth.

The $\alpha = K^{-1} \mathbf r$ cache from [Ch 02 §00](../02-bayesopt/00-gp.md)'s `GradientEnhancedGP` pattern applies here too: with the factor updated, $\alpha_{n+1}$ is one forward solve plus one back-substitution against $L_{n+1}$ — $O(n^2)$ total — reused across every correction call until the next incremental update.

## Change-detection on hyperparameters

Every $\sim 20$ prints, the residual GP re-fits its kernel hyperparameters $(\sigma_{f,c}^2, \{\ell_{a,c}\}, \sigma^2_\text{noise,c})$ via marginal-likelihood maximization — the same optimization [§00](00-calibration.md) runs on the initial anchor batch — and compares the new fit to the prior fit. The comparison is threshold-based on relative change in each hyperparameter:

- **Length-scale shift.** $|\ell_{a,c}^\text{new} - \ell_{a,c}^\text{old}| / \ell_{a,c}^\text{old}$ exceeding a per-dimension threshold indicates the bias structure on axis $a$ has changed characteristic scale. The threshold is a calibration parameter rather than a fundamental constant; the default `sim-opt` ships is loose enough that typical run-to-run variation does not trigger a false alarm and tight enough that a real drift is caught within a $\sim 20$-print window.
- **Signal-variance shift.** A large jump in $\sigma_{f,c}^2$ indicates the residual's overall magnitude has changed — a systematic over- or under-prediction shift in the sim that did not exist at the prior fit.
- **Noise-variance shift.** A jump in $\sigma^2_\text{noise,c}$ indicates either per-print variance grew (manufacturing got sloppier) or per-measurement noise grew (sensor calibration drifted). This is separable from the other two shifts because the kernel hyperparameters affect the structured part of the residual and $\sigma^2_\text{noise,c}$ affects the white part.

When any of the three triggers fires, the GP moves into a **down-weighting** regime for older training data rather than a hard reset. The parent spine's rationale is that drift is typically gradual, so old measurements carry partial information about the current bias structure even when they no longer carry full information. Down-weighting in a GP is implemented by inflating the per-sample observation-noise variance with an age-dependent factor:

$$
\tilde\sigma^2_\text{noise,c}(i) = \sigma^2_\text{noise,c} \cdot \exp\!\big(\lambda\, (t_\text{now} - t_i)\big),
$$

where $t_i$ is the print time of sample $i$ and $\lambda \geq 0$ is the forgetting rate the change-detection step sets proportional to the observed hyperparameter shift. At $\lambda = 0$ the behavior is standard stationary GP with all samples equally weighted; at $\lambda \gg 0$ recent samples dominate the posterior and old samples contribute near-zero effective weight without being removed. The implementation reuses the training data in place, with the age-weighted noise entering the diagonal of $K$ at factorization time — no data deletion, no separate fast-buffer vs slow-buffer architecture.

## Batch re-fit on explicit drift

When the designer declares an explicit printer change — different printer model, or a new resin batch or supplier — the continuous-drift assumption breaks and the correction reverts to a fresh calibration. `sim-opt` drops the existing residual-GP training data (archived to the `MeasurementReport` log, not deleted) and re-runs the [§00 anchor-print protocol](00-calibration.md#initial-hyperparameter-fit-from-10-to-30-anchor-prints) from scratch. The $n_0 \in [10, 30]$ anchor-print schedule [§00](00-calibration.md#initial-hyperparameter-fit-from-10-to-30-anchor-prints) names bookends the post-drift recalibration, and the residual GP re-enters the incremental-update regime once the new anchor batch is in.

Explicit drift is cheaper to handle than implicit drift because the *designer* made the declaration, so there is no need for the change-detection machinery to decide whether the shift is drift or noise. The branch in the online-update state machine is one-line: if `MeasurementReport::printer_change == true`, drop and re-anchor.

## The `MeasurementReport` boundary

The online-updating machinery lives in `sim-opt`, not `sim-soft` or `sim-ml-chassis`. Its input is the `MeasurementReport` type: a per-print structured record the designer's instrumentation pipeline produces.

```rust
use sim_ml_chassis::Tensor;

pub struct MeasurementReport {
    pub print_id:        PrintId,
    pub theta:            Tensor<f64>,
    pub protocol_version: ProtocolVersion,
    pub force_curve:      Option<Vec<(f64, f64)>>,   // (depth, force) pairs if collected
    pub pressure_map:     Option<PressureField>,      // per-patch pressures if collected
    pub diffusion:        Option<DiffusionCurve>,     // per-channel scattering curve if collected
    pub stiffness:        Option<f64>,                // probe-at-reference-depth if collected
    pub printer_id:       PrinterId,                  // changes here trigger batch re-fit
    pub resin_batch_id:   ResinBatchId,               // changes here trigger batch re-fit
    pub timestamp:        Timestamp,                  // for the age-weighted noise inflation
}

pub enum OnlineUpdateOutcome {
    Incremental { n_new: usize, factor_age: usize },       // rank-1 update applied
    Refactor    { n_current: usize },                      // full re-factorization performed
    Downweight  { lambda: f64, triggered_by: Hyperparam }, // forgetting-rate adjusted after change detection
    Rebuild     { anchor_prints_needed: usize },           // explicit printer/resin change
}
```

`sim-soft` does not look at `MeasurementReport`; `sim-ml-chassis` does not look at `MeasurementReport`. The type crosses the `sim-opt` boundary only, which keeps printer-state concerns out of the simulation and surrogate crates. This matches the crate-boundary discipline [Part 11 Ch 02 ml-chassis coupling](../../110-crate/02-coupling/03-ml-chassis.md) commits to: `sim-opt` is the seam between optimization-stack state (residual GP, BayesOpt GP, preference GP) and physical-world state (print records, measurements, printer configuration).

`OnlineUpdateOutcome` is the observable result of each print ingestion. The outer-loop controller in [Ch 06](../06-full-loop.md) reads it to decide whether the next inner-loop pass should trigger a `select_next_print_batch` call on a freshly rebuilt GP (after `Rebuild`) or continue at the incremental cadence (after `Incremental` or `Downweight`).

## Cost per cadence

| Cadence | Per-event cost | Per-week frequency at 5–20 prints/wk |
|---|---|---|
| Per-print incremental update | $O(n^2)$ triangular solve + factor extension | 5–20 updates |
| Per-$\sim 20$ prints re-factor (once $n \gtrsim 100$) | $O(n^3)$ full Cholesky | $\sim 0$–$1$ events |
| Per-$\sim 20$ prints marginal-likelihood re-fit | $O(n^3)$ per optimizer step × optimizer steps | $\sim 0$–$1$ events |
| Per-explicit-drift batch re-anchor | $\sim 10$ fresh prints × $\sim 15$ min each | rare (per-printer-swap) |

Incremental update and re-factorization both fit inside [Ch 06's residual-GP-update budget](../06-full-loop.md#budget-scales) of $< 1$ s per event at the sample-count scale $n \lesssim 200$ the loop targets. The per-$\sim 20$-print marginal-likelihood re-fit is the most expensive regular cadence — the optimizer iterates over kernel hyperparameters, paying an $O(n^3)$ Cholesky factorization per step — and `sim-opt` amortizes its cost by scheduling it to run in the background during the outer-loop's print-wait window. When the printer is physically running, the CPU is idle for BayesOpt purposes and the re-fit happens inline with the print. This ordering does not show up in the `OnlineUpdateOutcome` enum because it is a scheduling concern, not a state-machine concern. The batch re-anchor is the one path that carries meaningful wall-clock cost (the print and measurement time itself); its rarity keeps that cost amortized over long enough intervals to matter.

## Distinction from adaptive sim-parameter updating

Online residual-GP updating is not the only online sim-to-real approach. [Chebotar et al. 2019](https://arxiv.org/abs/1810.05687) adapts the *simulator's* parameter distribution based on real-world rollout feedback; the residual-GP approach adapts the *gap model* between a fixed simulator and reality. The choice is structural:

- Residual-GP updating preserves sim determinism. A given $\theta$ produces the same $R_\text{sim}(\theta)$ across the whole life of the optimization run, so the [Ch 02 §00](../02-bayesopt/00-gp.md) BayesOpt inner-loop GP's cached training data — the $(\theta, R_\text{sim}, \nabla_\theta R_\text{sim})$ tuples accumulated over prior iterations — remains valid as the sim-to-real correction layer updates. The residual GP can be re-fit, down-weighted, or rebuilt at any time without invalidating any BayesOpt-side state, and the [Ch 00 forward map](../00-forward.md) contract holds unchanged.
- Adaptive sim-parameter updating changes $R_\text{sim}(\theta)$ as the sim parameters update. The BayesOpt GP's training data goes stale every time the sim adapts, and the inner loop has to either re-run the sim at every cached training $\theta$ to refresh the observations, retrain the BayesOpt GP from scratch, or accept a mismatch between cached observations and the new sim.

`sim-soft` ships the residual-GP path because the sim-determinism property is cheap to preserve and expensive to lose. A designer who wants the adaptive-sim-parameter path would wire it against `sim-opt`'s same `MeasurementReport` consumer but bypass the residual GP entirely, treating the sim-parameter update as a separate downstream subsystem. That configuration is not blocked by the architecture but is not the default.

## What this sub-leaf commits the book to

- **Per-print incremental update is a bordered Cholesky extension** (triangular solve for $\ell_{n+1}$ plus scalar $d_{n+1}$) with $O(n^2)$ cost. Well-conditioned until $n_\text{real} \lesssim 100$; full re-factorization every $\sim 20$ prints beyond the threshold. Cites [Rasmussen & Williams 2006 ch. 2](https://gaussianprocess.org/gpml/) for the canonical formula.
- **Change-detection is threshold-based on relative hyperparameter shift** — length-scale, signal variance, noise variance — evaluated at the $\sim 20$-print re-fit cadence. Triggering the detector adjusts the forgetting rate $\lambda$ in an age-weighted observation-noise inflation $\tilde\sigma^2_\text{noise,c}(i) = \sigma^2_\text{noise,c}\, e^{\lambda(t_\text{now} - t_i)}$; old data is down-weighted, not deleted.
- **Explicit printer or resin change triggers batch re-anchor** via the [§00 anchor protocol](00-calibration.md#initial-hyperparameter-fit-from-10-to-30-anchor-prints). Old data archives to `MeasurementReport` log; the GP re-enters the incremental-update regime once the new anchor batch is in.
- **`MeasurementReport` is the `sim-opt` boundary type.** Carries per-print protocol, per-component measurements, printer and resin IDs, and timestamp. `OnlineUpdateOutcome` is the return enum; the outer-loop controller in [Ch 06](../06-full-loop.md) routes on it. `sim-soft` and `sim-ml-chassis` do not see `MeasurementReport`.
- **Cost per cadence fits inside [Ch 06's residual-GP-update budget](../06-full-loop.md#budget-scales)** at $n \lesssim 200$: $O(n^2)$ incremental updates, $O(n^3)$ re-factor per $\sim 20$ prints, $O(n^3)$-per-optimizer-step marginal-likelihood re-fit scheduled into the outer-loop print-wait window. Explicit-drift re-anchor is the rare path that carries meaningful wall-clock cost.
- **Residual-GP updating preserves sim-side determinism** — a given $\theta$ always produces the same $R_\text{sim}(\theta)$ — in contrast to adaptive-sim-parameter schemes ([Chebotar et al. 2019](https://arxiv.org/abs/1810.05687)) that rebuild the sim's own parameters. Sim-determinism is what keeps the BayesOpt inner loop's cached $(\theta, R_\text{sim}, \nabla_\theta R_\text{sim})$ training data valid across residual-GP updates and is why the residual-GP path is the default.
