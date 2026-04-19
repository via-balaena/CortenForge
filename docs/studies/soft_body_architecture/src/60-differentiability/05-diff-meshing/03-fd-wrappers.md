# Finite-difference wrappers

Given that no published differentiable-meshing approach ([§01](01-diff-mc.md), [§02](02-diff-delaunay.md)) composes cleanly with a 30k-tet production FEM at design-mode rates, `sim-soft` falls back on finite-difference wrappers at the SDF-parameter gradient boundary. This sub-leaf writes out the wrapper's cost, the step-size policy that controls its noise, what happens when a design-parameter perturbation straddles a topology-change threshold, and the chassis-API handshake that propagates the resulting noise estimate to the outer optimizer.

## What the wrapper computes

For an SDF-parameter gradient request $\partial L / \partial \theta_{\text{sdf},j}$, the wrapper performs two full re-meshed-and-resolved forward evaluations:

$$
\widehat{\partial L / \partial \theta_{\text{sdf},j}} \;=\; \frac{L(\theta + \delta_j \hat e_j) - L(\theta - \delta_j \hat e_j)}{2 \delta_j}
$$

with $\hat e_j$ the unit vector in the $j$th SDF-parameter direction and $\delta_j$ a per-parameter step size. Each evaluation is a full re-mesh through the [Part 7 pipeline](../../70-sdf-pipeline/00-sdf-primitive.md) plus a full forward solve from the warm-started previous-mesh state per [Part 7 Ch 04's state-transfer machinery](../../70-sdf-pipeline/04-live-remesh/02-state-transfer.md). Central difference is used rather than one-sided difference because the $O(\delta^2)$ truncation error is the smaller source of bias and the book's $2\times$ cost premium compared to one-sided is a rounding error against the cost of re-meshing.

For $n_\theta$ SDF parameters, one gradient evaluation costs $2 n_\theta$ full solves. At canonical-problem scale the wrapper is affordable at low-tens of parameters and prohibitive at hundreds: each full solve amortizes modestly under warm-started state transfer from [Part 7 Ch 04](../../70-sdf-pipeline/04-live-remesh.md), but the linear scaling in $n_\theta$ dominates long before the per-solve cost can be squeezed further. The FD wrapper is the fallback for small-ish SDF-parameter counts, not a scalable solution.

## Step-size policy

The central-difference bias is $\delta_j^2$ times a third-derivative factor; the noise floor from the forward solve's own convergence tolerance is fixed. The optimal $\delta_j$ balances the two:

- **Start from a geometric prior.** Default $\delta_j$ is a small fraction (≈1%) of the parameter's nominal operating range, clipped to a minimum so the perturbation reliably changes the mesh's SDF sample values above floating-point noise.
- **Adapt per parameter.** Parameters with small operating ranges (blend radii near zero) get tighter $\delta_j$; parameters with large ranges (cavity length) get looser $\delta_j$. The per-parameter step is stored in the `cf-design` binding so design-mode sweeps inherit a sensible default.
- **Recompute when noise exceeds bias.** If the measured variance of a gradient estimate exceeds the modelled truncation-bias contribution, the wrapper flags the estimate as noise-dominated and — if the optimizer budget permits — re-evaluates at a larger $\delta_j$ to push the signal above the noise floor.

None of the three rules is new; the finite-difference literature has treated step-size selection extensively. The twist in `sim-soft` is that the step-size choice interacts with mesh re-derivation: the wrapper cannot simply perturb $\theta$ and re-evaluate a smooth function; it re-runs the meshing pipeline, which has its own discontinuous-output risk (see next section).

## Topology crossings are the adversarial case

The wrapper's failure mode is a parameter value $\theta_j$ such that $\theta_j + \delta_j$ and $\theta_j - \delta_j$ straddle a topology-change threshold — the blend radius crossing a value where a hole opens or closes, the control-point position crossing a manifold where two primitives' surfaces collide. The two forward evaluations mesh to different topologies; the state transfer between them loses correspondence; the central-difference numerator is contaminated by the step discontinuity [§00](00-why-not-mc.md) identified, and the estimate's variance rises sharply.

Three mitigations and one concession:

- **Shrink $\delta_j$ to try to bracket one side only.** If the topology threshold can be detected (the Part 7 Ch 04 change detector fires on one side and not the other), the wrapper re-evaluates with a smaller $\delta_j$ on the side that does not cross the threshold and reports a one-sided difference instead. This costs one extra solve but preserves a usable gradient estimate.
- **If both sides straddle the threshold, one-sided is still the answer.** The topology-change endpoint itself is non-differentiable; the usable gradient is on whichever side the optimizer is moving toward.
- **Flag the estimate as `GradientEstimate::Noisy { variance }`.** When one-sided difference is not possible, or when the one-sided estimate still shows residual noise beyond the bias model, the chassis-API handshake reports the estimate with an upper-bound variance. The outer optimizer ([Part 10 Ch 00 — forward](../../100-optimization/00-forward.md)) weights it accordingly — BayesOpt's UCB acquisition inflates the variance on noisy samples; gradient-enhanced GPs down-weight them.
- **Accept that design-parameter gradients that cross topology thresholds are noisy.** The book is explicit: this is the compromise.

The [Part 7 Ch 04 change detector](../../70-sdf-pipeline/04-live-remesh/00-change-detection.md) already reports edit classes (`ParameterOnly`, `MaterialChanging`, `TopologyChanging`); the FD wrapper consumes those classifications to decide which mitigation path to take. On a `ParameterOnly` or `MaterialChanging` classification both sides of the central difference are on the same topology, and the estimate is exact-up-to-bias. On a `TopologyChanging` classification the mitigations above engage.

## Chassis-API handshake

The wrapper produces a `GradientEstimate` enum value alongside the scalar gradient:

```rust
pub enum GradientEstimate {
    Exact,
    Noisy { variance: f64 },
}
```

`Exact` is returned when the wrapper's two evaluations both resolved to the same topology and the convergence noise was below the bias-modelled floor. `Noisy { variance }` carries an upper-bound variance estimate composed from three sources — the forward solve's own convergence tolerance, the estimated truncation bias at the chosen $\delta_j$, and the topology-crossing penalty when applicable. Downstream optimizers see both the gradient and the variance, and can choose whether to use the gradient at full weight, inflated-variance weight, or discard it entirely in favour of a zero-order fallback.

This is the handshake the [Ch 00 §02 own-every-line commitment](../../60-differentiability/00-what-autograd-needs/02-why-not-burn.md) grounds: the chassis extension [Ch 01 §00 registration](../../60-differentiability/01-custom-vjps/00-registration.md) names the API for custom VJPs, and the `GradientEstimate` variant rides along as the signalling surface for backward closures that cannot produce an exact adjoint.

## The tens-of-parameters ceiling

The single hard limit on the FD wrapper is the $2 n_\theta$ cost scaling. The book's canonical problem at the low-tens-of-SDF-parameters scale lands inside the affordable regime; scaling to richer SDF-parameterizations — dense per-tet material fields, per-surface-point control meshes, learned SDF primitives with hundreds of learned weights — runs past the ceiling within a single gradient evaluation, and scales worse over an optimization run of many gradient evaluations.

That ceiling is the sharpest consequence of the differentiable-meshing open problem. [Part 12 Ch 07](../../120-roadmap/07-open-questions.md) names it as the book's highest-priority unsolved problem precisely because it is the only place `sim-soft`'s otherwise-differentiable-at-scale posture fails; closing the gap (via one of the [§04 frontier options](04-frontier.md)) would remove a hard ceiling, not just a soft cost premium.

## What this sub-leaf commits the book to

- **The FD wrapper is the fallback, not the plan.** Two full re-meshed-and-resolved forward evaluations per SDF-parameter gradient; $2 n_\theta$ scaling; usable at low-tens of parameters, out of reach at hundreds.
- **Per-parameter step sizes are tuned against the downstream forward solve's convergence noise** and adapted when measured variance exceeds the modelled truncation bias.
- **Topology-crossing perturbations are the adversarial case.** Mitigations include one-sided difference, step-size shrinkage, and reliance on the [Part 7 Ch 04 change detector](../../70-sdf-pipeline/04-live-remesh/00-change-detection.md) to classify when the perturbation straddles a threshold.
- **The chassis handshake is `GradientEstimate::{Exact, Noisy { variance }}`.** The variant is returned alongside the scalar gradient from any FD-wrapped VJP; outer optimizers consume it to weight noisy samples explicitly rather than silently.
- **The $n_\theta$ ceiling is the book's sharpest consequence of the differentiable-meshing open problem.** Closing the gap would remove a hard ceiling on SDF-parameter-space scaling, not a soft cost premium on the current regime.
