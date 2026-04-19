# Forward — parametric design → reward

Every chapter in Part 10 sits on top of one object: a map $F$ from a parametric design vector $\theta \in \mathbb{R}^d$ to a scalar reward $R \in \mathbb{R}$. This chapter defines that map, names what goes into $\theta$, names what comes out as $R$, names what a single evaluation costs in wall time and flops, and names what we expect the reward landscape to look like. The later chapters are techniques for optimizing against this map under different constraints (cheap vs expensive evaluations, exact vs noisy gradients, measurable vs subjective rewards). None of them make sense without this one fixed.

This is an inherent leaf — no sub-chapters. The content is dense because the rest of Part 10 and Part 12's [optimization milestone](../120-roadmap/06-optimization.md) cite it as the forward-map contract.

## What $\theta$ contains

The design-parameter vector $\theta$ is the flattened authoring state `cf-design` emits, plus a small number of `sim-soft` internals that are exposed to the optimizer. Concretely:

- **SDF parameters** from `cf-design`'s composition tree — primitive placements, radii, blend radii $k$ of smooth unions, spline control-point positions for boundary curves. Per [Part 7 Ch 00's commitment](../70-sdf-pipeline/00-sdf-primitive.md), these parameterize the `SdfField` ingested by `sim-soft`.
- **Material-field parameters** — stiffness values and spatial-field coefficients for the `MaterialField`: per-layer neo-Hookean $(\mu, \lambda)$ or Mooney-Rivlin $(C_{10}, C_{01})$ pairs, anisotropic fiber-direction fields' parameters, Prony-series $(\tau_i, g_i)$ terms for [viscoelasticity](../20-materials/07-viscoelastic.md).
- **Boundary and contact** — prescribed displacement magnitudes, contact-friction coefficient $\mu_c$, contact-partner kinematic parameters (probe radius / orientation, driven by [`sim-core`](../110-crate/02-coupling/00-mjcf.md)).
- **Reward-composition weights** — the weights $w_i$ on the [reward composition from Part 1 Ch 01](../10-physical/01-reward/04-composition.md). Promoted to $\theta$ because the designer tunes what they care about per-application, and because making the weights first-class lets [Ch 03 preference learning](03-preference.md) write over them from rating data.

Typical dimensionality on the canonical problem is $d \in [10, 50]$ for a single-layer sleeve with two or three material regions and four or five geometric parameters. High-dimensional configurations (many-primitive composition trees, per-element material fields) push $d$ to $\sim 200$; [Ch 02's high-dim sub-chapter](02-bayesopt/02-high-dim.md) addresses the scaling. Design spaces with $d \gtrsim 10^3$ are out of scope for the Phase A–I roadmap.

## The forward map, stage by stage

```text
                cf-design                 Part 7 Ch 04
θ ───────▶ SdfField + MaterialField ──▶ change detector ──▶ EditResult
                                                              │
                                                              ▼
                                                       (re-mesh or warm-start)
                                                              │
                              Part 5 Ch 00                    ▼
                      ┌────── Newton on U_n ◀─────── tet mesh + materials
                      │                                       │
                      ▼                                       ▼
                equilibrium x*                         cached Cholesky factor
                      │                                       │
         Part 1 Ch 01 │                                       │
                      ▼                                       │
                 readout R(x*; θ) ─────────────────┐          │
                                                   │          │
               ┌───────────────────────────────────┘          │
               ▼                                              ▼
        scalar reward R                          (stored on Tape for IFT)
```

Each stage is explicit about what it produces and what it caches for the backward pass:

1. **`cf-design` → `SdfField` + `MaterialField`.** Authoring layer only. No solver state. The boundary is the `SdfField` and `MaterialField` trait objects from [Part 7 Ch 00](../70-sdf-pipeline/00-sdf-primitive.md); `sim-soft` does not inspect the composition tree.
2. **Change detection → `EditResult`.** The [Part 7 Ch 04 classifier](../70-sdf-pipeline/04-live-remesh.md) labels the edit as `ParameterOnly`, `MaterialChanging`, or `TopologyChanging`. Cost regime is selected here, not downstream.
3. **Mesh + materials → equilibrium $x^\ast$.** The [Part 5 Ch 00 Newton loop](../50-time-integration/00-backward-euler.md) minimizes $U_n(x; \theta)$. At convergence, stores $x^\ast$, the `faer::sparse::linalg::solvers::Cholesky<f64>` factor, and $\partial r / \partial \theta$ on the autograd tape — the [factor-on-tape pattern](../50-time-integration/00-backward-euler.md) Part 5 commits to.
4. **Readout $R(x^\ast; \theta)$.** The scalar reward is the composition of pressure uniformity, coverage, peak-pressure barrier, and effective-stiffness-bound terms from [Part 1 Ch 01](../10-physical/01-reward.md), all smooth, all autograd-traceable.

The trait-level signature the optimizer sees is:

```rust
use sim_ml_chassis::{Tape, Tensor};
use sim_soft::{SoftScene, EditResult, GradientEstimate};

pub trait ForwardMap {
    /// One evaluation. Populates the tape with the IFT-ready factor if the
    /// edit class admitted smooth gradients; otherwise tape-recorded as opaque.
    fn evaluate(
        &mut self,
        theta: &Tensor<f64>,
        tape: &mut Tape,
    ) -> (Tensor<f64>, EditResult);

    /// Gradient, pulled from the tape when possible, FD-wrapped otherwise.
    fn gradient(
        &mut self,
        theta: &Tensor<f64>,
        tape: &Tape,
    ) -> (Tensor<f64>, GradientEstimate);
}
```

`EditResult` carries the edit classification and wall-time, so the optimizer knows per-evaluation whether it paid the 50-ms warm-start price or the 500-ms full re-mesh price. `GradientEstimate` carries `{Exact, Noisy { variance }}` per [Part 7 Ch 04's handoff contract](../70-sdf-pipeline/04-live-remesh.md).

## Cost model

Per-evaluation wall time is the dominant quantity for every downstream optimizer. `sim-soft`'s targets on the canonical ~30k-tet design-mode resolution on a consumer GPU, per the [Phase E budgets](../110-crate/03-build-order.md#the-committed-order):

| EditClass | Typical frequency | Forward wall | Gradient (IFT) | Total per step |
|---|---|---|---|---|
| `ParameterOnly` | ≈90% of design-space steps | ≤50 ms | ≤20 ms (back-substitution on cached factor) | ≤70 ms |
| `MaterialChanging` | ≈5% | ≤200 ms | ≤50 ms (re-factor + back-substitution) | ≤250 ms |
| `TopologyChanging` | ≈5% | ≤500 ms | 2× forward (FD, central) | ≤1500 ms + noise flag |

The factor-of-20 spread between the cheap and expensive cases is why no downstream optimizer treats evaluations as uniform cost. [Ch 04's cost-aware active learning](04-active-learning.md) spends directly against this table.

For experience-mode (~5k tets, Phase E 30+ FPS target), the wall times are several-fold faster — the DOF reduction is 6×, and the sparse-solver cost scales super-linearly in DOFs, so the actual speedup depends on the factorization regime and lands in the single-digit multiplier range. Design-mode is where the optimizer usually runs; experience-mode is where the designer previews a chosen $\theta$.

## Gradient availability — the 95/5 split

The core affordance Part 10 is built on: for ≈95% of steps in the canonical design space, $\partial R / \partial \theta$ is *exact*, produced by the [IFT machinery of Part 6 Ch 02](../60-differentiability/02-implicit-function.md) at the cost of one back-substitution on the cached Cholesky factor. For ≈5% of steps — specifically, topology-crossing edits per [Part 7 Ch 04 §4](../70-sdf-pipeline/04-live-remesh.md) — the gradient is [FD-wrapped](../60-differentiability/05-diff-meshing.md) and flagged as `GradientEstimate::Noisy { variance }`.

The same flag also rises on two additional events: [Part 7 Ch 03 adaptive-refinement](../70-sdf-pipeline/03-adaptive-refine.md) triggers (rare; occurs when stress feedback demands mid-episode mesh refinement) and [Part 5 Ch 02 adaptive-$\Delta t$ shrinks](../50-time-integration/02-adaptive-dt.md) (rare; occurs when Newton line search fails and the timestep halves). Both are topology- or state-graph-changing events that break the smoothness assumption the IFT needs.

The 95/5 split is not universal — it is an empirical expectation on the canonical sleeve-on-probe design space, where most design sweeps land inside a single topology. Design spaces dominated by big geometric moves (adding/removing a primitive per step) shift toward 70/30 and make Ch 02's noise-tolerant acquisition machinery load-bearing rather than edge-case. Part 10's design decisions are made assuming the split is in the 80/20 to 95/5 range; outside that range the trade-offs change.

## What the reward surface looks like — what we expect

A few properties of $R(\theta)$ are visible a priori from the physics; they shape every optimizer choice downstream.

- **Multi-modal in geometric parameters.** The conformity objective has multiple local maxima at different squeeze depths — one at light contact with even pressure, another at full wrap where peak pressure exceeds the comfort bound. The [peak-pressure barrier](../10-physical/01-reward/02-peak-bounds.md) suppresses the bad maxima but does not eliminate them near the barrier wall.
- **Smooth within a topology, non-smooth across topologies.** Inside a fixed mesh topology, $R$ is as smooth as $\Psi_\text{elastic}$, $\Psi_\text{contact}$, and the readout are — which is $C^2$ at least (neo-Hookean and IPC barrier are $C^\infty$ away from $J = 0$ and the gap-zero singularity). Across topology crossings, $R$ is continuous but not differentiable — the state-transfer step preserves the solution value but not its derivatives per [Part 7 Ch 04 §4](../70-sdf-pipeline/04-live-remesh.md).
- **Near-plateaus in material-composition-weight subspace.** Several weight vectors produce indistinguishable reward values because the reward terms are strongly correlated on designs that already conform well. This shows up as poor identifiability for the weights $w_i$; [Ch 03 preference learning](03-preference.md) addresses this by eliciting weights from user ratings rather than trying to optimize them from measurement alone.
- **Heavy tail at the peak-pressure bound.** Near the [peak-pressure barrier](../10-physical/01-reward/02-peak-bounds.md), a small parameter change can tip the worst-case pressure over the smoothed wall; $R$ drops sharply. The barrier is smooth, so the gradient is defined, but the local curvature is large. GP length-scales in [Ch 02](02-bayesopt.md) have to be short enough to see this without under-fitting the smooth interior.

None of these properties is a showstopper; all four shape the optimizer design.

## What this commits downstream

- **[Ch 01](01-surrogate.md) consumes the forward-map cost table** — surrogate models amortize the 50–500 ms evaluation by substituting a sub-millisecond surrogate for most acquisition-function inner-loop evaluations.
- **[Ch 02](02-bayesopt.md) consumes the 95/5 gradient split** — its acquisition-function design specifically accommodates the noise flag, and its gradient-enhanced GP ingests $(\theta, R, \nabla_\theta R)$ triples for the 95% smooth-gradient case.
- **[Ch 03](03-preference.md) consumes the weight-identifiability property** — the reward-composition weights are exactly the parameters preference learning should write to.
- **[Ch 04](04-active-learning.md) consumes the cost table** — information-gain-per-second is only meaningful given the per-EditClass wall-time spread.
- **[Ch 05](05-sim-to-real.md) consumes the reward readout** — the sim-side reward $R_\text{sim}(\theta)$ is the quantity the sim-to-real bias model compares against the measured real-world reward $R_\text{real}(\theta)$.
- **[Ch 06](06-full-loop.md) consumes everything** — the full design-print-rate loop closes on this forward map as its innermost building block.

No chapter in Part 10 questions this contract; the downstream question is how to optimize against it efficiently, not how to change it.
