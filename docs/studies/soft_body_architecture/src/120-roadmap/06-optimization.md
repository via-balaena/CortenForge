# Milestone — optimization loop

The optimizer is the only Part 12 milestone that is not a single phase close. It matures in three stages across Phases D, G, and I, cashing out in a post-Phase-I physical-print loop that [Ch 07](07-open-questions.md) names as the roadmap's highest-priority off-roadmap item. [Part 10 Ch 06](../100-optimization/06-full-loop.md) is the capability specification; this chapter is the milestone schedule — what subset of that spec is live at each phase close.

The maturation mirrors [Part 10 Ch 06's phase-staged rollout](../100-optimization/06-full-loop.md) and honors [Part 11 Ch 03](../110-crate/03-build-order.md#the-committed-order): phase 1 at Phase D, phase 2 at Phase G, phase 3 at Phase I, and the physical-print loop post-Phase-I.

## Phase 1 — at Phase D close

At Phase D, the inner loop from [Part 10 Ch 06](../100-optimization/06-full-loop.md) runs against a synthetic 2-parameter design space on the CPU Phase D solver. The BayesOpt posterior update from [Part 10 Ch 02](../100-optimization/02-bayesopt.md) is live with a gradient-enhanced GP consuming the Phase D IFT gradient. The sim-to-real correction is dormant (zero residual, no `MeasurementReport` ingestion), the preference GP is dormant (hardcoded reward weights), and the outer loop is a unit test with synthetic measurement data.

The Phase 1 deliverable proves end-to-end gradient flow from `cf-design` through `sim-soft`'s reward readout to `sim-opt`'s BayesOpt — the gradient arrow from [Part 10 Ch 00's forward-map diagram](../100-optimization/00-forward.md) is unbroken. It does not yet prove anything about the real design space.

## Phase 2 — at Phase G close

At Phase G, the inner loop extends to the real canonical design space (10–50 parameters per [Part 10 Ch 00](../100-optimization/00-forward.md)) with live `cf-design` authoring. The change-detection-driven warm-start path from [Part 7 Ch 04](../70-sdf-pipeline/04-live-remesh.md) is exercised, and the three-class `EditResult` wall-time table lands with measured numbers. The `GradientEstimate::Noisy { variance }` flag fires for the first time on a real design sweep — at `TopologyChanging` edits per [Part 6 Ch 05](../60-differentiability/05-diff-meshing.md), at adaptive-refinement events per [Part 7 Ch 03](../70-sdf-pipeline/03-adaptive-refine.md), and at adaptive-Δt shrinks per [Part 5 Ch 02](../50-time-integration/02-adaptive-dt.md). BayesOpt's noise-tolerant acquisition from [Part 10 Ch 02](../100-optimization/02-bayesopt.md) handles the mixed exact-and-noisy-gradient regime.

Still no physical printing, still no preference learning. Sim-to-real correction remains dormant; reward weights are hardcoded, typically as an equal-weight starting guess or a documented weighting from the canonical problem's reward composition in [Part 1 Ch 01](../10-physical/01-reward.md).

## Phase 3 — at Phase I close

At Phase I, the preference GP from [Part 10 Ch 03](../100-optimization/03-preference.md) comes online. The designer rates pairs of *rendered* designs from the Phase I visual layer, and the preference GP learns reward-composition weights from pairwise ratings. "Looks right" becomes a legitimate inner-loop signal alongside "computes well" — the visual layer feeding back into the optimizer is the thesis's visual-and-physical-are-one claim closing on a second axis after Phase I's rendering already closed it on the first.

Still no physical printing. Sim-to-real correction remains dormant; a printer, sensors, and measurement protocol are out of scope for Phases A–I.

## Post-Phase-I — off the roadmap

The [physical-print loop from Part 10 Ch 06](../100-optimization/06-full-loop.md) — `MeasurementReport` ingestion, residual GP online updates per [Part 10 Ch 05](../100-optimization/05-sim-to-real.md), batch-rating workflow, cross-print drift detection — is the platform's transition from a simulator into the engineering-workflow tool the thesis promised. It is named as post-Phase-I and explicitly *not on the Phase A–I roadmap* because it requires a specific printer, sensors, and measurement protocol that the simulation crate does not commit to acquiring. [Ch 07](07-open-questions.md) carries this as the roadmap's highest-priority off-roadmap item.

## Cross-phase invariants

Three commitments hold across all three phases of the optimizer milestone.

- **The `ForwardMap` trait contract from [Part 10 Ch 00](../100-optimization/00-forward.md) does not change.** Phase 1 implements it on a synthetic design space; Phase 2 extends it to the real design space; Phase 3 adds preference-weighted reward composition on top. The trait surface stays stable across all three.
- **The `GradientEstimate` contract is honored from Phase 1 onward.** Phase 1 does not exercise the `Noisy` variant because its synthetic design space does not cross topology boundaries; Phases 2 and 3 do. Downstream optimizers treat the flag as first-class at every phase, so that wiring in `Noisy` at Phase 2 is not a breaking change.
- **The `MeasurementReport` boundary with `sim-opt` is designed at Phase 1, left dormant through Phase I, and wired up post-Phase-I.** The trait surface exists and is testable on synthetic data in Phase 1; the real data pipe arrives with the physical printer.

## What this milestone ties off

Ch 06 is the one Part 12 chapter whose deliverable is distributed across the other milestones rather than pinned to a single phase close. That is why the book lists it separately on [Ch 00's dependency graph](00-dependencies.md) as a thread through Phases D, G, and I rather than a node. The optimizer is not a capability you unlock once; it is a capability that deepens as the substrate underneath it gets faster, richer, and prettier, and the post-Phase-I physical-print loop is what says the deepening is complete.
