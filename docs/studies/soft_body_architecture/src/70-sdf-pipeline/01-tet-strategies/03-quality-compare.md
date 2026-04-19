# Quality comparison

[Ch 01's Claim 3](../01-tet-strategies.md) names mesh quality as the determinant of solver conditioning, and the [Ch 01 sub-leaves](00-ftetwild.md) have named three generators. This leaf sets up the comparison: what metrics are measured, which generator wins on which metric, and what the Phase-B deliverable is for locking quality bounds against the canonical problem.

## What is being measured

Three metrics determine whether a tet mesh is acceptable for `sim-soft`'s solver, all defined in [Part 3 Ch 01 mesh-quality](../../30-discretization/01-mesh-quality.md):

- **Aspect ratio** — per-tet shape-quality metric $\rho = r_\text{ins}/r_\text{circ}$ (inscribed over circumscribed sphere radii). A regular tet has $\rho = 1/3$ (the upper bound); ratios toward 0 are degenerate slivers. `sim-soft` sets per-phase lower bounds at [ingest](../../30-discretization/01-mesh-quality.md); the comparison records the 5th-percentile and minimum ratios per generator.
- **Dihedral angle** — angle between adjacent tet faces. A unit-dihedral-angle tet has the regular-tetrahedron dihedral of $\arccos(1/3) \approx 70.53°$. Angles near 0 or π are degenerate; `sim-soft` sets tail bounds. The comparison records the minimum, maximum, and 5th-percentile/95th-percentile angles per generator.
- **Stiffness-matrix conditioning** — the end-to-end consequence of the two shape metrics. Measured as the condition number $\kappa(K)$ of the elastic-tangent block at a representative configuration (rest state, modest uniaxial strain). Ill-conditioning inflates Newton's iteration count and can trigger spurious [line-search retries](../../50-time-integration/00-backward-euler/02-line-search.md); the comparison reports condition numbers as the physics-side observable.

The ingest check catches degeneracies early, but a mesh can pass ingest and still produce Newton-cost or adaptive-$\Delta t$-shrink pathologies at simulation time; the stiffness-conditioning metric is the tighter filter. The comparison exercises all three to give a joint picture.

## Expected qualitative outcome

Without running the comparison yet (Phase B benchmark deliverable), the expected qualitative ranking based on generator design:

| Metric | fTetWild | Delaunay + TetGen | GPU (future) |
|---|---|---|---|
| Aspect ratio (median) | Best — post-improvement passes target uniformity | Acceptable — Delaunay's shape-quality post-pass | Depends on algorithm choice ([§02](02-gpu-tet.md)) |
| Aspect ratio (minimum) | Best — explicit lower-bound enforcement | Worse — Delaunay admits slivers near concave input features | Algorithm-dependent |
| Dihedral angle (tails) | Best — post-improvement targets both tails | Worse tail, especially max-angle tail | Algorithm-dependent; Labelle-Shewchuk has known 5.71° lower bound |
| Stiffness conditioning | Best (follows from shape metrics) | Acceptable at few-thousand-tet experience-mode scales | TBD |
| Meshing time | Order-of-seconds at 30k tets | Fast at few-thousand-tet experience-mode scales | Target: far below fTetWild's; exact target is a Phase-F call |
| Robustness to bad SDFs | Best — envelope pipeline handles defects | Poor — hard failure on non-manifold input | TBD |

The qualitative pattern: fTetWild wins on quality at the cost of meshing time; Delaunay wins on meshing time at the cost of quality (acceptable at experience-mode scales, not at design-mode); GPU is the long-term path toward closing the quality-vs-time gap.

## Phase-B deliverable: benchmark on the canonical problem

The specific numerical bounds `sim-soft` enforces at ingest are set by the Phase-B benchmark against the [MuJoCo flex regression baseline](../../110-crate/04-testing.md) on the canonical compliant-cavity-plus-probe problem. The benchmark protocol:

1. Mesh the canonical geometry (cylindrical cavity, wall thickness $t_c$, length $L_c$, probe radius $r_p$) at three resolutions — ~3k, ~12k, ~30k tets — using each shipping generator (fTetWild, Delaunay+TetGen).
2. Run the canonical step — probe inserted to engagement depth $\delta$, Newton-converged equilibrium — and record Newton iteration count, final residual, stiffness-matrix condition number, wall-time.
3. Compare the reward ([Part 1 Ch 01](../../10-physical/01-reward.md)) against the MuJoCo flex baseline at the same geometry.
4. Set per-phase quality bounds as the loosest values across the three resolutions that still produce Newton convergence in ≤ the [Phase-B target iterate budget](../../50-time-integration/00-backward-euler.md) and reward-parity with the baseline.

The benchmark is a single xtask target (Phase B) that ships at the [Phase-B closing commit](../../110-crate/03-build-order.md) and feeds the specific-threshold entries in [Part 3 Ch 01's mesh-quality leaves](../../30-discretization/01-mesh-quality.md). Until it runs, the bounds are named as "Phase-B-benchmark-committed values" in the ingest checks; after it runs, the specific numbers land in the leaves and the generator-specific quality claims in this sub-leaf can be tightened.

## What the comparison does not do

- **Recommend a single generator.** [Ch 01 Claim 2](../01-tet-strategies.md) commits to different generators for different regimes; the comparison informs the per-regime choice, not a single winner.
- **Close [Part 6 Ch 05's open problem](../../60-differentiability/05-diff-meshing.md).** Mesh quality is about element shape; differentiability through meshing is about whether gradients flow across topology changes. Disjoint concerns. A high-quality mesh is still not differentiable through topology changes.
- **Set bounds on non-canonical-problem scenes.** The benchmark is tied to the compliant-cavity-plus-probe configuration. Designs with geometry outside the canonical family (high-aspect-ratio thin shells, dense interface bands) may need per-design tuning; the global bounds are what `sim-soft` ingest enforces, and per-design tuning is a `cf-design`-side concern.

## What this sub-leaf commits the book to

- **Three quality metrics are measured: aspect ratio, dihedral angle, stiffness-matrix conditioning.** Defined in [Part 3 Ch 01](../../30-discretization/01-mesh-quality.md); the Phase-B benchmark reports distributions, not just means.
- **Expected ranking: fTetWild > Delaunay > GPU (TBD).** Based on generator design, pending Phase-B confirmation.
- **Phase-B benchmark sets the ingest bounds.** Protocol: mesh canonical problem at three resolutions with each generator, run Newton + measure conditioning + compare reward to MuJoCo flex, set loosest bounds that maintain convergence and reward-parity.
- **Benchmark ships as an xtask target at Phase-B close.** Until then, ingest bounds are "Phase-B-benchmark-committed"; after, specific numbers land in [Part 3 Ch 01's leaves](../../30-discretization/01-mesh-quality.md).
