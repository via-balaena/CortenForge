# Aspect ratio bounds

The [mesh-quality parent](../01-mesh-quality.md) named aspect ratio as an ingest-time gate — looser bounds for Tet4-tagged tets, tighter bounds for Tet10-tagged tets, with specific thresholds set against the [MuJoCo flex regression baseline](../../../120-roadmap/01-track-1b.md) at Phase D. This leaf writes down the aspect-ratio metric `sim-soft` uses, why it (rather than alternative shape metrics) is the gate, and the structural relationship between the chosen threshold and Newton-iteration count downstream.

## The metric — radius ratio

`sim-soft` uses the **radius ratio** $\rho$ as the per-tet aspect-ratio metric:

$$ \rho = \frac{r_\text{ins}}{r_\text{circ}} $$

where $r_\text{ins}$ is the radius of the inscribed sphere (largest sphere fitting inside the tet) and $r_\text{circ}$ is the radius of the circumscribed sphere (smallest sphere containing the tet). For a regular tet (all edges equal), $\rho = 1/3$ exactly; for a degenerate sliver-tet (four nearly-coplanar vertices), $\rho \to 0$. The metric is dimensionless, scale-invariant, and bounded in $(0, 1/3]$.

The radius ratio is preferred over two alternatives `sim-soft` does not use as the primary gate:

- **Edge-ratio aspect** (longest edge / shortest edge): bounded in $[1, \infty)$, easy to compute, but insensitive to certain bad shapes (a sliver-tet with all four edges of similar length but vertices nearly coplanar has edge-ratio ≈ 1 yet is geometrically degenerate). The radius ratio catches this case; edge-ratio does not.
- **Mean-ratio metrics** (variants normalized to 1.0 at the regular tet): closely related to radius ratio and equally valid as quality metrics. `sim-soft` picks radius ratio because the inscribed-circumscribed-sphere geometric interpretation is the form mesh-improvement post-passes ([fTetWild](../../../70-sdf-pipeline/01-tet-strategies/00-ftetwild.md), Laplacian smoothing schedules) typically optimize.

## Why aspect ratio drives Newton-iteration count

The element stiffness matrix $K^e$ of a tet with small $\rho$ is badly conditioned in a specific way: its smallest eigenvalues correspond to deformation modes nearly aligned with the tet's degenerate direction (the direction perpendicular to the nearly-coplanar face for a sliver-tet), and the eigenvalue spread $\kappa(K^e) = \lambda_\text{max}/\lambda_\text{min}$ deteriorates rapidly as $\rho \to 0$. The global stiffness matrix's condition number $\kappa(K)$ inherits the worst per-element condition number through standard FEM-conditioning bounds, and the [Part 5 Ch 00 Newton iteration](../../../50-time-integration/00-backward-euler.md) on a badly-conditioned tangent inflates iteration count to convergence (or fails the [line search](../../../50-time-integration/00-backward-euler/02-line-search.md) outright in pathological cases).

The downstream effect chain is therefore: bad aspect ratio → bad stiffness conditioning → inflated Newton iterations → inflated wall-clock per timestep, and possibly spurious [adaptive-$\Delta t$ shrinks](../../../50-time-integration/02-adaptive-dt.md) when the Newton solver falls back rather than converges. The mesh-quality gate cuts the chain at the source.

## Threshold setting and the Tet4 / Tet10 split

The specific threshold values $\rho_\text{min}^{\text{Tet4}}$ and $\rho_\text{min}^{\text{Tet10}}$ are Phase D measurement deliverables — they are picked by running the [MuJoCo flex regression baseline](../../../120-roadmap/01-track-1b.md) at progressively tighter $\rho_\text{min}$ values until the Newton-iteration-count regression stabilizes. The qualitative ordering is fixed:

$$ \rho_\text{min}^{\text{Tet10}} \;>\; \rho_\text{min}^{\text{Tet4}} $$

Tet10's quadratic shape functions amplify the conditioning sensitivity to element shape — the higher-order basis exposes more deformation modes, and badly-shaped Tet10 elements suffer disproportionately from quadrature-integration error on top of the conditioning penalty. The [Phase H Tet10-in-band commitment](../../00-element-choice/01-tet10.md) therefore requires the contact-band region's $\rho$ to clear the tighter Tet10 threshold; the bulk Tet4 region passes under the looser Tet4 threshold.

The thresholds are mesh-author-facing — `MeshIngestError::PoorAspectRatio { tet_id, rho }` reports the failing tet and its $\rho$ value, and the caller has the option to either (a) accept the failure and reject the mesh, (b) re-run upstream meshing with a lower $\rho_\text{min}$ tolerance setting at the [`sdf_bridge/`](../../../110-crate/00-module-layout/08-sdf-bridge.md) layer, or (c) request mesh-improvement-pass repair within the bridge. The threshold values themselves do not change at runtime; they are a [Part 11 Ch 04 testing-strategy](../../../110-crate/04-testing.md) regression-locked configuration.

## Distribution-level acceptance, not just per-tet

A mesh with one bad tet at $\rho = 0.05$ in a 100k-tet mesh is rejected; a mesh with all tets clearing $\rho_\text{min}$ is accepted. The 95th-percentile and minimum statistics are reported to the caller alongside the pass/fail verdict, so that mesh authors can track the tail of the quality distribution as well as the worst case. The 95th-percentile statistic is specifically the indicator that the upstream-meshing parameters are well-tuned; if 95% of tets are at $\rho > 0.2$ but the worst-case is $\rho = 0.06$, a single mesh-improvement pass typically lifts the worst case across the threshold without touching the bulk.

## What this sub-leaf commits the book to

- **Radius ratio $\rho = r_\text{ins}/r_\text{circ}$ is the per-tet aspect-ratio metric.** Edge-ratio and mean-ratio alternatives are rejected as primary gates; the radius-ratio interpretation aligns with the [fTetWild post-improvement passes](../../../70-sdf-pipeline/01-tet-strategies/00-ftetwild.md) `sim-soft` consumes upstream.
- **Threshold values are Phase D measurement deliverables, not pre-chosen constants.** The Tet4 and Tet10 minimum-$\rho$ thresholds come from regression-stabilization runs against the [MuJoCo flex baseline](../../../120-roadmap/01-track-1b.md); the values are then regression-locked in the [Part 11 Ch 04 testing-strategy](../../../110-crate/04-testing.md).
- **Tet10 thresholds are tighter than Tet4 thresholds.** Quadrature-integration error and conditioning sensitivity both scale up with element-shape sensitivity at higher polynomial order; the [Tet10 sub-leaf](../../00-element-choice/01-tet10.md)'s 4-point Gauss rule depends on this.
- **Distribution-level acceptance with worst-case + 95th-percentile reporting.** The verdict is per-tet-binary at the threshold but the diagnostic statistics returned to the caller are distributional, supporting upstream parameter tuning.
