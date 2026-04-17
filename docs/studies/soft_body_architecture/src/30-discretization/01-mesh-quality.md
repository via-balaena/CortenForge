# Mesh quality

A tet mesh with bad-quality elements poisons the whole solver. Aspect ratios outside the valid band or dihedral angles near 0 or π cause the elastic-tangent matrix to be badly conditioned, which inflates [Newton's iteration count](../50-time-integration/00-backward-euler.md), can cause spurious [line-search failures](../50-time-integration/00-backward-euler/02-line-search.md) that trigger [adaptive-$\Delta t$ shrinks](../50-time-integration/02-adaptive-dt.md) for reasons unrelated to contact or dynamics, and shows up as visible artifacts in the rendered deformation. This chapter names the quality metrics `sim-soft` enforces at mesh ingest and the specific bounds each is checked against.

| Section | What it covers |
|---|---|
| [Aspect ratio bounds](01-mesh-quality/00-aspect-ratio.md) | Ratio of the longest edge to the insphere radius; must be ≤30 for Phase D, ≤20 for Phase H with Tet10 |
| [Dihedral angle bounds](01-mesh-quality/01-dihedral.md) | Angle between adjacent tet faces; must be ≥10° and ≤170° globally, with 95th-percentile bounds tighter |
| [Volume consistency](01-mesh-quality/02-volume.md) | Per-tet signed volume ≥ a small positive threshold; inverted tets (negative signed volume) fail mesh ingest outright |

Three claims.

**Quality is enforced at ingest, not during simulation.** The [`sdf_bridge/`](../110-crate/00-module-layout/08-sdf-bridge.md) module runs a quality pass after [tet generation](../70-sdf-pipeline/01-tet-strategies.md) and either (a) accepts the mesh, (b) post-processes to lift it into the valid band (Laplacian smoothing, quality-driven edge flips per [fTetWild](../70-sdf-pipeline/01-tet-strategies/00-ftetwild.md)'s post-improvement passes), or (c) rejects the mesh and reports the violating elements to the caller. Simulation-time quality checks are off; the ingest-time guarantee is that every tet the solver sees is already within the bounds.

**Bounds are looser for Tet4, tighter for Tet10.** Tet4's constant-strain assumption is tolerant of aspect ratio up to ≈30 before conditioning dominates; Tet10's quadratic shape functions are more sensitive to element shape because the quadrature integration becomes inaccurate on elongated or skewed quadratic elements. Phase H's Tet10-in-band commitment ([Ch 00 Claim 2](00-element-choice.md)) requires the contact-band quality to meet the tighter bounds; the bulk-Tet4 region keeps the looser bounds.

**Inverted tets are a hard failure.** A tet with negative signed volume is geometrically inverted — its shape functions evaluate to invalid values, and any constitutive law evaluated on it produces garbage. `sim-soft` refuses to simulate an inverted mesh; the [`sdf_bridge/`](../110-crate/00-module-layout/08-sdf-bridge.md) returns a `MeshIngestError::InvertedElements(Vec<TetId>)` that the caller ([Part 7 Ch 04's live-remesh](../70-sdf-pipeline/04-live-remesh.md)) handles by either fixing-in-place (if the inversion is local and a single edge-flip resolves it) or falling back to a coarser mesh. Silent acceptance of inverted elements would be worse than an explicit failure; the latter surfaces the bug, the former corrupts the reward function without warning.
