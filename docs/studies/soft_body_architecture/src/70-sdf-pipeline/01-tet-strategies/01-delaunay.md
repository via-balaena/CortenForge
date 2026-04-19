# Delaunay tetrahedralization

`sim-soft` ships classical Delaunay tetrahedralization as the [Ch 01 parent's Claim 2](../01-tet-strategies.md) experience-mode default and the design-mode fallback when [fTetWild times out](00-ftetwild.md). This leaf names the specific library, the input-preparation step (Delaunay is not robust to non-manifold input), and where the strategy falls short.

## Library commitment: TetGen

`sim-soft` uses Hang Si's [TetGen](https://www.wias-berlin.de/software/tetgen/) ([Si 2015](../../appendices/00-references/03-diff-sim.md#si-2015), ACM TOMS 41(2) Article 11) for Delaunay tetrahedralization. TetGen is a Delaunay-based quality tet mesher with configurable edge-length and aspect-ratio targets. The choice over alternatives (CGAL's `Mesh_3` package, gmsh's Delaunay kernel) is driven by two factors:

- **Single-function API for constrained Delaunay.** TetGen's `tetrahedralize` call takes a surface mesh + quality constraints and returns an interior tet mesh; no multi-step kernel setup.
- **Rust-binding surface.** A sys-wrapper crate provides a safe-Rust surface over the C++ library; `sim-soft` depends on it through `sdf_bridge/`, gated behind a Cargo feature flag so the TetGen dependency is opt-in.

[gmsh](https://gmsh.info/) ([Geuzaine & Remacle 2009](../../appendices/00-references/03-diff-sim.md#geuzaine-2009), IJNME 79:1309–1331) is a second-order option for cases where TetGen's limitations bite (non-manifold input that Delaunay cannot handle; see below). It is not the default because it adds a larger dependency for marginal gain on the target use case.

## Input preparation: Delaunay requires clean surfaces

Unlike [fTetWild's envelope pipeline](00-ftetwild.md), classical Delaunay tetrahedralization demands that its input surface be closed, manifold, and non-self-intersecting. A self-intersecting surface produces Delaunay ambiguity at the intersection curve; a non-manifold surface (edge shared by more than two triangles) has no valid Delaunay completion. TetGen fails hard on both; `sim-soft`'s binding propagates the failure as `MeshIngestError::DelaunayInputInvalid`.

For experience-mode (the primary Delaunay path), the input surface is the iso-surface of a known-clean geometric SDF — the scene is loaded once, the surface is extracted once, and the designer is not iterating on the SDF. The input is clean by construction at scene-load time; Delaunay's preconditions hold.

For design-mode fallback (the secondary Delaunay path, invoked when fTetWild times out), the input surface may not be clean — fTetWild was trying to handle defects and didn't complete. In this case `sim-soft` runs a pre-processing step before Delaunay: extract a coarser iso-surface at a reduced resolution (lowering topology-defect probability), run a self-intersection repair pass ([libigl](https://libigl.github.io/) ships routines for this), and feed the result into TetGen. The resulting mesh is coarser than fTetWild's target but is produced within the budget; the experience of "design edit stalled briefly then produced a coarse mesh" is still better than "design edit stalled indefinitely."

## Quality post-processing

TetGen's default quality settings produce modest aspect-ratio distributions and dihedral angles bounded away from 0° and π, acceptable for experience-mode but looser than fTetWild's Phase-B band. `sim-soft` does not invoke additional post-processing on Delaunay output; the quality is what TetGen produces and the [Part 3 Ch 01 mesh-quality ingest check](../../30-discretization/01-mesh-quality.md) validates it at the boundary. The concrete quality distribution across canonical-scale meshes is the Phase-B [quality-comparison deliverable](03-quality-compare.md). If a mesh fails ingest, it is reported back to the caller with the violating elements; the caller decides whether to retry at a coarser resolution or abort the edit.

## When Delaunay is enough

Experience-mode runs on a [few-thousand-tet mesh](../../10-physical/03-thesis.md) that is meshed once at scene load and held constant. The meshing cost is one-time and hidden by the scene-load screen; the simulation cost (60 FPS over a few-thousand tets) is what the experience-mode budget targets. Delaunay's output quality is sufficient for Tet4-constant-strain elements at this resolution — the conditioning of the stiffness matrix is adequate, Newton converges in a handful of iterations, the line-search does not fail for mesh-quality reasons.

The [quality comparison in §03](03-quality-compare.md) benchmarks the specific quality distribution on the canonical problem; the qualitative conclusion is that Delaunay + TetGen's default settings is adequate for experience-mode but insufficient for design-mode's tighter targets (where fTetWild's post-improvement passes pull aspect ratios closer to the regular-tet limit and dihedral angles away from the degenerate tails).

## What Delaunay does not do well

- **Non-manifold input.** Already covered; hard failure.
- **Arbitrary SDF pathologies.** Graceful degradation is not a Delaunay property. An SDF whose iso-surface has self-intersections needs [fTetWild](00-ftetwild.md) or a pre-processing pass; Delaunay alone fails.
- **Adaptive refinement starting point.** [Ch 03's red-green subdivision](../03-adaptive-refine.md) operates post-meshing on any generator's output but produces higher-quality results when the initial mesh is further from the aspect-ratio limit. Delaunay's looser initial quality gives adaptive refinement less headroom than fTetWild's output does.

## What this sub-leaf commits the book to

- **Delaunay via TetGen is the experience-mode default.** One-shot mesh at scene load, ~few-thousand tets, default quality settings. Cargo-feature-gated so the TetGen dependency is opt-in.
- **Delaunay is also the design-mode fallback when fTetWild times out.** A coarser iso-surface extraction plus libigl cleanup feeds into TetGen; the resulting mesh is coarser than fTetWild's target but produced within the design-mode time budget.
- **Delaunay fails hard on non-manifold or self-intersecting input.** The `MeshIngestError::DelaunayInputInvalid` is propagated; the caller retries at a coarser resolution or aborts.
- **No post-processing passes beyond TetGen's defaults.** Quality is what TetGen produces; [Part 3 Ch 01 ingest check](../../30-discretization/01-mesh-quality.md) validates at the `sdf_bridge/` boundary.
