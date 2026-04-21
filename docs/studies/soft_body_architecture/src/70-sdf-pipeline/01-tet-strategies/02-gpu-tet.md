# GPU tet generation

GPU tet generation is a [Phase E+](../../110-crate/03-build-order.md) target in `sim-soft`'s build order, not a Phase B-D default. This leaf is honest about the state of the art: there is no production-ready GPU tet mesher that accepts SDF input and produces an FEM-ready tet mesh with the quality guarantees [fTetWild](00-ftetwild.md) offers on CPU. The chapter is a forward-reference to open research and an architectural placeholder, not a description of shipped machinery.

## Why GPU tet is wanted

The [Ch 04 live re-mesh budget](../04-live-remesh.md) targets **≤500 ms** for topology-changing edits at design-mode resolution and notes "longer for larger meshes." CPU fTetWild on canonical-scale inputs (~30k tets) lands in the order-of-seconds range that fits the budget for small meshes and exceeds it for larger ones — the single biggest latency on the topology-changing path. Topology changes are rare (~5% of edits per [Ch 04's change-detection classification](../04-live-remesh/00-change-detection.md)) so the budget is tolerable on average, but a GPU tet generator that amortizes meshing over many compute units would make topology-changing edits competitive with the parameter-only hot path at all mesh sizes.

The [Part 11 Phase F deliverable](../../110-crate/03-build-order.md) (fully GPU-resident forward solve) depends on tet mesh data structures being GPU-accessible; a GPU mesher that produces the mesh directly in GPU buffers avoids a CPU→GPU transfer that would otherwise dominate the re-mesh latency. Phase F's solver upgrade and GPU-tet generation are architecturally paired targets, though they may ship in different phases.

## State of the art

The literature on GPU-native tet generation from SDF input is sparse. The nearest published work clusters into three groups, none of which is a drop-in fit:

- **GPU Delaunay refinement** (e.g., Qi et al. 2019, PACT 2019, "Computing Three-dimensional Constrained Delaunay Refinement Using the GPU") — parallel Bowyer-Watson-style refinement of an initial Delaunay tetrahedralization on GPU. Addresses refinement, not generation from implicit-surface input; would need a CPU-side surface-extraction + initial Delaunay step upstream.
- **Isosurface stuffing** ([Labelle & Shewchuk 2007](../../appendices/00-references/03-diff-sim.md#labelle-shewchuk-2007), ACM TOG 26(3), "Isosurface Stuffing: Fast Tetrahedral Meshes with Good Dihedral Angles") — CPU algorithm that generates tet meshes directly from an isosurface input (evaluating the implicit function at grid points, then applying a BCC lattice template). The algorithm's per-cell locality suggests GPU parallelism would work; a published GPU port targeting FEM-quality output is not available at the time of writing.
- **Other GPU geometry libraries** — graph-oriented or Voronoi-focused, not FEM-quality / implicit-surface-input oriented.

No single published pipeline meets all three requirements: (a) accepts an implicit surface / SDF directly, (b) runs on GPU end-to-end, (c) produces an FEM-ready mesh with aspect-ratio and dihedral-angle guarantees. Shipping `sim-soft`'s Phase-B default on CPU fTetWild is the honest choice — Phase E+ GPU tet is contingent on the research either being completed externally or on `sim-soft` implementing the adapter itself.

## `sim-soft`'s position

Through [Phase E](../../110-crate/03-build-order.md), meshing runs on CPU via [the fTetWild-lineage Rust pipeline](00-ftetwild.md) (design-mode default) or [the TetGen-lineage Rust Delaunay pipeline](01-delaunay.md) (experience-mode default). The mesh is transferred to GPU for simulation as a one-time per-mesh upload; the transfer cost is amortized across the many simulation steps between re-meshes.

For Phase F and later, two candidate paths are on the table:

1. **GPU port of Labelle-Shewchuk isosurface stuffing.** The algorithm's BCC-lattice-template structure is embarrassingly parallel; a direct WGSL-on-wgpu port is feasible. The output's dihedral-angle bounds are known-good (Labelle-Shewchuk's original bound is 5.71° minimum, well within Tet4 conditioning range). The output mesh would not match fTetWild's envelope guarantees on arbitrary input, but Phase-F designs are assumed to be [cf-design](../../110-crate/02-coupling/04-cf-design.md) outputs with predictable SDF topology, so the guarantee gap is acceptable.
2. **External adoption of a future published pipeline.** Research on GPU-native tet-meshing-from-implicit-surface is active (partially driven by NeRF and differentiable-rendering demand); a production-ready pipeline published between now and Phase F would be a drop-in replacement. `sim-soft`'s `sdf_bridge/` is designed to be generator-agnostic — the generator is a trait (concretely, the method invoked as `tet_generator.mesh_sdf(&sdf, resolution)` in [Ch 04's live-remesh path](../04-live-remesh.md)) and a new generator is a new trait impl.

The decision between (1) and (2) is a Phase-F call, not a Phase-B commitment. This sub-leaf commits only to the architectural hook (trait-based generator selection) and the honest placeholder (GPU tet is deferred, CPU tet is shipping).

## What this sub-leaf commits the book to

- **GPU tet is deferred to Phase E+.** Through Phase D, meshing is CPU-side via fTetWild or Delaunay; the mesh is GPU-uploaded once per re-mesh for simulation.
- **No published GPU pipeline meets `sim-soft`'s SDF-input + FEM-quality requirements today.** The chapter is an architectural placeholder, not shipping machinery.
- **The `sdf_bridge/` generator interface is trait-based.** A future GPU generator is a new trait impl; the rest of `sim-soft` does not change to accommodate it.
- **Phase F and later can choose between a GPU port of Labelle-Shewchuk isosurface stuffing or adoption of a future external pipeline.** The decision is deferred and tied to external research progress, not pre-committed.
- **The ≤500 ms topology-change budget is contingent on mesh size.** Ch 04 spine commits to ≤500 ms at design-mode resolution and notes "longer for larger meshes"; fTetWild on ~30k-tet inputs approaches or exceeds that boundary depending on SDF complexity. GPU tet is the upgrade path to pull the budget in.
