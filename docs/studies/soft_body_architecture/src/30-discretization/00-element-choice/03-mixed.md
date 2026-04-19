# Mixed element meshes

The [element-choice parent](../00-element-choice.md) committed `sim-soft` to **Tet10-in-band, Tet4-elsewhere** as the Phase H targeting pattern, with fully automatic mixed-element refinement deferred beyond Phase H. This leaf writes down what "mixed-element mesh" means at the data-structure level, the constraint that has to hold at the Tet4↔Tet10 interface, and the per-element-type tagging the [`element/`](../../../110-crate/00-module-layout/01-element.md) module needs.

## Per-element type tagging

A mixed mesh is the same vertex array, edge graph, and tet array as a homogeneous mesh, plus an additional per-element `ElementType` tag the [`element/`](../../../110-crate/00-module-layout/01-element.md) module's element-iteration loop dispatches on:

```rust,ignore
enum ElementType { Tet4, Tet10 }
struct MixedTetMesh {
    vertices: Vec<Position>,
    tets: Vec<[VertexId; 4]>,          // 4 corner vertex indices per tet
    midpoint_nodes: Vec<EdgeId>,       // populated only for Tet10-tagged tets
    element_type: Vec<ElementType>,    // one tag per tet
}
```

Tet4-tagged tets reference 4 vertex IDs through `tets[i]`. Tet10-tagged tets reference the same 4 corners plus 6 midpoint-vertex IDs reachable through the `midpoint_nodes` map keyed by edge. Vertices in `vertices` are the union of corner vertices and midpoint vertices; the [Part 8 Ch 01 BSR-3 storage](../../../80-gpu/01-sparse-matrix/01-bsr.md) treats them uniformly as $3 \times 3$ blocks, with the global stiffness sparsity carrying both the Tet4-derived edges (corner-corner) and the Tet10-derived edges (corner-midpoint and midpoint-midpoint).

The dispatch happens at the per-element loop, not in the [`Material`](../../../20-materials/00-trait-hierarchy/00-trait-surface.md) trait — material laws do not need to know which element type they are running inside, only that they are called with the correct $F$ value at the correct evaluation count (1 for Tet4, 4 for Tet10 per the [Tet10 sub-leaf](01-tet10.md)).

## The interface constraint

A Tet4 element shares a face (3 corner vertices) with a neighbor that may be Tet4 or Tet10. When the neighbor is also Tet4, both elements interpolate displacement *linearly* across the shared face; the displacement at any face point is determined by the 3 shared corners and matches automatically — no constraint is needed. When the neighbor is Tet10, the Tet10 side interpolates *quadratically* across the shared face using the 3 corners plus the 3 edge-midpoint nodes that lie on the face; the Tet4 side still interpolates linearly. The shared-face displacement does not match between the two interpolations unless the midpoint nodes are constrained to lie on the linear-interpolation line.

The constraint is per-edge: for every edge shared between a Tet4 element and a Tet10 element, the edge's midpoint vertex $X_m$ must satisfy

$$ u(X_m) = \tfrac{1}{2}\,\big(u(X_a) + u(X_b)\big) $$

where $X_a, X_b$ are the edge endpoints. Mechanically: the midpoint deformation is the average of the two corner deformations, which is exactly the linear-interpolation result on the Tet4 side. Three implementation paths: (a) treat $u(X_m)$ as not a free DOF and substitute the constraint directly into the sparse system before solving (DOF elimination), reducing the global DOF count; (b) keep $u(X_m)$ as a free DOF and add an equality constraint $u(X_m) - \tfrac{1}{2}(u(X_a) + u(X_b)) = 0$ to the system, enforced by Lagrange multipliers or by penalty; (c) keep $u(X_m)$ free and let the solver discover the constraint approximately through normal stiffness coupling (incorrect; produces a small but persistent face-displacement mismatch that shows up as visible artifacts).

`sim-soft`'s Phase H Tet10-in-band path picks (a) — DOF elimination at the band's outer boundary edges. The midpoint nodes on a band-boundary edge are not free DOFs in the global system; they are "phantom" nodes whose values are computed from the corners during stress evaluation but are never asked of the solver. This keeps the global DOF count clean and avoids the Lagrange-multiplier saddle-point machinery that the [Part 2 Ch 05 mixed u-p](../../../20-materials/05-incompressibility/01-mixed-up.md) treatment would otherwise introduce here unnecessarily.

## Where the band boundary lives

The user defines the Tet10-in-band region by an SDF mask: any tet whose centroid lies inside the user's `band_sdf` is tagged Tet10; tets outside are Tet4. The band-boundary edges are the edges shared between an inside-tet and an outside-tet; the [`sdf_bridge/`](../../../110-crate/00-module-layout/08-sdf-bridge.md) module detects these at mesh-ingest time and populates the `midpoint_nodes` constraint table from them.

In Phase H this band-mask is user-authored; in a post-Phase-I "fully automatic mixed-element" extension, the same data structure consumes a band derived from the [Part 7 Ch 03 stress-gradient refinement criterion](../../../70-sdf-pipeline/03-adaptive-refine.md). The data flow is the same; the band-source changes from user-mask to solver-output.

## Why fully automatic mixed-element is deferred

The deferral the [parent's Claim 4](../00-element-choice.md) made has two parts. First, the band-evolution machinery: as deformation progresses and stress concentrations move, the band would have to migrate; that requires re-tagging tets, re-allocating midpoint nodes, and transferring solver state across the re-tagging — a non-trivial extension of the [Part 7 Ch 04 live-remesh path](../../../70-sdf-pipeline/04-live-remesh.md). Second, the test-surface explosion: every Tet10-edge / Tet4-edge interface is a regression-test surface where the constraint enforcement can fail subtly (visual artifacts at the band boundary, lost gradients through the constraint elimination, etc.); the [Part 11 Ch 04 testing strategy](../../../110-crate/04-testing.md) regression suite would expand significantly to cover the band-migration cases.

Phase H ships the static-band case because the band is user-authored and migration is not needed inside a Phase H deliverable. The dynamic-band case is the post-Phase-I concern Claim 4 calls out.

## What this sub-leaf commits the book to

- **Mixed mesh = same vertex array + per-element `ElementType` tag.** The [`element/`](../../../110-crate/00-module-layout/01-element.md) module dispatches on the tag at the per-element loop; downstream layers (sparse storage, material trait) do not change.
- **Tet4↔Tet10 interface constraint is DOF elimination at band-boundary edges.** Midpoint nodes on band-boundary edges are not free DOFs; their values are interpolated from corners during stress evaluation. No Lagrange multipliers, no penalty, no saddle-point machinery added at this layer.
- **Phase H ships static user-authored band-mask only.** The `band_sdf` is a user-supplied SDF that determines element type per tet at mesh-ingest. The band does not move during simulation.
- **Dynamic-band fully-automatic mixed-element is post-Phase-I.** The required machinery — band migration during simulation, state transfer across re-tagging, expanded test surface — is acknowledged but not in scope through Phase I.
