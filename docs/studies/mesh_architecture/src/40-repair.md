# mesh-repair — validation, welding, holes, winding, components

What this part covers (when authored to depth):

## Scope and tier

`mesh-repair` is L0 — depends on `mesh-types` only, no I/O. The "fix what's broken" layer that sits between mesh import and downstream operations that assume cleanliness (offset, shell, printability validation).

The crate covers eight categories of mesh hygiene:

1. **Validation** — `validate_mesh` returns a `MeshReport` with manifold-edge counts, boundary-edge counts, watertight check, vertex-count diagnostics.
2. **Vertex welding** — `weld_vertices` merges vertices within an epsilon distance, fusing duplicate-position vertices that arose from CAD imports or imprecise authoring.
3. **Degenerate-triangle removal** — strips zero-area triangles (`remove_degenerate_triangles` plus an `_enhanced` variant for near-degenerate cases).
4. **Duplicate-face removal** — strips faces that share the same vertex set, common in poorly-exported CAD meshes.
5. **Unreferenced-vertex cleanup** — drops vertices that no face references.
6. **Hole detection and filling** — `detect_holes` returns boundary loops; `fill_holes` triangulates them.
7. **Winding-order correction** — `fix_winding_order` propagates consistent CCW winding from a seed face across the connected mesh.
8. **Component analysis and self-intersection** — `find_connected_components`, `keep_largest_component`, `detect_self_intersections`.

`MeshAdjacency` is the supporting data structure — face-edge-vertex adjacency built on demand from `IndexedMesh`'s face list. Used internally by every operation that needs neighborhood queries; exposed publicly so callers can do their own adjacency-aware work.

## The repair pipeline pattern

The recommended flow is validate → diagnose → repair → re-validate. `RepairParams::default()` gives a sensible weld-tolerance / flag-set; `repair_mesh` applies a configurable subset of the operations above and returns a `RepairSummary` enumerating what was changed.

The depth pass covers: what each repair operation actually does to mesh topology and connectivity; the order operations must run in (welding before degenerate-removal because welding can produce degenerates; component analysis after welding for valid component count); the interaction with downstream operations (a non-watertight mesh produces ambiguous SDF queries from `mesh-sdf`, ill-defined offsets from `mesh-offset`, undefined shells from `mesh-shell`); and the cases where repair is genuinely impossible (mesh data so corrupted that there's no surface to recover).

## Repair as a precondition for downstream work

Many operations downstream of `mesh-repair` have implicit cleanliness preconditions:

| Operation | Requires | Failure mode if not met |
|---|---|---|
| `mesh-sdf::point_in_mesh` | watertight | inside/outside ambiguous |
| `mesh-offset::offset_mesh` | watertight, no self-intersect | offset surface has artifacts |
| `mesh-shell::generate_shell` | watertight, manifold | shell topology malformed |
| `mesh-printability::validate_for_printing` | manifold | overhang/orientation analysis unreliable |

The depth pass enumerates these explicitly. A future ecosystem-level change worth considering: should the operation crates accept a "repaired" wrapper type (`CleanMesh`?) that statically guarantees the precondition has been met? Currently every consumer either trusts the input or runs its own repair pass; a typed precondition would shift the contract from runtime to compile-time.

## Self-intersection — the hardest case

`detect_self_intersections` (`mesh-repair::intersect`) is the most computationally expensive operation in the crate — pairwise triangle intersection against an spatial-hash-accelerated candidate set. Returns a `SelfIntersectionResult` with the list of intersecting triangle pairs. **No automatic repair** — the crate detects but doesn't fix self-intersections, because automatic repair without losing significant geometry is an unsolved problem in the general case.

The depth pass covers: when self-intersections matter (any boolean-CSG operation; some manufacturability checks), the algorithmic frontier (commercial tools have proprietary fixes; academic literature has partial answers), and the recommended workflow (detect → fail loudly → designer fixes upstream → retry).

## What's NOT in `mesh-repair`

- **Mesh simplification (decimation).** Reducing triangle count while preserving shape is a separate concern; not in this crate. If/when needed, would land as a sibling `mesh-simplify` crate.
- **Subdivision (mesh refinement).** Increasing triangle density via Loop or Catmull-Clark subdivision; out of scope for repair, would be a separate crate.
- **Remeshing.** Replacing the entire face structure while preserving the surface is a research-frontier-adjacent operation; not in scope.
- **Geometric optimization (smoothing, regularization).** Laplacian smoothing, cotangent-weight smoothing, Taubin lambda-mu smoothing — geometric processing that isn't strictly "repair" of broken topology. Could land as a sibling crate or as `mesh-repair::smoothing` later.
