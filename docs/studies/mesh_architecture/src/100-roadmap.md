# Roadmap and open questions

What this part covers (when authored to depth):

## Phase A — gap fixes + first examples

In flight as the mesh examples PR. Two gap fixes plus 11 examples (Tier 1 + Tier 2 from [Part 8](80-examples.md)). Lands the foundation extensions soft-body needs and validates the mesh ecosystem visually for the first time.

- `mesh-types::AttributedMesh::extras: HashMap<String, Vec<f32>>` — extensible per-vertex attribute slot.
- `mesh-io::save_ply_attributed` — PLY writer for `AttributedMesh` with extras.
- 11 examples per the [Part 8 inventory](80-examples.md).

## Phase B — broader attribute typing

If use cases drive it: extend `extras` to support `Vec<u32>` (material IDs, integer labels) and `Vec<Vector3<f64>>` (vector fields like displacement direction). Probably an enum-typed value:

```rust
pub enum AttributeArray {
    F32(Vec<f32>),
    U32(Vec<u32>),
    Vec3(Vec<Vector3<f64>>),
}
pub extras: HashMap<String, AttributeArray>;
```

Drives PLY writer extensions (typed properties), 3MF metadata extensions (XML-typed values), and downstream consumer updates. Probably bundled with whichever soft-body or sim-cast use case first needs non-`f32` attributes.

## Phase C — Tier 3 examples

The five deferred examples from [Part 8](80-examples.md): `sdf_distance_field`, `lattice_gyroid_in_box`, `lattice_density_gradient`, `lattice_inside_shell`, `self_intersection_detection`. Each lands as its underlying use case bites — `lattice_*` when the layered silicone device middle layer needs lattice modeling; `sdf_distance_field` when casting demoldability analysis surfaces SDF-based metrics; `self_intersection_detection` when a robust import pipeline needs the visualization for debugging bad-CAD cases.

## Open question — tet mesh support

`mesh-types` does not currently include a `TetMesh` type; tet meshes live in `sim-soft::mesh`. This was the right call for the soft-body foundation (sim-soft's tet mesh has FEM-specific adjacency layouts and cached invariants). But two future use cases could push tet support upward into `mesh-types`:

1. **`sim-cast` sacrificial cores.** Hollow-cast parts may use lattice tet cores (e.g., PVA-printed gyroid tetrahedralized for stress analysis). If `sim-cast` grows tet meshing, the natural home for the type is `mesh-types`, not within `sim-cast`'s own crate.
2. **Generic tet-mesh interchange.** If `cf-design` ever needs to author tet-mesh primitives (rare), or if future external tools want tet-aware loading from the mesh-io family (e.g., a future `.msh` Gmsh format support), the type belongs at the foundation tier.

If/when this happens, the architectural question is: does `sim-soft::mesh::HandBuiltTetMesh` move to `mesh-types::TetMesh` (hard refactor; affects every soft-body test), or does `mesh-types::TetMesh` land as a separate type with conversion shims to/from `sim-soft`'s native form (soft introduction; some duplication)? The depth pass enumerates the tradeoffs.

Currently no driving force; flagged as future-when-relevant.

## Open question — typed precondition wrappers

Many operations across the ecosystem assume preconditions that aren't enforced at the type level:

- `mesh-sdf::point_in_mesh` assumes watertight.
- `mesh-offset::offset_mesh` assumes watertight + manifold.
- `mesh-shell::generate_shell` assumes watertight + manifold + closed boundary.
- `mesh-printability::validate_for_printing` assumes manifold.

Each consumer either trusts the input or runs its own repair pass. A type-level alternative: `CleanMesh` (or similar) wrapping `IndexedMesh` with a static guarantee that `mesh-repair::validate_mesh` returned without errors. Operations downstream of repair accept `&CleanMesh` instead of `&IndexedMesh`.

Pro: precondition violations become compile-time errors; clearer API contracts.
Con: every existing call site updates; the wrapper carries no runtime value beyond a marker; the discipline can be circumvented (`unsafe { CleanMesh::new_assumed_clean(mesh) }`).

The depth pass discusses this in detail; current lean is **not yet** — value/cost ratio doesn't justify the migration today.

## Open question — cross-domain reward composition

Both soft-body and casting will eventually contribute to a unified `RewardBreakdown` for the design loop. The mesh ecosystem doesn't directly compute reward terms today, but several operations could feed them:

- `mesh-printability::validate_for_printing` → printability reward term.
- `mesh-shell::ShellBuilder::build` failure → manufacturability classification.
- `mesh-measure::dimensions` → size-budget reward term.

Should the mesh ecosystem grow a reward-aware surface, or should reward composition be entirely the consuming domain's responsibility? Today: latter. Future: open question if the design loop's reward composition grows complicated enough that mesh-side operations should be self-aware about their role in optimization.

## Open question — GPU acceleration

All mesh-ecosystem crates are pure CPU. Several operations are computationally heavy and would benefit from GPU acceleration:

- `mesh-offset` marching cubes kernel.
- `mesh-sdf::SignedDistanceField::distance` (per-query closest-point search).
- `mesh-printability::find_optimal_orientation` (search over many orientations).
- `mesh-lattice` TPMS marching-cubes extraction.

Phase E in the soft-body book covers the GPU plumbing (wgpu-based, separate device from Bevy's rendering). The mesh ecosystem could ride the same machinery — `mesh-offset` as the most natural first port. Currently no driving force; flagged for when/if performance pressure surfaces.

## Open question — additional file formats

Current set: STL, OBJ, PLY, 3MF, STEP. Realistic additions:

- **VTK** — scientific visualization standard; preferred by ParaView for time-varying simulation data. Soft-body PLY frame-sequences could equivalently be VTK time-series.
- **USD (Universal Scene Description)** — Pixar's modern interchange format; gaining traction in industrial pipelines.
- **glTF** — modern game-engine-friendly format with PBR materials and animation; relevant if CortenForge's visualization layer wants to export to web viewers.
- **Gmsh `.msh`** — explicit volumetric mesh format; relevant if tet-mesh support matures.

Each is a feature-gated addition similar to the existing 3MF/STEP pattern. No current driving force; depth pass discusses each format's pros/cons.

## What this study does NOT commit to

- **Specific timelines.** Phase A is in flight; everything else lands when use cases drive it.
- **Stable API guarantees.** Pre-1.0; APIs may evolve as use cases bite. The deprecation policy follows whatever the workspace policy is at API-stability time.
- **Performance targets.** No specific FPS / throughput / memory targets. Foundation-level performance is "fast enough not to bottleneck consumers"; will be calibrated against real workloads as they materialize.
