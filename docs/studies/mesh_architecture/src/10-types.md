# mesh-types — IndexedMesh, AttributedMesh, the SoA design

## The core type — `IndexedMesh`

Re-exported from `cf-geometry`, the universal triangle-mesh representation in CortenForge:

```rust
pub struct IndexedMesh {
    pub vertices: Vec<Point3<f64>>,  // Positions
    pub faces:    Vec<[u32; 3]>,     // Triangle vertex indices, CCW
}
```

Geometry only: positions plus a CCW-wound face index list. Right-handed coordinate system (X width, Y depth, Z height); CCW winding when viewed from outside; outward normals by right-hand rule. Unit-agnostic (downstream crates assume millimeters per documented convention). Every operation crate in the ecosystem operates on this type as input and/or output.

Per-vertex attributes (normals, colors, UVs, custom scalars) live one layer up in [`AttributedMesh`](#attributedmesh--geometry-plus-per-vertex-attributes). Earlier the type carried an `Option<Vec<Vector3<f64>>>` `normals` slot for marching-cubes output, but that put the same conceptual field on two types; the [dual-normals consolidation](#the-dual-normals-consolidation-resolved) folded analytical normals into the wrapper.

The depth pass covers: why indexed face list rather than face soup or half-edge data structures; the `f64` choice (vs `f32` to match modern game engines); when `IndexedMesh` is the wrong type and an attributed wrapper is needed.

## `AttributedMesh` — geometry plus per-vertex attributes

The wrapper that adds per-vertex attributes in struct-of-arrays (SoA) layout:

```rust
#[non_exhaustive]
pub struct AttributedMesh {
    pub geometry:   IndexedMesh,
    pub normals:    Option<Vec<Vector3<f64>>>,
    pub colors:     Option<Vec<VertexColor>>,
    pub zone_ids:   Option<Vec<u32>>,
    pub clearances: Option<Vec<f32>>,
    pub offsets:    Option<Vec<f32>>,
    pub uvs:        Option<Vec<(f32, f32)>>,
    pub extras:     BTreeMap<String, Vec<f32>>,
}
```

SoA design rationale: better cache locality for position-only iteration (the common case in geometric algorithms), zero memory overhead when an attribute is absent, each attribute is independently present or absent. The closed attribute set covers the bench/cf-design use cases that drove its original shape — normals for shading, colors for visualization, zones for region-tagging, clearances and offsets for shell-building, UVs for texture mapping. The `extras` slot (added in the mesh-ecosystem-to-par PR) lifts the closed-set ceiling for arbitrary per-vertex `f32` fields used by simulation snapshots and similar open-ended workflows.

`#[non_exhaustive]` lets the platform add new attribute slots without breaking external struct-literal construction; downstream crates already build via `AttributedMesh::new(geometry)` / `AttributedMesh::from(geometry)` and then mutate public fields.

The depth pass covers: the SoA-vs-AoS tradeoff in detail; why these specific attributes were chosen; attribute lifetime through repair / offset / shell pipelines (which operations preserve which attributes, which invalidate them).

## The dual-normals consolidation (resolved)

Earlier in the platform's life, both `IndexedMesh` and `AttributedMesh` carried an `Option<Vec<Vector3<f64>>>` `normals` field, with no enforced consistency rule when an `AttributedMesh` wrapped an `IndexedMesh` that happened to carry its own normals. The two slots had different origins — `IndexedMesh::normals` was bolted on later for marching-cubes output (analytical surface normals from the SDF gradient at each emitted vertex); `AttributedMesh::normals` predated that and was populated by `compute_normals` (area-weighted averaging from triangle geometry).

The slot's coexistence was an architectural artifact, not a deliberate design: the SDF mesher in `cf-design` produced gradient normals as a side effect, and the path of least resistance was to attach them to its already-returned `IndexedMesh`. The base type's own doc claimed "positions only … no normals," which the field then quietly contradicted.

**Resolution (mesh-ecosystem-to-par PR commit 2):** removed `IndexedMesh::normals`. `AttributedMesh::normals` is the canonical per-vertex normals slot. `cf-design::Solid::mesh*` and the lower-level `mesh_field*` meshers now return `AttributedMesh` directly, with gradient normals populated when the mesher produces them (marching cubes) and `None` when it doesn't (dual contouring, where vertices come from QEF minimization rather than corner crossings). `cf-design` gained a workspace dep on `mesh-types`, expressing in the dep graph what the API surface already implied. The `sim-bevy` renderer split into two functions: `triangle_mesh_from_indexed(&IndexedMesh)` for the geometry-only crease-angle path and `triangle_mesh_from_attributed(&AttributedMesh)` for the analytical-normals fast path; `spawn_design_mesh` takes `&AttributedMesh` and dispatches accordingly.

Trade-off accepted: callers of `Solid::mesh*` who only want triangles now receive a thin wrapper they unwrap via `.geometry`. The cost is one field access per call site; the benefit is a single source of truth across the ecosystem and a future-proof API surface for the per-vertex attributes that `mesh-shell`, `mesh-printability`, and the soft-body PLY snapshot path will increasingly want to thread through cf-design's pipeline.

One known follow-up: `cf-geometry::Shape::TriangleMesh` stores `Arc<IndexedMesh>` and so cannot carry per-vertex normals. The path `mechanism::shapes::generate_mesh → Shape::TriangleMesh → sim-bevy::mesh_from_shape` therefore renders Mesh-mode mechanism collision shapes via crease-angle splitting rather than analytical normals. Pre-consolidation this path used the analytical fast-path because `IndexedMesh` itself carried a normals field. No example in the repo currently exercises Mesh-mode mechanism visualization (all design examples render `Solid::mesh*` output directly through `spawn_design_mesh`), so the regression is dormant; the proper fix lives one tier up — either lifting `Shape` to a layer that knows about `AttributedMesh`, or making the renderer take a richer input alongside the geometry-only `Shape`.

## The `extras` slot — open-ended per-vertex scalars

The `colors` / `zone_ids` / `clearances` / `offsets` / `uvs` / `normals` set covers cf-design and bench use cases, but soft-body PLY snapshots (per-tet-step physics output: stress, strain, displacement magnitude, temperature, material ID) and similar open-ended workflows need attributes the closed set can't name. The `extras: BTreeMap<String, Vec<f32>>` slot fills that gap: arbitrary scalar attributes keyed by name, each `Vec<f32>` validated against `vertex_count()` via the `insert_extra` setter.

`BTreeMap` (over `HashMap`) is deliberate — deterministic iteration order matters for serde round-trips, golden-file tests, and PLY field-data export where the writer needs to emit attributes in a stable order. Insert via the validated setter (`insert_extra(name, values) -> Result<(), AttributeMismatchError>`) for the length-checked path; direct field mutation is allowed for callers who can vouch for the invariant themselves. The error type is `#[non_exhaustive]` so future failure modes (e.g., reserved attribute names) can be added additively.

`f32` only by design: most physics scalars (stress components, temperatures, density fractions) fit comfortably; integer `Vec<u32>` (material IDs) and vector `Vec<Vector3<f64>>` slots are not in `extras` today and will be added if use cases drive them rather than speculatively. PLY writes extras natively (one `property float <name>` per attribute); 3MF carries them as metadata; STEP doesn't represent them at all.

## What's NOT in `mesh-types`

- **No tetrahedral meshes.** Tet support lives in `sim-soft::mesh` (tet-specific data layouts and FEM-aware adjacency). Whether a `TetMesh` type belongs in `mesh-types` is an open architectural question; see [Part 10 roadmap](100-roadmap.md). Driving force would be `sim-cast` needing tet meshing for sacrificial cores or graded-density fills.
- **No half-edge / DCEL data structures.** Edge-adjacency queries live in `mesh-repair::MeshAdjacency` (built on demand from `IndexedMesh`'s face list). The depth pass discusses why CortenForge avoided baking a half-edge structure into the canonical type.
- **No quad meshes / polygon meshes.** Triangulation is enforced at file-load boundaries (PLY's polygon faces are fan-triangulated on import). Quad-aware algorithms aren't in scope for the platform.

## Coordinate and unit conventions

| Aspect | Convention | Source |
|---|---|---|
| Coordinate system | Right-handed, X width / Y depth / Z height | `mesh-types/lib.rs` |
| Face winding | CCW when viewed from outside | `mesh-types/lib.rs` |
| Normal convention | Outward by right-hand rule | `mesh-types/lib.rs` |
| Vertex precision | `f64` (positions), `f32` (attributes) | `IndexedMesh`, `AttributedMesh` |
| Index width | `u32` for triangle vertices | `IndexedMesh::faces` |
| Units | Unit-agnostic at type level; mm assumed by downstream operation crates | `mesh-types/lib.rs` |

The depth pass enumerates these in a single conventions appendix and discusses the tradeoffs (e.g., `u32` indices cap mesh size at ~4G vertices; `f64` positions vs `f32` is a memory-vs-numerical-tolerance tradeoff).
