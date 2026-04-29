# mesh-types — IndexedMesh, AttributedMesh, the SoA design

What this part covers (when authored to depth):

## The core type — `IndexedMesh`

Re-exported from `cf-geometry`, the universal triangle-mesh representation in CortenForge:

```rust
pub struct IndexedMesh {
    pub vertices: Vec<Point3<f64>>,           // Positions
    pub faces:    Vec<[u32; 3]>,              // Triangle vertex indices, CCW
    pub normals:  Option<Vec<Vector3<f64>>>,  // Optional per-vertex normals
}
```

Triangle mesh, indexed face list, with an optional per-vertex normal slot. Right-handed coordinate system (X width, Y depth, Z height); CCW winding when viewed from outside; outward normals by right-hand rule. Unit-agnostic (downstream crates assume millimeters per documented convention). Every operation crate in the ecosystem operates on this type as input and/or output.

The optional `normals` field is populated when the mesh is generated from an SDF via marching cubes — the marching-cubes pass computes analytical surface normals from the SDF gradient at each vertex, giving smooth shading without post-hoc face-normal averaging. When loaded from a file or built programmatically, `normals` is typically `None` and the rendering layer computes them from triangle geometry on demand.

The depth pass covers: why indexed face list rather than face soup or half-edge data structures; the `f64` choice (vs `f32` to match modern game engines); when `IndexedMesh` is the wrong type and an attributed wrapper is needed; and the dual-normals architectural artifact (see the `AttributedMesh` section below).

## `AttributedMesh` — geometry plus per-vertex attributes

The wrapper that adds per-vertex attributes in struct-of-arrays (SoA) layout:

```rust
pub struct AttributedMesh {
    pub geometry:   IndexedMesh,
    pub normals:    Option<Vec<Vector3<f64>>>,
    pub colors:     Option<Vec<VertexColor>>,
    pub zone_ids:   Option<Vec<u32>>,
    pub clearances: Option<Vec<f32>>,
    pub offsets:    Option<Vec<f32>>,
    pub uvs:        Option<Vec<(f32, f32)>>,
}
```

SoA design rationale: better cache locality for position-only iteration (the common case in geometric algorithms), zero memory overhead when an attribute is absent, each attribute is independently present or absent. The closed attribute set covers the bench/cf-design use cases that drove its original shape — normals for shading, colors for visualization, zones for region-tagging, clearances and offsets for shell-building, UVs for texture mapping.

**The dual-normals artifact.** Both `IndexedMesh` (via `cf-geometry`) and `AttributedMesh` carry an `Option<Vec<Vector3<f64>>>` `normals` field. This is a real redundancy worth documenting: `IndexedMesh::normals` was added for marching-cubes-generated meshes (analytical normals from SDF gradient), while `AttributedMesh::normals` predates that and is populated via area-weighted averaging through `compute_normals`. Today both fields coexist with no enforced consistency rule when an `AttributedMesh` wraps an `IndexedMesh` that happens to carry its own normals. The depth pass discusses the resolution options (deprecate one, define an authoritative-source rule, or merge into a single shared backing) and the migration cost of each.

The depth pass covers: the SoA-vs-AoS tradeoff in detail; why these specific attributes were chosen; the cases where the closed set is insufficient (driving the planned `extras: HashMap<String, Vec<f32>>` extension); attribute lifetime through repair / offset / shell pipelines (which operations preserve which attributes, which invalidate them).

## The closed-attribute limitation and the extension path

`AttributedMesh`'s attribute set is **closed** — adding a new per-vertex attribute (e.g., von Mises stress, displacement magnitude, material ID) requires editing the struct. For soft-body PLY snapshots (per-tet-step physics output) and similar open-ended use cases, this is too rigid.

The planned extension: an `extras: HashMap<String, Vec<f32>>` slot for arbitrary scalar attributes, each `Vec<f32>` validated against `vertex_count()`. Future possible variants for `Vec<u32>` (material IDs) and `Vec<Vector3<f64>>` (vector fields) if use cases drive them; start with `f32` only. This extension lands in the mesh examples PR alongside the corresponding `mesh-io` PLY writer extension that consumes it.

The depth pass covers: the extension's API shape, validation invariants, serialization through every existing format that supports custom attributes (PLY natively, 3MF via metadata, STEP not at all), and why a HashMap is the right choice over a Vec-of-pairs or a typed-attribute-registry pattern.

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
