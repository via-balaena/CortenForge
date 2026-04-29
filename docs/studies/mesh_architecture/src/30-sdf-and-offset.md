# mesh-sdf and mesh-offset — the mesh→SDF half of the bridge

What this part covers (when authored to depth):

## The two-direction SDF↔mesh bridge

CortenForge has two complementary SDF surfaces, sitting at opposite ends of an SDF↔mesh bridge:

| Direction | Crate | Role |
|---|---|---|
| **Analytic SDF → mesh** | `sim-soft::sdf_bridge::Sdf` (trait) | Design primitive — author SDFs (`SphereSdf`, `DifferenceSdf`, future custom SDFs), feed into a mesher (`SdfMeshedTetMesh::from_sdf`) to get a tet mesh. |
| **Mesh → numerical SDF** | `mesh-sdf::SignedDistanceField` | Query an existing triangle mesh — given a point, return signed distance to the nearest surface. |

These are NOT redundant. They serve different purposes — analytic SDFs are *inputs* to mesh generation; numerical SDFs are *outputs* of mesh queries. A typical end-to-end pipeline uses both: author with `sim-soft::sdf_bridge::Sdf`, mesh into a tet mesh, extract a triangle surface, run `mesh-sdf::SignedDistanceField::new(triangle_surface)` to get back a numerical SDF the rest of the toolchain can query.

The depth pass covers: when each direction is the right tool, the precision/cost tradeoff (analytic SDFs are exact and fast; numerical SDFs are approximations bounded by mesh resolution), and the cases where the round-trip is needed (e.g., `mesh-offset` operating on a body that started life as an analytic SDF).

## `mesh-sdf` — numerical SDF from a triangle mesh

```rust
pub struct SignedDistanceField { /* ... */ }

impl SignedDistanceField {
    pub fn new(mesh: IndexedMesh) -> SdfResult<Self>;
    pub fn distance(&self, point: Point3) -> f64;
}

pub fn signed_distance(point: Point3, mesh: &IndexedMesh) -> f64;
pub fn point_in_mesh(point: Point3, mesh: &IndexedMesh) -> bool;
```

Two API patterns: `SignedDistanceField::new` for many queries against the same mesh (amortizes any pre-processing); `signed_distance` for one-off queries. Underlying algorithm: closest-point-on-triangle for unsigned distance, ray-cast or winding-number for inside/outside determination.

The depth pass covers the precise algorithms, accuracy bounds, performance characteristics, and the failure modes (e.g., what happens for non-watertight meshes — the inside/outside test becomes ill-defined; consumers must validate watertight-ness via `mesh-repair::validate_mesh` first or accept the ambiguity).

## `mesh-offset` — offset surfaces via SDF + marching cubes

```rust
pub fn offset_mesh(mesh: &IndexedMesh, distance: f64, config: &OffsetConfig) -> OffsetResult<IndexedMesh>;
```

Given a closed triangle mesh and a scalar offset distance, produce a new triangle mesh whose surface is at the specified perpendicular offset. Positive distance = expansion (dilation), negative = contraction (erosion). Algorithm:

1. Compute SDF of the input mesh on a 3D grid (using `mesh-sdf`).
2. Subtract `distance` from every grid sample (effectively translating the SDF zero-isosurface by `distance` along the gradient).
3. Extract the new zero-isosurface via marching cubes.

Configurable resolution via `OffsetConfig::preview()` / `default()` / `high_quality()` — coarser grids are faster but introduce mesh artifacts; finer grids are slower but produce smoother offset surfaces.

The depth pass covers: marching cubes specifics (Lorensen-Cline ambiguity resolution, the topology guarantees the platform commits to), the resolution-vs-quality tradeoff space, when offset is the right tool (mold-cavity inversion for casting, shell-wall generation, blob-blending), and when it's not (sharp-feature preservation under offset is fundamentally a research-frontier problem).

## Compositional patterns

The depth pass discusses how `mesh-sdf` + `mesh-offset` compose with the rest of the ecosystem:

- **`mesh-shell`** uses `mesh-offset` internally for SDF-based shell wall generation.
- **`mesh-printability::find_optimal_orientation`** can use `mesh-sdf` for thin-wall detection (small SDF-distance regions = potentially-thin walls).
- **Future `sim-cast::MoldGeometry`** will use `mesh-offset` for mold-wall thickness analysis (offset the part inward, observe where the offset surface self-intersects = wall too thin).
- **`sim-soft::sdf_bridge::DifferenceSdf` + `SdfMeshedTetMesh::from_sdf`** is the analytic-SDF mirror of `mesh-offset(part, -wall_thickness)` for the cavity inversion case — both produce hollowed bodies, but from different starting points.

## What's NOT in these crates

- **Differentiable SDFs.** Neither `mesh-sdf` queries nor `mesh-offset` operations expose gradients with respect to mesh perturbations. Differentiable meshing is a research frontier (the soft-body book's [Part 6 Ch 05](../../soft_body_architecture/src/60-differentiability/05-diff-meshing.md) discusses it). For now, FD wrappers around the operations are the pragmatic path when gradients are needed.
- **GPU acceleration.** Both crates are pure CPU. Future Phase E work could port hot paths (the SDF distance kernel, the marching cubes kernel) to wgpu compute shaders; not currently planned.
- **Smooth-blend SDF operations.** Standard SDF combinators (smooth union, smooth intersection) are conceptually a sibling crate's job; today only `sim-soft::sdf_bridge::DifferenceSdf` exists, with smooth variants as future work.
