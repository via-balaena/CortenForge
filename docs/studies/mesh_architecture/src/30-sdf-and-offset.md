# mesh-sdf and mesh-offset — the mesh→SDF half of the bridge

What this part covers (when authored to depth):

## The two-direction SDF↔mesh bridge

CortenForge has two complementary SDF surfaces, sitting at opposite ends of an SDF↔mesh bridge:

| Direction | Crate | Role |
|---|---|---|
| **Analytic SDF → mesh** | `sim-soft::sdf_bridge::Sdf` (trait) | Design primitive — author SDFs (`SphereSdf`, `DifferenceSdf`, future custom SDFs), feed into a mesher (`SdfMeshedTetMesh::from_sdf`) to get a tet mesh. |
| **Mesh → numerical SDF** | `mesh-sdf::Signed<TriMeshDistance, S>` | Query an existing triangle mesh — given a point, return signed distance to the nearest surface. |

These are NOT redundant. They serve different purposes — analytic SDFs are *inputs* to mesh generation; numerical SDFs are *outputs* of mesh queries. A typical end-to-end pipeline uses both: author with `sim-soft::sdf_bridge::Sdf`, mesh into a tet mesh, extract a triangle surface, compose a numerical SDF over it with `mesh-sdf` (`Signed { distance: TriMeshDistance::new(triangle_surface)?, sign }`) that the rest of the toolchain can query.

The depth pass covers: when each direction is the right tool, the precision/cost tradeoff (analytic SDFs are exact and fast; numerical SDFs are approximations bounded by mesh resolution), and the cases where the round-trip is needed (e.g., `mesh-offset` operating on a body that started life as an analytic SDF).

## `mesh-sdf` — numerical SDF from a triangle mesh

The API is two **orthogonal oracles** — unsigned distance and sign — composed
into a signed-distance source. This is the oracle-decomposition architecture
(see `docs/MESH_SDF_ORACLE_DECOMPOSITION_SPEC.md`); it replaced the earlier
single `SignedDistanceField` struct whose one face-normal sign heuristic
flipped signs at vertex/edge regions.

```rust
// Unsigned distance oracle — parry3d BVH-backed closest-triangle query.
pub struct TriMeshDistance { /* parry3d TriMesh + BVH */ }
impl TriMeshDistance {
    pub fn new(mesh: IndexedMesh) -> SdfResult<Self>;
}

// Composition: any `UnsignedDistance` oracle ⊥ any `Sign` oracle.
pub struct Signed<D: UnsignedDistance, S: Sign> { pub distance: D, pub sign: S }
impl<D: UnsignedDistance, S: Sign> Signed<D, S> {
    pub fn evaluate(&self, p: Point3) -> f64;          // signed distance
    pub fn unsigned_distance(&self, p: Point3) -> f64;
    pub fn closest_point(&self, p: Point3) -> Point3;
}

// Sign oracle #1 — angle-weighted pseudo-normal (fast; well-formed meshes).
pub struct PseudoNormalSign { /* ... */ }
impl PseudoNormalSign {
    pub fn from_distance(distance: &TriMeshDistance) -> Self;  // shares the BVH
}

// Sign oracle #2 — flood-fill topological reachability (robust; cleaned scans).
// Convenience ctor builds `TriMeshDistance` + `FloodFillSign` in one call.
pub fn flood_filled_sdf(
    mesh: IndexedMesh,
    bounds: Aabb,
    cell_size: f64,
    wall_threshold_factor: f64,  // WALL_THRESHOLD_FACTOR_DEFAULT
) -> SdfResult<(Signed<TriMeshDistance, FloodFillSign>, FloodFillReport)>;
```

Build the distance oracle once (its BVH amortizes across every query), then
choose the sign oracle to match the input: `PseudoNormalSign` for well-formed
synthetic meshes, `FloodFillSign` for cleaned body-part scans. Underlying
algorithm: parry3d's BVH gives unsigned distance and closest-point in
O(log faces); the sign oracle decides inside/outside independently.

The depth pass covers the precise algorithms, accuracy bounds, performance
characteristics, and the failure modes. The key one — non-watertight meshes,
where a naive inside/outside test becomes ill-defined — is exactly what drove
the two-oracle split: `PseudoNormalSign` is fast but fragile there, while
`FloodFillSign` derives the sign from topological reachability and stays robust
on cleaned scans. Consumers with well-formed meshes can still validate
watertight-ness via `mesh-repair::validate_mesh` and use the faster oracle.

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
