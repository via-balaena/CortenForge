# cf-geometry: Shared Geometric Kernel

> **Status**: Draft v2 (stress-tested)
> **Author**: Jon Hillesheim + Claude
> **Date**: 2026-03-13

---

## Motivation

CortenForge has 52+ crates across 7 domains. Three of those domains — **mesh**,
**sim**, and **routing** — all represent geometry, and today they each do it
independently:

| Concept | mesh-types | sim-core | cf-spatial | route-types |
|---------|-----------|----------|------------|-------------|
| AABB | `Aabb` | `Aabb` | `Aabb` | `Aabb` |
| Sphere | — | `CollisionShape::Sphere` | `Sphere` | `Sphere` |
| Ray | — | (in raycast.rs) | `Ray` | — |
| Triangle | `Triangle` | `mesh::Triangle` | — | — |
| Triangle mesh | `IndexedMesh` | `TriangleMeshData` | — | — |
| Convex hull | — | `ConvexHull` + `HullGraph` | — | — |
| BVH | — | `Bvh` + `BvhPrimitive` | — | — |
| Height field | — | `HeightFieldData` | — | — |
| Shape enum | — | `CollisionShape` (10 variants) | — | — |
| Mesh trait | `MeshTopology` | — | — | — |

**Seven independent `Aabb` implementations** — 4 public (mesh-types, sim-core,
cf-spatial, route-types) plus 1 private (mesh-repair defines its own for local
BVH/query code). Two independent
triangle types. Two independent triangle mesh representations. Zero shared
geometric queries.

This is the exact problem that Parasolid solves for Siemens: one geometric
kernel, consumed by every product (NX, Simcenter, Tecnomatix). Without it,
every product reinvents geometry and they can never interoperate cleanly.

`cf-geometry` is CortenForge's Parasolid.

---

## Position in Architecture

```
                        nalgebra
                           |
                      cf-geometry          <-- NEW: shared geometric kernel
                     /     |      \
              mesh-types  sim-core  cf-spatial
                |          |           |
             mesh-*     sim-*      route-types
                |          |           |
             mesh-io    sim-bevy   route-pathfind
                 \         |         /
                  \        |        /
                   cortenforge (L1)
```

**Key constraint**: `cf-geometry` is **Layer 0** — zero Bevy, zero GPU, zero
framework dependencies. Pure `nalgebra` + `thiserror` + optional `serde`.
Same dependency profile as `cf-spatial`.

### Dependency Rules

| Crate | Depends on cf-geometry | What it uses |
|-------|----------------------|--------------|
| **mesh-types** | Yes | `Aabb`, `Triangle`, `IndexedMesh`, `Bounded` |
| **mesh-*** (processing crates) | Via mesh-types | Operates on `cf-geometry::IndexedMesh` |
| **sim-core** | Yes | `Shape`, `Aabb`, `Axis`, `Bvh`, `ConvexHull`, `HeightFieldData`, `SdfGrid`, `Ray`, `RayHit` |
| **sim-bevy** | Via sim-core | Renders `cf-geometry` shapes as Bevy meshes |
| **cf-spatial** | Yes | `Aabb`, `Sphere`, `Ray` (replaces local definitions) |
| **route-types** | Yes | `Aabb`, `Sphere` (replaces local definitions) |
| ~~curve-types~~ | Removed | Removed in workspace trim (2026-03-19); will be rebuilt when needed |

### Dependency Validation

Verified against the workspace: no circular dependency risk. `cf-geometry`
depends only on leaf crates (`nalgebra`, `thiserror`, `serde`). Feature flags
compose correctly — all consumers use the same `serde = ["dep:serde",
"nalgebra/serde-serialize"]` pattern. The workspace nalgebra (0.33, no extra
features) adds zero bloat.

sim-core and mesh-types are currently independent (no cross-references).
cf-geometry does not couple them — both depend on cf-geometry for geometric
primitives, not for each other's types.

---

## Design Principles

### 1. Geometry owns shape. Physics owns force.

`cf-geometry` answers: *What is this thing? Where is it? Does it overlap that
other thing?* It does **not** answer: *What force results from this overlap?*

- **In cf-geometry**: `Aabb`, `Triangle`, `ConvexHull`, `gjk_distance()`,
  `ray_cast()`, `closest_point()`
- **In sim-core**: `ContactForce`, `ConstraintRow`, `friction_cone()`,
  `solve_contacts()`

This pattern is proven in industry — geometry kernels that separate shape
representation from force computation scale better than monolithic designs.

### 2. One canonical type per concept.

There is exactly one `Aabb`. One `Triangle`. One `IndexedMesh`. Domains do
not define their own — they use `cf-geometry`'s. This is enforced by
dependency structure: the old types get deleted, not wrapped.

### 3. Enum for known shapes. Traits for queries.

The `Shape` enum covers the 10 fundamental primitives (matching MuJoCo's geom
types) and enables exhaustive `match` for algorithm dispatch. Since `Shape` is
a closed enum, free-function dispatch (`ray_cast(&shape)`) is cleaner than
trait-object dispatch (`dyn Geometric`).

Three separate opt-in traits at different phases:
- `Bounded` — universal: every shape has an AABB (Phase 1, needed immediately)
- `Geometric` — most shapes: closest_point, contains, ray_intersect (Phase 5)
- `SupportMap` — convex shapes only: support function for GJK (Phase 5)

Not all shapes have a natural support function (heightfields, SDFs). Bundling
`support()` into a universal trait forces awkward no-ops. Keeping query traits
separate from the universal `Bounded` trait avoids this.

### 4. Positions, not vertices. Attributes stay in the domain.

`cf-geometry::IndexedMesh` stores `Vec<Point3<f64>>` — positions only. No
normals, no colors, no UVs, no zone IDs. Domain-specific attributes stay in
the consuming layer (`mesh-types::AttributedMesh`).

This pattern (positions-only kernel mesh, attributes in the domain layer)
is well-established across geometry libraries and eliminates the highest-risk
migration in the original spec (splitting the Vertex type).

### 5. f64 everywhere. All types `Send + Sync`.

All coordinates are `f64`. No `f32` geometric types. The rendering layer
(Bevy, f32) converts at the boundary.

All cf-geometry types must be `Send + Sync`. No `Cell`, no `RefCell`, no
interior mutability. Physics warm-start state (`Cell<usize>` on ConvexMesh)
stays in sim-core, not in the geometry kernel.

### 6. Data, not pipeline.

`cf-geometry` types are data structures with geometric queries. They are not
ECS components, not physics objects, not renderable assets. No collision
pipeline, no world, no phase management. Geometry kernels that bundle pipeline
logic inevitably duplicate the consuming engine's pipeline — keep them separate.

### 7. No premature abstraction.

Traits are introduced only when a second implementor exists. `IndexedMesh`
gets inherent methods (not `MeshTopology` trait) because it is the only mesh
type in cf-geometry. The `Bounded` trait exists because multiple types
implement it (Aabb, Sphere, Shape, IndexedMesh). Start concrete, extract
traits when the need is proven.

---

## Type Catalog

### Tier 1: Primitives (used by every domain)

These types are so fundamental that all domains need them. They replace the
7 independent `Aabb` implementations and 2 independent `Sphere`/`Ray` types.

```rust
/// Axis-aligned bounding box.
///
/// Replaces: mesh_types::Aabb, sim_core::Aabb, cf_spatial::Aabb,
///           route_types::Aabb, plus 1 private Aabb in mesh-repair
///
/// Derives: Debug, Clone, Copy, PartialEq + optional Serialize/Deserialize
///
/// API is the superset of all 4 public implementations. Key design decision:
/// `new()` is const and non-normalizing (sim-core hot-path requirement).
/// `from_corners()` normalizes (safe path for mesh-types/cf-spatial/route-types).
/// `Default` returns `empty()` (min=+INF, max=-INF sentinel for accumulation).
pub struct Aabb {
    pub min: Point3<f64>,
    pub max: Point3<f64>,
}

impl Aabb {
    /// Pre-validated min/max. Caller ensures min <= max per axis.
    /// Use `from_corners` if ordering is unknown.
    pub const fn new(min: Point3<f64>, max: Point3<f64>) -> Self;

    /// Normalizes: auto-determines min/max per axis.
    pub fn from_corners(a: Point3<f64>, b: Point3<f64>) -> Self;

    pub fn from_center(center: Point3<f64>, half_extents: Vector3<f64>) -> Self;
    pub const fn from_point(point: Point3<f64>) -> Self;
    pub fn from_points(points: impl Iterator<Item = &Point3<f64>>) -> Self;
    pub fn empty() -> Self; // min=+INF, max=-INF
    pub fn is_empty(&self) -> bool;
    pub fn center(&self) -> Point3<f64>;
    pub fn half_extents(&self) -> Vector3<f64>;
    pub fn size(&self) -> Vector3<f64>;
    pub fn volume(&self) -> f64;
    pub fn surface_area(&self) -> f64;
    pub fn diagonal(&self) -> f64;
    pub fn max_extent(&self) -> f64;
    pub fn min_extent(&self) -> f64;
    pub fn contains(&self, point: &Point3<f64>) -> bool;
    pub fn overlaps(&self, other: &Self) -> bool;
    pub fn intersection(&self, other: &Self) -> Self;
    pub fn union(&self, other: &Self) -> Self;
    pub fn expand_to_include(&mut self, point: &Point3<f64>);
    pub fn expanded(&self, margin: f64) -> Self;
    pub fn corners(&self) -> [Point3<f64>; 8];
    pub fn extent(&self, axis: Axis) -> f64;
    pub fn longest_axis(&self) -> Axis;
}

/// Axis enumeration for spatial queries.
///
/// Replaces: sim_core::Axis
pub enum Axis { X, Y, Z }

/// Sphere defined by center and radius.
///
/// Replaces: cf_spatial::Sphere, route_types::Sphere
/// Radius must be non-negative. Clamped in new().
pub struct Sphere {
    pub center: Point3<f64>,
    pub radius: f64,
}

/// Ray defined by origin and direction.
///
/// Replaces: cf_spatial::Ray, sim_core raycast internals
/// Direction should be unit-length. Callers that have a unit vector use new().
/// Callers with an arbitrary direction use new_normalize().
pub struct Ray {
    pub origin: Point3<f64>,
    pub direction: Vector3<f64>,
}

impl Ray {
    /// Trusts caller: direction must be unit-length.
    pub fn new(origin: Point3<f64>, direction: Vector3<f64>) -> Self;
    /// Normalizes direction. Returns None if direction is zero.
    pub fn new_normalize(origin: Point3<f64>, direction: Vector3<f64>) -> Option<Self>;
    pub fn point_at(&self, t: f64) -> Point3<f64>;
}

/// Result of a ray intersection query against a geometric shape.
///
/// Replaces: sim_core::RaycastHit
/// NOTE: cf_spatial::RaycastHit (voxel grid) is a DIFFERENT type with
/// VoxelCoord — it stays in cf-spatial. These serve different purposes.
pub struct RayHit {
    pub distance: f64,
    pub point: Point3<f64>,
    pub normal: Vector3<f64>,
}
```

### Tier 2: Mesh Types (mesh + sim domains)

The canonical triangle mesh representation. Replaces both `mesh_types::IndexedMesh`
and `sim_core::TriangleMeshData` with a single type.

```rust
/// A triangle defined by three points (resolved coordinates, not indices).
///
/// Replaces: mesh_types::Triangle
/// NOTE: sim_core::mesh::Triangle (index-based, v0/v1/v2: usize) is a
/// different concept. It becomes [u32; 3] in face index arrays, not this type.
///
/// Carries all of mesh-types::Triangle's methods: normal(), area(), centroid(),
/// edges(), aspect_ratio(), is_degenerate(), reversed(), etc.
pub struct Triangle {
    pub v0: Point3<f64>,
    pub v1: Point3<f64>,
    pub v2: Point3<f64>,
}

/// Indexed triangle mesh — the canonical mesh type.
///
/// Positions only. No normals, no colors, no attributes. Domain-specific
/// attributes (color, UV, zone_id, clearance) stay in mesh-types as
/// parallel arrays on AttributedMesh.
///
/// Replaces: mesh_types::IndexedMesh, sim_core::TriangleMeshData
///
/// Ownership lifecycle:
///   mesh-io loads it (owned) -> mesh-repair fixes it (owned, mutated)
///   -> Arc::new() -> sim-core, sim-bevy, route-pathfind share the Arc
pub struct IndexedMesh {
    pub vertices: Vec<Point3<f64>>,
    pub faces: Vec<[u32; 3]>,  // CCW winding
}

impl IndexedMesh {
    // Construction
    pub fn new() -> Self;
    pub fn with_capacity(vertex_count: usize, face_count: usize) -> Self;
    pub fn from_parts(vertices: Vec<Point3<f64>>, faces: Vec<[u32; 3]>) -> Self;
    pub fn from_raw(positions: &[f64], indices: &[u32]) -> Self;

    // Topology (replaces MeshTopology trait — only 1 implementor)
    pub fn vertex_count(&self) -> usize;
    pub fn face_count(&self) -> usize;
    pub fn is_empty(&self) -> bool;
    pub fn triangle(&self, face_index: usize) -> Option<Triangle>;
    pub fn triangles(&self) -> impl Iterator<Item = Triangle> + '_;
    pub fn positions(&self) -> &[Point3<f64>];  // slice access

    // Geometric queries
    pub fn signed_volume(&self) -> f64;
    pub fn volume(&self) -> f64;
    pub fn is_inside_out(&self) -> bool;
    pub fn surface_area(&self) -> f64;
    pub fn compute_face_normals(&self) -> Vec<Vector3<f64>>;
    pub fn compute_vertex_normals(&self) -> Vec<Vector3<f64>>;

    // Transforms
    pub fn translate(&mut self, offset: Vector3<f64>);
    pub fn scale(&mut self, factor: f64);
    pub fn scale_centered(&mut self, factor: f64);

    // Combinators
    pub fn merge(&mut self, other: &Self);
    pub fn reserve(&mut self, additional_vertices: usize, additional_faces: usize);
}

/// Trait for types that have a bounding box.
///
/// Replaces: mesh_types::MeshBounds (renamed, simplified)
/// Implemented by: Aabb, Sphere, IndexedMesh, Shape, ConvexHull, HeightFieldData, SdfGrid
pub trait Bounded {
    fn aabb(&self) -> Aabb;
}
```

### Tier 3: Shape Primitives (sim + future CAD)

The fundamental geometric shapes. These match MuJoCo's 10 geom types.
The `Shape` enum is pure data in Phase 2. Query dispatch (Geometric,
SupportMap traits) is added in Phase 5.

```rust
/// Geometric shape — the canonical shape representation.
///
/// Replaces: sim_core::CollisionShape
///
/// Each variant is a pure geometric description. Physics properties
/// (mass, friction, restitution, warm-start cache) are NOT part of the
/// shape — those belong to the physics layer (sim-core).
///
/// All variants are Send + Sync. No Cell, no RefCell.
pub enum Shape {
    Sphere { radius: f64 },
    Plane { normal: Vector3<f64>, distance: f64 },
    Box { half_extents: Vector3<f64> },
    Capsule { half_length: f64, radius: f64 },
    Cylinder { half_length: f64, radius: f64 },
    Ellipsoid { radii: Vector3<f64> },

    /// Convex mesh. hull contains vertices + adjacency for GJK.
    /// No vertex redundancy: ConvexHull.vertices IS the vertex data.
    ConvexMesh { hull: ConvexHull },

    TriangleMesh {
        mesh: Arc<IndexedMesh>,
        bvh: Arc<Bvh>,
    },

    HeightField { data: Arc<HeightFieldData> },
    Sdf { data: Arc<SdfGrid> },
}
```

**Warm-start migration**: sim-core currently stores `warm_start: Cell<usize>`
on `CollisionShape::ConvexMesh`. Since cf-geometry requires `Send + Sync`,
the warm-start cache moves to sim-core. The support functions shown below
are Phase 5 deliverables — this section explains the *migration pattern*,
not Phase 2 scope:

```rust
// sim-core after migration
pub struct GeomState {
    pub warm_start: Cell<usize>,  // GJK hill-climbing cache, per-geom
}

// Phase 5: cf-geometry provides pure support (no warm-start):
pub fn support(hull: &ConvexHull, direction: &Vector3<f64>) -> Point3<f64>;

// Phase 5: sim-core wraps with warm-start acceleration:
fn support_with_cache(
    hull: &ConvexHull, direction: &Vector3<f64>, cache: &Cell<usize>,
) -> Point3<f64>;
```

### Tier 4: Acceleration Structures

Spatial acceleration structures. Pure geometry — no physics semantics.

```rust
/// Convex hull with adjacency graph for GJK hill-climbing.
///
/// Replaces: sim_core::ConvexHull + sim_core::HullGraph
/// Adjacency is always present (cheap to compute, always useful).
pub struct ConvexHull {
    pub vertices: Vec<Point3<f64>>,
    pub faces: Vec<[u32; 3]>,        // CCW triangulated faces
    pub normals: Vec<Vector3<f64>>,   // outward unit normals, one per face
    pub adjacency: Vec<Vec<u32>>,     // vertex neighbor graph
}

/// Compute convex hull of a point cloud (Quickhull).
/// Returns None if < 4 non-coplanar points.
pub fn convex_hull(
    points: &[Point3<f64>],
    max_vertices: Option<usize>,
) -> Option<ConvexHull>;

/// Bounding Volume Hierarchy for broad-phase spatial queries.
///
/// Replaces: sim_core::Bvh
pub struct Bvh { /* internal node tree */ }

impl Bvh {
    pub fn build(primitives: Vec<BvhPrimitive>, max_per_leaf: usize) -> Self;
    pub fn query(&self, aabb: &Aabb) -> Vec<usize>;
    pub fn root_aabb(&self) -> Aabb;
}

/// Primitive stored in BVH.
pub struct BvhPrimitive {
    pub aabb: Aabb,
    pub index: usize,
    pub data: u32,
}

/// Build BVH from triangle mesh.
pub fn bvh_from_mesh(mesh: &IndexedMesh) -> Bvh;

/// 2D height field grid.
///
/// Replaces: sim_core::HeightFieldData
pub struct HeightFieldData {
    heights: Vec<f64>,
    width: usize,
    depth: usize,
    cell_size: f64,
    min_height: f64,
    max_height: f64,
}

/// 3D signed distance field grid.
///
/// Replaces: sim_core::SdfCollisionData
pub struct SdfGrid {
    values: Vec<f64>,     // ZYX order
    width: usize,
    height: usize,
    depth: usize,
    cell_size: f64,
    origin: Point3<f64>,
    min_value: f64,       // Cached: deepest inside (for fast rejection)
    max_value: f64,       // Cached: furthest outside
}
```

### Tier 5: Geometric Queries (Phase 5)

Pure geometric query functions and traits — no physics, no forces. These
use free-function dispatch on the `Shape` enum (exhaustive match), not
trait-object dispatch.

```rust
// --- Traits (Phase 5 only) ---

/// Implemented by types that support closest-point, containment, and ray queries.
pub trait Geometric: Bounded {
    fn closest_point(&self, point: &Point3<f64>) -> Point3<f64>;
    fn contains(&self, point: &Point3<f64>) -> bool;
    fn ray_intersect(&self, ray: &Ray, max_distance: f64) -> Option<RayHit>;
}

/// Convex shapes only: support function for GJK.
pub trait SupportMap: Bounded {
    fn support(&self, direction: &Vector3<f64>) -> Point3<f64>;
}

// --- Free functions (preferred dispatch for Shape enum) ---

pub fn ray_cast(ray: &Ray, shape: &Shape, max_dist: f64) -> Option<RayHit>;
pub fn closest_point(shape: &Shape, point: &Point3<f64>) -> Point3<f64>;
pub fn gjk_distance(a: &impl SupportMap, b: &impl SupportMap) -> f64;
pub fn epa_penetration(a: &impl SupportMap, b: &impl SupportMap) -> Option<Penetration>;

pub struct Penetration {
    pub depth: f64,
    pub normal: Vector3<f64>,
    pub point_a: Point3<f64>,
    pub point_b: Point3<f64>,
}

// NOTE: convex_hull() and bvh_from_mesh() are Tier 4 / Phase 2 deliverables,
// not Phase 5. They live alongside their data structures, not in query/.
```

---

## What cf-geometry Is NOT

1. **Not a physics engine.** No mass, inertia, forces, constraints, or solvers.
   cf-geometry tells you two shapes overlap by 2mm; sim-core tells you the
   resulting 50N contact force.

2. **Not a rendering system.** No materials, colors, shaders, or GPU buffers.
   cf-geometry provides the mesh; sim-bevy converts it to a Bevy `Mesh3d`.

3. **Not a mesh processing library.** No repair, boolean, simplification, or
   remeshing. Those stay in mesh-* crates, operating on `cf-geometry::IndexedMesh`.

4. **Not curve/surface geometry.** Parametric curves (Bezier, NURBS) and
   surfaces will be rebuilt when needed (curve-types removed in workspace
   trim, 2026-03-19).

5. **Not a voxel system.** Voxel grids and occupancy maps stay in `cf-spatial`.
   cf-spatial uses `cf-geometry::Ray` as input but owns the voxel structures.
   cf-spatial's `RaycastHit` (which includes `VoxelCoord`) is a different type
   from `cf-geometry::RayHit` — they are not unified.

6. **Not a collision pipeline.** No collision world, no broad/narrow phase
   management, no contact graph. The pipeline stays in sim-core.

---

## What Moves Where

### mesh-types becomes a thin domain layer

```
STAYS in mesh-types (domain-specific):
+-- VertexAttributes { normal, color, zone_id, clearance_mm, offset, uv }
+-- VertexColor
+-- AttributedMesh (NEW: wraps IndexedMesh + parallel attribute arrays)
+-- mesh-specific helpers: flip_normals(), rotate_x_90(), place_on_z_zero()
+-- From impls for domain convenience

MOVES to cf-geometry (fundamental geometry):
+-- Aabb                    -> cf_geometry::Aabb
+-- Triangle                -> cf_geometry::Triangle
+-- IndexedMesh             -> cf_geometry::IndexedMesh
+-- MeshBounds trait        -> cf_geometry::Bounded

DROPPED (replaced by inherent methods on cf_geometry::IndexedMesh):
+-- MeshTopology trait      -> vertex_count(), face_count(), triangle(), triangles(), etc.
+-- Vertex struct           -> replaced by Point3<f64> in IndexedMesh.vertices
```

After migration, `mesh-types` depends on `cf-geometry` and re-exports core
types. The 47+ downstream crates (19 mesh-* processing + 28 examples +
cortenforge) continue to depend on `mesh-types`.

mesh-types wraps cf-geometry::IndexedMesh with attribute storage:

```rust
// mesh-types after migration
use cf_geometry::{IndexedMesh, Aabb, Triangle, Bounded};

/// Struct-of-arrays attribute storage, parallel to IndexedMesh.vertices.
/// SoA pattern: better cache performance for position-only iteration,
/// zero memory overhead when attributes are absent.
pub struct AttributedMesh {
    pub geometry: IndexedMesh,
    pub normals: Option<Vec<Vector3<f64>>>,
    pub colors: Option<Vec<VertexColor>>,
    pub zone_ids: Option<Vec<u32>>,
    pub clearances: Option<Vec<f32>>,
    pub offsets: Option<Vec<f32>>,
    pub uvs: Option<Vec<(f32, f32)>>,
}
```

Migration at call sites:
- `mesh.vertices[i].position` -> `mesh.vertices[i]` (simpler — it's a Point3 directly)
- `mesh.vertices[i].attributes.normal` -> `mesh.normals.as_ref().map(|n| n[i])` (requires AttributedMesh)
- Most mesh-* crates only access positions — they get simpler, not more complex.
- 12 files access `.attributes` — these require restructuring to use AttributedMesh.

### sim-core becomes a physics layer over cf-geometry

```
STAYS in sim-core (physics-specific):
+-- ContactPoint, ContactManifold, ContactForce
+-- Constraint rows, solver state
+-- Mass properties, inertia tensors
+-- Penalty parameters (solref, solimp, friction coefficients)
+-- Warm-start state (GeomState with Cell<usize>)
+-- All contact force computation

MOVES to cf-geometry (pure geometry):
+-- CollisionShape enum     -> cf_geometry::Shape
+-- Aabb + Axis             -> cf_geometry::Aabb, cf_geometry::Axis
+-- ConvexHull + HullGraph  -> cf_geometry::ConvexHull
+-- TriangleMeshData        -> cf_geometry::IndexedMesh + cf_geometry::Bvh
+-- HeightFieldData         -> cf_geometry::HeightFieldData
+-- SdfCollisionData        -> cf_geometry::SdfGrid
+-- Bvh + BvhPrimitive      -> cf_geometry::Bvh, cf_geometry::BvhPrimitive
+-- RaycastHit              -> cf_geometry::RayHit
+-- quickhull algorithm     -> cf_geometry::convex_hull()
```

sim-core wraps cf-geometry's `Shape` with physics metadata:

```rust
// sim-core after migration
use cf_geometry::Shape;

/// A collision geom: geometry + physics properties + warm-start cache.
pub struct Geom {
    pub shape: Shape,
    pub pose: Isometry3<f64>,
    pub margin: f64,
    pub friction: Vector3<f64>,
    pub solref: [f64; 2],
    pub solimp: [f64; 5],
    pub body_id: usize,
    pub warm_start: Cell<usize>,  // GJK cache (was inside CollisionShape)
}
```

### cf-spatial drops its Aabb/Sphere/Ray, depends on cf-geometry

```
STAYS in cf-spatial:
+-- VoxelGrid, VoxelCoord, GridBounds
+-- OccupancyMap
+-- RaycastHit (voxel-specific, includes VoxelCoord — NOT unified with cf-geometry::RayHit)
+-- Voxel raycasting, overlap queries

MOVES to cf-geometry:
+-- Aabb       -> cf_geometry::Aabb (re-exported by cf-spatial)
+-- Sphere     -> cf_geometry::Sphere (re-exported by cf-spatial)
+-- Ray        -> cf_geometry::Ray (re-exported by cf-spatial)
```

### route-types drops its Aabb/Sphere, depends on cf-geometry

```
STAYS in route-types:
+-- KeepOutZone (using cf-geometry::Aabb and cf-geometry::Sphere)
+-- RouteConstraints, PhysicalConstraints
+-- Path, VoxelPath, ContinuousPath, Waypoint
+-- All routing types

MOVES to cf-geometry:
+-- Aabb       -> cf_geometry::Aabb
+-- Sphere     -> cf_geometry::Sphere
```

---

## The "Geometry Is The Physics" Invariant

> **The mesh you see in the viewport is the same mesh the physics engine
> collides against is the same mesh the router plans around.**

Today this is broken. With cf-geometry:

```
STL file
  -> mesh-io loads it as cf_geometry::IndexedMesh     (owned)
    -> mesh-repair fixes it                            (owned, mutated)
      -> Arc::new(mesh)                                (shared from here)
        -> sim-core builds Shape::TriangleMesh         (Arc<IndexedMesh>)
          -> sim-bevy renders the SAME Arc             (zero-copy)
            -> route-pathfind queries the SAME Arc     (zero-copy)
```

One `Arc<IndexedMesh>`. One truth. The `Arc` boundary is explicit: you can
mutate the mesh while it's owned, but once shared it's immutable.

---

## Crate Structure

```
crates/cf-geometry/
+-- Cargo.toml
+-- src/
|   +-- lib.rs              // Re-exports, crate docs
|   +-- aabb.rs             // Aabb, Axis, Bounded trait
|   +-- sphere.rs           // Sphere
|   +-- ray.rs              // Ray, RayHit
|   +-- triangle.rs         // Triangle
|   +-- mesh.rs             // IndexedMesh (inherent methods, no trait)
|   +-- shape.rs            // Shape enum (Phase 2)
|   +-- convex_hull.rs      // ConvexHull, quickhull algorithm (Phase 2)
|   +-- bvh.rs              // Bvh, BvhPrimitive (Phase 2)
|   +-- heightfield.rs      // HeightFieldData (Phase 2)
|   +-- sdf.rs              // SdfGrid (Phase 2)
|   +-- query/              // (Phase 5)
|       +-- mod.rs
|       +-- ray_cast.rs
|       +-- closest_point.rs
|       +-- gjk.rs
|       +-- epa.rs
+-- tests/
    +-- aabb_tests.rs
    +-- mesh_tests.rs
    +-- shape_tests.rs
    +-- bvh_tests.rs
    +-- query_tests.rs
```

### Dependencies

```toml
[package]
name = "cf-geometry"
version = "0.1.0"
edition = "2024"

[dependencies]
nalgebra = { workspace = true }
thiserror = { workspace = true }
serde = { workspace = true, optional = true }

[features]
default = []
serde = ["dep:serde", "nalgebra/serde-serialize"]
```

---

## Migration Risk Matrix

| # | Migration | Risk | Type | Files | Key concern |
|---|-----------|------|------|-------|-------------|
| 1 | mesh-types::Aabb | **MED** | Semantic | 3 | `new()` normalizes vs. doesn't; `Default` sentinel differs |
| 2 | sim-core::Aabb | **HIGH** | Semantic | ~12 | `const fn new()` must not normalize (hot path); Axis coupling |
| 3 | cf-spatial::Aabb | LOW | Mechanical | 1 | Strict subset API |
| 4 | route-types::Aabb | LOW | Mechanical | 2 | One method rename (`from_center_extents` -> `from_center`) |
| 5 | mesh-types::Triangle | LOW | Mechanical | 5 | Identical structure |
| 6 | sim-core::mesh::Triangle | **HIGH** | Structural | ~8 | Index-based (usize), not coordinate-based. Becomes [u32; 3] |
| 7 | mesh-types::IndexedMesh | **HIGH** | Structural | ~25 | Vertex -> Point3 change; 12 files access .attributes |
| 8 | sim-core::TriangleMeshData | **HIGH** | Structural | ~20 | Bundles vertices+triangles+AABB+BVH+hull; must decompose |
| 9 | CollisionShape -> Shape | **HIGH** | Semantic | ~18 | Cell<usize> is !Sync; ~307 references across sim-* |
| 10 | MeshBounds -> Bounded | LOW | Mechanical | 4 | 1 implementor; rename caught at compile time |

**Critical path**: Migrations 7, 8, 9 are the hardest. They are sequenced
carefully in the session plan (Phase 3 before Phase 4) so the API is proven
on sim-core before the wider mesh-types blast radius.

---

## Kill Signals

Conditions that mean the spec needs fundamental rework or abandonment:

1. **Aabb semantic incompatibility.** If the dual-constructor approach
   (`new` vs `from_corners`) regresses sim-core collision benchmarks by >2%.
   **Test**: benchmark `aabb_from_geom()` in Session 8.

2. **Vertex->Point3 cascade.** If the IndexedMesh change touches >20 files
   with semantic changes (not just import paths) in mesh-* crates.
   **Test**: dry-run grep before starting Phase 4.

3. **Conformance test failures require physics changes.** If Phase 3 breaks
   conformance tests in ways that need physics behavior changes (not just
   type wiring), geometry and physics are inseparable.

4. **GJK warm-start regression.** If moving `Cell<usize>` to sim-core causes
   >5% regression on the GJK benchmark, the architecture doesn't work.

5. **Divergent evolution.** If we discover during implementation that sim-core
   and mesh-types need their geometry types to evolve independently (different
   fields, different invariants), unification is actively harmful.

**Threshold**: If Phase 3 takes more than 7 sessions or breaks more than 3
conformance tests requiring non-trivial fixes, stop and reassess.

---

## Incremental Value

Each phase is independently positive. No valley of despair.

| Stop after | State | Value |
|------------|-------|-------|
| Phase 1 | cf-geometry exists. cf-spatial + route-types migrated. | **Positive**: 2/7 Aabb dupes gone. Foundation proven. |
| Phase 2 | Shape, ConvexHull, Bvh etc. exist but unused by sim-core. | **Positive**: types proven and tested. Ready for Phase 3. |
| Phase 3 | sim-core fully migrated. mesh-types untouched. | **Strongly positive**: biggest duplication source unified. |
| Phase 4 | All 4 domains unified. Queries still in sim-core. | **Very positive**: "geometry is the physics" invariant realized. |
| Phase 5 | Complete. Queries extracted. | **Maximum**: clean architecture end-to-end. |

**Critical constraint**: Phases 3 and 4 are each **atomic**. A half-migrated
sim-core (using both internal types AND cf-geometry) is worse than either
extreme. Once started, each must finish.

---

## Implementation Sessions

18 sessions across 5 phases. Each session is scoped to one conversation.

### Phase 1: Foundation Types (Sessions 1-3)

**Session 1: Crate Skeleton + Tier 1 Primitives**
- Scope: Create `crates/cf-geometry/`, add to workspace. Implement `Aabb`
  (superset API, dual constructors, all methods), `Axis`, `Sphere`, `Ray`,
  `RayHit`. Full unit tests.
- Entry: none
- Exit: `cargo test -p cf-geometry` passes. All Tier 1 types exist.

**Session 2: Tier 2 Mesh Types**
- Scope: `Triangle` (all methods from mesh-types), `IndexedMesh`
  (Vec<Point3>, inherent methods), `Bounded` trait.
  Full unit tests including volume, surface_area, merge.
- Entry: Session 1 complete
- Exit: `cargo test -p cf-geometry` passes. IndexedMesh + Triangle + Bounded exist.

**Session 3: cf-spatial + route-types Migration**
- Scope: cf-spatial deletes Aabb/Sphere/Ray, depends on cf-geometry, re-exports.
  route-types deletes Aabb/Sphere, depends on cf-geometry, re-exports.
  Update downstream call sites (method renames: `intersects` -> `overlaps`,
  `from_center_extents` -> `from_center`).
- Entry: Session 1 complete (only Tier 1 types needed — cf-spatial and route-types
  don't use Triangle, IndexedMesh, or Bounded)
- Exit: `cargo test -p cf-spatial -p route-types -p route-pathfind -p route-optimize` passes.

### Phase 2: Shapes + Acceleration (Sessions 4-7)

**Session 4: ConvexHull + Quickhull**
- Scope: `ConvexHull` struct. Port quickhull algorithm from sim-core
  (`convex_hull(points, max_vertices) -> Option<ConvexHull>`).
  Unit tests including degenerate cases.
- Entry: Session 1 complete (needs Aabb)
- Exit: `cargo test -p cf-geometry` passes. Quickhull produces correct hulls.

**Session 5: BVH**
- Scope: `Bvh`, `BvhPrimitive`. Port BVH construction (median split) from
  sim-core. `Bvh::query()` for AABB overlap. `bvh_from_mesh()`.
- Entry: Session 2 complete (needs Aabb, IndexedMesh)
- Exit: `cargo test -p cf-geometry` passes. BVH queries correct.

**Session 6: HeightField + SdfGrid**
- Scope: `HeightFieldData` (bilinear interp, normal computation, cell queries).
  `SdfGrid` (trilinear interp, gradient, min/max value cache).
- Entry: Session 1 complete (needs Aabb)
- Exit: `cargo test -p cf-geometry` passes.

**Session 7: Shape Enum**
- Scope: `Shape` enum (10 variants, data only). `Bounded` impl for Shape.
  No query dispatch yet (Phase 5).
- Entry: Sessions 4-6 complete (needs ConvexHull, Bvh, HeightFieldData, SdfGrid)
- Exit: `cargo test -p cf-geometry` passes. Shape enum with all variants.

### Phase 3: sim-core Migration (Sessions 8-12) [ATOMIC]

**Session 8: sim-core Aabb + Axis**
- Scope: sim-core depends on cf-geometry. Delete `sim_core::Aabb` and
  `sim_core::Axis`. Update all sim-core call sites (~12 files).
  Benchmark `aabb_from_geom()` before/after (**kill signal #1**).
- Entry: Session 1 complete
- Exit: `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` passes.
  No benchmark regression.

**Session 9: sim-core ConvexHull + Bvh**
- Scope: Delete `sim_core::ConvexHull`, `HullGraph`, quickhull. Delete
  `sim_core::Bvh`, `BvhPrimitive`. Replace with cf-geometry imports.
- Entry: Sessions 4-5 + Session 8 complete
- Exit: `cargo test -p sim-core -p sim-conformance-tests` passes.

**Session 10: sim-core TriangleMeshData**
- Scope: Replace `TriangleMeshData` internals with `Arc<cf_geometry::IndexedMesh>`
  + `Arc<cf_geometry::Bvh>`. sim-core's index-based `Triangle` becomes `[u32; 3]`
  or a type alias. Update sim-bevy rendering (usize -> u32 index cast).
  This is the hardest session in Phase 3.
- Entry: Session 9 complete
- Exit: `cargo test -p sim-core -p sim-bevy -p sim-conformance-tests` passes.

**Session 11: sim-core HeightField + SDF**
- Scope: Replace `HeightFieldData` with `cf_geometry::HeightFieldData`.
  Replace `SdfCollisionData` with `cf_geometry::SdfGrid`.
- Entry: Session 6 + Session 8 complete
- Exit: `cargo test -p sim-core -p sim-conformance-tests` passes.

**Session 12: CollisionShape -> Shape + Conformance Sweep**
- Scope: Replace `CollisionShape` enum with `cf_geometry::Shape`. Move
  `warm_start: Cell<usize>` to sim-core `GeomState` (or per-geom field on Model).
  Benchmark GJK (**kill signal #4**). Full 79-test conformance sweep.
- Entry: Sessions 9-11 complete
- Exit: `cargo test -p sim-conformance-tests` 79/79. No GJK regression.

### Phase 4: mesh-types Migration (Sessions 13-15) [ATOMIC]

**Session 13: mesh-types Core Types**
- Scope: mesh-types depends on cf-geometry. Delete `mesh_types::Aabb`
  (re-export cf-geometry). Delete `mesh_types::Triangle` (re-export).
  Delete `MeshBounds` trait (re-export `Bounded`).
- Entry: Session 2 complete (can run in parallel with Phase 3)
- Exit: `cargo test -p mesh-types` passes.

**Session 14: mesh-types IndexedMesh + Downstream**
- Scope: Replace `mesh_types::IndexedMesh` with `cf_geometry::IndexedMesh`
  (re-export). Delete `mesh_types::Vertex` + `VertexAttributes` as required
  fields. Introduce `AttributedMesh` (SoA pattern). Delete `MeshTopology`
  trait (inherent methods on IndexedMesh). Verify all 47+ downstream crates.
  Dry-run grep first (**kill signal #2**).
- Entry: Session 13 complete
- Exit: `cargo test -p mesh-types -p mesh -p mesh-io -p mesh-repair`
  (+ spot-check 5+ more) passes.

**Session 15: Private AABB Cleanup**
- Scope: mesh-repair: delete private Aabb definition, use cf-geometry::Aabb.
- Entry: Session 14 complete
- Exit: `cargo test -p mesh-repair` passes.

### Phase 5: Geometric Queries (Sessions 16-18)

**Session 16: Ray Cast + Closest Point**
- Scope: `ray_cast()` free function (dispatches on Shape enum). `closest_point()`
  free function. Port per-shape implementations from sim-core.
- Entry: Session 7 complete
- Exit: `cargo test -p cf-geometry` passes. Results match sim-core's implementations.

**Session 17: GJK + EPA**
- Scope: Port GJK distance, EPA penetration from sim-core. Implement
  `SupportMap` trait + `Geometric` trait. Support functions for each shape.
  No warm-start in cf-geometry (pure functions).
- Entry: Session 16 complete
- Exit: `cargo test -p cf-geometry` passes. GJK/EPA results match sim-core.

**Session 18: Integration + Cleanup**
- Scope: sim-core collision pipeline calls cf-geometry queries. Delete
  duplicate algorithm code from sim-core. Full conformance sweep.
- Entry: Sessions 12 + 17 complete
- Exit: ALL sim conformance tests pass. Zero duplicate geometry code in sim-core.

### Session Dependencies

Each session's prerequisites (verified against entry criteria):

```
S1:  (none)
S2:  S1
S3:  S1              (only Tier 1 types needed)
S4:  S1              (needs Aabb)
S5:  S2              (needs Aabb + IndexedMesh)
S6:  S1              (needs Aabb)
S7:  S4, S5, S6      (needs ConvexHull + Bvh + HeightFieldData + SdfGrid)
S8:  S1              (needs Aabb + Axis)
S9:  S4, S5, S8      (needs cf-geometry ConvexHull + Bvh + sim-core on cf-geometry)
S10: S9
S11: S6, S8          (needs cf-geometry HeightFieldData + SdfGrid + sim-core on cf-geometry)
S12: S9, S10, S11    (all sim-core types migrated -> swap CollisionShape)
S13: S2              (needs Triangle + IndexedMesh + Bounded)
S14: S13
S15: S14
S16: S7              (needs Shape enum)
S17: S16
S18: S12, S17        (needs both sim-core migrated + queries implemented)
```

**Parallelism opportunities:**
- S3, S4, S6, S8 can all start after S1 (four independent workstreams)
- S5, S13 can both start after S2
- Phase 3 (S8-S12) and Phase 4 (S13-S15) can overlap
- Phase 5 (S16-S17) can overlap with Phase 3 (S8-S12)

---

## Prior Art (studied for architectural lessons — none are dependencies)

These external projects were studied to learn from their design decisions and
mistakes. **None are CortenForge dependencies.** CortenForge's physics engine
is sim-core — we build our own. The value here is architectural patterns, not code.

| Project | Architecture | Lesson applied to cf-geometry |
|---------|-------------|-------------------------------|
| **Parry/Rapier** | Parry = geometry kernel. Rapier imports Parry, adds physics. | Validates separating geometry types from physics logic. Confirms positions-only mesh. |
| **ncollide** | Bundled `CollisionWorld` pipeline. Dropped in Parry rewrite. | Don't bundle pipeline logic in the kernel — that mistake cost a full rewrite. |
| **geo-types** | Shared types crate, re-exported by `geo`. Minimal deps. | Validates the shared-types-crate pattern. Monorepo eliminates version-coupling pain. |
| **Trimesh** (Python) | Core = vertices + faces. Attributes = parallel dicts. | Validates positions-only mesh + SoA attributes in domain layer. |
| **nalgebra** | Purely linear algebra. Zero geometric primitives. | cf-geometry fills exactly the gap nalgebra leaves open. |

---

## Coordinate System

cf-geometry uses a **right-handed, Z-up** coordinate system:

- **X**: forward / width
- **Y**: left / depth
- **Z**: up / height

This matches MuJoCo, cf-spatial, mesh-types, sim-core, and standard robotics.
Bevy (Y-up) converts at the Layer 1 boundary, as it does today.

---

## Open Questions

### Q1: Should cf-geometry own SDF generation?

**Resolved: No.** cf-geometry owns `SdfGrid` (the data type). mesh-sdf owns
the mesh-to-SDF algorithm. sim-core owns SDF collision queries. Same pattern
as BVH: cf-geometry owns the structure, consumers own domain-specific algorithms.

### Q2: `u32` vs `usize` for face indices?

**Resolved: `[u32; 3]`.** Matches GPU convention, Bevy, MuJoCo, and the
existing mesh-types. Index-type mismatches between crates are a known pain
point in Rust ecosystems. Standardize on u32.

### Q3: How does IndexedMesh handle vertex data?

**Resolved: `Vec<Point3<f64>>`.** Positions only, no Vertex struct, no normals.
Geometry kernels universally store positions only — attributes belong to the
domain layer. This eliminates the highest-risk migration (Vertex type split).
Normals are computed on demand (`compute_vertex_normals()`) or stored in
mesh-types `AttributedMesh`.

### Q4: Should Aabb::new() normalize min/max?

**Resolved: No.** `Aabb::new()` is `const fn`, non-normalizing (sim-core
hot-path requirement). `Aabb::from_corners()` normalizes (safe path). Dual
constructors serve both use cases. mesh-types call sites change
`Aabb::new(a, b)` -> `Aabb::from_corners(a, b)` where inputs may be unordered.

### Q5: Should MeshTopology be a trait?

**Resolved: No (for now).** Only one type implements it (`IndexedMesh`). Start
with inherent methods, extract a trait when a second implementor appears. This
avoids premature abstraction (Design Principle 7).

---

## Success Criteria

1. **Zero `Aabb` duplication.** One `Aabb` type, used by all 4 domains + 3 private sites.
2. **One mesh truth.** `Arc<IndexedMesh>` flows from loading through simulation
   through rendering. Same object, same memory.
3. **79/79 conformance.** All sim-core conformance tests pass after migration.
4. **47+/47+ downstream.** All mesh-types downstream crates compile and pass tests.
5. **Visible = collidable.** The Bevy mesh rendered on screen is built from
   the same `IndexedMesh` that sim-core collides against.
6. **Layer 0 purity.** cf-geometry has zero Bevy, zero GPU, zero framework deps.
7. **Send + Sync.** All cf-geometry types are thread-safe. No interior mutability.
8. **No benchmark regression.** AABB construction and GJK support within 2%
   of current sim-core performance.

---

*This document defines the architectural foundation. Implementation follows
the session plan above. The 5 phases are independently valuable — each is a
valid stopping point where the codebase is strictly better than before.*
