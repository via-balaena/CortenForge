# Raycast Consolidation & Scene Query Spec

**Status:** Complete
**Effort:** M
**Prerequisites:** None
**Test baseline:** sim domain tests pass (`cargo test -p sim-core`)

## Context

The raycasting codebase has two problems:

1. **~960 LOC of duplicated geometry code.** `sim-core/src/raycast.rs` (1,231 LOC)
   reimplements all 10 ray-shape intersection algorithms that already exist in
   `cf-geometry/src/query/ray_cast.rs` (929 LOC). Additionally,
   `closest_point_on_triangle` (84 LOC) is duplicated in `sim-core/src/mesh.rs`.

2. **No public scene-level raycast.** The only code that casts a ray against all
   geoms in a scene is buried inside the rangefinder sensor evaluation
   (`sensor/position.rs:284-337`). There is no reusable API for LIDAR-style
   queries, visibility checks, or the upcoming raycasting examples.

This spec eliminates the duplication and adds the missing API.

## Goal

After this spec:
- `sim-core::raycast_shape` delegates to `cf_geometry::ray_cast` — zero duplicated algorithms
- `closest_point_on_triangle` has a single source of truth in cf-geometry
- A new `raycast_scene()` public API enables scene-level ray queries
- The rangefinder sensor uses `raycast_scene()` instead of its own inline loop
- Net ~870 LOC deleted

---

## Phase 1 — Import `closest_point_on_triangle`

**Why:** Identical algorithm in both crates. cf-geometry's version is already
public at the crate root.

### File: `sim/L0/core/src/mesh.rs`

- Delete `closest_point_on_triangle` (lines 689–772, ~84 LOC)
- Add import: `use cf_geometry::closest_point_on_triangle;`
- Callers (same file, no signature change):
  - `triangle_sphere_contact()` — line 651
  - `closest_point_on_segment_to_triangle()` — line 819

### File: `sim/L0/core/src/lib.rs`

- Update re-export at line 149. Currently re-exports from `mesh::`. Change to:
  ```rust
  pub use cf_geometry::closest_point_on_triangle;
  ```
  Remove `closest_point_on_triangle` from the `pub use mesh::{...}` block.
  Public API preserved — `sim_core::closest_point_on_triangle` still exists.

### Verification

```
cargo test -p sim-core -- mesh
```
Existing tests `test_closest_point_on_triangle_vertex` and
`test_closest_point_on_triangle_face` validate the drop-in replacement.

---

## Phase 2 — Delete `ray_triangle_intersection`, use `cf_geometry::ray_triangle`

**Why:** The private `ray_triangle_intersection` in raycast.rs (lines 953–1004,
~50 LOC) is a Möller–Trumbore implementation identical to cf-geometry's public
`ray_triangle`.

### File: `sim/L0/core/src/raycast.rs`

- Add imports: `use cf_geometry::{Ray, RayHit, ray_cast, ray_triangle};`
- Delete `ray_triangle_intersection` (lines 953–1004)
- In `raycast_triangle_mesh_data` and `raycast_indexed_mesh`: replace calls to
  `ray_triangle_intersection(origin, dir, v0, v1, v2, max)` with:
  ```rust
  let local_ray = Ray::new(local_origin, local_dir);  // construct once before loop
  // inside BVH callback / brute-force loop:
  if let Some(hit) = ray_triangle(&local_ray, v0, v1, v2, cutoff) {
      let (t, pt, normal) = (hit.distance, hit.point, hit.normal);
      // ... existing closest-hit tracking ...
  }
  ```
- Rewrite `test_ray_triangle_intersection` test to call `ray_triangle` with a `Ray`.

### Verification

```
cargo test -p sim-core -- raycast
```

---

## Phase 3 — Delegate `raycast_shape` to `cf_geometry::ray_cast`

**Why:** This is the core deduplication. All 10 per-shape private functions in
sim-core reimplement algorithms that cf-geometry already provides. The only
unique logic sim-core adds is the world-space Pose transform — that's ~15 lines.

### File: `sim/L0/core/src/raycast.rs`

Replace the entire body of `raycast_shape` (lines 83–172) with:

```rust
pub fn raycast_shape(
    shape: &Shape,
    shape_pose: &Pose,
    ray_origin: Point3<f64>,
    ray_direction: UnitVector3<f64>,
    max_distance: f64,
) -> Option<RaycastHit> {
    // Transform ray to shape's local space
    let local_origin = shape_pose.inverse_transform_point(&ray_origin);
    let local_dir = shape_pose.inverse_transform_vector(ray_direction.as_ref());
    let local_ray = Ray::new(local_origin, local_dir);

    // Delegate to cf-geometry (all 10 shapes, local-space)
    let hit = ray_cast(&local_ray, shape, max_distance)?;

    // Transform hit back to world space
    let world_point = shape_pose.transform_point(&hit.point);
    let world_normal = safe_normalize(
        &shape_pose.transform_vector(&hit.normal),
        hit.normal,
    );
    Some(RaycastHit::new(hit.distance, world_point, world_normal))
}
```

Delete all 10 private per-shape functions (~800 LOC):
- `raycast_sphere` (lines 177–217)
- `raycast_plane` (lines 222–258)
- `raycast_box` (lines 261–321)
- `raycast_capsule` (lines 327–425)
- `raycast_cylinder` (lines 428–495)
- `raycast_ellipsoid` (lines 500–577)
- `raycast_convex_mesh` (lines 582–639)
- `raycast_heightfield` (lines 642–721)
- `raycast_sdf` (lines 724–777)
- `raycast_indexed_mesh` (lines 879–950)

Keep:
- `RaycastHit` struct and `safe_normalize` helper
- `raycast_triangle_mesh_data` (works with sim-core's `TriangleMeshData` type)

### Convex mesh correctness upgrade

sim-core's `raycast_convex_mesh` currently uses a **bounding sphere approximation**
(comment on line 599: "For a proper implementation, we'd use GJK-raycast").
cf-geometry's `ray_convex_mesh` does proper face-by-face Möller–Trumbore. This
is a correctness improvement — rays that previously false-hit (inside bounding
sphere but outside actual hull) will now correctly miss.

Add a regression test:
```rust
#[test]
fn test_raycast_convex_mesh_not_bounding_sphere() {
    // Long thin tetrahedron: bounding sphere much larger than hull
    // Ray aimed outside hull but inside bounding sphere → must return None
}
```

### Test updates

Tests that call private per-shape functions directly must be rewritten to go
through `raycast_shape` with a `Pose`. The existing `test_raycast_shape_dispatch`
test (line 1140) is the model pattern:

```rust
let shape = Shape::Sphere { radius: 1.0 };
let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
let hit = raycast_shape(&shape, &pose, origin, direction, 10.0);
```

### Verification

```
cargo test -p sim-core -- raycast
cargo test -p sim-core -- mesh
```

---

## Phase 4 — Add `raycast_scene`

**Why:** MuJoCo's `mj_ray()` is the standard scene-level ray query. The
rangefinder sensor reimplements it inline. The upcoming raycasting examples need
this as a public API.

### File: `sim/L0/core/src/raycast.rs`

Add new struct and function:

```rust
/// Result of a scene-level ray cast, identifying both the hit geom and
/// the intersection details.
#[derive(Debug, Clone, Copy)]
pub struct SceneRayHit {
    /// Index of the geom that was hit.
    pub geom_id: usize,
    /// Intersection details in world coordinates.
    pub hit: RaycastHit,
}

/// Cast a ray against all geoms in the scene, returning the closest hit.
///
/// Matches MuJoCo's `mj_ray` API. Iterates over all geoms, applying optional
/// filters, and returns the nearest intersection.
///
/// # Filters
///
/// - `bodyexclude` — skip all geoms attached to this body (e.g., to avoid
///   self-intersection). To exclude static/worldbody geoms, pass `Some(0)`.
/// - `geomgroup` — 6-element boolean array. A geom is skipped if
///   `!geomgroup[geom.group]`. Pass `None` to include all groups.
///
/// # Returns
///
/// The closest `SceneRayHit`, or `None` if no geom is hit within `max_distance`.
pub fn raycast_scene(
    model: &Model,
    data: &Data,
    ray_origin: Point3<f64>,
    ray_direction: UnitVector3<f64>,
    max_distance: f64,
    bodyexclude: Option<usize>,
    geomgroup: Option<&[bool; 6]>,
) -> Option<SceneRayHit> {
    // ...
}
```

Implementation outline:
1. `let mut closest: Option<SceneRayHit> = None;`
2. `let mut cutoff = max_distance;`
3. Loop `for geom_id in 0..model.ngeom`:
   - **Body filter:** if `bodyexclude == Some(body)` and `model.geom_body[geom_id] == body` → skip
   - **Group filter:** if `geomgroup` is `Some(groups)` and `!groups[model.geom_group[geom_id] as usize]` → skip
   - Build pose from `data.geom_xpos[geom_id]` + `data.geom_xmat[geom_id]`
   - **Primitive shapes** (Sphere, Box, Capsule, Cylinder, Ellipsoid): `geom_to_shape()` → `raycast_shape()`
   - **Plane**: construct `Shape::Plane { normal: Vector3::z(), distance: 0.0 }` (local-space, MuJoCo convention) → `raycast_shape()`
   - **Mesh**: `model.geom_mesh[geom_id]` → `raycast_triangle_mesh_data()`
   - **Hfield**: `model.geom_hfield[geom_id]` → construct `Shape::HeightField { data }` → `raycast_shape()`
   - **SDF**: `model.geom_shape[geom_id]` → `shape_data[id].sdf_grid_arc()` → construct `Shape::Sdf { data }` → `raycast_shape()`
   - Update `closest` and `cutoff` if hit is nearer.
4. Return `closest`

### Pose helper (private)

Extract the repeated pose-construction pattern into a helper:

```rust
fn geom_pose(data: &Data, geom_id: usize) -> Pose {
    let pos = data.geom_xpos[geom_id];
    let mat = data.geom_xmat[geom_id];
    let quat = UnitQuaternion::from_rotation_matrix(
        &Rotation3::from_matrix_unchecked(mat),
    );
    Pose::from_position_rotation(Point3::from(pos), quat)
}
```

### SDF access: add `sdf_grid_arc` to `PhysicsShape`

**File: `sim/L0/core/src/sdf/shape.rs`**

Add to the `PhysicsShape` trait:

```rust
/// The underlying SDF grid as a shared reference.
///
/// Used by `raycast_scene` to construct `Shape::Sdf` for ray queries.
fn sdf_grid_arc(&self) -> Arc<SdfGrid>;
```

Implement in all three concrete types (`ShapeSphere`, `ShapeConvex`,
`ShapeConcave`) — each stores `grid: Arc<SdfGrid>`, so: `self.grid.clone()`.

### File: `sim/L0/core/src/lib.rs`

Add re-exports:
```rust
pub use raycast::{RaycastHit, SceneRayHit, raycast_scene, raycast_shape};
```

### Verification

New tests:
```rust
#[test]
fn test_raycast_scene_hits_closest() { ... }

#[test]
fn test_raycast_scene_bodyexclude() { ... }

#[test]
fn test_raycast_scene_geomgroup_filter() { ... }

#[test]
fn test_raycast_scene_no_hit() { ... }

#[test]
fn test_raycast_scene_plane_geom() { ... }

#[test]
fn test_raycast_scene_mesh_geom() { ... }
```

---

## Phase 5 — Refactor rangefinder sensor to use `raycast_scene`

**Why:** The rangefinder sensor (sensor/position.rs:251-344) duplicates the
scene-level loop. Replace it with a single `raycast_scene` call.

### File: `sim/L0/core/src/sensor/position.rs`

Replace lines 284–337 (the loop over all geoms) with:

```rust
let parent_body = model.site_body[objid];
let result = crate::raycast::raycast_scene(
    model,
    data,
    ray_origin,
    ray_direction,
    max_range,
    Some(parent_body),
    None,
);
let closest_dist = result.map_or(-1.0, |r| r.hit.distance);
sensor_write(&mut data.sensordata, adr, 0, closest_dist);
```

Remove now-unused imports from sensor/position.rs:
- `geom_to_shape`, `GeomType`, `raycast_shape`, `raycast_triangle_mesh_data`
- `Pose`, `UnitQuaternion`, `Rotation3` (if no longer used elsewhere in the file)

**Behavioral note:** `raycast_scene` will now also hit Plane and HeightField
geoms that the current rangefinder code skips (because `geom_to_shape` returns
`None` for them). This is a correctness improvement — a rangefinder should
detect all surfaces. Verify existing rangefinder tests still pass; add a test
with a plane geom if none exists.

### Verification

```
cargo test -p sim-core -- rangefinder
cargo test -p sim-core -- sensor
```

---

## Summary of File Changes

| File | Change | LOC impact |
|------|--------|------------|
| `sim/L0/core/src/raycast.rs` | Delete 10 per-shape fns + `ray_triangle_intersection`; add delegation body; add `SceneRayHit` + `raycast_scene` + `geom_pose`; update tests | -800, +100 |
| `sim/L0/core/src/mesh.rs` | Delete `closest_point_on_triangle`; add import | -84, +1 |
| `sim/L0/core/src/lib.rs` | Update re-exports | ~3 lines |
| `sim/L0/core/src/sensor/position.rs` | Replace rangefinder loop with `raycast_scene` call | -55, +8 |
| `sim/L0/core/src/sdf/shape.rs` | Add `sdf_grid_arc` to trait | +3 |
| `sim/L0/core/src/sdf/shapes/sphere.rs` | Implement `sdf_grid_arc` | +3 |
| `sim/L0/core/src/sdf/shapes/convex.rs` | Implement `sdf_grid_arc` | +3 |
| `sim/L0/core/src/sdf/shapes/concave.rs` | Implement `sdf_grid_arc` | +3 |
| **Net** | | **~-870** |

No changes to cf-geometry — its existing public API is sufficient.

## Acceptance Criteria

1. `raycast_shape` body is ≤20 lines (transform → delegate → transform)
2. Zero per-shape ray functions remain in sim-core
3. Zero duplicated `closest_point_on_triangle`
4. `raycast_scene` exists with body exclusion + geom group filtering + geom ID return
5. Rangefinder sensor calls `raycast_scene`, not its own loop
6. Convex mesh raycasting uses proper face test (not bounding sphere)
7. All existing sim-core tests pass: `cargo test -p sim-core`
8. New tests cover `raycast_scene` (closest hit, body exclude, group filter, plane, mesh)
9. `cargo clippy -p sim-core -- -D warnings` clean
10. `cargo fmt --all -- --check` clean
