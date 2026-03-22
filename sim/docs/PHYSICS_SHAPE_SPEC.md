# PhysicsShape Trait Spec

## Status: IN PROGRESS — Session 1 complete, Sessions 2–3 pending

## Context

The SDF stacking blocker is resolved (6 bugs fixed, see
`SDF_PHYSICS_BRIDGE_SPEC.md` Part 1). SDF sphere stacking works
end-to-end. But the current collision code is **sphere-centric**: it
treats every SDF as a sphere with a direction-dependent radius and uses
the `r_a + r_b - dist` depth formula. This blocks three capabilities:

- **Non-sphere SDF shapes** (box, capsule, arbitrary convex/concave)
- **Multi-contact for flat/concave surfaces** (box on floor needs 4
  contacts, concave socket needs distributed contacts)
- **Rotation invariance** (ray-marched radius fluctuates as the grid
  rotates; analytical shapes don't have this problem)

This spec defines a `PhysicsShape` trait that gives each shape its own
contact algorithm, replacing the hardcoded sphere assumptions.

---

## Part 1 — Current architecture

### 1.1 How SDF collision works today

The narrow-phase dispatcher (`narrow.rs:83`) routes any geom pair where
either type is `Sdf` to `collide_with_sdf()`. That function:

1. Looks up the `SdfGrid` via `model.sdf_data[model.geom_sdf[geom_id]]`
2. Dispatches on the *other* geom's type:
   - **SDF-Plane** (`sdf_collide.rs:107-135`): ray-marches the SDF to
     find an effective radius, then uses `radius - distance_to_plane`
     for analytical depth
   - **SDF-SDF** (`sdf_collide.rs:136-165`): calls `sdf_sdf_contact()`
     for surface tracing, then **overrides** all penetration values with
     analytical depth from `r_a + r_b - center_dist`
   - **SDF-Primitive** (`sdf_collide.rs:168-207`): calls per-type
     functions (`sdf_sphere_contact`, `sdf_box_contact`, etc.) that
     sample the primitive's test points against the SDF grid

### 1.2 The sphere-centric functions

Two nearly-identical functions compute "effective radius":

- `sdf_ray_radius()` in `sdf_collide.rs:16-30`
- `sdf_radius_along_axis()` in `operations.rs:92-110`

Both sphere-trace from the SDF origin along a direction to find where
the SDF value crosses zero. This gives the distance from center to
surface — the "effective radius" along that direction.

The analytical depth override (`sdf_collide.rs:148-162`,
`operations.rs:58-85`) then uses:

```
depth = r_a + r_b - center_distance
contact_point = center_a + dir * (r_a - depth/2)
normal = separation_direction
```

And consolidates all traced contacts into **exactly 1 contact**.

### 1.3 Model storage

```rust
// model.rs
pub sdf_data: Vec<Arc<SdfGrid>>,       // shape_id → raw SDF grid
pub geom_sdf: Vec<Option<usize>>,      // geom_id → shape_id
pub nsdf: usize,                       // count
```

The SDF grid carries no metadata about what kind of shape it represents.
A sphere grid and a socket grid are both just voxel arrays.

### 1.4 SDF-Primitive contacts (no change needed)

The per-primitive functions in `primitives.rs` sample test points from
the analytical primitive (box corners, capsule axis, cylinder rings) and
query them against the SDF grid. These work correctly for any SDF shape
because the SDF grid provides accurate distance values at arbitrary
points. **These functions do not need to change.**

---

## Part 2 — What's wrong

Three assumptions are hardcoded into the SDF-SDF and SDF-Plane paths:

### 2.1 Sphere-like depth formula

`r_a + r_b - center_dist` assumes both shapes have a single meaningful
"radius from center." This breaks for:

- **Boxes**: depth depends on face/edge/vertex contact mode, not radii
- **Concave shapes**: the center may be outside the surface entirely
- **Flat surfaces**: effective radius along the contact axis is the
  full half-extent, giving wrong depth for edge/corner contacts

### 2.2 Single-contact consolidation

All traced contacts are consolidated into 1 contact on the center axis.
This is correct for sphere-sphere (point contact) but wrong for:

- **Box on floor**: needs 4 corner contacts to prevent tipping
- **Cylinder on floor**: needs a contact ring
- **Concave socket**: needs contacts on both sides of the cavity

### 2.3 Ray-march radius instability

`sdf_ray_radius()` sphere-traces through a voxel grid. As the body
rotates, different grid cells are sampled, causing the effective radius
to fluctuate by ~0.01mm per step. For sphere shapes this is unnecessary
since the analytical radius is constant (rotation-invariant).

---

## Part 3 — Design

### 3.1 The trait

```rust
/// Physics metadata for an SDF-backed collision shape.
///
/// Each SDF geom in the Model carries a PhysicsShape implementation
/// that provides shape-specific contact computation. The SDF grid
/// handles broadphase detection; the PhysicsShape handles the
/// analytical depth/normal/contact-point computation that the solver
/// needs.
pub trait PhysicsShape: Send + Sync + std::fmt::Debug {
    /// Distance from the shape center to the surface along a
    /// local-frame direction.
    ///
    /// Returns `Some(radius)` for convex shapes where the concept of
    /// "center to surface" is well-defined. The collision pipeline
    /// uses this for analytical depth: `r_a + r_b - center_dist`.
    ///
    /// Returns `None` for non-convex shapes (socket, torus, hollow
    /// body) where the center may be outside the surface. The
    /// pipeline falls back to grid-based multi-contact surface
    /// tracing.
    fn effective_radius(&self, local_dir: &Vector3<f64>) -> Option<f64>;

    /// The underlying SDF grid for broadphase detection and surface
    /// tracing fallback.
    fn sdf_grid(&self) -> &SdfGrid;
}
```

File: `sim/L0/core/src/sdf/shape.rs` (new)

### 3.2 Why `effective_radius` returns `Option`

This is the single design decision that controls the entire contact
strategy:

| `effective_radius` | Contact strategy | Use case |
|--------------------|-----------------|----------|
| Both `Some` | Analytical depth, single contact | Convex-convex (sphere, box, ellipsoid) |
| Either `None` | Grid surface tracing, multi-contact | Flat, concave, or unknown shapes |

No magic numbers, heuristics, or configuration flags. The shape author
makes this decision once when implementing the trait.

### 3.3 Free-function contact dispatch

Contact computation is a **free function**, not a method on the trait.
This solves double dispatch: the function sees both shapes and picks
the algorithm.

```rust
pub fn compute_shape_contact(
    a: &dyn PhysicsShape,
    pose_a: &Pose,
    b: &dyn PhysicsShape,
    pose_b: &Pose,
    margin: f64,
) -> Vec<SdfContact> {
    let dir = separation_direction(pose_a, pose_b)
        .unwrap_or(Vector3::z());

    // Transform to each shape's local frame
    let local_dir_a = pose_a.rotation.inverse() * dir;
    let local_dir_b = pose_b.rotation.inverse() * (-dir);

    // Analytical path: both convex
    if let (Some(r_a), Some(r_b)) = (
        a.effective_radius(&local_dir_a),
        b.effective_radius(&local_dir_b),
    ) {
        let center_dist = (pose_b.position - pose_a.position).norm();
        let depth = (r_a + r_b - center_dist).max(0.0);
        if depth > 0.0 || center_dist < r_a + r_b + margin {
            return vec![SdfContact {
                point: pose_a.position + dir * (r_a - depth * 0.5),
                normal: dir,
                penetration: depth,
            }];
        }
        return vec![];
    }

    // Fallback: grid-based multi-contact surface tracing
    sdf_sdf_contact_raw(
        a.sdf_grid(), pose_a,
        b.sdf_grid(), pose_b,
        margin,
    )
}
```

File: `sim/L0/core/src/sdf/shape.rs` or `operations.rs`

### 3.4 `compute_shape_plane_contact`

The SDF-Plane path also needs the trait, since it currently uses
`sdf_ray_radius()`:

```rust
pub fn compute_shape_plane_contact(
    shape: &dyn PhysicsShape,
    shape_pose: &Pose,
    plane_pos: &Vector3<f64>,
    plane_normal: &Vector3<f64>,
    margin: f64,
) -> Vec<SdfContact> {
    let local_dir = shape_pose.rotation.inverse() * (-*plane_normal);

    if let Some(radius) = shape.effective_radius(&local_dir) {
        // Analytical: depth = radius - distance_to_plane
        let dist_to_plane = (shape_pose.position - plane_pos).dot(plane_normal);
        let depth = radius - dist_to_plane;
        if depth > -margin {
            // Contact at the midpoint of the overlap region
            // (matches sdf_collide.rs convention)
            let contact_point =
                shape_pose.position - plane_normal * (radius - depth * 0.5);
            return vec![SdfContact {
                point: contact_point,
                normal: *plane_normal,
                penetration: depth.max(0.0),
            }];
        }
        return vec![];
    }

    // Fallback: grid-based plane contact (multi-contact)
    sdf_plane_contact(shape.sdf_grid(), shape_pose, plane_pos, plane_normal, margin)
}
```

### 3.5 Caller responsibilities

`compute_shape_contact` and `compute_shape_plane_contact` return
`Vec<SdfContact>` (point, normal, penetration). The caller
(`collide_with_sdf` in `sdf_collide.rs`) is responsible for:

1. **Margin floor**: floor margin at `cell_size * 0.5` before calling,
   to tolerate SDF grid discretization (currently `sdf_collide.rs:143`)
2. **Normal convention**: negate normal based on geom swap/plane
   direction (currently `sdf_collide.rs:74-91`)
3. **Pipeline conversion**: convert `SdfContact` → pipeline `Contact`
   via `make_contact_from_geoms` (currently `sdf_collide.rs:93-105`)

These responsibilities stay in `sdf_collide.rs`. The dispatch functions
are pure geometry — they don't know about geom IDs, solver params, or
pipeline conventions.

### 3.6 Implementations

All implementations live in `sim/L0/core/src/sdf/shapes/`.

**`ShapeSphere`** — the proven case (conformance test + cf-design test)

```rust
pub struct ShapeSphere {
    grid: Arc<SdfGrid>,
    radius: f64,
}

impl PhysicsShape for ShapeSphere {
    fn effective_radius(&self, _local_dir: &Vector3<f64>) -> Option<f64> {
        Some(self.radius)  // Constant, rotation-invariant
    }
    fn sdf_grid(&self) -> &SdfGrid { &self.grid }
}
```

No ray-marching. No grid sampling. Just the analytical radius. This
eliminates the rotation instability problem (2.3) for spheres entirely.

**`ShapeConvex`** — general convex fallback

```rust
pub struct ShapeConvex {
    grid: Arc<SdfGrid>,
}

impl PhysicsShape for ShapeConvex {
    fn effective_radius(&self, local_dir: &Vector3<f64>) -> Option<f64> {
        Some(sdf_radius_along_axis(&self.grid, *local_dir))
    }
    fn sdf_grid(&self) -> &SdfGrid { &self.grid }
}
```

Uses the existing ray-march. Same behavior as today, but encapsulated
in a shape. Used for CSG combinations that are known-convex but have
no analytical radius formula.

**`ShapeConcave`** — multi-contact path

```rust
pub struct ShapeConcave {
    grid: Arc<SdfGrid>,
}

impl PhysicsShape for ShapeConcave {
    fn effective_radius(&self, _local_dir: &Vector3<f64>) -> Option<f64> {
        None  // Forces multi-contact surface tracing
    }
    fn sdf_grid(&self) -> &SdfGrid { &self.grid }
}
```

Returns `None`, so `compute_shape_contact` always takes the
surface-tracing path. Used for sockets, gear teeth, hollow bodies.

**Future implementations (not in scope for this spec):**

- `ShapeBox { grid, half_extents }` — `effective_radius` projects
  half-extents onto direction. Rotation-invariant depth.
- `ShapeCapsule { grid, radius, half_length }` — radius varies along
  axis.
- `ShapeCylinder { grid, radius, half_height }` — direction-dependent.

### 3.7 Model storage

Replace `sdf_data` with `shape_data`:

```rust
// Before
pub sdf_data: Vec<Arc<SdfGrid>>,

// After
pub shape_data: Vec<Arc<dyn PhysicsShape>>,
```

The fields `geom_sdf` and `nsdf` rename to `geom_shape` and `nshape`.
The `SdfGrid` is still accessible via
`model.shape_data[id].sdf_grid()`.

### 3.8 cf-design integration

The `Solid` type knows its shape kind. The model builder uses this to
pick the right `PhysicsShape` implementation:

```rust
// In generate(), when creating SDF geoms:
let shape: Arc<dyn PhysicsShape> = match solid.shape_hint() {
    ShapeHint::Sphere(radius) => Arc::new(ShapeSphere::new(grid, radius)),
    ShapeHint::Convex          => Arc::new(ShapeConvex::new(grid)),
    ShapeHint::Concave         => Arc::new(ShapeConcave::new(grid)),
};
model.shape_data.push(shape);
```

The `shape_hint()` method on `Solid` returns the appropriate hint based
on the expression tree. Sphere primitives return `Sphere(radius)`. CSG
operations could return `Convex` or `Concave` based on the operator
(smooth_union of convex shapes is convex; subtraction may be concave).

A simple initial approach: all non-sphere shapes start as `ShapeConvex`
(same behavior as today). Concave support is added later when
multi-contact surface tracing is tested.

---

## Part 4 — Changes to existing code

### 4.1 New files

| File | Contents |
|------|----------|
| `sim/L0/core/src/sdf/shape.rs` | `PhysicsShape` trait, `compute_shape_contact`, `compute_shape_plane_contact` |
| `sim/L0/core/src/sdf/shapes/mod.rs` | Re-exports |
| `sim/L0/core/src/sdf/shapes/sphere.rs` | `ShapeSphere` |
| `sim/L0/core/src/sdf/shapes/convex.rs` | `ShapeConvex` |
| `sim/L0/core/src/sdf/shapes/concave.rs` | `ShapeConcave` |

### 4.2 Modified files

| File | Change |
|------|--------|
| `sim/L0/core/src/sdf/mod.rs` | Add `mod shape; mod shapes;`, re-export trait + implementations |
| `sim/L0/core/src/types/model.rs` | `sdf_data: Vec<Arc<SdfGrid>>` → `shape_data: Vec<Arc<dyn PhysicsShape>>` |
| `sim/L0/core/src/types/model_init.rs` | Update `empty()` and `Model::new()` defaults |
| `sim/L0/core/src/types/model_factories.rs` | Update `push_geom` default for geom_sdf/geom_shape |
| `sim/L0/core/src/collision/sdf_collide.rs` | Replace `sdf_ray_radius()` + inline overrides with `compute_shape_contact()` / `compute_shape_plane_contact()` calls |
| `sim/L0/core/src/sdf/operations.rs` | Extract raw surface tracing into `sdf_sdf_contact_raw()` (no consolidation). Remove `sdf_radius_along_axis()` — moved to `ShapeConvex` |
| `sim/L0/core/src/collision/flex_narrow.rs` | Update `sdf_data` → `shape_data` access (`.sdf_grid()` call) |
| `sim/L0/mjcf/src/builder/build.rs` | Update `sdf_data` → `shape_data`, wrap grids in `ShapeConvex` (MJCF has no shape metadata) |
| `sim/L0/mjcf/src/builder/init.rs` | Update field init |
| `design/cf-design/src/mechanism/model_builder.rs` | Wrap SDF grids in `ShapeSphere` / `ShapeConvex` based on `Solid` type |
| `sim/L0/tests/integration/collision_primitives.rs` | Update conformance test to use `ShapeSphere` |
| `sim/L1/bevy/src/model_data.rs` | Update `geom_sdf` field reference |

### 4.3 Deleted code

| Location | What | Why |
|----------|------|-----|
| `sdf_collide.rs:16-30` | `sdf_ray_radius()` | Replaced by `PhysicsShape::effective_radius()` |
| `operations.rs:92-110` | `sdf_radius_along_axis()` | Same — moved into `ShapeConvex::effective_radius()` |
| `sdf_collide.rs:107-135` | SDF-Plane analytical override block | Replaced by `compute_shape_plane_contact()` |
| `sdf_collide.rs:148-162` | SDF-SDF analytical override block | Replaced by `compute_shape_contact()` |
| `operations.rs:48-85` | Consolidation + analytical depth in `sdf_sdf_contact()` | Moved into `compute_shape_contact()` |

---

## Part 5 — Implementation plan

Three sessions. Each phase is independently testable and committable.

- **Session 1** (Phase A): **DONE** — Trait definition, `ShapeSphere` +
  `ShapeConvex` implementations, full Model cutover (`sdf_data` →
  `shape_data`), all builders updated. 2,977 tests pass, 0 failures.
  Also added `Solid::sphere_radius()` for cf-design sphere detection.
- **Session 2** (Phases B+C+D): Wire dispatch functions into collision
  pipeline, delete dead code. Tightly coupled — changing the SDF-SDF
  path without the SDF-Plane path would leave inconsistent code paths.
- **Session 3** (Phases E+F): `ShapeConcave` + multi-contact, cf-design
  `shape_hint()` integration. New capability — requires testing with
  non-sphere geometry.

### Phase A: Trait + `ShapeSphere` + `ShapeConvex` + Model cutover (Session 1) — DONE

The big-bang phase. All model changes happen here so there is no
dual-field period and no fallback guards.

1. ~~Create `sdf/shape.rs` with `PhysicsShape` trait~~
2. ~~Create `sdf/shapes/sphere.rs` with `ShapeSphere`~~
3. ~~Create `sdf/shapes/convex.rs` with `ShapeConvex` (wraps
   `sdf_radius_along_axis` — identical to current ray-march behavior)~~
4. ~~**Replace** `sdf_data: Vec<Arc<SdfGrid>>` with
   `shape_data: Vec<Arc<dyn PhysicsShape>>` in Model~~
5. ~~Update `Model::empty()`, `model_factories`, MJCF builder init~~
6. ~~MJCF builder: wrap all SDF grids in `ShapeConvex::new(grid)`~~
   (MJCF always has empty `shape_data` — no grids to wrap)
7. ~~cf-design builder: wrap sphere SDFs in
   `ShapeSphere::new(grid, radius)`, everything else in `ShapeConvex`~~
   (Added `Solid::sphere_radius()` for detection)
8. ~~Update all `model.sdf_data[id]` callers to
   `model.shape_data[id].sdf_grid()` (~6 call sites in production)~~
9. ~~Conformance test: use `ShapeSphere` instead of raw `SdfGrid`~~
10. ~~Tests: `ShapeSphere::effective_radius()` is constant for any
    direction. `ShapeConvex::effective_radius()` matches old
    `sdf_ray_radius()`. All existing tests pass unchanged (behavior
    is identical).~~

**No collision logic changes yet.** `sdf_collide.rs` still calls
`sdf_ray_radius()` / `sdf_radius_along_axis()` — it just gets the
grid via `.sdf_grid()` instead of directly.

### Phase B: Wire `compute_shape_contact` into SDF-SDF path (Session 2)

1. Create `compute_shape_contact()` in `sdf/shape.rs`
2. Extract raw surface tracing from `sdf_sdf_contact()` into
   `sdf_sdf_contact_raw()` (multi-contact, no consolidation)
3. In `sdf_collide.rs`, replace the SDF-SDF block (lines 136-165)
   with a call to `compute_shape_contact()` using
   `model.shape_data[sdf_id]`
4. This eliminates the dual ray-march: currently both `operations.rs`
   and `sdf_collide.rs` ray-march the same SDFs — the old
   `sdf_sdf_contact()` consolidates, then `sdf_collide.rs` overrides
   depth again. The new path does it once.
5. Tests: conformance test + cf-design stacking test still pass.
   Numerical equivalence test: old path vs new path on identical inputs.

### Phase C: Wire `compute_shape_plane_contact` into SDF-Plane path (Session 2)

1. Create `compute_shape_plane_contact()` in `sdf/shape.rs`
2. In `sdf_collide.rs`, replace the SDF-Plane block (lines 107-135)
   with a call to `compute_shape_plane_contact()`
3. Tests: stacking test still passes (sphere on floor + sphere on
   sphere). Numerical equivalence vs old path.

### Phase D: Delete dead code (Session 2)

1. Delete `sdf_ray_radius()` from `sdf_collide.rs`
2. Delete `sdf_radius_along_axis()` from `operations.rs`
3. Delete consolidation block from old `sdf_sdf_contact()` (now
   `sdf_sdf_contact_raw`)
4. Tests: all sim + design tests pass. No references to deleted
   functions remain.

### Phase E: `ShapeConcave` + multi-contact (Session 3)

1. Create `sdf/shapes/concave.rs` with `ShapeConcave` (returns `None`
   from `effective_radius` — forces multi-contact surface tracing)
2. cf-design: CSG subtractions and known-concave shapes use
   `ShapeConcave`
3. Tests: concave socket contacts produce distributed multi-contact.
   Box on floor (via ShapeConcave) produces corner contacts.

### Phase F: cf-design `shape_hint()` integration (Session 3)

1. Add `shape_hint()` to `Solid` that inspects the expression tree
2. Sphere → `ShapeHint::Sphere(radius)`
3. Box, cylinder, smooth_union of convex → `ShapeHint::Convex`
4. Smooth subtraction → `ShapeHint::Concave`
5. Model builder uses hint to select implementation
6. Tests: various `Solid` types produce correct shape hints

---

## Part 6 — Testing strategy

### Regression tests (must pass at every phase)

- `sim-conformance-tests`: SDF sphere stacking (MJCF path)
- `cf-design::sdf_sphere_stacking`: sphere stacking (cf-design path)
- `cf-design::diagnose_sdf_sphere_gap`: exhaustive 329-field diff
- All existing collision tests in `sim-core`

### New tests per phase

| Phase | Test |
|-------|------|
| A | `ShapeSphere::effective_radius` is constant for any direction |
| A | `ShapeConvex::effective_radius` matches old `sdf_ray_radius()` |
| A | `ShapeSphere::sdf_grid()` returns the underlying grid |
| A | All callers compile and pass with `shape_data` instead of `sdf_data` |
| B | SDF-SDF contact via `compute_shape_contact` matches existing results for sphere pairs |
| B | Numerical equivalence: old path vs new path on identical inputs |
| C | SDF-Plane contact via `compute_shape_plane_contact` matches existing results |
| D | No references to `sdf_ray_radius` or `sdf_radius_along_axis` remain |
| E | `ShapeConcave` always returns multi-contact (no consolidation) |
| F | `Solid::sphere().shape_hint()` returns `Sphere(radius)` |

### Verification approach

For phases B and C, the gold standard is numerical equivalence: run both
the old code path and the new trait path on the same inputs and assert
identical contact output (point, normal, depth within 1e-12).

---

## Appendix — Key files

| File | Role |
|------|------|
| `sim/L0/core/src/sdf/mod.rs` | SDF module root |
| `sim/L0/core/src/sdf/operations.rs` | Surface tracing, consolidation, `sdf_radius_along_axis` |
| `sim/L0/core/src/collision/sdf_collide.rs` | SDF collision dispatch, `sdf_ray_radius`, analytical overrides |
| `sim/L0/core/src/collision/narrow.rs` | Narrow-phase dispatcher (routes to `collide_with_sdf`) |
| `sim/L0/core/src/sdf/primitives.rs` | SDF-vs-primitive contacts (unchanged by this spec) |
| `sim/L0/core/src/types/model.rs` | Model struct (`sdf_data`, `geom_sdf`) |
| `sim/L0/core/src/collision/flex_narrow.rs` | Flex-SDF collision (uses `sdf_data`) |
| `sim/L0/mjcf/src/builder/build.rs` | MJCF model builder (creates `sdf_data`) |
| `design/cf-design/src/mechanism/model_builder.rs` | cf-design model builder |
| `sim/L0/tests/integration/collision_primitives.rs` | SDF stacking conformance test |
| `sim/docs/SDF_PHYSICS_BRIDGE_SPEC.md` | Parent spec (Parts 1-3 completed, Part 4 → this spec) |
