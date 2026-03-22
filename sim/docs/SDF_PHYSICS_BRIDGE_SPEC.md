# SDF–Physics Bridge Spec

## Status: Sphere stacking RESOLVED — generalization to non-sphere shapes is next

## Context

The SDF-SDF stacking blocker (two spheres falling through each other)
revealed that SDF collision and the constraint solver are not properly
unified. The SDF grid provides **detection** (are surfaces close?) but
the solver needs **analytical precision** (exact depth, normal, contact
point) to produce stable forces. Six independent bugs were identified
and fixed across sim-core and cf-design. SDF sphere stacking now works
end-to-end: both the conformance test and the cf-design model builder
produce stable 2000+ step simulations.

This spec captures what we learned and defines the architecture for
generalizing beyond spheres.

---

## Part 1 — Root cause analysis (completed)

Six independent issues broke SDF-SDF stacking:

### 1.1 Grid discretization in depth

`sdf.distance(origin)` returns ~-4.36 instead of -5.0 for a 5mm sphere
because the origin isn't a grid point and trilinear interpolation from
the 8 surrounding grid points gives a less-negative value. Using this as
the "effective radius" makes the analytical depth formula
(`r_a + r_b - dist`) underreport overlap by ~1.3mm, producing zero depth
at the touching configuration.

**Fix:** ray-march from the SDF center along the separation axis using
sphere tracing. This converges to the exact surface distance (5.000mm)
in 2-3 iterations.

### 1.2 SDF-plane floor contact discretization

`sdf_plane_contact` uses grid-based surface tracing to find where the
SDF penetrates the floor. The traced surface points have discretization
errors (~0.1mm) that differ from the analytical `collide_sphere_plane`
formula. These small depth errors destabilize the PGS solver's
convergence on the sphere-sphere constraint, causing the upper sphere to
gradually sink through.

**Fix:** replaced `sdf_plane_contact` with the analytical sphere-plane
formula in the SDF collision dispatcher. Radius from ray-march, depth
from `radius - distance_to_plane`.

### 1.3 Contact normal asymmetry

SDF gradient normals have lateral (X/Y) components from grid asymmetry.
For vertically stacked spheres, contact normals like (0.15, 0.15, 0.98)
create a net lateral force that pushes the spheres apart horizontally.
Over hundreds of steps, this lateral impulse accumulates and the spheres
slide off each other.

**Fix:** consolidate all surface-traced contacts into a single contact
with the separation direction as normal. This matches the analytical
sphere-sphere convention exactly.

### 1.4 Visual mesh collision (cf-design)

The cf-design model builder set `contype=1` for visual mesh geoms (group
0), causing them to participate in collision. Mesh-SDF and mesh-mesh
contacts polluted the contact list with redundant constraint rows.

**Fix:** set `contype=0, conaffinity=0` for visual-only mesh geoms.

### 1.5 Missing `diagapprox_bodyweight` (cf-design)

`Model::empty()` defaults `diagapprox_bodyweight = false`, which uses
exact M⁻¹ to compute impedance. MuJoCo defaults to `true` (bodyweight
approximation). For mm-scale bodies with tiny masses (O(10⁻⁴) kg), the
exact solve produces impedance values that destabilize contact stacking.

**Fix:** set `diagapprox_bodyweight = true` in the cf-design model
builder to match MuJoCo defaults.

### 1.6 Wrong constraint solver (cf-design)

`Model::empty()` defaults `solver_type = PGS`. MuJoCo defaults to
Newton. PGS accumulates constraint drift over hundreds of steps; for
vertically stacked spheres this manifests as late instability around
step 1400 where the upper sphere sinks through. Newton's quadratic
convergence with line search keeps the stack rock-solid.

**Diagnosis:** an exhaustive 329-field serialization-based Model diff
(HashMap<String, Vec<f64>> extraction + auto-diff) between the stable
MJCF model and the unstable cf-design model revealed `solver_type` as
the only unexplained physics-affecting difference. All other diffs were
either known (mass/inertia ~1-3% from SDF integration vs analytical) or
non-physics (rgba, geom_group, geom layout ordering).

**Fix:** set `solver_type = SolverType::Newton` in the cf-design model
builder. The `sdf_sphere_stacking` test now passes (gap=9.608, stable
through 2000 steps).

---

## Part 2 — The fundamental problem

The SDF grid and the physics geometry are **two separate representations
of the same shape** that don't share a common interface:

| Need                  | SDF grid provides          | Solver needs              |
|-----------------------|---------------------------|---------------------------|
| Contact detection     | distance(point) < margin  | same (works)              |
| Penetration depth     | grid value (discretized)  | exact overlap distance    |
| Contact normal        | gradient (grid asymmetry) | separation axis direction |
| Contact point         | traced surface (off-axis) | on center axis (no torque)|
| Effective radius      | not stored                | shape-specific parameter  |

The current fix computes the right-column values analytically (ray-march
radius, separation direction normal, midpoint contact position) and
**overrides** the SDF grid values. The SDF grid is used only for
detection. This works but has problems:

1. **Sphere-centric.** The `r_a + r_b - dist` depth formula assumes
   convex shapes with a meaningful "radius along axis." Non-convex
   shapes (sockets, gear teeth) need a different depth computation.

2. **Fragile bridge.** The override logic is scattered across
   `sdf_collide.rs` with ad-hoc ray-marching. Adding new SDF shape
   types requires extending this code for each case.

3. **Grid rotation instability.** Ray-marched radius fluctuates as the
   body rotates (the grid isn't rotationally invariant). Small
   variations in effective radius cause depth oscillations that can
   destabilize stacking.

---

## Part 3 — Remaining work

### 3.1 cf-design model builder integration gap — RESOLVED

Root causes were `diagapprox_bodyweight` (1.5) and `solver_type` (1.6).
Both were `Model::empty()` defaults that diverged from MuJoCo defaults.
Found via exhaustive 329-field serialization diff in the
`diagnose_sdf_sphere_gap` test (model_builder.rs).

The `sdf_sphere_stacking` test now passes: gap=9.608, stable through
2000 steps, matching the conformance test exactly.

### 3.2 Non-sphere SDF shapes

The current depth computation uses `r_a + r_b - center_dist`, which
assumes sphere-like shapes. For general convex shapes:

- **Box-box:** depth along the separating axis is not `r + r - dist`
  but depends on the face/edge/vertex contact mode
- **Capsule-capsule:** effective radius varies along the axis
- **Arbitrary SDF:** no analytical depth formula exists

**Proposed solution:** the `PhysicsShape` trait (see Part 4).

### 3.3 Multi-contact for flat/concave surfaces

The single-contact consolidation works for sphere-sphere (point contact)
but flat surfaces need distributed contacts for stability:

- **Box on floor:** needs 4 corner contacts to prevent tipping
- **Cylinder on floor:** needs a contact ring
- **Concave socket:** needs contacts on both sides of the cavity

The consolidation decision is made by the `PhysicsShape` implementation:
shapes that return `Some` from `effective_radius` get single-contact
consolidation (convex-convex). Shapes that return `None` get multi-contact
surface tracing (flat/concave). See Part 4.5.

### 3.4 Grid rotation invariance

SDF grids are axis-aligned voxel arrays. Rotating the body rotates the
grid, changing which grid points sample the surface. This causes:

- Ray-marched radius to fluctuate slightly per step
- Surface-traced contact points to shift discontinuously
- Normal vectors to have step-to-step noise

Long-term fix: use analytical shape metadata for depth/normal (the
`PhysicsShape` approach), with the SDF grid only for detection. The grid
discretization noise then only affects detection timing (which step the
contact first appears), not force computation.

### 3.5 Bugs fixed during spec review

Two additional bugs found and fixed during the stress-test review of
this spec (the other four are in Part 1):

**3.5a: World-frame ray-march in `operations.rs` (rotation bug)**

`sdf_sdf_contact()` passed the world-frame separation direction directly
to `sdf_radius_along_axis()`, but the SDF grid works in local coordinates.
For unrotated SDFs this happened to work; any rotation would produce wrong
effective radii and wrong contact points.

Fix: transform direction to each SDF's local frame via
`pose.rotation.inverse() * dir` before ray-marching.

**3.5b: Coincident-center panic in `sdf_collide.rs`**

`(other_pos - sdf_pos).normalize()` panics when both SDFs are at the
same position (deep penetration). The `operations.rs` version handled
this with `separation_direction()` returning `None` + fallback, but
`sdf_collide.rs` didn't have the safety check.

Fix: replaced `.normalize()` with `.try_normalize(1e-10).unwrap_or(Vector3::z())`.

---

## Part 4 — `PhysicsShape` trait architecture

**Full spec:** [`PHYSICS_SHAPE_SPEC.md`](PHYSICS_SHAPE_SPEC.md)

Summary of the proposed architecture (see dedicated spec for
implementation plan, file changes, and testing strategy):

### 4.1 Why not `contact_with` on the trait

The original design had `contact_with(&self, other: &dyn PhysicsShape, ...)`
on the trait. This breaks for pair-specific algorithms: a sphere contacting
a box needs the `r_a + half_extent_b` formula, but `contact_with` dispatches
on `self` only — it can't specialize on `other`. Double dispatch in Rust
requires `as_any()` downcasting or an enum, both of which are worse than a
simple free function.

### 4.2 Why `effective_radius` returns `Option`

Non-convex shapes (torus, socket, L-bracket) have no meaningful "radius from
center" — the center may be outside the surface entirely. Returning `Option`
lets the dispatch function detect this and fall back to grid-based multi-
contact instead of producing garbage depth values.

### 4.3 The trait

```rust
pub trait PhysicsShape: Send + Sync {
    /// Distance from the shape center to the surface along a local-frame
    /// direction, found by ray-marching or analytical formula.
    ///
    /// Returns `Some(radius)` for convex shapes where a ray from the center
    /// always crosses the surface exactly once.
    /// Returns `None` for non-convex shapes (torus, socket, hollow body)
    /// where the center-to-surface concept is undefined.
    ///
    /// - Sphere: always `Some(radius)` (constant, rotation-invariant)
    /// - Box: `Some(half_extents projected onto dir)` (direction-dependent)
    /// - Convex SDF: `Some(ray-march result)` (approximate but stable)
    /// - Concave SDF: `None`
    fn effective_radius(&self, local_dir: Vector3<f64>) -> Option<f64>;

    /// The underlying SDF grid for broadphase detection and surface tracing.
    fn sdf_grid(&self) -> &SdfGrid;
}
```

### 4.4 Free-function contact dispatch

Contact computation is a **free function**, not a method. This solves
double dispatch cleanly — the function sees both shapes and picks the
best algorithm:

```rust
pub struct ShapeContact {
    pub point: Point3<f64>,
    pub normal: Vector3<f64>,
    pub depth: f64,
}

/// Compute contact between two physics shapes.
///
/// Algorithm selection:
/// 1. Both shapes have `effective_radius` → analytical depth, single
///    consolidated contact (convex-convex path, proven for stacking)
/// 2. Either shape returns `None` → grid-based surface tracing with
///    multi-contact (concave/flat path, needed for boxes on floor, etc.)
pub fn compute_shape_contact(
    a: &dyn PhysicsShape,
    pose_a: &Pose,
    b: &dyn PhysicsShape,
    pose_b: &Pose,
    margin: f64,
) -> Vec<ShapeContact> {
    let sep = separation_direction(pose_a, pose_b);
    let dir = sep.unwrap_or(Vector3::z());

    // Transform separation direction to each SDF's local frame
    let local_dir_a = pose_a.rotation.inverse() * dir;
    let local_dir_b = pose_b.rotation.inverse() * (-dir);

    // Try analytical path (both convex)
    if let (Some(r_a), Some(r_b)) = (
        a.effective_radius(local_dir_a),
        b.effective_radius(local_dir_b),
    ) {
        let center_dist = (pose_b.position - pose_a.position).norm();
        let depth = (r_a + r_b - center_dist).max(0.0);
        if depth > 0.0 || center_dist < r_a + r_b + margin {
            let contact_point = pose_a.position + dir * (r_a - depth * 0.5);
            return vec![ShapeContact {
                point: Point3::from(contact_point),
                normal: dir,
                depth,
            }];
        }
        return vec![];
    }

    // Fallback: grid-based surface tracing (keeps multi-contact)
    surface_trace_contact(
        a.sdf_grid(), pose_a,
        b.sdf_grid(), pose_b,
        margin,
    )
}
```

### 4.5 Multi-contact consolidation rule

No magic numbers. The rule falls out of the type system:

- If both shapes return `Some` from `effective_radius` → **single contact**
  (convex-convex, consolidation is physically correct)
- If either returns `None` → **multi-contact from surface tracing**
  (flat/concave surfaces need distributed contacts for torque balance)

This replaces the previous "30-degree curvature heuristic" with a decision
the shape author makes once at implementation time.

### 4.6 Implementations

- `SdfSphere { grid: SdfGrid, radius: f64 }` — `effective_radius` always
  returns `Some(radius)`. Grid for detection only.
- `SdfBox { grid: SdfGrid, half_extents: Vector3<f64> }` — `effective_radius`
  returns `Some(half_extents projected onto dir)`. Grid for detection.
- `SdfConvex { grid: SdfGrid }` — `effective_radius` returns
  `Some(ray_march_result)`. General convex fallback.
- `SdfConcave { grid: SdfGrid }` — `effective_radius` returns `None`.
  Always uses multi-contact surface tracing.

### 4.7 How this helps

1. **Single source of truth.** The `PhysicsShape` carries both the SDF
   grid AND the analytical parameters. No more scattered overrides.

2. **Pair dispatch without double dispatch.** The free function sees both
   shapes. Specialized pair algorithms (sphere-sphere, sphere-box) can be
   added as match arms without changing the trait.

3. **Testable bridge.** Each implementation can be tested independently:
   "does `effective_radius` match the analytical formula?"

4. **Rotation-invariant depth.** Analytical implementations (`SdfSphere`)
   use constants. Ray-march implementations use local-frame directions
   (transformation handled by `compute_shape_contact`, not the shape).

5. **Correct concave handling.** Non-convex shapes opt out of consolidation
   by returning `None`, getting multi-contact automatically.

### 4.8 Migration path

1. Define the trait + `compute_shape_contact` in `sim-core/src/sdf/mod.rs`
2. Implement `SdfSphere` first (proven by conformance test)
3. Wire `compute_shape_contact` into `sdf_collide.rs`, replacing the raw
   `SdfGrid` + ad-hoc override logic (eliminates the duplicate ray-march
   in `operations.rs` and `sdf_collide.rs`)
4. Implement `SdfBox`, `SdfConvex`, `SdfConcave`
5. cf-design model builder stores `Arc<dyn PhysicsShape>` instead of
   bare `Arc<SdfGrid>` — the `Solid` knows its own shape kind and picks
   the right implementation at build time

---

## Part 5 — How the SDF grid is created vs used (reference)

### Creation (cf-design model builder)

```
Solid (exact implicit function)
  → sdf_grid_at(cell_size)
    → SdfGrid::from_fn(nx, ny, nz, cell_size, origin, |p| solid.evaluate(p))
      → grid values at each voxel center
  → stored in Model.sdf_data as Arc<SdfGrid>
```

The grid is in the **solid's local coordinate system** (centered at the
shape's origin). The `origin` field of SdfGrid is the world position of
grid corner (0,0,0), typically at `-(shape_extent + padding)`.

### Usage (collision pipeline)

```
collide_with_sdf(model, geom1, geom2, pos1, mat1, pos2, mat2, margin)
  → sdf_pose = Pose(geom_world_position, geom_rotation)
  → world_point → sdf_pose.inverse_transform → local_point
  → sdf.distance(local_point) → trilinear interpolation
```

The `sdf_pose.inverse_transform` puts the world point into the geom's
local frame, which aligns with the SDF grid's coordinate system (both
centered at the shape origin, both using the same orientation).

### The gap

The SDF grid stores **approximate** distance values. The physics engine
needs **exact** distance values. The approximation error (~0.1mm at 1mm
cell size) is enough to break the constraint solver's force balance for
mm-scale objects under gravity.

---

## Appendix — Key files

| File | Role |
|------|------|
| `sim/L0/core/src/sdf/operations.rs` | Surface tracing + consolidation |
| `sim/L0/core/src/collision/sdf_collide.rs` | SDF collision dispatch + depth override |
| `sim/L0/core/src/sdf/primitives.rs` | SDF-primitive contacts (plane, sphere, etc.) |
| `sim/L0/core/src/constraint/contact_assembly.rs` | Contact → constraint rows |
| `sim/L0/core/src/constraint/impedance.rs` | KBIP, aref, regularization |
| `design/cf-design/src/mechanism/model_builder.rs` | cf-design Model construction |
| `sim/L0/tests/integration/collision_primitives.rs` | SDF stacking conformance test |
| `sim/docs/SDF_SDF_CONTACT_SPEC.md` | Previous spec (surface-tracing algorithm) |
