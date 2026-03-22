# SDF–Physics Bridge Spec

## Status: DRAFT — findings from the stacking investigation, next steps

## Context

The SDF-SDF stacking blocker (two spheres falling through each other)
revealed that SDF collision and the constraint solver are not properly
unified. The SDF grid provides **detection** (are surfaces close?) but
the solver needs **analytical precision** (exact depth, normal, contact
point) to produce stable forces. Three independent bugs were identified
and fixed, but the fix is sphere-centric and a cf-design model builder
integration gap remains.

This spec captures what we learned and defines the architecture for a
general solution.

---

## Part 1 — Root cause analysis (completed)

Three independent issues broke SDF-SDF stacking:

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

### 3.1 cf-design model builder integration gap

**Status:** blocking example 07 (SDF sphere pair stacking)

The conformance test (MJCF model with geoms swapped to SDF) stacks
perfectly at gap=9.608 (matching analytical sphere-sphere). The cf-design
model builder produces a model with identical parameters (same mass,
timestep, gravity, radius) where the sphere falls through.

**Diagnosis approach:**
- Instrument `finalize_constraint_row` to log `efc_aref`, `efc_b`,
  `efc_R`, `efc_D`, `pos`, `margin`, `vel` for both models at the same
  sim time
- Compare the constraint system state step-by-step
- The divergence must be in a Model struct field that affects the
  Jacobian, mass matrix, or regularization
- Suspect: `body_ipos` (inertia frame offset), `body_iquat`, or
  `geom_pos` differences between MJCF and cf-design model construction

**Key files:**
- `sim/L0/core/src/constraint/assembly.rs` — finalize_constraint_row
- `sim/L0/core/src/constraint/contact_assembly.rs` — contact row setup
- `design/cf-design/src/mechanism/model_builder.rs` — model construction
- `sim/L0/tests/integration/collision_primitives.rs` — passing test
- `design/cf-design/src/mechanism/model_builder.rs` — failing test

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

The consolidation step should be conditional: consolidate for convex-
convex pairs (where a single contact is correct), keep multi-contact
for flat or concave surfaces.

**Detection heuristic:** if the contact patch spans more than ~30 degrees
of the surface curvature, keep multi-contact. Otherwise consolidate.

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

---

## Part 4 — Proposed architecture: `PhysicsShape` trait

Unify SDF and analytical geometry behind a single interface:

```rust
pub trait PhysicsShape {
    /// Distance from the shape center to the surface along a direction.
    /// For a sphere: always returns the radius.
    /// For a box: returns the half-extent projected onto the direction.
    fn effective_radius(&self, local_dir: Vector3<f64>) -> f64;

    /// Contact between two physics shapes at given poses.
    /// Returns: (depth, normal, contact_point) or None.
    /// Depth follows the analytical convention:
    ///   positive = overlap, zero = touching, negative = separated.
    fn contact_with(
        &self,
        self_pose: &Pose,
        other: &dyn PhysicsShape,
        other_pose: &Pose,
        margin: f64,
    ) -> Option<ShapeContact>;

    /// SDF distance query (for detection/broadphase).
    /// Returns None if point is outside the grid bounds.
    fn sdf_distance(&self, local_point: Point3<f64>) -> Option<f64>;
}
```

**Implementations:**

- `SdfSphere { grid: SdfGrid, radius: f64 }` — uses radius for depth,
  grid for detection
- `SdfBox { grid: SdfGrid, half_extents: Vector3<f64> }` — uses
  half-extents for depth, grid for detection
- `SdfArbitrary { grid: SdfGrid }` — uses ray-march for depth (fallback)

**How this helps:**

1. **Single source of truth.** The `PhysicsShape` carries both the SDF
   grid AND the analytical parameters. No more scattered overrides.

2. **Generalizes naturally.** New shapes implement the trait. The
   collision pipeline doesn't need per-type dispatch.

3. **Testable bridge.** Each implementation can be tested independently:
   "does `effective_radius` match the analytical formula?"

4. **Rotation-invariant depth.** The `effective_radius` method uses
   analytical parameters (constant), not grid queries (rotation-dependent).

**Migration path:**

1. Define the trait in `sim-core/src/sdf/mod.rs`
2. Implement `SdfSphere` first (proven by conformance test)
3. Migrate `sdf_collide.rs` to use `PhysicsShape` instead of raw
   `SdfGrid` + ad-hoc overrides
4. Implement `SdfBox`, `SdfCapsule`, `SdfArbitrary`
5. cf-design model builder stores `PhysicsShape` instead of bare `SdfGrid`

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
