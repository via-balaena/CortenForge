# Spec E — Deformable Complex-Geom Narrowphase: Spec

**Status:** Draft
**Phase:** Roadmap Phase 9 — Collision Completeness
**Effort:** S
**MuJoCo ref:** `engine_collision_primitive.c` — flex vertex collision dispatch
**MuJoCo version:** 3.x (main branch)
**Test baseline:** 2,100+ sim domain tests
**Prerequisites:**
- Spec A (§65 convex hull) — complete (`ba6c261`)
- Phase 9 sessions 1–23 all done

**Independence:** This spec has a **soft dependency** on Spec A (§65) per the
umbrella dependency graph. Spec A is complete — the hull path is the primary
path for flex-vs-mesh. No shared mutable files: Spec E modifies
`flex_collide.rs` (single-owner in Phase 9). Read-only consumers:
`mesh.rs`, `heightfield.rs`, `sdf.rs`, `collision_shape.rs`, `gjk_epa.rs`.

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the feature described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

---

## Scope Adjustment

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| Flex-vs-mesh: vertex sphere against triangle mesh (or convex hull) | MuJoCo treats flex vertices as spheres and uses the same collision path as rigid sphere-vs-mesh. For mesh collision, MuJoCo uses the mesh's convex hull (always computed at build time). Behavioral outcome: identical contact geometry (point, normal, depth) as rigid sphere-vs-mesh. | **In scope** — vertex sphere vs mesh, using convex hull GJK path (primary) and per-triangle BVH fallback |
| Flex-vs-hfield: vertex sphere against heightfield grid | MuJoCo uses standard sphere-vs-hfield collision for flex vertices. CortenForge has `heightfield_sphere_contact()` (point-sampling, corner-origin). For tiny flex vertex spheres, this approach gives equivalent results to the prism-based path. | **In scope** — vertex sphere vs hfield, reusing `heightfield_sphere_contact()` |
| Flex-vs-SDF: vertex sphere against SDF | MuJoCo uses standard sphere-vs-SDF collision for flex vertices. CortenForge has `sdf_sphere_contact()` which takes sphere center + radius. Direct reuse. | **In scope** — vertex sphere vs SDF, reusing `sdf_sphere_contact()` |
| Soft dependency on Spec A (convex hull) | Spec A complete (Session 7 review passed, commit `ba6c261`). `TriangleMeshData::convex_hull()` returns `Option<&ConvexHull>`. | **Resolved** — hull path is primary; per-triangle BVH is fallback |

**Final scope:**
1. Add flex-vs-mesh collision: vertex sphere against mesh (convex hull
   GJK primary, per-triangle BVH fallback for meshes without hull)
2. Add flex-vs-hfield collision: vertex sphere against heightfield
3. Add flex-vs-SDF collision: vertex sphere against SDF
4. All three integrate into `narrowphase_sphere_geom()` dispatch in
   `flex_collide.rs` via early-return match arms (Option B — see AD-1)

---

## Problem Statement

**Conformance gap:** MuJoCo computes flex-vertex-sphere collision against
all geom types including mesh, hfield, and SDF. CortenForge's
`narrowphase_sphere_geom()` in `flex_collide.rs:152-153` has a catch-all
`_ => return None` that silently skips `GeomType::Mesh`, `GeomType::Hfield`,
and `GeomType::Sdf`. This means flex bodies pass through mesh terrain,
heightfield terrain, and SDF geometry without generating any contacts.

MuJoCo's flex collision treats each vertex as a sphere with radius
`flex_radius` and computes collision using the same algorithms as rigid
sphere-vs-geom. CortenForge already has all three underlying collision
functions (`mesh_sphere_contact`, `heightfield_sphere_contact`,
`sdf_sphere_contact`). The gap is purely in dispatch — the match arms are
missing.

---

## MuJoCo Reference

### MuJoCo flex collision dispatch

**Source:** `engine_collision_primitive.c` — flex vertex collision loop.

MuJoCo treats each flex vertex as a sphere with radius `flex_radius`. The
flex collision loop iterates all vertices × all geoms and computes
sphere-vs-geom distances. For complex geom types (mesh, hfield, SDF),
MuJoCo uses the same collision algorithms as rigid sphere-vs-complex
collision. The **behavioral outcome is identical**: a flex vertex sphere at
position P with radius R produces the same contact geometry (point, normal,
depth) as a rigid sphere geom of radius R at position P against the same
geom.

The only difference between flex-vs-geom and rigid-sphere-vs-geom is contact
parameter combination: flex uses flex friction/condim/solref/solimp instead
of the rigid-rigid `contact_param()` path. The geometric collision (point,
normal, depth) is identical.

### Flex-vs-mesh

MuJoCo computes sphere-vs-mesh collision using the mesh's convex hull (which
MuJoCo always computes at build time) via GJK/EPA. The sphere is tested
against the convex hull as a convex shape pair.

CortenForge equivalents:
- **Hull path:** `CollisionShape::convex_mesh_from_hull(hull)` +
  `gjk_epa_contact(&sphere_shape, &sphere_pose, &hull_shape, &mesh_pose, ...)`
  from `gjk_epa.rs:1174`. Returns `Option<GjkContact>` with `point`,
  `normal` (from B toward A), `penetration`.
- **Per-triangle fallback:** `mesh_sphere_contact(mesh, &mesh_pose,
  sphere_center, sphere_radius, use_bvh)` from `mesh.rs:1076`. Returns
  `Option<MeshContact>` with `point`, `normal` (outward from mesh), `penetration`.

### Flex-vs-hfield

MuJoCo computes sphere-vs-hfield using `mjc_ConvexHField()` (prism-based
GJK). CortenForge has `heightfield_sphere_contact()` at
`heightfield.rs:457` — a point-sampling approach using corner-origin local
coordinates. For tiny flex vertex spheres (typically radius < 0.05), both
approaches give equivalent contacts.

`heightfield_sphere_contact` takes `(&HeightFieldData, &Pose,
Point3<f64>, f64)` and returns `Option<HeightFieldContact>` with `point`,
`normal` (upward from terrain), `penetration`.

**Corner-origin convention:** `heightfield_sphere_contact` uses corner-origin
local coordinates (`0..extent_x`, `0..extent_y`). The MuJoCo/CortenForge
geom position is center-origin. A centering offset must be applied when
constructing the hfield `Pose`:
```rust
let hf_offset = geom_mat * Vector3::new(-hf_size[0], -hf_size[1], 0.0);
let hf_pose = Pose::from_position_rotation(
    Point3::from(geom_pos + hf_offset), geom_quat);
```
This pattern is established in `sdf_collide.rs:88-90`.

### Flex-vs-SDF

MuJoCo computes sphere-vs-SDF by querying the SDF distance field at the
vertex position. CortenForge has `sdf_sphere_contact()` at `sdf.rs:583`.
Takes `(&SdfCollisionData, &Pose, Point3<f64>, f64)` and returns
`Option<SdfContact>` with `point`, `normal` (outward from SDF surface),
`penetration`. No centering offset needed — SDF uses its own internal origin.

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| Flex vertex vs mesh geom | Sphere-vs-convex-hull via GJK/EPA — produces contact with point, normal, depth | `_ => return None` at `flex_collide.rs:153` — no contact generated |
| Flex vertex vs hfield geom | Sphere-vs-hfield via `mjc_ConvexHField()` prism-based — produces contact | `_ => return None` — no contact generated |
| Flex vertex vs SDF geom | Sphere-vs-SDF via distance field query — produces contact | `_ => return None` — no contact generated |
| Normal direction | From geom surface toward vertex (outward from geom) | N/A (no contact generated) |
| Contact parameter combination | Uses flex-specific params (flex friction, condim, solref, solimp) | Already implemented in `make_contact_flex_rigid()` at `flex_collide.rs:169` — works for any geom type |
| Missing geom data guard | No crash if geom has no mesh/hfield/SDF data — skip silently | N/A (must implement: `geom_mesh[gi] = None` → no contact) |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| Flex vertex sphere | Sphere at vertex position with `flex_radius` | `sphere_pos = vpos`, `sphere_radius = radius + effective_margin` (already inflated in `mj_collision_flex` at `mod.rs:604`) | Direct port — `narrowphase_sphere_geom` receives the inflated radius |
| `MeshContact.normal` | Outward from mesh surface (toward sphere) | Same — `mesh.rs:1113` | Direct port — no negation needed |
| `HeightFieldContact.normal` | Upward from terrain (toward object) | Same — `heightfield.rs:503` | Direct port — no negation needed |
| `SdfContact.normal` | Outward from SDF surface (toward object) | Same — `sdf.rs:611` | Direct port — no negation needed |
| Hfield local coords | Center-origin | Corner-origin in `heightfield_sphere_contact` | Apply centering offset: `hf_offset = geom_mat * Vector3::new(-hf_size[0], -hf_size[1], 0.0)` |
| `Pose` construction | `(pos, mat)` pair | `Pose::from_position_rotation(Point3::from(pos), UnitQuaternion::from_matrix(&mat))` | Established pattern in `sdf_collide.rs:42-47` |
| `GjkContact.normal` | From A toward B (despite struct doc claiming B→A) | CortenForge actual behavior — when sphere is A and hull is B, raw normal points from sphere toward hull. Verified by `hfield.rs:170` comment. | **Negate** — `return Some((..., -gjk.normal, ...))` to get outward-from-mesh convention |
| BVH midphase flag | `DISABLE_MIDPHASE` controls BVH usage for mesh collision | `!disabled(model, DISABLE_MIDPHASE)` → `use_bvh` | Thread `use_bvh` for per-triangle fallback path |
| GJK/EPA solver params | Model-level `ccd_iterations` and `ccd_tolerance` control solver | `model.ccd_iterations`, `model.ccd_tolerance` — same fields used by `mesh_collide.rs:72-73` and `hfield.rs:167-168` | Use `model.ccd_iterations` and `model.ccd_tolerance` for hull-path GJK call — NOT hardcoded defaults |

---

## Architecture Decisions

### AD-1: Dispatch strategy — Option B (early return from match arms)

**Problem:** `narrowphase_sphere_geom` returns `Option<(f64, Vector3<f64>,
Vector3<f64>)>` which is `(depth, normal, contact_pos)`. The match block
computes `(d_surface, normal)` for primitive types, then post-match code
converts: `depth = sphere_radius - d_surface`, `contact_pos = v - normal *
d_surface`. The existing collision functions for complex types return their
own contact structs with `penetration`, `point`, and `normal`. Using
Option A would require converting `penetration → d_surface` then
re-converting back — a redundant round-trip.

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| A | Return `(d_surface, normal)` from match arms, let post-match convert | Uniform with primitives | Redundant conversion: `d_surface = radius - penetration` then `depth = radius - d_surface = penetration`. Post-match `contact_pos = v - normal * d_surface` disagrees with function's own contact `point` (especially for hfield edge clamping). |
| B | Early return `Some((depth, normal, contact_pos))` from match arms | Uses function's native contact point/depth directly. Precedent: Cylinder arm uses `return None` at line 123. No conversion errors. | Bypasses post-match code — different control flow than primitives. |
| C | Bypass `narrowphase_sphere_geom` entirely, handle in `mj_collision_flex()` | Clean separation of primitive/complex dispatch | Duplicates the guard logic and `make_contact_flex_rigid` call in `mod.rs`. Changes the outer loop architecture. |

**Chosen:** Option B — Early return from match arms. Justification:
1. Uses each function's native `penetration`, `point`, and `normal` directly
   — no conversion errors.
2. Precedent exists: the Cylinder arm already uses `return None` inside the
   match block (`flex_collide.rs:123`).
3. The heightfield contact point comes from the clamped sample position, not
   `v - normal * d_surface`. Using the function's contact point preserves
   correct edge behavior.
4. Minimal code change — adds 3 match arms with early returns.

---

## Specification

### S1. Flex-vs-mesh match arm

**File:** `sim/L0/core/src/collision/flex_collide.rs` (line 152, in the
match block of `narrowphase_sphere_geom`)
**MuJoCo equivalent:** Standard sphere-vs-mesh collision in
`engine_collision_primitive.c`
**Design decision:** Option B early return. Uses convex hull GJK path when
hull is available (MuJoCo-conformant), per-triangle BVH fallback when not
(CortenForge extension for meshes without hull). See AD-1.

**Before** (current code):
```rust
// Mesh/Hfield/Sdf: not yet supported for flex-vertex collision
_ => return None,
```

**After** (new match arm, inserted before the `_` catch-all):
```rust
GeomType::Mesh => {
    let mesh_id = model.geom_mesh[geom_idx]?;
    let mesh = &model.mesh_data[mesh_id];
    let geom_quat = UnitQuaternion::from_matrix(&geom_mat);
    let mesh_pose = Pose::from_position_rotation(
        Point3::from(geom_pos), geom_quat,
    );

    // Primary path: convex hull GJK (MuJoCo-conformant)
    if let Some(hull) = mesh.convex_hull() {
        let hull_shape = CollisionShape::convex_mesh_from_hull(hull);
        let sphere_shape = CollisionShape::Sphere {
            radius: sphere_radius,
        };
        let sphere_pose = Pose::from_position_rotation(
            Point3::from(v), UnitQuaternion::identity(),
        );

        if let Some(gjk) = gjk_epa_contact(
            &sphere_shape,
            &sphere_pose,
            &hull_shape,
            &mesh_pose,
            model.ccd_iterations,
            model.ccd_tolerance,
        ) {
            // GJK/EPA returns normal from shape_a toward shape_b
            // (see hfield.rs:170 comment). Here A=sphere, B=hull, so
            // the raw normal points from vertex toward mesh. Negate to
            // get outward-from-mesh-toward-vertex convention.
            return Some((gjk.penetration, -gjk.normal, gjk.point.coords));
        }
        return None;
    }

    // Fallback: per-triangle BVH (meshes without hull)
    let use_bvh = !disabled(model, DISABLE_MIDPHASE);
    if let Some(mc) = mesh_sphere_contact(
        mesh,
        &mesh_pose,
        Point3::from(v),
        sphere_radius,
        use_bvh,
    ) {
        // MeshContact.normal points outward from mesh. Matches convention.
        return Some((mc.penetration, mc.normal, mc.point.coords));
    }
    return None;
}
```

**New imports** (add to file header):
```rust
use crate::collision_shape::CollisionShape;
use crate::gjk_epa::gjk_epa_contact;
use crate::mesh::mesh_sphere_contact;
use crate::types::{DISABLE_MIDPHASE, disabled};
use nalgebra::{Point3, UnitQuaternion};
use sim_types::Pose;
```

### S2. Flex-vs-hfield match arm

**File:** `sim/L0/core/src/collision/flex_collide.rs` (in the same match
block, before the `_` catch-all)
**MuJoCo equivalent:** Standard sphere-vs-hfield collision in
`engine_collision_primitive.c` via `mjc_ConvexHField()`
**Design decision:** Option B early return. Reuses
`heightfield_sphere_contact()` directly — simpler than constructing a
virtual geom for `collide_hfield_multi`. Requires centering offset for
hfield corner-origin coords.

**After:**
```rust
GeomType::Hfield => {
    let hfield_id = model.geom_hfield[geom_idx]?;
    let hfield = &model.hfield_data[hfield_id];
    let hf_size = &model.hfield_size[hfield_id];

    // Centering offset: convert center-origin to corner-origin
    let hf_offset = geom_mat * Vector3::new(-hf_size[0], -hf_size[1], 0.0);
    let geom_quat = UnitQuaternion::from_matrix(&geom_mat);
    let hf_pose = Pose::from_position_rotation(
        Point3::from(geom_pos + hf_offset), geom_quat,
    );

    if let Some(hc) = heightfield_sphere_contact(
        hfield,
        &hf_pose,
        Point3::from(v),
        sphere_radius,
    ) {
        // HeightFieldContact.normal points up from terrain. Matches convention.
        return Some((hc.penetration, hc.normal, hc.point.coords));
    }
    return None;
}
```

**New imports:**
```rust
use crate::heightfield::heightfield_sphere_contact;
```

### S3. Flex-vs-SDF match arm

**File:** `sim/L0/core/src/collision/flex_collide.rs` (in the same match
block, before the `_` catch-all)
**MuJoCo equivalent:** Standard sphere-vs-SDF collision in
`engine_collision_primitive.c`
**Design decision:** Option B early return. Reuses `sdf_sphere_contact()`
directly. No centering offset needed — SDF uses its own internal origin.

**After:**
```rust
GeomType::Sdf => {
    let sdf_id = model.geom_sdf[geom_idx]?;
    let sdf = &model.sdf_data[sdf_id];
    let geom_quat = UnitQuaternion::from_matrix(&geom_mat);
    let sdf_pose = Pose::from_position_rotation(
        Point3::from(geom_pos), geom_quat,
    );

    if let Some(sc) = sdf_sphere_contact(
        sdf,
        &sdf_pose,
        Point3::from(v),
        sphere_radius,
    ) {
        // SdfContact.normal points outward from SDF. Matches convention.
        return Some((sc.penetration, sc.normal, sc.point.coords));
    }
    return None;
}
```

**New imports:**
```rust
use crate::sdf::sdf_sphere_contact;
```

### S4. Remove catch-all comment and update

**File:** `sim/L0/core/src/collision/flex_collide.rs`
**Design decision:** After S1–S3, all `GeomType` variants have explicit
arms. The `_` catch-all can be narrowed or removed. Since `GeomType` is
exhaustive and all variants are covered (Plane, Sphere, Box, Capsule,
Cylinder, Ellipsoid, Mesh, Hfield, Sdf), replace the `_` with an
exhaustive pattern or keep as defensive catch-all with updated comment.

**Before:**
```rust
// Mesh/Hfield/Sdf: not yet supported for flex-vertex collision
_ => return None,
```

**After:**
```rust
// All GeomType variants handled above. Defensive guard for future
// enum extensions — should never be reached for known types.
_ => return None,
```

---

## Acceptance Criteria

### AC1: Flex-vs-mesh generates contact *(runtime test — analytically derived)*
**Given:** A flex vertex at `(0, 0, 0.45)`, `sphere_radius = 0.1` (the
inflated radius passed to `narrowphase_sphere_geom`), and a cube mesh geom
(half-size 0.5, centered at origin with convex hull). Sphere extends from
Z=0.35 to Z=0.55. Hull top face at Z=0.50. Overlap region: Z=[0.35, 0.50].
**After:** Run `narrowphase_sphere_geom` with the vertex sphere against the mesh geom.
**Assert:** Contact is generated. `depth ≈ 0.15` ± 0.05 (GJK/EPA minimum
translation distance to separate: push sphere up by 0.15 so its bottom
clears the hull top at Z=0.50). Normal Z-component > 0.5 (pointing from
mesh toward vertex along +Z for this configuration).
**Field:** Return value of `narrowphase_sphere_geom` (Some with positive depth).

### AC2: Flex-vs-hfield generates contact *(runtime test — analytically derived)*
**Given:** A flex vertex at `(0, 0, 0.45)`, `sphere_radius = 0.1`, above a
flat heightfield at Z=0.5 (5×5 grid, half-extent 2.0). Vertex sphere bottom
at Z=0.35, terrain at Z=0.5 → penetrating.
**After:** Run `narrowphase_sphere_geom`.
**Assert:** Contact generated. `penetration ≈ 0.15` (terrain at 0.5, sphere
bottom at 0.35). Normal ≈ (0, 0, 1) (flat terrain).
**Field:** Return value of `narrowphase_sphere_geom`.

### AC3: Flex-vs-SDF generates contact *(runtime test — analytically derived)*
**Given:** A flex vertex at `(0, 0, 0.95)`, `sphere_radius = 0.1`, and a
sphere SDF geom (radius 1.0, centered at origin). Distance from vertex
center to SDF surface = 1.0 - 0.95 = 0.05. Since `sphere_radius` (0.1) >
distance (0.05), the sphere penetrates the SDF.
**After:** Run `narrowphase_sphere_geom`.
**Assert:** Contact generated. `depth ≈ 0.05` ± 0.02 (penetration =
sphere_radius - distance_to_surface = 0.1 - 0.05). Normal Z-component >
0.5 (outward from SDF surface toward vertex, approximately +Z for this
configuration).
**Field:** Return value of `narrowphase_sphere_geom`.

### AC4: Missing data guard — no contact, no panic *(runtime test)*
**Given:** A flex vertex sphere against a geom with `GeomType::Mesh` but
`geom_mesh[geom_idx] = None` (no mesh data assigned).
**After:** Run `narrowphase_sphere_geom`.
**Assert:** Returns `None` (no contact, no panic).
**Field:** Return value.

### AC5: Missing hfield data guard *(runtime test)*
**Given:** A flex vertex sphere against a geom with `GeomType::Hfield` but
`geom_hfield[geom_idx] = None`.
**After:** Run `narrowphase_sphere_geom`.
**Assert:** Returns `None`.

### AC6: Missing SDF data guard *(runtime test)*
**Given:** A flex vertex sphere against a geom with `GeomType::Sdf` but
`geom_sdf[geom_idx] = None`.
**After:** Run `narrowphase_sphere_geom`.
**Assert:** Returns `None`.

### AC7: Vertex above surface — no contact *(runtime test)*
**Given:** A flex vertex sphere well above a mesh/hfield/SDF surface (no
penetration).
**After:** Run `narrowphase_sphere_geom`.
**Assert:** Returns `None` for each complex geom type.

### AC8: Normal direction convention *(code review)*
**Verify:** In all three new match arms (Mesh, Hfield, Sdf), the returned
normal points from the geom surface toward the vertex. No normal negation
is applied. The existing contact functions already return normals in this
convention (documented in Convention Notes).

### AC9: Catch-all comment updated *(code review)*
**Verify:** The `_ => return None` catch-all at the end of the match block
no longer says "Mesh/Hfield/Sdf: not yet supported." All three types have
explicit match arms above it.

### AC10: Flex-vs-mesh per-triangle fallback *(runtime test — analytically derived)*
**Given:** A flex vertex sphere against a mesh geom that has no convex hull
(mesh with `convex_hull() = None`, BVH available).
**After:** Run `narrowphase_sphere_geom`.
**Assert:** Contact generated via per-triangle BVH fallback. Positive depth.
**Field:** Return value.

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (flex-vs-mesh contact) | T1, TS1 | Direct + edge case |
| AC2 (flex-vs-hfield contact) | T2, TS2, TS4 | Direct + edge cases |
| AC3 (flex-vs-SDF contact) | T3 | Direct |
| AC4 (no mesh data guard) | T4 | Direct |
| AC5 (no hfield data guard) | T4 | Direct (same test, different geom type) |
| AC6 (no SDF data guard) | T4 | Direct (same test, different geom type) |
| AC7 (above surface, no contact) | T5, TS3 | Direct + edge case |
| AC8 (normal direction) | — | Code review (manual) |
| AC9 (catch-all comment) | — | Code review (manual) |
| AC10 (per-triangle fallback) | T6 | Direct |

---

## Test Plan

### T1: Flex vertex sphere vs mesh with convex hull → AC1
**File:** `sim/L0/core/src/collision/flex_collide.rs` (test module)

Build a minimal `Model` with one mesh geom (unit cube, half-size 0.5, with
convex hull computed). Place a flex vertex sphere at `(0, 0, 0.45)` with
`sphere_radius = 0.1` (the inflated radius). Mesh centered at origin, hull
top face at Z=0.5. Sphere extends from Z=0.35 to Z=0.55.

Call `narrowphase_sphere_geom(sphere_pos, 0.1, geom_idx, &model,
geom_pos, geom_mat)`.

**Assert:**
- Returns `Some((depth, normal, contact_pos))`.
- `depth` in range `[0.10, 0.20]` (expected ≈ 0.15: minimum translation
  to separate is push sphere up so bottom clears hull top at Z=0.50).
- `normal.z > 0.5` (normal points upward from mesh toward vertex).

**Expected value derivation:** Analytically: sphere bottom at Z=0.35,
hull top at Z=0.50. Overlap = 0.15. GJK/EPA computes minimum penetration
depth, which for sphere-center-inside-cube is the shortest push-out
distance. The +Z face is nearest to the sphere center (distance 0.05),
so EPA penetration = 0.05 + sphere_radius_contribution ≈ 0.15.
Tolerance ±0.05 to accommodate GJK/EPA solver precision.

### T2: Flex vertex sphere vs heightfield → AC2
**File:** `sim/L0/core/src/collision/flex_collide.rs` (test module)

Build a minimal `Model` with one hfield geom (5×5 flat at height 0.5,
half-extent 2.0). Place vertex sphere at `(0, 0, 0.45)` with
`sphere_radius = 0.1`.

**Assert:**
- Returns `Some`.
- `depth ≈ 0.15` (terrain at 0.5, sphere bottom at 0.35) within ±0.05
  tolerance (point-sampling approximation).
- `normal ≈ (0, 0, 1)` (flat terrain, Z ≈ 1.0 within 0.1).

**Expected value derivation:** Analytically: sphere bottom =
0.45 - 0.1 = 0.35. Terrain at 0.5. Penetration = 0.5 - 0.35 = 0.15.

### T3: Flex vertex sphere vs SDF → AC3
**File:** `sim/L0/core/src/collision/flex_collide.rs` (test module)

Build a minimal `Model` with one SDF geom (sphere SDF, radius 1.0 centered
at origin). Place vertex sphere at `(0, 0, 0.95)` with `sphere_radius = 0.1`.
Sphere center is at distance 0.95 from origin, SDF surface at 1.0,
penetration = 0.1 - (1.0 - 0.95) = 0.05.

**Assert:**
- Returns `Some`.
- `depth ≈ 0.05` ± 0.02 (SDF distance query precision).
- `normal.z > 0.5` (outward from SDF surface toward vertex, ≈ +Z).

**Expected value derivation:** Analytically: distance from sphere center to
SDF surface = 1.0 - 0.95 = 0.05. Sphere radius = 0.1. Penetration =
sphere_radius - distance = 0.1 - 0.05 = 0.05. SDF gradient at (0,0,0.95)
points radially outward ≈ (0,0,1). Tolerance ±0.02 for SDF grid
interpolation precision.

### T4: Missing geom data → no contact, no panic → AC4, AC5, AC6
**File:** `sim/L0/core/src/collision/flex_collide.rs` (test module)

Build a minimal `Model` with geom types Mesh, Hfield, Sdf but with
`geom_mesh[idx] = None`, `geom_hfield[idx] = None`, `geom_sdf[idx] = None`.

**Assert:** `narrowphase_sphere_geom` returns `None` for each. No panic.

### T5: Vertex above surface — no contact → AC7
**File:** `sim/L0/core/src/collision/flex_collide.rs` (test module)

Place vertex sphere well above each complex geom (e.g., Z = 10.0 with
sphere_radius = 0.1, geom surface at Z ≤ 1.0).

**Assert:** Returns `None` for mesh, hfield, and SDF geom types.

### T6: Flex-vs-mesh per-triangle BVH fallback → AC10
**File:** `sim/L0/core/src/collision/flex_collide.rs` (test module)

Build a mesh geom (single triangle or small mesh) WITHOUT computing
convex hull (`mesh.convex_hull() = None`). Place vertex sphere penetrating
the mesh surface.

**Assert:**
- Returns `Some`.
- `depth > 0` (per-triangle contact detected via BVH fallback).

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| Mesh with no convex hull | Falls back to per-triangle BVH — verify fallback works | T6 | AC10 |
| `geom_mesh[idx] = None` | Missing data guard — `?` operator returns None | T4 | AC4 |
| `geom_hfield[idx] = None` | Missing data guard | T4 | AC5 |
| `geom_sdf[idx] = None` | Missing data guard | T4 | AC6 |
| Vertex above all surfaces | No contact should be generated | T5 | AC7 |
| Zero-radius flex vertex | `sphere_radius = 0` passed through; contact functions handle degenerately small spheres | TS1 | AC1 |
| Hfield vertex at grid boundary | Centering offset must be correct; `heightfield_sphere_contact` clamps to bounds | T2 | AC2 |
| Hfield vertex exactly on surface | Boundary between contact/no-contact — `penetration = 0` case | TS2 | AC2 |
| SDF vertex outside bounding box | `sdf.distance()` returns `None` for out-of-bounds queries → `sdf_sphere_contact` returns `None` | TS3 | AC7 |
| Single-triangle mesh (no hull) | Minimal mesh geometry — per-triangle fallback on simplest mesh | T6 | AC10 |
| Single-cell hfield (2×2 grid) | Minimal heightfield — verifies centering offset works at minimum grid size | TS4 | AC2 |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| TS1 (zero-radius vertex) | Zero-radius sphere against mesh | Degenerate input — verifies no panic and reasonable behavior |
| TS2 (hfield vertex on surface) | Vertex sphere bottom exactly at terrain height | Boundary condition: `penetration = sphere_radius - d_surface` ≈ 0 — verifies no false positive |
| TS3 (SDF vertex outside bbox) | Vertex sphere far outside SDF bounding box | Verifies `sdf_sphere_contact` returns None gracefully, not garbage values |
| TS4 (single-cell hfield) | 2×2 hfield with vertex sphere penetrating | Minimum valid hfield size — verifies centering offset and contact at degenerate grid |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| `narrowphase_sphere_geom` for Mesh/Hfield/Sdf geom types | Returns `None` (no contact) | Returns `Some(...)` when vertex sphere penetrates | Toward MuJoCo — flex bodies now correctly collide with complex geoms | Flex bodies near mesh/hfield/SDF geoms will generate contacts where none existed before | None — transparent conformance improvement |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/core/src/collision/flex_collide.rs` | Add 3 match arms (Mesh, Hfield, Sdf) with early returns; add imports; update catch-all comment | +60 / ~2 modified |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| All existing `flex_collide` tests | `flex_collide.rs` (no existing tests for complex types) | Pass (unchanged) | No existing tests exercise Mesh/Hfield/Sdf flex paths — they were no-ops |
| All existing mesh/hfield/SDF collision tests | Various | Pass (unchanged) | This spec adds new callers of existing functions, doesn't modify the functions |
| All Phase 8 constraint tests | Various | Pass (unchanged) | No constraint code modified |

### Non-Modification Sites

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| `collision/mod.rs:549-616` | `mj_collision_flex()` outer loop | Dispatch strategy is Option B (inside `narrowphase_sphere_geom`), not Option C (modify outer loop) |
| `collision/mod.rs:602-613` | Calls `narrowphase_sphere_geom` then `make_contact_flex_rigid` | Same call pattern works for all geom types — no changes needed |
| `collision/mesh_collide.rs` | `collide_with_mesh()` rigid-rigid dispatch | Different dispatch path; flex uses `narrowphase_sphere_geom`, not `collide_with_mesh` |
| `collision/sdf_collide.rs` | `collide_with_sdf()` rigid-rigid dispatch | Different dispatch path |
| `collision/hfield.rs` | `collide_hfield_multi()` rigid-rigid dispatch | Different dispatch path; flex uses `heightfield_sphere_contact` directly |
| `mesh.rs:1076` | `mesh_sphere_contact()` | Read-only consumer — called from new flex match arm, not modified |
| `heightfield.rs:457` | `heightfield_sphere_contact()` | Read-only consumer |
| `sdf.rs:583` | `sdf_sphere_contact()` | Read-only consumer |

---

## Execution Order

1. **S1 (Mesh arm)** first — largest change, requires imports. Verify with T1 + T4 + T5.
2. **S2 (Hfield arm)** after S1 — requires same imports plus `heightfield_sphere_contact`. Verify with T2.
3. **S3 (SDF arm)** after S2 — requires `sdf_sphere_contact` import. Verify with T3.
4. **S4 (catch-all comment)** last — trivial update. Verify AC9 by inspection.

All four sections land in a single commit since they affect one file and are
interdependent (imports shared across all three new arms). Run
`cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after all
sections complete.

---

## Out of Scope

- **Flex self-collision** (Phase 10, §42A-iv) — distinct subsystem.
  *Conformance impact: deferred to Phase 10.*
- **Flex-flex cross-body collision filtering** (Phase 10, §42A-v) — separate
  feature. *Conformance impact: deferred to Phase 10.*
- **SAP for flex broadphase** (DT-69) — performance optimization.
  *Conformance impact: none — brute-force produces identical contacts.*
- **Flex adhesion contacts** (DT-72) — low priority MuJoCo compat.
  *Conformance impact: minimal.*
- **GPU flex pipeline** (DT-67) — post-v1.0.
  *Conformance impact: none.*
- **Prism-based hfield collision for flex vertices** — MuJoCo uses
  `mjc_ConvexHField()` for all convex-vs-hfield, but the point-sampling
  `heightfield_sphere_contact()` produces equivalent results for tiny
  flex vertex spheres. If future analysis shows divergence for large-radius
  flex vertices, upgrade to prism-based path. *Conformance impact: negligible
  for typical flex radii (< 0.1).*
