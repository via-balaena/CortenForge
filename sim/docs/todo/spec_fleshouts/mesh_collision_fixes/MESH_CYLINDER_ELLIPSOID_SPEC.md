# Mesh–Cylinder and Mesh–Ellipsoid: GJK/EPA Dispatch Fix

**Status:** Draft
**Branch:** `feature/mesh-collision-examples`
**Blocked by:** Nothing — all infrastructure exists.
**Blocks:** mesh-collision examples (COVERAGE_SPEC §14)

---

## Problem

`collide_with_mesh()` in `sim/L0/core/src/collision/mesh_collide.rs` uses
approximations for two geometry types:

1. **Cylinder → Capsule** (lines 162–167, 215–220): `GeomType::Cylinder`
   is routed through `mesh_capsule_contact()`, which rounds the flat end-caps
   into hemispheres. A cylinder resting on its flat end on a mesh surface
   sinks into it; the contact point and normal are wrong. Specifically, the
   capsule approximation places the lowest point `radius` below the true
   cylinder bottom — a standing cylinder with `half_length=0.2, radius=0.1`
   rests with its center at `half_length + radius = 0.3` instead of the
   correct `half_length = 0.2`.

2. **Ellipsoid → Sphere** (lines 170–173, 223–225): `GeomType::Ellipsoid`
   is routed through `mesh_sphere_contact()` with `radius = max(rx, ry, rz)`.
   A flat ellipsoid (disc-like, e.g. radii `[0.3, 0.3, 0.05]`) gets a
   bounding sphere 6x too large in the thin axis — contact geometry is wrong
   and the object floats above the surface at `z = 0.3` instead of `z = 0.05`.

Both approximations were originally documented as "conservative" in the
module's doc comment (lines 19–21). They were acceptable placeholders when
mesh collision was first implemented, but they produce visually and
physically incorrect behavior that would be exposed by the mesh-collision
examples.

## Why GJK/EPA is the correct fix

The GJK/EPA pipeline already supports both shapes with exact support
functions:

| Shape | Support function | Location |
|-------|-----------------|----------|
| `Shape::Cylinder` | `support_cylinder()` | `gjk_epa.rs:170–173` |
| `Shape::Ellipsoid` | `support_ellipsoid()` | `gjk_epa.rs:174` |

The existing mesh–mesh convex-hull path (lines 60–143) demonstrates the
pattern: extract the mesh's `ConvexHull`, wrap it as `Shape::ConvexMesh`,
construct a `Shape` for the other geom, and run `gjk_epa_contact()` +
optional MULTICCD + margin-zone fallback. The cylinder and ellipsoid fixes
replicate this pattern exactly.

`geom_to_shape()` in `narrow.rs:339–343` already returns the correct
`Shape::Cylinder` and `Shape::Ellipsoid` variants. We just need to call it
from the mesh dispatch path.

## Non-convex fallback

When a mesh lacks a convex hull (i.e., `mesh.convex_hull()` returns `None`),
the current code falls back to triangle-level functions (`mesh_sphere_contact`,
`mesh_capsule_contact`, `mesh_box_contact`). There are no triangle-level
functions for cylinder or ellipsoid, and writing them is not justified:

- Every mesh built through the MJCF pipeline gets `compute_convex_hull()`
  called at build time (`mjcf/builder/mesh.rs:54–56`).
- The only way to hit the no-hull path is to manually construct a
  `TriangleMeshData` without calling `compute_convex_hull()`.
- MuJoCo always computes convex hulls for meshes (`MakeGraph()` in
  `user_mesh.cc`).

**Decision:** When the mesh lacks a convex hull and the other geom is
cylinder or ellipsoid, return no contacts and emit a `tracing::warn!`.
This matches MuJoCo's behavior (convex hull is always available) and avoids
writing dead code.

---

## Normal convention

The GJK/EPA contact normal convention is **"from B toward A"**
(`gjk_epa.rs:57`). The collision pipeline convention is: pass the normal
directly to `make_contact_from_geoms` when `gjk_epa_contact` is called as
`(shape_geom1, pose_geom1, shape_geom2, pose_geom2)` — matching the
argument order used by the narrow.rs GJK fallback (line 185) and the
mesh–mesh hull path (line 67).

**Consequence for the helper:** The helper must receive shapes in
**geom-order** (shape1 = geom1, shape2 = geom2), NOT in mesh-first order.
If the helper always put the mesh hull as the first GJK argument, the
swapped `(prim, Mesh)` branch would produce inverted normals because
geom1 = primitive, geom2 = mesh, but shape_A = hull = geom2.

Callers construct shapes in the correct geom order:

```
(Mesh, Cylinder): shape1 = hull (geom1=mesh), shape2 = cylinder (geom2)
(Cylinder, Mesh): shape1 = cylinder (geom1),   shape2 = hull (geom2=mesh)
```

No manual normal negation is needed in either branch.

---

## Changes

All changes are in `sim/L0/core/src/collision/mesh_collide.rs`.

### Change 1: Extract helper — `gjk_epa_shape_pair()`

Factor the GJK/EPA + MULTICCD + margin-zone logic from the mesh–mesh hull
path (lines 63–143) into a reusable helper. The helper mirrors the narrow.rs
GJK fallback pattern (narrow.rs:167–267) but lives in mesh_collide.rs to
keep mesh dispatch self-contained.

```rust
/// GJK/EPA collision between two convex shapes with MULTICCD and margin support.
///
/// Arguments must be in geom-order: shape1/pose1 correspond to geom1,
/// shape2/pose2 correspond to geom2. This ensures the GJK normal convention
/// ("from B toward A" = from geom2 toward geom1) is correct without
/// manual negation.
///
/// Returns Contact(s) directly — no MeshContact intermediate.
fn gjk_epa_shape_pair(
    model: &Model,
    shape1: &Shape,
    pose1: &Pose,
    shape2: &Shape,
    pose2: &Pose,
    geom1: usize,
    geom2: usize,
    margin: f64,
) -> Vec<Contact>
```

The helper contains the same logic currently inlined in the mesh–mesh hull
path: `gjk_epa_contact()` → optional `multiccd_contacts()` → fallback
`gjk_distance()` for margin-zone contacts.

This helper is called from:
- **Mesh–Mesh hull path** (existing, refactored):
  `gjk_epa_shape_pair(model, &hull1_shape, &pose1, &hull2_shape, &pose2, geom1, geom2, margin)`
- **Mesh–Cylinder** (new)
- **Mesh–Ellipsoid** (new)

### Change 2: Mesh–Cylinder dispatch

Replace the `GeomType::Capsule | GeomType::Cylinder` combined arm with
separate arms. Both branches (`(Mesh, prim)` and `(prim, Mesh)`) are shown.

**`(Mesh, prim)` branch** (geom1 = mesh, geom2 = cylinder):

```rust
GeomType::Capsule => {
    // ... existing capsule path (unchanged)
}
GeomType::Cylinder => {
    if let Some(hull) = mesh.convex_hull() {
        let shape1 = Shape::convex_mesh(hull.clone());
        let shape2 = Shape::Cylinder {
            half_length: size2.y,
            radius: size2.x,
        };
        return gjk_epa_shape_pair(
            model, &shape1, &pose1, &shape2, &pose2,
            geom1, geom2, margin,
        );
    }
    warn!("mesh-cylinder collision requires convex hull; returning no contacts");
    return vec![];
}
```

**`(prim, Mesh)` branch** (geom1 = cylinder, geom2 = mesh):

```rust
GeomType::Cylinder => {
    if let Some(hull) = mesh.convex_hull() {
        let shape1 = Shape::Cylinder {
            half_length: size1.y,
            radius: size1.x,
        };
        let shape2 = Shape::convex_mesh(hull.clone());
        return gjk_epa_shape_pair(
            model, &shape1, &pose1, &shape2, &pose2,
            geom1, geom2, margin,
        );
    }
    warn!("mesh-cylinder collision requires convex hull; returning no contacts");
    return vec![];
}
```

Note: the `(prim, Mesh)` branch currently goes through a `MeshContact`
intermediate with normal negation at lines 255–260. The new cylinder and
ellipsoid arms early-return `Vec<Contact>` directly (like the Plane arm at
lines 227–247), bypassing the `MeshContact` conversion and the blanket
normal negation. The helper handles normals correctly via geom-ordered
GJK arguments.

### Change 3: Mesh–Ellipsoid dispatch

Replace the sphere-approximation arm with GJK/EPA. Same geom-order pattern.

**`(Mesh, prim)` branch** (geom1 = mesh, geom2 = ellipsoid):

```rust
GeomType::Ellipsoid => {
    if let Some(hull) = mesh.convex_hull() {
        let shape1 = Shape::convex_mesh(hull.clone());
        let shape2 = Shape::Ellipsoid { radii: size2 };
        return gjk_epa_shape_pair(
            model, &shape1, &pose1, &shape2, &pose2,
            geom1, geom2, margin,
        );
    }
    warn!("mesh-ellipsoid collision requires convex hull; returning no contacts");
    return vec![];
}
```

**`(prim, Mesh)` branch** (geom1 = ellipsoid, geom2 = mesh):

```rust
GeomType::Ellipsoid => {
    if let Some(hull) = mesh.convex_hull() {
        let shape1 = Shape::Ellipsoid { radii: size1 };
        let shape2 = Shape::convex_mesh(hull.clone());
        return gjk_epa_shape_pair(
            model, &shape1, &pose1, &shape2, &pose2,
            geom1, geom2, margin,
        );
    }
    warn!("mesh-ellipsoid collision requires convex hull; returning no contacts");
    return vec![];
}
```

### Change 4: Update doc comment

Remove the "approximations" note from the module doc comment (lines 19–21)
since both approximations are eliminated.

---

## Tests

All tests in `sim-core` and `sim-conformance-tests`. No new crate dependencies.

### Unit tests (in `mesh_collide.rs` or `collision/mod.rs`)

**T1: Cylinder flat-end on mesh plane.**
A cylinder (half_length=0.2, radius=0.1) centered at `(0, 0, 0.2)` standing
upright on a mesh-defined ground quad (4 vertices, 2 triangles, Z=0). Mesh
has convex hull. Expected: contact normal = +Z, contact point near Z=0,
penetration depth ~0.0 (just touching). The cylinder center rests at
`z = half_length = 0.2`.

**T2: Cylinder curved-side on mesh.**
A cylinder (half_length=0.2, radius=0.1) centered at `(0, 0, 0.1)` lying
on its side (rotated 90 deg about X or Y) on the same mesh quad. Contact
should be on the curved surface. Contact normal = +Z. The cylinder center
rests at `z = radius = 0.1`.

**T3: Ellipsoid flat-axis on mesh.**
An oblate ellipsoid (radii `[0.3, 0.3, 0.05]`) centered at `(0, 0, 0.05)`
resting on a mesh quad at Z=0. Contact depth should reflect the actual
Z-radius (0.05), not the max radius (0.3). Expected: center at `z = 0.05`,
contact point near Z=0, penetration ~0.0.

**T4: Ellipsoid long-axis on mesh.**
A prolate ellipsoid (radii `[0.05, 0.05, 0.3]`) centered at `(0, 0, 0.3)`
standing upright on a mesh quad. Contact should use the Z-radius (0.3).
Expected: center at `z = 0.3`, contact near Z=0.

**T5: Cylinder vs mesh — capsule regression.**
Same geometry as T1. Verify the contact point is on the flat cap face at
`z = center_z - half_length`, NOT on a hemispherical surface at
`z = center_z - half_length - radius`. The capsule approximation would
place the rest height at `half_length + radius = 0.3`; the correct cylinder
rest height is `half_length = 0.2`. Verify rest height < 0.25 (rejects
capsule approximation).

**T6: Ellipsoid vs mesh — sphere regression.**
Same geometry as T3. The old sphere path would use `max_r = 0.3`, giving
rest height = 0.3; the correct rest height is 0.05. Verify rest height
< 0.1 (rejects sphere approximation).

**T7: Mesh–Cylinder swapped geom order.**
Same geometry as T1 but with `(Cylinder, Mesh)` geom order (triggers the
`(prim_type, GeomType::Mesh)` branch). Verify: contact normal and depth
produce identical physics — normal direction matches the `(Mesh, Cylinder)`
case after accounting for the geom1/geom2 convention. No manual negation
bugs.

**T8: Mesh–Ellipsoid swapped geom order.**
Same geometry as T3 but with `(Ellipsoid, Mesh)` order. Same verification
as T7.

**T9: No convex hull — graceful fallback.**
Create a `TriangleMeshData` without calling `compute_convex_hull()`. Call
`collide_with_mesh()` with a cylinder. Verify: returns empty contacts, no
panic.

### Integration tests (in `sim-conformance-tests`)

**T10: Cylinder-on-mesh-plane settling.**
MJCF scene: mesh-defined ground plane + free-body cylinder (half_length=0.2,
radius=0.1, density=1000), gravity=-9.81. Initial position: center at
`z = 0.5` (upright). Simulate 500 steps. Verify:
- Cylinder rests stably (velocity < 1e-3 after settling)
- Rest height of center ≈ 0.2 (half_length, within 1mm)
- Contact force ≈ weight (ratio within [0.99, 1.01])

**T11: Ellipsoid-on-mesh-plane settling.**
MJCF scene: mesh ground + free-body oblate ellipsoid (radii `[0.3, 0.3, 0.05]`,
density=1000). Initial position: center at `z = 0.5`. Simulate 500 steps.
Verify:
- Rest height of center ≈ 0.05 (not 0.3), within 1mm
- Contact force ≈ weight
- No interpenetration beyond solver tolerance

**T12: Mesh–Cylinder MULTICCD.**
Enable MULTICCD. Cylinder standing upright on mesh flat face. Verify >= 1
contact generated (MULTICCD may produce multiple for the flat cap).

---

## Session plan

| Session | Scope | Entry | Exit |
|---------|-------|-------|------|
| 1 | Extract `gjk_epa_shape_pair()` helper, refactor mesh–mesh hull path to use it. No behavior change. | This spec | All existing mesh collision tests pass. Helper compiles and is called from mesh–mesh path. |
| 2 | Mesh–Cylinder fix (Change 2) + tests T1, T2, T5, T7, T9. | Session 1 | T1–T2, T5, T7, T9 pass. Cylinder-on-mesh rests correctly. |
| 3 | Mesh–Ellipsoid fix (Change 3) + tests T3, T4, T6, T8. Update doc comment (Change 4). | Session 2 | T3–T4, T6, T8 pass. Ellipsoid-on-mesh rests at correct height. |
| 4 | Integration tests T10, T11, T12. Final `cargo test -p sim-core -p sim-conformance-tests`. | Session 3 | All 12 tests pass. No regressions. |

---

## Key files

| File | Role |
|------|------|
| `sim/L0/core/src/collision/mesh_collide.rs` | Primary change — dispatch fix + helper extraction |
| `sim/L0/core/src/collision/narrow.rs` | `geom_to_shape()` reference, GJK fallback pattern reference (read-only) |
| `sim/L0/core/src/gjk_epa.rs` | `support_cylinder`, `support_ellipsoid`, normal convention (read-only) |
| `sim/L0/core/src/collision/mod.rs` | Existing collision tests — add unit tests here |
| `sim/L0/tests/integration/` | Integration tests T10–T12 |
| `sim/L0/mjcf/src/builder/mesh.rs` | Convex hull build-time computation (read-only, confirms hull always exists) |

---

## Risk

**Low.** The GJK/EPA path for cylinder and ellipsoid is already exercised by
the non-mesh dispatch in `narrow.rs` (cylinder-cylinder, cylinder-box,
ellipsoid-* all go through GJK/EPA). This fix routes mesh-cylinder and
mesh-ellipsoid through the same proven path. The only new code is the helper
extraction and dispatch wiring.
