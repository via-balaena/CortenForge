# Cylinder MULTICCD Face Enumeration

**Status:** Draft
**Branch:** `feature/mesh-collision-examples`
**Blocked by:** Nothing — all infrastructure exists.
**Blocks:** T10 integration test (`mesh_cylinder_settling`), mesh-collision examples

---

## Problem

`support_face_points()` in `sim/L0/core/src/gjk_epa.rs:216–277` has
explicit arms for `Shape::Box` (returns up to 8 face vertices) and
`Shape::ConvexMesh` (returns all hull vertices sharing max dot product).
All other shapes — including `Shape::Cylinder` — fall through to the `_ =>`
catch-all at line 272, which returns a single support point.

This means MULTICCD (which calls `support_face_points` to enumerate contact
face vertices) produces only **1 contact** for cylinder flat caps, even
though the cap is geometrically flat and should produce multiple contacts
around the rim.

**Consequence:** A cylinder standing upright on a mesh surface gets a single
contact point on the cap rim. This is rotationally unstable — any
perturbation creates a torque that tips the cylinder. Over multiple
timesteps, the cylinder tips, accelerates, and tunnels through the mesh.
This was discovered when integration test T10 (`mesh_cylinder_settling`)
failed: the cylinder falls through a thick mesh slab despite correct
dispatch (unit tests T1, T2, T5, T7 all pass).

The ellipsoid settling test (T11) passes because an ellipsoid's contact
point at its pole is a natural stable equilibrium — any rotation moves the
contact point to resist the perturbation. Cylinders don't have this
property on their flat caps.

## Fix

Add a `Shape::Cylinder` arm to `support_face_points()` that returns
multiple points around the cap rim when the support direction aligns with
the cylinder axis (i.e., the contact face is a flat cap).

### Cylinder geometry

A cylinder with local Z-axis, half_length `h`, radius `r`:
- **Flat cap** (top): circle at `z = +h`, radius `r`
- **Flat cap** (bottom): circle at `z = -h`, radius `r`
- **Curved surface**: between the caps

When `support_face_points` is called with a direction that aligns with the
cylinder axis (e.g., `-Z` for a cylinder resting on its bottom cap), the
contact face is the entire bottom cap disc. We return N evenly-spaced
points around the cap rim.

When the direction is NOT aligned with the axis (contact on the curved
surface), the support is a single point — return it as-is (smooth surface,
same as current behavior).

### Algorithm

```
fn support_face_points_cylinder(pose, half_length, radius, direction):
    local_dir = pose.rotation.inverse() * direction
    
    // Decompose into axial and radial components
    axial = local_dir.z
    radial = sqrt(local_dir.x^2 + local_dir.y^2)
    
    // If direction is mostly axial (cap contact), enumerate rim points.
    // Threshold: axial component dominates radial by ~10:1.
    // This matches the regime where the cylinder support function returns
    // a cap point rather than a curved-surface point.
    if |axial| > radial * CAP_FACE_THRESHOLD:
        cap_z = sign(axial) * half_length
        // Generate N evenly-spaced points on the cap rim
        for i in 0..N_CAP_POINTS:
            angle = 2π * i / N_CAP_POINTS
            local_pt = (radius * cos(angle), radius * sin(angle), cap_z)
            points.push(pose.transform_point(local_pt))
        return points
    
    // Curved surface — single support point (smooth)
    return [support_cylinder(pose, half_length, radius, direction)]
```

### Parameters

- **`N_CAP_POINTS = 8`**: Number of rim points for flat cap enumeration.
  4 is the minimum for rotational stability (prevents tipping in any
  direction). 8 provides better force distribution and matches MuJoCo's
  typical multi-contact resolution for flat faces. More than 8 adds
  constraint rows without meaningful stability benefit.

- **`CAP_FACE_THRESHOLD = 10.0`**: Ratio threshold for detecting cap-aligned
  directions. When `|axial| / radial > 10`, the direction is within ~5.7
  degrees of the cylinder axis — firmly in cap-contact territory. Below
  this threshold, the contact is on the curved surface and a single point
  is appropriate.

---

## Changes

Single file: `sim/L0/core/src/gjk_epa.rs`.

### Change 1: Add `Shape::Cylinder` arm to `support_face_points()`

Insert before the `_ =>` catch-all (line 272):

```rust
Shape::Cylinder { half_length, radius } => {
    let local_dir = pose.rotation.inverse() * direction;
    let axial = local_dir.z.abs();
    let radial = (local_dir.x * local_dir.x + local_dir.y * local_dir.y).sqrt();

    if axial > radial * CAP_FACE_THRESHOLD {
        // Flat cap contact — enumerate rim points
        let cap_z = if local_dir.z >= 0.0 { *half_length } else { -*half_length };
        let n = N_CAP_POINTS;
        (0..n)
            .map(|i| {
                let angle = 2.0 * std::f64::consts::PI * i as f64 / n as f64;
                let local_pt = Point3::new(
                    radius * angle.cos(),
                    radius * angle.sin(),
                    cap_z,
                );
                pose.transform_point(&local_pt)
            })
            .collect()
    } else {
        // Curved surface — single support point
        let warm_start = Cell::new(0);
        vec![support(shape, pose, direction, &warm_start)]
    }
}
```

### Change 2: Add constants

```rust
/// Number of rim points for cylinder cap face enumeration in MULTICCD.
const N_CAP_POINTS: usize = 8;

/// Axial-to-radial ratio threshold for detecting cylinder cap contact.
/// When |axial| / radial > threshold, direction is within ~5.7 deg of axis.
const CAP_FACE_THRESHOLD: f64 = 10.0;
```

---

## Tests

All in `sim/L0/core/src/gjk_epa.rs` (unit tests) and
`sim/L0/tests/integration/mesh_cylinder_ellipsoid.rs` (integration).

### Unit tests (in `gjk_epa.rs` test module)

**T1: Cap-aligned direction returns N_CAP_POINTS.**
Call `support_face_points(cylinder, identity_pose, -Z)`. Expect 8 points.
All at `z = -half_length`. All at distance `radius` from axis. Evenly
spaced in angle (consecutive angular separation = π/4).

**T2: Radial direction returns 1 point.**
Call `support_face_points(cylinder, identity_pose, +X)`. Expect 1 point at
`(radius, 0, 0)` (on the curved surface, not a cap vertex).

**T3: Slightly-off-axis still returns cap points.**
Call with direction at 3 degrees from -Z axis. Should still trigger cap
enumeration (within the 5.7-degree threshold). Expect 8 points.

**T4: 45-degree direction returns 1 point.**
Call with direction `normalize(1, 0, -1)` (45 degrees from axis). Should
NOT trigger cap enumeration. Expect 1 point.

**T5: Rotated cylinder.**
Cylinder rotated 90 degrees about Y (axis now along X). Call with `-X`
direction. Should return 8 cap points all at `x = -half_length` (in world
frame), at distance `radius` from the cylinder axis.

**T6: Top cap vs bottom cap.**
Call with `+Z` → 8 points at `z = +half_length`.
Call with `-Z` → 8 points at `z = -half_length`.

### Integration test (existing, un-ignore)

**T10: Un-ignore `mesh_cylinder_settling`.**
Remove `#[ignore]` from the integration test. With MULTICCD + cylinder face
enumeration, the cylinder should produce 8 contacts on the cap face,
providing rotational stability. The test verifies rest height ≈ 0.2 and
force/weight ≈ 1.0.

---

## Session plan

| Session | Scope | Entry | Exit |
|---------|-------|-------|------|
| 1 | Add constants + `Shape::Cylinder` arm in `support_face_points()`. Unit tests T1–T6. | This spec | T1–T6 pass. Existing tests pass (no regressions). |
| 2 | Un-ignore T10 integration test. Final `cargo test -p sim-core -p sim-conformance-tests`. | Session 1 | T10 passes. All tests pass. No regressions. |

---

## Key files

| File | Role |
|------|------|
| `sim/L0/core/src/gjk_epa.rs` | `support_face_points()` — add Cylinder arm + constants |
| `sim/L0/core/src/collision/narrow.rs` | `multiccd_contacts()` — caller (read-only, no changes needed) |
| `sim/L0/tests/integration/mesh_cylinder_ellipsoid.rs` | Un-ignore T10 |

---

## Risk

**Low.** The change adds a new match arm to `support_face_points()` — no
existing arms are modified. The `_ =>` catch-all continues to handle all
other smooth shapes (Sphere, Capsule, Ellipsoid). The new arm only activates
when MULTICCD is enabled AND the contact direction aligns with the cylinder
axis. Non-MULTICCD paths are completely unaffected.

The cap enumeration geometry is trivial (evenly-spaced points on a circle).
The only tuning parameters are `N_CAP_POINTS` (8) and `CAP_FACE_THRESHOLD`
(10.0), both of which have clear geometric justifications and wide stable
ranges.
