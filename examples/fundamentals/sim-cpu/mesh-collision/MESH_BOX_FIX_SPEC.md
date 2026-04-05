# Mesh-Box Collision Fix Spec

## Bug

`triangle_box_contact()` (`sim/L0/core/src/mesh.rs:748-824`) is
algorithmically incomplete. It only tests triangle **vertices** clamped to the
box interior. When a box rests flat on a triangle face — the most common
case — all 3 vertices are outside and below the box, producing zero contacts.
The box falls through the mesh with no collision.

Stress test result: box dropped onto mesh ground slab — z=-43.8 (pure
freefall for 3 seconds, zero contacts generated).

## Root Cause

The narrowphase `triangle_box_contact()` has:
- A correct SAT broad-phase test (`aabb_triangle_intersect`, lines 827-883)
- An incomplete narrow-phase (lines 773-813) that only checks:
  - Triangle vertex inside box → face-distance penetration
  - Triangle vertex outside but near box → clamped-distance check

**Missing cases:**
- Box corner/edge penetrating triangle face (the resting case)
- Triangle edge intersecting box face
- Closest-point-on-triangle to closest-point-on-box

By contrast, `triangle_sphere_contact()` works because it calls
`closest_point_on_triangle()` — a complete geometric operation.

## Fix: Route Mesh-Box Through GJK/EPA

Replace the broken BVH triangle-level path with GJK/EPA on the mesh's convex
hull, matching the pattern already used for mesh-cylinder and mesh-ellipsoid.

### Why GJK/EPA instead of fixing triangle_box_contact

1. **Proven path** — GJK/EPA already works for cylinder, ellipsoid, and
   mesh-mesh pairs on this branch. Shape::Box is fully supported in
   `gjk_epa.rs` with support functions and face enumeration.

2. **MULTICCD support** — flat box face on flat mesh face gets multiple
   contact points automatically (same fix that stabilized cylinder caps).

3. **Simpler** — 6 lines of dispatch change vs. a full SAT contact-manifold
   rewrite of `triangle_box_contact()`.

4. **MuJoCo conformant** — MuJoCo treats all meshes as convex for collision.
   Using the convex hull is the correct behavior.

### What we lose

BVH triangle-level accuracy for non-convex meshes. But:
- MuJoCo doesn't support non-convex mesh collision either
- The convex hull is always computed at build time for meshes with >= 4
  non-coplanar vertices
- Meshes without a hull (< 4 vertices) get a warning + no contacts, same
  as cylinder and ellipsoid paths

## Changes

### 1. `mesh_collide.rs` — Mesh(geom1) vs Box(geom2) dispatch

**File:** `sim/L0/core/src/collision/mesh_collide.rs:207`

Before:
```rust
GeomType::Box => mesh_box_contact(mesh, &pose1, &pose2, &size2, use_bvh),
```

After:
```rust
GeomType::Box => {
    // Exact box collision via GJK/EPA on mesh convex hull
    if let Some(hull) = mesh.convex_hull() {
        let shape1 = Shape::convex_mesh(hull.clone());
        let shape2 = Shape::Box { half_extents: size2 };
        return gjk_epa_shape_pair(
            model, &shape1, &pose1, &shape2, &pose2, geom1, geom2, margin,
        );
    }
    warn!("mesh-box collision requires convex hull; returning no contacts");
    return vec![];
}
```

### 2. `mesh_collide.rs` — Box(geom1) vs Mesh(geom2) dispatch (swapped order)

**File:** `sim/L0/core/src/collision/mesh_collide.rs:282`

Before:
```rust
GeomType::Box => mesh_box_contact(mesh, &pose2, &pose1, &size1, use_bvh),
```

After:
```rust
GeomType::Box => {
    // Exact box collision via GJK/EPA on mesh convex hull.
    // Arguments in geom-order: shape1=box (geom1), shape2=hull (geom2).
    if let Some(hull) = mesh.convex_hull() {
        let shape1 = Shape::Box { half_extents: size1 };
        let shape2 = Shape::convex_mesh(hull.clone());
        return gjk_epa_shape_pair(
            model, &shape1, &pose1, &shape2, &pose2, geom1, geom2, margin,
        );
    }
    warn!("mesh-box collision requires convex hull; returning no contacts");
    return vec![];
}
```

### 3. Dead code cleanup

`mesh_box_contact()`, `triangle_box_contact()`, `aabb_triangle_intersect()`,
and `test_axis_triangle_box()` in `mesh.rs` become unused. Remove all four
functions plus:

- **Test:** `mesh_box_brute_force_matches_bvh` (`mesh.rs:1964-1983`)
- **Public exports:** Remove `mesh_box_contact` and `triangle_box_contact`
  from `sim/L0/core/src/lib.rs:150-151`
- **Benchmark:** Remove `triangle_box_contact` import and benchmark loop
  from `sim/L0/core/benches/collision_benchmarks.rs:27,434-443`

### 4. MULTICCD required for face-face stability

Like mesh-mesh and mesh-cylinder, the GJK/EPA path returns a single contact
for flat face-on-face contact. MULTICCD must be enabled for stable stacking.
This is not a limitation of the fix — it's inherent to single-point GJK/EPA
and affects all GJK/EPA pairs with planar contact.

### 5. Stress test: all checks pass

After the fix, `example-mesh-collision-stress-test` reports 21/21 PASS.
Box rests at z=0.0499 (expect 0.05), force/weight=1.0000.

## Validation

1. Stress test: `cargo run -p example-mesh-collision-stress-test --release`
   — expect 21/21 PASS
2. Integration tests: `cargo test -p sim-conformance-tests` — no regressions
3. Existing mesh tests in sim-core: `cargo test -p sim-core mesh` — verify
   removed test doesn't break anything else
4. Clippy: `cargo clippy -p sim-core -- -D warnings` — clean

## Not in scope

- Fixing `triangle_box_contact()` itself (replaced, not repaired)
- Non-convex mesh support (not supported by MuJoCo either)
- New integration tests (stress test covers this)
