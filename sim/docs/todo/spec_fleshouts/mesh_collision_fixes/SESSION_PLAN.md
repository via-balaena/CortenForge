# Mesh Collision Fixes — Session Plan

Source spec: `SPEC.md` (DT-179 + DT-180)

## Session 1: Profile + Fix DT-179 — Mesh AABB Bloat ✅

**Status:** Complete
**Test:** `sim/L0/tests/integration/mesh_collision_profile.rs`

### Root cause

`aabb_from_geom()` was a runtime type-dispatch function that reconstructed
bounding boxes from `(geom_type, size, pos, mat)`. For meshes, `size` doesn't
encode mesh geometry, so it used `MESH_DEFAULT_EXTENT = 10.0` — a 20m³
bounding box for every mesh geom regardless of actual size.

### Fix

Pre-computed local-frame AABBs for all geoms at build time, stored as
`model.geom_aabb: Vec<[f64; 6]>` (center offset + half-extents). At runtime,
`mj_collision` transforms these to world space via the standard rotated-box
formula — no type dispatch, no mesh vertex iteration.

- `model.geom_aabb` populated in `compute_geom_bounding_radii()` alongside
  `geom_rbound`, using the same mesh/hfield/sdf/primitive dispatch
- `aabb_from_geom_aabb()` replaces `aabb_from_geom()` — uniform for all types
- `MESH_DEFAULT_EXTENT` deleted
- Matches MuJoCo's `mjModel.geom_aabb` architecture

### Results (release, Apple M-series)

Collision phase (step1) with N well-separated mesh geoms + plane:

| N | Before (mesh step1) | After (mesh step1) | Before ratio | After ratio |
|---|---|---|---|---|
| 5 | 28.9µs | 4.3µs | 13.8× prim | 1.5× prim |
| 10 | 77.8µs | 6.9µs | 15.6× prim | 1.3× prim |
| 15 | 124.6µs | 12.5µs | 13.3× prim | 1.2× prim |

False broadphase pairs eliminated. Mesh collision cost now matches primitives.

---

## Session 2: Diagnose DT-180 — Mesh Contact Force Magnitude ✅

**Status:** Complete
**Test:** `sim/L0/tests/integration/mesh_contact_force_diagnostic.rs`

### Root cause

**Inverted contact normal in the mesh-plane collision path.**

The spec's primary suspect (mesh inertia/mass computation) was ruled out:
mesh body_mass is 87.3% of analytic sphere mass — expected error for
icosphere subdivision 1 (42 vertices), not a 10^16× scaling bug.

The actual root cause is in `sim/L0/core/src/collision/mesh_collide.rs`,
the `(prim_type, GeomType::Mesh)` branch (lines 189-229). When the plane
is `geom1` and mesh is `geom2`, the code negates the `MeshContact.normal`
(line 225-228). But `collide_mesh_plane()` returns `normal = plane_normal`
(+Z, pointing away from the plane surface), while other mesh collision
functions (`mesh_sphere_contact`, `mesh_capsule_contact`, etc.) return
normals pointing FROM the mesh surface outward. The negation assumes the
latter convention, so it flips the correct +Z to an incorrect -Z.

**Effect:**
- Contact normal = -Z → constraint force pushes ball DOWNWARD (into plane)
- Solver initially sets force=0 (can't help with inverted normal)
- Ball free-falls through the plane, accumulating penetration depth
- Eventually the large depth triggers non-zero force in the wrong direction
- Exponential divergence: step 100: 7.6e4 N → step 400: 2.4e10 N

**Comparison (primitive vs mesh, r=0.5, ρ=1000):**

| Property | Primitive | Mesh (sub1) | Ratio |
|---|---|---|---|
| body_mass | 523.60 kg | 457.34 kg | 0.87× |
| body_inertia | 52.36 | 41.82 | 0.80× |
| Contact normal | [0,0,+1] ✓ | [0,0,-1] ✗ | inverted |
| Steady-state force | 5136 N ✓ | 4.2e10 N ✗ | 8.2e6× |

### Fix (Session 3)

Change `collide_mesh_plane()` to return `normal = -plane_normal`
("from mesh outward", consistent with other mesh collision functions).
Then the existing negation in `(Plane, Mesh)` produces `+plane_normal` ✓,
and the `(Mesh, Plane)` path (no negation) gets `-plane_normal` ✓.

---

## Session 3: Fix DT-180 — Flip `collide_mesh_plane` Normal ✅

**Status:** Complete
**Test:** `sim/L0/tests/integration/mesh_contact_force_diagnostic.rs`

### Fix

Changed `collide_mesh_plane()` in `mesh_collide.rs` to return
`normal = -plane_normal` (from mesh outward, consistent with all other mesh
collision functions). Two-line change (both assignment sites in the function).

### Test upgrade

Replaced the single diagnostic test `dt180_mesh_vs_primitive_contact_force`
(eprintln-only) with two assertion tests:

- `dt180_mesh_contact_force_plane_mesh_order` — (Plane, Mesh) geom ordering:
  asserts force/weight within 5%, contact normal +Z
- `dt180_mesh_contact_force_mesh_plane_order` — (Mesh, Plane) geom ordering:
  asserts force/weight within 5%
- `dt180_mesh_mass_comparison` — retained (mass reference data with assertions)

### Results

Both orderings: force/weight ratio = 1.0000 (exact match at steady state).
Full regression: 2,221 tests pass (sim-core + sim-mjcf + sim-conformance-tests).

---

## Session 4: cf-design Integration + Spec Completion ✅

**Status:** Complete

### Changes

- `phase5_parameterized_grasp_optimization` test rewritten: objective changed
  from `-ball_mesh.volume()` to steady-state contact force measured via
  `efc_type`/`efc_force` filtering (200 settle + 300 measurement steps).
  Mechanism rebuilt per evaluation with anchor Z tracking ball radius for
  consistent ground clearance.
- `sim-core` added as dev-dependency to `cf-design` (for `ConstraintType`).
- `docs/CF_DESIGN_SPEC.md` Session 26 updated: removed "Blocked" paragraph,
  replaced volume proxy with contact force description, updated exit criterion.
- 696 cf-design tests pass, 2,221 sim tests pass.
