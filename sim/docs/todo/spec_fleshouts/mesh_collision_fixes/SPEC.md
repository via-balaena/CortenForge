# DT-179 + DT-180: Mesh Collision Fixes

**Status:** Complete (all 4 sessions done).
**Source:** cf-design Phase 5 (Session 26) — discovered while building the
design-through-physics optimization loop.
**Tier:** T2 (grouped spec — two bugs in the same subsystem)
**Resolved:** DT-179 (mesh AABB bloat) and DT-180 (inverted mesh-plane normal)
fixed. Phase 5 integration test upgraded from volume proxy to contact-force
objective. `docs/CF_DESIGN_SPEC.md` Session 26 fully complete.

---

## Overview

Two mesh collision bugs that together prevent mesh-based simulation from
producing usable contact forces. Primitive collisions (sphere-plane,
capsule-plane, etc.) work correctly — these bugs are specific to mesh geom
collision paths.

---

## DT-179: Mesh-plane collision performance

### Problem

Adding a ground plane (`<geom type="plane"/>`) to a scene with mesh geoms
makes each `step()` call ~40,000× slower (~120ms vs ~3µs without the plane).

### Evidence

```
# Timing test (release mode, sphere(1) mesh + sphere(0.5) mesh, free joint)
Without ground plane: 500 steps in 1.5ms  (3µs/step)
With ground plane:    500 steps in >60s   (~120ms/step)
```

The mesh has ~50-100 vertices. `collide_mesh_plane` is O(V) per call and
returns only **1 contact** (the deepest penetrating vertex). 50 vertex-plane
distance checks should take <1µs. The overhead is elsewhere.

### Primary suspect: mesh AABB bloat

Mesh geom AABBs use a fixed 10m extent fallback (`MESH_DEFAULT_EXTENT: f64 =
10.0` in `forward/position.rs`). This means every mesh geom has a massive
bounding box, causing the sweep-and-prune broadphase to generate false-positive
candidate pairs with every other geom in the scene. Adding a ground plane
(infinite extent) guarantees mesh-plane broadphase hits, but the 10m mesh AABBs
also cause mesh-mesh pairs to always overlap — potentially triggering expensive
GJK/EPA narrowphase on geom pairs that are nowhere near each other.

### Investigation plan

1. **Profile** a single `step()` with mesh + plane using `std::time::Instant`
   bracketing around broadphase, narrowphase, constraint assembly, and solver.
   Confirm which phase dominates.
2. **Check mesh AABB computation** — verify the 10m fallback is being used
   instead of tight-fitting AABBs computed from actual mesh vertex positions.
   If so, compute proper AABBs from transformed mesh vertices.
3. **Check broadphase pair count** — count candidate pairs with/without the
   plane. If bloated AABBs are producing O(n²) false positives, fixing the
   AABB computation resolves the issue.
4. **If broadphase is not the bottleneck**, check:
   - Narrowphase: is GJK/EPA being invoked for mesh-mesh pairs that should
     be rejected by AABB? (`narrow.rs` dispatch). Note: all mesh collision
     paths (mesh-plane, mesh-sphere, mesh-capsule, mesh-box, mesh-mesh)
     return at most **1 contact** (deepest vertex only), so constraint row
     explosion from contact count is ruled out.
   - Narrowphase cost: GJK/EPA on mesh convex hulls may itself be expensive
     if the hull has many vertices.
   - Solver: is the constraint solver doing expensive per-step work
     (e.g., factorization) that scales with geom count?
5. **Fix** the hot path. Expected: mesh-plane step cost should be comparable
   to primitive-plane (~10-100µs, not 120ms).
6. **Validate** with the cf-design integration test: 500-step simulation with
   mesh geom + ground plane completes in <1s (release).

### Key files

- `sim/L0/core/src/forward/position.rs` — mesh AABB computation (`MESH_DEFAULT_EXTENT`, `aabb_from_geom`)
- `sim/L0/core/src/collision/narrow.rs` — narrowphase dispatch (`collide_geoms`)
- `sim/L0/core/src/collision/mesh_collide.rs` — `collide_mesh_plane` (returns 1 contact)
- `sim/L0/core/src/constraint/assembly.rs` — constraint row counting and dispatch
- `sim/L0/core/src/constraint/contact_assembly.rs` — per-contact constraint row assembly
- `sim/L0/core/src/constraint/impedance.rs` — `compute_diag_approx_exact` (diagonal approximation)

---

## DT-180: Mesh-plane contact force magnitude

### Problem

Mesh-plane contact forces are ~10^16× too large compared to
primitive-plane contacts of equivalent geometry.

### Evidence

```
# Primitive sphere (radius=0.5, density=1000) on plane at Z=0:
#   mass = 524 kg, weight = 5140 N
#   Measured contact force: 5137 N  ✓ CORRECT

# Mesh sphere (radius=0.05, density=1250) on plane at Z=0:
#   mass ≈ 0.65 kg, weight ≈ 6.4 N
#   Measured contact force: 2.52e17 N  ✗ ~4e16× TOO LARGE
```

Both paths use the same `make_contact_from_geoms` → `Contact` struct →
constraint solver pipeline. The constraint assembly is completely uniform —
no mesh-specific logic in Jacobian computation, diagApprox, or impedance.

### Root cause (Session 2 finding)

**Inverted contact normal in the mesh-plane collision path.** Not mass/inertia.

Mesh mass is 87.3% of analytic sphere mass (expected icosphere approximation
error for 42 vertices). The original hypothesis (mesh inertia computation) was
ruled out — `body_mass` and `body_inertia` are correct to within geometric
approximation tolerance.

The bug is in `sim/L0/core/src/collision/mesh_collide.rs`, function
`collide_with_mesh()`. The `(prim_type, GeomType::Mesh)` branch (lines
189-229) negates the `MeshContact.normal` returned by all mesh collision
functions, assuming the convention "normal points FROM the mesh surface
outward." This is correct for `mesh_sphere_contact`, `mesh_capsule_contact`,
`mesh_box_contact`, and `mesh_mesh_deepest_contact`. But `collide_mesh_plane()`
returns `normal = plane_normal` (+Z for a Z=0 plane), which points AWAY FROM
THE PLANE, not from the mesh. The negation flips it to -Z, causing the
constraint force to push the ball downward into the plane instead of
supporting it.

**Effect (observed in diagnostic test):**
1. Contact normal = -Z → solver can't produce upward force (unilateral)
2. Ball free-falls through the plane (efc_force = 0 for first ~40 steps)
3. Increasing penetration depth eventually triggers non-zero force in -Z
4. Force in wrong direction accelerates the fall → exponential divergence
5. Step 100: 7.6e4 N → step 200: 5.4e6 N → step 400: 2.4e10 N

### Fix (Session 3 — complete)

Changed `collide_mesh_plane()` to return `normal = -plane_normal` ("from mesh
outward," consistent with all other mesh collision functions). Two-line change
(both assignment sites in the function).

- `(Plane, Mesh)` branch: negation of `-plane_normal` → `+plane_normal` ✓
- `(Mesh, Plane)` branch: no negation → `-plane_normal` ✓

Both match the `Contact.normal` convention: "from geom1 toward geom2."

Diagnostic test upgraded to assertion tests:
- `dt180_mesh_contact_force_plane_mesh_order`: force/weight = 1.0000, normal = +Z
- `dt180_mesh_contact_force_mesh_plane_order`: force/weight = 1.0000
- `dt180_mesh_mass_comparison`: mesh/analytic mass = 0.8735 (expected icosphere error)

### Key files

- `sim/L0/core/src/collision/mesh_collide.rs:258-296` — `collide_mesh_plane` (the fix)
- `sim/L0/core/src/collision/mesh_collide.rs:189-229` — `(prim_type, Mesh)` branch (normal negation)
- `sim/L0/core/src/collision/narrow.rs:445-487` — `make_contact_from_geoms`

---

## Session plan

| Session | Scope | Entry | Exit |
|---------|-------|-------|------|
| 1 | Profile + fix DT-179: identify AABB bloat, pre-compute `model.geom_aabb` | This spec | ✅ Mesh collision step1 within 1.5× of primitives; `MESH_DEFAULT_EXTENT` deleted |
| 2 | Diagnose DT-180: compare mesh vs primitive mass/inertia, identify root cause | This spec (independent of Session 1) | ✅ Root cause: inverted contact normal in `collide_mesh_plane`; `model.body_mass` comparison documented |
| 3 | Fix DT-180: flip `collide_mesh_plane` normal, upgrade diagnostic to assertion test | Session 2 | ✅ Mesh force/weight = 1.0000 for both (Plane,Mesh) and (Mesh,Plane); 2,221 tests pass |
| 4 | cf-design integration + spec completion | Sessions 1+3 | ✅ Phase 5 test upgraded to contact-force objective; `cargo test -p cf-design` passes; `docs/CF_DESIGN_SPEC.md` Session 26 updated (blocked status removed, full exit criteria met) |

---

## Validation test (Session 4)

After DT-179 and DT-180 are fixed, update
`crates/cf-design/src/mechanism/integration.rs::phase5_parameterized_grasp_optimization`
to use actual contact force instead of mesh volume:

```rust
// Current (volume proxy):
let ball_mesh = mechanism.parts()[1].solid().mesh(2.0);
-ball_mesh.volume()

// Target (steady-state contact force through simulation):
let settle_steps = 200;
let measure_steps = 300;
for _ in 0..settle_steps {
    data.step(&model).unwrap();
}
let mut contact_force = 0.0_f64;
for _ in 0..measure_steps {
    data.step(&model).unwrap();
    // Sum only contact constraint forces, excluding joint limits,
    // equality constraints, friction loss, etc.
    for (i, ct) in data.efc_type.iter().enumerate() {
        if matches!(ct,
            ConstraintType::ContactFrictionless
            | ConstraintType::ContactPyramidal
            | ConstraintType::ContactElliptic
        ) {
            contact_force += data.efc_force[i].abs();
        }
    }
}
-(contact_force / measure_steps as f64)
```

Key differences from the naive version:
1. **Settling period** (200 steps) — avoids transient impact forces dominating
2. **Steady-state averaging** (last 300 steps) — measures sustained grasp force
3. **Contact-only filtering** — uses `data.efc_type` to select only
   `ContactFrictionless`, `ContactPyramidal`, and `ContactElliptic` rows;
   excludes `Equality`, `FrictionLoss`, `LimitJoint`, `LimitTendon`, and
   `FlexEdge` rows

This completes the cf-design Phase 5 exit criterion: "a parameterized
gripper finger can be optimized to maximize grasp force by backpropagating
through the simulation."

---

## Full circle: docs/CF_DESIGN_SPEC.md completion

After Session 4 passes, update `docs/CF_DESIGN_SPEC.md` Session 26:

1. Remove the "Blocked" paragraph and deferred exit criterion.
2. Change the integration test description from "mesh volume" to
   "contact force through simulation."
3. Update the exit criterion to the original spec language:
   "Full Phase 5 exit criteria met: a parameterized gripper finger can be
   optimized to maximize grasp force by backpropagating through the
   simulation."
4. Mark Phase 5 fully complete — no remaining caveats.
