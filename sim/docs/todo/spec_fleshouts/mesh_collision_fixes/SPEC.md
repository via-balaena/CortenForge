# DT-179 + DT-180: Mesh Collision Fixes

**Status:** Open
**Source:** cf-design Phase 5 (Session 26) — discovered while building the
design-through-physics optimization loop.
**Tier:** T2 (grouped spec — two bugs in the same subsystem)
**Blocking:** cf-design contact-force-based design optimization. Once fixed,
the Phase 5 integration test can be upgraded from volume objective to actual
contact force with a one-line closure change.

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
constraint solver pipeline. The `collide_mesh_plane` function computes
penetration depth correctly (vertex projection onto plane). The divergence
must be in how the solver uses mesh-derived properties.

Note: `solref`/`solimp` are applied identically for mesh and primitive
contacts (confirmed in `contact_assembly.rs`), so solver impedance parameters
are not the differentiator. The root cause is almost certainly in **mesh
mass/inertia computation**.

### Primary suspect: mesh inertia computation

Body mass and inertia are computed during model building in
`sim/L0/mjcf/src/builder/mass.rs` (`compute_inertia_from_geoms()`), NOT at
runtime. Mesh inertia has **4 computation modes** dispatched by
`compute_mesh_inertia_by_mode()`:

- **Exact** — signed tetrahedron decomposition (Mirtich 1996)
- **Legacy** — absolute volume computation
- **Shell** — surface-area-weighted
- **ConvexHull** — inertia of convex hull

If the wrong mode is selected, or if mesh vertex scaling is not correctly
propagated to volume integrals (mass ∝ scale³, not scale), the computed mass
could be wildly wrong. Since the constraint solver uses the effective mass at
the contact point (via `diagApprox = J · M⁻¹ · Jᵀ`), wrong mass → wrong
diagApprox → forces blow up proportionally.

### Investigation plan

1. **Compare mass/inertia** — print `model.body_mass[i]` and
   `model.body_inertia[i]` for a mesh sphere vs a primitive sphere of
   equivalent size and density. If mass is wrong, this is the root cause.
2. **Trace mesh inertia computation** — follow the path from
   `compute_mesh_inertia_by_mode()` through the selected mode (Exact/Legacy/
   Shell/ConvexHull). Check:
   - Is mesh scale applied as scale³ for volume/mass (not linear)?
   - Is the volume integral correct for the mesh topology?
   - Does the selected mode match expectations for the mesh type?
3. **Compare Contact structs** (secondary) — dump `Contact` produced by
   primitive-plane vs mesh-plane. Check `depth`, `dim`, `mu`,
   `includemargin`. Useful for ruling out contact-generation bugs, but likely
   not the root cause given identical `solref`/`solimp` handling.
4. **Fix** the root cause. If mass is wrong, fix the inertia computation.
   `diagApprox` and forces will correct automatically.
5. **Validate**: mesh sphere on plane produces contact force ≈ weight
   (within 5% at steady state, averaged over last 100 of 500 steps).

### Key files

- `sim/L0/mjcf/src/builder/mass.rs` — `compute_inertia_from_geoms()`, mesh inertia modes
- `sim/L0/mjcf/src/builder/mesh.rs` — mesh loading, vertex scale application, `compute_mesh_inertia_by_mode()`
- `sim/L0/core/src/collision/mesh_collide.rs:258-296` — `collide_mesh_plane`
- `sim/L0/core/src/collision/narrow.rs:445-487` — `make_contact_from_geoms`
- `sim/L0/core/src/constraint/impedance.rs` — `compute_diag_approx_exact` (downstream of mass)
- `sim/L0/core/src/constraint/contact_assembly.rs` — contact constraint row assembly

---

## Session plan

| Session | Scope | Entry | Exit |
|---------|-------|-------|------|
| 1 | Profile DT-179: bracket broadphase/narrow/assembly/solver, verify AABB bloat hypothesis | This spec | Hot path identified with timing data; mesh AABB behavior documented |
| 2 | Diagnose DT-180: compare mesh vs primitive mass/inertia, trace inertia computation | This spec (independent of Session 1) | Root cause identified; `model.body_mass` comparison for mesh vs primitive sphere documented |
| 3 | Fix DT-179 | Session 1 | Mesh-plane step <1ms (release); broadphase pair count matches primitives |
| 4 | Fix DT-180 | Session 2 | Mesh contact force within 5% of weight at steady state |
| 5 | cf-design integration + spec completion | Sessions 3+4 | Phase 5 test upgraded to contact-force objective; `cargo test -p cf-design` passes; `CF_DESIGN_SPEC.md` Session 26 updated (blocked status removed, full exit criteria met) |

---

## Validation test (Session 5)

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

## Full circle: CF_DESIGN_SPEC.md completion

After Session 5 passes, update `CF_DESIGN_SPEC.md` Session 26:

1. Remove the "Blocked" paragraph and deferred exit criterion.
2. Change the integration test description from "mesh volume" to
   "contact force through simulation."
3. Update the exit criterion to the original spec language:
   "Full Phase 5 exit criteria met: a parameterized gripper finger can be
   optimized to maximize grasp force by backpropagating through the
   simulation."
4. Mark Phase 5 fully complete — no remaining caveats.
