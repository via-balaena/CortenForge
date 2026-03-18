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

The mesh has ~50-100 vertices. `collide_mesh_plane` is O(n) per vertex.
50 vertex-plane distance checks should take <1µs, not 120ms. The overhead
is elsewhere — likely in broadphase pair enumeration, constraint assembly,
or solver setup that scales poorly with mesh geoms.

### Investigation plan

1. **Profile** a single `step()` with mesh + plane to find the hot path.
   Candidates:
   - Broadphase: is mesh-mesh pair checking running GJK/EPA even when
     AABBs don't overlap? (`narrow.rs` dispatch)
   - Constraint assembly: is `make_contact_from_geoms` doing expensive
     work for mesh contacts that it doesn't for primitives?
   - Solver: is the constraint solver iterating excessively due to mesh
     contact properties (e.g., poorly conditioned mass matrix from mesh
     inertia computation)?
2. **Fix** the hot path. Expected: mesh-plane step cost should be
   comparable to primitive-plane (~10-100µs, not 120ms).
3. **Validate** with the cf-design integration test: 500-step simulation
   with mesh geom + ground plane completes in <1s (release).

### Key files

- `sim/L0/core/src/collision/narrow.rs` — narrowphase dispatch
- `sim/L0/core/src/collision/mesh_collide.rs` — `collide_mesh_plane`
- `sim/L0/core/src/constraint/assembly.rs` — constraint row assembly

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

### Investigation plan

1. **Compare Contact structs** — dump the `Contact` produced by
   primitive-plane vs mesh-plane for equivalent geometry. Check:
   - `depth` (penetration) — should be similar
   - `solref`, `solimp` — solver impedance parameters
   - `dim`, `mu` — friction cone dimensionality
   - `includemargin` — margin handling
2. **Check body mass/inertia from mesh** — the constraint solver uses the
   effective mass at the contact point (from the body's mass matrix). If
   mesh inertia computation produces wildly wrong values, forces scale
   accordingly. Compare `model.body_mass[i]` and `model.body_inertia[i]`
   for a mesh sphere vs a primitive sphere of the same size/density.
3. **Check `diagApprox`** — the diagonal approximation of the constraint
   Jacobian inverse uses body mass. If mesh body mass is wrong, diagApprox
   is wrong, and forces blow up.
4. **Fix** the root cause (likely mass computation or solver parameter
   derivation for mesh geoms).
5. **Validate**: mesh sphere on plane produces contact force ≈ weight
   (within 5% at steady state).

### Key files

- `sim/L0/core/src/collision/mesh_collide.rs:258-296` — `collide_mesh_plane`
- `sim/L0/core/src/collision/narrow.rs:445-485` — `make_contact_from_geoms`
- `sim/L0/core/src/forward/position.rs` — body mass/inertia from geoms
- `sim/L0/mjcf/src/builder/mesh.rs` — mesh loading and scale application
- `sim/L0/core/src/constraint/assembly.rs` — constraint force computation

---

## Session plan

| Session | Scope | Entry | Exit |
|---------|-------|-------|------|
| 1 | Profile DT-179 + diagnose DT-180 root cause | This spec | Hot path identified; mesh Contact vs primitive Contact compared; mass/inertia values checked |
| 2 | Fix both bugs | Session 1 | Mesh-plane step <1ms (release); mesh contact force within 5% of weight at steady state |
| 3 | cf-design integration | Session 2 | Phase 5 test upgraded to contact-force objective; `cargo test -p cf-design` passes |

---

## Validation test (Session 3)

After DT-179 and DT-180 are fixed, update
`crates/cf-design/src/mechanism/integration.rs::phase5_parameterized_grasp_optimization`
to use actual contact force instead of mesh volume:

```rust
// Current (volume proxy):
let ball_mesh = mechanism.parts()[1].solid().mesh(2.0);
-ball_mesh.volume()

// Target (actual contact force through simulation):
let mut impulse = 0.0_f64;
for _ in 0..500 {
    data.step(&model).unwrap();
    impulse += data.efc_force.iter().map(|f| f.abs()).sum::<f64>();
}
-impulse
```

This completes the cf-design Phase 5 exit criterion: "a parameterized
gripper finger can be optimized to maximize grasp force by backpropagating
through the simulation."

---

## Full circle: CF_DESIGN_SPEC.md completion

After Session 3 passes, update `CF_DESIGN_SPEC.md` Session 26:

1. Remove the "Blocked" paragraph and deferred exit criterion.
2. Change the integration test description from "mesh volume" to
   "contact force through simulation."
3. Update the exit criterion to the original spec language:
   "Full Phase 5 exit criteria met: a parameterized gripper finger can be
   optimized to maximize grasp force by backpropagating through the
   simulation."
4. Mark Phase 5 fully complete — no remaining caveats.
