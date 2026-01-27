# Phase 7-8 Implementation: Todorov Quality Standard

> **Origin**: This document consolidates PHASE_7_COMPLETION_PLAN.md, PHASE_7_TODOROV_SPEC.md,
> and PHASE_8_CLEANUP_ITEMS.md into a single iterative implementation plan.
>
> **Return to**: [CONSOLIDATION_PLAN.md](./CONSOLIDATION_PLAN.md) after completing Phase 8.

---

## Overview

The Model/Data architecture is structurally complete (Phases 1-6). However, several implementations
use shortcuts that pass tests but don't match MuJoCo's approach:

| Component | Tests Pass? | Todorov Quality? | Current Issue |
|-----------|-------------|------------------|---------------|
| FK (mj_fwd_position) | ✓ | ✓ | None |
| CRBA (mass matrix) | ✓ | ✗ | O(n³) not O(n) |
| RNE (bias forces) | ✓ | ✗ | O(n²) not O(n) |
| Joint limits | ✓ | ✓ | None |
| Contact detection | ✓ | ✗ | O(n²) all-pairs, missing primitives |
| Contact forces | ✓* | ✗ | Penalty-based, free joints only |
| URDF loading | ✓ | ✓ | Single path via MJCF (correct) |
| Performance | ✓* | ✗ | Debug threshold lowered to 50 |

*Tests pass due to band-aids, not correct implementation.

---

## Iteration 1: Collision Detection (mj_collision)

**Goal**: O(n log n) broad-phase, analytical primitives, uniform filtering.

### 1.1 Add Broad-Phase Spatial Hashing

The current implementation checks all pairs O(n²). MuJoCo uses spatial hashing or sweep-and-prune.

```rust
struct SpatialHash {
    cell_size: f64,
    cells: HashMap<(i32, i32, i32), Vec<usize>>,
}

impl SpatialHash {
    fn insert(&mut self, geom_id: usize, aabb: &AABB) { ... }
    fn query_candidates(&self) -> Vec<(usize, usize)> { ... }
}
```

**Files to modify**:
- `sim-core/src/mujoco_pipeline.rs` - Add `mj_collision()` function

### 1.2 Implement Missing Analytical Primitives

| Pair | Priority | Algorithm |
|------|----------|-----------|
| sphere-plane | High | Point-plane distance |
| sphere-sphere | High | Center distance |
| sphere-capsule | High | Point-segment distance |
| sphere-box | High | Closest point on box |
| capsule-capsule | High | Segment-segment distance |
| capsule-plane | Medium | Segment-plane distance |
| capsule-box | Medium | Segment-box distance |
| box-box | Medium | SAT (15 axes) |
| box-plane | Medium | Vertex enumeration |

**Files to modify**:
- `sim-core/src/collision/primitives.rs` (new file)
- `sim-core/src/mujoco_pipeline.rs` - narrow_phase dispatch

### 1.3 Uniform Collision Filtering

Remove special cases for world body. Use contype/conaffinity uniformly:

```rust
fn check_collision_affinity(model: &Model, g1: usize, g2: usize) -> bool {
    let body1 = model.geom_body[g1];
    let body2 = model.geom_body[g2];

    // Same body - no collision
    if body1 == body2 { return false; }

    // Parent-child - no collision
    if model.body_parent[body1] == body2 || model.body_parent[body2] == body1 {
        return false;
    }

    // contype/conaffinity bitmask check
    let c1 = model.geom_contype[g1];
    let a1 = model.geom_conaffinity[g1];
    let c2 = model.geom_contype[g2];
    let a2 = model.geom_conaffinity[g2];

    (c1 & a2) != 0 || (c2 & a1) != 0
}
```

### 1.4 Success Criteria

- [ ] Broad-phase reduces candidate pairs by >90% for typical scenes
- [ ] All analytical primitives implemented (no GJK/EPA for primitives)
- [ ] Debug mode >1000 steps/sec
- [ ] No special cases for world body in hot loop

---

## Iteration 2: Contact Force Application (mj_fwd_constraint)

**Goal**: Constraint-based forces via Jacobian transpose, works for any joint type.

### 2.1 Contact Jacobian Computation

For a contact at world point p with normal n between bodies b1 and b2:

```rust
fn contact_jacobian(model: &Model, data: &Data, contact: &Contact) -> DMatrix<f64> {
    let nv = model.nv;
    let mut J = DMatrix::zeros(3, nv);  // normal + 2 tangent directions

    // Walk kinematic tree for each body, accumulating Jacobian contributions
    add_body_jacobian_contribution(&mut J, model, data, contact.body1, contact.pos, ...);
    add_body_jacobian_contribution(&mut J, model, data, contact.body2, contact.pos, ...);

    J
}
```

**Key insight**: J maps qvel → contact-frame velocity. J^T maps contact forces → qfrc_constraint.

### 2.2 PGS Solver with Coulomb Friction

```rust
fn mj_fwd_constraint(model: &Model, data: &mut Data) {
    // Build constraint system: A = J * M^{-1} * J^T
    // Solve: A * λ = b with friction cone bounds
    // Apply: qfrc_constraint = J^T * λ

    for _iter in 0..model.solver_iterations {
        for contact in &data.contacts {
            // Normal: λ_n >= 0
            // Friction: |λ_t| <= μ * λ_n
        }
    }
}
```

### 2.3 Success Criteria

- [ ] Contact forces work for articulated bodies (humanoid standing)
- [ ] Jacobian transpose method used (not direct acceleration)
- [ ] PGS with proper Coulomb friction cone
- [ ] Ball stack stable via constraint physics (not penalty)

---

## Iteration 3: Performance (CRBA/RNE)

**Goal**: O(n) algorithms using Featherstone's method.

### 3.1 Pre-compute Ancestor Data

At Model construction, compute and cache ancestor joint/DOF lists:

```rust
impl Model {
    pub fn compute_ancestors(&mut self) {
        self.body_ancestor_joints = vec![vec![]; self.nbody];
        for body in 1..self.nbody {
            // Walk to root, collecting joints
        }
    }
}
```

### 3.2 Featherstone CRBA

Replace O(n³) with composite rigid body algorithm:

```
// Backward pass: composite inertias
for body = n down to 1:
    Ic[parent] += transform(Ic[body])

// Forward pass: mass matrix columns
for body = 1 to n:
    F = Ic[body] * S
    M[dof, dof] = S^T * F
    propagate F to ancestors
```

### 3.3 Recursive Newton-Euler

Replace O(n²) with single-pass RNE:

```
// Forward pass: velocities/accelerations
for body = 1 to n:
    v[body] = transform(v[parent]) + S * qdot
    a[body] = transform(a[parent]) + S * qddot + v × (S * qdot)

// Backward pass: forces
for body = n down to 1:
    f[body] = I * a + v × (I * v)
    f[parent] += transform(f[body])
    tau[dof] = S^T * f[body]
```

### 3.4 Success Criteria

- [ ] CRBA is O(n) for tree-structured robots
- [ ] RNE is O(n) with no ancestor list rebuilding
- [ ] Humanoid >10,000 steps/sec release mode
- [ ] Pendulum >100,000 steps/sec release mode
- [ ] Debug mode >1,000 steps/sec (threshold restored)

---

## Iteration 4: Testing and Cleanup (Phase 8)

### 4.1 Remove Band-Aids

1. **Restore performance threshold** (`validation.rs`):
```rust
// Remove:
#[cfg(debug_assertions)]
let min_threshold = 50.0;

// Replace with uniform threshold:
let min_threshold = 1_000.0;
```

2. **Remove old World API** from sim-urdf:
   - Delete `sim-urdf/src/loader.rs`
   - Update all usages to `load_urdf_model()`

### 4.2 New Tests Required

```rust
#[test]
fn test_articulated_body_contact() {
    // Double pendulum hitting ground
    // Verifies J^T force propagation
}

#[test]
fn test_humanoid_standing_stability() {
    // Humanoid on ground plane
    // Verifies multi-contact constraint solving
}
```

### 4.3 Cleanup Checklist

| Item | Priority | Status |
|------|----------|--------|
| Restore debug threshold to 1000 | Critical | TODO |
| Remove old World API from sim-urdf | Critical | TODO |
| Add contype/conaffinity fields to Model | Moderate | TODO |
| Add mesh support to URDF converter | Moderate | TODO |
| Release-mode benchmark CI job | Lower | TODO |

---

## Definition of Done

Phase 7/8 is complete **to Todorov's standards** when:

- [ ] Broad-phase collision (no O(n²) all-pairs)
- [ ] Analytical primitives (no GJK fallback for sphere/capsule/box)
- [ ] Jacobian transpose contact forces (works for articulated bodies)
- [ ] PGS with Coulomb friction cone
- [ ] Featherstone O(n) CRBA
- [ ] Recursive O(n) RNE
- [ ] Debug threshold 1,000 steps/sec
- [ ] Humanoid with ground contact stable
- [ ] No special cases in hot paths
- [ ] All tests pass (no `#[ignore]`)

---

## After Completion

Return to [CONSOLIDATION_PLAN.md](./CONSOLIDATION_PLAN.md) Phase 8 for:
- Delete old World/Stepper code
- Merge sim-contact into sim-core
- Consolidate crate structure
- Final documentation

---

## References

- Featherstone, R. (2008). "Rigid Body Dynamics Algorithms"
- MuJoCo Documentation: Computation chapter
- Todorov, E. (2014). "Convex and analytically-invertible dynamics"
