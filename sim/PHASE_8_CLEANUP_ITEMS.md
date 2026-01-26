# Phase 8 Cleanup Items: Todorov-Aligned Refactoring

> **Principle**: If it's not how Todorov/MuJoCo would do it, fix it.
>
> This document tracks technical debt and misalignments identified during Phase 7 completion.
> These should be addressed before adding new features.

## Critical Fixes (Must Do)

### 1. Performance Test Thresholds

**Problem**: Debug mode threshold was lowered to 50 steps/sec to pass CI after contact physics was added.

**Current State** (`validation.rs:1299-1302`):
```rust
#[cfg(debug_assertions)]
let min_threshold = 50.0; // Debug: very slow due to collision detection
#[cfg(not(debug_assertions))]
let min_threshold = 1_000.0; // Release: should be fast
```

**What Todorov Would Do**:
- Add proper broad-phase collision detection (spatial hashing, BVH)
- Skip collision pairs that can't possibly collide (distance > sum of bounding radii)
- The test threshold should not need to be lowered

**Fix Required**:
1. Implement AABB broad-phase in `mj_collision()`
2. Add spatial hashing for O(n) average case instead of O(n²)
3. Restore threshold to 1,000+ steps/sec in debug mode

### 2. Old World API Still Exists

**Problem**: `sim-urdf/src/loader.rs` uses the old `World`, `Body`, `Joint` API alongside the new `Model/Data` API.

**What Todorov Would Do**:
- Single API. Model/Data is the truth. No parallel systems.

**Fix Required**:
1. Deprecate `LoadedRobot`, `SpawnedRobot`, `UrdfLoader`
2. Update all usages to use `load_urdf_model()` → `Model` → `Data`
3. Remove `sim-urdf/src/loader.rs` once all consumers migrated

### 3. Contact Force Application

**Problem**: Contact forces currently applied as direct accelerations to free joints only. Articulated bodies don't respond correctly.

**Current State** (`mujoco_pipeline.rs`):
```rust
// Only apply to free joints (first 6 DOFs are linear/angular)
if model.jnt_type[0] == MjJointType::Free {
    // Direct force application
}
```

**What Todorov Would Do**:
- Use Jacobian transpose to map contact forces to generalized forces
- `qfrc_constraint = J^T * λ` where λ are constraint forces
- Works for any joint type

**Fix Required**:
1. Compute contact Jacobian J for each contact
2. Solve for constraint forces λ
3. Apply `qfrc_constraint = J^T * λ`

## Moderate Priority

### 4. Collision Detection Primitive Coverage

**Problem**: Only sphere-plane, sphere-sphere, capsule-capsule, sphere-capsule implemented analytically. Other pairs fall back to GJK/EPA which is slow.

**Missing Pairs**:
- box-box (SAT)
- box-sphere
- box-capsule
- capsule-plane
- cylinder-* (all)

**Fix Required**: Implement analytical collision for all MuJoCo primitive pairs.

### 5. Parent-Child Collision Filtering

**Problem**: Current implementation has special case for world body:
```rust
if body1 != 0 && body2 != 0 {
    if model.body_parent[body1] == body2 || model.body_parent[body2] == body1 {
        continue;
    }
}
```

**What Todorov Would Do**:
- Use `contype`/`conaffinity` bitmasks for all filtering
- Pre-compute collision groups at model compile time
- No special cases in the hot loop

### 6. URDF Mesh Support

**Problem**: URDF→MJCF converter skips mesh geometries:
```rust
UrdfGeometry::Mesh { .. } => {
    // TODO: Add mesh asset support
    return Ok(());
}
```

**Fix Required**:
1. Generate MJCF `<asset><mesh>` elements
2. Reference meshes in `<geom>` elements
3. Handle scale parameter

## Lower Priority

### 7. Sparse CRBA/RNE

**Problem**: Current CRBA is O(n³) dense matrix operations.

**What Todorov Would Do**:
- Exploit kinematic tree sparsity
- O(n) composite rigid body inertias
- O(n) recursive Newton-Euler

### 8. Contact Solver Improvements

**Current**: Simple penalty-based contacts with fixed stiffness/damping.

**MuJoCo approach**:
- Constraint-based LCP solver
- Soft constraints with impedance parameters
- Warm-starting from previous solution

### 9. Simulation Parameter Inheritance

**Problem**: URDF converter uses hardcoded defaults:
```rust
timestep: 0.002,
gravity: Vector3::new(0.0, 0.0, -9.81),
solver_iterations: 100,
```

**Fix Required**: Allow configuration through conversion options or URDF extensions.

## Testing Debt

### 10. Release Mode Performance Validation

**Problem**: Performance tests only validate minimum thresholds. No CI check that release mode actually achieves spec targets.

**Spec Targets**:
- Simple pendulum: >100,000 steps/sec
- Humanoid (20+ DOF): >10,000 steps/sec

**Fix Required**: Add release-mode benchmark CI job.

---

## Tracking

| Item | Priority | Status | Assigned |
|------|----------|--------|----------|
| 1. Performance thresholds | Critical | TODO | - |
| 2. Old World API removal | Critical | TODO | - |
| 3. Contact force Jacobians | Critical | TODO | - |
| 4. Collision primitives | Moderate | TODO | - |
| 5. Collision filtering | Moderate | TODO | - |
| 6. URDF mesh support | Moderate | TODO | - |
| 7. Sparse CRBA/RNE | Lower | TODO | - |
| 8. Contact solver | Lower | TODO | - |
| 9. Parameter inheritance | Lower | TODO | - |
| 10. Release benchmarks | Lower | TODO | - |
