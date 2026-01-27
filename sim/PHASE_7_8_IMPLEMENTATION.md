# Phase 7-8 Implementation: Todorov Quality Standard

> **Origin**: This document consolidates PHASE_7_COMPLETION_PLAN.md, PHASE_7_TODOROV_SPEC.md,
> and PHASE_8_CLEANUP_ITEMS.md into a single iterative implementation plan.
>
> **Return to**: [CONSOLIDATION_PLAN.md](./CONSOLIDATION_PLAN.md) after completing Phase 8.
>
> **Last Updated**: 2026-01-26

---

## Overview

The Model/Data architecture is structurally complete (Phases 1-6). This document tracks
progress toward MuJoCo-quality implementations.

| Component | Tests Pass? | Todorov Quality? | Status |
|-----------|-------------|------------------|--------|
| FK (mj_fwd_position) | ✓ | ✓ | **DONE** |
| CRBA (mass matrix) | ✓ | ✓ | **DONE** - O(n) with spatial force transform |
| RNE (bias forces) | ✓ | ✓ | **DONE** - O(n) analytical Featherstone (gravity + Coriolis) |
| Joint limits | ✓ | ✓ | **DONE** |
| Contact detection | ✓ | ✓ | **DONE** - Spatial hashing + analytical primitives |
| Contact forces | ✓ | ✓ | **DONE** - PGS with Coulomb friction |
| URDF loading | ✓ | ✓ | **DONE** - Single path via MJCF |
| Performance | ✓ | ⚠️ | **PARTIAL** - Release OK, debug below target |

---

## Iteration 1: Collision Detection (mj_collision) - COMPLETE

**Status**: ✅ All major items implemented

### 1.1 Broad-Phase Spatial Hashing - DONE

Implemented in `mujoco_pipeline.rs`:
- `SpatialHash` struct with configurable cell size
- O(n) insertion, O(n + k) query for k candidate pairs
- Automatic fallback to O(n²) for small scenes (< 16 geoms)

### 1.2 Analytical Primitives - DONE

All high-priority primitives implemented:

| Pair | Status | Implementation |
|------|--------|----------------|
| sphere-plane | ✅ | `collide_with_plane()` |
| sphere-sphere | ✅ | `collide_sphere_sphere()` |
| sphere-capsule | ✅ | `collide_sphere_capsule()` |
| sphere-box | ✅ | `collide_sphere_box()` |
| capsule-capsule | ✅ | `collide_capsule_capsule()` |
| capsule-plane | ✅ | `collide_with_plane()` |
| capsule-box | ✅ | `collide_capsule_box()` |
| box-box | ✅ | `collide_box_box()` - SAT with 15 axes |
| box-plane | ✅ | `collide_with_plane()` |

Remaining pairs (cylinder, ellipsoid, mesh) fall back to GJK/EPA.

### 1.3 Collision Filtering - DONE

Implemented in `check_collision_affinity()`:
- Same-body filtering
- Parent-child filtering (with world body exception)
- contype/conaffinity bitmask support

### 1.4 Success Criteria

- [x] Broad-phase reduces candidate pairs by >90% for typical scenes
- [x] All analytical primitives implemented for sphere/capsule/box
- [ ] Debug mode >1000 steps/sec (currently ~50)
- [x] No special cases for world body in hot loop

---

## Iteration 2: Contact Force Application (mj_fwd_constraint) - COMPLETE

**Status**: ✅ All items implemented

### 2.1 Contact Jacobian Computation - DONE

Implemented in `pgs_solve_contacts()`:
- Computes contact Jacobian by walking kinematic tree
- Supports all joint types (hinge, slide, ball, free)

### 2.2 PGS Solver with Coulomb Friction - DONE

Implemented in `PGSSolver`:
- Gauss-Seidel iteration with constraint projection
- Normal force: λ_n >= 0
- Friction cone: |λ_t| <= μ * λ_n
- Configurable iterations and tolerance

### 2.3 Success Criteria

- [x] Contact forces work for articulated bodies
- [x] Jacobian transpose method used (qfrc_constraint = J^T * λ)
- [x] PGS with proper Coulomb friction cone
- [x] Ball stack stable via constraint physics

---

## Iteration 3: Performance (CRBA/RNE) - COMPLETE

**Status**: ✅ All core algorithms done, O(n) for all operations

### 3.1 Featherstone CRBA - DONE

Implemented in `mj_crba()`:
- Backward pass: composite inertia accumulation
- Forward pass: mass matrix via spatial force transform
- Configuration-dependent M[i,j] verified (varies with joint angles)

### 3.2 Recursive Newton-Euler - DONE

Implemented in `mj_rne()`:
- O(n) gravity using precomputed subtree mass/COM
- Gyroscopic terms for Ball/Free joints
- **O(n) Coriolis via analytical Featherstone RNE** (replaces numerical Christoffel)

The analytical RNE implementation:
1. **Velocity propagation** (`mj_fwd_velocity`): Correctly handles lever arm effect
   - `v[i] = X[i] @ v[parent] + S[i] @ qdot[i]`
   - Linear velocity includes `ω × r` term for body offset
2. **Bias acceleration forward pass**: `a_bias[i] = a_bias[parent] + v[parent] ×_m (S @ qdot)`
3. **Bias force backward pass**: `f_bias[i] = I[i] @ a_bias[i] + v[i] ×* (I[i] @ v[i])`
4. **Joint-space projection**: `τ = S^T @ f_bias`

All operations are O(n) with no system size limitations.

### 3.3 Success Criteria

- [x] CRBA is O(n) for tree-structured robots
- [x] RNE is O(n) for gravity
- [x] RNE Coriolis O(n) for all system sizes (no nv > 32 limitation)
- [x] Humanoid >1,000 steps/sec release mode (achieved: 5,393)
- [x] Pendulum >100,000 steps/sec release mode (achieved: 1,094,753)
- [ ] Debug mode >1,000 steps/sec (currently ~33-50)

---

## Iteration 4: Testing and Cleanup (Phase 8) - IN PROGRESS

### 4.1 Remove Band-Aids

| Item | Status |
|------|--------|
| Restore debug threshold to 1000 | ❌ TODO (requires perf optimization) |
| Remove old World API from sim-urdf | ❌ TODO |

### 4.2 Tests

| Test | Status |
|------|--------|
| Energy conservation (simple/double pendulum) | ✅ PASS |
| Coriolis validation (double pendulum) | ✅ PASS |
| CRBA analytical comparison | ✅ PASS |
| Ball stack contact stability | ✅ PASS |
| Humanoid loads and steps | ✅ PASS |
| 38 MuJoCo conformance tests | ✅ PASS |

### 4.3 Cleanup Checklist

| Item | Priority | Status |
|------|----------|--------|
| Restore debug threshold to 1000 | Critical | TODO |
| Remove old World API from sim-urdf | Critical | TODO |
| Consolidate broad_phase.rs with mujoco_pipeline | Moderate | TODO |
| Add mesh collision support | Moderate | TODO |
| Release-mode benchmark CI job | Lower | TODO |

---

## Definition of Done

Phase 7/8 is complete **to Todorov's standards** when:

- [x] Broad-phase collision (spatial hashing for 16+ geoms)
- [x] Analytical primitives (sphere/capsule/box covered)
- [x] Jacobian transpose contact forces (works for articulated bodies)
- [x] PGS with Coulomb friction cone
- [x] Featherstone O(n) CRBA
- [x] O(n) gravity in RNE
- [x] O(n) Coriolis for all systems (analytical Featherstone RNE)
- [ ] Debug threshold 1,000 steps/sec
- [x] Humanoid with ground contact stable
- [x] No special cases in hot paths
- [x] All tests pass (no `#[ignore]`)

**Current**: 10/11 criteria met. Only debug performance remains.

---

## Remaining Work

### High Priority

1. **Debug mode performance**
   - Profile hot paths
   - Consider SIMD for matrix operations

### Medium Priority

3. **Consolidate collision systems** - DONE
   - Single `CollisionShape` in `collision_shape.rs`
   - `world.rs` re-exports from canonical source
   - No duplication between world and collision_shape modules

4. **Remove old World API** - IN PROGRESS
   - Legacy World/Body/Joint marked deprecated
   - CollisionShape unified
   - sim-urdf/sim-mjcf still use deprecated World API (functional but marked for removal)

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
