# Simulation — Future Work

Objectives ordered by dependency chain and expected value. Each builds on the
previous where noted.

---

## 1. In-Place Cholesky Factorization

**Current state:** `mj_fwd_acceleration_implicit()` in `mujoco_pipeline.rs` uses
`std::mem::replace` to swap the scratch matrix out for Cholesky consumption. This
allocates O(nv²) per step for the replacement zero matrix — acceptable for small
models but wasteful at scale.

**Objective:** Implement an in-place Cholesky factorization that operates on
`&mut DMatrix<f64>` directly. Wrap LAPACK's `dpotrf` (which factorizes in place) or
implement a manual lower-triangular decomposition. The factored result overwrites
`scratch_m_impl` in place; a thin wrapper provides the `solve` interface.

**Impact:** Eliminates the last per-step allocation in the implicit integrator path.
For nv=100 DOFs this saves ~80 KB/step.

**Stretch goal:** For serial kinematic chains the mass matrix is banded. A sparse
L^T D L factorization (using the existing `qLD_diag` / `qLD_L` fields in `Data`)
would reduce factorization from O(nv³) to O(nv). Featherstone's ABA algorithm is
an equivalent O(nv) alternative.

**Files:** `sim/L0/core/src/mujoco_pipeline.rs` (~line 8351)

---

## 2. CGSolver Pipeline Integration

**Current state:** `CGSolver` (1,664 lines, Block Jacobi preconditioner, 13 tests) is
fully implemented in `sim-constraint/src/cg.rs` but has zero callers. The active
pipeline uses `pgs_solve_contacts()` in `mujoco_pipeline.rs`.

**Objective:** Add CGSolver as an alternative constraint solver selectable via
`Model::solver_type` or MJCF `<option solver="CG"/>`. PGS remains the default.

**Approach:**
1. Build an adapter mapping `mujoco_pipeline.rs` contact structures (`Contact`,
   `efc_J`, `efc_D`) to CGSolver's `Joint` trait interface.
2. Add `SolverType` enum (`PGS`, `CG`) to `Model`. Wire selection into
   `mj_fwd_constraint()`.
3. Benchmark against PGS on humanoid models (30–100 DOFs, 50–500 contacts).

**Trigger:** When scenes with 100+ simultaneous contacts show PGS convergence
degradation. CG with Block Jacobi preconditioning maintains stable iteration counts
where PGS degrades linearly with constraint count.

**Files:** `sim/L0/core/src/mujoco_pipeline.rs`, `sim/L0/constraint/src/cg.rs`,
`sim/L0/core/src/lib.rs` (new `SolverType`)

---

## 3. Higher-Order Integrators

**Current state:** Semi-implicit Euler is the only production integrator. RK4 and
Velocity Verlet exist in `sim-core/src/integrators.rs` but are standalone functions
not wired into the MuJoCo pipeline.

**Objective:** Add RK4 (4th-order Runge-Kutta) and implicit Euler as pipeline-integrated
integrators selectable via `Model::integrator`.

**Why it matters:** Higher-order methods improve accuracy for stiff systems — high
damping, high stiffness springs, and small timestep requirements. Implicit Euler is
unconditionally stable for arbitrarily stiff problems.

**Prerequisites:** In-place Cholesky (#1) benefits implicit Euler, which requires a
linear solve per step.

**Files:** `sim/L0/core/src/mujoco_pipeline.rs`, `sim/L0/core/src/integrators.rs`

---

## 4. Deformable Bodies

**Current state:** `sim-deformable` crate has XPBD solver, cloth, soft body, and
capsule chain implementations. Not yet integrated into the MuJoCo pipeline.

**Objective:** Integrate deformable bodies into the pipeline so rigid and soft bodies
interact through the same contact solver.

**Scope:**
- FEM-based soft bodies (tetrahedral meshes, Neo-Hookean material)
- Position-based dynamics cloth (triangle meshes, bending constraints)
- Coupling: deformable-rigid contact via the existing collision pipeline

**Use cases:** Soft tissue simulation, cloth/fabric, pneumatic actuators.

**Files:** `sim/L0/deformable/`, `sim/L0/core/src/mujoco_pipeline.rs`

---

## 5. Batched Simulation

**Current state:** Single-environment execution only. Each `Model`/`Data` pair runs
one simulation.

**Objective:** CPU-parallel multi-environment architecture for RL training. Step N
environments in parallel, return N observation tensors.

**Approach:**
- Thread-per-env execution via rayon (shared-nothing, no synchronization)
- SIMD-across-envs for batched math where applicable
- Batched API: `BatchData::step_all(&model)` → `Vec<Observation>`
- Validate memory layout and training loop integration on CPU before GPU port

**Why it matters:** Prerequisite to GPU acceleration. Thousands of parallel humanoid
environments are needed for policy learning; the batching architecture is independent
of the compute backend.

**Prerequisites:** None — can proceed in parallel with #1–#4.

---

## 6. GPU Acceleration

**Current state:** CPU-only.

**Objective:** wgpu backend for batch simulation. Thousands of parallel environments
on a single GPU for RL training at scale.

**Approach:** Port the inner loop of `Data::step()` (FK, collision, PGS, integration)
to compute shaders. The `sim-gpu` crate (`mesh/mesh-gpu`) provides the wgpu
infrastructure.

**Prerequisites:** Batched Simulation (#5) — the CPU batching API defines the memory
layout that the GPU backend fills.

**Use cases:** Large-scale RL training, real-time multi-agent simulation.

---
---

## Completed Work (Reference)

Historical context for decisions referenced above. Details collapsed to keep this
document focused on future objectives.

### Crate Consolidation

Three-phase effort to clean the sim dependency graph. Eliminated two crates
(sim-contact, sim-contact's solver infrastructure) and ~10,000 lines of dead code.

| Phase | Commit | Summary |
|-------|--------|---------|
| 1 — Remove phantom deps | `ba1d729` | Removed 4 phantom Cargo.toml deps (sim-mjcf, sim-urdf, sim-bevy, sim-tests → sim-constraint). Removed dead re-exports (`ContactSolverConfig`). Moved sim-constraint from sim-core deps to dev-deps. |
| 2 — Consolidate sim-contact | `9ed88ea` | Moved `ContactPoint`, `ContactForce`, `ContactManifold` into `sim-core/src/contact.rs`. Deleted all other sim-contact types. Deleted `sim/L0/contact/` crate. Updated sim-bevy, sim-physics, CI, docs. |
| 3 — Reduce sim-constraint | `a5cef72` | Deleted PGS, Newton, sparse Jacobian, island discovery, parallel solver modules (6 files, ~6,400 lines). Extracted `BodyState`/`JointForce` to types.rs. Preserved CGSolver (1,664 lines). Removed `nalgebra-sparse` from workspace. |

**Current dependency graph:**
```
sim-physics → sim-core        (Model/Data, contact types, collision, integration)
            → sim-constraint  (joint types, motors, limits, CGSolver)
sim-bevy    → sim-core        (ContactPoint for debug visualization)
```

sim-core has zero dependency on sim-constraint.

**Phase 4 (deferred):** Merging sim-constraint into sim-types was evaluated and
deferred. sim-constraint contains behavioral types (`JointLimits.compute_force()`,
`Joint` trait with geometric methods) that don't belong in a pure-data crate. Three
name collisions (`JointLimits`, `JointType`, `JointState`) between sim-types and
sim-constraint would require renames with no functional gain. CGSolver (1,664 lines)
would still need a home. The consolidation effort achieved its goal: clean dependency
graph, no dead code, no phantom deps.

### Code Quality Fixes

Seven issues identified during solref/solimp review, all resolved.

| # | Issue | Fix |
|---|-------|-----|
| 1 | Hardcoded Baumgarte parameters | `solref_to_penalty()` + `compute_impedance()` read solref/solimp. Hardcoded values retained as fallback. |
| 2 | Distance equality constraints missing | `apply_distance_constraint` with Baumgarte stabilization. 7 integration tests. |
| 3 | Contact clone in hot path | Borrow split via `std::mem::take` on `efc_lambda`. PGS signature: `&Data` + `&mut HashMap`. |
| 4 | Cholesky clone for implicit integrator | `std::mem::replace` swaps matrix out for Cholesky. Zero-fill replacement. Further optimization → Future Work #1. |
| 5 | Duplicate contact structures | Old `ContactPoint` removed from pipeline. `Contact` (pipeline constraint) and `ContactPoint` (geometric contact) are distinct types. |
| 6 | Block Jacobi placeholder in CGSolver | Full Block Jacobi preconditioner: Cholesky-inverted diagonal blocks, enum dispatch. 2 tests. |
| 7 | Hardcoded numerical thresholds | 6 new `Model` fields (`regularization`, `default_eq_stiffness`, `default_eq_damping`, `max_constraint_vel`, `max_constraint_angvel`, `friction_smoothing`). Parseable from MJCF `<option>`. |

<details>
<summary>Issue 3 — Borrow split pattern (architectural reference)</summary>

**Problem:** `data.contacts` cannot be borrowed while `&mut Data` is passed to
`pgs_solve_contacts()` — the `&mut` prevents any shared reference.

**Solution:** `std::mem::take` extracts `efc_lambda` from `data` before the contact
block. PGS signature changed to `data: &Data` + `efc_lambda: &mut HashMap`.
`data.contacts` is then borrowed as `&[Contact]` without cloning.

**Location:** `mj_fwd_constraint()` at `mujoco_pipeline.rs`
</details>

<details>
<summary>Issue 4 — Cholesky allocation pattern (architectural reference)</summary>

**Problem:** nalgebra's `.cholesky()` consumes the matrix (takes ownership). Since
`scratch_m_impl` lives inside `Data`, it must be cloned to produce an owned value.

**Current fix:** `std::mem::replace` swaps the populated matrix with a same-sized
zero matrix. Cheaper than clone (zero-fill vs element-copy) but still O(nv²).

**True fix:** In-place Cholesky — see Future Work #1.

**Location:** `mj_fwd_acceleration_implicit()` at `mujoco_pipeline.rs`
</details>
