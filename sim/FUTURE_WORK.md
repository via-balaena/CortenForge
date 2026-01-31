# Simulation — Future Work

Objectives ordered by expected value. Dependencies noted where they exist;
#1–#5 can largely proceed in parallel, #6 requires #5.

---

## 1. In-Place Cholesky Factorization

**Current state:** `mj_fwd_acceleration_implicit()` in `mujoco_pipeline.rs` uses
`std::mem::replace` to swap the scratch matrix out for Cholesky consumption. This
allocates O(nv²) per step for the replacement zero matrix (nv = degrees of freedom) —
acceptable for small models but wasteful at scale.

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

**Files:** `sim/L0/core/src/mujoco_pipeline.rs` (`mj_fwd_acceleration_implicit()`)

---

## 2. CGSolver Pipeline Integration

**Current state:** `CGSolver` (1,664 lines, Block Jacobi preconditioner) is fully
implemented in `sim-constraint/src/cg.rs` but has zero callers in the pipeline. The
active pipeline uses PGS (Projected Gauss-Seidel) via `pgs_solve_contacts()` in
`mujoco_pipeline.rs`.

**Objective:** Add CGSolver as an alternative constraint solver selectable via
`Model::solver_type` or MJCF `<option solver="CG"/>`. PGS remains the default.

**Approach:**
1. Build an adapter mapping `mujoco_pipeline.rs` contact structures (`Contact`,
   per-contact Jacobian matrices) to CGSolver's `Joint` trait interface.
2. Add `SolverType` enum (`PGS`, `CG`) to `Model`. Wire selection into
   `mj_fwd_constraint()`.
3. Benchmark against PGS on humanoid models (30–100 DOFs, 50–500 contacts).

**Motivation:** PGS iteration count grows linearly with constraint count. CG with
Block Jacobi preconditioning maintains stable iteration counts. The crossover point
is around 100+ simultaneous contacts on humanoid-scale models.

**Files:** `sim/L0/core/src/mujoco_pipeline.rs` (new `SolverType` enum alongside
`Integrator`, adapter in `mj_fwd_constraint()`), `sim/L0/constraint/src/cg.rs`

---

## 3. Higher-Order Integrators

**Current state:** Two parallel integrator systems exist:
- Pipeline `Integrator` enum in `mujoco_pipeline.rs` — three variants (`Euler`,
  `RungeKutta4`, `Implicit`), but `Euler` and `RungeKutta4` both run the same
  semi-implicit Euler code path. `Implicit` runs an implicit midpoint method that
  accounts for spring/damper stiffness via the `scratch_m_impl` mass matrix.
- Separate `Integrator` trait (not the pipeline enum) in `integrators.rs` — six implementations
  (`ExplicitEuler`, `SemiImplicitEuler`, `VelocityVerlet`, `RungeKutta4`,
  `ImplicitVelocity`, `ImplicitFast`) with a matching `IntegrationMethod` enum
  in sim-types. Not used by the pipeline.

**Objective:** Implement actual RK4 multi-stage integration behind the existing
`Integrator::RungeKutta4` pipeline variant, and add a new `ImplicitEuler` pipeline
variant for unconditionally stable integration of arbitrarily stiff systems.

**Why it matters:** Higher-order methods improve accuracy for stiff systems — high
damping, high stiffness springs, and small timestep requirements. Implicit Euler is
unconditionally stable for arbitrarily stiff problems.

**Note:** In-place Cholesky (#1) would make implicit Euler's per-step linear solve
allocation-free, but is not required.

**Files:** `sim/L0/core/src/mujoco_pipeline.rs`, `sim/L0/core/src/integrators.rs`

---

## 4. Deformable Bodies

**Current state:** `sim-deformable` is a standalone 7,700-line crate (86 tests) with:
- `XpbdSolver` — XPBD constraint solver with configurable substeps, damping, sleeping
- `Cloth` — triangle meshes with distance + dihedral bending constraints
- `SoftBody` — tetrahedral meshes with distance + volume constraints
- `CapsuleChain` — 1D particle chains with distance + bending constraints (ropes, cables)
- `Material` — continuum mechanics model (Young's modulus, Poisson's ratio, density)
  with 14 presets (rubber, cloth, soft tissue, muscle, etc.)
- Skeletal skinning — `Skeleton`, `Bone`, `SkinnedMesh` with Linear Blend Skinning
  and Dual Quaternion Skinning (1,410 lines)
- `FlexEdge` constraints — stretch, shear, twist variants

The crate is fully implemented but has zero coupling to the MuJoCo pipeline. `sim-physics`
already re-exports it behind an optional `deformable` feature flag with full prelude
coverage — the API surface is wired, the pipeline integration is not.

**Objective:** Couple deformable bodies into the rigid body pipeline so they interact
through the same contact solver and step in the same simulation loop.

**Approach:**
1. **Collision detection:** Broadphase (deformable vertex AABBs vs rigid geom AABBs)
   and narrowphase (vertex-vs-geom closest point) to produce `Contact` structs between
   deformable vertices and rigid bodies.
2. **Contact solver coupling:** Feed deformable-rigid `Contact`s into PGS (or CG)
   alongside rigid-rigid contacts. Deformable vertices act as 3-DOF point masses with
   per-particle inverse mass from the XPBD solver.
3. **Force feedback:** Apply PGS contact impulses back to deformable vertex velocities
   and rigid body `qfrc_constraint`. XPBD constraint projection runs after contact
   resolution within the same timestep.
4. **Pipeline wiring:** Add a deformable substep to `Data::step()` that calls
   `XpbdSolver::step()` for each registered deformable body, interleaved with rigid
   body integration.

**Material model extensibility:** The XPBD solver derives compliance from Young's
modulus, Poisson's ratio, and density — fast, stable, and sufficient for RL training.
A `MaterialModel` trait would allow plugging in alternative constitutive models (e.g.,
Neo-Hookean FEM for quantitative stress accuracy) behind the same `DeformableBody`
interface. XPBD remains the default; FEM is opt-in for use cases that require it
(surgical simulation, material validation).

**Use cases:** Soft tissue simulation, cloth/fabric, ropes/cables, pneumatic actuators.

**Files:** `sim/L0/deformable/`, `sim/L0/core/src/mujoco_pipeline.rs`

---

## 5. Batched Simulation

**Current state:** Single-environment execution only. Each `Model`/`Data` pair runs
one simulation.

**Objective:** CPU-parallel multi-environment architecture for RL training. Step N
environments in parallel, return N observation tensors.

**Approach:**
- Task-per-env parallelism via rayon's work-stealing thread pool (shared-nothing `Data`)
- SIMD-across-envs for batched math via `sim-simd` (batch dot products, AABB tests,
  contact force computation already implemented)
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
to compute shaders via wgpu. The `mesh-gpu` crate provides wgpu context and buffer
management; simulation-specific shaders will need to be built.

**Prerequisites:** Batched Simulation (#5) — the CPU batching API defines the memory
layout that the GPU backend fills.

**Use cases:** Large-scale RL training, real-time multi-agent simulation.

---

## Completed Work (Reference)

Historical context for decisions referenced above. Details collapsed to keep this
document focused on future objectives.

### Crate Consolidation

Three-phase effort to clean the sim dependency graph. Eliminated sim-contact
(merged into sim-core) and stripped sim-constraint to types + CGSolver, removing
~11,000 lines of dead code.

| Phase | Commit | Summary |
|-------|--------|---------|
| 1 — Remove phantom deps | `ba1d729` | Removed 4 phantom Cargo.toml deps (sim-mjcf, sim-urdf, sim-bevy, sim-tests → sim-constraint). Removed dead re-exports (`ContactSolverConfig`). Moved sim-constraint from sim-core deps to dev-deps. |
| 2 — Consolidate sim-contact | `9ed88ea` | Moved `ContactPoint`, `ContactForce`, `ContactManifold` into `sim-core/src/contact.rs`. Deleted all other sim-contact types. Deleted `sim/L0/contact/` crate. Updated sim-bevy, sim-physics, CI, docs. |
| 3 — Reduce sim-constraint | `a5cef72` | Deleted PGS, Newton, sparse Jacobian, island discovery, parallel solver modules (6 files, ~6,000 lines). Extracted `BodyState`/`JointForce` to types.rs. Preserved CGSolver (1,664 lines). Removed `nalgebra-sparse` from workspace. |

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
