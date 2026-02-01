# Simulation — Future Work

Objectives ordered by expected value. Dependencies noted where they exist;
#1–#5 can proceed in parallel, #6 requires #5, #7 requires #1.

---

## 1. In-Place Cholesky Factorization

**Current state:** `mj_fwd_acceleration_implicit()` in `mujoco_pipeline.rs` uses
`std::mem::replace` to swap `scratch_m_impl` out for Cholesky consumption, allocating
an O(nv²) zero matrix every step. For nv=100 DOFs this is 80 KB/step of unnecessary
allocation.

**Objective:** Zero-allocation Cholesky factorization. The factored result overwrites
`scratch_m_impl` in place; no intermediate matrix is created.

**Specification:**

Implement `cholesky_in_place(m: &mut DMatrix<f64>) -> Result<(), StepError>` in pure
Rust. The algorithm:

1. For each column j = 0..nv:
   - `L[j,j] = sqrt(M[j,j] - Σ(L[j,k]² for k < j))`
   - For each row i > j: `L[i,j] = (M[i,j] - Σ(L[i,k]·L[j,k] for k < j)) / L[j,j]`
2. Overwrite the lower triangle of `m` with L. The upper triangle is unused.
3. Return `Err(StepError::CholeskyFailed)` if any diagonal element is ≤ 0 (matrix
   is not positive definite).

Implement `cholesky_solve_in_place(m: &DMatrix<f64>, x: &mut DVector<f64>)` for
forward/back substitution against the factored lower triangle:

1. Forward solve L·y = x (overwrite x with y).
2. Back solve L^T·z = y (overwrite x with z).

Both functions operate on borrowed data. No allocations, no ownership transfers,
no nalgebra `Cholesky` type.

**Contract:**
- `mj_fwd_acceleration_implicit()` calls `cholesky_in_place(&mut data.scratch_m_impl)`
  followed by `cholesky_solve_in_place(&data.scratch_m_impl, &mut data.scratch_v_new)`.
- `std::mem::replace` and `DMatrix::zeros(nv, nv)` are removed entirely.
- `scratch_m_impl` is reused across steps via `copy_from(&data.qM)` as before.
- Numeric results must match nalgebra's `Cholesky::solve` to f64 precision (≤ 1e-12
  relative error on the solved vector).

**See also:** Sparse L^T D L Factorization (#7) extends this to O(nv) for
tree-structured robots.

**Files:** `sim/L0/core/src/mujoco_pipeline.rs` (`mj_fwd_acceleration_implicit()`)

---

## 2. CG Contact Solver

**Current state:** `CGSolver` in `sim-constraint/src/cg.rs` implements preconditioned
conjugate gradient with Block Jacobi preconditioning (1,665 lines, fully tested). It
solves joint constraints via the `Joint` trait. The pipeline uses PGS (Projected
Gauss-Seidel) via `pgs_solve_contacts()` for contact constraints. CGSolver has zero
callers in the pipeline.

The `Joint` trait expects rigid-body joint semantics: `parent()`, `child()`,
`parent_anchor()`, `child_anchor()`, `joint_type()`. Contact constraints have none
of these — they are geometric collisions defined by a normal, tangent frame, and
friction coefficient. There is no adapter between these representations; the type
systems are incompatible.

**Objective:** CG-based contact constraint solver as an alternative to PGS,
selectable per-model.

**Specification:**

New function in `cg.rs`:

```
pub fn cg_solve_contacts(
    contacts: &[Contact],
    jacobians: &[DMatrix<f64>],
    model: &Model,
    data: &Data,
    efc_lambda: &mut HashMap<(usize, usize), [f64; 3]>,
    config: &CGSolverConfig,
) -> CGContactResult
```

The algorithm:

1. Assemble the Delassus matrix W = J M⁻¹ J^T from pre-computed contact Jacobians
   (from `compute_contact_jacobian()`).
2. Build the constraint RHS from penetration depths, approach velocities, and
   solref/solimp parameters (same physics as PGS).
3. Solve W·λ = rhs via preconditioned conjugate gradient with friction cone
   projection at each iteration (λ_n ≥ 0, |λ_t| ≤ μ·λ_n).
4. Each contact is one 3-row block (normal + 2 friction tangents). Block Jacobi
   preconditioner inverts these 3×3 diagonal blocks via Cholesky.

```
pub struct CGContactResult {
    pub forces: Vec<Vector3<f64>>,
    pub iterations_used: usize,
    pub residual_norm: f64,
    pub converged: bool,
}
```

New enum in `mujoco_pipeline.rs`:

```
pub enum SolverType {
    PGS,
    CG,
}
```

Stored as `solver_type: SolverType` in `Model` alongside `integrator: Integrator`.
Default: `PGS`. Parsed from MJCF `<option solver="CG"/>`.

`mj_fwd_constraint()` dispatches on `model.solver_type`. If CG does not converge
within `config.max_iterations`, the step falls back to PGS and logs a warning.

**Contract:**
- For any contact configuration where PGS converges, CG must produce contact forces
  that satisfy the same friction cone constraints (λ_n ≥ 0, |λ_t| ≤ μ·λ_n).
- CG and PGS are interchangeable — switching `solver_type` does not change the
  simulation's physical behavior, only solver performance characteristics.
- PGS iteration count grows linearly with contact count. CG with Block Jacobi
  maintains stable iteration counts. Expected crossover: ~100 simultaneous contacts.

**Files:** `sim/L0/constraint/src/cg.rs` (new `cg_solve_contacts()`),
`sim/L0/core/src/mujoco_pipeline.rs` (`SolverType` enum, dispatch in
`mj_fwd_constraint()`), `sim/L0/mjcf/src/parser.rs` (`solver` attribute)

---

## 3. Higher-Order Integrators

**Current state:** The pipeline `Integrator` enum has three variants (`Euler`,
`RungeKutta4`, `Implicit`). `Euler` and `RungeKutta4` both dispatch to
`mj_fwd_acceleration_explicit()` — identical semi-implicit Euler. `Implicit` solves
`(M + h·D + h²·K)·v_new = M·v_old + h·f_ext - h·K·(q - q_eq)` for diagonal per-DOF
spring stiffness K and damping D. This is an implicit spring-damper velocity
integrator, not an implicit midpoint method (the enum comment is incorrect).

A separate `Integrator` trait in `integrators.rs` has six implementations not used
by the pipeline. Its `RungeKutta4` uses a constant-acceleration reduction equivalent
to Velocity Verlet — not true multi-stage RK4.

**Objective:** Two new integration methods in the pipeline.

**Specification:**

**RK4 (replace current `Integrator::RungeKutta4` placeholder):**

True 4-stage Runge-Kutta with force re-evaluation at intermediate states:

```
k1 = f(t,       q,              v)
k2 = f(t + h/2, q + h/2·v,     v + h/2·k1)
k3 = f(t + h/2, q + h/2·v,     v + h/2·k2)
k4 = f(t + h,   q + h·v,       v + h·k3)
v_new = v + (h/6)·(k1 + 2·k2 + 2·k3 + k4)
q_new = q + (h/6)·(v + 2·(v + h/2·k1) + 2·(v + h/2·k2) + (v + h·k3))
```

Each `f()` call evaluates forward kinematics, collision detection, and force
computation at the trial state (q, v). This is 4× the cost of semi-implicit Euler
per step but achieves O(h⁴) local truncation error vs O(h) for Euler.

Implementation: `mj_fwd_acceleration_rk4(model: &Model, data: &mut Data)`. Uses
scratch buffers for intermediate k-values and trial states. No heap allocation
beyond what `Data` already provides — add scratch fields to `Data` if needed.

**ImplicitEuler (new `Integrator::ImplicitEuler` variant):**

Backward Euler: evaluate forces at the end-of-step state (t+h).

```
v_new = v + h · M⁻¹ · f(t+h, q + h·v_new, v_new)
```

Linearize: `f(t+h) ≈ f(t) + (∂f/∂q)·h·v_new + (∂f/∂v)·v_new`. For diagonal
spring-damper forces, ∂f/∂q = -K and ∂f/∂v = -D, yielding:

```
(M + h·D + h²·K)·v_new = M·v + h·f(t)
```

This is the same linear system as the current `Implicit` variant. The difference
is semantic: ImplicitEuler is the correct name for the method and will serve as
the foundation for extending to coupled (non-diagonal) implicit forces later.

**Rename:** Current `Implicit` variant → `ImplicitSpringDamper`. This accurately
describes its scope: diagonal per-DOF spring/damper terms only. Friction forces
in `qfrc_passive` remain explicit. No coupling between DOFs.

**Contract:**
- RK4 matches semi-implicit Euler forces at h→0 (consistency). For smooth force
  fields, RK4 with timestep 4h achieves comparable accuracy to Euler with timestep h.
- ImplicitEuler is unconditionally stable for arbitrarily stiff spring/damper
  systems. Energy dissipation per step is bounded.
- In-place Cholesky (#1) eliminates ImplicitEuler's per-step allocation but is not
  a prerequisite — the current `std::mem::replace` pattern works until #1 lands.

**Files:** `sim/L0/core/src/mujoco_pipeline.rs` (`Integrator` enum,
`mj_fwd_acceleration_rk4()`, `ImplicitEuler` variant, rename `Implicit` →
`ImplicitSpringDamper`), `sim/L0/core/src/integrators.rs` (align trait
implementations if warranted)

---

## 4. Deformable Body Pipeline Integration

**Current state:** `sim-deformable` is a standalone 7,700-line crate (86 tests):

| Component | Description |
|-----------|-------------|
| `XpbdSolver` | XPBD constraint solver. `step(&mut self, body: &mut dyn DeformableBody, gravity: Vector3<f64>, dt: f64)`. Configurable substeps, damping, sleeping. |
| `Cloth` | Triangle meshes with distance + dihedral bending constraints. Presets: cotton, silk, leather, rubber, paper, membrane. |
| `SoftBody` | Tetrahedral meshes with distance + volume constraints. Presets: rubber, gelatin, soft tissue, muscle, foam, stiff. |
| `CapsuleChain` | 1D particle chains with distance + bending constraints. Presets: rope, steel cable, soft cable, hair, chain. |
| `Material` | Young's modulus, Poisson's ratio, density. 14 presets (Rubber, Cloth, SoftTissue, Muscle, Cartilage, Gelatin, Foam, Leather, Rope, SteelCable, Tendon, Paper, FlexiblePlastic, SoftWood). |
| Skeletal skinning | `Skeleton`, `Bone`, `SkinnedMesh`. Linear Blend Skinning + Dual Quaternion Skinning via `SkinningMethod` enum. 1,410 lines. |
| `FlexEdge` | Stretch, shear, twist constraint variants. |

The crate has zero coupling to the MuJoCo pipeline. `sim-physics` re-exports it
behind the `deformable` feature flag. A `ConstraintType::Collision` variant is
defined in the constraint system but unimplemented. `Material.friction` and per-body
`radius`/`thickness` fields are declared but unused.

**Objective:** Deformable bodies interact with rigid bodies through the same contact
solver and step in the same simulation loop.

**Specification:**

**Collision detection** (greenfield — no infrastructure exists in either crate):

1. Broadphase: deformable vertex AABBs vs rigid geom AABBs. Vertex AABBs are
   point + radius/thickness margin. Use existing `sim-simd` `batch_aabb_overlap_4()`
   for batched broadphase queries.
2. Narrowphase: vertex-vs-geom closest point computation. Produces `Contact` structs
   identical to rigid-rigid contacts (same normal, depth, friction, solref/solimp).
3. Friction coefficient from `Material.friction` (deformable side) combined with
   geom friction (rigid side) using the same combination rule as rigid-rigid contacts.

**Contact solver coupling:**

Deformable-rigid contacts feed into PGS (or CG, per #2) alongside rigid-rigid
contacts. Each deformable vertex is a 3-DOF point mass. Its inverse mass comes from
the XPBD solver's per-particle mass. Contact Jacobians for deformable vertices are
3×3 identity blocks (point mass — no rotational DOFs).

**Force feedback:**

Contact impulses from PGS apply to deformable vertex velocities directly and to
rigid body `qfrc_constraint` through the standard Jacobian transpose. XPBD
constraint projection runs after contact resolution within the same timestep.

**Pipeline integration in `Data::step()`:**

```
1. Rigid: forward kinematics, collision, forces
2. Deformable-rigid collision detection → Contact list
3. Combined contact solve (rigid + deformable contacts)
4. Apply contact impulses to rigid bodies and deformable vertices
5. XpbdSolver::step() for each registered deformable body
6. Rigid: position integration
```

**Contract:**
- A rigid body resting on a deformable surface experiences the same contact forces
  as resting on a rigid surface of equivalent geometry.
- XPBD internal constraints (distance, bending, volume) are satisfied after contact
  resolution — contact forces do not violate deformable material properties.
- Zero-deformable-body configurations have zero overhead (no broadphase, no substep).

**Files:** `sim/L0/deformable/` (collision detection), `sim/L0/core/src/mujoco_pipeline.rs`
(pipeline integration in `Data::step()`)

---

## 5. Batched Simulation

**Current state:** Single-environment execution. `Data::step(&mut self, &Model)`
steps one simulation. `Model` is immutable after construction, uses
`Arc<TriangleMeshData>` for shared mesh data. `Data` is fully independent — no
shared mutable state, no interior mutability, derives `Clone`.

**Objective:** Step N independent environments in parallel on CPU. Foundation for
GPU acceleration (#6) and large-scale RL training.

**Specification:**

```
pub struct BatchSim {
    model: Arc<Model>,
    envs: Vec<Data>,
}

impl BatchSim {
    pub fn new(model: Arc<Model>, n: usize) -> Self;
    pub fn step_all(&mut self) -> BatchResult;
    pub fn reset(&mut self, env_idx: usize);
    pub fn reset_where(&mut self, mask: &[bool]);
}
```

`step_all()` uses rayon `par_iter_mut` over `envs`. Each `Data` steps independently
against the shared `Arc<Model>`. rayon 1.10 is already a workspace dependency;
sim-core declares it optional under the `parallel` feature flag.

```
pub struct BatchResult {
    pub states: DMatrix<f64>,       // (n_envs, nq + nv) row-major
    pub rewards: DVector<f64>,      // (n_envs,)
    pub terminated: Vec<bool>,      // per-env episode termination
    pub truncated: Vec<bool>,       // per-env time limit
    pub errors: Vec<Option<StepError>>, // None = success
}
```

`states` is a contiguous matrix for direct consumption by RL frameworks (numpy
interop via row-major layout). Reward computation is user-defined via a
`RewardFn` trait:

```
pub trait RewardFn: Send + Sync {
    fn compute(&self, model: &Model, data: &Data) -> f64;
}
```

**Error handling:** Environments that fail (e.g., `CholeskyFailed`,
`SingularMassMatrix`) are recorded in `errors`, flagged in `terminated`, and
auto-reset on the next `step_all()` call. The batch never aborts due to a single
environment failure.

**SIMD integration:** `sim-simd` provides within-environment acceleration:
`batch_dot_product_4()`, `batch_aabb_overlap_4()`, `batch_normal_force_4()`,
`batch_friction_force_4()`, `batch_integrate_position_4()`,
`batch_integrate_velocity_4()`. These accelerate the inner loop of each
environment's step. Cross-environment parallelism comes from rayon, not SIMD.

**Contract:**
- `BatchSim::step_all()` produces identical results to calling `Data::step()` on
  each environment sequentially. Parallelism does not change simulation output.
- `states` matrix layout is stable across versions (row = env, cols = qpos ++ qvel).
- Failed environments do not affect healthy environments in the same batch.

**Prerequisites:** None — can proceed in parallel with #1–#4.

**Files:** `sim/L0/core/src/` (new `batch.rs` module), `sim/L0/core/Cargo.toml`
(enable rayon under `parallel` feature)

---

## 6. GPU Acceleration

**Current state:** CPU-only.

**Objective:** wgpu compute shader backend for batch simulation. Thousands of
parallel environments on a single GPU for RL training at scale.

**Approach:** Port the inner loop of `Data::step()` (FK, collision, PGS, integration)
to compute shaders via wgpu. The `mesh-gpu` crate provides wgpu context and buffer
management; simulation-specific shaders are new work. The `BatchSim` API from #5
defines the memory layout that the GPU backend fills.

**Prerequisites:** Batched Simulation (#5).

---

## 7. Sparse L^T D L Factorization

**Current state:** Dense Cholesky in `mj_fwd_acceleration_implicit()` is O(nv³). For
tree-structured robots (humanoids, quadrupeds, manipulators), the mass matrix has
banded sparsity — DOF i couples only with its ancestor DOFs in the kinematic tree.

Data scaffolds exist in `Data`: `qLD_diag` (diagonal of D), `qLD_L` (sparse lower
triangular entries per row), `qLD_valid` (validity flag). These fields are initialized
to defaults and never populated. No factorization function exists.

**Objective:** O(nv) factorization and solve for tree-structured robots.

**Specification:**

Implement `mj_factor_sparse(model: &Model, data: &mut Data)`:

1. Walk the kinematic tree from leaves to root.
2. For each DOF i, accumulate composite inertia from child DOFs.
3. Compute `qLD_diag[i]` (diagonal pivot) and `qLD_L[i]` (off-diagonal entries
   for ancestor DOFs only).
4. Set `qLD_valid = true`.

Implement `mj_solve_sparse(data: &Data, x: &mut DVector<f64>)`:

1. Forward substitution: L·y = x, walking tree root-to-leaves.
2. Diagonal solve: D·z = y.
3. Back substitution: L^T·w = z, walking tree leaves-to-root.

Each row of L has at most `depth(i)` non-zero entries (tree depth of DOF i).
Total work is O(Σ depth(i)) which is O(nv) for balanced trees and O(nv·d) for
chains of depth d.

**Contract:**
- For a humanoid (nv=30), ~30 multiply-adds vs ~9,000 for dense — 300× reduction.
- Numeric results match dense Cholesky to f64 precision (≤ 1e-12 relative error).
- `mj_fwd_acceleration_implicit()` checks `data.qLD_valid` and dispatches to
  sparse solve when available, dense otherwise.
- Sparse factorization is recomputed when the mass matrix changes (after CRBA).

**Prerequisites:** In-Place Cholesky (#1) establishes the allocation-free pattern.

**Files:** `sim/L0/core/src/mujoco_pipeline.rs` (`mj_factor_sparse()`,
`mj_solve_sparse()`, dispatch in `mj_fwd_acceleration_implicit()`)

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
