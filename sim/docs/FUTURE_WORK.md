# Simulation — Future Work

## Priority Framework

Items are scored on three axes:

| Axis | Definition |
|------|-----------|
| **RL Impact** | How directly this unblocks reinforcement-learning training workflows (batch stepping, observation fidelity, GPU throughput). |
| **Correctness** | Whether this fixes a simulation bug, stub, or semantic mismatch that produces wrong results today. |
| **Effort** | Implementation size: **S** (< 200 LOC), **M** (200–800 LOC), **L** (800–2 000 LOC), **XL** (> 2 000 LOC). |

Priority is **RL Impact + Correctness**, tie-broken by inverse Effort. Items within a group
can be tackled in any order unless a prerequisite is noted.

| # | Item | RL Impact | Correctness | Effort | Prerequisites |
|---|------|-----------|-------------|--------|---------------|
| 1 | In-Place Cholesky | Low | Low | S | None |
| 2 | Sparse L^T D L | Medium | Low | M | #1 |
| 3 | CG Contact Solver | Medium | Low | L | None (#1, #2 help perf) |
| 4 | Tendon Pipeline | Low | High | L | None |
| 5 | Muscle Pipeline | Low | High | L | #4 |
| 6 | Sensor Completion | High | High | S (remaining) | #4 for tendon sensors |
| 7 | Integrator Rename | Low | Medium | S | None |
| 8 | True RK4 Integration | Low | Medium | M | None |
| 9 | Deformable Body Integration | Medium | Low | XL | None |
| 10 | Batched Simulation | High | Low | L | None |
| 11 | GPU Acceleration | High | Low | XL | #10 |

## Dependency Graph

```
   ┌───┐
   │ 1 │ In-Place Cholesky
   └─┬─┘
     │
     ▼
   ┌───┐         ┌───┐
   │ 2 │ Sparse  │ 3 │ CG Solver ←──(perf benefits from #1, #2; not hard deps)
   └───┘         └───┘

   ┌───┐
   │ 4 │ Tendon Pipeline
   └─┬─┘
     │
     ▼
   ┌───┐
   │ 5 │ Muscle Pipeline
   └───┘

   ┌───┐
   │ 6 │ Sensor Completion              (independent)
   └───┘

   ┌───┐   ┌───┐
   │ 7 │   │ 8 │ Integrator items        (independent)
   └───┘   └───┘

   ┌───┐
   │ 9 │ Deformable Body                (independent)
   └───┘

   ┌────┐
   │ 10 │ Batched Simulation
   └─┬──┘
     │
     ▼
   ┌────┐
   │ 11 │ GPU Acceleration
   └────┘
```

---

## Group A — Solver & Linear Algebra

### 1. In-Place Cholesky Factorization
**Status:** ✅ Done | **Effort:** S | **Prerequisites:** None

#### Current State
`mj_fwd_acceleration_implicit()` (`mujoco_pipeline.rs:8790`) uses `std::mem::replace`
to swap `scratch_m_impl` out for nalgebra's `.cholesky()` (which takes ownership),
replacing it with a freshly allocated `DMatrix::zeros(nv, nv)` every step
(`mujoco_pipeline.rs:8846`). For nv=100 this is ~78 KB/step of unnecessary heap
allocation and zeroing.

The current flow (`mujoco_pipeline.rs:8846-8852`):
```rust
let chol = std::mem::replace(&mut data.scratch_m_impl, DMatrix::zeros(nv, nv))
    .cholesky()
    .ok_or(StepError::CholeskyFailed)?;
data.scratch_v_new.copy_from(&data.scratch_rhs);
chol.solve_mut(&mut data.scratch_v_new);
```

`scratch_m_impl` is pre-allocated in `make_data()` (`mujoco_pipeline.rs:1771`) and
populated each step via `copy_from(&data.qM)` (`mujoco_pipeline.rs:8814`) which
overwrites the full matrix. After Cholesky, the matrix is never read again until
the next step's `copy_from`. The replacement zero matrix is therefore wasted — the
next `copy_from` would overwrite it entirely regardless of contents.

`StepError::CholeskyFailed` already exists (`mujoco_pipeline.rs:655`).

#### Objective
Zero-allocation Cholesky factorization that overwrites `scratch_m_impl` in place,
eliminating the `std::mem::replace` + `DMatrix::zeros` pattern.

#### Specification

Add two functions to `mujoco_pipeline.rs`, directly above `mj_fwd_acceleration_implicit()`:

**`cholesky_in_place(m: &mut DMatrix<f64>) -> Result<(), StepError>`**

Standard Cholesky (LL^T) factorization, overwriting the lower triangle in place.
Columns are processed left-to-right; references to `m[(j,k)]` for `k < j` read
already-computed L entries from previous columns (this is what makes it in-place):

1. Let `n = m.nrows()`.
2. For each column j = 0..n:
   - Compute `diag = m[(j,j)] - Σ(m[(j,k)]² for k < j)`.
   - If `diag` ≤ 0: return `Err(StepError::CholeskyFailed)` (matrix is not SPD).
   - Set `m[(j,j)] = sqrt(diag)`.
   - For each row i = (j+1)..n:
     `m[(i,j)] = (m[(i,j)] - Σ(m[(i,k)]·m[(j,k)] for k < j)) / m[(j,j)]`
3. The lower triangle of `m` now contains L. The upper triangle retains its
   original values (stale M_impl entries) — this is safe because `copy_from(&data.qM)`
   fully overwrites the matrix at the start of the next step, and
   `cholesky_solve_in_place` only reads the lower triangle.

**`cholesky_solve_in_place(l: &DMatrix<f64>, x: &mut DVector<f64>)`**

Solve L·L^T·x = b where L is stored in the lower triangle of `l`:

1. Let `n = l.nrows()`.
2. Forward substitution (L·y = x): for j = 0..n,
   `x[j] = (x[j] - Σ(l[(j,k)] · x[k] for k < j)) / l[(j,j)]`.
3. Back substitution (L^T·z = y): for j = (n-1)..=0,
   `x[j] = (x[j] - Σ(l[(k,j)] · x[k] for k = (j+1)..n)) / l[(j,j)]`.
   (L^T elements accessed via `l[(k,j)]` — reading column j of L as row j of L^T.)

Both functions operate on borrowed data. No allocations, no ownership transfers,
no nalgebra `Cholesky` type.

**Modify `mj_fwd_acceleration_implicit()`** — replace lines 8838-8852
(the Cholesky comment block, `let nv`, `std::mem::replace`, and `chol.solve_mut`)
with:

```rust
// Factorize M_impl in place (overwrites lower triangle with L where M_impl = L·L^T).
// M_impl is SPD (M is SPD from CRBA, D ≥ 0, K ≥ 0).
cholesky_in_place(&mut data.scratch_m_impl)?;
data.scratch_v_new.copy_from(&data.scratch_rhs);
cholesky_solve_in_place(&data.scratch_m_impl, &mut data.scratch_v_new);
```

This removes `let nv = model.nv` (no longer needed), `std::mem::replace`,
`DMatrix::zeros(nv, nv)`, and the old comment block explaining the replace pattern.

The `Cholesky` import stays — it is still used by `qM_cholesky: Option<Cholesky<f64, Dyn>>`
(`mujoco_pipeline.rs:1325`) for cached mass matrix factorization in CRBA.

#### Acceptance Criteria
1. `std::mem::replace` and `DMatrix::zeros(nv, nv)` are removed from `mj_fwd_acceleration_implicit()`.
2. `scratch_m_impl` is reused across steps — populated via `copy_from(&data.qM)`,
   factorized in place, then fully overwritten again next step.
3. Numeric results match nalgebra `Cholesky::solve` to ≤ 1e-12 relative error on the
   solved vector. Verified by a unit test that compares both paths on random SPD matrices.
4. Zero heap allocations in the factorize/solve path — neither function allocates.
   (No `#[global_allocator]` test required; the functions take only `&mut` references
   and contain no `Vec`, `Box`, or `DMatrix` constructors.)
5. Existing implicit integration tests pass without modification.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (`cholesky_in_place()`,
  `cholesky_solve_in_place()`, refactor `mj_fwd_acceleration_implicit()`)

---

### 2. Sparse L^T D L Factorization
**Status:** Done | **Effort:** M | **Prerequisites:** #1

#### Current State

Three consumers solve linear systems involving the mass matrix M:

| Consumer | Location | Current method | Cost |
|----------|----------|---------------|------|
| `mj_fwd_acceleration_explicit()` | `mujoco_pipeline.rs:8729` | `data.qM_cholesky` (nalgebra) | O(nv²) solve, O(nv³) factorize |
| `mj_fwd_acceleration_implicit()` | `mujoco_pipeline.rs:8845` | `cholesky_in_place()` on M_impl | O(nv³) factorize + solve |
| `pgs_solve_contacts()` | `mujoco_pipeline.rs:7041` | `data.qM_cholesky` (nalgebra) | O(nv²) solve, O(nv³) factorize |

For tree-structured robots, `M[i,j] ≠ 0` only when DOF `j` is an ancestor of DOF `i`
(or vice versa) in the kinematic tree encoded by `model.dof_parent: Vec<Option<usize>>`
(`mujoco_pipeline.rs:796`). This tree sparsity enables O(nv) factorization and solve.

**Data scaffolds** exist in `Data` (`mujoco_pipeline.rs:1344-1351`):
```rust
pub qLD_diag: DVector<f64>,                  // D[i,i] diagonal
pub qLD_L: Vec<Vec<(usize, f64)>>,           // L[i,:] sparse off-diagonal per row
pub qLD_valid: bool,                         // dispatch flag
```
Initialized at `mujoco_pipeline.rs:1728-1730` (zeros/empty/false), never populated.
The `TODO(FUTURE_WORK#2)` comment is at `mujoco_pipeline.rs:1328`.

The `qM_cholesky: Option<Cholesky<f64, Dyn>>` at `mujoco_pipeline.rs:1325` is computed
in `mj_crba()` Phase 5 (`mujoco_pipeline.rs:6148`) via `data.qM.clone().cholesky()` —
this clone+factorize is the O(nv³) cost we eliminate for the explicit/contact paths.

#### Objective
O(nv) factorization and solve for tree-structured robots, matching MuJoCo's
`mj_factorM` / `mj_solveM` semantics.

#### Specification

##### Data Structure: keep existing scaffolds

Keep `qLD_diag`, `qLD_L`, `qLD_valid` as-is. The `Vec<Vec<(usize, f64)>>` for `qLD_L`
involves heap allocations per row, but rows are allocated once in `make_data()` and
reused via `clear()` + `push()`. For nv ≤ 200 (typical RL), the overhead is negligible
vs. the O(nv³→nv) savings. Switching to flat CSR is a future optimization.

##### Step 1: `mj_factor_sparse(model: &Model, data: &mut Data)`

Computes `M = L^T D L` where `L` is unit lower triangular (`L[i,i] = 1` implicitly,
stored entries are off-diagonal only) and `D` is diagonal.

The identity `M[i,j] = Σ_k L[k,i] · D[k] · L[k,j]` means: when processing DOF `i`
from leaves to root, we subtract the rank-1 contributions of already-processed
descendants, then extract `D[i]` and `L[i,:]` from the residual.

```rust
fn mj_factor_sparse(model: &Model, data: &mut Data) {
    let nv = model.nv;

    // Phase 1: Copy M's sparse entries into qLD working storage.
    // qLD_diag[i] starts as M[i,i]; qLD_L[i] holds (col, M[i,col]) for ancestors.
    for i in 0..nv {
        data.qLD_diag[i] = data.qM[(i, i)];
        data.qLD_L[i].clear();
        let mut p = model.dof_parent[i];
        while let Some(j) = p {
            data.qLD_L[i].push((j, data.qM[(i, j)]));
            p = model.dof_parent[j];
        }
        // Parent chain yields descending col indices; reverse for ascending order.
        data.qLD_L[i].reverse();
    }

    // Phase 2: Eliminate from leaves to root.
    // When we process DOF i, all descendants k > i are already factored.
    // We propagate i's contribution to ancestors j < i, which requires
    // mutating qLD_L[j] while reading qLD_L[i]. Since j < i always,
    // we use split_at_mut to get non-overlapping borrows.
    for i in (0..nv).rev() {
        let di = data.qLD_diag[i];

        // Scale off-diagonals: L[i,j] = working[i,j] / D[i]
        for entry in &mut data.qLD_L[i] {
            entry.1 /= di;
        }

        // Propagate rank-1 update to ancestors.
        // Split: [0..i] and [i..nv] give non-overlapping mutable access.
        let (lo, hi) = data.qLD_L.split_at_mut(i);
        let row_i = &hi[0]; // qLD_L[i], immutable borrow

        for idx_a in 0..row_i.len() {
            let (j, lij) = row_i[idx_a];
            data.qLD_diag[j] -= lij * lij * di;
            for idx_b in 0..idx_a {
                let (k, lik) = row_i[idx_b];
                // k < j < i, so both are in `lo`
                if let Some(entry) = lo[j].iter_mut()
                    .find(|(col, _)| *col == k)
                {
                    entry.1 -= lij * lik * di;
                }
            }
        }
    }

    data.qLD_valid = true;
}
```

**Complexity:** O(Σ depth(i)²). For balanced trees (depth ~ log n) this is O(n log² n).
For chains (depth = n) it degrades to O(n³), matching dense — expected since chains
produce dense mass matrices. For typical humanoids (depth ≤ 8), effectively O(n).

##### Step 2: `mj_solve_sparse(...)`

Solves `L^T D L x = b` where `x` initially contains `b`.

The function borrows `qLD_diag` and `qLD_L` separately from `x` to avoid
borrow conflicts when `x` is a field of `Data` (e.g., `data.qacc`):

```rust
fn mj_solve_sparse(
    qld_diag: &DVector<f64>,
    qld_l: &[Vec<(usize, f64)>],
    x: &mut DVector<f64>,
) {
    let nv = x.len();

    // Phase 1: Solve L^T y = b (scatter: propagate each DOF to its ancestors)
    for i in (0..nv).rev() {
        for &(j, lij) in &qld_l[i] {
            x[j] -= lij * x[i];
        }
    }

    // Phase 2: Solve D z = y
    for i in 0..nv {
        x[i] /= qld_diag[i];
    }

    // Phase 3: Solve L w = z (standard forward substitution)
    for i in 0..nv {
        for &(j, lij) in &qld_l[i] {
            x[i] -= lij * x[j];
        }
    }
}
```

##### Step 3: Integration into `mj_crba()` (Phase 5 replacement)

At `mujoco_pipeline.rs:6138-6148`, replace the nalgebra Cholesky with sparse factorization:

```rust
// Phase 5: Factorize mass matrix (sparse L^T D L)
if model.nv > 0 {
    mj_factor_sparse(model, data);
}
// Remove: data.qM_cholesky = data.qM.clone().cholesky();
// The sparse factorization replaces this for all consumers.
```

Also set `data.qLD_valid = false` at the **top** of `mj_crba()` (before qM is zeroed),
so stale factorization is never used during the CRBA computation.

Remove the `qM_cholesky` field from `Data` entirely (along with its initialization
in `make_data()` and the doc comments referencing it).

##### Step 4: Dispatch in `mj_fwd_acceleration_explicit()` (line 8729)

Replace the `qM_cholesky` match with sparse solve:

```rust
fn mj_fwd_acceleration_explicit(model: &Model, data: &mut Data) {
    // ... sum forces into qfrc_total ...

    // Solve M * qacc = qfrc_total using sparse L^T D L factorization
    data.qacc.copy_from(&qfrc_total);
    mj_solve_sparse(&data.qLD_diag, &data.qLD_L, &mut data.qacc);
}
```

Remove the `qM_cholesky`-based path and the LU/diagonal fallbacks entirely.
If the mass matrix is degenerate, `mj_factor_sparse` will produce a zero or
negative `qLD_diag[i]`, and the solve will produce inf/NaN — same as MuJoCo's
behavior (garbage-in-garbage-out for degenerate systems).

##### Step 5: Migrate `pgs_solve_contacts()` (line 7041)

Replace all `chol.solve(&x)` calls with sparse solve:

```rust
// Was: let qacc_smooth = chol.solve(&qfrc_smooth);
let mut qacc_smooth = qfrc_smooth;   // move, no clone needed
mj_solve_sparse(&data.qLD_diag, &data.qLD_L, &mut qacc_smooth);

// Was: let x = chol.solve(&jt_col);
let mut x = jt_col;                  // already owned
mj_solve_sparse(&data.qLD_diag, &data.qLD_L, &mut x);
```

Remove the `qM_cholesky` match guard at the top. The fallback to penalty
for singular M is no longer needed — if sparse factorization is invalid
(`!data.qLD_valid`), that's a bug (CRBA always runs before contacts).

##### Step 6: Dispatch in `mj_fwd_acceleration_implicit()` (line 8845)

The implicit path uses `M_impl = M + h·D + h²·K`. The diagonal modification
(`+= h*d[i] + h²*k[i]` at line 8871) means we **cannot** reuse the CRBA sparse
factorization — the modified matrix has different diagonal values.

**Keep dense for implicit.** Rationale: the implicit path already uses
`scratch_m_impl` (a dense DMatrix copied from qM + diagonal mods) and our
`cholesky_in_place()` from #1. Sparse factorization would require duplicating
the qLD fields. The benefit is marginal since implicit integration is rare
in RL (most RL uses Euler). We can upgrade in a future PR if profiling shows
it matters.

The implicit path (`mj_fwd_acceleration_implicit`) is **unchanged** by this PR.

#### Acceptance Criteria

1. **Numeric correctness:** For all joint types (hinge, ball, free, mixed chains,
   branching trees), `mj_solve_sparse(&qLD_diag, &qLD_L, &mut x)` produces the same
   result as nalgebra `cholesky().solve(&x)` to ≤ 1e-12 relative error.
2. **Consumers migrated:** `mj_fwd_acceleration_explicit()` and `pgs_solve_contacts()`
   both use `mj_solve_sparse`. Implicit path unchanged (dense `cholesky_in_place`).
3. **`qM_cholesky` removed:** The `Option<Cholesky<f64, Dyn>>` field, its initialization
   in `make_data()`, and all references are deleted. No nalgebra Cholesky allocation.
4. **Invalidation:** `qLD_valid` is set to `false` at top of `mj_crba()`.
5. **Tree coverage:** Tests include:
   - Single hinge (nv=1, trivial)
   - 2-link chain (nv=2, single ancestor)
   - Branching tree (nv=3, two children of same parent)
   - Ball joint chain (nv=6, multi-DOF within joint)
   - Free body + chain (nv=7+, mixed joint types)
6. **No new allocations in hot path:** `qLD_L[i].clear()` + `push()` reuses
   existing `Vec` capacity after the first step. No `DMatrix`/`DVector` created
   in factorization or solve.
7. **Existing tests pass:** All implicit integration tests, explicit tests,
   and contact tests remain green.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify:
  - `mj_factor_sparse()` (new function, ~50 lines)
  - `mj_solve_sparse()` (new function, ~25 lines)
  - `mj_crba()`: replace Phase 5 (sparse factorize + invalidation)
  - `mj_fwd_acceleration_explicit()`: replace with sparse solve
  - `pgs_solve_contacts()`: replace `chol.solve()` calls with sparse solve
  - Remove `qM_cholesky` field + initialization + doc comments
  - Remove `use nalgebra::linalg::Cholesky` import (if unused after removal)
  - New test module `sparse_factorization_tests` (~100 lines)
- `sim/docs/MUJOCO_REFERENCE.md` — update Phase 5 description (line 143-145)
- `sim/docs/ARCHITECTURE.md` — update `qM_cholesky` reference (line 74)

---

### 3. CG Contact Solver
**Status:** Not started | **Effort:** L | **Prerequisites:** None (#1, #2 improve M⁻¹ performance but are not hard dependencies)

#### Current State
`CGSolver` in `sim-constraint/src/cg.rs` (1,664 lines, `cg.rs:309`) implements
preconditioned conjugate gradient with Block Jacobi preconditioning. It solves joint
constraints via the `Joint` trait (`cg.rs:408`): `solve<J: Joint>(&mut self, joints: &[J], ...)`.
The pipeline uses PGS via `pgs_solve_contacts()` (`mujoco_pipeline.rs:6563`) for contact
constraints. CGSolver has zero callers in the pipeline.

The `Joint` trait expects rigid-body joint semantics (`parent()`, `child()`,
`parent_anchor()`, `joint_type()`). Contact constraints have none of these — they are
geometric collisions defined by normal, tangent frame, and friction coefficient. The
type systems are incompatible; no adapter exists.

#### Objective
CG-based contact constraint solver as an alternative to PGS, selectable per-model.

#### Specification

New function in `cg.rs`:

```rust
pub fn cg_solve_contacts(
    contacts: &[Contact],
    jacobians: &[DMatrix<f64>],
    model: &Model,
    data: &Data,
    efc_lambda: &mut HashMap<(usize, usize), [f64; 3]>,
    config: &CGSolverConfig,
) -> CGContactResult
```

Algorithm:

1. Assemble the Delassus matrix W = J M⁻¹ J^T from pre-computed contact Jacobians
   (from `compute_contact_jacobian()` at `mujoco_pipeline.rs:6383`).
2. Build the constraint RHS from penetration depths, approach velocities, and
   solref/solimp parameters (same physics as PGS).
3. Solve W·λ = rhs via preconditioned conjugate gradient with friction cone
   projection at each iteration (λ_n ≥ 0, |λ_t| ≤ μ·λ_n).
4. Each contact is one 3-row block (normal + 2 friction tangents). Block Jacobi
   preconditioner inverts these 3×3 diagonal blocks via Cholesky.

```rust
pub struct CGContactResult {
    pub forces: Vec<Vector3<f64>>,
    pub iterations_used: usize,
    pub residual_norm: f64,
    pub converged: bool,
}
```

New enum in `mujoco_pipeline.rs`:

```rust
pub enum SolverType { PGS, CG }
```

Stored as `solver_type: SolverType` in `Model`. Default: `PGS`. Parsed from MJCF
`<option solver="CG"/>`. `mj_fwd_constraint()` (`mujoco_pipeline.rs:7991`) dispatches
on `model.solver_type`.

**Fallback policy (testing concern):** If CG does not converge within
`config.max_iterations`, the step falls back to PGS and logs a warning. This
silent-fallback pattern means CG correctness bugs can hide behind PGS. The test
suite should include cases where CG is required to converge (no fallback) to
catch regressions. Consider a `SolverType::CGStrict` variant that returns
`Err(StepError::SolverFailed)` instead of falling back, for use in tests.

#### Acceptance Criteria
1. For any contact configuration where PGS converges, CG produces forces satisfying the same friction cone constraints (λ_n ≥ 0, |λ_t| ≤ μ·λ_n).
2. CG and PGS are interchangeable — switching `solver_type` does not change physical behavior, only solver performance.
3. Test suite includes ≥ 5 contact scenarios run with both PGS and CG, verifying force equivalence within tolerance.
4. Test suite includes `CGStrict` mode tests that fail on non-convergence rather than silently falling back.
5. Benchmark showing stable CG iteration count as contact count grows (expected crossover vs PGS at ~100 simultaneous contacts).

#### Files
- `sim/L0/constraint/src/cg.rs` — modify (new `cg_solve_contacts()`)
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (`SolverType` enum, dispatch in `mj_fwd_constraint()`)
- `sim/L0/mjcf/src/parser.rs` — modify (`solver` attribute parsing)

---
## Group B — Pipeline Integration

### 4. Tendon Pipeline
**Status:** Not started | **Effort:** L | **Prerequisites:** None

#### Current State
sim-tendon is a standalone 3,919-line crate with path routing (`cable.rs`, `spatial.rs`,
`wrapping.rs`, `pulley.rs`, `fixed.rs`), a `TendonActuator` trait (`tendon/src/lib.rs:146`),
and zero coupling to the MuJoCo pipeline.

Model fields are fully declared (`mujoco_pipeline.rs:907-929`): `ntendon`,
`tendon_stiffness`, `tendon_damping`, `tendon_lengthspring`, `tendon_length0`,
`tendon_range`, `tendon_limited`, `tendon_num`, `tendon_adr`, `tendon_name`.

Data scaffolds exist (`mujoco_pipeline.rs:1366-1376`): `ten_length`, `ten_velocity`,
`ten_force`, `ten_J` — all initialized to defaults at `mujoco_pipeline.rs:1735-1738`
and never populated.

`mj_fwd_actuation()` (`mujoco_pipeline.rs:5475`) has an explicit placeholder at
`mujoco_pipeline.rs:5495-5498`:
```rust
ActuatorTransmission::Tendon | ActuatorTransmission::Site => {
    // Placeholder for tendon/site actuation
}
```

#### Objective
Wire tendon kinematics, passive forces, and actuation into the MuJoCo pipeline so
that tendon-driven actuators produce joint forces.

#### Specification

**`mj_fwd_tendon(model: &Model, data: &mut Data)`** — position-dependent:

1. For each tendon t = 0..ntendon:
   - Walk the tendon path (wrap objects from `tendon_adr[t]` to `tendon_adr[t] + tendon_num[t]`).
   - Compute `ten_length[t]` as the sum of segment lengths along the path.
   - Compute `ten_J[t]` (Jacobian mapping joint velocities to tendon length change)
     via finite differencing or analytic path derivatives.
2. Call site: after forward kinematics, before force computation.

**`mj_fwd_tendon_vel(model: &Model, data: &mut Data)`** — velocity-dependent:

1. For each tendon t: `ten_velocity[t] = ten_J[t].dot(&data.qvel)`.
2. Call site: after `mj_fwd_tendon()` and velocity computation.

**Tendon passive forces:**

1. Spring force: `f_spring = -tendon_stiffness[t] * (ten_length[t] - tendon_lengthspring[t])`.
2. Damping force: `f_damp = -tendon_damping[t] * ten_velocity[t]`.
3. Limit force: if `tendon_limited[t]` and length outside `tendon_range[t]`,
   apply Baumgarte-style restoring force using solref/solimp.
4. `ten_force[t] = f_spring + f_damp + f_limit`.
5. Map to joint forces: `qfrc_passive += ten_J[t]^T * ten_force[t]`.

**Tendon actuation** (fills the placeholder at `mujoco_pipeline.rs:5495`):

For `ActuatorTransmission::Tendon`: look up the target tendon via `actuator_trnid`.
Apply `ctrl * gear` as a force along the tendon, mapped to joints via:
`qfrc_actuator += ten_J[tendon_id]^T * (ctrl * gear)`.

#### Acceptance Criteria
1. `ten_length`, `ten_velocity`, `ten_force`, `ten_J` are populated every step for all tendons.
2. A tendon-driven actuator produces the same joint torque as a direct joint actuator with equivalent moment arm (within 1e-10 tolerance).
3. Tendon passive spring/damper forces appear in `qfrc_passive` and affect joint dynamics.
4. Tendon limit forces activate when length exceeds `tendon_range` and use solref/solimp parameters.
5. Zero-tendon models (`ntendon == 0`) have zero overhead.
6. The `ActuatorTransmission::Tendon` placeholder is replaced with working code.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (`mj_fwd_tendon()`, `mj_fwd_tendon_vel()`, tendon forces in `mj_fwd_passive()`, tendon actuation in `mj_fwd_actuation()`)
- `sim/L0/tendon/src/` — reference for path computation algorithms

---

### 5. Muscle Pipeline
**Status:** Not started | **Effort:** L | **Prerequisites:** #4 (tendon pipeline provides `ten_length`, `ten_velocity`, `ten_J`)

#### Current State
sim-muscle is a standalone 2,550-line crate with:
- `ActivationDynamics` (`activation.rs`): first-order `da/dt = (u - a) / τ(u, a)` with asymmetric time constants (τ_act ≈ 10-20 ms, τ_deact ≈ 40-80 ms).
- `HillMuscle` (`hill.rs`): contractile element (force-length, force-velocity), parallel elastic element, series elastic tendon. Presets: biceps, quadriceps, gastrocnemius, soleus.
- Force-length/velocity curves (`curves.rs`).
- `MuscleKinematics` (`kinematics.rs`): pennation angle, fiber length from MTU length.

The pipeline declares `ActuatorDynamics::Muscle` (`mujoco_pipeline.rs:434`) and
`data.act` (`mujoco_pipeline.rs:1249`) but `mj_fwd_actuation()` has no muscle-specific
code path — it only handles `ActuatorTransmission::Joint` (direct force) and stubs
`::Tendon`/`::Site`.

#### Objective
Wire muscle activation dynamics and Hill-type force generation into the pipeline so
that muscle actuators produce physiologically realistic joint forces.

#### Specification

**Activation dynamics** (in `mj_fwd_actuation()` or a new `mj_fwd_activation()`):

For each actuator where `actuator_dyntype[i] == Muscle`:

1. Read excitation `u = ctrl[i]` (clamped to [0, 1]).
2. Update activation: `act[i] += dt * (u - act[i]) / τ(u, act[i])` where
   τ = τ_act if u > act[i], τ = τ_deact otherwise.
3. Clamp `act[i]` to [min_activation, 1.0].

**Force generation** (after tendon kinematics from #4):

1. Get muscle-tendon unit length from `ten_length[tendon_id]` (the tendon this
   muscle actuator drives) and velocity from `ten_velocity[tendon_id]`.
2. Compute fiber length and velocity via pennation angle geometry
   (from sim-muscle `MuscleKinematics`).
3. Compute active force: `F_active = act[i] * f_l(l_fiber) * f_v(v_fiber) * F_max`.
4. Compute passive force: `F_passive = f_pe(l_fiber) * F_max`.
5. Total muscle force: `F_muscle = (F_active + F_passive) * cos(pennation_angle)`.
6. Map to joint forces via tendon Jacobian: `qfrc_actuator += ten_J[tendon_id]^T * F_muscle`.

**Model fields needed** (new or repurposed):
- `actuator_muscle_config: Vec<Option<HillMuscleConfig>>` — per-actuator muscle parameters (F_max, optimal fiber length, tendon slack length, pennation angle, time constants).
- Parse from MJCF `<muscle>` element attributes.

#### Acceptance Criteria
1. `data.act` is updated every step for all muscle actuators using the first-order activation model.
2. Muscle force follows the Hill model: zero at zero activation, monotonically increasing with activation, exhibiting force-length and force-velocity relationships.
3. A muscle actuator at full activation with optimal fiber length produces `F_max * cos(pennation_angle)` force along the tendon.
4. Activation dynamics exhibit correct asymmetry: activation faster than deactivation.
5. Non-muscle actuators (`ActuatorDynamics::None/Filter/Integrator`) are unaffected.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (`mj_fwd_actuation()` or new `mj_fwd_activation()`, muscle force computation)
- `sim/L0/muscle/src/` — reference for Hill model and activation dynamics
- `sim/L0/mjcf/src/parser.rs` — modify (parse `<muscle>` element)

---

### 6. Sensor Completion
**Status:** Mostly complete | **Effort:** S (remaining work) | **Prerequisites:** #4 for tendon sensors

#### Current State
A correctness sub-spec (`sim/docs/SENSOR_FIXES_SPEC.md`) was implemented, addressing
rangefinder ray direction (+Z parity with MuJoCo), BallQuat/BallAngVel sensor types,
JointPos/JointVel restriction to hinge/slide, accelerometer comment/test hardening,
and bounds-checked `sensor_write` helpers across all evaluation functions.

Separately, Force, Torque, Touch, Rangefinder, Magnetometer, and SubtreeAngMom were
brought from stubs to full implementations. The remaining gaps are tendon-dependent
sensors blocked on #4:

| Sensor | Status | Detail |
|--------|--------|--------|
| Force | Done | Inverse dynamics via `compute_site_force_torque()` |
| Torque | Done | Inverse dynamics via `compute_site_force_torque()` |
| Touch | Done | Sums normal force from `efc_lambda` for matching contacts |
| Rangefinder | Done | Ray-cast along +Z with mesh support, -1.0 no-hit sentinel |
| Magnetometer | Done | `model.magnetic` transformed to sensor frame |
| SubtreeAngMom | Done | `compute_subtree_angmom()` about subtree COM |
| BallQuat | Done | Normalized quaternion from qpos, identity fallback |
| BallAngVel | Done | 3-DOF angular velocity from qvel |
| JointPos | Done | Hinge/slide only (Ball/Free no longer written) |
| JointVel | Done | Hinge/slide only (Ball/Free no longer written) |
| TendonPos | Stub (0.0) | Blocked on #4 — will read `ten_length[objid]` |
| TendonVel | Stub (0.0) | Blocked on #4 — will read `ten_velocity[objid]` |
| ActuatorPos | Partial | Joint transmission done; Tendon/Site stub (blocked on #4) |
| ActuatorVel | Partial | Joint transmission done; Tendon/Site stub (blocked on #4) |

All sensor writes use bounds-checked helpers (`sensor_write`, `sensor_write3`,
`sensor_write4`). Postprocessing handles positive-type sensors (Touch, Rangefinder)
with `.min(cutoff)` to preserve the -1.0 no-hit sentinel.

#### Remaining Work
The 4 remaining items are trivial once tendon pipeline (#4) lands:

1. **TendonPos:** Replace `0.0` stub with `ten_length[objid]`.
2. **TendonVel:** Replace `0.0` stub with `ten_velocity[objid]`.
3. **ActuatorPos (Tendon/Site):** Read tendon length × gear for tendon transmissions.
4. **ActuatorVel (Tendon/Site):** Read tendon velocity × gear for tendon transmissions.

#### Acceptance Criteria
1. ~~Force and Torque sensors return non-zero values when constraint forces are active.~~ Done.
2. ~~Touch sensor returns actual contact force magnitude (not `depth * 10000`).~~ Done.
3. ~~Rangefinder returns correct distance for a known geom placement (within 1e-6).~~ Done.
4. ~~Magnetometer returns the global magnetic field rotated into the site frame.~~ Done.
5. TendonPos/TendonVel return `ten_length`/`ten_velocity` when tendon pipeline (#4) is active.
6. ActuatorPos/ActuatorVel return correct values for all transmission types.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (4 stub sites in `mj_sensor_pos()` and `mj_sensor_vel()`)
- `sim/docs/SENSOR_FIXES_SPEC.md` — reference for completed correctness fixes

---
## Group C — Integrator & Dynamics

### 7. Integrator Rename (Implicit → ImplicitSpringDamper)
**Status:** ✅ Done | **Effort:** S | **Prerequisites:** None

#### Current State
The pipeline `Integrator` enum (`mujoco_pipeline.rs:628-638`) has three variants:
`Euler`, `RungeKutta4`, `Implicit`. The `Implicit` variant's doc comment has been
corrected to "Implicit Euler for diagonal per-DOF spring/damper forces"
(`mujoco_pipeline.rs:636`). The actual implementation in
`mj_fwd_acceleration_implicit()` (`mujoco_pipeline.rs:8790`) solves:

```
(M + h·D + h²·K)·v_new = M·v_old + h·f_ext - h·K·(q - q_eq)
```

This is implicit Euler for diagonal per-DOF spring stiffness K and damping D — not
implicit midpoint. Friction forces in `qfrc_passive` remain explicit. There is no
coupling between DOFs (K and D are diagonal).

#### Objective
Rename `Implicit` to `ImplicitSpringDamper` to accurately describe its scope. This
is a semantic-only change — no math changes. It removes a misleading name and creates
a clear slot for a future true `ImplicitEuler` variant that handles coupled
(non-diagonal) implicit forces.

#### Specification

1. Rename `Integrator::Implicit` → `Integrator::ImplicitSpringDamper` in the enum.
2. Doc comment is already correct ("Implicit Euler for diagonal per-DOF spring/damper
   forces") — no change needed.
3. Update all match arms and equality checks that reference `Integrator::Implicit`:
   - `mujoco_pipeline.rs:2343` — velocity update skip in `Data::integrate()`
   - `mujoco_pipeline.rs:6648` — `implicit_mode` flag in `mj_fwd_passive()`
   - `mujoco_pipeline.rs:8718` — acceleration dispatch in `mj_fwd_acceleration()`
4. Update MJCF layer:
   - `types.rs` — rename `MjcfIntegrator::Implicit` variant
   - `model_builder.rs:459` — update `MjcfIntegrator::Implicit` → Sim conversion
   - `parser.rs` — accept both `"implicit"` and `"implicitspringdamper"` strings
5. Update tests:
   - `parser.rs:2178` — parser test referencing `MjcfIntegrator::Implicit`
   - `implicit_integration.rs:61` — integration test assertion

#### Acceptance Criteria
1. `Integrator::Implicit` no longer exists — all references use `ImplicitSpringDamper`.
2. The doc comment accurately describes the method (diagonal spring/damper, not midpoint).
3. MJCF `<option integrator="implicit"/>` still parses correctly (backward compat).
4. All existing tests pass without modification (behavior is identical).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (enum definition, 3 match arms/equality checks)
- `sim/L0/mjcf/src/types.rs` — modify (`MjcfIntegrator` enum variant rename, `from_str`, `as_str`)
- `sim/L0/mjcf/src/config.rs` — modify (`From<MjcfIntegrator>` impl + its test)
- `sim/L0/mjcf/src/model_builder.rs` — modify (MJCF → Sim integrator conversion)
- `sim/L0/mjcf/src/parser.rs` — modify (integrator string parsing, parser test)
- `sim/L0/tests/integration/implicit_integration.rs` — modify (test assertion)
- `sim/docs/MUJOCO_REFERENCE.md` — modify (doc reference)

---

### 8. True RK4 Integration
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
`Integrator::RungeKutta4` in the pipeline dispatches to the same code path as
`Integrator::Euler` in both locations:

- `mj_fwd_acceleration()` (`mujoco_pipeline.rs:8258-8264`): both call
  `mj_fwd_acceleration_explicit()`.
- `Data::integrate()` (`mujoco_pipeline.rs:2321-2337`): both use the same
  semi-implicit Euler velocity/position update.

It is a pure placeholder — selecting RK4 produces identical results to Euler.

A separate `Integrator` trait in `integrators.rs` (1,005 lines, `integrators.rs:36`)
has a `RungeKutta4` struct (`integrators.rs:239`) that uses a constant-acceleration
reduction (equivalent to Velocity Verlet). This is not true multi-stage RK4; the
comment at `integrators.rs:248` acknowledges this: "For constant acceleration, RK4
reduces to..."

#### Objective
Replace the `RungeKutta4` placeholder with a true 4-stage Runge-Kutta integrator
that re-evaluates forces at intermediate states.

#### Specification

Implement `mj_fwd_acceleration_rk4(model: &Model, data: &mut Data) -> Result<(), StepError>`:

True 4-stage RK4 with force re-evaluation:

```
k1 = f(t,       q,            v)
k2 = f(t + h/2, q + h/2·v,   v + h/2·k1)
k3 = f(t + h/2, q + h/2·v,   v + h/2·k2)
k4 = f(t + h,   q + h·v,     v + h·k3)

v_new = v + (h/6)·(k1 + 2·k2 + 2·k3 + k4)
q_new = q + (h/6)·(v + 2·(v+h/2·k1) + 2·(v+h/2·k2) + (v+h·k3))
```

Each `f()` call evaluates forward kinematics, collision detection, and force
computation at the trial state (q, v). This is 4× the cost of semi-implicit Euler
per step but achieves O(h⁴) local truncation error vs O(h) for Euler.

**Implementation details:**
- Add scratch fields to `Data` for intermediate k-values and trial states
  (`scratch_qpos_rk`, `scratch_qvel_rk`, `scratch_qacc_k1..k4`).
- No heap allocation beyond what `Data` already provides — scratch fields are
  pre-allocated in `Model::make_data()`.
- `mj_fwd_acceleration()` dispatches `RungeKutta4` to the new function instead of
  falling through to `mj_fwd_acceleration_explicit()`.
- `Data::integrate()` skips the velocity update for RK4 (already done in
  `mj_fwd_acceleration_rk4()`), similar to how `Implicit` is handled.

#### Acceptance Criteria
1. `Integrator::RungeKutta4` no longer produces identical results to `Integrator::Euler`.
2. For smooth force fields (no contacts), RK4 with timestep 4h achieves comparable or better accuracy to Euler with timestep h (measured as position error vs analytic solution).
3. RK4 matches Euler forces at h→0 (consistency check).
4. Energy conservation for a frictionless pendulum is measurably better with RK4 than Euler at the same timestep.
5. No heap allocations in the RK4 path beyond pre-allocated `Data` scratch fields.
6. RK4 cost is ~4× Euler per step (verified by benchmark).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (`mj_fwd_acceleration_rk4()`, dispatch in `mj_fwd_acceleration()`, scratch fields in `Data`, skip velocity update in `Data::integrate()`)

---
## Group D — Deformable Body

### 9. Deformable Body Pipeline Integration
**Status:** Not started | **Effort:** XL | **Prerequisites:** None

#### Current State
sim-deformable is a standalone 7,733-line crate (86 tests):

| Component | Location | Description |
|-----------|----------|-------------|
| `XpbdSolver` | `solver.rs:134` | XPBD constraint solver. `step(&mut self, body: &mut dyn DeformableBody, gravity: Vector3<f64>, dt: f64)` (`solver.rs:196`). Configurable substeps, damping, sleeping. |
| `DeformableBody` trait | `lib.rs:173` | Common interface for Cloth, SoftBody, CapsuleChain. |
| `Cloth` | `cloth.rs` | Triangle meshes with distance + dihedral bending constraints. `thickness` field (`cloth.rs:50`). Presets: cotton, silk, leather, rubber, paper, membrane. |
| `SoftBody` | `soft_body.rs` | Tetrahedral meshes with distance + volume constraints. Presets: rubber, gelatin, soft tissue, muscle, foam, stiff. |
| `CapsuleChain` | `capsule_chain.rs` | 1D particle chains with distance + bending constraints. `radius` field (`capsule_chain.rs:41`). Presets: rope, steel cable, hair, chain. |
| `Material` | `material.rs` | Young's modulus, Poisson's ratio, density, `friction` (`material.rs:94`). 14 presets. |
| `ConstraintType::Collision` | `constraints.rs:42-43` | Enum variant defined but unimplemented — no constraint implements it. |
| `FlexEdge` | `constraints.rs` | Stretch, shear, twist constraint variants. |

The crate has zero coupling to the MuJoCo pipeline. sim-physics re-exports it behind
the `deformable` feature flag (`physics/src/lib.rs:109-110`). `Material.friction` and
per-body `radius`/`thickness` fields are declared but unused by the collision system
(which doesn't exist yet).

#### Objective
Deformable bodies interact with rigid bodies through the same contact solver and
step in the same simulation loop.

#### Specification

**Collision detection** (greenfield — no infrastructure exists):

1. **Broadphase:** Deformable vertex AABBs vs rigid geom AABBs. Vertex AABBs are
   point + radius/thickness margin. Use `batch_aabb_overlap_4()` (`simd/src/batch_ops.rs:251`)
   for batched broadphase queries.
2. **Narrowphase:** Vertex-vs-geom closest point computation. Produces `Contact` structs
   identical to rigid-rigid contacts (same normal, depth, friction, solref/solimp).
3. **Friction:** Combine `Material.friction` (deformable side) with geom friction
   (rigid side) using the same combination rule as rigid-rigid contacts.

**Contact solver coupling:**

Deformable-rigid contacts feed into PGS (or CG, per #3) alongside rigid-rigid
contacts. Each deformable vertex is a 3-DOF point mass. Its inverse mass comes from
the XPBD solver's per-particle mass. Contact Jacobians for deformable vertices are
3×3 identity blocks (point mass — no rotational DOFs).

**Force feedback:**

Contact impulses from PGS apply to deformable vertex velocities directly and to
rigid body `qfrc_constraint` through the standard Jacobian transpose. XPBD
constraint projection runs after contact resolution within the same timestep.

**Substep iteration (XPBD/contact ordering):**

A single pass (contact solve → XPBD) may leave contacts invalid because XPBD
constraint projection moves vertices after contacts are computed. For stiff
deformable bodies or deep penetrations, this causes jitter.

Options (to be chosen at implementation time):
- **Option A (simple):** Single pass, accept minor inaccuracy. Sufficient for cloth
  and rope where deformation is small relative to contact depth.
- **Option B (robust):** Iterate contact-detection + solve + XPBD for
  `n_substep_iterations` (default 1, configurable up to 4). Each iteration
  re-detects contacts at updated vertex positions. More expensive but handles
  stiff soft bodies contacting rigid surfaces.

The choice should be configurable per-model via an MJCF option.

**Pipeline integration in `Data::step()`:**

```
1. Rigid: forward kinematics, collision, forces
2. Deformable-rigid collision detection → Contact list
3. Combined contact solve (rigid + deformable contacts)
4. Apply contact impulses to rigid bodies and deformable vertices
5. XpbdSolver::step() for each registered deformable body
6. (Optional) Repeat steps 2-5 for substep iterations > 1
7. Rigid: position integration
```

#### Acceptance Criteria
1. A rigid body resting on a deformable surface experiences the same contact forces as resting on a rigid surface of equivalent geometry.
2. XPBD internal constraints (distance, bending, volume) are satisfied after contact resolution — contact forces do not violate deformable material properties.
3. `Material.friction` is used in deformable-rigid contacts (not hardcoded).
4. `ConstraintType::Collision` is implemented in the constraint system.
5. Zero-deformable-body configurations have zero overhead (no broadphase, no substep).
6. Cloth draped over a rigid sphere reaches stable equilibrium without jitter.
7. Substep iteration count is configurable; default (1) works for cloth/rope use cases.

#### Files
- `sim/L0/deformable/src/` — modify (collision detection, `ConstraintType::Collision` implementation)
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (pipeline integration in `Data::step()`, deformable body registration)
- `sim/L0/simd/src/batch_ops.rs` — reference (`batch_aabb_overlap_4()`)

---
## Group E — Scaling & Performance

### 10. Batched Simulation
**Status:** Not started | **Effort:** L | **Prerequisites:** None

#### Current State
Single-environment execution. `Data::step(&mut self, &Model)` (`mujoco_pipeline.rs:2228`)
steps one simulation. `Model` is immutable after construction, uses
`Arc<TriangleMeshData>` for shared mesh data (`mujoco_pipeline.rs:843`). `Data` is
fully independent — no shared mutable state, no interior mutability, derives `Clone`
(`mujoco_pipeline.rs:1232`).

#### Objective
Step N independent environments in parallel on CPU. Foundation for GPU acceleration
(#11) and large-scale RL training.

#### Specification

```rust
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
sim-core declares it optional under the `parallel` feature flag
(`core/Cargo.toml:19,33`).

```rust
pub struct BatchResult {
    pub states: DMatrix<f64>,       // (n_envs, nq + nv) row-major
    pub rewards: DVector<f64>,      // (n_envs,)
    pub terminated: Vec<bool>,      // per-env episode termination
    pub truncated: Vec<bool>,       // per-env time limit
    pub errors: Vec<Option<StepError>>, // None = success
}
```

`states` is a contiguous matrix for direct consumption by RL frameworks (numpy
interop via row-major layout). Reward computation is user-defined:

```rust
pub trait RewardFn: Send + Sync {
    fn compute(&self, model: &Model, data: &Data) -> f64;
}
```

**Design constraint — single Model:** All environments share the same `Arc<Model>`
(same nq, nv, geom set). The `states: DMatrix<f64>` layout (n_envs, nq + nv)
requires uniform state dimensions. Multi-model batching (different robots in the
same batch) would require a fundamentally different design and is explicitly
out of scope.

**Error handling:** Environments that fail (e.g., `CholeskyFailed`,
`SingularMassMatrix`) are recorded in `errors`, flagged in `terminated`, and
auto-reset on the next `step_all()` call. The batch never aborts due to a single
environment failure.

**SIMD integration:** sim-simd provides within-environment acceleration:
`batch_dot_product_4()`, `batch_aabb_overlap_4()`, `batch_normal_force_4()`,
`batch_friction_force_4()`, `batch_integrate_position_4()`,
`batch_integrate_velocity_4()`. These accelerate the inner loop of each
environment's step. Cross-environment parallelism comes from rayon, not SIMD.

#### Acceptance Criteria
1. `BatchSim::step_all()` produces identical results to calling `Data::step()` on each environment sequentially — parallelism does not change simulation output.
2. `states` matrix layout is stable: row = env, cols = qpos ++ qvel.
3. Failed environments do not affect healthy environments in the same batch.
4. Linear throughput scaling up to available CPU cores (verified by benchmark with 1, 2, 4, 8 threads).
5. Zero-copy state extraction — `states` is filled directly from `Data` fields without intermediate allocations.
6. `reset_where()` resets only flagged environments without touching others.

#### Files
- `sim/L0/core/src/` — create `batch.rs` module
- `sim/L0/core/Cargo.toml` — modify (enable rayon under `parallel` feature)

---

### 11. GPU Acceleration
**Status:** Not started | **Effort:** XL | **Prerequisites:** #10

#### Current State
CPU-only. No GPU infrastructure exists in the simulation pipeline. The `mesh-gpu`
crate provides wgpu context and buffer management for rendering but has no compute
shader infrastructure.

#### Objective
wgpu compute shader backend for batch simulation. Thousands of parallel environments
on a single GPU for RL training at scale.

#### Specification

Port the inner loop of `Data::step()` (FK, collision, PGS, integration) to compute
shaders via wgpu. The `BatchSim` API from #10 defines the memory layout that the
GPU backend fills.

This item is intentionally kept sparse — it is blocked on #10 and should not be
over-specified until the CPU batching API stabilizes. Key design decisions to be
made at implementation time:

- Which pipeline stages move to GPU first (integration is simplest, collision is
  most impactful).
- Data transfer strategy (host-pinned memory, persistent GPU buffers, double-buffering).
- Whether to use wgpu compute shaders or a lower-level API (Vulkan compute, Metal).
- Fallback path for systems without GPU support.

#### Acceptance Criteria
1. GPU-batched simulation produces identical results to CPU-batched (#10) for the same inputs.
2. For ≥ 1024 environments, GPU throughput exceeds CPU throughput on supported hardware.
3. Graceful fallback to CPU batching when no GPU is available.

#### Files
- `sim/L0/core/src/` — create `gpu.rs` module or new `sim-gpu` crate
- `mesh-gpu/` — reference for wgpu context management

---
## Cleanup Tasks

### C1. Disambiguate integrators.rs
**Effort:** S

The `Integrator` trait in `integrators.rs` (`integrators.rs:36`, 1,005 lines) defines
6 implementations (`ExplicitEuler`, `SemiImplicitEuler`, `VelocityVerlet`,
`RungeKutta4`, `ImplicitVelocity`, `ImplicitFast`) that operate on single
`RigidBodyState` objects via an `IntegrationMethod` enum from sim-types.

The pipeline uses its own `Integrator` enum (`mujoco_pipeline.rs:623`) with
completely separate integration logic in `Data::integrate()` (`mujoco_pipeline.rs:2321`).
The two systems share no code. `integrate_with_method()` is re-exported by sim-physics
(`physics/src/lib.rs:167`) but has zero callers in the pipeline.

**Decision needed:** Either:
- **(a) Remove** the trait system entirely (it's dead code with respect to the pipeline), or
- **(b) Consolidate** by having `Data::integrate()` delegate to the trait system
  (requires adapting the trait to work with `Data` instead of `RigidBodyState`).

Option (a) is simpler and removes 1,005 lines of unused code. Option (b) is only
warranted if users of sim-physics need standalone integration outside the pipeline.

### C2. ~~Correct stale MUJOCO_GAP_ANALYSIS.md~~ ✅ DONE

GAP_ANALYSIS.md was corrected: overall completion revised to ~65-70%, false claims
about Newton solver / PGS-SOR / island discovery / parallel solving removed, sensor
and tendon statuses marked as stubs. World/Stepper code samples replaced with
Model/Data API. Stale file references (world.rs, stepper.rs, broad_phase.rs, loader.rs)
annotated or updated. Sleeping bodies status corrected from "Implemented" to
"Not implemented". solref/solimp marked as implemented.

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
