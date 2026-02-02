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
`mj_fwd_acceleration_implicit()` (`mujoco_pipeline.rs:8969`) uses `std::mem::replace`
to swap `scratch_m_impl` out for nalgebra's `.cholesky()` (which takes ownership),
replacing it with a freshly allocated `DMatrix::zeros(nv, nv)` every step
(`mujoco_pipeline.rs:9019`). For nv=100 this is ~78 KB/step of unnecessary heap
allocation and zeroing.

The current flow (`mujoco_pipeline.rs:9019-9025`):
```rust
let chol = std::mem::replace(&mut data.scratch_m_impl, DMatrix::zeros(nv, nv))
    .cholesky()
    .ok_or(StepError::CholeskyFailed)?;
data.scratch_v_new.copy_from(&data.scratch_rhs);
chol.solve_mut(&mut data.scratch_v_new);
```

`scratch_m_impl` is pre-allocated in `make_data()` (`mujoco_pipeline.rs:1785`) and
populated each step via `copy_from(&data.qM)` (`mujoco_pipeline.rs:8993`) which
overwrites the full matrix. After Cholesky, the matrix is never read again until
the next step's `copy_from`. The replacement zero matrix is therefore wasted — the
next `copy_from` would overwrite it entirely regardless of contents.

`StepError::CholeskyFailed` already exists (`mujoco_pipeline.rs:654`).

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

The `Cholesky` import stayed at the time of #1 — it was later removed by #2 along with the
`qM_cholesky` field (replaced by sparse `qLD_diag`/`qLD_L` factorization).

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
| `mj_fwd_acceleration_explicit()` | `mujoco_pipeline.rs:8779` | `data.qM_cholesky` (nalgebra) | O(nv²) solve, O(nv³) factorize |
| `mj_fwd_acceleration_implicit()` | `mujoco_pipeline.rs:8969` | `cholesky_in_place()` on M_impl | O(nv³) factorize + solve |
| `pgs_solve_contacts()` | `mujoco_pipeline.rs:7081` | `data.qM_cholesky` (nalgebra) | O(nv²) solve, O(nv³) factorize |

For tree-structured robots, `M[i,j] ≠ 0` only when DOF `j` is an ancestor of DOF `i`
(or vice versa) in the kinematic tree encoded by `model.dof_parent: Vec<Option<usize>>`
(`mujoco_pipeline.rs:795`). This tree sparsity enables O(nv) factorization and solve.

**Data scaffolds** exist in `Data` (`mujoco_pipeline.rs:1332-1339`):
```rust
pub qLD_diag: DVector<f64>,                  // D[i,i] diagonal
pub qLD_L: Vec<Vec<(usize, f64)>>,           // L[i,:] sparse off-diagonal per row
pub qLD_valid: bool,                         // dispatch flag
```
Initialized at `mujoco_pipeline.rs:1742-1744` (zeros/empty/false), never populated.
The `TODO(FUTURE_WORK#2)` comment was at `mujoco_pipeline.rs:1327` (removed when #2 was completed).

The `qM_cholesky: Option<Cholesky<f64, Dyn>>` (removed when #2 was completed) was computed
in `mj_crba()` Phase 5 (`mujoco_pipeline.rs:6049`) via `data.qM.clone().cholesky()` —
this clone+factorize was the O(nv³) cost eliminated for the explicit/contact paths.

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

At `mujoco_pipeline.rs:6049-6050`, replace the nalgebra Cholesky with sparse factorization:

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

##### Step 4: Dispatch in `mj_fwd_acceleration_explicit()` (line 8779)

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

##### Step 5: Migrate `pgs_solve_contacts()` (line 7081)

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

##### Step 6: Dispatch in `mj_fwd_acceleration_implicit()` (line 8969)

The implicit path uses `M_impl = M + h·D + h²·K`. The diagonal modification
(`+= h*d[i] + h²*k[i]` at line 8995) means we **cannot** reuse the CRBA sparse
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
The pipeline uses PGS via `pgs_solve_contacts()` (`mujoco_pipeline.rs:7081`) for contact
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
   (from `compute_contact_jacobian()` at `mujoco_pipeline.rs:6901`).
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
`<option solver="CG"/>`. `mj_fwd_constraint()` (`mujoco_pipeline.rs:8500`) dispatches
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

Data scaffolds exist (`mujoco_pipeline.rs:1362-1371`): `ten_length`, `ten_velocity`,
`ten_force`, `ten_J` — all initialized to defaults at `mujoco_pipeline.rs:1756-1759`
and never populated.

`mj_fwd_actuation()` (`mujoco_pipeline.rs:5997`) has an explicit placeholder at
`mujoco_pipeline.rs:6017-6020`:
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

**Tendon actuation** (fills the placeholder at `mujoco_pipeline.rs:6017`):

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

The pipeline declares `ActuatorDynamics::Muscle` (`mujoco_pipeline.rs:435`) and
`data.act` (`mujoco_pipeline.rs:1255`) but `mj_fwd_actuation()` has no muscle-specific
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
The pipeline `Integrator` enum (`mujoco_pipeline.rs:629-637`) has three variants:
`Euler`, `RungeKutta4`, `Implicit`. The `Implicit` variant's doc comment has been
corrected to "Implicit Euler for diagonal per-DOF spring/damper forces"
(`mujoco_pipeline.rs:636`). The actual implementation in
`mj_fwd_acceleration_implicit()` (`mujoco_pipeline.rs:8969`) solves:

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
   - `mujoco_pipeline.rs:2403` — velocity update skip in `Data::integrate()`
   - `mujoco_pipeline.rs:6707` — `implicit_mode` flag in `mj_fwd_passive()`
   - `mujoco_pipeline.rs:8768` — acceleration dispatch in `mj_fwd_acceleration()`
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
**Status:** ✅ Done | **Effort:** M | **Prerequisites:** None

#### Current State
`Integrator::RungeKutta4` (`mujoco_pipeline.rs:629`) dispatches to the same code
path as `Integrator::Euler` in both locations:

- `mj_fwd_acceleration()` (`mujoco_pipeline.rs:8762`): both call
  `mj_fwd_acceleration_explicit()`.
- `Data::integrate()` (`mujoco_pipeline.rs:2391`): both use the same
  semi-implicit Euler velocity/position update.

It is a pure placeholder — selecting RK4 produces identical results to Euler.

A separate `Integrator` trait in `integrators.rs` (`integrators.rs:36`) has a
`RungeKutta4` struct (`integrators.rs:239`) that uses a constant-acceleration
reduction (equivalent to Velocity Verlet). This is not true multi-stage RK4; the
comment at `integrators.rs:248` acknowledges this. That trait system is **not used
by the MuJoCo pipeline** (see FUTURE_WORK C1).

#### Objective
Replace the `RungeKutta4` placeholder with a true 4-stage Runge-Kutta integrator
that calls the full forward dynamics pipeline (including collision detection) at each
intermediate stage, matching MuJoCo's `mj_RungeKutta` in `engine_forward.c`.

#### Reference: MuJoCo's Algorithm

MuJoCo's RK4 (`mj_RungeKutta` in `engine_forward.c`) uses the standard Butcher
tableau. Stage 0 reuses the qacc from the preceding `mj_forward()` call. At each
subsequent stage (1-3), it calls `mj_forwardSkip(m, d, mjSTAGE_NONE, 1)` — the full
pipeline including collision detection, but skipping sensor evaluation. Positions are
integrated via `mj_integratePos` (quaternion exponential map on SO(3)), not linear
addition. The state at each stage always starts from `X[0]` (initial state), not
from the previous stage's state.

#### Specification

##### Butcher Tableau Constants

```
RK4_A: [f64; 9] = [0.5, 0.0, 0.0,   // Stage 1→2: dX = 0.5 * F[0]
                    0.0, 0.5, 0.0,   // Stage 2→3: dX = 0.5 * F[1]
                    0.0, 0.0, 1.0]   // Stage 3→4: dX = 1.0 * F[2]

RK4_B: [f64; 4] = [1.0/6.0, 1.0/3.0, 1.0/3.0, 1.0/6.0]  // Final combination

Indexing: A[(i-1)*3 + j] for stage i ∈ {1,2,3}, coefficient j ∈ {0,1,2}.
```

##### Data Struct Additions

Add after `scratch_v_new` (`mujoco_pipeline.rs:1427`), before `body_min_mass`:

```rust
// ==================== RK4 Scratch Buffers ====================
// State and rate buffers for 4-stage Runge-Kutta integration.
// Pre-allocated in make_data() — no heap allocation during stepping.

/// Saved initial position for RK4: X[0].qpos (length nq).
/// Read at every stage (step 2c) and the final advance (step 4).
pub rk4_qpos_saved: DVector<f64>,

/// Scratch position for the current RK4 stage (length nq).
/// Written at step 2c, copied to data.qpos at step 2e, then overwritten
/// next iteration. Intermediate positions are not needed for the final
/// B-weight combination (which only sums velocities and accelerations).
pub rk4_qpos_stage: DVector<f64>,

/// RK4 stage velocities: X[i].qvel (4 buffers, each length nv).
/// All 4 are needed for the final B-weight combination in step 3.
pub rk4_qvel: [DVector<f64>; 4],

/// RK4 stage accelerations: F[i].qacc (4 buffers, each length nv).
/// All 4 are needed for the final B-weight combination in step 3.
pub rk4_qacc: [DVector<f64>; 4],

/// Weighted velocity sum for manifold position integration (length nv).
pub rk4_dX_vel: DVector<f64>,

/// Weighted acceleration sum for velocity update (length nv).
pub rk4_dX_acc: DVector<f64>,
```

In `Model::make_data()` (after line 1788), add:

```rust
rk4_qpos_saved: DVector::zeros(self.nq),
rk4_qpos_stage: DVector::zeros(self.nq),
rk4_qvel: std::array::from_fn(|_| DVector::zeros(self.nv)),
rk4_qacc: std::array::from_fn(|_| DVector::zeros(self.nv)),
rk4_dX_vel: DVector::zeros(self.nv),
rk4_dX_acc: DVector::zeros(self.nv),
```

Memory overhead: `2*nq + 4*2*nv + 2*nv` = `2*nq + 10*nv` floats. For a 30-DOF
humanoid (nq≈37, nv=30): 374 f64 = 2.9 KiB. Negligible.

##### New Method: `Data::forward_skip_sensors()`

Identical to `forward()` (`mujoco_pipeline.rs:2300`) but skips all 4 sensor
stages (`mj_sensor_pos`, `mj_sensor_vel`, `mj_sensor_acc`, `mj_sensor_postprocess`).
Matches MuJoCo's `mj_forwardSkip(m, d, mjSTAGE_NONE, 1)` where `1` = skip sensors.

```rust
fn forward_skip_sensors(&mut self, model: &Model) -> Result<(), StepError> {
    mj_fwd_position(model, self);
    mj_collision(model, self);
    // skip: mj_sensor_pos
    mj_energy_pos(model, self);
    mj_fwd_velocity(model, self);
    // skip: mj_sensor_vel
    mj_fwd_actuation(model, self);
    mj_crba(model, self);
    mj_rne(model, self);
    mj_energy_vel(model, self);
    mj_fwd_passive(model, self);
    mj_fwd_constraint(model, self);
    mj_fwd_acceleration(model, self)?;
    // skip: mj_sensor_acc, mj_sensor_postprocess
    Ok(())
}
```

##### New Function: `mj_runge_kutta()`

Core RK4 implementation (~90 lines). Free function matching existing patterns.

**Algorithm** (matching MuJoCo's `mj_RungeKutta`):

```
fn mj_runge_kutta(model: &Model, data: &mut Data) -> Result<(), StepError>

Precondition: data.forward() has already been called (qacc is valid).

1. SAVE initial state:
   rk4_qpos_saved.copy_from(&data.qpos)
   X[0].qvel = data.qvel
   F[0].qacc = data.qacc
   t0 = data.time
   efc_lambda_saved = data.efc_lambda.clone()

2. FOR i = 1, 2, 3:
   a. Weighted velocity:  dX_vel[v] = Σ_{j=0}^{2} A[(i-1)*3+j] * X[j].qvel[v]
   b. Weighted accel:     dX_acc[v] = Σ_{j=0}^{2} A[(i-1)*3+j] * F[j].qacc[v]
      (For this tableau, only j = i−1 has a non-zero coefficient.)
   c. Position (manifold): mj_integrate_pos_explicit(model, &mut rk4_qpos_stage, &rk4_qpos_saved, &dX_vel, h)
   d. Velocity (linear):   X[i].qvel = X[0].qvel + h * dX_acc
   e. Set Data state:      data.qpos.copy_from(&rk4_qpos_stage), data.qvel.copy_from(&X[i].qvel)
   f. Set Data time:       data.time = t0 + h * {0.5, 0.5, 1.0}[i-1]
   g. Evaluate:            data.forward_skip_sensors(model)?
   h. Store rate:          F[i].qacc = data.qacc

3. FINAL combination using B weights:
   dX_vel[v] = Σ_j B[j] * X[j].qvel[v]
   dX_acc[v] = Σ_j B[j] * F[j].qacc[v]

4. ADVANCE from initial state:
   data.qacc_warmstart = data.qacc  (save stage 3 qacc for next step — matches MuJoCo)
   data.qvel.copy_from(&X[0].qvel)    (restore)
   data.qvel += h * dX_acc             (velocity update)
   mj_integrate_pos_explicit(model, &mut data.qpos, &rk4_qpos_saved, &dX_vel, h)  (position on manifold)
   mj_normalize_quat(model, data)
   data.efc_lambda = efc_lambda_saved  (restore warmstart from initial solve)
   data.time = t0 + h

   NOTE: After mj_runge_kutta() returns, all derived quantities are stale
   from stage 3 and do NOT correspond to the final (qpos, qvel) state:
   - qacc (stage 3 acceleration, not final-state acceleration)
   - xpos, xquat, xmat (body poses at stage 3 trial state)
   - geom_xpos, geom_xmat, site_xpos, site_xmat (geometry/site poses)
   - contacts, ncon (contact set from stage 3 collision detection)
   - qfrc_actuator, qfrc_bias, qfrc_passive, qfrc_constraint (forces)
   - energy_potential, energy_kinetic
   This matches MuJoCo's behavior — mj_step does NOT call mj_forward
   after mj_RungeKutta. Users needing consistent derived quantities at
   the advanced state should call data.forward(model) after step().
```

**Key design points:**
- Position integration uses `mj_integrate_pos_explicit()` (`mujoco_pipeline.rs:9332`)
  at every stage, which handles quaternion exponential map for ball/free joints.
  The A coefficients are already baked into `dX_vel`, so `h` (not `h*A[i]`) is passed
  as the timestep parameter.
- Each stage starts from `X[0]` (initial state), not from the previous stage. This
  matches MuJoCo and ensures Butcher tableau correctness.
- **Borrow-checker note:** Step 2d reads `rk4_qvel[0]` while writing `rk4_qvel[i]`.
  Rust's borrow checker cannot prove disjointness of runtime-indexed array elements.
  Use `split_at_mut(1)` on the `rk4_qvel` array at the top of each loop iteration
  to split into `(head, tail)`, giving simultaneous `&head[0]` and `&mut tail[i-1]`.
  This is zero-cost and idiomatic. Position buffers don't need this treatment —
  `rk4_qpos_saved` and `rk4_qpos_stage` are separate struct fields with independent
  borrows.
- `efc_lambda` (warmstart HashMap, `mujoco_pipeline.rs:1400`) is saved before the
  loop and restored after the final advance. Intermediate stages may modify it
  freely (the PGS solver benefits from warmstarting), but the persistent warmstart
  across timesteps comes from the initial solve.
- `qacc_warmstart` is saved **before** restoring X[0], so it captures stage 3's
  raw qacc. This matches MuJoCo's `mj_advance` behavior (`mju_copy(d->qacc_warmstart,
  d->qacc, nv)`). Our codebase does not currently read `qacc_warmstart` (the PGS
  solver uses `efc_lambda` instead), but saving it preserves forward compatibility
  with MuJoCo's acceleration-based warmstart pattern.
- The one heap allocation is `efc_lambda.clone()`. For typical contact counts (<100
  pairs): ~2.4 KiB, ~100ns. If profiling shows this matters, add a pre-allocated
  `efc_lambda_saved` field to `Data`.

##### Changes to `Data::step()`

Replace the current implementation (`mujoco_pipeline.rs:2257`):

```rust
pub fn step(&mut self, model: &Model) -> Result<(), StepError> {
    if model.timestep <= 0.0 || !model.timestep.is_finite() {
        return Err(StepError::InvalidTimestep);
    }
    mj_check_pos(model, self)?;
    mj_check_vel(model, self)?;

    match model.integrator {
        Integrator::RungeKutta4 => {
            // RK4: forward() evaluates initial state (with sensors).
            // mj_runge_kutta() then calls forward_skip_sensors() 3 more times.
            self.forward(model)?;
            mj_check_acc(model, self)?;
            mj_runge_kutta(model, self)?;
        }
        Integrator::Euler | Integrator::ImplicitSpringDamper => {
            self.forward(model)?;
            mj_check_acc(model, self)?;
            self.integrate(model);
        }
    }
    Ok(())
}
```

##### Changes to `Data::integrate()`

Remove `RungeKutta4` from the `Euler` arm (`mujoco_pipeline.rs:2391`):

```rust
match model.integrator {
    Integrator::Euler => {
        for i in 0..model.nv {
            self.qvel[i] += self.qacc[i] * h;
        }
    }
    Integrator::ImplicitSpringDamper => {
        // Velocity already updated by mj_fwd_acceleration_implicit
    }
    Integrator::RungeKutta4 => {
        unreachable!("RK4 integration handled by mj_runge_kutta()")
    }
}
```

##### No Changes to `mj_fwd_acceleration()`

`mj_fwd_acceleration()` (`mujoco_pipeline.rs:8762`) continues to dispatch
`Euler | RungeKutta4` to `mj_fwd_acceleration_explicit()`. Each forward evaluation
within the RK4 loop computes `qacc = M^{-1} * f` in the standard way; the difference
is that RK4 calls it 4 times at different states.

##### Activation State (Deferred)

MuJoCo's `mj_RungeKutta` includes actuator activation state (`act`, `act_dot`) in
its state and rate vectors. Our `Data.act` exists (`mujoco_pipeline.rs:1255`) but
`act_dot` does not — `mj_fwd_actuation()` (`mujoco_pipeline.rs:5997`) is a stateless
`ctrl * gear` function with no activation dynamics. Activation integration is deferred
until FUTURE_WORK #5 (Muscle Pipeline) introduces `act_dot`. A `TODO(FUTURE_WORK#5)`
comment should mark the extension points in `mj_runge_kutta()`.

##### Sensor Evaluation

Sensors are evaluated once per `step()` call:
1. `step()` calls `forward()` (with sensors) before `mj_runge_kutta()`.
2. The 3 intermediate evaluations use `forward_skip_sensors()`.
3. After `mj_runge_kutta()` returns, `data.sensordata` contains values from the
   pre-step state. This matches MuJoCo's behavior.

#### Acceptance Criteria
1. `Integrator::RungeKutta4` no longer produces identical results to `Integrator::Euler`.
2. **Order verification (smooth systems):** For a frictionless pendulum with analytic
   solution, run RK4 at timestep `h` and Euler at timestep `h`. Compute position
   error at `t = 1s`. Halving `h` should reduce RK4 error by ~16× (O(h⁴)) vs ~2–4×
   for symplectic Euler (O(h)–O(h²) in position depending on observable and system
   structure; symplectic Euler preserves energy well but position convergence is
   typically first-order). Test at `h = 0.01` and `h = 0.005`.
3. **Consistency:** `lim(h→0)` the RK4 and Euler trajectories converge. Verify
   using the same pendulum model as AC #2 at `h = 1e-5` for 10 steps: max position
   difference < 1e-8.
4. **Energy conservation:** For a frictionless pendulum (no contacts) at `h = 0.001`,
   RK4 energy drift after 1000 steps (1.0 s simulated time) shall be less than
   `1e-10` relative to initial energy. Additionally, halving `h` should reduce RK4
   energy drift by ~32×–64× (O(h⁵)–O(h⁶) secular drift on near-harmonic systems),
   verified at `h = 0.002` and `h = 0.001`.
5. **Contact handling:** Bouncing-ball test produces physically plausible results
   (ball does not explode or tunnel, contacts are detected). Energy remains bounded.
   Note: the PGS constraint solver without restitution absorbs kinetic energy on
   impact, so the ball settles rather than bouncing — this is correct behavior.
6. **Quaternion correctness:** A free-body rotation with known angular velocity
   produces the correct final orientation (error < 1e-6 vs analytic) after 100 RK4
   steps. Quaternion norm stays within 1e-10 of 1.0 at every step.
7. **No new heap allocations** introduced by `mj_runge_kutta()` itself beyond the
   one `efc_lambda.clone()`. (The forward pipeline already allocates per call — e.g.,
   force accumulation in `mj_fwd_acceleration_explicit`, Jacobians and lambda vectors
   in `pgs_solve_contacts` — but these are inherited, not new to RK4.)
8. **Cost:** RK4 wall-clock time is 3.5×–4.5× Euler per step (verified by benchmark
   on a 10-body chain).
9. **Sensors:** After `step()` with RK4, `data.sensordata` contains values from the
   pre-step state. Intermediate stages do not corrupt sensor data.
10. **Warmstart preservation:** `efc_lambda` after `step()` reflects the initial
    state's contact solve, not stage 3's. Verify by comparing `efc_lambda` before
    and after one RK4 step: call `forward()` → clone `efc_lambda` → call
    `mj_runge_kutta()` → assert `efc_lambda` is bitwise identical to the clone.
    This directly tests the save/restore mechanism without requiring steady-state
    warmup or cross-step comparison.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify:
  - `Data` struct: add 6 RK4 scratch fields (`rk4_qpos_saved`, `rk4_qpos_stage`,
    `rk4_qvel`, `rk4_qacc`, `rk4_dX_vel`, `rk4_dX_acc`) after `scratch_v_new`
    (line 1427)
  - `Model::make_data()`: allocate RK4 scratch buffers (after line 1788)
  - `Data::step()`: add `RungeKutta4` match arm (line 2257)
  - `Data::integrate()`: remove `RungeKutta4` from `Euler` arm (line 2391)
  - New `Data::forward_skip_sensors()`: forward pipeline minus sensor stages
  - New `mj_runge_kutta()`: core RK4 implementation (~90 lines)
- `sim/L0/tests/integration/rk4_integration.rs` — new test module:
  - Pendulum order-of-convergence test (AC #2)
  - Euler/RK4 consistency test (AC #3)
  - Energy conservation test (AC #4)
  - Quaternion free-body rotation test (AC #6)
  - Contact bouncing-ball test (AC #5)
  - Sensor non-corruption test (AC #9)
  - Warmstart preservation test (AC #10)
- `sim/L0/tests/integration/mod.rs` — register `rk4_integration` module

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
Single-environment execution. `Data::step(&mut self, &Model)` (`mujoco_pipeline.rs:2257`)
steps one simulation. `Model` is immutable after construction, uses
`Arc<TriangleMeshData>` for shared mesh data (`mujoco_pipeline.rs:849`). `Data` is
fully independent — no shared mutable state, no interior mutability, derives `Clone`
(`mujoco_pipeline.rs:1240`).

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

### C1. ~~Remove Dead Standalone Integrator System~~ ✅ DONE
**Status:** Complete | **Effort:** S | **RL Impact:** Low | **Correctness:** Medium | **Prerequisites:** None

#### Current State

Two completely independent integrator systems coexist in the codebase. They share
no code, no types, and no callers.

**System A — Standalone trait system** (`sim/L0/core/src/integrators.rs`, 1,005 lines):

| Type | Location | Description |
|------|----------|-------------|
| `Integrator` trait | `integrators.rs:36` | `fn integrate(state: &mut RigidBodyState, ...)` |
| `ExplicitEuler` | `integrators.rs:137` | First-order explicit |
| `SemiImplicitEuler` | `integrators.rs:171` | Symplectic Euler |
| `VelocityVerlet` | `integrators.rs:204` | Second-order symplectic |
| `RungeKutta4` | `integrators.rs:239` | Constant-acceleration reduction (not true multi-stage) |
| `ImplicitVelocity` | `integrators.rs:296` | Implicit-in-velocity with damping |
| `ImplicitFast` | `integrators.rs:384` | Implicit, skips Coriolis terms |
| `integrate_with_method()` | `integrators.rs:54` | Dispatch by `IntegrationMethod` enum |
| `integrate_with_method_and_damping()` | `integrators.rs:90` | Dispatch with damping support |
| `apply_damping()` | `integrators.rs:480` | Post-integration damping |
| `clamp_velocities()` | `integrators.rs:490` | Velocity clamping |
| Tests | `integrators.rs:508-1005` | ~500 lines of internal unit tests |

These operate on `RigidBodyState` (a single-body type from sim-types) via
`IntegrationMethod` enum dispatch. The pipeline never touches `RigidBodyState`.

**System B — Pipeline integrator** (`mujoco_pipeline.rs:629`):

```rust
pub enum Integrator {
    Euler,                  // Semi-implicit Euler
    RungeKutta4,            // True 4-stage RK4 (mj_runge_kutta)
    ImplicitSpringDamper,   // Diagonal spring/damper implicit Euler
}
```

This enum drives `Data::step()` (`mujoco_pipeline.rs:2257`), `Data::integrate()`
(`mujoco_pipeline.rs:2391`), `mj_fwd_acceleration()` (`mujoco_pipeline.rs:8768`),
and `mj_fwd_passive()` (`mujoco_pipeline.rs:6707`). It operates on the full
generalized-coordinate state (`qpos`/`qvel`/`qacc`) with quaternion-aware position
integration via `mj_integrate_pos_explicit()`.

**Callers of System A: zero.**

Verified by exhaustive grep. No code outside `integrators.rs` itself calls any
function from the module. Specifically:

| Potential caller | Status |
|------------------|--------|
| `mujoco_pipeline.rs` | Zero imports from `crate::integrators`. Uses own `Integrator` enum. |
| `sim/L0/tests/integration/` | All tests use `Data::step()` / pipeline. None import `integrators::`. |
| `sim/L0/core/src/lib.rs:78` | Declares `pub mod integrators` — export only, no calls. |
| `sim/L0/physics/src/lib.rs:165-168` | Re-exports to prelude — export only, no calls. |
| `sim/L0/core/benches/` | Uses pipeline `Model`/`Data`. No integrator trait calls. |

**Orphaned supporting types:**

The standalone system depends on `IntegrationMethod` from sim-types
(`types/src/config.rs:387-418`), a 6-variant enum with helper methods (`order()`,
`is_symplectic()`, `is_implicit()`, `skips_coriolis()`, `relative_cost()`,
`Display` impl) totaling ~90 lines. This enum is also stored in
`SolverConfig.integration` (`types/src/config.rs:153`).

`IntegrationMethod` has exactly 3 consumers:

1. `integrators.rs:33` — dispatches on it (dead).
2. `mjcf/src/config.rs:12-21` — `From<MjcfIntegrator> for IntegrationMethod` conversion,
   feeding `SolverConfig` via `SimulationConfig::from(&MjcfOption)`. But the pipeline
   never reads `SimulationConfig` or `SolverConfig` (`mujoco_pipeline.rs` has zero
   references to either type). The pipeline uses a separate
   `From<MjcfIntegrator> for Integrator` in `model_builder.rs:457-460`.
3. `types/src/config.rs` — enum definition and self-tests.

The `From<MjcfIntegrator> for IntegrationMethod` impl and associated test
(`config.rs:219-236`) will become dead code once `integrators.rs` is removed,
since nothing reads the `IntegrationMethod` field from `SolverConfig`.

**Name collision:** The standalone `Integrator` *trait* and the pipeline `Integrator`
*enum* collide. The sim-physics prelude resolves this with
`Integrator as MjIntegrator` (`physics/src/lib.rs:162`). Removing the trait
eliminates the collision and the alias.

#### Objective

Remove the dead standalone integrator system and its orphaned dependencies. One
integrator system (the pipeline's `Integrator` enum) remains as the single source
of truth.

#### Decision: Option (a) — Remove

Option (b) (consolidating by having `Data::integrate()` delegate to the trait) was
evaluated and rejected:

- The trait operates on `RigidBodyState` (position + velocity of a single rigid
  body). The pipeline operates on generalized coordinates (`qpos`/`qvel` vectors
  spanning all joints) with quaternion-aware manifold integration. The type systems
  are fundamentally incompatible — adapting the trait would require rewriting it
  from scratch, at which point it's not consolidation.
- The pipeline's RK4 is true multi-stage (4 calls to `forward_skip_sensors()`).
  The standalone `RungeKutta4` is a constant-acceleration reduction. Consolidation
  would mean deleting the standalone version anyway.
- The pipeline's implicit integrator modifies the mass matrix
  (`M + h·D + h²·K`) and solves a coupled system. The standalone
  `ImplicitVelocity` solves a scalar `(M - h*D) * v = M*v + h*f`. Different
  algorithms, different inputs.
- No external users of sim-physics have been identified that consume the
  standalone integrators independent of the pipeline.

#### Specification

Steps are grouped by file to minimize context-switching during implementation.
The compiler will catch any missed references — after each step, `cargo check`
should surface the next breakage.

##### Step 1: Delete `integrators.rs`

Delete `sim/L0/core/src/integrators.rs` (1,005 lines — 497 lines of code,
508 lines of tests and doc-comments including a module-level doc-test).

##### Step 2: Update `sim/L0/core/src/lib.rs`

Two changes in one file:

1. Remove module declaration (line 78):
   ```rust
   pub mod integrators;  // DELETE
   ```

2. Remove `IntegrationMethod` from the sim-types re-export (line 137):
   ```rust
   pub use sim_types::{
       Action, ActionType, BodyId, ExternalForce, Gravity, JointCommand,
       JointCommandType, JointId, JointLimits, JointState, JointType,
       MassProperties, Observation, ObservationType, Pose, RigidBodyState,
       SimError, SimulationConfig, SolverConfig, Twist,
       // REMOVE: IntegrationMethod
   };
   ```

##### Step 3: Update `sim/L0/types/src/config.rs`

Remove the `IntegrationMethod` enum and all its dependents:

1. Delete the `IntegrationMethod` enum (lines 384-418), `impl IntegrationMethod`
   block (lines 420-462), and `Display` impl (lines 464-473).

2. Remove `IntegrationMethod` from `SolverConfig`:
   - Delete `pub integration: IntegrationMethod` field (line 153).
   - Delete `integration: IntegrationMethod::SemiImplicitEuler` from `Default`
     impl (line 275).
   - Delete `integration: IntegrationMethod::RungeKutta4` from `high_accuracy()`
     (line 294).
   - Delete `integration: IntegrationMethod::SemiImplicitEuler` from `fast()`
     (line 311).
   - Delete builder method `pub fn integration(...)` (lines 325-329).

3. Update tests:
   - In `test_solver_configs`: remove `hifi.integration` assertion (line 537).
   - Delete `test_integration_method` (lines 562-569).

4. Update `sim/L0/types/src/lib.rs` line 74: remove `IntegrationMethod` from the
   `pub use config::` re-export.

##### Step 4: Update `sim/L0/mjcf/src/config.rs`

Four changes in one file:

1. Remove `IntegrationMethod` from the import (line 7). Keep `Gravity`,
   `ParallelConfig`, `SimulationConfig`, `SolverConfig`.

2. Delete the `From<MjcfIntegrator> for IntegrationMethod` impl (lines 12-21).

3. In `From<&MjcfOption> for SimulationConfig` (line 24): delete the
   `integration: option.integrator.into()` line from the `SolverConfig`
   construction (line 35).

4. Delete test `test_integrator_conversion` (lines 219-236).

##### Step 5: Update `sim/L0/physics/src/lib.rs`

Three changes in the prelude:

1. Remove `IntegrationMethod` from the sim-types re-export (line 146):
   ```rust
   pub use sim_types::{Gravity, SimulationConfig, SolverConfig};
   // REMOVE: IntegrationMethod
   ```

2. Delete the standalone integrator re-export block (lines 164-168):
   ```rust
   // DELETE entire block:
   pub use sim_core::integrators::{
       ExplicitEuler, Integrator, RungeKutta4, SemiImplicitEuler, VelocityVerlet,
       integrate_with_method,
   };
   ```

3. Remove the `Integrator as MjIntegrator` alias (line 162) and export the
   pipeline enum directly under its real name:
   ```rust
   pub use sim_core::{Contact, GeomType, Integrator, MjJointType, StepError};
   ```

##### Step 6: Update documentation

Two doc files reference the deleted system: `ARCHITECTURE.md` and
`MUJOCO_GAP_ANALYSIS.md`.

**6a. `sim/docs/ARCHITECTURE.md` (lines 203-206)**

Remove the `integrators.rs` bullet from the sim-core file listing. Replace:

```markdown
- `integrators.rs` — standalone trait system with 6 integrators (ExplicitEuler,
  SemiImplicitEuler, VelocityVerlet, RungeKutta4, ImplicitVelocity, ImplicitFast).
  **Not used by the MuJoCo pipeline** — the pipeline has its own `Integrator` enum
  in `mujoco_pipeline.rs`. See [FUTURE_WORK C1](./FUTURE_WORK.md) for disambiguation plan.
```

with nothing (delete the 4-line bullet entirely). The pipeline's `Integrator`
enum is already documented under `mujoco_pipeline.rs` in the same listing.

**6b. `sim/docs/MUJOCO_GAP_ANALYSIS.md`**

This file has six locations referencing the deleted system.

**Location 1: "Not in pipeline" summary (line 44)**

Replace:
```
- `integrators.rs` trait system (1,005 lines) — disconnected from pipeline ([FUTURE_WORK C1](./FUTURE_WORK.md))
```
with:
```
- ~~`integrators.rs` trait system~~ — removed in FUTURE_WORK C1
```

**Location 2: Integration status table (lines 93-98)**

Lines 93-96 list four standalone integrators as "Standalone (in integrators.rs
trait, not in pipeline)". Replace the four rows with:

```markdown
| Explicit Euler | - | - | Not implemented (removed standalone-only code) | - | - |
| Velocity Verlet | - | - | Not implemented (removed standalone-only code) | - | - |
| Implicit-in-velocity | Core feature | - | **Implemented** (pipeline `ImplicitSpringDamper` for diagonal spring/damper) | - | - |
| Implicit-fast (no Coriolis) | Optimization | - | Not separately implemented; `ImplicitSpringDamper` covers the primary use case | - | - |
```

Replace the blockquote (line 98) with:

```markdown
> The pipeline uses a single `Integrator` enum (`mujoco_pipeline.rs:629`)
> with three variants: `Euler` (semi-implicit), `RungeKutta4` (true 4-stage),
> and `ImplicitSpringDamper` (diagonal spring/damper implicit Euler).
> A standalone trait-based integrator system was removed in FUTURE_WORK C1.
```

**Location 3: Implicit Integration notes (lines 100-146)**

Replace the "Implemented" list (lines 112-117) with:

```markdown
**Implemented (pipeline):**
- `Integrator::ImplicitSpringDamper` in `mujoco_pipeline.rs` — implicit Euler
  for diagonal per-DOF spring stiffness K and damping D.
- Solves: `(M + h·D + h²·K)·v_new = M·v_old + h·f_ext - h·K·(q - q_eq)`
```

Replace the "Usage" code block (lines 119-143) with a pipeline-based example:

    **Usage:**
    ```rust
    use sim_mjcf::load_model;

    let mjcf = r#"<mujoco>
        <option integrator="implicit"/>
        <worldbody>
            <body pos="0 0 1">
                <joint type="hinge" stiffness="100" damping="10"/>
                <geom type="sphere" size="0.1" mass="1"/>
            </body>
        </worldbody>
    </mujoco>"#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.step(&model).expect("step");
    ```

Update "Files modified" (line 146): replace
`sim-core/src/integrators.rs, sim-types/src/config.rs` with
`sim-core/src/mujoco_pipeline.rs (Integrator::ImplicitSpringDamper)`.

Update section heading (line 100) to:
`### Implementation Notes: Implicit Integration ✅ COMPLETED`
(remove the "(standalone only)" qualifier).

**Location 4: Implicit-fast checklist (line 1454)**

Replace:
```
3. ~~**Implicit-fast (no Coriolis)**: Skip Coriolis terms for performance~~ ✅ → ⚠️ **Standalone** (in `integrators.rs` trait, not in pipeline; see [FUTURE_WORK C1](./FUTURE_WORK.md))
```
with:
```
3. ~~**Implicit-fast (no Coriolis)**: Skip Coriolis terms for performance~~ — Removed (standalone `integrators.rs` deleted in FUTURE_WORK C1; pipeline uses `ImplicitSpringDamper`)
```

**Location 5: Implicit-Fast block (lines 1471-1517)**

The code example (lines 1478-1510) references deleted types
(`NewtonConstraintSolver`, `ImplicitFast`, `integrate_with_method`,
`IntegrationMethod::ImplicitFast`). The Newton solver was already removed in
Phase 3 consolidation; the integrator types are removed by this PR.

Replace lines 1471-1517 with:

```markdown
**Implicit-Fast Integration (removed):**
- The standalone `ImplicitFast` integrator and its `IntegrationMethod::ImplicitFast`
  dispatch variant were removed in FUTURE_WORK C1 (dead code — not called by
  the pipeline). The pipeline's `Integrator::ImplicitSpringDamper` covers the
  primary use case (diagonal spring/damper implicit Euler).

**Files (removed in consolidation):**
- `sim-constraint/src/sparse.rs` — removed in Phase 3
- `sim-constraint/src/newton.rs` — removed in Phase 3
- `sim-core/src/integrators.rs` — removed in FUTURE_WORK C1
```

**Location 6: File listing table (line 1975)**

Remove `integrators.rs` from the sim-core "Key Files" column. Change:
```
`mujoco_pipeline.rs`, `integrators.rs`, `collision_shape.rs`, ...
```
to:
```
`mujoco_pipeline.rs`, `collision_shape.rs`, ...
```

#### Acceptance Criteria

1. `sim/L0/core/src/integrators.rs` no longer exists.
2. `IntegrationMethod` enum no longer exists in sim-types.
3. `SolverConfig` no longer has an `integration` field.
4. The sim-physics prelude exports `Integrator` (the pipeline enum) directly,
   without the `as MjIntegrator` alias. No standalone integrator types
   (`ExplicitEuler`, `SemiImplicitEuler`, `VelocityVerlet`,
   `integrate_with_method`) appear in the public API.
5. `cargo build --workspace` succeeds with zero errors and zero warnings
   related to unused imports or dead code.
6. `cargo test --workspace` passes — all existing pipeline tests
   (implicit integration, RK4 integration, collision, sensor, MJCF parsing)
   remain green. The only tests removed are the internal `integrators.rs` tests
   and `test_integrator_conversion` / `test_integration_method` in config modules.
7. `cargo doc --workspace` succeeds with no broken intra-doc links.
8. `MUJOCO_GAP_ANALYSIS.md` and `ARCHITECTURE.md` contain no `use` statements
   referencing `sim_core::integrators`, no `IntegrationMethod::` paths, and no
   `ImplicitVelocity` / `integrate_with_method` references. The `integrators.rs`
   file listing is removed from both docs.
   Note: `MjcfIntegrator::ImplicitFast` in `mjcf/src/types.rs`, `parser.rs`,
   and `model_builder.rs` is **retained** — it is live MJCF parsing code that
   maps to the pipeline's `Integrator::ImplicitSpringDamper`, not part of the
   dead standalone system.
9. No `#[allow(dead_code)]` or `#[allow(unused_imports)]` annotations were added
   to suppress warnings — dead code is deleted, not silenced.
10. `RigidBodyState`, `Twist`, `Pose`, and other sim-types used by sim-sensor
    and elsewhere are **not** removed — only `IntegrationMethod` and its
    dependents are deleted.

#### Verification Strategy

After applying each step, run `cargo check --workspace` to confirm the next
set of compile errors matches expectations:

| After Step | Expected errors |
|------------|----------------|
| 1 (delete file) | `mod integrators` in `lib.rs` → unresolved module |
| 2 (update lib.rs) | `integrators::` paths in `physics/src/lib.rs` → unresolved |
| 3 (update types) | `IntegrationMethod` in `mjcf/src/config.rs` → unresolved |
| 4 (update mjcf) | `IntegrationMethod` in `physics/src/lib.rs` prelude → unresolved |
| 5 (update physics) | Zero errors. `cargo test --workspace` should pass. |
| 6 (update docs) | N/A (markdown, not compiled). Grep-verify: `grep -rn 'integrators\.\|IntegrationMethod\|ImplicitVelocity\|integrate_with_method' sim/docs/` should return only FUTURE_WORK.md (historical reference). |

#### Net Change

| Component | Lines removed | Notes |
|-----------|--------------|-------|
| `integrators.rs` (code) | ~497 | 6 structs, 2 dispatch fns, trait, utilities |
| `integrators.rs` (tests + docs) | ~508 | Module doc-test + ~20 unit tests |
| `IntegrationMethod` enum + impls + Display | ~90 | `types/src/config.rs` |
| `SolverConfig.integration` field + builder + defaults | ~15 | `types/src/config.rs` |
| `From<MjcfIntegrator> for IntegrationMethod` + test | ~28 | `mjcf/src/config.rs` |
| Re-export lines (core lib.rs, physics lib.rs) | ~10 | Module decl + 2 re-export blocks |
| Config test lines | ~10 | `test_integration_method`, `test_integrator_conversion` |
| ARCHITECTURE.md stale bullet | 4 | `integrators.rs` listing deleted |
| GAP_ANALYSIS.md stale sections | ~90 | 6 locations; replaced with ~35 lines of updated text |
| **Total removed** | **~1,200** | |
| **Total added (doc replacements)** | **~35** | |
| **Net** | **~−1,165** | |

#### Files

- `sim/L0/core/src/integrators.rs` — **delete** (1,005 lines)
- `sim/L0/core/src/lib.rs` — modify (remove `pub mod integrators` line 78,
  remove `IntegrationMethod` from re-export line 137)
- `sim/L0/types/src/config.rs` — modify (remove `IntegrationMethod` enum +
  impls lines 384-473, remove `SolverConfig.integration` field + builder +
  defaults + test assertions)
- `sim/L0/types/src/lib.rs` — modify (remove `IntegrationMethod` from
  re-export line 74)
- `sim/L0/mjcf/src/config.rs` — modify (remove `IntegrationMethod` import
  line 7, remove `From` impl lines 12-21, remove `integration:` field
  construction line 35, remove test lines 219-236)
- `sim/L0/physics/src/lib.rs` — modify (remove `IntegrationMethod` from
  prelude line 146, delete integrator re-export block lines 164-168,
  replace `Integrator as MjIntegrator` alias line 162 with direct export)
- `sim/docs/ARCHITECTURE.md` — modify (remove `integrators.rs` bullet
  lines 203-206)
- `sim/docs/MUJOCO_GAP_ANALYSIS.md` — modify (6 locations: summary line 44,
  integration table lines 93-98, implicit notes lines 100-146, checklist
  line 1454, implicit-fast block lines 1471-1517, file table line 1975)

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
