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
| ~~1~~ | ~~In-Place Cholesky~~ | ~~Low~~ | ~~Low~~ | ~~S~~ | ~~None~~ ✅ |
| ~~2~~ | ~~Sparse L^T D L~~ | ~~Medium~~ | ~~Low~~ | ~~M~~ | ~~#1~~ ✅ |
| 3 | CG Contact Solver | Medium | Low | L | None (#1, #2 help perf) |
| ~~4~~ | ~~Tendon Pipeline~~ | ~~Low~~ | ~~High~~ | ~~L~~ | ~~None~~ ✅ |
| ~~5~~ | ~~Muscle Pipeline~~ | ~~Low~~ | ~~High~~ | ~~L~~ | ~~#4~~ ✅ |
| ~~6~~ | ~~Sensor Completion~~ | ~~High~~ | ~~High~~ | ~~S~~ | ~~#4 for tendon sensors~~ ✅ |
| ~~7~~ | ~~Integrator Rename~~ | ~~Low~~ | ~~Medium~~ | ~~S~~ | ~~None~~ ✅ |
| ~~8~~ | ~~True RK4 Integration~~ | ~~Low~~ | ~~Medium~~ | ~~M~~ | ~~None~~ ✅ |
| 9 | Deformable Body Integration | Medium | Low | XL | None |
| 10 | Batched Simulation | High | Low | L | None |
| 11 | GPU Acceleration | High | Low | XL | #10 |
| ~~12~~ | ~~General Gain/Bias Actuator Force Model~~ | ~~High~~ | ~~High~~ | ~~M~~ | ~~#5~~ ✅ |

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
   └─┬─┘
     │
     ▼
   ┌────┐
   │ 12 │ General Gain/Bias Actuator Force Model
   └────┘

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
`mj_fwd_acceleration_implicit()` (`mujoco_pipeline.rs:9677`) previously used
`std::mem::replace` to swap `scratch_m_impl` out for nalgebra's `.cholesky()` (which
takes ownership), replacing it with a freshly allocated `DMatrix::zeros(nv, nv)` every
step. For nv=100 this was ~78 KB/step of unnecessary heap allocation and zeroing.

The pre-implementation flow was:
```rust
let chol = std::mem::replace(&mut data.scratch_m_impl, DMatrix::zeros(nv, nv))
    .cholesky()
    .ok_or(StepError::CholeskyFailed)?;
data.scratch_v_new.copy_from(&data.scratch_rhs);
chol.solve_mut(&mut data.scratch_v_new);
```

`scratch_m_impl` is pre-allocated in `make_data()` (`mujoco_pipeline.rs:1910`) and
populated each step via `copy_from(&data.qM)` (`mujoco_pipeline.rs:9701`) which
overwrites the full matrix. After Cholesky, the matrix is never read again until
the next step's `copy_from`. The replacement zero matrix was therefore wasted — the
next `copy_from` would overwrite it entirely regardless of contents.

`StepError::CholeskyFailed` already exists (`mujoco_pipeline.rs:696`).

#### Objective
Zero-allocation Cholesky factorization that overwrites `scratch_m_impl` in place,
eliminating the `std::mem::replace` + `DMatrix::zeros` pattern.

#### Specification

Add two functions to `mujoco_pipeline.rs`, directly above `mj_fwd_acceleration_implicit()` (`mujoco_pipeline.rs:9677`):

**`cholesky_in_place(m: &mut DMatrix<f64>) -> Result<(), StepError>`** (`mujoco_pipeline.rs:9505`)

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

**`cholesky_solve_in_place(l: &DMatrix<f64>, x: &mut DVector<f64>)`** (`mujoco_pipeline.rs:9535`)

Solve L·L^T·x = b where L is stored in the lower triangle of `l`:

1. Let `n = l.nrows()`.
2. Forward substitution (L·y = x): for j = 0..n,
   `x[j] = (x[j] - Σ(l[(j,k)] · x[k] for k < j)) / l[(j,j)]`.
3. Back substitution (L^T·z = y): for j = (n-1)..=0,
   `x[j] = (x[j] - Σ(l[(k,j)] · x[k] for k = (j+1)..n)) / l[(j,j)]`.
   (L^T elements accessed via `l[(k,j)]` — reading column j of L as row j of L^T.)

Both functions operate on borrowed data. No allocations, no ownership transfers,
no nalgebra `Cholesky` type.

**Modify `mj_fwd_acceleration_implicit()`** — replace the old Cholesky comment block,
`let nv`, `std::mem::replace`, and `chol.solve_mut` with (now at `mujoco_pipeline.rs:9725-9729`):

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
   solved vector. Verified by `in_place_cholesky_matches_nalgebra()` (`mujoco_pipeline.rs:12307`)
   which compares both paths on random SPD matrices at sizes 1, 2, 3, 5, 10, 20.
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

| Consumer | Location | Pre-impl method | Pre-impl cost |
|----------|----------|----------------|---------------|
| `mj_fwd_acceleration_explicit()` | `mujoco_pipeline.rs:9487` | `data.qM_cholesky` (nalgebra) | O(nv²) solve, O(nv³) factorize |
| `mj_fwd_acceleration_implicit()` | `mujoco_pipeline.rs:9677` | `cholesky_in_place()` on M_impl | O(nv³) factorize + solve |
| `pgs_solve_contacts()` | `mujoco_pipeline.rs:7736` | `data.qM_cholesky` (nalgebra) | O(nv²) solve, O(nv³) factorize |

For tree-structured robots, `M[i,j] ≠ 0` only when DOF `j` is an ancestor of DOF `i`
(or vice versa) in the kinematic tree encoded by `model.dof_parent: Vec<Option<usize>>`
(`mujoco_pipeline.rs:837`). This tree sparsity enables O(nv) factorization and solve.

**Data fields** in `Data` (`mujoco_pipeline.rs:1436-1443`):
```rust
pub qLD_diag: DVector<f64>,                  // D[i,i] diagonal
pub qLD_L: Vec<Vec<(usize, f64)>>,           // L[i,:] sparse off-diagonal per row
pub qLD_valid: bool,                         // dispatch flag
```
Initialized at `mujoco_pipeline.rs:1867-1869` (zeros/empty/false), populated by
`mj_factor_sparse()` (`mujoco_pipeline.rs:9572`).

The `qM_cholesky: Option<Cholesky<f64, Dyn>>` (removed when #2 was completed) was
previously computed in `mj_crba()` Phase 5 via `data.qM.clone().cholesky()` — that
clone+factorize was the O(nv³) cost eliminated for the explicit/contact paths. Phase 5
now calls `mj_factor_sparse()` (`mujoco_pipeline.rs:6812`).

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

At `mj_crba()` (`mujoco_pipeline.rs:6654`), replace the nalgebra Cholesky with sparse factorization (now at line 6812):

```rust
// Phase 5: Factorize mass matrix (sparse L^T D L)
if model.nv > 0 {
    mj_factor_sparse(model, data);
}
// Remove: data.qM_cholesky = data.qM.clone().cholesky();
// The sparse factorization replaces this for all consumers.
```

Also set `data.qLD_valid = false` at the **top** of `mj_crba()` (`mujoco_pipeline.rs:6655`),
so stale factorization is never used during the CRBA computation.

Remove the `qM_cholesky` field from `Data` entirely (along with its initialization
in `make_data()` and the doc comments referencing it).

##### Step 4: Dispatch in `mj_fwd_acceleration_explicit()` (`mujoco_pipeline.rs:9487`)

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

##### Step 5: Migrate `pgs_solve_contacts()` (`mujoco_pipeline.rs:7736`)

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

##### Step 6: Dispatch in `mj_fwd_acceleration_implicit()` (`mujoco_pipeline.rs:9677`)

The implicit path uses `M_impl = M + h·D + h²·K`. The diagonal modification
(`+= h*d[i] + h²*k[i]` at `mujoco_pipeline.rs:9703`) means we **cannot** reuse the CRBA sparse
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
- `sim/docs/MUJOCO_REFERENCE.md` — update Phase 5 description (line 291)
- `sim/docs/ARCHITECTURE.md` — update data fields table (line 80)

---

### 3. CG Contact Solver
**Status:** Not started | **Effort:** L | **Prerequisites:** None (#1, #2 improve M⁻¹ performance but are not hard dependencies)

#### Current State

`pgs_solve_contacts()` (`mujoco_pipeline.rs:7736`) solves the mixed LCP for contact
constraints using Projected Gauss-Seidel. It takes `&Model`, `&Data`, `&[Contact]`,
`&[DMatrix<f64>]` (Jacobians), `max_iterations`, `tolerance`, and
`&mut HashMap<WarmstartKey, [f64; 3]>` (warmstart cache). The function assembles the
Delassus matrix `A = J · M⁻¹ · J^T` (lines 7783–7891), builds the RHS from
unconstrained accelerations, contact velocities, and Baumgarte stabilization (lines
7893–7933), warmstarts from `efc_lambda` keyed by `WarmstartKey` (geom pair + spatial
grid cell), and iterates Gauss-Seidel with friction cone projection (lines
7965–8017). It returns `Vec<Vector3<f64>>`.

`mj_fwd_constraint()` (`mujoco_pipeline.rs:9155`) builds contact Jacobians via
`compute_contact_jacobian()` (`mujoco_pipeline.rs:7556`), calls `pgs_solve_contacts()`
(line 9302), and applies forces via `J^T` (lines 9321–9339). The solver is
unconditionally PGS — there is no dispatch mechanism for alternative solvers.

`CGSolver` in `sim-constraint/src/cg.rs` (1,664 lines, line 309) implements
preconditioned CG for joint constraints via the `Joint` trait (`joint.rs:19`, imported in `cg.rs:59`). It
requires `parent()`, `child()`, `parent_anchor()`, `joint_type()` — none of which
exist for contact constraints. The type systems are incompatible. Furthermore,
`sim-core` and `sim-constraint` are decoupled crates with no cross-dependencies;
`cg.rs` cannot import `Contact`, `Model`, or `Data`.

The MJCF parser already parses `<option solver="CG"/>` into `MjcfSolverType::CG`
(`types.rs:92`) and stores it in `MjcfOption.solver` (`types.rs:248`).
`ExtendedSolverConfig` (`config.rs:56`) holds `solver_type: MjcfSolverType` (`config.rs:61`). However,
the `ModelBuilder`'s `set_options()` (`model_builder.rs:545–568`) does NOT propagate `option.solver` to
the `Model` struct — the `Model` has `solver_iterations` (`mujoco_pipeline.rs:1072`)
and `solver_tolerance` (`mujoco_pipeline.rs:1074`) but no `solver_type` field. The
solver type parsed from MJCF is currently dead data.

`Data` has a scaffold field `solver_niter: usize` (`mujoco_pipeline.rs:1492`),
initialized to 0 (`mujoco_pipeline.rs:1895`), never written to by PGS. Both solvers
should populate it. Since `pgs_solve_contacts()` takes `&Data` (immutable), it cannot
set `solver_niter` internally. This task changes PGS to return `(Vec<Vector3<f64>>,
usize)` — the force vector and actual iteration count — so `mj_fwd_constraint()` sets
`data.solver_niter` in the dispatch code for both solver paths.

#### Objective

CG-based contact constraint solver in `mujoco_pipeline.rs` as an alternative to PGS,
selectable per-model via `<option solver="CG"/>`.

#### Specification

##### Step 1: `SolverType` enum and Model field

Add `SolverType` enum directly above `Integrator` (before `mujoco_pipeline.rs:669`):

```rust
/// Contact constraint solver algorithm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SolverType {
    /// Projected Gauss-Seidel (default, matches MuJoCo).
    #[default]
    PGS,
    /// Conjugate Gradient with friction cone projection (contact solver).
    /// Falls back to PGS if CG fails to converge.
    /// Note: this is the contact CG in sim-core, not the joint CGSolver
    /// in sim-constraint (which uses the Joint trait independently).
    CG,
    /// Strict CG: sets data.solver_niter = max_iterations and returns zero
    /// forces on non-convergence. Use in tests to catch CG regressions
    /// without silent PGS fallback.
    CGStrict,
}
```

Add field to `Model`, after `integrator` (`mujoco_pipeline.rs:1097`):

```rust
/// Contact constraint solver algorithm (PGS or CG).
pub solver_type: SolverType,
```

Initialize to `SolverType::PGS` in `Model::empty()` (line 1583; insert after
`integrator: Integrator::Euler,` at `mujoco_pipeline.rs:1757`).

Add `SolverType` to the `pub use mujoco_pipeline::{...}` re-export in `lib.rs`
(after `Integrator` at line 119), so that `model_builder.rs` can import it via
`use sim_core::SolverType`.

##### Step 2: `cg_solve_contacts()` function

Place directly below `pgs_solve_contacts()` (after `mujoco_pipeline.rs:8043`). The
signature mirrors PGS exactly, with `Option` return for convergence signaling:

```rust
/// Solve contact constraints using preconditioned Conjugate Gradient.
///
/// Solves the same system as pgs_solve_contacts:
///   A * lambda = -b
/// where A is the Delassus matrix (with CFM regularization on diagonal)
/// and b is the constraint RHS. Friction cone projection applied after
/// each CG iteration (projected CG).
///
/// Returns `Some((forces, iterations_used))` on convergence, `None` on non-convergence.
/// The caller decides whether to fall back to PGS or signal failure.
///
/// **Design note:** `None` discards the CG iteration count. On the fallback path,
/// `data.solver_niter` reflects PGS iterations only — the CG attempt's cost is not
/// observable. A future improvement could return `Result<(forces, iters), iters>` to
/// expose CG iteration count on failure, but this adds API complexity for a diagnostic
/// that is only useful during solver tuning.
fn cg_solve_contacts(
    model: &Model,
    data: &Data,
    contacts: &[Contact],
    jacobians: &[DMatrix<f64>],
    max_iterations: usize,
    tolerance: f64,
    efc_lambda: &mut HashMap<WarmstartKey, [f64; 3]>,
) -> Option<(Vec<Vector3<f64>>, usize)>
```

Returns `Option<(forces, iterations_used)>`.

##### Step 3: Extract `assemble_contact_system()`

Extract Delassus matrix + RHS assembly from `pgs_solve_contacts()` (lines 7755–7933)
into a shared helper so both PGS and CG use identical physics formulas.

**Scalability note:** Both PGS and CG form the dense Delassus matrix A (O(nefc²)
memory, O(nefc² · nv) assembly). For 100 contacts this is ~700 KB; for 1000 contacts,
~69 MB. MuJoCo's production CG avoids this by computing `J · M⁻¹ · J^T · p` as a
sequence of sparse operations (O(nv · nefc) per iteration). Transitioning to implicit
matrix-vector products is a future optimization for large contact counts.

```rust
/// Assemble Delassus matrix A and constraint RHS b for contact constraints.
/// Shared by PGS and CG solvers.
///
/// A[i,j] = J_i · M⁻¹ · J_j^T (with CFM regularization on diagonal).
/// b includes unconstrained acceleration, contact velocities, and Baumgarte
/// stabilization from solref/solimp.
fn assemble_contact_system(
    model: &Model,
    data: &Data,
    contacts: &[Contact],
    jacobians: &[DMatrix<f64>],
) -> (DMatrix<f64>, DVector<f64>)
```

The assembly contains:

**Unconstrained acceleration** (current PGS lines 7763–7767):
```
qacc_smooth = qfrc_applied + qfrc_actuator + qfrc_passive - qfrc_bias
mj_solve_sparse(&qLD_diag, &qLD_L, &mut qacc_smooth)
```

**M⁻¹ · J^T precomputation** (current PGS lines 7770–7781): Each contact Jacobian
`J_i` is a `3 × nv` matrix (row 0 = normal, rows 1–2 = friction directions).
For each contact, solve 3 sparse systems `M · x = J_i^T[:, col]` via
`mj_solve_sparse()` (`mujoco_pipeline.rs:9630`), yielding `minv_jt[i]`: an
`nv × 3` matrix.

**Delassus diagonal blocks** (current PGS lines 7802–7810):
```
A[i,i] = J_i · minv_jt[i] + cfm · I
```
Where `cfm = base_regularization + (1 - d) · model.regularization · 100`, and
`d = compute_impedance(contact.solimp, |contact.depth|)` (`mujoco_pipeline.rs:8258`).

**Delassus off-diagonal blocks** (current PGS lines 7819–7891):
```
A[i,j] = J_i · minv_jt[j]     (symmetric, only if contacts share dynamic bodies)
```
Uses `bodies_share_chain()` (`mujoco_pipeline.rs:7696`) bitmask optimization.

**Impedance, CFM, and ERP from solref/solimp** (current PGS lines 7785–7800 for
impedance/CFM, 7812–7817 for ERP; lines 7802–7810 are the diagonal blocks above):
```
timeconst = max(contact.solref[0], 0.001)
dampratio = max(contact.solref[1], 0.0)
dt        = max(model.timestep, 1e-6)        // line 7816 — prevents division by zero
erp_i = (dt / (dt + timeconst)) · min(dampratio, 1.0)   // per-contact (code: erp_i, line 7817)
```

**RHS assembly** (current PGS lines 7893–7933):
```
b[i·3]     = J·qacc_smooth[0] + vn/dt + (erp_i/dt)·depth   (normal)
b[i·3 + 1] = J·qacc_smooth[1] + vt1                         (friction 1)
b[i·3 + 2] = J·qacc_smooth[2] + vt2                         (friction 2)
```
Where `vn`, `vt1`, `vt2` are relative contact-frame velocities from
`compute_point_velocity()` (`mujoco_pipeline.rs:9343`).

After extraction, `pgs_solve_contacts()` becomes:
1. Call `assemble_contact_system()` → `(A, b)`
2. Load warmstart from `efc_lambda` into `lambda` (lines 7935–7949, duplicated in CG)
3. Compute `diag_inv` — per-element inverse of A's diagonal (lines 7951–7963, PGS-only;
   CG uses block Jacobi preconditioner instead)
4. Run the existing Gauss-Seidel iteration loop (lines 7965–8017, unchanged)
5. Call `extract_forces(&lambda, ncon)` to convert the lambda vector to `Vec<Vector3>`
   (replaces the inline loop at lines 8032–8040)

`cg_solve_contacts()` calls the same `assemble_contact_system()` helper then runs
the CG loop below.

Define `project_friction_cone()` as a utility that projects an entire lambda vector
onto the friction cone in a single pass. **Only CG calls this function** (after each
full CG iteration). PGS retains its existing per-contact inline projection inside the
Gauss-Seidel sweep (lines 7991–8005) — this ordering is essential to GS semantics,
where contact `i+1` sees the already-projected lambda of contact `i`. Changing PGS
to call the all-at-once function would alter iteration trajectories and break AC #11
(bit-identity). The function's per-contact logic is identical to PGS's inline code:

```rust
/// Project lambda onto the friction cone for all contacts.
/// Used by CG after each full iteration. PGS does NOT call this -- it inlines
/// per-contact projection inside the GS sweep for correct Gauss-Seidel ordering.
fn project_friction_cone(lambda: &mut DVector<f64>, contacts: &[Contact], ncon: usize) {
    for i in 0..ncon {
        let base = i * 3;
        lambda[base] = lambda[base].max(0.0);           // λ_n ≥ 0
        let mu = contacts[i].friction;
        let max_friction = mu * lambda[base];
        let friction_mag = (lambda[base + 1].powi(2) + lambda[base + 2].powi(2)).sqrt();
        if friction_mag > max_friction && friction_mag > 1e-10 {
            let scale = max_friction / friction_mag;
            lambda[base + 1] *= scale;
            lambda[base + 2] *= scale;
        }
    }
}
```

And `extract_forces()`:

```rust
fn extract_forces(lambda: &DVector<f64>, ncon: usize) -> Vec<Vector3<f64>> {
    (0..ncon)
        .map(|i| Vector3::new(lambda[i * 3], lambda[i * 3 + 1], lambda[i * 3 + 2]))
        .collect()
}
```

##### Step 4: CG iteration loop with friction cone projection

Fletcher-Reeves CG with non-negative beta restart, adapted for contact constraints.
Friction cone projection after each iteration destroys CG conjugacy; the restart clamp
`beta = max(0, ...)` automatically resets the search direction to steepest descent when
conjugacy is lost (`beta_FR = rz_new / rz` can become negative after projection
invalidates the normal CG invariants; for standard preconditioned CG with an SPD
system, `rz = r^T M^{-1} r >= 0` always, but projection makes this non-standard).
Residual is recomputed from scratch each iteration (not incremental) because projection
modifies `lambda` non-linearly.

```
fn cg_solve_contacts(...) -> Option<(Vec<Vector3<f64>>, usize)>:
    ncon = contacts.len()
    if ncon == 0: return Some((vec![], 0))
    nefc = ncon * 3

    (A, b) = assemble_contact_system(model, data, contacts, jacobians)

    // ---- Block Jacobi preconditioner (Step 5) ----
    precond_inv = compute_block_jacobi_preconditioner(&A, ncon)

    // ---- Warmstart ----
    lambda = DVector::zeros(nefc)
    for (i, contact) in contacts.enumerate():
        key = warmstart_key(contact)
        if let Some(prev) = efc_lambda.get(&key):
            lambda[i*3] = prev[0]; lambda[i*3+1] = prev[1]; lambda[i*3+2] = prev[2]

    // ---- Single-contact direct solve ----
    // Warmstart loaded above is unused — direct solve gives the exact answer.
    if ncon == 1:
        // Direct solve: precond_inv[0] is the Cholesky inverse of the 3×3 block A[0:3,0:3].
        // cfm > 0 guarantees SPD, so Cholesky cannot fail here. If the scalar Jacobi
        // fallback were used (degenerate block), this would be approximate, not exact.
        lam = -(precond_inv[0] * Vector3::new(b[0], b[1], b[2]))
        // Project onto friction cone (inline, since lam is Vector3 not DVector):
        lam[0] = lam[0].max(0.0)                    // λ_n ≥ 0
        let mu = contacts[0].friction
        let max_f = mu * lam[0]
        let f_mag = (lam[1].powi(2) + lam[2].powi(2)).sqrt()
        if f_mag > max_f && f_mag > 1e-10:
            let scale = max_f / f_mag
            lam[1] *= scale; lam[2] *= scale
        // Store warmstart (clear stale entries from previous frames):
        efc_lambda.clear()
        key = warmstart_key(&contacts[0])
        efc_lambda.insert(key, [lam[0], lam[1], lam[2]])
        return Some((vec![lam], 0))

    // ---- Project initial guess onto feasible set ----
    project_friction_cone(&mut lambda, contacts, ncon)

    // ---- Initial residual: r = -(A · lambda + b) ----
    r = -(&A * &lambda + &b)
    z = apply_preconditioner(&precond_inv, &r, ncon)
    p = z.clone()
    rz = r.dot(&z)
    b_norm = b.norm()
    if b_norm < 1e-14:
        efc_lambda.clear()  // Don't carry stale warmstart to next frame
        return Some((vec![Vector3::zeros(); ncon], 0))
    // b ≈ 0 means the unconstrained system already satisfies constraints;
    // return zero contact forces and clear warmstart cache.

    converged = false
    iters_used = max_iterations
    for iter in 0..max_iterations:
        // Matrix-vector product
        Ap = &A * &p

        // Step length
        pAp = p.dot(&Ap)
        if pAp <= 0.0:
            iters_used = iter + 1
            break   // A not positive definite along p

        alpha = rz / pAp

        // Update and project
        lambda += alpha * &p
        project_friction_cone(&mut lambda, contacts, ncon)

        // Recompute residual from scratch (projection invalidates incremental update)
        r = -(&A * &lambda + &b)

        // Convergence check: relative residual after projection.
        // This measures how close lambda is to the unconstrained solution A*lambda = -b.
        // When the unconstrained solution is inside the friction cone, convergence is
        // exact. When projection is active, the criterion may oscillate near tolerance
        // (projection can increase the residual). The non-negative beta restart prevents
        // divergence. If oscillation is observed in practice, tightening tolerance by 10x
        // or switching to a complementarity residual (KKT violation) would be more robust.
        if r.norm() / b_norm < tolerance:
            converged = true
            iters_used = iter + 1
            break

        // Precondition
        z_new = apply_preconditioner(&precond_inv, &r, ncon)
        rz_new = r.dot(&z_new)

        // Fletcher-Reeves with non-negative beta restart: beta = max(0, rz_new / rz)
        beta = if rz.abs() < 1e-30 { 0.0 } else { (rz_new / rz).max(0.0) }
        p = &z_new + beta * &p
        rz = rz_new

    // ---- Store warmstart (even on non-convergence — partial solution helps next frame) ----
    // Note: if pAp <= 0 triggered on iteration 0, lambda is just the projected
    // warmstart (alpha is never computed on this path because break precedes the
    // division). The warmstart is still stored because the projected initial guess
    // is a reasonable starting point for the next frame.
    efc_lambda.clear()
    for (i, contact) in contacts.enumerate():
        key = warmstart_key(contact)
        efc_lambda.insert(key, [lambda[i*3], lambda[i*3+1], lambda[i*3+2]])

    if converged: Some((extract_forces(&lambda, ncon), iters_used)) else: None
```

**Resolved (warmstart key collision):** The warmstart key now uses `WarmstartKey =
(geom_lo, geom_hi, cell_x, cell_y, cell_z)` where the spatial component is a
discretized grid cell (1 cm resolution) of the contact position. This disambiguates
multiple contacts within the same geom pair (e.g., 4 corner contacts of a box on a
plane), giving each its own cached lambda. See `warmstart_key()` in
`mujoco_pipeline.rs`.

##### Step 5: Block Jacobi preconditioner

Extract 3×3 diagonal blocks from `A`, invert each via `nalgebra::Matrix3::cholesky()`.
Each block is SPD because `A_ii = J_i · M⁻¹ · J_i^T + cfm · I` where M is SPD and
cfm > 0. If Cholesky fails (degenerate contact geometry), fall back to scalar Jacobi
(invert diagonal only).

```rust
fn compute_block_jacobi_preconditioner(
    a: &DMatrix<f64>, ncon: usize,
) -> Vec<Matrix3<f64>> {
    let mut blocks = Vec::with_capacity(ncon);
    for i in 0..ncon {
        let base = i * 3;
        let block = Matrix3::new(
            a[(base,base)],   a[(base,base+1)],   a[(base,base+2)],
            a[(base+1,base)], a[(base+1,base+1)], a[(base+1,base+2)],
            a[(base+2,base)], a[(base+2,base+1)], a[(base+2,base+2)],
        );
        let inv = match block.cholesky() {
            Some(chol) => chol.inverse(),
            None => {
                // Scalar Jacobi fallback: invert diagonal only
                let d0 = if block[(0,0)].abs() > 1e-12 { 1.0/block[(0,0)] } else { 0.0 };
                let d1 = if block[(1,1)].abs() > 1e-12 { 1.0/block[(1,1)] } else { 0.0 };
                let d2 = if block[(2,2)].abs() > 1e-12 { 1.0/block[(2,2)] } else { 0.0 };
                Matrix3::new(d0, 0.0, 0.0,  0.0, d1, 0.0,  0.0, 0.0, d2)
            }
        };
        blocks.push(inv);
    }
    blocks
}

fn apply_preconditioner(
    precond_inv: &[Matrix3<f64>], r: &DVector<f64>, ncon: usize,
) -> DVector<f64> {
    let mut z = DVector::zeros(ncon * 3);
    for i in 0..ncon {
        let base = i * 3;
        let r_block = Vector3::new(r[base], r[base + 1], r[base + 2]);
        let z_block = precond_inv[i] * r_block;
        z[base] = z_block[0]; z[base + 1] = z_block[1]; z[base + 2] = z_block[2];
    }
    z
}
```

##### Step 6: Convergence and fallback

**Convergence criterion:** Relative residual `||r|| / ||b|| < tolerance`.
Values of `tolerance` and `max_iterations` come from `model.solver_tolerance`
and `model.solver_iterations` (`mujoco_pipeline.rs:1072–1074`), same as PGS.

**Note on criterion asymmetry:** PGS uses a different convergence measure —
`max_delta < tolerance` where `max_delta` is the maximum absolute change in any
`lambda` component per iteration (lines 8008–8016). CG uses relative residual norm.
The same `tolerance` value therefore has different meanings for the two solvers: a
tolerance of 1e-8 is very tight for PGS (absolute change) but moderate for CG
(relative residual). This is intentional — CG's relative criterion is standard for
Krylov methods and well-suited to the matrix formulation. Acceptance criteria compare
solver *outputs* (forces, velocities) rather than convergence metrics, so the
asymmetry does not affect correctness testing.

**Fallback policy by solver type:**

| `SolverType` | On convergence | On non-convergence |
|---|---|---|
| `PGS` | Use PGS result | N/A (PGS always returns a result) |
| `CG` | Use CG result | `#[cfg(debug_assertions)]` warning, rerun with PGS |
| `CGStrict` | Use CG result | Set `data.solver_niter = max_iterations`, return zero forces |

Both PGS and CG return their actual iteration count. The `mj_fwd_constraint()`
dispatch (Step 7) sets `data.solver_niter` (`mujoco_pipeline.rs:1492`) for both
paths. This requires changing `pgs_solve_contacts()` return type from
`Vec<Vector3<f64>>` to `(Vec<Vector3<f64>>, usize)` — the force vector and
iteration count at which convergence occurred (or `max_iterations` if the loop
exhausts all iterations — this value is the same whether convergence happened on the
final iteration or not, matching standard solver convention).

**PGS iteration tracking change:** The current PGS loop variable is `_iter`
(unused, line 7965). Change to `iter`, add `let mut iters_used = max_iterations;`
before the loop, and on early break (`max_delta < tolerance`, line 8014) set
`iters_used = iter + 1;`. Return `(forces, iters_used)` instead of `forces`.

##### Step 7: Dispatch in `mj_fwd_constraint()`

Replace the unconditional PGS call at `mujoco_pipeline.rs:9301–9316` with:

```rust
// The existing std::mem::take pattern (line 9292) remains unchanged:
// let mut efc_lambda = std::mem::take(&mut data.efc_lambda);
// This separates efc_lambda from data so we can pass &Data + &mut HashMap
// without borrow conflicts. After the match, restore: data.efc_lambda = efc_lambda;

let constraint_forces = match model.solver_type {
    SolverType::PGS => {
        let (forces, niter) = pgs_solve_contacts(
            model, data, &data.contacts, &jacobians,
            model.solver_iterations.max(10), model.solver_tolerance.max(1e-8),
            &mut efc_lambda,
        );
        data.solver_niter = niter;
        forces
    }
    SolverType::CG => {
        match cg_solve_contacts(
            model, data, &data.contacts, &jacobians,
            model.solver_iterations.max(10), model.solver_tolerance.max(1e-8),
            &mut efc_lambda,
        ) {
            Some((forces, niter)) => {
                data.solver_niter = niter;
                forces
            }
            None => {
                // CG did not converge — fall back to PGS.
                // efc_lambda already updated by CG with partial solution;
                // PGS benefits from this warmstart.
                // NOTE: PGS calls assemble_contact_system() internally, so the
                // Delassus matrix is assembled twice on this path (once in CG,
                // once in PGS). This O(ncon²) duplication is acceptable for a
                // fallback path; if profiling shows it matters, a future
                // optimization could pass the pre-assembled (A, b) to PGS.
                #[cfg(debug_assertions)]
                eprintln!(
                    "warn: CG contact solver did not converge ({} contacts), \
                     falling back to PGS", data.contacts.len()
                );
                let (forces, niter) = pgs_solve_contacts(
                    model, data, &data.contacts, &jacobians,
                    model.solver_iterations.max(10), model.solver_tolerance.max(1e-8),
                    &mut efc_lambda,
                );
                data.solver_niter = niter;
                forces
            }
        }
    }
    SolverType::CGStrict => {
        match cg_solve_contacts(
            model, data, &data.contacts, &jacobians,
            model.solver_iterations.max(10), model.solver_tolerance.max(1e-8),
            &mut efc_lambda,
        ) {
            Some((forces, niter)) => {
                data.solver_niter = niter;
                forces
            }
            None => {
                // Use the clamped value (same as what was passed to the solver)
                data.solver_niter = model.solver_iterations.max(10);
                vec![Vector3::zeros(); data.contacts.len()]
            }
        }
    }
};
```

##### Step 8: MJCF wiring

**Parser** (`parser.rs`): Already parses `<option solver="CG"/>` into
`MjcfSolverType::CG`. No changes needed.

**Model builder** (`model_builder.rs`):

Add `SolverType` to the existing `use sim_core::{...}` import (line 17).
Add `solver_type` field to the builder struct (after `integrator` at line 314).
Initialize to `SolverType::PGS` in `ModelBuilder::new()` (after `integrator` init at
line 487).

Wire in `set_options()` (after the integrator match block, line 563):
```rust
self.solver_type = match option.solver {
    MjcfSolverType::PGS | MjcfSolverType::Newton => SolverType::PGS,
    MjcfSolverType::CG => SolverType::CG,
    // Newton maps to PGS: MjcfSolverType::Newton is the #[default] (types.rs:94),
    // so every MJCF without explicit solver="CG" must remain PGS to avoid
    // behavioral regression.
};
```

Propagate in `build()` (after `integrator: self.integrator,` at line 2051):
```rust
solver_type: self.solver_type,
```

#### Acceptance Criteria

1. **PGS parity (sphere-on-plane):** A sphere (mass 1 kg, radius 0.05 m) resting on
   a ground plane under gravity (g = -9.81). After 1000 steps (dt = 0.002), CG and PGS
   produce contact normal forces within 1e-4 relative error. Run with
   `SolverType::CGStrict` to prove convergence.

2. **PGS parity (multi-contact stack):** A stack of 3 boxes (half-extents 0.05 m,
   mass 1 kg each, friction = 0.5) on a ground plane under gravity (g = -9.81).
   After 500 steps (dt = 0.002), CG and PGS produce `qfrc_constraint` within 1e-3
   relative error (`||qfrc_cg - qfrc_pgs|| / ||qfrc_pgs||`). Run with
   `SolverType::CGStrict`.

3. **PGS parity (friction slide):** A box (half-extents 0.05 m, mass 1 kg,
   friction = 0.3) on a tilted plane (25°) under gravity (g = -9.81). After 200
   steps (dt = 0.002), CG and PGS produce final velocities within 1e-3 relative
   error. Run with `SolverType::CGStrict`.

4. **Friction cone satisfaction:** For each scenario above, every contact force
   satisfies `lambda_n >= 0` and
   `sqrt(lambda_t1² + lambda_t2²) <= mu · lambda_n + 1e-10`.

5. **Zero contacts:** Call `cg_solve_contacts()` directly (not through
   `mj_fwd_constraint()`, which early-returns at line 9281 for empty contacts).
   Pass `contacts = &[]`. Verify it returns `Some((vec![], 0))`.

6. **Single-contact direct solve:** Call `cg_solve_contacts()` directly with exactly
   1 contact. Verify it uses the direct 3×3 solve (no iteration loop) and returns
   `iterations_used == 0`.

7. **CGStrict failure detection:** Two sub-tests:
   **(a)** Call `cg_solve_contacts()` directly on the 3-box stack from AC #2
   (ensures ncon >= 2, bypassing the single-contact direct solve) with
   `max_iterations = 1` and `tolerance = 1e-15`. Verify it returns `None`.
   **(b)** Verify the `CGStrict` dispatch arm's zero-force behavior: build a Model
   with `solver_type = SolverType::CGStrict`, `solver_iterations = 1`,
   `solver_tolerance = 1e-15`. Call `mj_fwd_constraint()` on a 10-box stack
   (same parameters as AC #2: half-extents 0.05 m, mass 1 kg, friction = 0.5,
   g = -9.81, dt = 0.002). Assert `data.contacts.len() >= 20` to confirm
   sufficient contact count. The clamped 10 iterations at 1e-8 are insufficient
   for 20+ contacts. Verify `data.solver_niter == model.solver_iterations.max(10)`
   (i.e., 10) and that all contact forces are zero.

8. **CG fallback to PGS:** Two sub-tests:
   **(a)** Call `cg_solve_contacts()` directly with the 3-box stack from AC #2,
   `max_iterations = 1`, and `tolerance = 1e-15` to confirm it returns `None`.
   **(b)** Build a Model with `solver_type = SolverType::CG`, `solver_iterations = 1`,
   `solver_tolerance = 1e-15`. Call `mj_fwd_constraint()` on the 10-box stack
   from AC #7(b) (ncon >= 20, ensuring the clamped 10 iterations at 1e-8 are
   insufficient). Verify that `data.solver_niter` reflects PGS (not CG) and
   that all contact forces are non-zero (PGS fallback activated).

9. **Warmstart effectiveness:** Run the 3-box stack from AC #2 for 100 frames with
   `SolverType::CGStrict` (ncon >= 2, ensuring the CG iteration loop is used — the
   sphere-on-plane scene has only 1 contact and always takes the direct-solve path
   with `iterations_used == 0`). Read `data.solver_niter` each frame. On frame 1
   (cold start, no warmstart), CG may use many iterations. After frame 10
   (warmstarted), the average `solver_niter` over frames 10–100 is at least 30%
   lower than frame 1's value.

10. **MJCF round-trip:** Parse `<option solver="CG"/>`, build a Model, verify
    `model.solver_type == SolverType::CG`.

11. **Shared assembly correctness:** After extracting `assemble_contact_system()`, PGS
    produces bit-identical results to the pre-refactor version on the sphere-on-plane
    test. Run at both `opt-level=0` (debug) and `opt-level=2` (release). If the
    compiler reorders floating-point operations across the function boundary, release
    mode may diverge; in that case, mark the bit-identity test as debug-only and relax
    the release criterion to 1e-14 relative error.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — modify:
  - `SolverType` enum (before line 669)
  - `solver_type: SolverType` field in `Model` (after `integrator` at line 1097) and
    `Model::empty()` (after `integrator` init at line 1757)
  - `pgs_solve_contacts()` return type changed to `(Vec<Vector3<f64>>, usize)`
  - `assemble_contact_system()` extracted from `pgs_solve_contacts()` (lines 7755–7933)
  - `project_friction_cone()` new utility (CG-only; PGS retains inline per-contact projection)
  - `extract_forces()` helper
  - `cg_solve_contacts()` (after line 8043)
  - `compute_block_jacobi_preconditioner()` and `apply_preconditioner()`
  - `mj_fwd_constraint()` dispatch (lines 9301–9316)
  - PGS and CG both set `data.solver_niter` (line 1492)
  - Tests: `test_cg_pgs_parity_sphere_plane`, `test_cg_pgs_parity_box_stack`,
    `test_cg_pgs_parity_friction_slide`, `test_cg_friction_cone`,
    `test_cg_zero_contacts`, `test_cg_single_contact_direct`,
    `test_cg_strict_failure`, `test_cg_fallback`, `test_cg_warmstart`,
    `test_cg_mjcf_roundtrip`, `test_cg_shared_assembly`
- `sim/L0/core/src/lib.rs` — modify:
  - Add `SolverType` to `pub use mujoco_pipeline::{...}` re-export (after line 119)
- `sim/L0/mjcf/src/model_builder.rs` — modify:
  - Add `SolverType` to `use sim_core::{...}` import (line 17)
  - `solver_type` field (after `integrator` at line 314), initialization (after line 487),
    `set_options()` wiring (after line 563), `build()` propagation (after line 2051)
- `sim/L0/mjcf/src/parser.rs` — no changes (already parses solver attribute)

---
## Group B — Pipeline Integration

### 4. Tendon Pipeline
**Status:** ✅ Done (fixed tendons) | **Effort:** L | **Prerequisites:** None

#### Scope Decision: Fixed Tendons Only

This PR implements **fixed (linear-coupling) tendons only**. Spatial tendons (3D path
routing through sites with wrapping geometry) are deferred to a follow-up.

**Rationale:**
- Fixed tendons are the dominant tendon type in RL models (MuJoCo Menagerie humanoids,
  DM Control suite). They cover differential drives, antagonistic pairs, and linear
  coupling — the core use cases for tendon-driven actuators.
- Fixed tendons have **constant Jacobians** (the coupling coefficients), eliminating
  the need for body positional Jacobians. `compute_body_jacobian_at_point()`
  (`mujoco_pipeline.rs:6816`) is incomplete (only x-component per DOF, marked
  `#[allow(dead_code)]`) and would need a full rewrite for spatial tendons.
- Spatial tendons require wrapping geometry (sphere/cylinder geodesics) and
  configuration-dependent Jacobians via the chain rule through `mj_jac()`. This is
  a separate body of work that can land independently.
- The Model scaffolds (`wrap_type`, `wrap_objid`, `wrap_prm`) and sim-tendon's
  `SpatialTendon`/`SphereWrap`/`CylinderWrap` remain available for the spatial
  follow-up. Nothing in this PR precludes it.

The pipeline functions (`mj_fwd_tendon`, etc.) are written to dispatch on
`TendonType::Fixed` vs `::Spatial`, with the `Spatial` arm silently zeroing
the tendon length and Jacobian (no per-frame warning — that would spam logs).
A one-time warning is logged at model load time (in the model builder) when a
spatial tendon is encountered, informing the user that spatial tendons are not
yet supported and will produce zero forces. This ensures spatial tendons in MJCF
files are safely ignored rather than silently producing incorrect forces.

#### Current State

> **Note:** This section describes the pre-implementation baseline. All "Missing
> pieces" below have been implemented — see status ✅ Done above.

**sim-tendon crate** (`sim/L0/tendon/src/`, 3,919 lines, 52 tests):
- `FixedTendon` (`fixed.rs:65`): linear coupling `L = L₀ + Σᵢ cᵢ qᵢ`, implements
  `TendonActuator` trait. `compute_length()` (`fixed.rs:311`), `compute_velocity()`
  (`fixed.rs:315`), `jacobian()` (`fixed.rs:344`), `compute_joint_forces(tension)`
  (`fixed.rs:248`).
- `CableProperties` (`cable.rs:26`): one-way spring model.
  `compute_force(length, velocity)` (`cable.rs:195`): `F = k(L - L₀) + c·v` if
  `L > L₀`, else `0`.
- `SpatialTendon`, `TendonPath`, `SphereWrap`, `CylinderWrap`, `PulleySystem` —
  all standalone, no pipeline coupling. Deferred to spatial follow-up.

**Pipeline scaffolds** (all in `mujoco_pipeline.rs`):

| Component | Location | Status |
|-----------|----------|--------|
| `ntendon`, `nwrap` | Model, line 915-917 | Declared, always 0 |
| `tendon_stiffness`, `tendon_damping`, etc. | Model, lines 919-935 | Declared, empty vecs |
| `wrap_type`, `wrap_objid`, `wrap_prm` | Model, lines 939-943 | Declared, empty vecs |
| `ten_length`, `ten_velocity`, `ten_force`, `ten_J` | Data, lines 1363-1371 | Declared, zero-initialized, never populated |
| `ActuatorTransmission::Tendon` | Enum, line 419 | Defined |
| `ActuatorTransmission::Tendon` in `mj_fwd_actuation()` | Line 6017-6020 | No-op placeholder |
| `MjSensorType::TendonPos` | `mj_sensor_pos()`, line 5101 | Stub, writes 0.0 |
| `MjSensorType::TendonVel` | `mj_sensor_vel()`, line 5307 | Stub, writes 0.0 |
| `EqualityType::Tendon` | Line 467 | Defined, not implemented (out of scope) |
| `WrapType` enum | Line 438-450 | Defined (Site, Geom, Joint, Pulley) |

**MJCF parsing** (complete for fixed + spatial):

| Component | Location | Status |
|-----------|----------|--------|
| `MjcfTendon` struct | `types.rs:1999-2048` | Complete: name, type, stiffness, damping, joints, sites, wrapping_geoms |
| `MjcfTendonType` enum | `types.rs:1961-1982` | Complete: Fixed, Spatial |
| `parse_tendons()` | `parser.rs:1662-1694` | Complete: parses `<tendon>` section |
| `parse_tendon()` | `parser.rs:1697-1750` | Complete: child `<joint>`, `<site>`, `<geom>` elements |
| `MjcfTendonDefaults` | `types.rs:501-521` | Complete: range, limited, stiffness, damping, frictionloss |
| `apply_to_tendon()` | `defaults.rs:323-375` | Complete: merges defaults |
| Model builder | `model_builder.rs:1009-1018` | **Rejects** tendon transmission with error |
| Model builder | `model_builder.rs:1449-1463` | Initializes all tendon fields to empty |

**Missing pieces** (to be added by this PR):

1. Model builder: populate `Model` tendon fields from parsed `MjcfTendon` data.
2. Model builder: accept `ActuatorTransmission::Tendon` instead of rejecting it.
3. New Model fields: `tendon_solref`, `tendon_solimp` (for limit constraints).
4. New Model field: `tendon_frictionloss` (velocity-dependent friction).
5. New Model field: `tendon_type` (Fixed vs Spatial, for dispatch).
6. Pipeline function: `mj_fwd_tendon()` — compute lengths and Jacobians.
7. Velocity: `ten_velocity[t] = ten_J[t] · qvel` in `mj_fwd_velocity()`.
8. Passive forces: tendon spring/damper in `mj_fwd_passive()`.
9. Limit forces: tendon limits in `mj_fwd_constraint()`.
10. Actuation: tendon transmission in `mj_fwd_actuation()`.
11. Sensors: TendonPos/TendonVel stubs → real reads.
12. MJCF validation: check referenced joints exist, coefficients are finite.
13. `tendon_length0` computation from `qpos0` at model load time.

#### Objective

Wire fixed-tendon kinematics, passive forces, limit constraints, and actuation into
the MuJoCo pipeline so that tendon-driven actuators produce joint forces. Establish
the pipeline structure for spatial tendons to land in a follow-up without refactoring.

#### Specification

##### Step 1: Add New Model Fields

Add a `TendonType` enum to `mujoco_pipeline.rs` (after the `WrapType` enum at line 450),
matching the pipeline's convention of defining its own enums rather than importing
from `sim-mjcf`. `sim-core` does NOT depend on `sim-mjcf`, so `MjcfTendonType` is
not available in the pipeline. The model builder converts `MjcfTendonType` → `TendonType`.

```rust
/// Tendon type (pipeline-local enum, converted from MjcfTendonType in model builder).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum TendonType {
    /// Fixed (linear coupling): L = Σ coef_i * q_i, constant Jacobian.
    #[default]
    Fixed,
    /// Spatial (3D path routing through sites): not yet implemented.
    Spatial,
}
```

Add to `Model` struct after `tendon_name` (line 935):

```rust
/// Tendon type: Fixed (linear coupling) or Spatial (3D path routing).
pub tendon_type: Vec<TendonType>,

/// Solver parameters for tendon limit constraints (2 elements per tendon).
/// [0] = timeconst (>0) or -stiffness (≤0), [1] = dampratio or -damping.
/// Default: [0.0, 0.0] → uses model.default_eq_stiffness/default_eq_damping.
pub tendon_solref: Vec<[f64; 2]>,

/// Impedance parameters for tendon limit constraints (5 elements per tendon).
/// [d_min, d_max, width, midpoint, power]. Default: [0.9, 0.95, 0.001, 0.5, 2.0].
pub tendon_solimp: Vec<[f64; 5]>,

/// Velocity-dependent friction loss per tendon (N).
/// When > 0, adds a friction force opposing tendon velocity: F = -frictionloss * sign(v).
/// Mapped to joints via J^T. Default: 0.0 (no friction).
pub tendon_frictionloss: Vec<f64>,
```

Initialize in `Model::empty()` (after line 1603) and in `model_builder.rs`
(inside the Model struct literal at line 1449, alongside the existing tendon fields):
```rust
tendon_type: vec![],
tendon_solref: vec![],
tendon_solimp: vec![],
tendon_frictionloss: vec![],
```

##### Step 2: Model Builder — Populate Tendon Fields from MJCF

In `model_builder.rs`, add a `process_tendons()` method (or inline in the main
conversion function) that populates builder fields from `mjcf_model.tendons`. This
must run **before** `process_actuator()` calls, since actuators may reference tendons
via `self.tendon_name_to_id`.

Add these builder fields to `ModelBuilder`:
```rust
tendon_name_to_id: HashMap<String, usize>,
tendon_type: Vec<TendonType>,
tendon_range: Vec<(f64, f64)>,
tendon_limited: Vec<bool>,
tendon_stiffness: Vec<f64>,
tendon_damping: Vec<f64>,
tendon_frictionloss: Vec<f64>,
tendon_lengthspring: Vec<f64>,
tendon_length0: Vec<f64>,
tendon_name: Vec<Option<String>>,
tendon_solref: Vec<[f64; 2]>,
tendon_solimp: Vec<[f64; 5]>,
tendon_num: Vec<usize>,
tendon_adr: Vec<usize>,
wrap_type: Vec<WrapType>,
wrap_objid: Vec<usize>,
wrap_prm: Vec<f64>,
```

Then replace the empty tendon initialization in the Model struct literal
(lines 1449-1463) with `self.tendon_type`, `self.tendon_range`, etc.

Tendon processing logic:

```rust
fn process_tendons(
    &mut self,
    tendons: &[MjcfTendon],
) -> std::result::Result<(), ModelConversionError> {
    for (t_idx, tendon) in tendons.iter().enumerate() {
        self.tendon_name_to_id.insert(tendon.name.clone(), t_idx);
        self.tendon_type.push(match tendon.tendon_type {
            MjcfTendonType::Fixed => TendonType::Fixed,
            MjcfTendonType::Spatial => TendonType::Spatial,
        });
        self.tendon_range.push(tendon.range.unwrap_or((-f64::MAX, f64::MAX)));
        self.tendon_limited.push(tendon.limited);
        self.tendon_stiffness.push(tendon.stiffness);
        self.tendon_damping.push(tendon.damping);
        self.tendon_frictionloss.push(tendon.frictionloss);
        self.tendon_name.push(Some(tendon.name.clone()));
        self.tendon_solref.push([0.0, 0.0]); // Default: use model defaults
        self.tendon_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]); // MuJoCo defaults
        self.tendon_lengthspring.push(0.0); // Set to length0 if stiffness > 0
        self.tendon_length0.push(0.0);      // Computed after construction from qpos0

        let wrap_start = self.wrap_type.len();
        self.tendon_adr.push(wrap_start);

        match tendon.tendon_type {
            MjcfTendonType::Fixed => {
                for (joint_name, coef) in &tendon.joints {
                    // Resolve joint name → index → DOF address
                    let jnt_idx = *self.joint_name_to_id.get(joint_name.as_str())
                        .ok_or_else(|| ModelConversionError {
                            message: format!(
                                "Tendon '{}' references unknown joint '{}'",
                                tendon.name, joint_name
                            ),
                        })?;
                    let dof_adr = self.jnt_dof_adr[jnt_idx];
                    self.wrap_type.push(WrapType::Joint);
                    self.wrap_objid.push(dof_adr);
                    self.wrap_prm.push(*coef);
                }
            }
            MjcfTendonType::Spatial => {
                // Spatial tendon wrap objects: sites, wrapping geoms.
                // Populate wrap arrays for future spatial tendon support.
                // Log a one-time warning (matches Scope Decision: warn at load time).
                // model_builder.rs already has `use tracing::warn;` at line 22.
                warn!(
                    "Tendon '{}' is spatial — spatial tendons are not yet implemented \
                     and will produce zero forces. Fixed tendons are fully supported.",
                    tendon.name
                );
                for site_name in &tendon.sites {
                    let site_idx = *self.site_name_to_id.get(site_name.as_str())
                        .ok_or_else(|| ModelConversionError {
                            message: format!(
                                "Tendon '{}' references unknown site '{}'",
                                tendon.name, site_name
                            ),
                        })?;
                    self.wrap_type.push(WrapType::Site);
                    self.wrap_objid.push(site_idx);
                    self.wrap_prm.push(1.0); // Default divisor
                }
                // Note: wrapping geom entries would also go here.
            }
        }
        self.tendon_num.push(self.wrap_type.len() - wrap_start);
    }
    Ok(())
}
```

In the Model struct literal (`build()`, lines 1449-1463), replace the empty
initialization with:

```rust
ntendon: self.tendon_type.len(),
nwrap: self.wrap_type.len(),
tendon_type: self.tendon_type,
tendon_range: self.tendon_range,
tendon_limited: self.tendon_limited,
tendon_stiffness: self.tendon_stiffness,
tendon_damping: self.tendon_damping,
tendon_frictionloss: self.tendon_frictionloss,
tendon_lengthspring: self.tendon_lengthspring,
tendon_length0: self.tendon_length0,
tendon_num: self.tendon_num,
tendon_adr: self.tendon_adr,
tendon_name: self.tendon_name,
tendon_solref: self.tendon_solref,
tendon_solimp: self.tendon_solimp,
wrap_type: self.wrap_type,
wrap_objid: self.wrap_objid,
wrap_prm: self.wrap_prm,
```

**Resolve `tendon_lengthspring`:** After model construction, for each tendon with
`stiffness > 0.0` and no explicit `lengthspring` from MJCF: set
`tendon_lengthspring[t] = compute_tendon_length_at_qpos0(model, t)`. This matches
MuJoCo's behavior where the spring rest length defaults to the length at the
reference configuration.

**Resolve `tendon_length0`:** Computed identically — the tendon length at `qpos0`.
For fixed tendons: `tendon_length0[t] = Σ wrap_prm[w] * qpos0[wrap_objid[w]]` for
`w` in the tendon's wrap range.

##### Step 3: Model Builder — Accept Tendon Transmission

Replace the tendon rejection block (`model_builder.rs:1009-1018`). The existing
code uses a `let (trntype, trnid) = if ... else if ...` chain (line 1001) that
returns a tuple, with pushes at lines 1060-1062. Replace the `return Err(...)`
arm with a tuple return matching the Joint/Site pattern:

```rust
} else if let Some(ref tendon_name) = actuator.tendon {
    // Tendon transmission: resolve tendon name → index
    let tendon_idx = *self.tendon_name_to_id.get(tendon_name.as_str())
        .ok_or_else(|| ModelConversionError {
            message: format!(
                "Actuator '{}' references unknown tendon '{}'",
                actuator.name, tendon_name
            ),
        })?;
    (ActuatorTransmission::Tendon, tendon_idx)
}
```

The existing `self.actuator_trntype.push(trntype)` and
`self.actuator_trnid.push(trnid)` at lines 1060-1062 handle the rest.

Add `tendon_name_to_id: HashMap<String, usize>` as a builder field, populated
alongside the tendon construction loop in Step 2 (matching the existing
`joint_name_to_id` pattern).

##### Step 4: MJCF Validation

Add to `sim/L0/mjcf/src/validation.rs`:

```rust
/// Validate tendon definitions against model structure.
/// Returns Err on fatal issues (unknown references), matching the existing
/// validation pattern in validation.rs which uses Result<T> with MjcfError.
pub fn validate_tendons(model: &MjcfModel) -> Result<()> {
    // Collect joint and site names from the body tree (joints/sites are nested
    // inside MjcfBody, not top-level on MjcfModel). Matches the pattern used
    // by validate() which traverses the tree via traverse_body().
    let mut joint_names = HashSet::new();
    let mut site_names = HashSet::new();
    fn collect_names(body: &MjcfBody, joints: &mut HashSet<String>, sites: &mut HashSet<String>) {
        for joint in &body.joints {
            joints.insert(joint.name.clone());
        }
        for site in &body.sites {
            sites.insert(site.name.clone());
        }
        for child in &body.children {
            collect_names(child, joints, sites);
        }
    }
    for body in &model.worldbody.children {
        collect_names(body, &mut joint_names, &mut site_names);
    }

    for tendon in &model.tendons {
        // Fixed tendons must reference existing joints with valid coefficients
        if tendon.tendon_type == MjcfTendonType::Fixed {
            if tendon.joints.is_empty() {
                return Err(MjcfError::invalid_option(
                    "tendon",
                    format!("Fixed tendon '{}' has no joint entries", tendon.name),
                ));
            }
            for (joint_name, coef) in &tendon.joints {
                if !joint_names.contains(joint_name.as_str()) {
                    return Err(MjcfError::invalid_option(
                        "tendon",
                        format!(
                            "Tendon '{}' references unknown joint '{}'",
                            tendon.name, joint_name
                        ),
                    ));
                }
                if !coef.is_finite() {
                    return Err(MjcfError::invalid_option(
                        "tendon",
                        format!(
                            "Tendon '{}' has non-finite coefficient {} for joint '{}'",
                            tendon.name, coef, joint_name
                        ),
                    ));
                }
            }
        }

        // Spatial tendons must reference existing sites
        if tendon.tendon_type == MjcfTendonType::Spatial {
            if tendon.sites.len() < 2 {
                return Err(MjcfError::invalid_option(
                    "tendon",
                    format!(
                        "Spatial tendon '{}' needs at least 2 sites, has {}",
                        tendon.name, tendon.sites.len()
                    ),
                ));
            }
            for site_name in &tendon.sites {
                if !site_names.contains(site_name.as_str()) {
                    return Err(MjcfError::invalid_option(
                        "tendon",
                        format!(
                            "Tendon '{}' references unknown site '{}'",
                            tendon.name, site_name
                        ),
                    ));
                }
            }
        }

        // Parameter validation
        if tendon.stiffness < 0.0 {
            return Err(MjcfError::invalid_option(
                "tendon",
                format!("Tendon '{}' has negative stiffness {}", tendon.name, tendon.stiffness),
            ));
        }
        if tendon.damping < 0.0 {
            return Err(MjcfError::invalid_option(
                "tendon",
                format!("Tendon '{}' has negative damping {}", tendon.name, tendon.damping),
            ));
        }
        if tendon.limited {
            if let Some((min, max)) = tendon.range {
                if min >= max {
                    return Err(MjcfError::invalid_option(
                        "tendon",
                        format!("Tendon '{}' has invalid range [{}, {}]", tendon.name, min, max),
                    ));
                }
            } else {
                return Err(MjcfError::invalid_option(
                    "tendon",
                    format!("Tendon '{}' is limited but has no range", tendon.name),
                ));
            }
        }
    }
    Ok(())
}
```

The existing `validation.rs` uses `MjcfError::invalid_option(option, message)` (the
helper method on `MjcfError`) for validation failures and returns `Result<()>`. This
function follows the same pattern. The `MjcfError` and `Result` types are from
`crate::error`. Joint and site names are collected by recursively traversing the body
tree (matching the `traverse_body()` pattern in `validate()`), since `MjcfModel` stores
joints and sites nested inside `MjcfBody`, not as top-level collections. The function
also needs `use crate::types::{MjcfBody, MjcfTendonType};` added to the imports
(or add to the existing `use crate::types::` block if already present).

Call `validate_tendons()` from the existing `validate()` function after body/joint
validation, or call it from the model builder before tendon construction.

##### Step 5: `mj_fwd_tendon()` — Tendon Kinematics (Position-Dependent)

New function in `mujoco_pipeline.rs`, called from `mj_fwd_position()` after site
pose computation (line 2603) and before subtree mass/COM computation (line 2607).

This placement matches MuJoCo's pipeline: `mj_tendon` runs inside `mj_fwdPosition`,
after `mj_kinematics` (which computes body/geom/site poses) and before `mj_makeM`.

```rust
/// Compute tendon lengths and Jacobians from current joint state.
///
/// For fixed tendons: L = Σ coef_i * qpos[dof_adr_i], J[dof_adr_i] = coef_i.
/// For spatial tendons: silently skipped (deferred to spatial tendon PR).
///
/// Called from mj_fwd_position() after site transforms, before subtree COM.
fn mj_fwd_tendon(model: &Model, data: &mut Data) {
    if model.ntendon == 0 {
        return;
    }

    for t in 0..model.ntendon {
        match model.tendon_type[t] {
            TendonType::Fixed => {
                mj_fwd_tendon_fixed(model, data, t);
            }
            TendonType::Spatial => {
                // Spatial tendons not yet implemented.
                // Set length to 0, Jacobian to zero vector.
                // This is safe: zero-length tendon produces zero spring force
                // and zero actuation force. Sensors will read 0.
                data.ten_length[t] = 0.0;
                data.ten_J[t].fill(0.0);
            }
        }
    }
}

/// Fixed tendon kinematics for a single tendon.
///
/// Fixed tendon length is a linear combination of joint positions:
///   L_t = Σ_w coef_w * qpos[dof_adr_w]
///
/// The Jacobian is constant (configuration-independent):
///   J_t[dof_adr_w] = coef_w
///
/// Wrap entries for fixed tendons use:
///   wrap_type[w] = WrapType::Joint
///   wrap_objid[w] = DOF address (index into qpos/qvel)
///   wrap_prm[w] = coupling coefficient
#[inline]
fn mj_fwd_tendon_fixed(model: &Model, data: &mut Data, t: usize) {
    let adr = model.tendon_adr[t];
    let num = model.tendon_num[t];

    // Zero the Jacobian row. For fixed tendons, only a few entries are non-zero.
    data.ten_J[t].fill(0.0);

    let mut length = 0.0;
    for w in adr..(adr + num) {
        debug_assert_eq!(model.wrap_type[w], WrapType::Joint);
        let dof_adr = model.wrap_objid[w];
        let coef = model.wrap_prm[w];

        // Length contribution: coef * q
        // For hinge joints, dof_adr indexes into qpos (1-DOF, qposadr == dofadr).
        // For slide joints, same (1-DOF).
        // Ball/Free joints in fixed tendons are unusual but supported: the
        // coefficient applies to the DOF at dof_adr (one component of the
        // multi-DOF joint).
        if dof_adr < data.qpos.len() {
            length += coef * data.qpos[dof_adr];
        }

        // Jacobian: dL/dq[dof_adr] = coef
        if dof_adr < model.nv {
            data.ten_J[t][dof_adr] = coef;
        }
    }

    data.ten_length[t] = length;
}
```

**Borrow-checker note:** `data.ten_J[t]` is a `DVector<f64>` in a `Vec`. We index
by `t` (the tendon index) and then by `dof_adr` within the vector. No aliasing
issues — `ten_J`, `ten_length`, and `qpos` are separate fields of `Data`.

**qpos vs qvel indexing for fixed tendons:** MuJoCo fixed tendons use generalized
coordinate indices. For 1-DOF joints (hinge, slide), `qposadr == dofadr` so the
wrap_objid (DOF address) indexes correctly into both `qpos` and `qvel`. For
multi-DOF joints (ball: 4 qpos but 3 qvel; free: 7 qpos but 6 qvel), MuJoCo's
fixed tendons couple to individual DOFs, not to quaternion components. The model
builder must store the DOF address (into qvel space), not the qpos address. For
hinge/slide this distinction is irrelevant (they're equal). Ball/Free joints in
fixed tendons are rare and should be validated — if encountered, the builder should
map to the appropriate qvel DOF index and use `qvel[dof_adr]` for velocity. The
length computation for multi-DOF joints requires using `qvel` (generalized velocity)
rather than `qpos` (which includes quaternions). This is a known subtlety — for the
initial implementation, we validate that fixed tendon joints are hinge or slide only,
and emit a warning for ball/free.

##### Step 6: Tendon Velocity Computation

Add tendon velocity computation to `mj_fwd_velocity()`. This is a single matrix-vector
dot product per tendon. Add after the existing velocity kinematics in `mj_fwd_velocity()`
(after body velocity computation, before the function returns):

```rust
// Tendon velocities: v_t = J_t · qvel
for t in 0..model.ntendon {
    data.ten_velocity[t] = data.ten_J[t].dot(&data.qvel);
}
```

This matches MuJoCo's `ten_velocity = ten_J * qvel` computation.

##### Step 7: Tendon Passive Forces in `mj_fwd_passive()`

Add tendon spring/damper/friction forces after the existing joint passive force
computation in `mj_fwd_passive()` (after `model.visit_joints(&mut visitor)`,
before the function returns):

```rust
// Tendon passive forces: spring + damper + friction loss.
// Pattern matches joint passive forces: spring/damper skipped in implicit mode
// (handled by mj_fwd_acceleration_implicit), friction always explicit.
for t in 0..model.ntendon {
    let length = data.ten_length[t];
    let velocity = data.ten_velocity[t];
    let mut force = 0.0;

    if !implicit_mode {
        // Spring: F = -k * (L - L_ref)
        let k = model.tendon_stiffness[t];
        if k > 0.0 {
            let l_ref = model.tendon_lengthspring[t];
            force -= k * (length - l_ref);
        }

        // Damper: F = -b * v
        let b = model.tendon_damping[t];
        if b > 0.0 {
            force -= b * velocity;
        }
    }
    // NOTE: In implicit mode, tendon spring/damper forces are skipped here.
    // Joint springs/dampers ARE handled in implicit mode — via diagonal K and D
    // modifications in mj_fwd_acceleration_implicit(). Tendon springs/dampers
    // couple multiple joints (K_q = J^T * k * J, which is generally non-diagonal),
    // so they CANNOT be absorbed into the existing diagonal implicit modification.
    // For now, tendon springs/dampers produce zero force in implicit mode — this
    // is a known limitation, NOT equivalent to joint spring/damper handling.
    // A future enhancement could add the tendon contribution to scratch_m_impl
    // as a non-diagonal term, but this would require refactoring the implicit
    // solver.

    // Friction loss: F = -frictionloss * sign(v)
    // Always explicit (velocity-sign-dependent, cannot linearize).
    // Matches joint friction_loss pattern in PassiveForceVisitor.
    let fl = model.tendon_frictionloss[t];
    if fl > 0.0 && velocity.abs() > 1e-10 {
        force -= fl * velocity.signum();
    }

    data.ten_force[t] = force;

    // Map tendon force to joint forces via sparse J^T multiplication.
    // For fixed tendons: qfrc_passive[dof_adr] += coef * force.
    // Uses wrap arrays directly (sparse) rather than iterating over all nv DOFs.
    if force != 0.0 {
        let adr = model.tendon_adr[t];
        let num = model.tendon_num[t];
        for w in adr..(adr + num) {
            let dof_adr = model.wrap_objid[w];
            let coef = model.wrap_prm[w];
            if dof_adr < model.nv {
                data.qfrc_passive[dof_adr] += coef * force;
            }
        }
    }
}
```

**Note on spatial tendons:** When spatial tendons are added in the follow-up, their
Jacobians are dense (all `nv` entries may be non-zero). The J^T multiplication for
spatial tendons should use the dense `ten_J[t]` vector instead of the sparse wrap
arrays. The dispatch on `tendon_type` in the force-mapping block will select the
appropriate path.

##### Step 8: Tendon Limit Forces in `mj_fwd_constraint()`

Tendon limits are **constraint forces**, not passive forces. They belong in
`mj_fwd_constraint()` (`mujoco_pipeline.rs:8500`), alongside joint limit constraints.

Add after the joint limit loop (after line 8554):

```rust
// Tendon limit constraints
for t in 0..model.ntendon {
    if !model.tendon_limited[t] {
        continue;
    }

    let (limit_min, limit_max) = model.tendon_range[t];
    let length = data.ten_length[t];

    let (limit_stiffness, limit_damping) = solref_to_penalty(
        model.tendon_solref[t],
        model.default_eq_stiffness,
        model.default_eq_damping,
        model.timestep,
    );

    // Lower limit violation: length < min
    if length < limit_min {
        let penetration = limit_min - length;
        // Tendon velocity into the limit (negative velocity = moving toward shorter)
        let vel_into = (-data.ten_velocity[t]).max(0.0);
        let force = limit_stiffness * penetration + limit_damping * vel_into;

        // Map to joint forces via J^T (force pushes tendon toward longer)
        let adr = model.tendon_adr[t];
        let num = model.tendon_num[t];
        for w in adr..(adr + num) {
            let dof_adr = model.wrap_objid[w];
            let coef = model.wrap_prm[w];
            if dof_adr < model.nv {
                // Positive tendon force → increase length
                data.qfrc_constraint[dof_adr] += coef * force;
            }
        }
    }

    // Upper limit violation: length > max
    if length > limit_max {
        let penetration = length - limit_max;
        let vel_into = data.ten_velocity[t].max(0.0);
        let force = limit_stiffness * penetration + limit_damping * vel_into;

        // Map to joint forces via J^T (force pushes tendon toward shorter)
        let adr = model.tendon_adr[t];
        let num = model.tendon_num[t];
        for w in adr..(adr + num) {
            let dof_adr = model.wrap_objid[w];
            let coef = model.wrap_prm[w];
            if dof_adr < model.nv {
                data.qfrc_constraint[dof_adr] -= coef * force;
            }
        }
    }
}
```

This follows exactly the same pattern as joint limits (lines 8513-8554): check
limited flag, get bounds, `solref_to_penalty()`, compute penetration and
velocity-into-limit, apply stiffness + damping force.

##### Step 9: Tendon Actuation in `mj_fwd_actuation()`

Replace the placeholder at `mujoco_pipeline.rs:6017-6020`:

```rust
ActuatorTransmission::Tendon => {
    // Tendon transmission: apply ctrl * gear as force along tendon,
    // mapped to joints via tendon Jacobian J^T.
    let tendon_id = trnid;
    if tendon_id < model.ntendon {
        let force = ctrl * gear;
        // Sparse J^T multiplication using wrap arrays
        let adr = model.tendon_adr[tendon_id];
        let num = model.tendon_num[tendon_id];
        for w in adr..(adr + num) {
            let dof_adr = model.wrap_objid[w];
            let coef = model.wrap_prm[w];
            if dof_adr < model.nv {
                data.qfrc_actuator[dof_adr] += coef * force;
            }
        }
    }
}
ActuatorTransmission::Site => {
    // Site transmission: not yet implemented (requires spatial tendon support).
    // No-op placeholder preserved.
}
```

##### Step 10: Sensor Stubs → Real Reads

In `mj_sensor_pos()` (line 5101), replace TendonPos stub:

```rust
MjSensorType::TendonPos => {
    let tendon_id = model.sensor_objid[sensor_id];
    let value = if tendon_id < model.ntendon {
        data.ten_length[tendon_id]
    } else {
        0.0
    };
    sensor_write(&mut data.sensordata, adr, 0, value);
}
```

In `mj_sensor_vel()` (line 5307), replace TendonVel stub:

```rust
MjSensorType::TendonVel => {
    let tendon_id = model.sensor_objid[sensor_id];
    let value = if tendon_id < model.ntendon {
        data.ten_velocity[tendon_id]
    } else {
        0.0
    };
    sensor_write(&mut data.sensordata, adr, 0, value);
}
```

In `mj_sensor_pos()` (line 5093), replace the combined `Tendon | Site` stub
with separate arms (the existing code is `ActuatorTransmission::Tendon |
ActuatorTransmission::Site => { sensor_write(..., 0.0); }`):

```rust
ActuatorTransmission::Tendon => {
    let tendon_id = model.actuator_trnid[act_id];
    let value = if tendon_id < model.ntendon {
        data.ten_length[tendon_id] * model.actuator_gear[act_id]
    } else {
        0.0
    };
    sensor_write(&mut data.sensordata, adr, 0, value);
}
ActuatorTransmission::Site => {
    // Site transmission length not yet available (requires spatial tendon support).
    sensor_write(&mut data.sensordata, adr, 0, 0.0);
}
```

In `mj_sensor_vel()` (line 5299), replace the combined `Tendon | Site` stub
with separate arms:

```rust
ActuatorTransmission::Tendon => {
    let tendon_id = model.actuator_trnid[act_id];
    let value = if tendon_id < model.ntendon {
        data.ten_velocity[tendon_id] * model.actuator_gear[act_id]
    } else {
        0.0
    };
    sensor_write(&mut data.sensordata, adr, 0, value);
}
ActuatorTransmission::Site => {
    // Site transmission velocity not yet available (requires spatial tendon support).
    sensor_write(&mut data.sensordata, adr, 0, 0.0);
}
```

##### Step 11: Pipeline Integration Points

Modify `mj_fwd_position()` (line 2603) — add `mj_fwd_tendon` call after site poses:

```rust
// Site poses (existing, line 2593-2603)
for site_id in 0..model.nsite { ... }

// Tendon kinematics (NEW)
mj_fwd_tendon(model, data);

// Subtree mass and COM (existing, line 2607)
```

Modify `mj_fwd_velocity()` — add tendon velocity after body velocity computation
(see Step 6).

No changes needed to `forward()` or `forward_skip_sensors()` — both call
`mj_fwd_position()` and `mj_fwd_velocity()` which now include tendon computation
internally. The RK4 integrator's `forward_skip_sensors()` path inherits tendon
support automatically, so tendon forces are correctly evaluated at each RK4 stage.

##### Step 12: `MjcfTendon` Export

Add `MjcfTendon` and `MjcfTendonType` to `sim/L0/mjcf/src/lib.rs` line 191:

```rust
MjcfTendon, MjcfTendonDefaults, MjcfTendonType,
```

##### Step 13: Cargo.toml — sim-core Dependency on sim-tendon

No dependency needed. The pipeline does not call sim-tendon code — it reimplements
the fixed tendon computation inline (a 10-line loop). The sim-tendon crate remains
a standalone reference implementation with its own richer type system (`CableProperties`,
`TendonActuator` trait, etc.). Pipeline-side, tendon computation is a thin layer
over the Model/Data arrays, matching the existing pattern for joints, contacts, and
constraints.

**Rationale:** Adding a sim-core → sim-tendon dependency would require bridging
between two type systems (`Model`/`Data` arrays vs. `FixedTendon`/`CableProperties`
structs). The pipeline computation is simple enough (linear combination for length,
constant Jacobian, spring/damper force) that reimplementation is cleaner than
adaptation. The sim-tendon crate remains useful for standalone tendon analysis,
prototyping, and as documentation of the algorithms.

#### Acceptance Criteria

1. **Fixed tendon kinematics:** `ten_length[t]` equals `Σ coef_w * qpos[dof_adr_w]`
   for all fixed tendons, verified by constructing a 2-joint chain with a differential
   tendon (`coef = [1.0, -1.0]`) and checking length matches `q1 - q2`.

2. **Tendon Jacobian correctness:** `ten_J[t][dof_adr]` equals the coupling
   coefficient for each wrap entry. Verified by comparing `ten_J[t].dot(&qvel)` to
   finite-difference `(L(q + ε·e_i) - L(q)) / ε` for each DOF `i`, with tolerance
   `≤ 1e-8` (exact for fixed tendons, but FD test catches indexing bugs).

3. **Tendon velocity:** `ten_velocity[t]` equals `ten_J[t].dot(&data.qvel)` after
   each step. Verified by the same differential tendon: velocity should equal
   `qdot1 - qdot2`.

4. **Tendon actuation equivalence:** A tendon-driven actuator with `coef = 1.0` on
   joint `j` and `gear = 1.0` produces the same `qfrc_actuator[j]` as a direct
   `ActuatorTransmission::Joint` actuator with the same `ctrl` and `gear`. Verified
   by running both configurations and comparing `qfrc_actuator` to `≤ 1e-14`.

5. **Differential tendon:** An antagonistic tendon with `coef = [1.0, -1.0]` on
   joints `j1, j2` with `ctrl = 1.0` and `gear = 1.0` produces
   `qfrc_actuator[j1] = 1.0` and `qfrc_actuator[j2] = -1.0`.

6. **Passive spring force:** A tendon with `stiffness = 100.0`, `lengthspring = 0.0`,
   and `coef = [1.0]` on a hinge joint produces `qfrc_passive[j] = -100 * q_j`.
   Verified by setting `qpos[j] = 0.1` and checking `qfrc_passive[j] ≈ -10.0`.

7. **Passive damper force:** A tendon with `damping = 10.0` and `coef = [1.0]` on a
   hinge joint produces `qfrc_passive[j] = -10 * qdot_j`. Verified by setting
   `qvel[j] = 0.5` and checking `qfrc_passive[j] ≈ -5.0`.

8. **Friction loss:** A tendon with `frictionloss = 2.0` and positive velocity
   produces `ten_force` contribution of `-2.0` (opposing motion). Verified similarly.

9. **Limit constraint (lower):** A tendon with `limited = true`, `range = (0.1, 0.5)`,
   and length = 0.05 produces a positive constraint force pushing toward longer
   (via `qfrc_constraint`). Force magnitude follows `solref_to_penalty` with default
   parameters. Verified by checking `qfrc_constraint` is non-zero and in the correct
   direction.

10. **Limit constraint (upper):** Same tendon with length = 0.6 produces a negative
    constraint force pushing toward shorter. Verified symmetrically.

11. **TendonPos sensor:** After `forward()`, `sensordata` for a TendonPos sensor
    reads `ten_length[tendon_id]`, not 0.0.

12. **TendonVel sensor:** After `forward()`, `sensordata` for a TendonVel sensor
    reads `ten_velocity[tendon_id]`, not 0.0.

13. **ActuatorPos/ActuatorVel sensors:** For a tendon-transmission actuator, the
    sensors read `ten_length * gear` and `ten_velocity * gear` respectively.

14. **Zero-tendon overhead:** A model with `ntendon == 0` has identical performance
    to before this PR. All tendon functions early-return on `ntendon == 0`.

15. **Spatial tendon graceful degradation:** A model with a spatial tendon parses
    without error. The spatial tendon's `ten_length` is 0.0, `ten_J` is zero,
    and no forces are produced. A warning is logged at model load time.

16. **MJCF round-trip:** A model with `<tendon><fixed name="t1"><joint joint="j1" coef="1.0"/></fixed></tendon>` parses, builds a Model with `ntendon = 1`, and
    the tendon correctly couples to joint `j1`.

17. **Model builder rejects bad references:** An MJCF file referencing a nonexistent
    joint in a fixed tendon produces a `ModelConversionError`.

18. **Existing tests pass:** All existing tests (implicit integration, RK4, collision,
    sensors, etc.) remain green. No behavioral changes for zero-tendon models.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — modify:
  - `Model` struct: add `tendon_type`, `tendon_solref`, `tendon_solimp`,
    `tendon_frictionloss` fields (after line 935)
  - `Model::empty()`: initialize new fields (after line 1603)
  - `mj_fwd_position()`: add `mj_fwd_tendon()` call (after line 2603)
  - `mj_fwd_velocity()`: add tendon velocity computation
  - `mj_fwd_passive()`: add tendon spring/damper/friction forces
  - `mj_fwd_constraint()`: add tendon limit constraint forces (after line 8554)
  - `mj_fwd_actuation()`: replace Tendon placeholder (line 6017-6020)
  - `mj_sensor_pos()`: replace TendonPos stub (line 5101) and ActuatorPos tendon
    stub (line 5093)
  - `mj_sensor_vel()`: replace TendonVel stub (line 5307) and ActuatorVel tendon
    stub (line 5299)
  - New functions: `mj_fwd_tendon()`, `mj_fwd_tendon_fixed()` (~40 lines total)
- `sim/L0/mjcf/src/model_builder.rs` — modify:
  - Add builder fields for tendon state (`tendon_name_to_id`, `tendon_type`, etc.)
  - New `process_tendons()` method (~60 lines)
  - Replace tendon rejection in `process_actuator()` (line 1009-1018) with tuple return
  - Replace empty tendon initialization in `build()` (lines 1449-1463) with builder fields
  - Add `tendon_length0` / `tendon_lengthspring` post-construction computation
- `sim/L0/mjcf/src/validation.rs` — modify:
  - Add `validate_tendons()` function (~60 lines)
- `sim/L0/mjcf/src/lib.rs` — modify:
  - Export `MjcfTendon`, `MjcfTendonType` (line 191)
- `sim/L0/core/src/mujoco_pipeline.rs` — new test module `tendon_tests` (~200 lines):
  - Fixed tendon length computation test (AC #1)
  - Jacobian correctness test (AC #2)
  - Tendon velocity test (AC #3)
  - Actuation equivalence test (AC #4)
  - Differential tendon test (AC #5)
  - Passive spring test (AC #6)
  - Passive damper test (AC #7)
  - Friction loss test (AC #8)
  - Lower limit test (AC #9)
  - Upper limit test (AC #10)
  - TendonPos sensor test (AC #11)
  - TendonVel sensor test (AC #12)
  - Actuator sensor test (AC #13)
  - Zero-tendon overhead test (AC #14)
  - Spatial tendon graceful skip test (AC #15)
  - MJCF round-trip test (AC #16)
  - Bad reference rejection test (AC #17)

---

### 5. Muscle Pipeline
**Status:** ✅ Done | **Effort:** L | **Prerequisites:** #4 ✅ (tendon pipeline provides `ten_length`, `ten_velocity`, `ten_J`)

#### Architectural Decision: MuJoCo FLV Model (Not sim-muscle)

This PR implements **MuJoCo's simplified FLV muscle model** directly in the pipeline,
matching MuJoCo's `mju_muscleGain`, `mju_muscleBias`, and `mju_muscleDynamics` semantics.
It does **not** wire in the `sim-muscle` crate's richer Hill-type model.

**Rationale:**
- The MJCF parser already parses MuJoCo's muscle parameters (`timeconst`, `range`,
  `force`, `scale`, `lmin`, `lmax`, `vmax`, `fpmax`, `fvmax`). These map directly to
  MuJoCo's `dynprm[3]` and `gainprm[9]`/`biasprm[9]` arrays.
- MuJoCo's FLV curves are ~80 lines of piecewise-quadratic code. No external crate needed.
- Avoids adding a `sim-core` → `sim-muscle` dependency for the pipeline hot path.
- Produces results directly comparable to MuJoCo ground truth for any MJCF model.
- `sim-muscle`'s richer Hill model (compliant tendons, pennation geometry, Newton
  iteration) can be offered as an alternative `ActuatorDynamics::HillMuscle` variant
  in a follow-up, for users who need biomechanical fidelity beyond MuJoCo parity.

#### Current State

**Pipeline scaffolds** (all in `mujoco_pipeline.rs`):

| Component | Location | Status |
|-----------|----------|--------|
| `ActuatorDynamics::Muscle` | Enum, line 435 | Defined |
| `data.act: DVector<f64>` | Data, line 1277 | Declared, zero-initialized, **never updated** |
| `data.ctrl: DVector<f64>` | Data, line 1275 | Used directly as force (no dynamics applied) |
| `data.qfrc_actuator: DVector<f64>` | Data, line 1279 | Populated by `mj_fwd_actuation()` |
| `actuator_dyntype: Vec<ActuatorDynamics>` | Model, line 907 | Stored, **never read** in pipeline |
| `actuator_act_adr: Vec<usize>` | Model, line 919 | Stored, **never read** in pipeline |
| `actuator_act_num: Vec<usize>` | Model, line 921 | Stored, **incorrectly assigns 2 for Muscle** |
| `ten_length: Vec<f64>` | Data, line 1385 | Populated by `mj_fwd_tendon()` ✅ |
| `ten_velocity: Vec<f64>` | Data, line 1388 | Populated in `mj_fwd_velocity()` ✅ |
| `ten_J: Vec<DVector<f64>>` | Data, line 1393 | Populated by `mj_fwd_tendon()` ✅ |

**MJCF parsing** (complete — all muscle attributes parsed, then discarded):

| Component | Location | Status |
|-----------|----------|--------|
| `MjcfActuator.muscle_timeconst` | `types.rs:1819` | Parsed, **not transferred to Model** |
| `MjcfActuator.range` | `types.rs:1823` | Parsed, **not transferred to Model** |
| `MjcfActuator.force` | `types.rs:1826` | Parsed, **not transferred to Model** |
| `MjcfActuator.scale` | `types.rs:1829` | Parsed, **not transferred to Model** |
| `MjcfActuator.lmin` | `types.rs:1831` | Parsed, **not transferred to Model** |
| `MjcfActuator.lmax` | `types.rs:1833` | Parsed, **not transferred to Model** |
| `MjcfActuator.vmax` | `types.rs:1835` | Parsed, **not transferred to Model** |
| `MjcfActuator.fpmax` | `types.rs:1837` | Parsed, **not transferred to Model** |
| `MjcfActuator.fvmax` | `types.rs:1839` | Parsed, **not transferred to Model** |
| Muscle attribute parsing | `parser.rs:1134-1177` | Complete with tests |

**Bugs to fix:**
- `actuator_act_num` for Muscle is set to 2 (`model_builder.rs:1231`) but MuJoCo
  muscles have **1 activation state** (the activation level `act`). The builder
  comment says "MuJoCo semantics: Muscle=2" — this is wrong. MuJoCo's `mjDYN_MUSCLE`
  contributes exactly 1 entry to `mjData.act`.

**Missing infrastructure:**
- No `actuator_dynprm`, `actuator_gainprm`, `actuator_biasprm` fields in Model.
- No `actuator_length`, `actuator_velocity` fields in Data (needed for muscle L/V).
- No `actuator_lengthrange` or `actuator_acc0` in Model (needed for F0 auto-computation
  and length normalization).
- `mj_fwd_actuation()` ignores `actuator_dyntype` entirely — applies `ctrl * gear`
  for all actuators regardless of dynamics type.
- `actuator_ctrlrange` and `actuator_forcerange` are stored but never enforced.

#### Objective

Implement MuJoCo-compatible muscle actuation: activation dynamics, FLV force curves,
and force-to-joint mapping. Also fix the actuation pipeline to respect activation
states and control/force clamping.

#### Specification

##### Step 1: Fix `actuator_act_num` for Muscle

In `model_builder.rs`, change the Muscle activation state count from 2 to 1:

```rust
let act_num = match dyntype {
    ActuatorDynamics::None => 0,
    ActuatorDynamics::Filter | ActuatorDynamics::Integrator => 1,
    ActuatorDynamics::Muscle => 1,  // was 2, MuJoCo muscles have 1 activation state
};
```

##### Step 2: Add Model Fields for Muscle Parameters

Add to `Model` struct (after `actuator_act_num`, line 921):

```rust
/// Dynamics parameters per actuator (3 elements each).
/// For Muscle: [tau_act, tau_deact, tausmooth]. Default: [0.01, 0.04, 0.0].
/// For Filter: [tau, 0, 0]. For Integrator/None: unused.
pub actuator_dynprm: Vec<[f64; 3]>,

/// Gain parameters per actuator (9 elements each).
/// For Muscle: [range0, range1, force, scale, lmin, lmax, vmax, fpmax, fvmax].
/// Default: [0.75, 1.05, -1.0, 200.0, 0.5, 1.6, 1.5, 1.3, 1.2].
/// For other types: [gain, 0, ...] (gain multiplier, index 0 only).
pub actuator_gainprm: Vec<[f64; 9]>,

/// Bias parameters per actuator (9 elements each).
/// For Muscle: same layout as gainprm (shared parameter set in MuJoCo).
/// For other types: [bias0, bias1, bias2, 0, ...] (constant + length + velocity).
pub actuator_biasprm: Vec<[f64; 9]>,

/// Actuator length range [min, max] — the transmission length extremes.
/// For tendon-transmission muscles: computed from tendon length at joint limits.
/// Used to normalize muscle length: L = range0 + (len - lengthrange0) / L0.
pub actuator_lengthrange: Vec<(f64, f64)>,

/// Acceleration produced by unit actuator force (scalar, per actuator).
/// Used for auto-computing F0 when `force < 0`: F0 = scale / acc0.
/// Computed at model build time from M^{-1} and the transmission Jacobian.
pub actuator_acc0: Vec<f64>,
```

Initialize all to empty vecs in `Model::empty()`.

##### Step 3: Add Data Fields for Actuator State

Add to `Data` struct (after `qfrc_actuator`, line 1279):

```rust
/// Actuator length (gear * transmission_length, length `nu`).
/// For Joint transmission: `gear * qpos[qpos_adr]` (hinge/slide only).
/// For Tendon transmission: `gear * ten_length[tendon_id]`.
pub actuator_length: Vec<f64>,

/// Actuator velocity (gear * transmission_velocity, length `nu`).
/// For Joint transmission: `gear * qvel[dof_adr]` (hinge/slide only).
/// For Tendon transmission: `gear * ten_velocity[tendon_id]`.
pub actuator_velocity: Vec<f64>,

/// Actuator force output (length `nu`).
/// The scalar force produced by each actuator after gain/bias/activation.
pub actuator_force: Vec<f64>,

/// Activation time-derivative (length `na`).
/// Computed by `mj_fwd_actuation()`, integrated by the integrator (Euler/RK4).
/// Separating derivative from integration matches MuJoCo's `mjData.act_dot`
/// and is required for correct RK4 integration of activation states.
pub act_dot: DVector<f64>,
```

Initialize in `make_data()` struct literal (alongside existing fields):
`actuator_length`, `actuator_velocity`, `actuator_force` as `vec![0.0; self.nu]`,
`act_dot` as `DVector::zeros(self.na)`.

Also add RK4 scratch buffers for activation (alongside `rk4_qvel`/`rk4_qacc`):

```rust
// RK4 activation scratch (pre-allocated to avoid per-step heap allocation)
rk4_act_saved: DVector::zeros(self.na),
rk4_act_dot: std::array::from_fn(|_| DVector::zeros(self.na)),
```

Update `Data::reset()` to clear the new fields:

```rust
self.act_dot.fill(0.0);
self.actuator_length.fill(0.0);
self.actuator_velocity.fill(0.0);
self.actuator_force.fill(0.0);
```

##### Step 4: Model Builder — Transfer Muscle Parameters

In `model_builder.rs`, `process_actuator()`:

**Fix ctrlrange/forcerange gating:** The existing builder ignores `ctrllimited`
and always stores a ctrlrange (defaulting to `(-1, 1)`). Step 9 introduces
control clamping, which would incorrectly clamp when `ctrllimited = false` but
`ctrlrange` is explicitly set. Fix both to mirror MuJoCo's gating:

```rust
// Replace existing ctrlrange push:
self.actuator_ctrlrange.push(
    if actuator.ctrllimited {
        actuator.ctrlrange.unwrap_or((-1.0, 1.0))
    } else {
        (f64::NEG_INFINITY, f64::INFINITY)
    },
);

// Replace existing forcerange push (add forcelimited gating):
self.actuator_forcerange.push(
    if actuator.forcelimited {
        actuator.forcerange.unwrap_or((f64::NEG_INFINITY, f64::INFINITY))
    } else {
        (f64::NEG_INFINITY, f64::INFINITY)
    },
);
```

After pushing `actuator_act_num`, add the new muscle parameter fields:

```rust
// Dynamics parameters
let dynprm = match actuator.actuator_type {
    MjcfActuatorType::Muscle => [
        actuator.muscle_timeconst.0,  // tau_act (default 0.01)
        actuator.muscle_timeconst.1,  // tau_deact (default 0.04)
        0.0,                           // tausmooth (default 0, hard switch)
    ],
    MjcfActuatorType::Position | MjcfActuatorType::Velocity => [
        // Filter time constant: position/velocity servos use timeconst
        // from general actuator attributes if available, else 0.0
        0.0, 0.0, 0.0,
    ],
    _ => [0.0; 3],
};
self.actuator_dynprm.push(dynprm);

// Gain/Bias parameters (shared layout for muscle)
let gainprm = match actuator.actuator_type {
    MjcfActuatorType::Muscle => [
        actuator.range.0,   // range[0], default 0.75
        actuator.range.1,   // range[1], default 1.05
        actuator.force,     // force, default -1.0 (auto)
        actuator.scale,     // scale, default 200.0
        actuator.lmin,      // lmin, default 0.5
        actuator.lmax,      // lmax, default 1.6
        actuator.vmax,      // vmax, default 1.5
        actuator.fpmax,     // fpmax, default 1.3
        actuator.fvmax,     // fvmax, default 1.2
    ],
    _ => {
        let mut prm = [0.0; 9];
        prm[0] = actuator.gear;  // placeholder; not used by Step 9's non-muscle path
        prm
    },
};
self.actuator_gainprm.push(gainprm);
// Muscle: biasprm = gainprm (shared layout, MuJoCo convention). The runtime
// muscle path in Step 9 reads gainprm only; biasprm is stored for MuJoCo
// memory layout compatibility and for #12 (general gain/bias system).
// For non-muscle actuators: placeholder, will be populated by #12.
self.actuator_biasprm.push(gainprm);

// Lengthrange and acc0: initialized to zero, computed in Step 5 post-processing
self.actuator_lengthrange.push((0.0, 0.0));
self.actuator_acc0.push(0.0);
```

##### Step 5: Compute `actuator_lengthrange`, `acc0`, and `F0` at Build Time

Add a single public method `compute_muscle_params(&mut self)` on `Model` in
`mujoco_pipeline.rs`, following the pattern of `compute_ancestors()` and
`compute_implicit_params()`. This method handles all muscle-derived parameters:
`actuator_lengthrange`, `actuator_acc0`, and `F0` resolution.

Putting everything in one Model method (rather than a separate free function in
`model_builder.rs`) ensures that manually-constructed Models (e.g., in tests) get
correct muscle parameters by calling a single method.

`acc0 = ||M^{-1} * moment||` — the L2 norm of the joint-space acceleration vector
produced by unit actuator force through the transmission. `moment` is the
gear-scaled transmission Jacobian. MuJoCo computes this via `mj_solveM` +
`mju_norm` in `engine_setconst.c:set0()`.

```rust
/// Compute muscle-derived parameters: lengthrange, acc0, and F0.
///
/// For each muscle actuator:
///   1. Computes `actuator_lengthrange` from tendon/joint limits (gear-scaled).
///   2. Runs a forward pass at `qpos0` to get M, then computes
///      `acc0 = ||M^{-1} * moment||` via sparse solve.
///   3. Resolves `F0` (gainprm[2]) when `force < 0`: `F0 = scale / acc0`.
///
/// Must be called after `compute_ancestors()` and `compute_implicit_params()`,
/// and after all tendon/actuator fields are populated.
pub fn compute_muscle_params(&mut self) {
    if self.nu == 0 {
        return;
    }

    let has_muscles = (0..self.nu)
        .any(|i| self.actuator_dyntype[i] == ActuatorDynamics::Muscle);
    if !has_muscles {
        return;
    }

    // --- Phase 1: Compute actuator_lengthrange from limits ---
    for i in 0..self.nu {
        if self.actuator_dyntype[i] != ActuatorDynamics::Muscle {
            continue;
        }

        let gear = self.actuator_gear[i];

        // actuator_length = gear * transmission_length (Step 6),
        // so lengthrange = gear * transmission_lengthrange.
        // If gear < 0, min/max swap.
        let scale_range = |lo: f64, hi: f64| -> (f64, f64) {
            let a = gear * lo;
            let b = gear * hi;
            (a.min(b), a.max(b))
        };

        match self.actuator_trntype[i] {
            ActuatorTransmission::Tendon => {
                let tid = self.actuator_trnid[i];
                if tid < self.ntendon {
                    if self.tendon_limited[tid] {
                        let (lo, hi) = self.tendon_range[tid];
                        self.actuator_lengthrange[i] = scale_range(lo, hi);
                    } else {
                        // For unlimited tendons: estimate from joint ranges.
                        // Fixed tendon length = Σ coef_i * q_i, so extremes come
                        // from each joint at its range limit (sign-aware).
                        // Note: jnt_range defaults to (-π, π) for unlimited joints.
                        let adr = self.tendon_adr[tid];
                        let num = self.tendon_num[tid];
                        let (mut lmin, mut lmax) = (0.0, 0.0);
                        for w in adr..(adr + num) {
                            let dof = self.wrap_objid[w];
                            let coef = self.wrap_prm[w];
                            if let Some(jid) = (0..self.njnt).find(|&j| {
                                self.jnt_dof_adr[j] == dof
                            }) {
                                let (qlo, qhi) = self.jnt_range[jid];
                                if coef >= 0.0 {
                                    lmin += coef * qlo;
                                    lmax += coef * qhi;
                                } else {
                                    lmin += coef * qhi;
                                    lmax += coef * qlo;
                                }
                            }
                        }
                        self.actuator_lengthrange[i] = scale_range(lmin, lmax);
                    }
                }
            }
            ActuatorTransmission::Joint => {
                let jid = self.actuator_trnid[i];
                if jid < self.njnt {
                    let (lo, hi) = self.jnt_range[jid];
                    self.actuator_lengthrange[i] = scale_range(lo, hi);
                }
            }
            _ => {}
        }
    }

    // **Known limitation:** Unlimited joints default to jnt_range = (-π, π).
    // For hinge joints this is physically reasonable, but for slide (prismatic)
    // joints it produces an arbitrary 2π-meter range. MuJoCo uses a bisection
    // solver (mj_setLengthRange) to compute lengthrange from actual kinematics,
    // which handles unlimited slide joints correctly. Adding bisection-based
    // lengthrange is a possible follow-up but is not needed for the common case
    // of muscles on limited hinge joints.

    // --- Phase 2: Forward pass at qpos0 for acc0 ---
    // FK populates cinert (body spatial inertias), CRBA builds M and factors
    // it (L^T D L). Tendon evaluation is not needed because we construct the
    // moment vector directly from wrap_objid/wrap_prm (constant for fixed tendons).
    let mut data = self.make_data();  // qpos = qpos0
    mj_fwd_position(self, &mut data); // FK: cinert from qpos0
    mj_crba(self, &mut data);         // Mass matrix M + sparse factorization

    for i in 0..self.nu {
        if self.actuator_dyntype[i] != ActuatorDynamics::Muscle {
            continue;
        }

        // Build transmission moment J (maps unit actuator force → generalized forces).
        // MuJoCo's moment = gear * raw_Jacobian. For joint transmission, raw J is a
        // unit vector at the joint DOF. For tendon transmission, raw J is the tendon
        // coupling vector. Gear scales both.
        let gear = self.actuator_gear[i];
        let mut j_vec = DVector::zeros(self.nv);
        match self.actuator_trntype[i] {
            ActuatorTransmission::Joint => {
                let jid = self.actuator_trnid[i];
                if jid < self.njnt {
                    let dof_adr = self.jnt_dof_adr[jid];
                    j_vec[dof_adr] = gear;
                }
            }
            ActuatorTransmission::Tendon => {
                let tid = self.actuator_trnid[i];
                if tid < self.ntendon {
                    let adr = self.tendon_adr[tid];
                    let num = self.tendon_num[tid];
                    for w in adr..(adr + num) {
                        let dof_adr = self.wrap_objid[w];
                        let coef = self.wrap_prm[w];
                        if dof_adr < self.nv {
                            j_vec[dof_adr] = gear * coef;
                        }
                    }
                }
            }
            ActuatorTransmission::Site => {}
        }

        // Solve M * x = J using the sparse L^T D L factorization from CRBA.
        // x = M^{-1} * J is the joint-space acceleration from unit actuator force.
        let mut x = j_vec.clone();
        mj_solve_sparse(&data.qLD_diag, &data.qLD_L, &mut x);

        // acc0 = ||M^{-1} J||_2 (L2 norm of acceleration vector).
        // Matches MuJoCo's mju_norm(tmp, nv) in engine_setconst.c.
        self.actuator_acc0[i] = x.norm().max(1e-10);

        // --- Phase 3: Resolve F0 ---
        if self.actuator_gainprm[i][2] < 0.0 {
            // force < 0 means auto-compute: F0 = scale / acc0
            self.actuator_gainprm[i][2] =
                self.actuator_gainprm[i][3] / self.actuator_acc0[i];
            // Sync biasprm (MuJoCo layout: muscles share gain/bias parameters).
            // The runtime muscle path reads gainprm only, but keeping biasprm in
            // sync ensures the Model data matches MuJoCo's memory layout for
            // introspection and for #12 (general gain/bias).
            self.actuator_biasprm[i][2] = self.actuator_gainprm[i][2];
        }
    }
}
```

Called from `model_builder.rs` after `compute_implicit_params()`:

```rust
model.compute_ancestors();
model.compute_implicit_params();
model.compute_muscle_params();  // NEW: lengthrange + acc0 + F0 resolution
```

Phase 2 uses the sparse `L^T D L` factorization from #2 (computed by `mj_crba()`),
so the solve is O(nv) per actuator for tree-structured robots.

##### Step 6: Compute `actuator_length` and `actuator_velocity`

Add a new function `mj_actuator_length()`, called from `forward()` (and
`forward_skip_sensors()`) after `mj_fwd_velocity()`. At this point both tendon
lengths (from `mj_fwd_tendon()` inside `mj_fwd_position()`) and tendon velocities
(from `mj_fwd_velocity()`) are available:

```rust
fn mj_actuator_length(model: &Model, data: &mut Data) {
    for i in 0..model.nu {
        let gear = model.actuator_gear[i];
        match model.actuator_trntype[i] {
            ActuatorTransmission::Joint => {
                let jid = model.actuator_trnid[i];
                if jid < model.njnt {
                    // Joint transmission only meaningful for Hinge/Slide (scalar qpos).
                    // Ball/Free joints have multi-DOF qpos (quaternion/pose) — these
                    // cannot be used as actuator length. MuJoCo enforces the same
                    // restriction. Skip gracefully for Ball/Free.
                    let nv = model.jnt_type[jid].nv();
                    if nv == 1 {
                        let qadr = model.jnt_qpos_adr[jid];
                        let dof_adr = model.jnt_dof_adr[jid];
                        data.actuator_length[i] = gear * data.qpos[qadr];
                        data.actuator_velocity[i] = gear * data.qvel[dof_adr];
                    }
                }
            }
            ActuatorTransmission::Tendon => {
                let tid = model.actuator_trnid[i];
                if tid < model.ntendon {
                    data.actuator_length[i] = gear * data.ten_length[tid];
                    data.actuator_velocity[i] = gear * data.ten_velocity[tid];
                }
            }
            ActuatorTransmission::Site => {
                // Not yet implemented
            }
        }
    }
}
```

MuJoCo's `mj_transmission()` multiplies all actuator lengths and velocities by the
gear ratio. The gear also scales the transmission moment (Jacobian), so that
`actuator_velocity = moment · qvel` where `moment = gear * raw_Jacobian`.

##### Step 7: Implement MuJoCo FLV Curves

Add three functions to `mujoco_pipeline.rs`, above `mj_fwd_actuation()`. These
implement MuJoCo's exact piecewise-quadratic muscle curves from
`engine_util_misc.c`:

**`muscle_gain_length(length: f64, lmin: f64, lmax: f64) -> f64`** (FL curve)

Active force-length: piecewise quadratic bump, 0 outside `[lmin, lmax]`, peak 1.0
at `L = 1.0`. Four segments joined at midpoints `a = 0.5*(lmin+1)` and
`b = 0.5*(1+lmax)`:

```rust
fn muscle_gain_length(length: f64, lmin: f64, lmax: f64) -> f64 {
    const EPS: f64 = 1e-10; // matches MuJoCo's mjMINVAL guards
    if length < lmin || length > lmax {
        return 0.0;
    }
    let a = 0.5 * (lmin + 1.0); // midpoint of [lmin, 1]
    let b = 0.5 * (1.0 + lmax); // midpoint of [1, lmax]

    if length <= a {
        let x = (length - lmin) / (a - lmin).max(EPS);
        0.5 * x * x
    } else if length <= 1.0 {
        let x = (1.0 - length) / (1.0 - a).max(EPS);
        1.0 - 0.5 * x * x
    } else if length <= b {
        let x = (length - 1.0) / (b - 1.0).max(EPS);
        1.0 - 0.5 * x * x
    } else {
        let x = (lmax - length) / (lmax - b).max(EPS);
        0.5 * x * x
    }
}
```

**`muscle_gain_velocity(velocity: f64, fvmax: f64) -> f64`** (FV curve)

Force-velocity: piecewise quadratic. `V` is normalized by `L0 * vmax` so
`V = -1` is max shortening. Let `y = fvmax - 1`:

```rust
fn muscle_gain_velocity(velocity: f64, fvmax: f64) -> f64 {
    const EPS: f64 = 1e-10;
    let y = fvmax - 1.0;
    if velocity <= -1.0 {
        0.0
    } else if velocity <= 0.0 {
        (velocity + 1.0) * (velocity + 1.0)
    } else if velocity <= y {
        fvmax - (y - velocity) * (y - velocity) / y.max(EPS)
    } else {
        fvmax
    }
}
```

**`muscle_passive_force(length: f64, lmax: f64, fpmax: f64) -> f64`** (FP curve)

Passive force: zero below `L = 1.0`, quadratic onset, linear beyond midpoint `b`:

```rust
fn muscle_passive_force(length: f64, lmax: f64, fpmax: f64) -> f64 {
    const EPS: f64 = 1e-10;
    let b = 0.5 * (1.0 + lmax);
    if length <= 1.0 {
        0.0
    } else if length <= b {
        let x = (length - 1.0) / (b - 1.0).max(EPS);
        fpmax * 0.5 * x * x
    } else {
        let x = (length - b) / (b - 1.0).max(EPS);
        fpmax * (0.5 + x)
    }
}
```

##### Step 8: Implement Activation Dynamics

Add `muscle_activation_dynamics()` to `mujoco_pipeline.rs`. This matches MuJoCo's
`mju_muscleDynamics()` from `engine_util_misc.c`:

```rust
/// Quintic smoothstep (C2-continuous Hermite), matching MuJoCo's mju_sigmoid.
fn sigmoid(x: f64) -> f64 {
    if x <= 0.0 { return 0.0; }
    if x >= 1.0 { return 1.0; }
    x * x * x * (6.0 * x * x - 15.0 * x + 10.0)
}

/// Compute d(act)/dt for muscle activation dynamics.
///
/// Follows Millard et al. (2013) with activation-dependent time constants:
///   tau_act_eff   = tau_act   * (0.5 + 1.5 * act)
///   tau_deact_eff = tau_deact / (0.5 + 1.5 * act)
///
/// When tausmooth > 0, uses quintic sigmoid to blend between tau_act and tau_deact
/// instead of a hard switch at ctrl == act.
fn muscle_activation_dynamics(
    ctrl: f64,
    act: f64,
    dynprm: &[f64; 3],
) -> f64 {
    let ctrl_clamped = ctrl.clamp(0.0, 1.0);
    let act_clamped = act.clamp(0.0, 1.0);

    // Activation-dependent effective time constants (Millard et al. 2013)
    let tau_act = dynprm[0] * (0.5 + 1.5 * act_clamped);
    let tau_deact = dynprm[1] / (0.5 + 1.5 * act_clamped);
    let tausmooth = dynprm[2];

    let dctrl = ctrl_clamped - act;

    // Select time constant
    let tau = if tausmooth < 1e-10 {
        // Hard switch
        if dctrl > 0.0 { tau_act } else { tau_deact }
    } else {
        // Smooth blending via quintic sigmoid
        tau_deact + (tau_act - tau_deact) * sigmoid(dctrl / tausmooth + 0.5)
    };

    dctrl / tau.max(1e-10)
}
```

##### Step 9: Refactor `mj_fwd_actuation()`

Replace the current `mj_fwd_actuation()` with a version that:
1. Computes activation derivatives (`data.act_dot`) without modifying `data.act`.
2. Computes actuator force using gain/bias and the **current** `data.act`
   (muscle FLV for muscles, `ctrl` for simple actuators).
3. Clamps control inputs and output forces to their declared ranges.
4. Maps actuator force to joint forces via the transmission.

**Critical design:** `mj_fwd_actuation` does NOT integrate `data.act`. It only
computes `data.act_dot`. Integration is done by the integrator (Euler or RK4).
This matches MuJoCo's `mjData.act_dot` architecture and is required for correct
RK4 integration — otherwise force would be computed with the wrong activation
(post-Euler rather than current), and RK4 would have to reverse-engineer
derivatives from clamped Euler steps.

```rust
fn mj_fwd_actuation(model: &Model, data: &mut Data) {
    data.qfrc_actuator.fill(0.0);

    for i in 0..model.nu {
        // --- Phase 1: Activation dynamics (compute act_dot, do NOT integrate) ---
        let ctrl = data.ctrl[i].clamp(
            model.actuator_ctrlrange[i].0,
            model.actuator_ctrlrange[i].1,
        );

        let input = match model.actuator_dyntype[i] {
            ActuatorDynamics::None => ctrl,
            ActuatorDynamics::Muscle => {
                let act_adr = model.actuator_act_adr[i];
                data.act_dot[act_adr] = muscle_activation_dynamics(
                    ctrl, data.act[act_adr], &model.actuator_dynprm[i],
                );
                data.act[act_adr]  // use CURRENT activation for force
            }
            ActuatorDynamics::Filter => {
                // First-order filter: d(act)/dt = (ctrl - act) / tau
                let act_adr = model.actuator_act_adr[i];
                let tau = model.actuator_dynprm[i][0].max(1e-10);
                data.act_dot[act_adr] = (ctrl - data.act[act_adr]) / tau;
                data.act[act_adr]  // use CURRENT activation for force
            }
            ActuatorDynamics::Integrator => {
                // Integrator: d(act)/dt = ctrl
                let act_adr = model.actuator_act_adr[i];
                data.act_dot[act_adr] = ctrl;
                data.act[act_adr]  // use CURRENT activation for force
            }
        };

        // --- Phase 2: Force generation (gain * input + bias) ---
        //
        // MuJoCo's actuator_force is the scalar output of gain/bias BEFORE the
        // transmission maps it to generalized forces. Gear enters via the transmission
        // moment (Phase 3), not via actuator_force.
        let force = match model.actuator_dyntype[i] {
            ActuatorDynamics::Muscle => {
                let prm = &model.actuator_gainprm[i];
                let len = data.actuator_length[i];
                let vel = data.actuator_velocity[i];
                let lengthrange = model.actuator_lengthrange[i];
                let f0 = prm[2]; // force (already resolved, always > 0 after Step 5)

                // Normalize length and velocity
                let l0 = (lengthrange.1 - lengthrange.0)
                    / (prm[1] - prm[0]).max(1e-10);
                let norm_len = prm[0] + (len - lengthrange.0) / l0.max(1e-10);
                let norm_vel = vel / (l0 * prm[6]).max(1e-10); // prm[6] = vmax

                // gain = -F0 * FL(L) * FV(V), bias = -F0 * FP(L)
                let fl = muscle_gain_length(norm_len, prm[4], prm[5]);
                let fv = muscle_gain_velocity(norm_vel, prm[8]);
                let fp = muscle_passive_force(norm_len, prm[5], prm[7]);

                let gain = -f0 * fl * fv;
                let bias = -f0 * fp;
                gain * input + bias  // = -F0 * (FL*FV*act + FP)
            }
            _ => {
                // Simple actuators: force = input (ctrl or filtered activation).
                // MuJoCo's full gain/bias system (gain_type, bias_type, gainprm,
                // biasprm) is not implemented for non-muscle actuators in this spec.
                // Position/Velocity servos will need gain/bias in a future step.
                input
            }
        };

        // Clamp to force range
        let force = force.clamp(
            model.actuator_forcerange[i].0,
            model.actuator_forcerange[i].1,
        );
        data.actuator_force[i] = force;

        // --- Phase 3: Transmission (actuator_force → generalized forces) ---
        //
        // qfrc_actuator += moment^T * actuator_force
        // where moment = gear * raw_Jacobian.
        //
        // For Joint:  moment[dof] = gear, so qfrc[dof] += gear * force.
        // For Tendon: moment[dof] = gear * coef, so qfrc[dof] += gear * coef * force.
        let gear = model.actuator_gear[i];
        let trnid = model.actuator_trnid[i];
        match model.actuator_trntype[i] {
            ActuatorTransmission::Joint => {
                if trnid < model.njnt {
                    let dof_adr = model.jnt_dof_adr[trnid];
                    let nv = model.jnt_type[trnid].nv();
                    if nv > 0 {
                        data.qfrc_actuator[dof_adr] += gear * force;
                    }
                }
            }
            ActuatorTransmission::Tendon => {
                let tendon_id = trnid;
                if tendon_id < model.ntendon {
                    let adr = model.tendon_adr[tendon_id];
                    let num = model.tendon_num[tendon_id];
                    for w in adr..(adr + num) {
                        let dof_adr = model.wrap_objid[w];
                        let coef = model.wrap_prm[w];
                        if dof_adr < model.nv {
                            data.qfrc_actuator[dof_adr] += gear * coef * force;
                        }
                    }
                }
            }
            ActuatorTransmission::Site => {}
        }
    }
}
```

**Key changes from current implementation:**
- Phase 1 computes `data.act_dot` based on `actuator_dyntype` (was ignored).
  Does NOT integrate `data.act` — integration is done by the integrator.
- Phase 2 computes force via gain/bias for muscles using **current** `data.act`
  (was `ctrl * gear` for all). For non-muscle actuators, `actuator_force = input`
  (the raw or filtered control).
- Phase 3 applies gear in the transmission: `qfrc += gear * force` (joint) or
  `qfrc += gear * coef * force` (tendon). This matches MuJoCo's moment-based
  architecture where `qfrc = moment^T * actuator_force` and `moment = gear * J_raw`.
- Control clamping to `actuator_ctrlrange` (was unclamped).
- Force clamping to `actuator_forcerange` (was unclamped).
- Stores per-actuator scalar force in `data.actuator_force` (new).
- Function no longer takes `dt` — derivative-only, no integration.

**Behavioral equivalence for `DynType::None` actuators:** The current code does
`qfrc[dof] += ctrl * gear` (joint) and `qfrc[dof] += coef * ctrl * gear` (tendon).
The new code does `actuator_force = ctrl`, then `qfrc[dof] += gear * ctrl` (joint)
and `qfrc[dof] += gear * coef * ctrl` (tendon). These are numerically identical.

**Behavioral change for Filter/Integrator actuators:** The current code ignores
`actuator_dyntype` and always applies `ctrl * gear`. The new code uses the
**current activation** (`data.act`) for force generation, with `act_dot` stored
for integration by the integrator. This means Filter/Integrator actuators will
now respond to their filtered/integrated state rather than raw ctrl — a
correctness fix, but a change from current behavior. On the first step (act=0),
a Filter actuator with `ctrl=1` will produce zero force until `act` ramps up.

**Known limitation — Position/Velocity servo force generation:** The non-muscle
force path uses `force = input` (raw ctrl or filtered activation), not MuJoCo's
general gain/bias formula (`kp * (act - length)` for Position servos, etc.).
The current code (`ctrl * gear`) is also wrong for these actuators, so this
spec does not regress them. Tracked as **#12 General Gain/Bias Actuator Force
Model**.

##### Step 10: Wire Into Pipeline

**10a. Forward pipeline:** Update `forward()` **and** `forward_skip_sensors()` to
call `mj_actuator_length()`. Both functions have the same pipeline structure and
must receive the same changes. (`forward_skip_sensors()` is used by RK4's
intermediate stages via `mj_runge_kutta()`.)

Note: `mj_fwd_actuation` no longer takes `dt` — it only computes derivatives.

```rust
fn forward(&mut self, model: &Model) {
    // ... existing stages ...
    mj_fwd_position(model, self);  // includes mj_fwd_tendon → ten_length, ten_J
    mj_collision(model, self);
    self.mj_sensor_pos(model);

    mj_fwd_velocity(model, self); // sets qvel, ten_velocity
    mj_actuator_length(model, self); // NEW: sets actuator_length, actuator_velocity
    self.mj_sensor_vel(model);

    mj_fwd_actuation(model, self); // CHANGED: signature (no dt), computes act_dot
    // ... rest unchanged ...
}

fn forward_skip_sensors(&mut self, model: &Model) {
    mj_fwd_position(model, self);
    mj_collision(model, self);

    mj_fwd_velocity(model, self);
    mj_actuator_length(model, self); // NEW (same as forward)

    mj_fwd_actuation(model, self); // CHANGED (same as forward)
    // ... rest unchanged ...
}
```

**10b. Euler/ImplicitSpringDamper integrator:** Add activation integration to
`Data::integrate()` (line 2417), **before** velocity integration (matching
MuJoCo's `mj_advance()` order: activation → velocity → position). This runs
for both Euler and ImplicitSpringDamper (both go through `integrate()`):

```rust
// Integrate activation: act += dt * act_dot, then clamp muscles to [0, 1]
for i in 0..model.nu {
    let act_adr = model.actuator_act_adr[i];
    let act_num = model.actuator_act_num[i];
    for k in 0..act_num {
        data.act[act_adr + k] += model.timestep * data.act_dot[act_adr + k];
    }
    // Clamp muscle activation to [0, 1]
    if model.actuator_dyntype[i] == ActuatorDynamics::Muscle {
        for k in 0..act_num {
            data.act[act_adr + k] = data.act[act_adr + k].clamp(0.0, 1.0);
        }
    }
}
```

**10c. RK4 integrator:** Update `mj_runge_kutta()` to integrate `data.act`
alongside `qpos`/`qvel`. Since `mj_fwd_actuation` now writes `data.act_dot`
without modifying `data.act`, RK4 can read `act_dot` directly at each stage —
no recovery-by-differencing needed.

The existing code already has a TODO placeholder at line 9774:
`// TODO(FUTURE_WORK#5): When activation dynamics are added (act_dot),
save/restore/integrate act alongside qpos/qvel at each stage.`

```rust
// In mj_runge_kutta(), alongside the existing qpos/qvel save:
data.rk4_act_saved.copy_from(&data.act);  // uses pre-allocated Data field

// Stage 0: collect act_dot from the initial forward() pass.
data.rk4_act_dot[0].copy_from(&data.act_dot);

// In the stage loop (i = 1..4), before each forward_skip_sensors():
//   - set data.act = rk4_act_saved + h * Σ A[j] * rk4_act_dot[j]  (trial state)
//   - clamp muscle activations to [0, 1]
// After forward_skip_sensors():
//   - collect data.rk4_act_dot[i] from data.act_dot

// Final combination:
for a in 0..model.na {
    let dact_combined: f64 = (0..4)
        .map(|j| RK4_B[j] * data.rk4_act_dot[j][a])
        .sum();
    data.act[a] = data.rk4_act_saved[a] + h * dact_combined;
}
// Clamp muscle activations to [0, 1]
for i in 0..model.nu {
    if model.actuator_dyntype[i] == ActuatorDynamics::Muscle {
        let act_adr = model.actuator_act_adr[i];
        for k in 0..model.actuator_act_num[i] {
            data.act[act_adr + k] = data.act[act_adr + k].clamp(0.0, 1.0);
        }
    }
}
```

This integrates activation with the same RK4 weights as qpos/qvel, matching
MuJoCo's treatment of `act` as part of the state vector. The `na` field
(total activation dimension = `Σ actuator_act_num`) already exists on Model.
Since each trial stage reconstructs `data.act` from `rk4_act_saved` (the
initial save), and `mj_fwd_actuation` only writes `act_dot` (never `act`),
there is no need for additional per-stage save/restore of `data.act` — the
pattern mirrors how `rk4_qpos_saved` is used for position.

All RK4 activation buffers (`rk4_act_saved`, `rk4_act_dot`) are pre-allocated
Data fields (Step 3) to avoid per-step heap allocation on the hot path.

#### Acceptance Criteria

1. **Activation dynamics correctness:** For a muscle actuator with `ctrl = 1.0`
   starting from `act = 0.0`, after 100 steps at `dt = 0.001` with default
   `tau_act = 0.01`, activation reaches ≥ 0.95. With `ctrl = 0.0` starting from
   `act = 1.0` with default `tau_deact = 0.04`, activation falls to ≤ 0.10 after
   200 steps.

2. **Activation asymmetry:** Time to reach 90% activation from 0 (with `ctrl = 1`)
   is less than time to reach 10% activation from 1 (with `ctrl = 0`), matching
   `tau_act < tau_deact`.

3. **FL curve shape:** `muscle_gain_length(1.0, 0.5, 1.6) == 1.0` (peak at optimal).
   `muscle_gain_length(0.5, 0.5, 1.6) == 0.0` (zero at lmin).
   `muscle_gain_length(1.6, 0.5, 1.6) == 0.0` (zero at lmax).
   `muscle_gain_length(0.3, 0.5, 1.6) == 0.0` (zero outside range).

4. **FV curve shape:** `muscle_gain_velocity(0.0, 1.2) == 1.0` (isometric).
   `muscle_gain_velocity(-1.0, 1.2) == 0.0` (max shortening, zero force).
   `muscle_gain_velocity(0.2, 1.2) == 1.2` (eccentric plateau).

5. **FP curve shape:** `muscle_passive_force(0.8, 1.6, 1.3) == 0.0` (no passive
   below optimal). `muscle_passive_force(1.3, 1.6, 1.3) == 0.5 * 1.3 * 1.0`
   (at midpoint b = 1.3, FP = fpmax * 0.5).

6. **End-to-end muscle force:** A muscle with `force = 100.0`, `act = 1.0`,
   at optimal length (`L = 1.0`) and isometric (`V = 0.0`) produces
   `actuator_force = -100.0` (pulling). The corresponding `qfrc_actuator` DOFs
   receive force via the tendon Jacobian transpose.

7. **act_num fix:** Muscle actuators contribute exactly 1 entry to `data.act`.
   `model.actuator_act_num[i] == 1` for all muscle actuators.

8. **Non-muscle `DynType::None` actuators unchanged:** A Motor actuator
   (`DynType::None`) with `ctrl = 0.5`, `gear = 2.0` still produces
   `qfrc_actuator = 1.0` at the target DOF. Filter and Integrator actuators
   now use `data.act` for force (not raw ctrl), and `act_dot` is correctly
   computed and integrated.

9. **Control clamping:** Setting `ctrl = 5.0` with `ctrlrange = (-1, 1)` and
   `ctrllimited = true` produces the same result as `ctrl = 1.0`. An actuator
   with `ctrllimited = false` passes ctrl through unclamped regardless of
   ctrlrange value.

10. **Force clamping:** A muscle producing `-500 N` with `forcerange = (-100, 0)`
    and `forcelimited = true` is clamped to `-100 N`. An actuator with
    `forcelimited = false` passes force through unclamped.

11. **Existing tests pass:** All pipeline tests (tendon, collision, integration,
    constraint, sensor) remain green.

12. **MJCF round-trip:** A `<muscle>` element with explicit `timeconst`, `range`,
    `force`, `lmin`, `lmax`, `vmax`, `fpmax`, `fvmax` attributes produces a Model
    with matching `actuator_dynprm` and `actuator_gainprm` values.

13. **`acc0` computation:** `acc0 = ||M^{-1} J||` where `J` is the gear-scaled
    transmission moment. For a single-DOF hinge with inertia `I` and joint
    transmission with `gear = g`: `acc0 = |g| / I`. For a tendon with coupling
    coefficient `c` and `gear = g` on a hinge with inertia `I`:
    `acc0 = |g * c| / I`. Verified against analytic values to ≤ 1e-10.

14. **`F0` auto-computation:** A muscle with `force = -1` and `scale = 200` on a
    joint with `acc0 = 0.5` produces `F0 = 200 / 0.5 = 400`. A muscle with
    explicit `force = 500` ignores `acc0` and has `F0 = 500`.

15. **RK4 activation integration:** A muscle with `ctrl = 1.0`, `act = 0.0`,
    integrated for 1 step with RK4 at `dt = 0.001` produces the same `act` value
    (to ≤ 1e-6) as a manually computed RK4 combination of 4 dact evaluations.
    Activation is NOT advanced 4× (i.e., the RK4 save/restore logic is correct).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify:
  - New Model fields: `actuator_dynprm`, `actuator_gainprm`, `actuator_biasprm`,
    `actuator_lengthrange`, `actuator_acc0`
  - New Data fields: `actuator_length`, `actuator_velocity`, `actuator_force`, `act_dot`,
    `rk4_act_saved`, `rk4_act_dot` (RK4 scratch buffers)
  - New functions: `muscle_gain_length()`, `muscle_gain_velocity()`,
    `muscle_passive_force()`, `muscle_activation_dynamics()`, `sigmoid()`,
    `mj_actuator_length()`
  - New method: `Model::compute_muscle_params()` (lengthrange + forward pass for `acc0` + F0)
  - Refactored: `mj_fwd_actuation()` (computes `act_dot` + gain/bias force + clamping; no integration)
  - Modified: `forward()` and `forward_skip_sensors()` (call `mj_actuator_length`)
  - Modified: `Data::integrate()` (Euler: integrate `act += dt * act_dot`, clamp muscles)
  - Modified: `mj_runge_kutta()` (integrate `data.act` with RK4 weights via `act_dot`)
  - Modified: `Model::empty()` (initialize new fields)
  - Modified: `Data::make_data()` (initialize new fields + RK4 scratch buffers)
  - Modified: `Data::reset()` (clear `act_dot`, `actuator_length/velocity/force`)
  - New test module: `muscle_tests` (~300 lines, covering AC #1–#15)
- `sim/L0/mjcf/src/model_builder.rs` — modify:
  - Fix `act_num` for Muscle: 2 → 1
  - Add builder fields: `actuator_dynprm`, `actuator_gainprm`, `actuator_biasprm`,
    `actuator_lengthrange`, `actuator_acc0`
  - Transfer muscle parameters from `MjcfActuator` to Model arrays
  - Call `model.compute_muscle_params()` after `compute_implicit_params()`
- `sim/L0/core/Cargo.toml` — **no change** (no sim-muscle dependency added)
- `sim/L0/mjcf/src/parser.rs` — **no change** (muscle parsing already complete)
- `sim/docs/MUJOCO_REFERENCE.md` — update Stage 3 (actuation) to document
  activation dynamics and muscle force generation

---

### 6. Sensor Completion
**Status:** ✅ Done | **Effort:** M | **Prerequisites:** #4 ✅, #5 ✅

#### Implementation Summary

All 8 steps have been implemented. MJCF `<sensor>` elements are now fully wired
into the pipeline via `model_builder.rs`. The MJCF parser recognizes all 32
`MjcfSensorType` variants (30 map to pipeline types, 2 unsupported are skipped
with a warning). `set_options()` propagates `magnetic`, `wind`, `density`, and
`viscosity` from `MjcfOption`. Magnetometer is evaluated in the correct
(Position) stage. ActuatorVel reads from the pre-computed
`data.actuator_velocity` field. Dead code (unreachable Touch arm in
`mj_sensor_pos()`) has been removed. 8 integration tests in
`sim/L0/tests/integration/mjcf_sensors.rs` verify the full MJCF→pipeline
round-trip.

**Remaining scope exclusions** (documented below in the specification):
- **JointLimitFrc / TendonLimitFrc** — requires constraint force decomposition
- **Site transmission** — blocked on spatial site infrastructure (6 stubs remain)
- **Frame sensor `objtype` attribute** — resolved by name priority (site→body→geom)
- **User sensor `dim` attribute** — parser does not capture; User gets 0 slots
- **Sensor `<default>` class resolution** — `DefaultResolver` not called (cross-cutting)
- **Multi-geom Touch bodies** — resolves to first geom only
- **Sensor `reftype`/`refid`** — fields exist but not wired

#### Pre-Implementation State (historical)

**Sensor evaluation functions** (all in `mujoco_pipeline.rs`) were fully implemented
for all 30 `MjSensorType` variants across four pipeline stages:

| Stage | Function | Sensors |
|-------|----------|---------|
| Position | `mj_sensor_pos()` | JointPos, BallQuat, TendonPos, ActuatorPos, FramePos, FrameQuat, FrameXAxis, FrameYAxis, FrameZAxis, SubtreeCom, Rangefinder, Magnetometer |
| Velocity | `mj_sensor_vel()` | JointVel, BallAngVel, TendonVel, ActuatorVel, Gyro, Velocimeter, FrameLinVel, FrameAngVel, SubtreeLinVel, SubtreeAngMom |
| Acceleration | `mj_sensor_acc()` | Accelerometer, Force, Torque, Touch, ActuatorFrc, FrameLinAcc, FrameAngAcc |
| Postprocess | `mj_sensor_postprocess()` | Cutoff clamping (positive-type for Touch/Rangefinder, symmetric for others) |

Each stage filters by `model.sensor_datatype[sensor_id]` with a `continue` at
the top of the loop. A sensor with the wrong datatype is silently skipped and
produces zeros.

**Gaps that were resolved:**

| Gap | Severity | Resolution |
|-----|----------|------------|
| **A. MJCF → pipeline sensor wiring** | HIGH | `process_sensors()` and `resolve_sensor_object()` in `model_builder.rs` populate all 13 sensor arrays from parsed `MjcfSensor` objects. `actuator_name_to_id` map added for actuator sensor resolution. |
| **B. MJCF parser missing 8 sensor types** | MEDIUM | 8 variants added to `MjcfSensorType`: Velocimeter, Magnetometer, Rangefinder, Subtreecom, Subtreelinvel, Subtreeangmom, Framelinacc, Frameangacc. Total: 32 variants. |
| **C. Pipeline missing 2 sensor types** | LOW | Deferred — JointLimitFrc/TendonLimitFrc skipped with `warn!`. |
| **D. ActuatorVel duplicate computation** | LOW | Simplified to read `data.actuator_velocity[act_id]`. |
| **E. Site transmission stubs** | LOW | Out of scope — 6 stubs remain. |
| **F. Magnetometer evaluation stage** | LOW | Moved from `mj_sensor_acc()` to `mj_sensor_pos()`. Datatype changed to `Position`. |
| **G. Dead Touch arm in mj_sensor_pos()** | LOW | Removed. |
| **H. `set_options()` missing physics fields** | MEDIUM | `magnetic`, `wind`, `density`, `viscosity` now copied from `MjcfOption`. |

#### Objective

Wire MJCF-parsed sensors into the pipeline model builder so that `<sensor>`
elements in MJCF files produce working sensors. Fix the MJCF parser to recognize
all sensor types that the pipeline supports. Fix `set_options()` to propagate
physics environment fields needed by sensors. Simplify the ActuatorVel sensor to
read from its pre-computed field. Move magnetometer to the correct evaluation
stage. Clean up dead code.

#### Specification

##### Step 1: Add Missing Sensor Types to MJCF Parser (Gap B)

Add 8 variants to `MjcfSensorType` in `sim/L0/mjcf/src/types.rs` (after `Gyro`,
line 2191):

```rust
/// Velocimeter (linear velocity, 3D).
Velocimeter,
/// Magnetometer (magnetic field, 3D).
Magnetometer,
/// Rangefinder (distance to nearest surface, 1D).
Rangefinder,
/// Subtree center of mass (3D).
Subtreecom,
/// Subtree linear velocity/momentum (3D).
Subtreelinvel,
/// Subtree angular momentum (3D).
Subtreeangmom,
/// Frame linear acceleration (3D).
Framelinacc,
/// Frame angular acceleration (3D).
Frameangacc,
```

Update `MjcfSensorType::from_str()` (line 2200) with 8 new arms:
`"velocimeter"`, `"magnetometer"`, `"rangefinder"`, `"subtreecom"`,
`"subtreelinvel"`, `"subtreeangmom"`, `"framelinacc"`, `"frameangacc"`.

Update `MjcfSensorType::as_str()` (line 2232) with the corresponding reverse
mappings.

Update `MjcfSensorType::dim()` (line 2264) to include the new variants:
- Dim 1: `Rangefinder` (distance scalar)
- Dim 3: `Velocimeter`, `Magnetometer`, `Subtreecom`, `Subtreelinvel`,
  `Subtreeangmom`, `Framelinacc`, `Frameangacc`

Without this update, the match in `dim()` would be non-exhaustive and the
crate would fail to compile.

##### Step 2: Wire `set_options()` Physics Environment Fields (Gap H)

**Rationale:** This must precede Step 5 (magnetometer move) because without it,
`model.magnetic` is zero for all MJCF-loaded models, making the magnetometer
integration test (Step 8) unable to verify nonzero output. It also fixes a
pre-existing bug where `wind`, `density`, and `viscosity` are silently dropped.

In `set_options()` (`model_builder.rs`, line 506), add after the existing field
copies (after the integrator conversion, ~line 524):

```rust
self.magnetic = option.magnetic;
self.wind = option.wind;
self.density = option.density;
self.viscosity = option.viscosity;
```

These 4 fields already exist on both `ModelBuilder` (lines 450-453) and
`MjcfOption` (lines 302-311 of `types.rs`). The builder initializes them to
zero; `MjcfOption` defaults are: `magnetic = (0, -0.5, 0)`, `wind = (0, 0, 0)`,
`density = 0.0`, `viscosity = 0.0`.

After this change, models loaded from MJCF will inherit MuJoCo's default
magnetic field `(0, -0.5, 0)` unless overridden by `<option magnetic="..."/>`.

##### Step 3: MJCF → Pipeline Sensor Wiring (Gap A)

This is the critical missing piece. Add `process_sensors()` to the model builder
(`model_builder.rs`). It converts parsed `MjcfSensor` objects into the 13
pipeline sensor arrays.

**Prerequisite: Add imports to `model_builder.rs`.** The following types are not
currently imported and must be added:

From `sim_core` (add to the existing `use sim_core::{...}` block at line 16):
- `MjSensorType`
- `MjSensorDataType`
- `MjObjectType`

From `crate::types` (add to the existing `use crate::types::{...}` block at
line 26):
- `MjcfSensor`
- `MjcfSensorType`

Note: `tracing::warn` is already imported (line 23). No additional dependency
changes needed — `tracing` is already in `sim-mjcf`'s `Cargo.toml` (line 18).

**3a. Add `actuator_name_to_id` map to `ModelBuilder`.**

Add alongside the existing name maps (lines 317-323):

```rust
actuator_name_to_id: HashMap<String, usize>,
```

Initialize to empty in `ModelBuilder::new()`.

Populate it in `process_actuator()` (after line 1157 where `act_id` is computed).
`MjcfActuator.name` is a non-optional `String` (line 1778 of `types.rs`):

```rust
if !actuator.name.is_empty() {
    self.actuator_name_to_id.insert(actuator.name.clone(), act_id);
}
```

**3b. Add builder fields for sensor accumulation.**

The builder needs temporary storage that mirrors the Model's sensor arrays.
Add to `ModelBuilder` (matching the existing pattern where builder fields
accumulate data before being moved into the final Model struct):

```rust
// Sensor accumulation fields
sensor_type: Vec<MjSensorType>,
sensor_datatype: Vec<MjSensorDataType>,
sensor_objtype: Vec<MjObjectType>,
sensor_objid: Vec<usize>,
sensor_reftype: Vec<MjObjectType>,
sensor_refid: Vec<usize>,
sensor_adr: Vec<usize>,
sensor_dim: Vec<usize>,
sensor_noise: Vec<f64>,
sensor_cutoff: Vec<f64>,
sensor_name: Vec<Option<String>>,
nsensor: usize,
nsensordata: usize,
```

Initialize all to empty/zero in `ModelBuilder::new()`.

**3c. Type mapping function** (`MjcfSensorType` → `MjSensorType`):

```rust
fn convert_sensor_type(mjcf: MjcfSensorType) -> Option<MjSensorType> {
    match mjcf {
        MjcfSensorType::Jointpos => Some(MjSensorType::JointPos),
        MjcfSensorType::Jointvel => Some(MjSensorType::JointVel),
        MjcfSensorType::Ballquat => Some(MjSensorType::BallQuat),
        MjcfSensorType::Ballangvel => Some(MjSensorType::BallAngVel),
        MjcfSensorType::Tendonpos => Some(MjSensorType::TendonPos),
        MjcfSensorType::Tendonvel => Some(MjSensorType::TendonVel),
        MjcfSensorType::Actuatorpos => Some(MjSensorType::ActuatorPos),
        MjcfSensorType::Actuatorvel => Some(MjSensorType::ActuatorVel),
        MjcfSensorType::Actuatorfrc => Some(MjSensorType::ActuatorFrc),
        MjcfSensorType::Framepos => Some(MjSensorType::FramePos),
        MjcfSensorType::Framequat => Some(MjSensorType::FrameQuat),
        MjcfSensorType::Framexaxis => Some(MjSensorType::FrameXAxis),
        MjcfSensorType::Frameyaxis => Some(MjSensorType::FrameYAxis),
        MjcfSensorType::Framezaxis => Some(MjSensorType::FrameZAxis),
        MjcfSensorType::Framelinvel => Some(MjSensorType::FrameLinVel),
        MjcfSensorType::Frameangvel => Some(MjSensorType::FrameAngVel),
        MjcfSensorType::Framelinacc => Some(MjSensorType::FrameLinAcc),
        MjcfSensorType::Frameangacc => Some(MjSensorType::FrameAngAcc),
        MjcfSensorType::Touch => Some(MjSensorType::Touch),
        MjcfSensorType::Force => Some(MjSensorType::Force),
        MjcfSensorType::Torque => Some(MjSensorType::Torque),
        MjcfSensorType::Accelerometer => Some(MjSensorType::Accelerometer),
        MjcfSensorType::Gyro => Some(MjSensorType::Gyro),
        MjcfSensorType::Velocimeter => Some(MjSensorType::Velocimeter),
        MjcfSensorType::Magnetometer => Some(MjSensorType::Magnetometer),
        MjcfSensorType::Rangefinder => Some(MjSensorType::Rangefinder),
        MjcfSensorType::Subtreecom => Some(MjSensorType::SubtreeCom),
        MjcfSensorType::Subtreelinvel => Some(MjSensorType::SubtreeLinVel),
        MjcfSensorType::Subtreeangmom => Some(MjSensorType::SubtreeAngMom),
        MjcfSensorType::User => Some(MjSensorType::User),
        // Not yet implemented in pipeline — skip with warning
        MjcfSensorType::Jointlimitfrc | MjcfSensorType::Tendonlimitfrc => None,
    }
}
```

**3d. Datatype assignment** (determines which evaluation stage processes the
sensor). This is critical — a sensor with the wrong datatype is silently skipped.
Verified against existing test code (`add_sensor()` calls, lines 11146-12305).

**Implementation ordering constraint:** This function assigns
`MjSensorDataType::Position` for Magnetometer, which is only correct after
Step 5 moves the Magnetometer match arm from `mj_sensor_acc()` to
`mj_sensor_pos()`. If Step 3 is implemented before Step 5, MJCF-loaded
magnetometer sensors will be silently skipped (datatype `Position` but
evaluation code still in `mj_sensor_acc()`). The recommended implementation
order (Steps 1 → 2 → 3 → 4 → 5 → 6 → 7 → 8) ensures Step 5 runs before
the integration tests in Step 8 validate magnetometer output. However, if
steps are implemented out of order, temporarily assigning
`MjSensorDataType::Acceleration` for Magnetometer and updating it in Step 5
is an acceptable interim approach.

```rust
fn sensor_datatype(t: MjSensorType) -> MjSensorDataType {
    match t {
        // Position stage (evaluated in mj_sensor_pos, after FK)
        MjSensorType::JointPos | MjSensorType::BallQuat
        | MjSensorType::TendonPos | MjSensorType::ActuatorPos
        | MjSensorType::FramePos | MjSensorType::FrameQuat
        | MjSensorType::FrameXAxis | MjSensorType::FrameYAxis
        | MjSensorType::FrameZAxis | MjSensorType::SubtreeCom
        | MjSensorType::Rangefinder
        | MjSensorType::Magnetometer  // moved from acc → pos (Step 5)
        => MjSensorDataType::Position,

        // Velocity stage (evaluated in mj_sensor_vel, after velocity FK)
        MjSensorType::JointVel | MjSensorType::BallAngVel
        | MjSensorType::TendonVel | MjSensorType::ActuatorVel
        | MjSensorType::Gyro | MjSensorType::Velocimeter
        | MjSensorType::FrameLinVel | MjSensorType::FrameAngVel
        | MjSensorType::SubtreeLinVel | MjSensorType::SubtreeAngMom
        => MjSensorDataType::Velocity,

        // Acceleration stage (evaluated in mj_sensor_acc, after constraint solver)
        MjSensorType::Accelerometer | MjSensorType::Force
        | MjSensorType::Torque | MjSensorType::Touch
        | MjSensorType::ActuatorFrc
        | MjSensorType::FrameLinAcc | MjSensorType::FrameAngAcc
        => MjSensorDataType::Acceleration,

        // User sensors default to acceleration (evaluated last)
        MjSensorType::User => MjSensorDataType::Acceleration,
    }
}
```

**Key difference from MuJoCo:** Touch is `Acceleration` in our pipeline (needs
`efc_lambda` from constraint solver), whereas MuJoCo evaluates it in the
position stage because MuJoCo's sensor pipeline runs once after all dynamics
stages complete. Our per-stage filter design requires Touch to be tagged
`Acceleration` so it sees solved contact forces. This is correct.

**3e. Object type and ID resolution.**

Each sensor type expects a specific `MjObjectType`. The MJCF `objname` string
is resolved to a numeric index via the builder's name→id maps:

```rust
/// Returns (MjObjectType, objid) for a given sensor type and MJCF objname.
fn resolve_sensor_object(
    &self,
    sensor_type: MjSensorType,
    objname: &Option<String>,
) -> Result<(MjObjectType, usize), ModelConversionError> {
    // User sensors have no object reference — handle before unwrapping objname
    if sensor_type == MjSensorType::User {
        return Ok((MjObjectType::None, 0));
    }

    let name = objname.as_ref().ok_or_else(|| {
        ModelConversionError { message: "sensor missing object name".into() }
    })?;

    match sensor_type {
        // === Joint sensors: objname is a joint name ===
        MjSensorType::JointPos | MjSensorType::JointVel
        | MjSensorType::BallQuat | MjSensorType::BallAngVel => {
            let id = *self.joint_name_to_id.get(name).ok_or_else(|| {
                ModelConversionError { message:
                    format!("sensor references unknown joint '{name}'") }
            })?;
            Ok((MjObjectType::Joint, id))
        }

        // === Tendon sensors: objname is a tendon name ===
        MjSensorType::TendonPos | MjSensorType::TendonVel => {
            let id = *self.tendon_name_to_id.get(name).ok_or_else(|| {
                ModelConversionError { message:
                    format!("sensor references unknown tendon '{name}'") }
            })?;
            Ok((MjObjectType::Tendon, id))
        }

        // === Actuator sensors: objname is an actuator name ===
        MjSensorType::ActuatorPos | MjSensorType::ActuatorVel
        | MjSensorType::ActuatorFrc => {
            let id = *self.actuator_name_to_id.get(name).ok_or_else(|| {
                ModelConversionError { message:
                    format!("sensor references unknown actuator '{name}'") }
            })?;
            Ok((MjObjectType::Actuator, id))
        }

        // === Site-attached sensors: objname is a site name ===
        // Accelerometer, Velocimeter, Gyro, Force, Torque, Magnetometer,
        // Rangefinder all attach to sites.
        MjSensorType::Accelerometer | MjSensorType::Velocimeter
        | MjSensorType::Gyro | MjSensorType::Force | MjSensorType::Torque
        | MjSensorType::Magnetometer | MjSensorType::Rangefinder => {
            let id = *self.site_name_to_id.get(name).ok_or_else(|| {
                ModelConversionError { message:
                    format!("sensor references unknown site '{name}'") }
            })?;
            Ok((MjObjectType::Site, id))
        }

        // === Touch sensor: MJCF uses site=, but pipeline expects geom ID ===
        // MuJoCo's <touch> takes a site name. MuJoCo resolves this to the
        // site's parent body and sums contact forces for all geoms on that
        // body.
        //
        // Our pipeline's Touch evaluation code (mj_sensor_acc, line 5751)
        // compares contact.geom1/geom2 against objid, treating it as a geom
        // index. This is a pre-existing simplification that only matches
        // contacts for a single geom rather than all geoms on a body.
        //
        // Resolution: site name → site_id → body_id → first geom on body.
        // This is correct for the common case (one geom per body). Multi-geom
        // bodies would need the evaluation code to iterate all geoms on the
        // body — deferred (see Scope Exclusion 6).
        //
        // If no geom is found on the body, store objtype=Geom with
        // id=usize::MAX as a sentinel (sensor will produce 0.0 since no
        // contact will match). This avoids a hard error for bodyless sites.
        //
        // Builder needs site_body: Vec<usize> and geom_body: Vec<usize> to
        // be populated before process_sensors() runs. These are already
        // populated during process_body() (worldbody geoms/sites and
        // recursive body processing).
        MjSensorType::Touch => {
            let site_id = *self.site_name_to_id.get(name).ok_or_else(|| {
                ModelConversionError { message:
                    format!("touch sensor references unknown site '{name}'") }
            })?;
            let body_id = *self.site_body.get(site_id).ok_or_else(|| {
                ModelConversionError { message:
                    format!("touch sensor: site_id {site_id} out of range for site_body") }
            })?;
            // Find first geom belonging to this body
            let geom_id = self.geom_body.iter().position(|&b| b == body_id)
                .unwrap_or(usize::MAX);
            Ok((MjObjectType::Geom, geom_id))
        }

        // === Frame sensors: objname resolved by MJCF attribute context ===
        // In MJCF, frame sensors use different attributes to specify their
        // target: objtype="site" + site="name", or objtype="body" + body="name",
        // etc. The parser stores the resolved objname from whichever attribute
        // was present (site, body, or objname) — see parse_sensor_attrs()
        // (parser.rs line 1851) which tries joint, site, body, tendon,
        // actuator, objname in order.
        //
        // Since the parser does not pass through WHICH attribute was used,
        // we resolve by trying name maps in order: site → body → geom.
        // This handles the common case (<framepos site="s1"/>) correctly.
        //
        // LIMITATION: If a site and body share the same name, the site wins.
        // Full objtype attribute parsing is deferred (see Scope Exclusion 4).
        MjSensorType::FramePos | MjSensorType::FrameQuat
        | MjSensorType::FrameXAxis | MjSensorType::FrameYAxis
        | MjSensorType::FrameZAxis | MjSensorType::FrameLinVel
        | MjSensorType::FrameAngVel | MjSensorType::FrameLinAcc
        | MjSensorType::FrameAngAcc => {
            if let Some(&id) = self.site_name_to_id.get(name) {
                Ok((MjObjectType::Site, id))
            } else if let Some(&id) = self.body_name_to_id.get(name) {
                Ok((MjObjectType::Body, id))
            } else if let Some(&id) = self.geom_name_to_id.get(name) {
                Ok((MjObjectType::Geom, id))
            } else {
                Err(ModelConversionError { message:
                    format!("frame sensor references unknown object '{name}'") })
            }
        }

        // === Subtree sensors: objname is a body name ===
        MjSensorType::SubtreeCom | MjSensorType::SubtreeLinVel
        | MjSensorType::SubtreeAngMom => {
            let id = *self.body_name_to_id.get(name).ok_or_else(|| {
                ModelConversionError { message:
                    format!("sensor references unknown body '{name}'") }
            })?;
            Ok((MjObjectType::Body, id))
        }

        // User is handled above (early return before objname unwrap)
        MjSensorType::User => unreachable!(),
    }
}
```

**3f. `process_sensors()` main loop.**

The `process_sensors()` function uses the pipeline-side `MjSensorType::dim()`
(not `MjcfSensorType::dim()`) to compute sensor dimensions for address
allocation. This is important for User sensors: `MjSensorType::User.dim()`
returns 0 (variable, must be set explicitly via a `dim` attribute that we
don't yet parse), while `MjcfSensorType::User.dim()` returns 1. Using the
pipeline-side dim ensures User sensors get 0 slots — consistent with the
pipeline's expectation that User sensor dimension is explicitly specified.
See Scope Exclusion 7 for the follow-up to parse the `dim` attribute.

```rust
fn process_sensors(
    &mut self,
    sensors: &[MjcfSensor],
) -> Result<(), ModelConversionError> {
    let mut adr = 0usize;

    for mjcf_sensor in sensors {
        let Some(sensor_type) = convert_sensor_type(mjcf_sensor.sensor_type) else {
            // Unsupported type (Jointlimitfrc, Tendonlimitfrc) — skip with log
            warn!(
                "Skipping unsupported sensor type '{:?}' (sensor '{}')",
                mjcf_sensor.sensor_type,
                mjcf_sensor.name,
            );
            continue;
        };
        let dim = sensor_type.dim();
        let datatype = sensor_datatype(sensor_type);
        let (objtype, objid) = self.resolve_sensor_object(
            sensor_type, &mjcf_sensor.objname
        )?;

        self.sensor_type.push(sensor_type);
        self.sensor_datatype.push(datatype);
        self.sensor_objtype.push(objtype);
        self.sensor_objid.push(objid);
        self.sensor_reftype.push(MjObjectType::None);
        self.sensor_refid.push(0);
        self.sensor_adr.push(adr);
        self.sensor_dim.push(dim);
        self.sensor_noise.push(mjcf_sensor.noise);
        self.sensor_cutoff.push(mjcf_sensor.cutoff);
        self.sensor_name.push(if mjcf_sensor.name.is_empty() {
            None
        } else {
            Some(mjcf_sensor.name.clone())
        });

        adr += dim;
    }

    self.nsensor = self.sensor_type.len();
    self.nsensordata = adr;
    Ok(())
}
```

**3g. Call site in `model_from_mjcf()`.**

Insert after equality constraints (line 134) and before `builder.build()` (line 137):

```rust
// 7.5. Process sensors (after tendons, actuators, equality constraints)
builder.process_sensors(&mjcf.sensors)?;
```

**3h. Transfer builder fields to Model in `build()`.**

In the `build()` method, replace the empty sensor initialization (lines 1733-1746)
with the accumulated builder fields:

```rust
nsensor: self.nsensor,
nsensordata: self.nsensordata,
sensor_type: self.sensor_type,
sensor_datatype: self.sensor_datatype,
sensor_objtype: self.sensor_objtype,
sensor_objid: self.sensor_objid,
sensor_reftype: self.sensor_reftype,
sensor_refid: self.sensor_refid,
sensor_adr: self.sensor_adr,
sensor_dim: self.sensor_dim,
sensor_noise: self.sensor_noise,
sensor_cutoff: self.sensor_cutoff,
sensor_name: self.sensor_name,
```

`make_data()` already allocates `sensordata: DVector::zeros(self.nsensordata)`
(line 1900), so no change needed there — it will now allocate the correct size
based on the populated `nsensordata`.

##### Step 4: Simplify ActuatorVel Sensor (Gap D)

**Pipeline ordering constraint:** In `forward()`, the call order is:

```
mj_sensor_pos()       ← position sensors (ActuatorPos evaluated here)
mj_actuator_length()  ← populates data.actuator_length / data.actuator_velocity
mj_sensor_vel()       ← velocity sensors (ActuatorVel evaluated here)
```

Because `mj_actuator_length()` runs AFTER `mj_sensor_pos()`, **ActuatorPos
cannot read `data.actuator_length`** — it would be zero. ActuatorPos must
continue computing inline (current behavior is correct). However, ActuatorVel
runs in the velocity stage, which is after `mj_actuator_length()`, so it CAN
read from the pre-computed field.

In `mj_sensor_vel()`, replace the ActuatorVel match arm (~lines 5600-5630):

```rust
MjSensorType::ActuatorVel => {
    let act_id = model.sensor_objid[sensor_id];
    if act_id < model.nu {
        sensor_write(&mut data.sensordata, adr, 0, data.actuator_velocity[act_id]);
    }
}
```

**ActuatorPos is left unchanged** — it continues to compute inline per
transmission type. This is functionally correct and necessary due to pipeline
ordering. The Site transmission stub in the ActuatorPos arm (line 5409) remains
and will continue to produce 0.0 until Site transmission is implemented.

##### Step 5: Move Magnetometer to Position Stage (Gap F)

**Dependency:** Step 2 must be applied first so that `model.magnetic` is nonzero
for MJCF-loaded models. Without Step 2, the magnetometer code move is correct
but integration tests cannot verify nonzero output.

Move the `MjSensorType::Magnetometer` match arm from `mj_sensor_acc()` to
`mj_sensor_pos()`. The magnetometer only depends on `model.magnetic` (constant)
and `site_xmat` (computed during FK in `mj_fwd_position()`). MuJoCo evaluates
it in `mj_sensorPos`.

In `mj_sensor_pos()`, add after the Rangefinder arm. The code must be moved
verbatim from `mj_sensor_acc()` — it already dispatches on `sensor_objtype`
to support both Site and Body objects with an identity-matrix fallback:

```rust
MjSensorType::Magnetometer => {
    // Magnetometer: measures the global magnetic field in the sensor's
    // local frame. B_sensor = R^T * B_world
    let site_mat = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => data.site_xmat[objid],
        MjObjectType::Body if objid < model.nbody => data.xmat[objid],
        _ => Matrix3::identity(),
    };
    let b_sensor = site_mat.transpose() * model.magnetic;
    sensor_write3(&mut data.sensordata, adr, &b_sensor);
}
```

Remove the Magnetometer arm from `mj_sensor_acc()`.

**Important:** This is a verbatim move — do not simplify the code. The existing
implementation correctly handles both Site and Body object types and has an
identity-matrix fallback for unrecognized types.

The datatype mapping in Step 3d already assigns `MjSensorDataType::Position`
for Magnetometer. Two existing tests construct Models programmatically with
`MjSensorDataType::Acceleration` for Magnetometer and must be updated to use
`MjSensorDataType::Position`:
- `test_magnetometer_identity_frame` (line 11200 of `mujoco_pipeline.rs`)
- `test_magnetometer_zero_field` (line 11230 of `mujoco_pipeline.rs`)

##### Step 6: Remove Dead Touch Arm from mj_sensor_pos() (Gap G)

Remove the unreachable `MjSensorType::Touch` match arm at line 5277 of
`mj_sensor_pos()`. Touch has datatype `Acceleration`, so the `continue` filter
at line 5164 skips it. The 0.0 write never executes. Removing it eliminates
confusion about Touch's evaluation stage.

##### Step 7: Verify `forward()` Pipeline Order

No changes needed to `forward()` or `forward_skip_sensors()`. Confirm that the
existing call order supports the sensor changes:

```
forward():
  mj_fwd_position()    ← FK: body/geom/site poses, tendon kinematics
  mj_collision()       ← contact detection
  mj_sensor_pos()      ← position sensors (+ magnetometer after Step 5)
  mj_energy_pos()
  mj_fwd_velocity()    ← velocity kinematics
  mj_actuator_length() ← actuator_length/velocity (read by ActuatorVel in Step 4)
  mj_sensor_vel()      ← velocity sensors
  mj_fwd_actuation()   ← actuator forces
  mj_crba()            ← mass matrix
  mj_rne()             ← bias forces
  mj_energy_vel()
  mj_fwd_passive()     ← spring/damper forces
  mj_fwd_constraint()  ← contact solver → efc_lambda
  mj_fwd_acceleration()← final qacc
  mj_sensor_acc()      ← acceleration sensors (Touch reads efc_lambda here)
  mj_sensor_postprocess()
```

Key ordering constraints verified:
- `mj_actuator_length()` runs AFTER `mj_sensor_pos()` but BEFORE `mj_sensor_vel()`
  → ActuatorPos must compute inline (cannot read `data.actuator_length`).
  → ActuatorVel can safely read `data.actuator_velocity` (Step 4).
- `mj_fwd_constraint()` runs before `mj_sensor_acc()` → Touch reads solved
  `efc_lambda`.
- `site_xmat` available before `mj_sensor_pos()` → Magnetometer reads site
  rotation matrix.

Note: `forward_skip_sensors()` (line 2632) is a private method used internally
by the RK4 integrator (line 10268). It skips all 4 sensor stages. No changes
needed.

##### Step 8: Integration Test (`mjcf_sensors.rs`)

Create `sim/L0/tests/integration/mjcf_sensors.rs` and register it in
`sim/L0/tests/integration/mod.rs` as `pub mod mjcf_sensors;`.

Note: `sensors.rs` already exists in `sim/L0/tests/integration/` for the
standalone `sim-sensor` crate tests. This new file tests the MuJoCo pipeline's
MJCF→sensor wiring specifically.

The test should exercise the full MJCF→pipeline round-trip for at least
one sensor from each category (position, velocity, acceleration). Minimal
example for `jointpos`:

```rust
use sim_core::MjSensorType;
use sim_mjcf::load_model;

#[test]
fn test_jointpos_sensor_roundtrip() {
    let mjcf = r#"
        <mujoco model="sensor_test">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="hinge1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos joint="hinge1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_eq!(model.nsensor, 1);
    assert_eq!(model.nsensordata, 1);
    assert_eq!(model.sensor_type[0], MjSensorType::JointPos);

    let mut data = model.make_data();
    data.qpos[model.jnt_qpos_adr[0]] = 0.5; // set joint angle
    data.forward(&model).expect("forward");
    assert!((data.sensordata[0] - 0.5).abs() < 1e-12);
}
```

Additional tests to include (each as a separate `#[test]` function):
- `test_framepos_sensor_site` — `<framepos site="s1"/>`, verify 3D position matches `data.site_xpos[id]`
- `test_magnetometer_sensor_roundtrip` — verify nonzero output using MuJoCo's
  default magnetic field `(0, -0.5, 0)` (requires Step 2 for `set_options()`
  fix). Use an identity-rotation site and verify `sensordata` matches the
  global field rotated into the site frame.
- `test_tendonpos_sensor` — requires `<tendon>` + `<spatial>`, verify `sensordata` matches `data.ten_length[id]`
- `test_actuatorpos_sensor` — requires `<actuator>` + `<general>`, verify inline computation works
- `test_unsupported_sensor_skipped` — `<jointlimitfrc joint="j1"/>` produces `nsensor == 0` with warning
- `test_missing_reference_error` — `<jointpos joint="nonexistent"/>` returns `Err`

#### Scope Exclusions

The following are explicitly **out of scope** for this item:

1. **Site transmission** (Gap E) — Implementing `ActuatorTransmission::Site`
   requires computing site-to-site distance and its time derivative. This is
   blocked on spatial site infrastructure and is better addressed as part of
   spatial tendon support. The 6 stub locations will continue to produce 0.0.
   After Step 4, ActuatorVel sensors will correctly report 0.0 for
   Site-transmission actuators (matching `mj_actuator_length()`'s stub output).
   ActuatorPos retains its inline computation with its own Site stub (line 5409).

2. **JointLimitFrc / TendonLimitFrc** (Gap C) — Requires decomposing
   `qfrc_constraint` by constraint type. The constraint solver currently
   aggregates all constraint forces. Deferred.

3. **Sensor noise** — The pipeline has `model.sensor_noise` but intentionally
   does not apply it (deterministic physics for RL). This is documented and
   intentional (comment at line 5896), not a gap.

4. **Frame sensor `objtype` attribute parsing** — MuJoCo frame sensors support
   an `objtype=` attribute in MJCF (`"site"`, `"body"`, `"geom"`) to specify
   the target object type. Our parser does not read this attribute; Step 3e
   resolves frame sensor objname by trying site → body → geom in order. This
   is correct for the common case (`<framepos site="s1"/>`) but does not handle
   `<framepos body="b1"/>` if a site with the same name also exists. Full
   `objtype` parsing is a follow-up.

5. **Frame sensor `reftype`/`refid`** — MuJoCo frame sensors can measure
   quantities relative to a reference frame (another site/body). The
   `sensor_reftype`/`sensor_refid` fields exist but are not wired. This is a
   follow-up enhancement.

6. **Multi-geom Touch bodies** — The Touch sensor wiring (Step 3e) resolves
   to the first geom on the site's parent body. If a body has multiple geoms,
   only the first geom's contacts are summed. Full multi-geom support requires
   changing the evaluation code in `mj_sensor_acc()` to iterate all geoms on
   the body, which is a separate fix.

7. **User sensor `dim` attribute** — MuJoCo's `<user>` sensor element accepts
   a `dim=` attribute to specify the output dimension. The MJCF parser does
   not capture this field (`MjcfSensor` has no `dim` field).
   `MjSensorType::User.dim()` (pipeline-side, `mujoco_pipeline.rs` line 630)
   returns 0, meaning User sensors get no sensordata slots allocated. Note:
   `MjcfSensorType::User.dim()` (parser-side, `types.rs` line 2292) returns 1,
   but `process_sensors()` uses the pipeline-side dim (via
   `sensor_type.dim()`), so this mismatch does not cause incorrect allocation.
   Parsing the `dim` attribute and propagating it through `process_sensors()`
   is a follow-up.

8. **Sensor `<default>` class resolution** — `DefaultResolver.apply_to_sensor()`
   exists and can apply noise/cutoff/user defaults from `<default>` classes,
   but it is not called during model building (`model_from_mjcf()` does not
   invoke the `DefaultResolver` for any element type). `process_sensors()`
   reads `mjcf_sensor.noise` and `mjcf_sensor.cutoff` directly from the
   parsed values (0.0 if not specified in MJCF). This matches the existing
   pattern for all other element types. Wiring `DefaultResolver` into the
   model building pipeline is a cross-cutting concern, not sensor-specific.

#### Acceptance Criteria

1. **Physics environment fields wired:** `set_options()` copies `magnetic`,
   `wind`, `density`, and `viscosity` from `MjcfOption` to the builder. Models
   loaded from MJCF inherit MuJoCo's default magnetic field `(0, -0.5, 0)`.
2. **MJCF round-trip:** An MJCF file with `<sensor><jointpos joint="hinge1"/></sensor>`
   produces a `Model` with `nsensor == 1`, `sensor_type[0] == JointPos`,
   `sensor_objid[0]` pointing to the correct joint index, and `sensordata`
   allocated to the correct size (1 element).
3. **All 30 sensor types parseable:** Every `MjcfSensorType` variant (32 total
   after Step 1, minus 2 unsupported) maps to a pipeline `MjSensorType` and
   produces correct output after `forward()`.
4. **Datatype correctness:** Each wired sensor has the correct `MjSensorDataType`
   so it is evaluated in the right pipeline stage. Specifically: Touch is
   `Acceleration` (not Position), Magnetometer is `Position` (not Acceleration).
5. **Object type correctness:** `sensor_objtype` uses `MjObjectType` variants
   that match what the evaluation code dispatches on. Specifically: Touch is
   `MjObjectType::Geom` (not Site), Frame sensors are Site/Body/Geom based on
   resolution, Subtree sensors are `Body`.
6. **ActuatorVel reads from pre-computed field:** `ActuatorVel` returns
   `data.actuator_velocity[act_id]` rather than recomputing per-transmission.
   `ActuatorPos` continues to compute inline (pipeline ordering constraint:
   `mj_actuator_length()` runs after `mj_sensor_pos()`).
7. **Magnetometer in position stage:** Magnetometer output is available after
   `mj_sensor_pos()` completes. Existing magnetometer tests updated to use
   `MjSensorDataType::Position`.
8. **Dead code removed:** The unreachable Touch arm in `mj_sensor_pos()` is
   deleted.
9. **Existing tests pass:** All current sensor tests (constructed with hand-
   populated `Model` sensor arrays) continue to pass. The only required
   change is updating magnetometer tests to use `MjSensorDataType::Position`
   (Step 5); behavioral results are identical.
10. **New integration test** (`sim/L0/tests/integration/mjcf_sensors.rs`, Step 8):
    Uses `sim_mjcf::load_model()` to parse MJCF strings with sensors, calls
    `data.forward(&model)`, verifies `data.sensordata` contains expected values.
    Covers at minimum: `jointpos`, `framepos` (site target), `magnetometer`
    (nonzero output via default magnetic field), `tendonpos`, `actuatorpos`,
    unsupported type skipping, missing reference error.
11. **Unsupported types produce warning:** `Jointlimitfrc` and `Tendonlimitfrc`
    sensors in MJCF are skipped with a `warn!` log (already imported at
    `model_builder.rs` line 23), not silently ignored.
12. **Error on missing references:** A sensor referencing a nonexistent joint/
    site/body/tendon/actuator produces `ModelConversionError`, not a silent
    default.

#### Files

- `sim/L0/mjcf/src/types.rs` — modify: add 8 `MjcfSensorType` variants, update `from_str`/`as_str`/`dim()`
- `sim/L0/mjcf/src/model_builder.rs` — modify:
  - Add 4 lines to `set_options()` for `magnetic`, `wind`, `density`, `viscosity`
  - Add `actuator_name_to_id` HashMap to `ModelBuilder`
  - Add sensor accumulation fields to `ModelBuilder`
  - Add imports: `MjSensorType`, `MjSensorDataType`, `MjObjectType` from `sim_core`;
    `MjcfSensor`, `MjcfSensorType` from `crate::types`
  - Add `process_sensors()`, `convert_sensor_type()`, `sensor_datatype()`, `resolve_sensor_object()`
  - Update `build()` to transfer sensor fields to Model
  - Update `model_from_mjcf()` call order to include sensor processing
- `sim/L0/core/src/mujoco_pipeline.rs` — modify:
  - Simplify `ActuatorVel` match arm in `mj_sensor_vel()` to read from `data.actuator_velocity` (ActuatorPos stays inline due to pipeline ordering)
  - Move `Magnetometer` from `mj_sensor_acc()` to `mj_sensor_pos()`
  - Remove dead `Touch` arm from `mj_sensor_pos()`
  - Update magnetometer test(s) to use `MjSensorDataType::Position`
- `sim/L0/tests/integration/mjcf_sensors.rs` — new: MJCF sensor wiring integration test
  (Note: `sensors.rs` already exists in `sim/L0/tests/integration/` for standalone
  `sim-sensor` crate tests; this file tests the MuJoCo pipeline's MJCF→sensor
  wiring. Must be registered in `sim/L0/tests/integration/mod.rs` as
  `pub mod mjcf_sensors;`)
- `sim/docs/SENSOR_FIXES_SPEC.md` — reference for completed correctness fixes (no changes)

---
## Group C — Integrator & Dynamics

### 7. Integrator Rename (Implicit → ImplicitSpringDamper)
**Status:** ✅ Done | **Effort:** S | **Prerequisites:** None

#### Current State
The pipeline `Integrator` enum (`mujoco_pipeline.rs:671-679`) has three variants:
`Euler`, `RungeKutta4`, `ImplicitSpringDamper`. The doc comment reads "Implicit Euler
for diagonal per-DOF spring/damper forces" (`mujoco_pipeline.rs:677`). The actual
implementation in `mj_fwd_acceleration_implicit()` (`mujoco_pipeline.rs:9677`) solves:

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
   - `mujoco_pipeline.rs:2547` — integrator dispatch in `Data::step()`
   - `mujoco_pipeline.rs:2707` — velocity update skip in `Data::integrate()`
   - `mujoco_pipeline.rs:7312` — `implicit_mode` flag in `mj_fwd_passive()`
   - `mujoco_pipeline.rs:9476` — acceleration dispatch in `mj_fwd_acceleration()`
4. Update MJCF layer:
   - `types.rs` — rename `MjcfIntegrator::Implicit` variant
   - `model_builder.rs:560` — update `MjcfIntegrator::Implicit` → Sim conversion
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
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (enum definition, 4 match arms/equality checks)
- `sim/L0/mjcf/src/types.rs` — modify (`MjcfIntegrator` enum variant rename, `from_str`, `as_str`)
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

##### Activation State ✅ (Resolved by FUTURE_WORK #5)

MuJoCo's `mj_RungeKutta` includes actuator activation state (`act`, `act_dot`) in
its state and rate vectors. ~~Our `Data.act` exists but `act_dot` does not.~~
**Resolved:** `act_dot` is now computed by `mj_fwd_actuation()` and integrated by
both `integrate()` (Euler) and `mj_runge_kutta()` (RK4). RK4 carries `rk4_act_dot`
and `rk4_act_saved` buffers for proper 4-stage activation integration with the same
Butcher tableau weights as `qpos`/`qvel`.

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
## Group B (cont.) — Actuation

### 12. ~~General Gain/Bias Actuator Force Model~~ ✅ DONE
**Status:** Complete | **Effort:** M | **Prerequisites:** #5 ✅

#### Pre-Implementation State (historical)

> The description below documents the state **before** #12 was implemented.
> It is preserved for reference to explain the motivation for the changes.
> See the implementation (steps 1–7 above) for the current state.

`mj_fwd_actuation()` previously computed force for non-muscle actuators as
`force = input` — no gain/bias processing. Phase 2 dispatched on `dyntype`
instead of `gaintype`/`biastype`. No `GainType` or `BiasType` enums existed.
Position/Velocity actuators were unconditionally assigned `Filter` dynamics
(should be `None` unless `timeconst > 0`). `gainprm`/`biasprm` were set to
`[gear, 0, ..., 0]` for all non-muscle types (should be type-specific).
`kp`/`kv` were parsed from MJCF but discarded by the model builder.

#### Objective

Implement MuJoCo-compatible gain/bias force generation for all non-muscle
actuator types, replacing the `force = input` stub with the general formula:

```
force = gain * input + bias
```

where `gain` and `bias` are computed from `gaintype`/`biastype` and their
respective parameter arrays, and `input` is either `ctrl` (when
`dyntype = None`) or the current activation value `act` (for
Filter/FilterExact/Integrator).

#### MuJoCo Reference: Shortcut Expansion Table

Each MJCF shortcut actuator type expands to a `general` actuator with specific
gain/bias/dynamics settings. These expansions are authoritative — verified
against MuJoCo source (`src/user/user_api.cc`):

| Shortcut | `gaintype` | `gainprm[0..3]` | `biastype` | `biasprm[0..3]` | `dyntype` | `dynprm[0]` |
|----------|-----------|-----------------|-----------|-----------------|----------|------------|
| **Motor** | Fixed | `[1, 0, 0]` | None | `[0, 0, 0]` | None | — |
| **Position** | Fixed | `[kp, 0, 0]` | Affine | `[0, -kp, -kv]` | None (default) | — |
| **Position** (timeconst>0) | Fixed | `[kp, 0, 0]` | Affine | `[0, -kp, -kv]` | FilterExact | timeconst |
| **Velocity** | Fixed | `[kv, 0, 0]` | Affine | `[0, 0, -kv]` | None | — |
| **Damper** | Affine | `[0, 0, -kv]` | None | `[0, 0, 0]` | None | — |
| **Cylinder** | Fixed | `[area, 0, 0]` | Affine | `[bias0, bias1, bias2]` | Filter | timeconst |
| **Muscle** | Muscle | (FLV params) | Muscle | (FLV params) | Muscle | (tau_act, tau_deact) |
| **Adhesion** | Fixed | `[gain, 0, 0]` | None | `[0, 0, 0]` | None | — |

For Position: `kv` comes from either explicit `kv` attribute or is `0.0` by
default. The `dampratio` attribute (alternative to `kv`) is **deferred** — see
Scope Decision below. When both `kp` and `kv` are specified:
`force = kp * input + (0 - kp * length - kv * velocity) = kp * (input - length) - kv * velocity`.

For Damper: `gain = 0 + 0*length + (-kv)*velocity = -kv * velocity`, so
`force = (-kv * velocity) * ctrl`. Since `ctrl ∈ [0, ctrlrange_max]` (damper
requires `ctrllimited=true`, non-negative range), this produces a
velocity-opposing force scaled by control input.

#### Scope Decision: `dampratio` Deferred

MuJoCo position actuators support `dampratio` as an alternative to `kv`:
`kv = dampratio * 2 * sqrt(kp * inertia)`. This requires computing effective
inertia from `acc0` at model compile time. The `dampratio` attribute is not
parsed today, and `acc0` is only computed for muscle actuators (in
`compute_muscle_params()`).

**This PR does not implement `dampratio`.** Rationale:
- `dampratio` is not commonly specified in RL models (most use explicit `kv` or
  `kv = 0` for pure position servos).
- `acc0` computation for non-muscle actuators requires extending
  `compute_muscle_params()` into a general `compute_actuator_params()` function,
  which is orthogonal to the gain/bias force path.
- The gain/bias formula itself is complete without `dampratio` — it only
  affects how `biasprm[2]` is resolved in the builder. Adding `dampratio`
  support later is a builder-only change with no pipeline modifications.

When `dampratio` is added, the builder will:
1. Parse `dampratio` from MJCF (mutually exclusive with `kv`).
2. Compute reflected inertia for all actuators (extending
   `compute_muscle_params()` into a general `compute_actuator_params()`).
   MuJoCo computes this from `dof_M0` and the actuator transmission, not from
   `1/acc0` — the two are equivalent for simple transmissions but differ for
   compound transmissions.
3. Resolve `kv = dampratio * 2 * sqrt(kp * inertia)` (where `inertia` is the
   reflected inertia from step 2). Reference: MuJoCo `engine_setconst.c`.
4. Store `-kv` in `biasprm[2]`.

#### Scope Decision: `general` Actuator MJCF Parsing Deferred

MuJoCo's `<general>` actuator element accepts explicit `gaintype`, `biastype`,
`dyntype`, `gainprm`, `biasprm`, `dynprm` attributes. The parser currently
handles `<general>` by routing it through `MjcfActuatorType::General` with
the same attribute set as other shortcuts.

**This PR does not add `gaintype`/`biastype`/`dyntype` MJCF attribute parsing
for `<general>`.** Rationale:
- No RL models in common use (MuJoCo Menagerie, DM Control, Gymnasium) use
  `<general>` with explicit gain/bias types. They all use the shortcut types.
- The gain/bias *runtime* is fully general — any combination of Fixed/Affine/None
  works. Only the *builder wiring* from MJCF attributes is missing.
- Adding MJCF parsing for `gaintype`/`biastype` is a parser+builder change
  with no pipeline modifications. It can land independently.

For now, `<general>` actuators are treated as Motor-like (gaintype=Fixed,
gainprm=[1,0,0], biastype=None).

#### Specification

**Implementation ordering note:** Steps are numbered by logical grouping, not
strict code order. In the builder function, the `timeconst` and `kv` resolution
from Step 6 must execute *before* the dyntype match (Step 3) and the gain/bias
match (Step 4), since both depend on the resolved values. Concretely, place the
`let timeconst = ...` and `let kv = ...` lines at the top of the actuator
processing loop, before the `let dyntype = match ...` block.

##### Step 1: Add `FilterExact` Variant, `GainType` and `BiasType` Enums

Add `FilterExact` to the existing `ActuatorDynamics` enum
(`mujoco_pipeline.rs:426`):

```rust
pub enum ActuatorDynamics {
    /// No dynamics — input = ctrl (direct passthrough).
    #[default]
    None,
    /// First-order filter (Euler): act_dot = (ctrl - act) / tau.
    Filter,
    /// First-order filter (exact): act_dot = (ctrl - act) / tau,
    /// integrated as act += act_dot * tau * (1 - exp(-h/tau)).
    /// MuJoCo reference: `mjDYN_FILTEREXACT`.
    FilterExact,
    /// Integrator: act_dot = ctrl.
    Integrator,
    /// Muscle activation dynamics.
    Muscle,
}
```

The `FilterExact` variant computes `act_dot` identically to `Filter` in Phase 1.
The difference is only at integration time — `FilterExact` uses the closed-form
solution `act += act_dot * tau * (1 - exp(-h/tau))` instead of Euler
`act += h * act_dot`. This is MuJoCo's `mjDYN_FILTEREXACT`, used by Position
actuators with `timeconst > 0`.

Then add `GainType` and `BiasType` enums directly after `ActuatorDynamics`:

```rust
/// Actuator gain type — controls how `gain` is computed in Phase 2.
///
/// MuJoCo reference: `mjtGain` enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum GainType {
    /// gain = gainprm[0] (constant).
    #[default]
    Fixed,
    /// gain = gainprm[0] + gainprm[1]*length + gainprm[2]*velocity.
    Affine,
    /// Muscle FLV gain (handled separately in the Muscle path).
    Muscle,
}

/// Actuator bias type — controls how `bias` is computed in Phase 2.
///
/// MuJoCo reference: `mjtBias` enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum BiasType {
    /// bias = 0.
    #[default]
    None,
    /// bias = biasprm[0] + biasprm[1]*length + biasprm[2]*velocity.
    Affine,
    /// Muscle passive force (handled separately in the Muscle path).
    Muscle,
}
```

##### Step 2: Add `actuator_gaintype` and `actuator_biastype` to Model

Add to the Model struct, after `actuator_act_num` (line 921) and before
`actuator_dynprm` (line 926):

```rust
/// Gain type per actuator — dispatches force gain computation.
pub actuator_gaintype: Vec<GainType>,
/// Bias type per actuator — dispatches force bias computation.
pub actuator_biastype: Vec<BiasType>,
```

Initialize to empty vecs in `Model::empty()` (alongside other actuator
Vec inits).

##### Step 3: Fix Dynamics Type Assignment in Model Builder

Replace the dynamics type match in `model_builder.rs` (lines 1209-1217):

```rust
// Current (WRONG):
let dyntype = match actuator.actuator_type {
    MjcfActuatorType::Motor
    | MjcfActuatorType::General
    | MjcfActuatorType::Damper
    | MjcfActuatorType::Adhesion => ActuatorDynamics::None,
    MjcfActuatorType::Position | MjcfActuatorType::Velocity => ActuatorDynamics::Filter,
    MjcfActuatorType::Muscle => ActuatorDynamics::Muscle,
    MjcfActuatorType::Cylinder => ActuatorDynamics::Integrator,
};
```

With:

```rust
// MuJoCo semantics: dyntype is determined by the shortcut type AND timeconst.
// Position defaults to None; if timeconst > 0, uses FilterExact (exact discrete filter).
// Cylinder always uses Filter (Euler-approximated; timeconst defaults to 1.0).
// Motor/General/Damper/Adhesion/Velocity: no dynamics.
// Muscle: muscle activation dynamics.
let dyntype = match actuator.actuator_type {
    MjcfActuatorType::Motor
    | MjcfActuatorType::General
    | MjcfActuatorType::Damper
    | MjcfActuatorType::Adhesion
    | MjcfActuatorType::Velocity => ActuatorDynamics::None,
    MjcfActuatorType::Position => {
        // timeconst already resolved from Option<f64> (default 0.0 for position)
        if timeconst > 0.0 {
            ActuatorDynamics::FilterExact // MuJoCo: mjDYN_FILTEREXACT
        } else {
            ActuatorDynamics::None
        }
    }
    MjcfActuatorType::Muscle => ActuatorDynamics::Muscle,
    MjcfActuatorType::Cylinder => ActuatorDynamics::Filter, // was Integrator — MuJoCo uses Filter
};
```

**Update `act_num` match** (`model_builder.rs:1248-1251`): The `FilterExact`
variant needs `act_num = 1` (it has an activation state, same as `Filter`).
Update the existing match:

```rust
let act_num = match dyntype {
    ActuatorDynamics::None => 0,
    ActuatorDynamics::Filter
    | ActuatorDynamics::FilterExact
    | ActuatorDynamics::Integrator
    | ActuatorDynamics::Muscle => 1,
};
```

**Update Phase 1 `input` match** (`mujoco_pipeline.rs:6500-6521`): This is an
exhaustive match over `ActuatorDynamics` that will fail to compile without a
`FilterExact` arm. See Step 5a for the arm to add.

**Behavioral changes:**

- Position and Velocity actuators with default `timeconst=0` will now have
  `dyntype=None`, `act_num=0` (no activation state), and `input = ctrl`
  (direct passthrough). Previously they had `dyntype=Filter`, `act_num=1`,
  and `input = act` (first-order filtered). This is the correct MuJoCo
  behavior — unfiltered position/velocity servos respond instantly. This
  changes `Model.na` (total activation states), which affects `Data.act`
  and `Data.act_dot` dimensions. Existing tests that construct
  Position/Velocity actuators without explicit `timeconst > 0` will see
  `na` decrease. This is correct — the old behavior was a bug.

- Cylinder actuators change from `ActuatorDynamics::Integrator` to
  `ActuatorDynamics::Filter`. MuJoCo uses `mjDYN_FILTER` for cylinders —
  the pressure is filtered (first-order lag from `ctrl`), not integrated.
  The old `Integrator` assignment was incorrect: a pure integrator would
  integrate `ctrl` as `act_dot = ctrl`, making `act` grow unboundedly,
  while a filter converges to `ctrl` with time constant `dynprm[0]`.

- Velocity actuators are unconditionally `dyntype=None`. The `timeconst`
  attribute is silently ignored for Velocity — if someone writes
  `<velocity joint="j1" timeconst="0.1"/>`, the parser stores it in
  `MjcfActuator.timeconst` but the builder never reads it for Velocity.
  This matches MuJoCo: `mjs_setToVelocity` does not accept a `timeconst`
  parameter and always sets `dyntype = mjDYN_NONE`.

**Note on `MjcfActuator.timeconst` default:** The `Default` impl for
`MjcfActuator` sets `timeconst: 1.0` (`types.rs:1867`). This default is
for Cylinder actuators (MuJoCo's cylinder default is `timeconst=1.0`). For
Position actuators, MuJoCo's default is `timeconst=0` (no filter). To avoid
the Position path inheriting the Cylinder default, the builder must check
whether `timeconst` was explicitly set. The simplest approach: change
`MjcfActuator.timeconst` to `Option<f64>` (defaulting to `None`), and resolve
the default per actuator type in the builder:

```rust
let timeconst = actuator.timeconst.unwrap_or(match actuator.actuator_type {
    MjcfActuatorType::Cylinder => 1.0,
    _ => 0.0,
});
```

This requires updating `types.rs` (field type), `parser.rs` (parse into
`Some(value)`), and the builder. The `MjcfActuator::cylinder()` constructor
should continue to produce `timeconst: Some(1.0)`.

**`FilterExact` vs `Filter`:** MuJoCo's Position actuator (with `timeconst > 0`)
uses `mjDYN_FILTEREXACT`, while Cylinder uses `mjDYN_FILTER`. Both compute
`act_dot = (ctrl - act) / tau` identically in Phase 1. The difference is at
integration time:

- **Filter** (Euler): `act += h * act_dot`
- **FilterExact** (exact): `act += act_dot * tau * (1 - exp(-h/tau))`

The exact formula is the closed-form solution of `d(act)/dt = (ctrl - act) / tau`
over timestep `h`. For `tau >> h` both converge, but FilterExact is more accurate
when `tau` is comparable to `h`. MuJoCo uses FilterExact specifically for Position
actuators. See Step 5a below for integration changes.

##### Step 4: Populate `gaintype`, `biastype`, `gainprm`, `biasprm`, `dynprm` per Shortcut Type

Replace the dynprm/gain/bias parameter block in `model_builder.rs` (lines 1257-1292)
with per-type expansion. This is the core of the shortcut-to-general mapping:

```rust
// Gain/Bias/Dynamics parameters — expand shortcut type to general actuator.
// Reference: MuJoCo src/user/user_api.cc (mjs_setToMotor, mjs_setToPosition, etc.)
let (gaintype, biastype, gainprm, biasprm, dynprm) = match actuator.actuator_type {
    MjcfActuatorType::Motor => (
        GainType::Fixed,
        BiasType::None,
        {
            let mut p = [0.0; 9];
            p[0] = 1.0; // unit gain
            p
        },
        [0.0; 9],
        [0.0; 3],
    ),

    MjcfActuatorType::Position => {
        let kp = actuator.kp; // default 1.0
        // kv resolved above from Option<f64> (default 0.0 for position)
        let tc = timeconst;   // resolved above (default 0.0 for position)
        let mut gp = [0.0; 9];
        gp[0] = kp;
        let mut bp = [0.0; 9];
        bp[1] = -kp;
        bp[2] = -kv;
        (
            GainType::Fixed,
            BiasType::Affine,
            gp,
            bp,
            [tc, 0.0, 0.0],
        )
    }

    MjcfActuatorType::Velocity => {
        // kv resolved above from Option<f64> (default 1.0 for velocity)
        let mut gp = [0.0; 9];
        gp[0] = kv;
        let mut bp = [0.0; 9];
        bp[2] = -kv;
        (
            GainType::Fixed,
            BiasType::Affine,
            gp,
            bp,
            [0.0; 3],
        )
    }

    MjcfActuatorType::Damper => {
        // kv resolved above from Option<f64> (default 0.0 for damper)
        let mut gp = [0.0; 9];
        gp[2] = -kv; // gain = -kv * velocity
        (
            GainType::Affine,
            BiasType::None,
            gp,
            [0.0; 9],
            [0.0; 3],
        )
    }

    MjcfActuatorType::Cylinder => {
        let area = if let Some(d) = actuator.diameter {
            std::f64::consts::PI / 4.0 * d * d
        } else {
            actuator.area
        };
        let tc = timeconst; // resolved above (default 1.0 for cylinder)
        let mut gp = [0.0; 9];
        gp[0] = area;
        let mut bp = [0.0; 9];
        bp[0] = actuator.bias[0];
        bp[1] = actuator.bias[1];
        bp[2] = actuator.bias[2];
        (
            GainType::Fixed,
            BiasType::Affine,
            gp,
            bp,
            [tc, 0.0, 0.0],
        )
    }

    MjcfActuatorType::Adhesion => {
        let mut gp = [0.0; 9];
        gp[0] = actuator.gain;
        (
            GainType::Fixed,
            BiasType::None,
            gp,
            [0.0; 9],
            [0.0; 3],
        )
    }

    MjcfActuatorType::Muscle => {
        // Muscle: unchanged from #5 implementation.
        let gp = [
            actuator.range.0, actuator.range.1, actuator.force,
            actuator.scale, actuator.lmin, actuator.lmax,
            actuator.vmax, actuator.fpmax, actuator.fvmax,
        ];
        (
            GainType::Muscle,
            BiasType::Muscle,
            gp,
            gp, // biasprm = gainprm (shared layout, MuJoCo convention)
            [actuator.muscle_timeconst.0, actuator.muscle_timeconst.1, 0.0],
        )
    }

    MjcfActuatorType::General => {
        // TODO(general-actuator): <general> without explicit gaintype/biastype
        // is treated as Motor-like. Full MJCF attribute parsing for gaintype,
        // biastype, dyntype, gainprm, biasprm, dynprm is deferred. When added,
        // emit a warning if <general> is encountered without these attributes,
        // since the Motor-like fallback may not match user intent.
        // See Scope Decision: `general` Actuator MJCF Parsing Deferred.
        let mut gp = [0.0; 9];
        gp[0] = 1.0;
        (
            GainType::Fixed,
            BiasType::None,
            gp,
            [0.0; 9],
            [0.0; 3],
        )
    }
};

self.actuator_gaintype.push(gaintype);
self.actuator_biastype.push(biastype);
self.actuator_gainprm.push(gainprm);
self.actuator_biasprm.push(biasprm);
self.actuator_dynprm.push(dynprm);
```

This replaces the existing `is_muscle` branch and the `[0.0; 3]` dynprm
fallback. The Muscle arm is equivalent to the existing #5 code; the new arms
populate the correct shortcut-specific parameters.

**Damper/Adhesion `ctrllimited` enforcement:** MuJoCo's damper and adhesion
shortcuts unconditionally set `ctrllimited = 1` (`mjs_setToDamper` and
`mjs_setToAdhesion` in `user_api.cc`). Add this before the ctrlrange gate
(`model_builder.rs:1224`):

```rust
// Damper and Adhesion actuators force ctrllimited
// (MuJoCo: mjs_setToDamper and mjs_setToAdhesion both set ctrllimited=1).
let ctrllimited = match actuator.actuator_type {
    MjcfActuatorType::Damper | MjcfActuatorType::Adhesion => true,
    _ => actuator.ctrllimited,
};
```

Then use `ctrllimited` (the local variable) instead of `actuator.ctrllimited` at
line 1226. Without this, a `<damper joint="j1" kv="5"/>` (no explicit
`ctrllimited="true"`) would get unbounded ctrlrange, allowing negative control
values that produce force in the direction of motion — the opposite of a damper.
Similarly, an `<adhesion>` without explicit `ctrllimited` would allow negative
control values, which is invalid for adhesion force scaling.

##### Step 5: Replace Phase 2 Force Dispatch in `mj_fwd_actuation()`

Replace the Phase 2 block (`mujoco_pipeline.rs:6523-6552`) with gain/bias
dispatch on `gaintype`/`biastype`:

```rust
// --- Phase 2: Force generation (gain * input + bias) ---
let length = data.actuator_length[i];
let velocity = data.actuator_velocity[i];

let gain = match model.actuator_gaintype[i] {
    GainType::Fixed => model.actuator_gainprm[i][0],
    GainType::Affine => {
        model.actuator_gainprm[i][0]
            + model.actuator_gainprm[i][1] * length
            + model.actuator_gainprm[i][2] * velocity
    }
    GainType::Muscle => {
        // Muscle gain = -F0 * FL(L) * FV(V)
        let prm = &model.actuator_gainprm[i];
        let lengthrange = model.actuator_lengthrange[i];
        let f0 = prm[2]; // resolved by compute_muscle_params()

        let l0 = (lengthrange.1 - lengthrange.0) / (prm[1] - prm[0]).max(1e-10);
        let norm_len = prm[0] + (length - lengthrange.0) / l0.max(1e-10);
        let norm_vel = velocity / (l0 * prm[6]).max(1e-10);

        let fl = muscle_gain_length(norm_len, prm[4], prm[5]);
        let fv = muscle_gain_velocity(norm_vel, prm[8]);
        -f0 * fl * fv
    }
};

let bias = match model.actuator_biastype[i] {
    BiasType::None => 0.0,
    BiasType::Affine => {
        model.actuator_biasprm[i][0]
            + model.actuator_biasprm[i][1] * length
            + model.actuator_biasprm[i][2] * velocity
    }
    BiasType::Muscle => {
        let prm = &model.actuator_gainprm[i]; // muscle uses gainprm for both
        let lengthrange = model.actuator_lengthrange[i];
        let f0 = prm[2];

        let l0 = (lengthrange.1 - lengthrange.0) / (prm[1] - prm[0]).max(1e-10);
        let norm_len = prm[0] + (length - lengthrange.0) / l0.max(1e-10);

        let fp = muscle_passive_force(norm_len, prm[5], prm[7]);
        -f0 * fp
    }
};

let force = gain * input + bias;
```

This replaces the existing `match model.actuator_dyntype[i]` dispatch with
`match model.actuator_gaintype[i]` + `match model.actuator_biastype[i]`.

**The muscle path is logically identical to the existing code** — same FLV
curves, same F0 handling — just restructured to use the gain/bias dispatch.
This eliminates the special-case `ActuatorDynamics::Muscle` match in Phase 2
and unifies all actuator types under one formula: `force = gain * input + bias`.

##### Step 5a: Add `FilterExact` to Phase 1 and Integration

**Phase 1 (`mj_fwd_actuation`, line 6500):** Add a `FilterExact` arm to the
`input` match. The `act_dot` computation is identical to `Filter`:

```rust
ActuatorDynamics::FilterExact => {
    // Same act_dot as Filter; exact integration happens in integrate().
    let act_adr = model.actuator_act_adr[i];
    let tau = model.actuator_dynprm[i][0].max(1e-10);
    data.act_dot[act_adr] = (ctrl - data.act[act_adr]) / tau;
    data.act[act_adr]
}
```

**Euler integration (`Data::integrate()`, line 2635):** Replace the uniform
`act += h * act_dot` with a per-actuator integration step:

```rust
for i in 0..model.nu {
    let act_adr = model.actuator_act_adr[i];
    let act_num = model.actuator_act_num[i];
    for k in 0..act_num {
        let j = act_adr + k;
        match model.actuator_dyntype[i] {
            ActuatorDynamics::FilterExact => {
                // Exact: act += act_dot * tau * (1 - exp(-h/tau))
                let tau = model.actuator_dynprm[i][0].max(1e-10);
                self.act[j] += self.act_dot[j] * tau * (1.0 - (-h / tau).exp());
            }
            _ => {
                // Euler: act += h * act_dot
                self.act[j] += h * self.act_dot[j];
            }
        }
    }
    // Clamp muscle activation to [0, 1]
    if model.actuator_dyntype[i] == ActuatorDynamics::Muscle {
        for k in 0..act_num {
            self.act[act_adr + k] = self.act[act_adr + k].clamp(0.0, 1.0);
        }
    }
}
```

**RK4 integration (`mj_runge_kutta`, line 10229):** Same change for the final
activation combination. Replace `act = saved + h * combined` with:

```rust
for a in 0..na {
    let dact_combined: f64 = (0..4).map(|j| RK4_B[j] * data.rk4_act_dot[j][a]).sum();
    // Find which actuator owns this activation slot
    // (actuator_act_adr[i] <= a < actuator_act_adr[i] + actuator_act_num[i])
    let actuator_idx = model.actuator_act_adr.iter()
        .enumerate()
        .rev()
        .find(|&(i, &adr)| adr <= a && a < adr + model.actuator_act_num[i])
        .map(|(i, _)| i)
        .unwrap();
    match model.actuator_dyntype[actuator_idx] {
        ActuatorDynamics::FilterExact => {
            let tau = model.actuator_dynprm[actuator_idx][0].max(1e-10);
            data.act[a] = data.rk4_act_saved[a]
                + dact_combined * tau * (1.0 - (-h / tau).exp());
        }
        _ => {
            data.act[a] = data.rk4_act_saved[a] + h * dact_combined;
        }
    }
}
```

The same per-actuator check applies to the RK4 trial states (line 10164). The
reverse lookup from activation index to actuator index can be precomputed as a
`Vec<usize>` of length `na` at model build time if the per-step lookup is a
performance concern. For now the linear scan is correct and `nu` is typically
small (<100).

##### Step 6: Fix `MjcfActuator` Per-Type Defaults

Several `MjcfActuator` fields have a single struct-level default that is wrong
for some actuator types. MuJoCo resolves defaults per actuator type at compile
time. To distinguish "attribute not set" from "attribute set to 0", change these
fields from `f64` to `Option<f64>`:

**`timeconst`** — in `MjcfActuator` struct (line 1814):
```rust
/// Activation dynamics time constant (s). `None` means use actuator-type default.
pub timeconst: Option<f64>,
```

In `Default` impl (line 1867): `timeconst: None,`
In `MjcfActuator::cylinder()`: `timeconst: Some(1.0),`
In `parser.rs` (line 1123): parse into `Some(value)`.

Builder resolution:
```rust
let timeconst = actuator.timeconst.unwrap_or(match actuator.actuator_type {
    MjcfActuatorType::Cylinder => 1.0,
    _ => 0.0,
});
```

**`kv`** — in `MjcfActuator` struct (line 1804):
```rust
/// Velocity gain. `None` means use actuator-type default.
pub kv: Option<f64>,
```

In `Default` impl (line 1863): `kv: None,`
In `MjcfActuator::velocity()`: `kv: Some(kv),`
In `parser.rs` (line 1110): parse into `Some(value)`.

Builder resolution:
```rust
let kv = actuator.kv.unwrap_or(match actuator.actuator_type {
    MjcfActuatorType::Velocity => 1.0,
    _ => 0.0, // Damper defaults to 0.0 (MuJoCo: mjs_setToDamper uses kv=0)
});
```

MuJoCo defaults: `kv = 1.0` for `<velocity>`, `kv = 0.0` for `<damper>`,
`<position>`, and all other types. A `<damper>` without explicit `kv` is
effectively a no-op in MuJoCo (zero gain); in practice all real MJCF damper
actuators specify `kv` explicitly. Without `Option`, a `<velocity joint="j1"/>`
without explicit `kv` would use `kv = 0.0` (producing zero force), which is
wrong.

**`kp`** remains `f64` with default `1.0` — this matches MuJoCo's default for
all actuator types that use `kp` (Position). No per-type resolution needed.

**Add `damper()` constructor** — `MjcfActuator` has constructors for motor,
position, velocity, cylinder, muscle, and adhesion, but not damper. Add one
to `types.rs` (after `cylinder()`, line 1931):

```rust
/// Create a damper actuator for a joint.
#[must_use]
pub fn damper(name: impl Into<String>, joint: impl Into<String>, kv: f64) -> Self {
    Self {
        name: name.into(),
        actuator_type: MjcfActuatorType::Damper,
        joint: Some(joint.into()),
        kv: Some(kv),
        ..Default::default()
    }
}
```

This enables clean test construction for damper acceptance criteria (#6, #13)
without MJCF XML boilerplate.

##### Step 7: Update `MUJOCO_REFERENCE.md` Phase 2 Documentation

Replace the non-muscle stub (line 182):
```python
    else:
        force = input                 # ctrl or filtered activation
```

With:
```python
    else:
        # General gain/bias formula
        if gaintype[i] == Fixed:
            gain = gainprm[i][0]
        elif gaintype[i] == Affine:
            gain = gainprm[i][0] + gainprm[i][1]*length[i] + gainprm[i][2]*velocity[i]

        if biastype[i] == None:
            bias = 0.0
        elif biastype[i] == Affine:
            bias = biasprm[i][0] + biasprm[i][1]*length[i] + biasprm[i][2]*velocity[i]

        force = gain * input + bias
```

#### Acceptance Criteria

1. **Motor actuator (regression):** `DynType::None`, `gaintype=Fixed`,
   `gainprm=[1,0,0]`, `biastype=None`. With `ctrl = 5.0`:
   `force = 1.0 * 5.0 + 0.0 = 5.0`. Identical to pre-change behavior.

2. **Position servo (P-only):** `kp = 100`, `kv = 0`, `timeconst = 0` (no
   filter). `dyntype=None`, `gaintype=Fixed`, `gainprm=[100,0,0]`,
   `biastype=Affine`, `biasprm=[0,-100,0]`. With `ctrl = 1.0`,
   `actuator_length = 0.5`:
   `force = 100 * 1.0 + (0 - 100*0.5 - 0) = 100 - 50 = 50`.
   Equivalently: `force = kp * (ctrl - length) = 100 * (1.0 - 0.5) = 50`.

3. **Position servo (PD):** `kp = 100`, `kv = 10`, `timeconst = 0`.
   With `ctrl = 1.0`, `actuator_length = 0.5`, `actuator_velocity = 0.2`:
   `force = 100 * 1.0 + (0 - 100*0.5 - 10*0.2) = 100 - 50 - 2 = 48`.
   Equivalently: `force = kp*(ctrl - length) - kv*velocity`.

4. **Position servo (filtered):** `kp = 100`, `kv = 0`, `timeconst = 0.1`.
   `dyntype=FilterExact`, `dynprm=[0.1,0,0]`, `act_num=1`. After several steps
   with `ctrl = 1.0`, activation converges toward `1.0`. Force at steady state:
   `force = kp * (act - length)`. Verify exact filter dynamics: after one step
   with `h = 0.002`, `act = 0 + (1-0)/0.1 * 0.1 * (1 - exp(-0.002/0.1))`
   = `1 * (1 - exp(-0.02))` ≈ `0.0198`. Compare to Euler: `0 + 0.002 * 1/0.1`
   = `0.02`. The exact result is slightly smaller.

5. **Velocity servo:** `kv = 10`, `dyntype=None`. With `ctrl = 1.0`,
   `actuator_velocity = 0.2`:
   `force = 10 * 1.0 + (0 + 0 - 10*0.2) = 10 - 2 = 8`.
   Equivalently: `force = kv * (ctrl - velocity) = 10 * (1.0 - 0.2) = 8`.

6. **Damper:** `kv = 5`, `gaintype=Affine`, `gainprm=[0,0,-5]`,
   `biastype=None`. With `ctrl = 0.8`, `actuator_velocity = 2.0`:
   `gain = 0 + 0 + (-5)*2.0 = -10`, `force = -10 * 0.8 + 0 = -8.0`.
   Force opposes velocity, scaled by control.

7. **Cylinder:** `area = 0.01`, `bias = [0, 0, -100]`, `timeconst = 1.0`,
   `dyntype=Filter`. `gainprm=[0.01,0,0]`, `biasprm=[0,0,-100]`.
   With `act = 500000` (pressure, filtered from ctrl), `velocity = 0.1`:
   `force = 0.01 * 500000 + (0 + 0 - 100*0.1) = 5000 - 10 = 4990`.

8. **Adhesion:** `gain = 50`, `gaintype=Fixed`, `gainprm=[50,0,0]`,
   `biastype=None`. With `ctrl = 1.0`: `force = 50 * 1.0 = 50`.

9. **Muscle path unchanged:** Verify the restructured gain/bias dispatch
   produces identical results to the pre-change muscle-specific code path.
   Compare force output for a muscle actuator at known `(act, length, velocity)`
   to ≤ 1e-15 relative error.

10. **Dynamics type fix for Position/Velocity:** A Position actuator with
    default `timeconst = 0` has `dyntype=None`, `act_num=0`, `input = ctrl`.
    A Position actuator with `timeconst = 0.1` has `dyntype=FilterExact`,
    `act_num=1`, `input = act`.

11. **Dynamics type fix for Cylinder:** Cylinder has `dyntype=Filter` (not
    `Integrator`), `dynprm=[1.0, 0, 0]`. Filter dynamics:
    `act_dot = (ctrl - act) / 1.0`. Activation converges to `ctrl`.

12. **`kv` default for Velocity:** `<velocity joint="j1"/>` without explicit
    `kv` produces `kv = 1.0` (MuJoCo default), not `kv = 0.0`.

13. **Damper/Adhesion `ctrllimited` enforcement:** `<damper joint="j1" kv="5"/>`
    and `<adhesion body="b1"/>` without explicit `ctrllimited` both have
    `ctrllimited=true` and bounded `ctrlrange` (not `(-inf, inf)`).

14. **Existing tests pass or are updated:** All integration tests (implicit, RK4,
    musculoskeletal, collision, sensor) pass. The builder test
    `test_actuator_activation_states` (`model_builder.rs:2812`) must be
    updated: Position/Velocity without `timeconst` now have `dyntype=None`,
    `act_num=0`, `na=0` (was `Filter`, `1`, `2`). The parser test
    `test_parse_cylinder_actuator` (`parser.rs:2953`) must be updated for
    `timeconst: Option<f64>`. Both are correct fixes — the old assertions
    encoded wrong semantics or types.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify:
  - Add `FilterExact` variant to `ActuatorDynamics` enum (line 426)
  - Add `GainType`, `BiasType` enums (after `ActuatorDynamics`, line 436)
  - Add `actuator_gaintype`, `actuator_biastype` fields to Model (line 921)
  - Initialize new fields in `Model::empty()`
  - Add `FilterExact` arm to Phase 1 in `mj_fwd_actuation()` (line 6500)
  - Replace Phase 2 in `mj_fwd_actuation()` (lines 6523-6552)
  - Update `Data::integrate()` for `FilterExact` (line 2635)
  - Update `mj_runge_kutta()` for `FilterExact` (lines 10164, 10229)
- `sim/L0/mjcf/src/model_builder.rs` — modify:
  - Fix dynamics type assignment (lines 1209-1217)
  - Add `FilterExact` to `act_num` match (lines 1248-1251)
  - Replace dynprm/gain/bias population (lines 1257-1292, the `is_muscle` branch)
  - Push `gaintype`, `biastype` to Model
  - Resolve `timeconst` per actuator type (before dyntype match)
  - Resolve `kv` per actuator type (before dyntype match)
  - Enforce `ctrllimited = true` for Damper and Adhesion (before line 1226)
- `sim/L0/mjcf/src/types.rs` — modify:
  - Change `MjcfActuator.timeconst` from `f64` to `Option<f64>` (line 1814)
  - Change `MjcfActuator.kv` from `f64` to `Option<f64>` (line 1804)
  - Update `Default` impl (line 1867): `timeconst: None`, `kv: None`
  - Update `MjcfActuator::cylinder()` constructor (`timeconst: Some(1.0)`)
  - Update `MjcfActuator::velocity()` constructor (`kv: Some(kv)`)
  - Add `MjcfActuator::damper()` constructor (after `cylinder()`, line 1931)
- `sim/L0/mjcf/src/parser.rs` — modify:
  - Parse `timeconst` into `Some(value)` (line 1123)
  - Parse `kv` into `Some(value)` (line 1110)
  - Update test `test_parse_cylinder_actuator` (line 2953): `cyl.timeconst` becomes
    `Option<f64>`, so change `assert_relative_eq!(cyl.timeconst, 0.5, ...)` to
    `assert_eq!(cyl.timeconst, Some(0.5))`.
- `sim/L0/mjcf/src/defaults.rs` — modify:
  - Update `apply_to_actuator()` (line 276): sentinel check `result.kv == 0.0`
    becomes `result.kv.is_none()`, and `result.kv = kv` becomes
    `result.kv = Some(kv)`.
- `sim/docs/MUJOCO_REFERENCE.md` — modify:
  - Update Phase 2 documentation (line 182)

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
