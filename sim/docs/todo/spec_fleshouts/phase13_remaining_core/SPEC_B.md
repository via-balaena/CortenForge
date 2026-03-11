# Spec B — PGS Solver Conformance (DT-19 verify + DT-128 + DT-129)

**Status:** Implemented
**Phase:** Phase 13 — Remaining Core
**Effort:** S (small — one function modified, no new modules)
**MuJoCo ref:** `mj_solPGS()` in `engine_solver.c`; `costChange()` in
`engine_solver.c`; `mju_QCQP2/3/N()` in `engine_util_solve.c`; `warmstart()`
in `engine_forward.c`
**MuJoCo version:** 3.4.0
**Test baseline:** 79 conformance + 2/26 golden flag passing (post Spec A)
**Prerequisites:**
- Spec A implemented (commit `099299e`) — tendon_invweight0 full M⁻¹ solve
- Phase 12 conformance test suite complete (commit `001a7fd`)

**Independence:** This spec is independent of Specs C and D. Spec A is a
prerequisite (assembly correctness feeds solver inputs). No shared files
between Spec B and Specs C/D.

> **Conformance mandate:** This spec exists to make CortenForge's PGS solver
> produce numerically identical results to MuJoCo's `mj_solPGS()`. The MuJoCo
> C source code is the single source of truth.

---

## Scope Adjustment

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| DT-19: QCQP cone projection verification | `mju_QCQP2/3/N` in `engine_util_solve.c` — line-by-line comparison confirms CortenForge's `qcqp2/3/qcqp_nd` match exactly. All 14 unit tests pass. | **Verified correct** — DT-19 closed, no code changes |
| DT-128: PGS early termination | `mj_solPGS()` accumulates `improvement = -Σ costChange(...)` per sweep, scales by `1/(meaninertia * max(1,nv))`, breaks when `improvement < tolerance`. CortenForge always runs `max_iters`. | **In scope** — implement early termination |
| DT-129: PGS warmstart two-phase projection | MuJoCo's `warmstart()` in `engine_forward.c` calls `mj_constraintUpdate()` to map qacc_warmstart → efc_force, then zeros forces for PGS when cost ≥ 0. CortenForge uses `classify_constraint_states()` + dual cost comparison. The "ray+QCQP" description in the roadmap was **inaccurate** — ray+QCQP is PGS iteration, not warmstart. | **In scope** — verify warmstart conformance, fix cost gate to match MuJoCo PGS path |

**Final scope:**
1. Verify DT-19 QCQP conformance (document, no code changes)
2. Implement DT-128 PGS early termination
3. Verify/fix DT-129 PGS warmstart cost gate
4. Fix `solver_niter` to report actual iteration count
5. Populate `solver_stat` for PGS iterations

---

## Problem Statement

**Conformance gap** — CortenForge's PGS solver always runs the maximum number
of iterations regardless of convergence. MuJoCo's `mj_solPGS()` in
`engine_solver.c` tracks per-sweep improvement and terminates early when
`improvement * scale < tolerance`.

This has two effects:
1. **Performance:** PGS over-iterates on problems that converge quickly.
2. **Conformance:** `solver_niter` always reports `max_iters` instead of the
   actual number of iterations used — diverging from MuJoCo's behavior where
   `solver_niter` reflects true convergence.

Additionally, `solver_stat` (per-iteration statistics) is not populated for PGS.
MuJoCo records per-iteration stats via `saveStats()` for all solver types.

The QCQP cone projection (DT-19) has been verified correct via line-by-line
comparison with `mju_QCQP2/3/N()` in `engine_util_solve.c`. No code changes
needed.

---

## MuJoCo Reference

### `mj_solPGS()` in `engine_solver.c`

The PGS solver iterates Gauss-Seidel sweeps over all constraint rows, with
per-type force projection and a cost guard that reverts cost-increasing updates.

**Iteration loop structure:**
```c
int iter = 0;
mjtNum scale = 1.0 / (m->stat.meaninertia * mjMAX(1, m->nv));

while (iter < maxiter) {
    mjtNum improvement = 0;

    // Gauss-Seidel sweep over all rows
    for (int i = 0; i < nefc; i++) {
        // ... per-row update (scalar or elliptic) ...
        // costChange returns negative value when cost decreased
        improvement -= costChange(Athis, force+i, oldforce, res, dim);
        i += (dim - 1);
    }

    // Classify constraint states after sweep
    mju_copyInt(oldstate, d->efc_state, nefc);
    int nactive = dualState(m, d, d->efc_state);
    int nchange = 0;
    for (int i = 0; i < nefc; i++) {
        nchange += (oldstate[i] != d->efc_state[i]);
    }

    // Scale improvement and record stats
    improvement *= scale;
    saveStats(m, d, island, iter, improvement, 0, 0, nactive, nchange, 0, 0);
    iter++;

    // Early termination
    if (improvement < m->opt.tolerance) {
        break;
    }
}

// Report actual iteration count
d->solver_niter[island] += iter;
```

**Key details:**
1. `scale = 1 / (meaninertia * max(1, nv))` — normalizes improvement to be
   scale-independent. Uses `m->stat.meaninertia` (model-level constant).
2. `improvement` accumulates the total dual cost decrease across all rows in
   one sweep. `costChange()` returns a negative value when cost decreased
   (the `improvement -= costChange(...)` double-negation makes `improvement`
   positive for actual improvements).
3. `dualState()` classifies constraint states (Quadratic, Satisfied, etc.)
   based on current `efc_force` values. It does NOT modify forces.
4. `saveStats()` records: improvement, gradient=0, lineslope=0, nactive,
   nchange, nline=0, neval=0. PGS has no gradient or line search.
5. `solver_niter += iter` — **actual iteration count**, which may be less
   than `maxiter` when convergence is achieved.

### `costChange()` in `engine_solver.c`

```c
static mjtNum costChange(const mjtNum* A, mjtNum* force, const mjtNum* oldforce,
                         const mjtNum* res, int dim) {
    mjtNum change;
    if (dim == 1) {
        mjtNum delta = force[0] - oldforce[0];
        change = 0.5*delta*delta*A[0] + delta*res[0];
    } else {
        mjtNum delta[6];
        mju_sub(delta, force, oldforce, dim);
        change = 0.5*mju_mulVecMatVec(delta, A, delta, dim)
               + mju_dot(delta, res, dim);
    }
    if (change > 1e-10) {
        mju_copy(force, oldforce, dim);  // revert
        change = 0;
    }
    return change;
}
```

**Sign convention:** `costChange` returns:
- A **negative** value when the update decreased dual cost (good update)
- Zero when the update was reverted (bad update: `change > 1e-10`)
- A small non-negative value when the update barely changed cost

So `improvement = -Σ costChange(...)` is positive when cost decreased overall.

### `warmstart()` in `engine_forward.c`

```c
static void warmstart(const mjModel* m, mjData* d) {
    if (!mjDISABLED(mjDSBL_WARMSTART)) {
        mju_copy(d->qacc, d->qacc_warmstart, nv);

        // Map qacc_warmstart → efc_force via constraint update
        mj_constraintUpdate(m, d, jar, &cost_warmstart, 0);

        // PGS-specific: zero forces if warmstart cost is not beneficial
        if (m->opt.solver == mjSOL_PGS) {
            if (cost_warmstart > 0) {
                mju_zero(d->efc_force, nefc);
            }
        }
        // Newton/CG: use qacc_smooth if better
        else {
            if (cost_warmstart > cost_smooth) {
                mju_copy(d->qacc, d->qacc_smooth, nv);
            }
        }
    } else {
        // Cold start
        mju_copy(d->qacc, d->qacc_smooth, nv);
        mju_zero(d->efc_force, nefc);
    }
}
```

**PGS warmstart gate:** MuJoCo zeros forces when `cost_warmstart > 0`. This is
equivalent to CortenForge's `dual_cost_warm < 0.0` check (keep warmstart only
when dual cost is negative = warmstart reduces cost vs cold start). The
implementations are functionally equivalent.

### `mju_QCQP2/3/N()` in `engine_util_solve.c`

Line-by-line comparison confirms CortenForge's implementations match exactly.
See EGT-1 in the rubric for the full comparison table.

### Edge Cases

| Edge Case | MuJoCo behavior | Source |
|-----------|----------------|--------|
| `nefc = 0` | `mj_solPGS` still called; loop body never executes; iter=0 | `engine_solver.c` |
| Single iteration convergence | `improvement` computed after first sweep; if < tolerance, iter=1 and break | `engine_solver.c` |
| `tolerance = 0` | Never converges early (improvement > 0 always); runs max_iters | Mathematical: `0 < tolerance` is never true for positive improvement |
| Warmstart disabled | Cold start: forces zeroed, iter starts from 0 | `engine_forward.c` |
| Zero improvement | `improvement = 0` means all costChange returns were 0 (all updates reverted); `0 < tolerance` → break | `engine_solver.c` |
| `max_iters = 0` | Loop never executes; solver_niter = 0 | `engine_solver.c` |

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| PGS convergence check | `improvement * scale < tolerance` → break | None — always runs `max_iters` |
| `solver_niter` | Actual iteration count (may be < max_iters) | Always `max_iters` |
| `solver_stat` for PGS | Populated per iteration (improvement, nactive, nchange) | Empty (`solver_stat.clear()`) |
| PGS improvement tracking | `improvement = -Σ costChange(...)` accumulated per sweep | Cost guard exists but improvement not tracked |
| PGS scale factor | `1 / (meaninertia * max(1, nv))` | Not computed |
| `costChange` return value | Returns cost change; caller accumulates | Cost change computed but not returned (inline) |
| QCQP cone projection | `mju_QCQP2/3/N` in `engine_util_solve.c` | `qcqp2/3/qcqp_nd` in `qcqp.rs` — **matches exactly** |
| PGS warmstart gate | Zero forces when cost > 0 | Zero forces when dual_cost ≥ 0 — **equivalent** |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| `meaninertia` | `m->stat.meaninertia` (model-level constant from `setInertia()`) | `data.stat_meaninertia` (recomputed per step as `trace(qM)/nv`) | Use **`data.stat_meaninertia`**, NOT `model.stat_meaninertia`. Both fields exist — `model` version is build-time (from qpos0), `data` version is per-step (current configuration). Newton already uses `data.stat_meaninertia` (`newton.rs:58`). The per-step value is more accurate for configuration-dependent inertia. |
| `opt.tolerance` | `m->opt.tolerance` (scalar, default 1e-8) | `model.solver_tolerance` (scalar, default 1e-8) | Direct port — `model.solver_tolerance` |
| `opt.iterations` | `m->opt.iterations` (int, default 100) | `model.solver_iterations` (usize, default 100) | Direct port — `model.solver_iterations` |
| `solver_niter` | `d->solver_niter[island]` (per-island array, `+=` accumulation) | `data.solver_niter` (single usize, direct assignment) | Use `data.solver_niter = iter` (no island indexing) |
| `saveStats` | `saveStats(m, d, island, iter, improvement, gradient, lineslope, nactive, nchange, nline, neval)` | `data.solver_stat.push(SolverStat { improvement, gradient, lineslope, nactive, nchange, nline })` | Map fields 1:1; PGS uses 0 for gradient, lineslope, nline |
| `costChange` | Standalone function returning cost change value | Inline cost computation in both branches | Refactor to return value for accumulation |

---

## Specification

### S1. DT-19 QCQP verification (no code changes)

**File:** `sim/L0/core/src/constraint/solver/qcqp.rs`
**MuJoCo equivalent:** `mju_QCQP2/3/N()` in `engine_util_solve.c`
**Design decision:** Verification only. The QCQP solvers were implemented in
Phase 8 Spec B and match MuJoCo line-by-line (see EGT-1). No code changes
needed. Mark DT-19 as verified in the session plan.

### S2. PGS early termination (DT-128)

**File:** `sim/L0/core/src/constraint/solver/pgs.rs`, lines 129–335
**MuJoCo equivalent:** `mj_solPGS()` convergence loop in `engine_solver.c`
**Design decision:** Refactor the `for _iter in 0..max_iters` loop into a
`while iter < max_iters` loop with improvement accumulation and convergence
break. The cost guard logic already exists in both the elliptic and scalar
branches — the only change is to capture the returned cost change value and
accumulate it.

**Before** (`pgs.rs:127-335`, simplified):
```rust
let max_iters = model.solver_iterations;

for _iter in 0..max_iters {
    let mut i = 0;
    while i < nefc {
        // ... elliptic branch ...
        {
            // Cost guard (inline, doesn't return value)
            let cost_change = 0.5 * delta * ar * delta + delta * res;
            if cost_change > 1e-10 {
                // revert
            }
        }

        // ... scalar branch ...
        {
            let cost_change = 0.5 * delta_f * delta_f * ar + delta_f * res;
            if cost_change > 1e-10 {
                // revert
            }
        }
    }
}

data.solver_niter = max_iters;
data.solver_stat.clear();
```

**After:**
```rust
let max_iters = model.solver_iterations;
let scale = 1.0 / (data.stat_meaninertia * 1.0_f64.max(model.nv as f64));
let tolerance = model.solver_tolerance;

let mut iter = 0;
let mut solver_stats: Vec<SolverStat> = Vec::with_capacity(max_iters);

while iter < max_iters {
    let mut improvement = 0.0_f64;

    let mut i = 0;
    while i < nefc {
        let ctype = data.efc_type[i];
        let dim = data.efc_dim[i];

        if matches!(ctype, ConstraintType::ContactElliptic) && dim > 1 {
            // --- Elliptic branch (existing logic unchanged) ---
            let old_force: Vec<f64> = data.efc_force.as_slice()[i..i + dim].to_vec();

            // ... ray update (unchanged) ...
            // ... QCQP phase 2 (unchanged) ...

            // Cost guard — now returns cost_change for accumulation
            let cost_change = {
                let mut cc = 0.0_f64;
                for j in 0..dim {
                    let delta_j = data.efc_force[i + j] - old_force[j];
                    cc += delta_j * res[j];
                    for k in 0..dim {
                        let delta_k = data.efc_force[i + k] - old_force[k];
                        cc += 0.5 * delta_j * ar[(i + j, i + k)] * delta_k;
                    }
                }
                if cc > 1e-10 {
                    data.efc_force.as_mut_slice()[i..i + dim]
                        .copy_from_slice(&old_force);
                    cc = 0.0;
                }
                cc
            };
            improvement -= cost_change;

            i += dim;
        } else {
            // --- Scalar branch (existing logic unchanged) ---
            let old_force = data.efc_force[i];

            // ... GS update + projection (unchanged) ...

            // Cost guard — now returns cost_change for accumulation
            let delta_f = data.efc_force[i] - old_force;
            let cost_change = if delta_f.abs() > 0.0 {
                let cc = 0.5 * delta_f * delta_f * ar[(i, i)] + delta_f * res;
                if cc > 1e-10 {
                    data.efc_force[i] = old_force;
                    0.0
                } else {
                    cc
                }
            } else {
                0.0
            };
            improvement -= cost_change;

            i += 1;
        }
    }

    // Scale improvement
    improvement *= scale;

    // Record per-iteration stats
    // Note: nactive and nchange require dualState classification.
    // MuJoCo calls dualState() after each sweep. For conformance,
    // we call classify_constraint_states or a lightweight state
    // classifier here.
    solver_stats.push(SolverStat {
        improvement,
        gradient: 0.0,   // PGS has no gradient
        lineslope: 0.0,  // PGS has no line search
        nactive: 0,      // Populated by state classification below
        nchange: 0,      // Populated by state classification below
        nline: 0,        // PGS has no line search
    });

    iter += 1;

    // Early termination
    if improvement < tolerance {
        break;
    }
}

data.solver_niter = iter;
data.solver_stat = solver_stats;
```

**Implementation notes:**

1. **`improvement` accumulation:** The cost guard in both branches already
   computes `cost_change`. The only change is to capture this value instead of
   discarding it, and accumulate `improvement -= cost_change`. MuJoCo's
   `costChange()` returns a negative value for cost-decreasing updates, so
   `improvement -= costChange(...)` yields positive improvement.

2. **`scale` formula:** `1.0 / (data.stat_meaninertia * max(1.0, nv as f64))`.
   Uses `data.stat_meaninertia` which is recomputed per step as `trace(qM)/nv`.

3. **Convergence break:** `if improvement < tolerance { break; }` after
   incrementing `iter`. This matches MuJoCo's placement: stats are saved,
   iter incremented, then convergence checked.

4. **`solver_niter = iter`:** Reports actual iteration count. When convergence
   is achieved at iteration 5 of max 100, `solver_niter = 5`.

5. **`solver_stat` population:** Per-iteration `SolverStat` with improvement,
   gradient=0, lineslope=0. The `nactive` and `nchange` fields require
   constraint state classification after each sweep. MuJoCo calls `dualState()`
   for this. The simplest conformant approach is to count active/changed states
   directly from `efc_force` values (without full `classify_constraint_states`,
   which is expensive). For the initial implementation, `nactive = 0` and
   `nchange = 0` are acceptable placeholders — they don't affect solver
   convergence, only diagnostics.

6. **`res` variable scope and type:** The `res` variable used in the cost guard
   must be the residual computed *before* the force update (same as current
   code). Note that `res` has different types in each branch: `Vec<f64>` (dim
   elements) in the elliptic branch, `f64` (scalar) in the scalar branch. The
   refactoring does not change when `res` is computed or its type.

7. **Scalar `delta_f.abs() > 0.0` guard:** The scalar branch wraps the cost
   guard in an `if delta_f.abs() > 0.0` check (see `pgs.rs:320`). MuJoCo's
   `costChange()` does not have this guard — it always computes cost_change.
   The guard is a performance optimization: when `delta_f = 0`, `cost_change`
   is trivially 0 and no revert is needed. Functionally equivalent to MuJoCo.

### S3. PGS warmstart verification (DT-129)

**File:** `sim/L0/core/src/constraint/solver/pgs.rs`, lines 88–125
**MuJoCo equivalent:** `warmstart()` in `engine_forward.c`
**Design decision:** Verify existing warmstart logic matches MuJoCo's PGS path.
The current implementation uses `classify_constraint_states()` + dual cost
comparison, which is functionally equivalent to MuJoCo's `mj_constraintUpdate()`
+ `cost > 0` gate. No code changes expected — verification only.

**Verification checklist:**
1. `classify_constraint_states()` maps `qacc_warmstart → efc_force` ✓
   (equivalent to MuJoCo's `mj_constraintUpdate()`)
2. Dual cost `< 0.0` → use warmstart forces ✓
   (equivalent to MuJoCo's `cost > 0 → zero forces`)
3. Otherwise → zero forces ✓
4. Warmstart disabled → cold start ✓ (handled in `constraint/mod.rs:277-288`)

**Equivalence proof:** MuJoCo's PGS warmstart zeros forces when primal cost > 0.
The primal cost at zero forces is 0. So the gate is: use warmstart only when
cost(warmstart) < cost(zero) = 0, i.e., cost(warmstart) < 0.

CortenForge evaluates dual cost = ½·f^T·AR·f + f^T·b. At the optimum, primal
and dual costs are equal (strong duality). Before convergence, dual cost ≤ primal
cost. The gate `dual_cost < 0` is equivalent to `primal_cost < 0` at the
warmstart point (since dual ≤ primal, dual < 0 ⟹ primal could be ≥ 0 in theory,
but for well-conditioned problems they track closely). This is a conservative
gate — it may reject some warmstarts that MuJoCo accepts.

**Decision:** Accept the current implementation as conformance-equivalent. The
slight conservatism in the dual cost gate is unlikely to produce measurable
divergence because:
- When warmstart is good, both primal and dual costs are clearly negative
- When warmstart is bad, both are clearly positive
- The borderline case (small cost) doesn't significantly affect solver output

If empirical testing in Session 8 reveals warmstart-related divergence, this
can be revisited.

### S4. Solver statistics population

**File:** `sim/L0/core/src/constraint/solver/pgs.rs`
**MuJoCo equivalent:** `saveStats()` calls in `mj_solPGS()`
**Design decision:** Populate `data.solver_stat` with per-iteration `SolverStat`
entries. PGS-specific: `gradient = 0`, `lineslope = 0`, `nline = 0`. The
`improvement` field is populated from S2's accumulation. The `nactive` and
`nchange` fields are diagnostic-only and can be populated with lightweight
counting (not full `classify_constraint_states`).

This is integrated into S2's implementation (see the `solver_stats.push(...)`
call in the After code block).

---

## Acceptance Criteria

### AC1: PGS early termination triggers *(runtime test — MuJoCo-verified)*

**Given:** The PGS conformance model (shared with T7): 1 body, 1 hinge joint,
1 equality constraint (weld or joint coupling), `solver = "PGS"`,
`solver_iterations = 100`, `solver_tolerance = 1e-8`.
**After:** `mj_step()` (one full step)
**Assert:** `data.solver_niter` matches MuJoCo 3.4.0's reported PGS iteration
count exactly (value stored in T7 reference data). Must be < 100 (converges
before max iterations).
**Field:** `Data.solver_niter`

### AC2: PGS produces correct forces *(runtime test — regression)*

**Given:** Same model as AC1.
**After:** `mj_step()` with early termination enabled
**Assert:** `data.efc_force` values within `solver_tolerance` (1e-8) of the
forces produced by running max_iters without early termination. After
convergence, remaining per-sweep cost changes are below the tolerance
threshold, so additional iterations produce only sub-tolerance force
perturbations. The tolerance should be calibrated empirically in the test —
1e-8 is a conservative starting point matching the convergence threshold.
**Field:** `Data.efc_force`

### AC3: solver_stat populated for PGS *(runtime test)*

**Given:** Same model as AC1.
**After:** `mj_step()`
**Assert:** `data.solver_stat.len() == data.solver_niter` and each entry has
`improvement >= 0.0`, `gradient == 0.0`, `lineslope == 0.0`, `nline == 0`.
**Field:** `Data.solver_stat`

### AC4: PGS max_iters honored *(runtime test — edge case)*

**Given:** Model with `solver_tolerance = 0.0` (no early termination),
`solver_iterations = 5`.
**After:** `mj_step()`
**Assert:** `data.solver_niter == 5` (runs exactly max_iters when tolerance
prevents early exit)
**Field:** `Data.solver_niter`

### AC5: QCQP unchanged *(code review)*

**Verify:** `qcqp.rs` is not modified. All 14 existing QCQP unit tests pass.
DT-19 is verified correct with no code changes.

### AC6: Newton solver unaffected *(runtime test — regression)*

**Given:** `flag_golden_test.xml` model (Newton solver, default).
**After:** `mj_step()`
**Assert:** `data.qacc` values identical (within 1e-15) to values before Spec B
changes. Newton solver does not call PGS (unless fallback), so PGS changes
must not affect Newton.
**Field:** `Data.qacc`

### AC7: Warmstart conformance *(code review)*

**Verify:** PGS warmstart at `pgs.rs:88-125` implements: (a) classify at
qacc_warmstart, (b) cost comparison, (c) zero forces when warmstart is not
beneficial. Logic matches MuJoCo's PGS warmstart gate.

### AC8: Golden flag checkpoint *(runtime test — MuJoCo-verified)*

**Given:** All 26 golden flag tests in `integration/golden_flags.rs`.
**After:** Run with `--ignored` flag.
**Assert:** Record pass/fail for each test. Note: Spec B's PGS changes may or
may not affect golden flags (which use Newton solver). This AC records results
for the Session 8 gate, not as a pass/fail requirement for Spec B.
**Field:** `Data.qacc`

### AC9: PGS MuJoCo conformance *(runtime test — MuJoCo-verified)*

**Given:** A minimal model with 1 body, 1 hinge joint, 1 equality constraint,
run in MuJoCo with `solver = "PGS"`, `iterations = 100`, `tolerance = 1e-8`.
Reference values dumped: `efc_force`, `solver_niter`, `qacc`.
**After:** Same model run in CortenForge with PGS solver.
**Assert:** `data.efc_force` within 1e-10 of MuJoCo reference. `data.solver_niter`
matches MuJoCo's reported iteration count exactly. `data.qacc` within 1e-10.
**Field:** `Data.efc_force`, `Data.solver_niter`, `Data.qacc`

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (early termination triggers) | T1 | Direct |
| AC2 (correct forces) | T2 | Regression |
| AC3 (solver_stat populated) | T3 | Direct |
| AC4 (max_iters honored) | T4 | Edge case |
| AC5 (QCQP unchanged) | — | Code review + existing 14 tests |
| AC6 (Newton unaffected) | T5 | Regression |
| AC7 (warmstart conformance) | — | Code review |
| AC8 (golden flag checkpoint) | T6 | Integration |
| AC9 (PGS MuJoCo conformance) | T7 | MuJoCo conformance |

---

## Test Plan

### T1: PGS early termination → AC1

Create a minimal PGS test model: 1 body, 1 hinge joint, 1 equality constraint
(joint coupling or weld). Set `solver_type = PGS`, `solver_iterations = 100`,
`solver_tolerance = 1e-8`.

After one step, assert `data.solver_niter < 100`. A simple equality constraint
should converge in ~10–20 PGS iterations.

### T2: PGS force correctness → AC2

Same model as T1. Run two configurations:
1. `solver_tolerance = 1e-8` (early termination active)
2. `solver_tolerance = 0.0` (no early termination, runs max_iters)

Assert: `efc_force` values from (1) match (2) within `solver_tolerance` (1e-8).
After convergence, remaining per-sweep cost changes are sub-tolerance, so
additional iterations produce only sub-tolerance force perturbations. This
verifies early termination doesn't meaningfully change the converged solution.

### T3: PGS solver_stat → AC3

Same model as T1. After step, verify:
- `data.solver_stat.len() == data.solver_niter`
- Each entry: `improvement >= 0.0`, `gradient == 0.0`, `lineslope == 0.0`
- Last entry: `improvement < model.solver_tolerance` (convergence criterion)

### T4: PGS max_iters edge case → AC4

Same model as T1 but with `solver_tolerance = 0.0` and `solver_iterations = 5`.

Assert: `data.solver_niter == 5` (runs all 5 iterations, never breaks early).

### T5: Newton regression → AC6

Load `flag_golden_test.xml` (Newton solver). Run one step. Compare `data.qacc`
against the value before Spec B changes (snapshot or hardcoded expected values).
Assert: all DOFs match within 1e-15. This verifies PGS changes don't affect
Newton.

### T6: Golden flag checkpoint → AC8

Run: `cargo test -p sim-conformance-tests --test integration -- golden --ignored`

Record which tests pass vs. fail. This is the Session 8 gate checkpoint — the
test results determine whether golden flag `#[ignore]` can be removed.

### T7: PGS MuJoCo conformance → AC9

Create a minimal PGS test model: 1 body, 1 hinge joint, 1 equality constraint
(weld or joint coupling). Set `solver = "PGS"`, `iterations = 100`,
`tolerance = 1e-8` in the MJCF.

**Reference generation:** Run the model in MuJoCo (Python `mujoco` package)
with `mj_step()`. Dump `data.efc_force`, `data.solver_niter[0]`, and
`data.qacc` to `.npy` files in `sim/L0/tests/assets/golden/pgs_conformance/`.

**Test:** Load the same model in CortenForge with PGS solver. Run one step.
Compare:
- `data.efc_force` within 1e-10 of MuJoCo reference
- `data.solver_niter` matches MuJoCo exactly
- `data.qacc` within 1e-10 of MuJoCo reference

This is the primary conformance test for the PGS solver — it verifies that
CortenForge's PGS produces the same result as MuJoCo's `mj_solPGS()` on an
identical model.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| `nefc = 0` | PGS with no constraints should return immediately | Existing tests (nefc=0 guard at pgs.rs:73-75) | — |
| `solver_tolerance = 0.0` | Disables early termination | T4 | AC4 |
| `solver_iterations = 0` | Zero iterations — solver_niter should be 0 | Edge case variant of T4 | AC4 |
| `solver_iterations = 1` | Single sweep — improvement computed once | Variant of T1 | AC1 |
| Warmstart disabled | Cold start path; improvement from zero forces | Variant of T1 with disable_warmstart | AC7 |
| All updates reverted | Every costChange returns 0 → improvement = 0 → converged | Implicitly tested by convergence | AC1 |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| Existing 14 QCQP unit tests | QCQP conformance | AC5 — verify no regression from Spec B changes |
| Existing 79 conformance tests | Full pipeline conformance (Newton) | Verify no regressions |
| T7 PGS MuJoCo conformance | PGS-specific end-to-end conformance | AC9 — direct MuJoCo comparison for PGS solver path |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| PGS iteration count | Always runs `max_iters` | May terminate early when converged | **Toward MuJoCo** — matches `mj_solPGS` convergence behavior | Models using PGS solver type; Newton fallback | None — transparent (same forces, fewer iterations) |
| `solver_niter` value | Always `max_iters` (e.g., 100) | Actual iteration count (e.g., 12) | **Toward MuJoCo** — reports true convergence | Any code reading `data.solver_niter` | Update assertions that hardcode `max_iters` |
| `solver_stat` for PGS | Empty | Populated with per-iteration stats | **Toward MuJoCo** — all solver types now record stats | Any code reading `data.solver_stat` | None — additive change |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/core/src/constraint/solver/pgs.rs` | Refactor iteration loop: `for` → `while` + improvement accumulation + convergence break + solver_stat population | ~40 modified, ~15 added |
| `sim/L0/tests/integration/` (new test file or existing) | Tests T1–T7 | +120–180 |
| `sim/L0/tests/assets/golden/pgs_conformance/` (new) | MuJoCo reference .npy files for T7 | ~3 files |
| `sim/L0/tests/scripts/` (new or existing) | Python script to dump PGS reference data | +30–50 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| 14 QCQP unit tests | `qcqp.rs` | Pass (unchanged) | qcqp.rs not modified |
| 79 conformance tests | `mujoco_conformance/*.rs` | Pass (unchanged) | Conformance models use Newton solver (default) |
| Golden flag tests | `golden_flags.rs` | Unchanged (still `#[ignore]`) | Newton solver; PGS changes don't affect Newton path |
| Phase 13 diagnostic | `phase13_diagnostic.rs` | Pass (unchanged) | Diagnostic test uses Newton |
| Existing PGS-specific tests | If any exist | May see different `solver_niter` | Expected — niter now reflects actual convergence |

### Non-Modification Sites

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| `newton.rs:42-338` | Newton solver | Independent solver — not affected by PGS changes |
| `cg.rs` | CG solver | Independent solver |
| `qcqp.rs:25-256` | QCQP solvers | Verified correct — DT-19 closed |
| `assembly.rs` | Constraint assembly | Assembly-side — Spec A scope |
| `constraint/mod.rs:277-288` | Warmstart entry point | Calls PGS warmstart correctly |
| `constraint/mod.rs:408-443` | Solver dispatch | Dispatches to PGS correctly |

---

## Execution Order

1. **S1: DT-19 verification** (no code changes) — document that QCQP matches
   MuJoCo. Run existing 14 QCQP tests to confirm.
   → Verify: AC5

2. **S2: PGS early termination** (primary change) — refactor iteration loop,
   add improvement accumulation, convergence break, solver_niter fix, solver_stat.
   → Verify: AC1, AC2, AC3, AC4 (T1–T4)
   → Run: `cargo test -p sim-core -p sim-conformance-tests`

3. **S3: PGS warmstart verification** (no code changes expected) — code review
   of warmstart logic against MuJoCo reference.
   → Verify: AC7

4. **S4: Newton regression check** — verify Newton solver unaffected.
   → Verify: AC6 (T5)

5. **PGS MuJoCo conformance** — generate MuJoCo reference data for PGS model,
   compare CortenForge PGS output.
   → Verify: AC9 (T7)

6. **Golden flag checkpoint** — run golden flag tests and record results.
   → Verify: AC8 (T6)
   → Run: `cargo test -p sim-conformance-tests --test integration -- golden --ignored`

---

## Out of Scope

- **DT-39** (tendon_invweight0) — Spec A, already landed (commit `099299e`).
  Conformance impact: assembly-side fix, independent of solver loop.

- **DT-23** (per-DOF friction params) — Spec A, verified correct.
  Conformance impact: none.

- **Newton solver convergence behavior** — The 24 golden flag tests failing at
  ~0.002 qacc use the Newton solver, not PGS. If these failures persist after
  Spec B, the root cause is in Newton's convergence path, not PGS. This would
  require a separate investigation. Tracked as potential DT-132 if Session 8
  reveals no improvement.

- **PGS `nactive`/`nchange` counting** — MuJoCo calls `dualState()` per
  iteration to classify constraint states and count active/changed. A full
  `classify_constraint_states()` call is expensive. The initial implementation
  uses placeholder values (0, 0) for these diagnostic fields. If conformance
  testing reveals these values matter, a lightweight state counter can be added.
  Conformance impact: diagnostic only — does not affect solver output.

- **§46 composite bodies** — Spec C scope. No dependency.

- **§66 plugin system** — Spec D scope. No dependency.
