# Spec A — Constraint Assembly Conformance (DT-39 + DT-23)

**Status:** Draft
**Phase:** Phase 13 — Remaining Core
**Effort:** S (small — single function fix + verification)
**MuJoCo ref:** `setInertia()` in `engine_setconst.c`
**MuJoCo version:** 3.4.0
**Test baseline:** 79 conformance tests + 2/26 golden flag passing
**Prerequisites:**
- Phase 12 conformance test suite complete (commit `001a7fd`)
- Phase 13 Session 0 diagnostic complete (commit `cc20e60`)

**Independence:** This spec is independent of Spec B (solver loop). Spec A
fixes assembly-side inputs (tendon_invweight0); Spec B fixes solver-side
iteration (PGS convergence, warmstart). No shared files.

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for constraint assembly of
> tendon-involving constraints. The MuJoCo C source code is the single
> source of truth.

---

## Scope Adjustment

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| DT-39: `diagApprox` remaining code paths | Root cause is `tendon_invweight0` diagonal-only bug in `compute_invweight0()` at `model_init.rs:1062-1074`. Not in diagApprox dispatch — the dispatch correctly reads `tendon_invweight0[t]`; the stored value is wrong. | In scope — fix `tendon_invweight0` to use full `J · M⁻¹ · J^T` |
| DT-23: Per-DOF friction solver params | Assembly code correctly routes per-DOF params at `assembly.rs:369-371`. Tests T7–T11 verify. No divergence in diagnostic. | In scope — verification only, no code changes |
| FILTERPARENT (DT-131) | Same root cause as baseline (~0.020 vs ~0.002). Not independent. | Drop — retracted per Session 0 |

**Final scope:**
1. Fix `tendon_invweight0` for all tendon types (DT-39)
2. Verify DT-23 routing (no code changes expected)

---

## Problem Statement

**Correctness bug** — CortenForge computes wrong `tendon_invweight0` values.

`tendon_invweight0[t]` is the inverse inertia weight for tendon `t`, used by
`mj_diagApprox()` to compute constraint regularization (R, D) for tendon
friction loss and tendon limit constraint rows. CortenForge's
`compute_invweight0()` at `model_init.rs:1062-1074` computes this as:

```
tendon_invweight0 = Σ(coef_i² · M⁻¹[i,i])     (diagonal only)
```

MuJoCo's `setInertia()` in `engine_setconst.c` computes:

```
tendon_invweight0 = J_tendon · M⁻¹ · J_tendon^T     (full quadratic form)
```

The full quadratic form includes off-diagonal elements of M⁻¹ that represent
coupling between DOFs in a serial kinematic chain. For the `flag_golden_test.xml`
model, the missing off-diagonal term is 90.29 (17% of the total), causing
the tendon friction loss constraint row's regularization to be wrong, which
cascades through the Newton solver to produce 0.002–0.02 qacc divergence
across all 8 DOFs.

This is the single root cause of 24 of 26 failing golden flag tests
(the other 2 already pass). Session 0's diagnostic confirmed this with
row-by-row constraint data comparison.

---

## MuJoCo Reference

### `setInertia()` in `engine_setconst.c`

Called during model compilation (`mj_setConst`). At this point, a forward
pass at `qpos0` has already been executed, populating `data->ten_J` (tendon
Jacobians) and factoring the mass matrix M.

The tendon_invweight0 computation:

```c
// For each tendon:
for (int i = 0; i < m->ntendon; i++) {
    // ten_J[i] is an nv-length row: d(length)/d(qvel)
    mjtNum* J = d->ten_J + i * m->nv;

    // Copy J to workspace, then solve M * res = J (in-place)
    mju_copy(res, J, m->nv);
    mj_solveM(m, d, res, res, 1);
    // Now res = M⁻¹ · J^T

    // tendon_invweight0 = J · M⁻¹ · J^T (scalar: 1×nv · nv×1)
    m->tendon_invweight0[i] = mju_max(mjMINVAL, mju_dot(J, res, m->nv));
}
```

Key observations:
1. **Uses `ten_J`** — the full nv-length Jacobian computed by `mj_tendon()`,
   not reconstructed from wrap coefficients.
2. **`mj_solveM` uses factored LDL** — equivalent to CortenForge's
   `mj_solve_sparse_batch`.
3. **Applied to ALL tendons uniformly** — fixed and spatial use the same
   algorithm. No type-specific branching.
4. **Result is clamped to `mjMINVAL`** (1e-15).

### `mj_diagApprox()` in `engine_core_constraint.c`

Returns the precomputed `tendon_invweight0[t]` for tendon-related constraints:

```c
case mjCNSTR_FRICTION_LOSS:
    // id = tendon index for tendon friction
    return m->tendon_invweight0[id];

case mjCNSTR_LIMIT_TENDON:
    return m->tendon_invweight0[id];
```

This function does NOT recompute invweight0 — it reads the value stored by
`setInertia()`. Therefore, fixing the stored value in `compute_invweight0()`
is sufficient; no changes needed in the diagApprox dispatch or assembly code.

### `mj_makeConstraint()` in `engine_core_constraint.c`

Assembles tendon friction loss rows using `tendon_solref_friction` and
`tendon_solimp_friction` per-tendon. For DOF friction, uses per-DOF
`dof_solref_friction` and `dof_solimp_friction`. CortenForge's assembly
at `assembly.rs:369-371` (DOF) and `assembly.rs:396-398` (tendon)
already routes these correctly — confirmed by Session 0 diagnostic (DT-23).

### Edge Cases

| Edge Case | MuJoCo behavior | Source |
|-----------|----------------|--------|
| Zero-Jacobian tendon (all coefficients zero) | `mju_dot` returns 0.0 → clamped to `mjMINVAL` | `engine_setconst.c` |
| Single-joint tendon (1 non-zero coef) | `J · M⁻¹ · J^T = coef² · M⁻¹[d,d]` — identical to diagonal formula | Mathematical identity |
| Spatial tendon | Same algorithm as fixed — uses `ten_J` from `mj_tendon()` | `engine_setconst.c` (no type branch) |
| nv=0 model (no DOFs) | `ten_J` is empty → `mju_dot` returns 0 → `mjMINVAL` | Early return guard |
| Tendon with ball/free joint wraps | `ten_J[dof_adr]` = coef for first DOF component (angular or linear velocity axis). Off-diagonal coupling through multi-DOF subblock is captured by M⁻¹ solve. | `mj_tendon()` in `engine_core_smooth.c` |

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| tendon_invweight0 computation | `J · M⁻¹ · J^T` via `setInertia()` in `engine_setconst.c` | Diagonal-only `Σ(coef² · M⁻¹[i,i])` for fixed; body_invweight0 average for spatial |
| diagApprox for tendon constraints | Returns precomputed `tendon_invweight0[t]` | Same — reads stored value (correct dispatch, wrong stored value) |
| DT-23: per-DOF friction solref/solimp | `dof_solref_friction[dof]` per-DOF | `model.dof_solref[dof_idx]` per-DOF (correct) |
| DT-23: per-tendon friction solref/solimp | `tendon_solref_friction[t]` per-tendon | `model.tendon_solref_fri[t]` per-tendon (correct) |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| `ten_J` layout | `d->ten_J + i*nv` (contiguous C array) | `data.ten_J[t]: DVector<f64>` (Vec of DVector) | Access `data.ten_J[t]` directly — no index arithmetic needed |
| `mj_solveM` | `mj_solveM(m, d, res, res, 1)` (in-place, single RHS) | `mj_solve_sparse_batch(rowadr, rownnz, colind, qLD_data, qLD_diag_inv, &mut rhs)` (DMatrix batch) | Wrap single DVector in 1-column DMatrix for batch solver |
| `mjMINVAL` | `1e-15` | `MIN_VAL` = `1e-15` (local const in `compute_invweight0`) | Direct port — same value |
| `mju_dot` | `mju_dot(J, res, nv)` | `j_tendon.dot(&w_col)` (DVector dot) or manual loop | Use DVector::dot or manual loop over nv elements |

---

## Specification

### S1. Fix `tendon_invweight0` — full M⁻¹ solve

**File:** `sim/L0/core/src/types/model_init.rs`, lines 1056–1115
**MuJoCo equivalent:** `setInertia()` in `engine_setconst.c`, tendon loop
**Design decision:** Use `data.ten_J[t]` (already populated by `mj_fwd_position`
→ `mj_fwd_tendon`) instead of rebuilding J from wrap arrays. This is simpler,
handles both fixed and spatial tendons uniformly, and matches MuJoCo's approach
(which also uses the previously-computed `ten_J`). The `mj_solve_sparse_batch`
function already exists and is used for `body_invweight0` — same pattern.

**Before** (current code, `model_init.rs:1056-1115`):
```rust
// --- tendon_invweight0 ---
self.tendon_invweight0 = vec![0.0; self.ntendon];
for t in 0..self.ntendon {
    let adr = self.tendon_adr[t];
    let num = self.tendon_num[t];
    match self.tendon_type[t] {
        TendonType::Fixed => {
            // Sum coef² * dof_invweight0[dof] for each joint in the tendon.
            let mut w = 0.0;
            for wr in adr..(adr + num) {
                if self.wrap_type[wr] == WrapType::Joint {
                    let dof_adr = self.wrap_objid[wr];
                    if dof_adr < self.nv {
                        let coef = self.wrap_prm[wr];
                        w += coef * coef * self.dof_invweight0[dof_adr];
                    }
                }
            }
            self.tendon_invweight0[t] = w.max(MIN_VAL);
        }
        TendonType::Spatial => {
            // Spatial tendons: use average translational invweight of endpoint bodies.
            let mut w = 0.0;
            let mut count = 0;
            for wr in adr..(adr + num) {
                let body_opt = match self.wrap_type[wr] {
                    WrapType::Site => {
                        let sid = self.wrap_objid[wr];
                        if sid < self.site_body.len() {
                            Some(self.site_body[sid])
                        } else {
                            None
                        }
                    }
                    WrapType::Geom => {
                        let gid = self.wrap_objid[wr];
                        if gid < self.geom_body.len() {
                            Some(self.geom_body[gid])
                        } else {
                            None
                        }
                    }
                    _ => None,
                };
                if let Some(bid) = body_opt {
                    if bid > 0 {
                        w += self.body_invweight0[bid][0];
                        count += 1;
                    }
                }
            }
            self.tendon_invweight0[t] = if count > 0 {
                (w / f64::from(count)).max(MIN_VAL)
            } else {
                MIN_VAL
            };
        }
    }
}
```

**After** (new implementation):
```rust
// --- tendon_invweight0 ---
// MuJoCo ref: setInertia() in engine_setconst.c
// For each tendon: invweight0 = J_tendon · M⁻¹ · J_tendon^T
// where J_tendon = data.ten_J[t] (populated by mj_fwd_tendon in mj_fwd_position).
// This uses the FULL quadratic form, capturing off-diagonal M⁻¹ coupling
// between DOFs in serial kinematic chains.
//
// NOTE: The old code used TendonType/WrapType for type-specific branching.
// The new unified algorithm removes that dependency. Remove the
// `use crate::types::{TendonType, WrapType};` import at line 903 if these
// types are no longer used elsewhere in compute_invweight0().
self.tendon_invweight0 = vec![0.0; self.ntendon];
for t in 0..self.ntendon {
    let j_tendon = &data.ten_J[t];

    // Skip zero Jacobians (no DOFs contribute to this tendon)
    let j_norm_sq: f64 = j_tendon.iter().map(|&v| v * v).sum();
    if j_norm_sq < MIN_VAL {
        self.tendon_invweight0[t] = MIN_VAL;
        continue;
    }

    // Build 1-column DMatrix from J_tendon for the batch solver
    let mut rhs = DMatrix::zeros(self.nv, 1);
    for i in 0..self.nv {
        rhs[(i, 0)] = j_tendon[i];
    }

    // Solve M · w = J_tendon → w = M⁻¹ · J_tendon
    mj_solve_sparse_batch(
        &rowadr,
        &rownnz,
        &colind,
        &data.qLD_data,
        &data.qLD_diag_inv,
        &mut rhs,
    );

    // tendon_invweight0 = J_tendon^T · w = J_tendon · M⁻¹ · J_tendon^T
    let mut w = 0.0;
    for i in 0..self.nv {
        w += j_tendon[i] * rhs[(i, 0)];
    }
    self.tendon_invweight0[t] = w.max(MIN_VAL);
}
```

**Import cleanup:** The `use crate::types::{TendonType, WrapType};` at line 903
is only used by the old tendon_invweight0 match branches. After replacing with
the unified algorithm, remove this import (or reduce to just `TendonType` if
still used elsewhere in the function). Failure to remove will trigger `cargo
clippy -- -D warnings`.

**Why this is correct:**
- Uses `data.ten_J[t]` — same source as MuJoCo's `d->ten_J + i*nv`
- `mj_solve_sparse_batch` matches MuJoCo's `mj_solveM` (both use factored LDL)
- Dot product matches MuJoCo's `mju_dot`
- MIN_VAL clamp matches MuJoCo's `mju_max(mjMINVAL, ...)`
- Both fixed and spatial tendons are handled uniformly (no `match tendon_type`)
- `data.ten_J[t]` is already populated by `mj_fwd_position` → `mj_fwd_tendon`
  at line 185 of `forward/position.rs`, which runs at line 927 of `model_init.rs`

**Numerical verification (flag model):**
- Fixed tendon with coefs [1.0, -1.0] on hinge1/hinge2
- J_tendon = [1.0, -1.0, 0, 0, 0, 0, 0, 0] (nv=8)
- M⁻¹ · J includes off-diagonal coupling: M⁻¹[0,1] = -45.15
- Full result: 1²×45.15 + 2×1×(-1)×(-45.15) + 1²×5253.48 = 5388.92
- Current diagonal-only: 1²×45.15 + 1²×5253.48 = 5298.63 (wrong by 90.29)

### S2. Update docstring

**File:** `sim/L0/core/src/types/model_init.rs`, lines 890–901
**Design decision:** The docstring currently documents the incorrect diagonal
formula. Update to reflect the correct full M⁻¹ solve.

**Before:**
```rust
/// - `tendon_invweight0[t]` = Σ(coef² · dof_invweight0\[dof\]) for fixed tendons
```

**After:**
```rust
/// - `tendon_invweight0[t]` = `J_tendon · M⁻¹ · J_tendon^T` (full quadratic form,
///   matching MuJoCo's `setInertia()` in `engine_setconst.c`)
```

### S3. DT-23 verification (no code changes)

**File:** `sim/L0/core/src/constraint/assembly.rs`, lines 369–371 (DOF) and 396–398 (tendon)
**MuJoCo equivalent:** `mj_makeConstraint()` in `engine_core_constraint.c`
**Design decision:** Verify existing code matches MuJoCo. No changes expected.

**Verification checklist:**
1. `assembly.rs:370` reads `model.dof_solref[dof_idx]` — per-DOF ✓
2. `assembly.rs:371` reads `model.dof_solimp[dof_idx]` — per-DOF ✓
3. `assembly.rs:397` reads `model.tendon_solref_fri[t]` — per-tendon ✓
4. `assembly.rs:398` reads `model.tendon_solimp_fri[t]` — per-tendon ✓
5. `builder/joint.rs:160-163` fans out per-DOF in `for i in 0..nv` loop ✓
6. `builder/tendon.rs:43-48` pushes per-tendon ✓
7. `defaults.rs:209-213` cascades `solreffriction` for joints ✓
8. `defaults.rs:641-645` cascades `solreffriction` for tendons ✓
9. Tests T7–T11 in `assembly.rs:1064-1496` verify all pipeline stages ✓

**Conclusion:** DT-23 is already correctly implemented and tested. No code
changes needed. Mark DT-23 as verified.

---

## Acceptance Criteria

### AC1: Flag model tendon_invweight0 *(runtime test — MuJoCo-verified)*

**Given:** `flag_golden_test.xml` model (2-link arm + free ball, nv=8).
Fixed tendon wrapping hinge1 (coef=1.0) and hinge2 (coef=-1.0).
**After:** Model compilation (`compute_invweight0()`)
**Assert:** `model.tendon_invweight0[0]` = 5388.92 ± 1e-2
**Field:** `Model.tendon_invweight0`
**Source:** MuJoCo 3.4.0, `dump_constraint_diagnostic.py` (EGT-1)

### AC2: Single-joint tendon regression *(runtime test — analytically derived)*

**Given:** Model with one hinge joint and one fixed tendon wrapping only that
joint with coef=1.0.
**After:** Model compilation
**Assert:** `tendon_invweight0[0]` = `dof_invweight0[0]` ± 1e-12
(single-joint: full form collapses to diagonal — same as `coef² · M⁻¹[d,d]`)
**Field:** `Model.tendon_invweight0`, `Model.dof_invweight0`

### AC3: Multi-joint off-diagonal coupling *(runtime test — analytically derived)*

**Given:** Model with 2 serial hinge joints and one fixed tendon wrapping both
(coef=1.0 for joint 0, coef=-1.0 for joint 1) — this is the flag model's
tendon configuration.
**After:** Model compilation
**Assert:** `tendon_invweight0[0]` > `Σ(coef² · dof_invweight0[d])` (strictly
greater, because off-diagonal M⁻¹ coupling adds positive terms for serial chains)
**Field:** `Model.tendon_invweight0`

### AC4: Golden flag tests pass *(runtime test — MuJoCo-verified)*

**Given:** All 26 golden flag tests in `integration/golden_flags.rs`
**After:** Remove `#[ignore]` from ALL 26 tests (24 currently-failing + 2 already-passing)
**Assert:** All 26 pass at `TOLERANCE = 1e-8` (`golden_flags.rs:22`, element-wise qacc
comparison over 10 steps)
**Field:** `Data.qacc`
**Note:** 2 tests (`golden_disable_constraint`, `golden_disable_frictionloss`)
already pass when run with `--ignored` but still carry `#[ignore]` annotations.
All 26 `#[ignore]` attributes must be removed for a clean test state. The 24
remaining tests have current divergences in 0.001–0.02 range (per Session 0
diagnostic), all traced to the tendon_invweight0 bug. After the fix, divergence
should drop below 1e-8.

### AC5: Docstring accuracy *(code review)*

**Verify:** `compute_invweight0()` docstring at `model_init.rs:898` describes
the full `J · M⁻¹ · J^T` formula, not the diagonal-only formula.

### AC6: DT-23 routing verification *(code review)*

**Verify:** Assembly code at `assembly.rs:369-371` reads `model.dof_solref[dof_idx]`
per-DOF (not global defaults). Assembly code at `assembly.rs:396-398` reads
`model.tendon_solref_fri[t]` per-tendon. Existing tests T7–T11 cover
multi-DOF fan-out, defaults cascade, and end-to-end routing.

### AC7: Spatial tendon regression guard *(runtime test)*

**Given:** A model with at least one spatial tendon (from existing test suite
or minimal synthetic model)
**After:** Model compilation
**Assert:** `tendon_invweight0[t] > MIN_VAL` (1e-15) for each spatial tendon.
No panics or regressions in spatial tendon model loading/stepping.
**Field:** `Model.tendon_invweight0`

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (flag model invweight0) | T1 | Direct (MuJoCo conformance) |
| AC2 (single-joint regression) | T2 | Edge case |
| AC3 (multi-joint coupling) | T3 | Direct |
| AC4 (golden flag tests) | T4 | Integration (end-to-end cascade) |
| AC5 (docstring) | — | Code review (manual) |
| AC6 (DT-23 routing) | — | Code review (manual) + existing T7–T11 |
| AC7 (spatial tendon regression) | T5 | Regression |

---

## Test Plan

### T1: Flag model tendon_invweight0 conformance → AC1

Extend or modify the existing Phase 13 diagnostic test at
`sim/L0/tests/integration/phase13_diagnostic.rs`.

Load `flag_golden_test.xml`, compile model, assert:
- `model.tendon_invweight0[0]` ≈ 5388.92 ± 1e-2

MuJoCo version: 3.4.0. Expected value from `dump_constraint_diagnostic.py`.

### T2: Single-joint tendon — diagonal equivalence → AC2

Create a minimal model programmatically: one body, one hinge joint,
one fixed tendon wrapping that single joint with coef=1.0.

Assert: `tendon_invweight0[0]` = `dof_invweight0[0]` ± 1e-12.

This verifies the fix doesn't regress the single-joint case (where diagonal
and full form produce identical results).

### T3: Two-joint tendon — off-diagonal coupling → AC3

Create a model with 2 serial hinge joints and a fixed tendon wrapping both
(coef=1.0, coef=-1.0). This is a simplified version of the flag model.

Assert: `tendon_invweight0[0]` > `coef[0]² * dof_invweight0[0] + coef[1]² * dof_invweight0[1]`
(strictly greater due to off-diagonal coupling in serial chain).

### T4: Golden flag tests — end-to-end cascade → AC4

Run all 26 golden flag tests with `#[ignore]` removed:
```
cargo test -p sim-conformance-tests --test integration -- golden
```

All 26 must pass at `TOLERANCE = 1e-8`. This is the primary conformance gate
for Spec A.

### T5: Spatial tendon invweight0 — regression guard → AC7

If a model with spatial tendons exists in the test suite (check
`sim/L0/tests/integration/spatial_tendons.rs`), verify that the model still
loads and simulates correctly after the fix. The old code used a
`body_invweight0` averaging heuristic for spatial tendons; the new code uses
the full `J · M⁻¹ · J^T` solve. Since the full solve is strictly more
correct, values may change but should not cause test failures.

If no existing spatial tendon test exercises `tendon_invweight0`, add a
minimal smoke test: load a spatial tendon model, verify
`tendon_invweight0[t] > MIN_VAL` for each spatial tendon.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| Single-joint tendon | Full form must collapse to diagonal for 1 DOF | T2 | AC2 |
| Multi-joint serial chain | Off-diagonal M⁻¹ coupling is the bug | T3 | AC3 |
| Multi-joint independent bodies | Full form must equal diagonal when M is block-diagonal (no coupling) | Existing `tendon_invweight0_fixed_tendon` in `diagapprox_bodyweight.rs` | AC2 (generalized) |
| Zero-coef tendon wraps | J_tendon all zeros → MIN_VAL result | T2 variant (if all coefs zero) | AC1 |
| Ball/free joint wraps | ten_J[dof_adr] = coef at first DOF component; multi-DOF subblock coupling captured by M⁻¹ solve | No dedicated test (covered implicitly by unified algorithm) | — |
| nv=0 model | Early return in `compute_invweight0` — tendon_invweight0 stays zero | Existing `compute_invweight0` nv=0 guard (line 918-922) | — |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| Existing T7–T11 | DT-23 routing (multi-DOF, defaults, end-to-end) | AC6 verification — these already exist and pass |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| `tendon_invweight0` values for multi-joint fixed tendons | Diagonal-only: `Σ(coef² · M⁻¹[i,i])` | Full: `J · M⁻¹ · J^T` | **Toward MuJoCo** — eliminates 90.29 gap for flag model | All downstream consumers via `efc_diagApprox` | None — transparent change, values become more correct |
| `tendon_invweight0` values for spatial tendons | Average `body_invweight0[b][0]` of endpoint bodies | Full: `J · M⁻¹ · J^T` from `ten_J` | **Toward MuJoCo** — uses correct algorithm | Same as above | None — transparent change |
| Single-joint tendon invweight0 | `coef² · M⁻¹[d,d]` | `coef² · M⁻¹[d,d]` (unchanged) | Already conformant | Nobody | None — mathematically identical for 1 DOF |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/core/src/types/model_init.rs` | Replace tendon_invweight0 computation (lines 1056–1115) + docstring (line 898) + remove unused `TendonType`/`WrapType` import (line 903) | ~-60, +25 (net reduction — unified algorithm is shorter) |
| `sim/L0/tests/integration/phase13_diagnostic.rs` | Add/modify invweight0 assertion (T1) | +10 |
| `sim/L0/tests/integration/golden_flags.rs` | Remove `#[ignore]` from all 26 tests | ~-26 |
| `sim/L0/tests/integration/diagapprox_bodyweight.rs` | Update module docstring (line 6) and test comment (line 233) to reflect full `J·M⁻¹·J^T` formula instead of diagonal-only | ~4 lines modified |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `phase13_diagnostic` | `phase13_diagnostic.rs` | Values change (become correct) | tendon_invweight0 now matches MuJoCo |
| `tendon_invweight0_fixed_tendon` | `diagapprox_bodyweight.rs:209` | **Pass (unchanged)** | Model has 2 independent bodies (siblings of worldbody) → M is block-diagonal → M⁻¹ off-diagonals are zero → full form = diagonal form. The assertion `4.0 * dof_invweight0[dof1] + 9.0 * dof_invweight0[dof2]` still holds. |
| `golden_baseline` | `golden_flags.rs` | Pass (was failing at 0.0021) | Cascade fix |
| `golden_disable_equality` | `golden_flags.rs` | Pass (was failing at 0.0069) | Cascade fix |
| `golden_disable_filterparent` | `golden_flags.rs` | Pass (was failing at 0.020) | Cascade fix |
| `golden_disable_constraint` | `golden_flags.rs` | Pass (already passing) | `#[ignore]` removed |
| `golden_disable_frictionloss` | `golden_flags.rs` | Pass (already passing) | `#[ignore]` removed |
| 21 other golden flag tests | `golden_flags.rs` | Pass (were failing at ~0.002) | Cascade fix |
| All 79 conformance tests | `mujoco_conformance/*.rs` | Pass (unchanged) | No tendon friction rows in conformance models |
| T7–T11 (DT-23 routing) | `assembly.rs` | Pass (unchanged) | routing code not modified |

### Non-Modification Sites

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| `assembly.rs:395` | Reads `model.tendon_invweight0.get(t)` | Consumer — reads the corrected value, no logic change |
| `assembly.rs:535` | Reads `model.tendon_invweight0.get(t)` | Consumer — reads the corrected value, no logic change |
| `impedance.rs:366-368` | Returns `model.tendon_invweight0[id]` | Consumer — dispatch already correct |
| `builder/build.rs:514` | Calls `model.compute_invweight0()` | Caller — already calls the right function |

---

## Execution Order

1. **S1: Fix `tendon_invweight0`** — the primary fix. Replace the diagonal-only
   / heuristic code with the full `J · M⁻¹ · J^T` solve using `data.ten_J[t]`.
   Remove unused `TendonType`/`WrapType` import.
   → Verify: T1 (invweight0 value), T2 (single-joint regression), T3 (multi-joint coupling), T5 (spatial tendon regression)
   → Run: `cargo test -p sim-core -p sim-conformance-tests`

2. **S2: Update docstrings** — fix `model_init.rs:898` and `diagapprox_bodyweight.rs:6,233`
   to reflect the full `J·M⁻¹·J^T` formula. Trivial change, no runtime effect.
   → Verify: AC5 (code review)

3. **S3: DT-23 verification** — code review only, no changes.
   → Verify: AC6 (code review), confirm T7–T11 still pass

4. **Golden flag checkpoint** — remove `#[ignore]` from all 26 tests, run all 26.
   → Verify: T4 (all 26 golden flag tests pass at `TOLERANCE = 1e-8`)
   → Run: `cargo test -p sim-conformance-tests --test integration -- golden`

---

## Out of Scope

- **DT-19** (QCQP cone projection verification) — Spec B scope. Independent
  of assembly-side invweight0. Conformance impact: none for golden flags
  (QCQP is not the root cause per Session 0 diagnostic).
- **DT-128** (PGS early termination) — Spec B scope. Solver-side iteration,
  not assembly. Conformance impact: solver over-iterates but converges correctly.
- **DT-129** (PGS warmstart projection) — Spec B scope. Conformance impact:
  minor convergence behavior, not blocking any tests.
- **Pyramidal diagApprox factor-of-2** — cosmetic mismatch (CF=4.0, MJ=8.0
  for contact rows). No downstream effect — `efc_R` matches after Rpy
  post-processing. Not a conformance bug. Tracked as informational note only.
