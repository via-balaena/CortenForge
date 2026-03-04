# Spec B — QCQP Cone Projection: Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_B.md`
**Implementation session(s):** Session 10 (from SESSION_PLAN.md)
**Reviewer:** AI agent
**Date:** 2026-03-04

---

## Purpose

This review verifies that the implementation matches the approved spec,
surfaces weakly implemented items that should be fixed now, and ensures
deferred work is properly tracked so nothing falls through the cracks.

**Five questions this review answers:**

1. **Closed?** Are the conformance gaps from the spec's Key Behaviors
   table actually closed?
2. **Faithful?** Does the implementation match the spec — every section,
   every AC, every convention note, every planned test?
3. **Predicted?** Did the blast radius match the spec's predictions, or
   were there surprises?
4. **Solid?** Are there any items that technically "work" but are weakly
   implemented (hacks, TODOs, incomplete edge cases, loose tolerances)?
5. **Tracked?** Is every piece of deferred or out-of-scope work tracked
   in `sim/docs/todo/` or `sim/docs/ROADMAP_V1.md`?

---

## 1. Key Behaviors Gap Closure

The spec's Key Behaviors table has a "CortenForge (current)" column showing
the conformance gap *before* implementation. Verify each gap is now closed.

| Behavior | MuJoCo (from spec) | CortenForge Before | CortenForge After | Gap Closed? |
|----------|-------------------|--------------------|-------------------|-------------|
| PGS elliptic projection | Two-phase: ray update along current force direction + QCQP on friction subblock (`mj_solPGS` in `engine_solver.c`) | Per-row scalar GS + simple scaling (`project_elliptic_cone` at pgs.rs:148) — fundamentally different | | |
| QCQP λ update | No clamping: `λ += delta` (`mju_QCQP2` in `engine_util_solve.c`) | Clamps: `lam = lam.max(0.0)` (noslip.rs:137, 248) — non-conformant | | |
| QCQP convergence | `val < 1e-10` (one-sided) | `phi.abs() < 1e-10` (two-sided, noslip.rs:123, 234) — non-conformant | | |
| QCQP SPD check | `det < 1e-10` → return zero | `det.abs() < MJ_MINVAL` (1e-15, noslip.rs:113, 216) — different threshold and abs treatment | | |
| QCQP step guard | `delta < 1e-10` → break | `dphi.abs() < MJ_MINVAL` (noslip.rs:133, 244) — different check | | |
| QCQP control flow | Loop values reused directly, no final solve | Final solve outside loop + degenerate rescaling fallback (noslip.rs:140-165) — extra code not in MuJoCo | | |
| QCQP degenerate | Return zero vector | Simple rescaling fallback (noslip.rs:146-152) — non-conformant | | |
| Noslip elliptic bias | `bc = res - Ac * oldforce` | `f_unc` from scalar GS update (noslip.rs:528-533) — different formulation | | |
| Noslip condim=6 | `mju_QCQP()` (N-dim Cholesky) | Simple rescaling fallback (noslip.rs:586-603) — non-conformant | | |
| Noslip pyramidal cost threshold | `1e-10` (costChange) | `MJ_MINVAL` = 1e-15 (noslip.rs:689) — over-aggressive revert | | |
| Noslip iter-0 cost correction | `0.5*f²*R` added to improvement | Missing (noslip.rs:478) — affects convergence | | |
| PGS cost guard (elliptic) | AR subblock (`Athis`) with `costChange()` | Full-matrix reconstruction (`pgs_cost_change` at pgs.rs:220-255) — numerically equivalent but different approach | | |
| CG/Newton elliptic | Primal zone-based classifier (`PrimalUpdateConstraint`) | `classify_constraint_states` (pgs.rs:272-488) — correct, no change needed | | |

**Unclosed gaps:**
{To be filled during review execution.}

---

## 2. Spec Section Compliance

### S1. QCQP Utility Functions

**Grade:**

**Spec says:**
Create new `solver/qcqp.rs` module with `qcqp2`, `qcqp3`, `qcqp_nd` functions
matching `mju_QCQP2/3/N` in `engine_util_solve.c` line-for-line. Constants
`QCQP_TOL = 1e-10`, `QCQP_MAX_ITER = 20`. Functions do NOT perform ellipsoidal
rescaling — caller handles that. 2×2 closed-form inverse, 3×3 cofactor inverse,
N×N Cholesky. No λ clamping, one-sided convergence check, `det < 1e-10` SPD
check, `delta < 1e-10` step guard. Register module in `solver/mod.rs`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. PGS Two-Phase Elliptic Projection

**Grade:**

**Spec says:**
Replace PGS elliptic branch (pgs.rs:131-160) with two-phase algorithm: Phase 1
ray update (scale along current force direction, guard normal ≥ 0), Phase 2
friction QCQP (build Ac from AR subblock, build bc with normal force change
cross-coupling, dispatch `qcqp2/3/nd` by fdim, ellipsoidal rescale when active).
Cost guard uses AR subblock directly with `costChange` formula (threshold 1e-10,
revert if cost increases). Remove `pgs_cost_change` function. Remove
`project_elliptic_cone` import.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. Noslip Elliptic QCQP Rewrite

**Grade:**

**Spec says:**
Rewrite noslip Phase B elliptic friction path. Replace `f_unc`-based bias with
`bc = res - Ac * old` formulation using unregularized Delassus. Dispatch to
`qcqp2/3/nd` by `group_len` (2→qcqp2, 3→qcqp3, else→qcqp_nd). Ellipsoidal
rescale when active. Remove old `noslip_qcqp2` and `noslip_qcqp3` functions.
Add `#[allow(dead_code)]` to `project_elliptic_cone` if clippy flags it.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. Noslip Convergence and Cost Fixes

**Grade:**

**Spec says:**
Fix 1: Change pyramidal cost rollback threshold from `MJ_MINVAL` (1e-15) to
`1e-10` (matching `costChange` in MuJoCo). Fix 2: Add iter-0 cost correction
`improvement += 0.5 * f[fi]² * efc_R[row]` at start of noslip iteration loop
for all noslip rows when `iter_num == 0`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. Verification and Cleanup

**Grade:**

**Spec says:**
5a: Verify PGS scalar projection (pgs.rs:161-207) matches MuJoCo — equality no
projection, friction loss box clamp, limits/frictionless/pyramidal unilateral
max(0,f), scalar cost guard with 1e-10 threshold. No changes needed.
5b: Verify CG/Newton primal classifier (pgs.rs:272-488) — three zones (Top,
Bottom, Middle) correct. No changes needed.
5c: Verify cone Hessian (hessian.rs) — Newton-only, orthogonal to QCQP. No
changes needed.
5d: Module registration — add `pub mod qcqp;` to `solver/mod.rs`.
5e: Remove dead code — `pgs_cost_change` from pgs.rs, `noslip_qcqp2/3` from
noslip.rs, `project_elliptic_cone` import from pgs.rs.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | QCQP2 matches MuJoCo for identity A, b=[-2,-2], d=[1,1], r=1 → [1/√2, 1/√2], active=true | T1, T12, T13, T18 | | |
| AC2 | QCQP3 matches MuJoCo for identity A, b=[-3,-4,0], d=[1,1,1], r=1 → [0.6, 0.8, 0.0], active=true | T2, T22, T23 | | |
| AC3 | QCQP_ND 5D: identity A, b=[-1;5], d=[0.5;5], r=0.5 → all ≈ √5/20, active=true | T3, T24 | | |
| AC4 | PGS elliptic contact: sphere on plane, 500 steps, stable resting forces, cone constraint satisfied | T4 | | |
| AC5 | PGS ray update preserves non-negative normal force (cold start + adversarial warmstart) | T5, T17 | | |
| AC6 | Noslip QCQP bias construction: sphere on 15° tilted plane, friction forces satisfy cone, correct direction | T6 | | |
| AC7 | Noslip condim=6 dispatches to `qcqp_nd` for group_len > 3, no simple rescaling fallback (code review) | — (code review) | | |
| AC8 | Noslip pyramidal cost threshold is 1e-10, not MJ_MINVAL; iter-0 correction present (code review + runtime) | T7, T14, T25, code review | | |
| AC9 | CG/Newton elliptic behavior unchanged — bitwise identical to pre-Spec-B baseline | T8, T16 | | |
| AC10 | No regression on existing solver tests | T9 | | |
| AC11 | QCQP degenerate input (zero matrix) returns ([0,0], false), no rescaling fallback | T10, T21 | | |
| AC12 | QCQP unconstrained (inside cone): active=false, result = unconstrained optimum | T11, T20 | | |
| AC13 | Noslip iter-0 cost correction: `0.5*f²*R` added at iter 0 for all noslip rows (code review) | — (code review) | | |
| AC14 | `project_elliptic_cone` no longer called from PGS; function retained with dead_code annotation | T15 | | |
| AC15 | PGS elliptic forces match MuJoCo reference output within 1e-8 per element | T19 | | |

**Missing or failing ACs:**
{To be filled during review execution.}

---

## 4. Test Plan Completeness

> **Test numbering note:** The spec uses T1-T27. Implementation test function
> names should follow the pattern `t{NN}_{description}` in the test file.
> Cross-reference mapping to be documented here during review execution.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | QCQP2 conformance — constrained case, A=I, b=[-2,-2], d=[1,1], r=1 → [1/√2, 1/√2] | | | |
| T2 | QCQP3 conformance — constrained case, A=I₃, b=[-3,-4,0], d=[1,1,1], r=1 → [0.6, 0.8, 0] | | | |
| T3 | QCQP_ND 5D — A=I₅, b=[-1;5], d=[0.5;5], r=0.5 → all ≈ √5/20 | | | |
| T4 | PGS elliptic — sphere at rest, 500 steps, stable forces | | | |
| T5 | PGS ray update — non-negative normal (cold start + adversarial warmstart) | | | |
| T6 | Noslip elliptic on tilted plane — correct bias, cone satisfied | | | |
| T7 | Noslip pyramidal finite results — finite forces, no NaN/Inf | | | |
| T8 | Newton/CG elliptic regression — unchanged behavior | | | |
| T9 | Full regression suite — all existing tests pass | | | |
| T10 | QCQP degenerate input — zero matrix → ([0,0], false) | | | |
| T11 | QCQP unconstrained (inside cone) — active=false, result = [0.1, 0.1] | | | |
| T12 | QCQP2 asymmetric mu — d=[1, 0.001], degenerate cone axis | | | |
| T13 | QCQP2 iteration exhaustion — ill-conditioned A, slow convergence | | | |
| T14 | Noslip pyramidal near-degenerate 2×2 block — finite forces | | | |
| T15 | Negative test — pyramidal contacts bypass QCQP | | | |
| T16 | Negative test — CG/Newton use primal classifier, not QCQP | | | |
| T17 | PGS ray update with pure-normal force — v=[1,0,0] | | | |
| T18 | QCQP2 delta step guard — early exit, very stiff A | | | |
| T19 | MuJoCo direct conformance — PGS elliptic efc_force vs MuJoCo reference | | | |
| T20 | QCQP2 very large friction coefficients — inside cone | | | |
| T21 | QCQP2 zero mu in one direction — degenerate, returns zero | | | |
| T22 | QCQP3 degenerate input — zero matrix → ([0,0,0], false) | | | |
| T23 | QCQP3 inside cone — unconstrained, active=false | | | |
| T24 | QCQP_ND degenerate — 5D zero matrix → ([0;5], false) | | | |
| T25 | Noslip pyramidal threshold discrimination — 5e-12 cost accepted under 1e-10 | | | |

### Supplementary Tests

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| T26 | QCQP2 random stress — multiple random inputs, verify cone satisfaction | | |
| T27 | Noslip cone regression — existing `test_noslip_elliptic_cone_satisfied` still passes | | |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| Zero normal force (friction zeroed) | PGS Phase 1 scalar path + Phase 2 clear | | | |
| Degenerate AR subblock (`det < 1e-10`) | QCQP returns zero, no rescaling | | | |
| Unconstrained inside cone (no projection) | λ stays 0, active=false | | | |
| Condim=1 frictionless | Never enters elliptic branch, scalar max(0,f) | | | |
| Condim=3 (2-DOF QCQP) | Most common elliptic case | | | |
| Condim=4 (3-DOF QCQP) | Torsional friction | | | |
| Condim=6 (5-DOF QCQP_ND) | Rolling friction, requires N-dim solver | | | |
| PGS ray update with pure-normal force | v = [fn, 0, 0], tests ray direction | | | |
| PGS ray update denom < MINVAL | Skip ray update, keep old forces | | | |
| QCQP Newton step guard (delta < 1e-10) | Early exit before max iterations | | | |
| QCQP Newton max iterations (20) | Convergence check after all iters | | | |
| Noslip pyramidal cost threshold | Revert at 1e-10, not 1e-15 | | | |
| Noslip pyramidal near-degenerate 2×2 | Graceful handling when det ≈ 0 | | | |
| Noslip pyramidal threshold discrimination | Verify 1e-10 vs 1e-15 has effect | | | |
| QCQP3 degenerate (zero matrix) | 3D solver graceful failure | | | |
| QCQP3 inside cone | 3D solver unconstrained path | | | |
| QCQP_ND degenerate (5D zero matrix) | N-dim solver graceful failure | | | |
| Noslip iter-0 correction | Regularization cost accounted | | | |
| mu ≈ 0 for one direction (asymmetric) | Degenerate cone dimension | | | |
| mu = 0 exactly for one direction | Division by zero in rescale | | | |
| Very large friction coefficient | Cone is very wide, solution inside | | | |
| MuJoCo direct conformance | Gold-standard numeric match | | | |
| Pyramidal contacts bypass QCQP | PGS scalar path only, no QCQP | | | |
| CG/Newton bypass QCQP | Primal classifier, not dual | | | |

**Missing tests:**
{To be filled during review execution.}

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| PGS elliptic force values change from per-row scalar GS + simple scaling to two-phase ray+QCQP (toward MuJoCo) | | |
| Noslip elliptic forces change from QCQP with `f_unc` bias to `bc=res-Ac*old` + condim=6 Cholesky (toward MuJoCo) | | |
| Noslip pyramidal cost threshold changes from 1e-15 to 1e-10 (toward MuJoCo) — less aggressive revert | | |
| Noslip convergence timing changes: iter-0 correction adds 0.5*f²*R (toward MuJoCo) | | |
| CG/Newton elliptic unchanged (primal zone classifier, not QCQP) | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `core/src/constraint/solver/qcqp.rs` (new, ~250 lines) | | |
| `core/src/constraint/solver/mod.rs` (+1 line) | | |
| `core/src/constraint/solver/pgs.rs` (~90 modified, -40 removed) | | |
| `core/src/constraint/solver/noslip.rs` (~80 modified, -210 removed, +10 added) | | |
| `L0/tests/integration/` (new test file or additions, +300 lines) | | |

{Unexpected files changed — to be filled during review execution:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| `test_s32_elliptic_no_regression` (unified_solvers.rs:1702) | Pass (unchanged — Newton, not PGS) | | |
| `test_pgs_contact_force_quantitative` (newton_solver.rs:1655) | Pass (unchanged — pyramidal, not elliptic) | | |
| `test_pgs_cost_guard_dual_cost_nonpositive` (unified_solvers.rs:286) | Pass (unchanged — scalar PGS cost guard) | | |
| `test_pgs_cost_guard_with_friction_loss_and_contacts` (unified_solvers.rs:362) | Pass (unchanged — friction loss + contacts, scalar PGS) | | |
| `test_noslip_elliptic_cone_satisfied` (noslip.rs:682) | Pass (assertion loose: s ≤ \|fn\| + 1e-6) | | |
| `test_pgs_noslip_reduces_slip` (noslip.rs:23) | Pass (assertion: slip reduces, not exact values) | | |
| `test_noslip_preserves_normal_forces` (noslip.rs:220) | Pass (unchanged — noslip only modifies friction rows) | | |
| `test_noslip_friction_loss_clamping` (noslip.rs:282) | Pass (unchanged — friction loss rows, not elliptic) | | |
| `test_noslip_pyramidal_reduces_slip` (noslip.rs:359) | May change numerically (threshold 1e-15→1e-10 changes convergence) | | |
| `test_noslip_pyramidal_forces_nonnegative` (noslip.rs:407) | Pass (unchanged — non-negativity preserved) | | |
| `test_noslip_pyramidal_finite_results_all_solvers` (noslip.rs:447) | Pass (unchanged — finiteness preserved) | | |
| `test_cg_friction_cone` (cg_solver.rs:221) | Pass (unchanged — CG uses primal, not QCQP) | | |
| `test_newton_noslip_regression` (noslip.rs:118) | May change numerically (noslip QCQP changes) | | |
| `test_noslip_processes_all_contacts_regardless_of_state` (noslip.rs:792) | Pass (structural test, not value-dependent) | | |
| `test_s31_solreffriction_affects_newton_and_pgs` (unified_solvers.rs:970) | Pass (unchanged — tests solreffriction, not cone projection) | | |

**Unexpected regressions:**
{To be filled during review execution.}

### Non-Modification Sites Verification

The spec explicitly identifies 6 sites that must NOT be modified. Verify no
unintended changes were made.

| File:line | What it does | Why NOT modified | Changed? |
|-----------|-------------|-----------------|----------|
| `pgs.rs:272-488` | `classify_constraint_states` (primal classifier) | CG/Newton only, not QCQP. Already correct. | |
| `hessian.rs` | Cone Hessian for Newton | Orthogonal to QCQP projection. Already correct. | |
| `primal.rs` | Primal constraint update | Shared by CG/Newton, not PGS/noslip. | |
| `assembly.rs` | Constraint row assembly | Different pipeline stage. Not touched by Spec B. | |
| `noslip.rs:621-706` | Phase C pyramidal 2×2 block solve | Algorithm unchanged; only cost threshold fixed (line 689). | |
| `noslip.rs:481-496` | Phase A friction loss | Scalar PGS + box clamp. Already correct. | |

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| `contact.friction[5]` → `efc_mu[i]: [f64; 5]` | Direct port — same 5-element layout | | |
| `contact.dim` → `efc_dim[i]: usize` (per-row, same for all rows of contact) | Direct port — index with `data.efc_dim[i]` | | |
| `efc_AR` flat nefc×nefc row-major → `DMatrix<f64>` (nalgebra, column-major internally) | Use `ar[(i,j)]` — nalgebra handles storage order | | |
| `force+i` pointer arithmetic → `data.efc_force` `DVector` slice | Use `.as_slice()` / `.as_mut_slice()` for contiguous access | | |
| `contact.mu` → `efc_mu[i][0..5]` (same values, set in assembly) | Verify `efc_mu` populated with same friction values as MuJoCo | | |
| `flg_subR` flag: PGS regularized, noslip unregularized | PGS uses `ar` (includes R). Noslip uses `a_sub` (excludes R). | | |
| `extractBlock` → `ar.slice((i,i),(dim,dim))` from nalgebra | Use nalgebra `slice` method | | |
| `Ac[j*(dim-1)+k]` row-major flat → `[[f64; N]; N]` for N=2,3; `Vec<f64>` flat row-major for N-dim | For qcqp2/3: use `[[f64; N]; N]`. For qcqp_nd: use flat `&[f64]` row-major. | | |
| `ARinv[i]` = `1.0 / AR[i,i]` precomputed → `ar_diag_inv[i]` | Direct port — same computation | | |
| `efc_R[i]` per-constraint regularization → `data.efc_R: Vec<f64>` | Used in noslip iter-0 correction (S4) | | |

---

## 7. Weak Implementation Inventory

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|
| | | | | |

{To be filled during review execution. Methodology: grep for TODO, FIXME, HACK,
todo!, unimplemented! in all new/modified files. Review all new production code
for unwrap(), loose tolerances, and algorithm deviations from spec.}

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| DT-18 (Zero-friction condim downgrade) | Out of Scope, bullet 1 | | | |
| PGS early termination / improvement tracking (DT-126) | Out of Scope, bullet 2 | | | |
| `solreffriction` on contacts | Out of Scope, bullet 3 | | | |
| Spec A items (DT-23, DT-33) | Out of Scope, bullet 4 | | | |
| DT-25 verification | Out of Scope, bullet 5 | | | |
| Warmstart improvements | Out of Scope, bullet 6 | | | |
| Dense AR matrix optimization | Out of Scope, bullet 7 | | | |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| | | | | |

{To be filled during review execution.}

### Discovered During Review

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| | | | | |

{To be filled during review execution.}

### Spec Gaps Found During Implementation

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|
| | | | |

{To be filled during review execution.}

---

## 9. Test Coverage Summary

**Domain test results:**
```
{To be filled during review execution — per-crate breakdown:}
sim-core:              N passed, 0 failed, M ignored
sim-mjcf:              N passed, 0 failed, M ignored
sim-constraint:        N passed, 0 failed, M ignored
sim-tendon:            N passed, 0 failed, M ignored
sim-conformance-tests: N passed, 0 failed, M ignored
Total:                 N passed, 0 failed, M ignored
```

**New tests added:** {count}
**Tests modified:** {count}
**Pre-existing test regressions:** {count — should be 0}

**Clippy:** {clean / N warnings}
**Fmt:** {clean / issues}

---

## 10. Review Verdict

| Category | Section | Status |
|----------|---------|--------|
| Key behaviors gap closure | 1 | |
| Spec section compliance | 2 | |
| Acceptance criteria | 3 | |
| Test plan completeness | 4 | |
| Blast radius accuracy | 5 | |
| Convention fidelity | 6 | |
| Weak items | 7 | |
| Deferred work tracking | 8 | |
| Test health | 9 | |

**Overall:** {Ship / Ship after fixes / Needs rework}

**Items fixed during review:**

1. ...

**Items to fix before shipping:**
1. ...

**Items tracked for future work:**
1. ...
