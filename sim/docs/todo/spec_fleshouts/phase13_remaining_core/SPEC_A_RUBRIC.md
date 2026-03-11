# Spec A — Constraint Assembly Conformance (DT-39 + DT-23) — Rubric

Grades the SPEC_A spec on 9 criteria. Target: A+ on every criterion
before implementation begins. A+ means "an implementer could build this
without asking a single clarifying question — and the result would produce
numerically identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** P1 (MuJoCo Reference Fidelity)
is the most important criterion — grade it first and hardest.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Scope Adjustment

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| DT-39: `diagApprox` remaining code paths | Root cause is `tendon_invweight0` diagonal-only bug in `compute_invweight0()` — NOT in diagApprox dispatch logic | In scope — fix `tendon_invweight0` to use full `J · M⁻¹ · J^T` |
| DT-23: Per-DOF friction solver params routing | Diagnostic found no divergence in DOF friction row (row 1 matches within tolerance). Assembly code correctly reads `model.dof_solref[dof_idx]` per-DOF. Tests T7–T11 already verify. | In scope — verification only, no code changes expected |
| FILTERPARENT (DT-131) | Same root cause as baseline (~0.020 divergence, not ~51.9). Not an independent bug. | Drop — retracted per Session 0 |

**Final scope:**
1. Fix `tendon_invweight0` computation for fixed and spatial tendons (DT-39)
2. Verify DT-23 per-DOF friction params routing (verification-only)

---

## Empirical Ground Truth

### EGT-1: tendon_invweight0 numeric divergence (MuJoCo 3.4.0)

Source: Phase 13 Session 0 diagnostic (`DIAGNOSTIC_REPORT.md`), MuJoCo 3.4.0
reference dump via `dump_constraint_diagnostic.py`.

Model: `flag_golden_test.xml` — 2-link arm + free ball, nv=8, nq=9.
Tendon: fixed tendon wrapping hinge1 (coef=1.0) and hinge2 (coef=-1.0).

| Value | CortenForge (diagonal) | MuJoCo (full) | Diff |
|-------|------------------------|---------------|------|
| `tendon_invweight0[0]` | 5298.63 | 5388.92 | 90.29 |
| `efc_diagApprox[row 2]` | 5298.63 | 5388.92 | 90.29 |
| `efc_R[row 2]` | 588.74 | 598.77 | 10.03 |
| `efc_D[row 2]` | 1.699e-3 | 1.670e-3 | 2.85e-5 |
| `efc_force[row 2]` | 2.709e-3 | 2.694e-3 | 1.50e-5 |
| `qacc[dof 1]` | 92.359 | 92.379 | 2.03e-2 |

Missing term: `2 × c₀ × c₁ × M⁻¹[0,1]` = `2 × 1 × (−1) × (−45.15)` = **90.29**.

### EGT-2: DT-23 per-DOF friction routing (no divergence)

Source: Session 0 diagnostic.

Row 1 (DOF friction loss for hinge1) matches MuJoCo within tolerance:
- `efc_diagApprox[row 1]`: MATCH
- `efc_R[row 1]`: MATCH
- `efc_D[row 1]`: MATCH
- `efc_solref[row 1]`: correctly reads `model.dof_solref[dof_idx]`

Assembly code at `assembly.rs:369-371` routes `model.dof_solref[dof_idx]`
and `model.dof_solimp[dof_idx]` per-DOF. Tests T7–T11 cover multi-DOF
fan-out, defaults cascade, and end-to-end routing.

### EGT-3: MuJoCo `setInertia()` algorithm for tendon_invweight0

Source: MuJoCo C source `engine_setconst.c`, `setInertia()` function.

For each tendon `i`:
```c
// ten_J already populated by mj_fwdPosition → mj_tendon()
mjtNum* J = d->ten_J + i*m->nv;
mju_copy(res, J, m->nv);
mj_solveM(m, d, res, res, 1);  // res = M⁻¹ · J^T
m->tendon_invweight0[i] = mju_max(mjMINVAL, mju_dot(J, res, m->nv));
```

Key facts:
- Uses `ten_J[t]` (full nv-length Jacobian), NOT just diagonal elements
- `mj_solveM` uses the factored M (LDL) — same as CortenForge's
  `mj_solve_sparse_batch`
- Applied to ALL tendons (fixed and spatial) uniformly
- `ten_J` is available because `mj_fwdPosition` → `mj_tendon()` runs first

### Codebase Context

| File:line | What | Role |
|-----------|------|------|
| `model_init.rs:1056-1115` | `compute_invweight0()` tendon section | **THE BUG** — diagonal-only for fixed, heuristic for spatial |
| `model_init.rs:890-901` | Docstring for `compute_invweight0()` | Documents (incorrectly) the diagonal formula |
| `model_init.rs:926-934` | `mj_fwd_position + mj_crba + mj_factor_sparse` | Creates Data with ten_J populated and M factored |
| `assembly.rs:395` | `model.tendon_invweight0.get(t)` | Consumer: passes bw to `finalize_row!` |
| `assembly.rs:535` | `model.tendon_invweight0.get(t)` | Consumer: tendon limit rows |
| `impedance.rs:335-370` | `compute_diag_approx_bodyweight()` | Consumer: returns `tendon_invweight0[id]` |
| `linalg.rs:290-340` | `mj_solve_sparse_batch()` | Solver: `M · X = B` via LDL |
| `tendon/fixed.rs:25-56` | `mj_fwd_tendon_fixed()` | Populates `data.ten_J[t]` for fixed tendons |
| `tendon/spatial.rs:21+` | `mj_fwd_tendon_spatial()` | Populates `data.ten_J[t]` for spatial tendons |
| `forward/position.rs:185` | `mj_fwd_tendon(model, data)` | Called during `mj_fwd_position()` — ten_J available |

---

## Criteria

> **Criterion priority:** P1 is cardinal. Grade it first and hardest.

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does for tendon_invweight0 computation.

| Grade | Bar |
|-------|-----|
| **A+** | Spec cites `setInertia()` in `engine_setconst.c` with the exact C code for tendon_invweight0 computation: `mju_copy` + `mj_solveM` + `mju_dot`. Explains that `ten_J` is used (not wrap coefficients directly), that `mj_solveM` performs `M⁻¹ · J^T` via factored LDL, and that the result is `J · M⁻¹ · J^T`. Edge cases documented: (1) zero-Jacobian tendon (nv=0 or all coefficients zero) → MIN_VAL, (2) single-joint tendon → collapses to diagonal (same result as current code), (3) spatial tendons use the same algorithm as fixed tendons. Numerical expectation for flag model tendon stated (5388.92 from EGT-1). |
| **A** | MuJoCo behavior described correctly. Minor gap in edge cases (e.g., spatial tendon handling). |
| **B** | Correct at high level but missing the specific C functions or the distinction between ten_J and wrap coefficients. |
| **C** | Algorithm described inaccurately or based on docs rather than C source. |

### P2. Algorithm Completeness

> Every algorithmic step is specified in Rust. No "see MuJoCo source" gaps.

| Grade | Bar |
|-------|-----|
| **A+** | Complete Rust implementation for the fix: build DVector from `data.ten_J[t]`, create 1-column DMatrix, call `mj_solve_sparse_batch`, compute dot product, clamp to MIN_VAL. Both fixed and spatial tendon paths unified into one algorithm. No pseudocode — real types, real function calls. |
| **A** | Algorithm complete. One or two minor details left implicit (e.g., exact zero-check threshold). |
| **B** | Algorithm structure clear but some steps hand-waved or deferred. |
| **C** | Skeleton only. |

### P3. Convention Awareness

> MuJoCo → CortenForge translation explicitly addressed.

| Grade | Bar |
|-------|-----|
| **A+** | Convention table present with porting rules for: (1) `ten_J` layout (Vec<DVector> vs contiguous `d->ten_J + i*nv`), (2) `mj_solveM` → `mj_solve_sparse_batch`, (3) `mjMINVAL` → `MIN_VAL` (1e-15). States that `ten_J` Jacobian layout matches MuJoCo (nv-length, same DOF indexing). No empty porting rule cells. |
| **A** | Major conventions documented. Minor translation left to implementer. |
| **B** | Some conventions noted, others not. |
| **C** | MuJoCo code pasted without adaptation. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable with concrete values.

| Grade | Bar |
|-------|-----|
| **A+** | Primary AC states: "For `flag_golden_test.xml`, `tendon_invweight0[0]` = 5388.92 ± 1e-6 (MuJoCo 3.4.0 verified, per EGT-1)." At least one AC per code path (fixed tendon, single-joint tendon, spatial tendon if applicable). Code-review AC for docstring update. Golden flag test AC: "24 currently-failing golden tests pass at ε=0.01." |
| **A** | ACs testable. Some lack MuJoCo-verified values. |
| **B** | ACs directionally correct but vague on values. |
| **C** | ACs are aspirational statements. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, and regression.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Tests include: (1) flag model tendon_invweight0 vs MuJoCo reference, (2) single-joint tendon regression (same value as diagonal), (3) multi-joint tendon with off-diagonal coupling (the bug case), (4) golden flag test run showing 24→0 failures. Edge case inventory: zero-coef tendon, nv=0 model, single-joint tendon. |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering, and interactions explicitly stated.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order unambiguous: S1 (tendon_invweight0 fix) → S2 (DT-23 verification) → golden flag checkpoint. States dependency on Phase 12 (commit cc20e60). States that `mj_fwd_position` must run before the fix (already does). No cross-spec interactions needed. |
| **A** | Order clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Every file touched, every behavior that changes, every test that might break.

| Grade | Bar |
|-------|-----|
| **A+** | File list: `model_init.rs` (fix + docstring). Behavioral change: `tendon_invweight0` values change toward MuJoCo conformance. Existing test impact: names specific tests that read `tendon_invweight0` and states whether values change. Golden flag tests: 24 expected to un-ignore (named). No other files modified. States that downstream consumers (`assembly.rs`, `impedance.rs`) are read-only — they already correctly use `tendon_invweight0`. |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions. Terminology uniform. Cross-references accurate.

| Grade | Bar |
|-------|-----|
| **A+** | `tendon_invweight0` terminology consistent throughout. AC numbers match traceability matrix. Edge cases in MuJoCo Reference appear in Test Plan. File paths in Specification match Files Affected. |
| **A** | Consistent. One or two minor issues. |
| **B** | Some sections use different names for same concept. |
| **C** | Contradictions between sections. |

### P9. Cascade Verification *(domain-specific)*

> The fix propagates through diagApprox → R → D → efc_force → qacc. The
> spec must verify the entire cascade, not just the tendon_invweight0 value.

| Grade | Bar |
|-------|-----|
| **A+** | Spec traces the full cascade from `tendon_invweight0` → `efc_diagApprox` → `efc_R` → `efc_D` → `efc_force` → `qacc`. At least one AC verifies a downstream value (not just tendon_invweight0 itself). Golden flag test serves as end-to-end cascade verification. States that no changes are needed in the cascade code — only the root input changes. |
| **A** | Cascade described. Downstream verification present but incomplete. |
| **B** | Fix described but cascade not traced. |
| **C** | No cascade analysis. |

> **P1 vs P9 boundary:** P1 grades whether the spec GOT the MuJoCo reference
> right (what setInertia does). P9 grades whether the spec TRACES the effect
> through the CortenForge pipeline (diagApprox → R → qacc).

---

## Rubric Self-Audit

- [x] **Specificity:** Every A+ bar names exact functions (`setInertia`,
      `mj_solve_sparse_batch`), exact files (`model_init.rs`), exact values
      (5388.92), and exact edge cases (zero-Jacobian, single-joint, spatial).

- [x] **Non-overlap:** P1 (MuJoCo reference) vs P9 (cascade) boundary
      explicitly stated. P2 (algorithm) covers the Rust implementation;
      P1 covers the MuJoCo C source understanding. P4 (ACs) covers
      testability; P5 (tests) covers test design.

- [x] **Completeness:** 9 criteria cover: MuJoCo fidelity (P1), algorithm
      (P2), conventions (P3), ACs (P4), tests (P5), ordering (P6), blast
      radius (P7), consistency (P8), cascade (P9). The fix is localized
      to one function — no API design, pipeline integration, or type
      architecture criteria needed.

- [x] **Gradeability:** P1 → MuJoCo Reference + Convention Notes. P2 →
      Specification (S1). P3 → Convention Notes. P4 → Acceptance Criteria.
      P5 → Test Plan + Traceability. P6 → Prerequisites + Execution Order.
      P7 → Risk & Blast Radius. P8 → cross-cutting. P9 → Specification
      cascade discussion + ACs.

- [x] **Conformance primacy:** P1 is tailored with `setInertia()`,
      `engine_setconst.c`, and specific edge cases. P4 and P5 reference
      MuJoCo-verified values from EGT-1.

- [x] **Empirical grounding:** EGT-1 through EGT-3 provide MuJoCo 3.4.0
      verified values and C source analysis. All criterion bars reference
      these.

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors, Convention Notes |
| P2 | Specification (S1, S2) |
| P3 | Convention Notes |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Behavioral Changes, Files Affected, Existing Test Impact) |
| P8 | *Cross-cutting — all sections* |
| P9 | Specification cascade discussion, Acceptance Criteria (downstream values) |

---

## Scorecard

| Criterion | Grade | Evidence |
|-----------|-------|----------|
| P1. MuJoCo Reference Fidelity | **A+** | Cites `setInertia()` in `engine_setconst.c` with C code snippet showing `mju_copy` + `mj_solveM` + `mju_dot`. Cites `mj_diagApprox()` in `engine_core_constraint.c` showing tendon_invweight0 return for FrictionLoss and LimitTendon. 5 edge cases documented (zero-J, single-joint, spatial, nv=0, ball/free). Numerical expectation 5388.92 from MuJoCo 3.4.0 (EGT-1). `mj_makeConstraint` tendon friction section cited for DT-23. |
| P2. Algorithm Completeness | **A+** | Complete Rust before/after in S1 with real types (DMatrix, DVector, mj_solve_sparse_batch). Unified algorithm for fixed+spatial. Zero-J guard, MIN_VAL clamp. Import cleanup for `TendonType`/`WrapType` documented. S3 DT-23 verification is a 9-point checklist with file:line references. No pseudocode. |
| P3. Convention Awareness | **A+** | Convention table with 4 entries (ten_J layout, mj_solveM→mj_solve_sparse_batch, mjMINVAL→MIN_VAL, mju_dot→DVector loop). All porting rules explicit and actionable. No empty cells. |
| P4. Acceptance Criteria Rigor | **A+** | AC1: MuJoCo-verified 5388.92 ± 1e-2. AC2: single-joint diagonal equivalence ± 1e-12. AC3: multi-joint strict inequality. AC4: 26/26 golden flags at `TOLERANCE = 1e-8` (matches `golden_flags.rs:22`). AC5-6: code review ACs. AC7: spatial tendon regression guard. |
| P5. Test Plan Coverage | **A+** | AC→Test traceability matrix with 7 ACs → 5 runtime tests + 2 code reviews. T1-T4 cover MuJoCo conformance, edge case, coupling, and end-to-end. T5 covers spatial tendon regression. Edge case inventory with 6 cases (including ball/free wraps and independent bodies). Supplementary tests reference existing T7-T11 for DT-23. |
| P6. Dependency Clarity | **A+** | Execution order: S1→S2→S3→golden. Prerequisites with commit hashes (001a7fd, cc20e60). Independence from Spec B stated with shared-file analysis ("No shared files"). |
| P7. Blast Radius & Risk | **A+** | 4 files affected with line estimates (including `diagapprox_bodyweight.rs` docstring update). Behavioral changes table with conformance direction. 30+ specific tests named with expected impact (including `tendon_invweight0_fixed_tendon` and both already-passing golden tests). Non-modification sites table (4 entries) documents why assembly.rs, impedance.rs, builder/build.rs are NOT modified. |
| P8. Internal Consistency | **A+** | "tendon_invweight0" used consistently (no "tendon_invweight" or "invweight" shorthand). AC numbers match traceability matrix (AC1-7 → T1-5 + code review). All 5 edge cases from MuJoCo Reference appear in Edge Case Inventory (6 entries total including independent bodies and ball/free wraps). File paths in S1 (`model_init.rs:1056-1115`) match Files Affected table (4 files). |
| P9. Cascade Verification | **A+** | Problem Statement traces full cascade: invweight0 → diagApprox → R → D → force → qacc with specific numeric values at each stage (EGT-1). AC4 (golden flags) provides end-to-end cascade verification. S1 "Why this is correct" section explains that downstream code (assembly, impedance) is read-only — only the root input changes. |

**Overall: A+ (Rev 1) — 7 gaps found in stress test, all resolved**

Convergence: Rev 0 (0 gaps, self-graded A+) → Stress test (7 gaps found) → Rev 1 (7 gaps resolved, A+)

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| G1 | P2 | Unused `TendonType`/`WrapType` import after removing match branch. Clippy `--deny warnings` would fail. | Stress test | Added import cleanup note in S1 "After" code and explicit instruction in S1 body. Added to Files Affected. | Rev 1 |
| G2 | P4 | AC4 tolerance said "ε=0.01" but golden_flags.rs uses `TOLERANCE = 1e-8` (4 orders of magnitude tighter). | Stress test | Fixed AC4 to state `TOLERANCE = 1e-8` matching `golden_flags.rs:22`. | Rev 1 |
| G3 | P4 | AC4 said "Remove `#[ignore]` from 24 tests" but all 26 are `#[ignore]`d (2 already pass but still carry annotations). | Stress test | Fixed AC4 to say "all 26 tests". Added both already-passing tests to Existing Test Impact. | Rev 1 |
| G4 | P7 | `diagapprox_bodyweight.rs:209 tendon_invweight0_fixed_tendon()` not listed in Existing Test Impact despite asserting the diagonal formula. | Stress test | Added to Existing Test Impact with "Pass (unchanged)" — independent bodies have block-diagonal M, so full form = diagonal form. | Rev 1 |
| G5 | P7 | `diagapprox_bodyweight.rs` docstring (line 6) and test comment (line 233) document the wrong formula. Not in Files Affected. | Stress test | Added `diagapprox_bodyweight.rs` to Files Affected with docstring/comment update. Added to S2 execution order. | Rev 1 |
| G6 | P8 | MuJoCo Reference has 5 edge cases; Edge Case Inventory had 4 (missing "ball/free joint wraps"). | Stress test | Added "ball/free joint wraps" and "multi-joint independent bodies" to Edge Case Inventory (now 6 entries). | Rev 1 |
| G7 | P5 | No spatial tendon test. Fix changes spatial tendon behavior (heuristic → full M⁻¹) but no test covered it. | Stress test | Added AC7 (spatial tendon regression guard), T5 (spatial tendon test), updated traceability matrix. | Rev 1 |
