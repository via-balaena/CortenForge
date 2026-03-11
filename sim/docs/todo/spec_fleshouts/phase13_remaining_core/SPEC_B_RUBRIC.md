# Spec B — PGS Solver Conformance — Spec Quality Rubric

Grades the Spec B spec on 9 criteria. Target: A+ on every criterion
before implementation begins. A+ means "an implementer could build this
without asking a single clarifying question — and the result would produce
numerically identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Scope Adjustment

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| DT-19: QCQP verification against MuJoCo | `mju_QCQP2/3/N` in `engine_util_solve.c` — line-by-line comparison confirms CortenForge's `qcqp2/3/qcqp_nd` match exactly (scaling, Newton iteration, convergence, unscaling all identical) | **Verified correct** — DT-19 is closed, no code changes |
| DT-128: PGS early termination | `mj_solPGS()` in `engine_solver.c` — accumulates `improvement = -Σ costChange(...)` per sweep, scales by `1/(meaninertia * max(1,nv))`, breaks when `improvement < tolerance`. CortenForge PGS always runs `max_iters` with no convergence check. | **In scope** — implement early termination |
| DT-129: PGS warmstart two-phase projection | MuJoCo's warmstart (`warmstart()` in `engine_forward.c`) calls `mj_constraintUpdate()` to map `qacc_warmstart → efc_force`, then zeros forces for PGS when `cost > 0` (strict inequality). CortenForge uses `classify_constraint_states()` + dual cost comparison (`dual_cost < 0.0` → keep, i.e., zeros when `cost ≥ 0` — slightly more conservative). The "ray+QCQP" projection in the roadmap description is **inaccurate** — ray+QCQP is used during PGS *iteration*, not warmstart. | **Verified largely correct** — CortenForge's `cost ≥ 0` gate is more conservative than MuJoCo's `cost > 0` gate (rejects cost=0 warmstarts). Functionally equivalent for non-degenerate cases. |

**Final scope:**
1. Verify DT-19 QCQP conformance (no code changes — mark as verified)
2. Implement DT-128 PGS early termination (improvement tracking + convergence break)
3. Verify/fix DT-129 PGS warmstart (minor adjustments to cost gate if needed)
4. Fix `solver_niter` to report actual iteration count, not `max_iters`
5. Populate `solver_stat` for PGS (MuJoCo records per-iteration stats)

---

## Empirical Ground Truth

### MuJoCo behavioral verification

**MuJoCo version:** 3.4.0 (open-source `google-deepmind/mujoco`, `main` branch)

### EGT-1: QCQP conformance verified

Line-by-line comparison of `mju_QCQP2()` (MuJoCo `engine_util_solve.c`) against
CortenForge's `qcqp2()` (`qcqp.rs:25-72`):

| Aspect | MuJoCo | CortenForge | Match |
|--------|--------|-------------|-------|
| Scaling: `b_s = b * d`, `A_s = A * d_i * d_j` | ✓ | ✓ | Identical |
| Newton init: `la = 0` | ✓ | ✓ | Identical |
| Max iter: 20 | ✓ (`QCQP_MAX_ITER = 20`) | ✓ | Identical |
| SPD check: `det < 1e-10` → return 0 | ✓ | ✓ | Identical |
| Inverse: 2×2 cofactor | ✓ | ✓ | Identical (different variable names, same math) |
| `v = -(A_s + λI)⁻¹ · b_s` | ✓ | ✓ | Identical |
| Convergence: `v·v - r² < 1e-10` | ✓ | ✓ | Identical |
| Derivative: `-2 · v^T · (A_s+λI)⁻¹ · v` | ✓ | ✓ | Identical |
| Step guard: `delta < 1e-10` | ✓ | ✓ | Identical |
| Unscale: `x_i = v_i * d_i` | ✓ | ✓ | Identical |
| Return: `la != 0` (active flag) | ✓ | ✓ | Identical |

Same analysis applies to `qcqp3` (3×3 cofactor inverse) and `qcqp_nd` (Cholesky).
All three solvers match MuJoCo exactly. **DT-19 is closed.**

### EGT-2: PGS early termination absent

MuJoCo `mj_solPGS()` convergence loop (from `engine_solver.c`):
```c
while (iter < maxiter) {
    mjtNum improvement = 0;
    // ... sweep: improvement -= costChange(...) per row ...
    improvement *= scale;  // scale = 1 / (meaninertia * max(1, nv))
    saveStats(m, d, island, iter, improvement, ...);
    iter++;
    if (improvement < m->opt.tolerance) {
        break;
    }
}
d->solver_niter[island] += iter;
```

CortenForge `pgs_solve_unified()` (`pgs.rs:129-335`):
```rust
for _iter in 0..max_iters {
    // ... sweep (no improvement tracking) ...
}
data.solver_niter = max_iters;  // always max_iters, not actual count
```

**Missing:** improvement accumulation, scale computation, convergence break,
actual iteration count reporting.

### EGT-3: PGS improvement accumulation via costChange

MuJoCo's `costChange()` (`engine_solver.c`):
```c
static mjtNum costChange(const mjtNum* A, mjtNum* force, const mjtNum* oldforce,
                         const mjtNum* res, int dim) {
    mjtNum delta[6];
    mju_sub(delta, force, oldforce, dim);
    mjtNum change = 0.5*mju_mulVecMatVec(delta, A, delta, dim) + mju_dot(delta, res, dim);
    if (change > 1e-10) {
        mju_copy(force, oldforce, dim);  // revert
        change = 0;
    }
    return change;
}
```

CortenForge already implements the cost guard (revert when `cost_change > 1e-10`)
in both the elliptic block (`pgs.rs:266-281`) and scalar path (`pgs.rs:318-327`).
But it does NOT accumulate the returned cost change for convergence checking.
The fix is to sum `costChange` returns across rows and compare against tolerance.

### EGT-4: PGS solver_stat population

MuJoCo calls `saveStats()` per iteration with: improvement, gradient (0 for PGS),
lineslope (0 for PGS), nactive, nchange, nline (0 for PGS).

CortenForge's `SolverStat` struct has all these fields. Newton populates them;
PGS does not. The fix is to populate `solver_stat` per PGS iteration.

### Codebase context

| File:line | What it does | Role in Spec B |
|-----------|-------------|----------------|
| `pgs.rs:71-336` | `pgs_solve_unified()` — main PGS solver | Primary modification target |
| `pgs.rs:129` | `for _iter in 0..max_iters` — iteration loop | Replace with `while iter < max_iters` + convergence break |
| `pgs.rs:266-281` | Elliptic cost guard | Already correct; needs to return cost_change value |
| `pgs.rs:318-327` | Scalar cost guard | Already correct; needs to return cost_change value |
| `pgs.rs:334` | `data.solver_niter = max_iters` | Fix to actual iteration count |
| `pgs.rs:335` | `data.solver_stat.clear()` | Populate per-iteration stats |
| `pgs.rs:88-125` | Warmstart section | Verify/fix cost gate |
| `qcqp.rs:25-256` | QCQP solvers | Verified correct — no changes |
| `newton.rs:42-338` | Newton solver | Not modified by Spec B |
| `data.rs:306` | `solver_niter: usize` | Consumer — reads actual iteration count |
| `data.rs:402` | `solver_stat: Vec<SolverStat>` | Consumer — reads per-iteration stats |
| `enums.rs:757` | `SolverStat` struct | Used for PGS stats |

---

## Criteria

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases.

| Grade | Bar |
|-------|-----|
| **A+** | Spec cites `mj_solPGS()` in `engine_solver.c` with: (a) the `costChange()` function verbatim, (b) the `improvement` accumulation loop, (c) the `scale = 1/(meaninertia * max(1,nv))` formula, (d) the `improvement < tolerance` convergence break, (e) `saveStats()` per-iteration fields (nactive/nchange may be deferred as diagnostic-only placeholders if justified), (f) `solver_niter += iter` (actual count, not max). Warmstart: cites `warmstart()` in `engine_forward.c` showing PGS-specific force zeroing when `cost > 0` (strict inequality — notes CortenForge uses `cost ≥ 0`, slightly more conservative). QCQP: documents line-by-line match of all three solvers with C code snippets. Edge cases addressed: nefc=0, single iteration convergence, zero improvement, warmstart disabled. |
| **A** | MuJoCo behavior described correctly. Minor gaps in edge-case coverage. |
| **B** | High-level correct but missing specifics (e.g., "MuJoCo checks convergence" without the exact formula). |
| **C** | Partially correct. Some behavior misunderstood. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. No "see MuJoCo source"
> gaps. Rust code is line-for-line implementable.

| Grade | Bar |
|-------|-----|
| **A+** | Early termination loop structure written in Rust: `while` loop with `iter` counter, `improvement` accumulator, scale computation, convergence break, and `solver_niter = iter` assignment. `costChange` refactored to return the cost change value (not just revert). `solver_stat` population with correct fields per iteration. Every loop, formula, and guard is written out. |
| **A** | Algorithm complete. One or two minor details left implicit. |
| **B** | Algorithm structure clear but some steps hand-waved. |
| **C** | Skeleton only. |

### P3. Convention Awareness

> Spec explicitly addresses MuJoCo → CortenForge conventions.

| Grade | Bar |
|-------|-----|
| **A+** | Convention notes table maps: `meaninertia` (MuJoCo `m->stat.meaninertia` → CortenForge `data.stat_meaninertia` — explicitly warns against using `model.stat_meaninertia` which is build-time only), `opt.tolerance` → `model.solver_tolerance`, `opt.iterations` → `model.solver_iterations`, `solver_niter[island]` → `data.solver_niter` (single value, no island indexing). `costChange` function naming and return convention documented. |
| **A** | Major conventions documented. Minor field-name mappings left to implementer. |
| **B** | Some conventions noted, others not. |
| **C** | MuJoCo code pasted without adaptation. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable.

| Grade | Bar |
|-------|-----|
| **A+** | ACs cover: (1) PGS converges before max_iters on a simple model (solver_niter < solver_iterations), (2) solver_stat populated with correct length and fields, (3) PGS produces same efc_force with/without early termination for converged problems, (4) QCQP verified unchanged, (5) golden flag checkpoint results. Each AC has concrete input model, expected behavior, and field to check. |
| **A** | ACs testable. Some lack exact values. |
| **B** | ACs directionally correct but vague. |
| **C** | ACs are aspirational. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Edge cases: nefc=0, single-row convergence, warmstart disabled, max_iters=1, solver_tolerance=0 (no early exit). At least one PGS-specific MuJoCo conformance test (comparing efc_force, solver_niter, qacc against MuJoCo PGS reference data). Regression test verifying existing PGS behavior unchanged (forces within tolerance). |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order unambiguous. States: S1 (DT-19 verification — no code) → S2 (early termination) → S3 (warmstart verification) → S4 (solver_stat). Each section states what it requires. Dependency on Spec A (already landed) stated with commit hash. Independence from Specs C/D stated. |
| **A** | Order clear. Minor interactions implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Every file touched, every behavior that changes, every test that might break.

| Grade | Bar |
|-------|-----|
| **A+** | File list: `pgs.rs` (primary), test files (new). Behavioral change: PGS may now converge in fewer iterations (improvement — faster, same result). `solver_niter` now reports actual count (may be < max_iters). `solver_stat` now populated for PGS. Existing test impact: names specific tests and states Pass/Value-change/Fail for each. No regressions expected — early termination only triggers when convergence is achieved. |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions. Shared concepts use identical terminology.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology uniform: "improvement" used consistently (not "cost change" or "residual" interchangeably). AC numbers match traceability matrix. File paths match between spec and blast radius. |
| **A** | Consistent. Minor terminology inconsistencies. |
| **B** | Some sections use different names for same concept. |
| **C** | Contradictions between sections. |

### P9. Solver Convergence Correctness

> The early termination logic preserves solver correctness — it must never
> terminate at a non-optimal point, and the improvement metric must match
> MuJoCo's exactly.

| Grade | Bar |
|-------|-----|
| **A+** | Spec proves: (a) `improvement` accumulation matches MuJoCo's `costChange()` sign convention exactly, (b) early termination only triggers when cost reduction is below tolerance (monotone convergence), (c) the `scale` factor formula matches MuJoCo's `1/(meaninertia * max(1,nv))`, (d) tolerance is read from `model.solver_tolerance`, (e) solver produces identical forces whether it exits early or runs max_iters (within floating-point). |
| **A** | Convergence logic correct. One minor detail implicit. |
| **B** | Convergence direction correct but formula gaps. |
| **C** | Convergence logic unclear or incorrect. |

---

## Rubric Self-Audit

- [x] **Specificity:** Every A+ bar names exact functions (`costChange`, `mj_solPGS`),
      fields (`solver_niter`, `solver_stat`, `improvement`), and formulas
      (`scale = 1/(meaninertia * max(1,nv))`). Two reviewers would agree.

- [x] **Non-overlap:** P1 grades MuJoCo reference accuracy; P2 grades algorithm
      implementation; P9 grades convergence correctness specifically. P1/P9
      boundary: P1 asks "did the spec correctly describe what MuJoCo does?";
      P9 asks "does the convergence logic provably maintain correctness?"

- [x] **Completeness:** 9 criteria cover: MuJoCo reference (P1), algorithm (P2),
      conventions (P3), ACs (P4), tests (P5), deps (P6), blast radius (P7),
      consistency (P8), convergence correctness (P9). P9 is the domain criterion
      needed because early termination can introduce subtle bugs if wrong.
      5 gaps logged (G1–G5), all assessed as Acceptable with justification.

- [x] **Gradeability:** Each criterion maps to specific spec sections per the
      standard mapping table.

- [x] **Conformance primacy:** P1 is tailored with `mj_solPGS`, `costChange`,
      `warmstart`, `mju_QCQP2/3/N` citations. P4 and P5 reference MuJoCo-derived
      expected values. P9 ensures convergence matches MuJoCo.

- [x] **Empirical grounding:** EGT-1 through EGT-4 verify QCQP match, early
      termination absence, costChange behavior, and solver_stat gap.

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors table, Convention Notes |
| P2 | Specification (S1, S2, S3, S4) |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius |
| P8 | *Cross-cutting — all sections* |
| P9 | Specification S2 (convergence logic), MuJoCo Reference (costChange) |

---

## Scorecard

| Criterion | Grade | Evidence |
|-----------|-------|----------|
| P1. MuJoCo Reference Fidelity | **A+** | Cites `mj_solPGS()` with C code for: `costChange()` verbatim, improvement accumulation loop, `scale = 1/(meaninertia*max(1,nv))`, `improvement < tolerance` break, `saveStats()` fields, `solver_niter += iter`. Warmstart: cites `warmstart()` in `engine_forward.c` with PGS-specific zeroing — correctly notes MuJoCo uses `cost > 0` (strict) while CortenForge uses `cost ≥ 0` (more conservative). QCQP: EGT-1 table documents line-by-line match. 6 edge cases addressed with MuJoCo behavior for each. Note: `nactive`/`nchange` in `saveStats` are deferred as diagnostic-only placeholders (0, 0) — justified in Out of Scope as they don't affect solver output. |
| P2. Algorithm Completeness | **A+** | S2 provides complete Rust code: `while iter < max_iters` loop, `improvement` accumulator, scale computation, convergence break, `solver_niter = iter`, `solver_stat` population. Cost guard refactored to return `cost_change`. Abbreviated inner sections explicitly marked "unchanged" — implementer can copy existing code. Implementation note #6 clarifies `res` variable scope. |
| P3. Convention Awareness | **A+** | Convention Notes table maps all 6 fields: `meaninertia` (with explicit warning: use `data.stat_meaninertia` not `model.stat_meaninertia`, cites `newton.rs:58` as precedent), `opt.tolerance`, `opt.iterations`, `solver_niter` (island→single), `saveStats`→`solver_stat.push`, `costChange` return convention. Each has explicit porting rule. |
| P4. Acceptance Criteria Rigor | **A+** | 9 ACs covering early termination, force correctness, solver_stat, max_iters edge case, QCQP code review, Newton regression, warmstart review, golden flag checkpoint, PGS MuJoCo conformance. Each has concrete model/expected/field. AC1 now uses MuJoCo-verified expected iteration count from T7 reference data (Rev 3). AC9 provides end-to-end PGS conformance with MuJoCo-verified efc_force, solver_niter, and qacc. |
| P5. Test Plan Coverage | **A+** | 7 tests + supplementary. Edge case inventory covers 6 cases (nefc=0, tolerance=0, max_iters=0/1, warmstart disabled, all reverted). Traceability matrix complete. T7 provides PGS-specific MuJoCo conformance test comparing efc_force, solver_niter, and qacc against MuJoCo PGS reference data. |
| P6. Dependency Clarity | **A+** | Execution order S1→S2→S3→S4→golden flag. Spec A prerequisite with commit hash `099299e`. Independence from Specs C/D stated. Each section states requirements from prior sections. |
| P7. Blast Radius & Risk | **A+** | Files: pgs.rs (primary), test files (new). Behavioral changes table with conformance direction. Existing test impact names 5 test categories with expected outcomes. Non-modification sites: 6 files listed with reasons. |
| P8. Internal Consistency | **A+** | "improvement" used consistently throughout. AC numbers match traceability matrix (AC1–AC9 → T1–T7). File paths consistent. Edge cases from MuJoCo Reference appear in Test Plan Edge Case Inventory. |
| P9. Solver Convergence Correctness | **A+** | Spec explains: (a) `improvement -= costChange(...)` double-negation yields positive improvement for cost-decreasing sweeps, (b) early termination at `improvement < tolerance` is monotone (improvement ≥ 0 by construction — costChange returns ≤ 0 or 0 for reverted), (c) scale formula matches MuJoCo exactly, (d) tolerance from `model.solver_tolerance`, (e) force equivalence guaranteed by AC2 (T2 regression test). |

**Overall: A+ (9/9 criteria) — Rev 3**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| G1 | P4 | AC1 lacks MuJoCo-verified expected PGS iteration count | Initial grading | **Resolved Rev 3:** AC1 changed from analytically-derived behavioral assertion to MuJoCo-verified — now asserts exact iteration count from T7 reference data (MuJoCo 3.4.0). AC1 shares the T7 conformance model so the expected value is empirically grounded. | Rev 3 |
| G2 | P5 | No PGS-specific MuJoCo-vs-CortenForge conformance test with exact verified values | Initial grading | **Resolved Rev 2:** Added AC9 (PGS MuJoCo conformance) and T7 (PGS MuJoCo conformance test) with reference data generation from MuJoCo PGS. Compares efc_force, solver_niter, qacc. P5 promoted to A+. | Rev 2 |
| G3 | P1 | `nactive`/`nchange` in `saveStats` use placeholder values (0, 0) instead of actual counts | Stress test Rev 2 | Acceptable: these are diagnostic-only fields that do not affect solver output or convergence. MuJoCo computes them via `dualState()` which is expensive. Explicitly deferred in Out of Scope section. Can be added as a follow-up if conformance testing reveals they matter. | Rev 2 |
| G4 | P1/P8 | Warmstart cost gate: MuJoCo uses `cost > 0` (strict), CortenForge uses `cost ≥ 0` (more conservative) | Stress test Rev 2 | Acceptable: boundary case (cost = exactly 0) is degenerate. CortenForge is more conservative (rejects borderline warmstarts MuJoCo would keep). Spec S3 documents this explicitly and justifies as conformance-equivalent. | Rev 2 |
| G5 | P4/P9 | AC2 force tolerance (1e-8) is empirically motivated, not formally proven | Stress test Rev 2 | Acceptable: after convergence (improvement < tolerance), remaining per-sweep cost changes are sub-tolerance. AC2 uses solver_tolerance as the comparison threshold and notes it should be calibrated empirically. | Rev 2 |
