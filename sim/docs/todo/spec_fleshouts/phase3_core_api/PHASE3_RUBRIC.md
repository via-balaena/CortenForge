# Phase 3 Audit — Grading Rubric

Grades the implementation of each Phase 3 item. Each criterion is scored
A/B/C/D/F. The phase ships at A across all criteria.

---

## P1. MuJoCo Conformance

> Implementation matches MuJoCo 3.4.0 behavior for every applicable API.

| Grade | Bar |
|-------|-----|
| **A** | Every item's algorithm, pipeline stage, data layout, and edge-case handling matches MuJoCo. Deviations are intentional, documented, and justified (e.g., Rust-idiomatic Arc callbacks vs C function pointers). |
| **B** | Core algorithms match. Minor deviations in non-physics behavior (e.g., missing rendering-only element types). |
| **C** | Some algorithms diverge without justification. |
| **D** | Major behavioral differences. |
| **F** | Implementation contradicts MuJoCo semantics. |

**Current: B** — Core physics algorithms match. Verified against MuJoCo C source.
Conformance gaps: (1) xfrc_applied sleep-gating differs (G19), (2) inverse
formula missing `qfrc_constraint` term (G22), (3) `fwdinv_error` format differs
from `solver_fwdinv[2]` (G23). Intentional divergences documented: PGS fallback
(G21), SpatialVector layout (G20), contactfilter polarity (G25). Deferred:
`ENABLE_INVDISCRETE` (G9), `mjcb_time` is profiler hook not physics (G16).

---

## P2. Data Integrity

> All Data fields are correctly initialized, zeroed on reset, and populated
> at the correct pipeline stage.

| Grade | Bar |
|-------|-----|
| **A** | Every Phase 3 Data field is: (1) initialized in `make_data()`, (2) zeroed in `reset()`, (3) populated at the correct pipeline stage, (4) documented with type/semantics. |
| **B** | Initialization and population correct, but some fields missing from `reset()`. |
| **C** | Fields defined but reset/population incomplete. |
| **D** | Fields exist but are never populated. |
| **F** | Fields missing entirely. |

**Current: B** — `cacc`, `cfrc_int`, `cfrc_ext`, `qfrc_inverse`, `fwdinv_error`
not zeroed in `reset()`. All else correct.

---

## P3. Test Coverage

> Every acceptance criterion from the original spec has a corresponding test.
> Edge cases and interactions are covered.

| Grade | Bar |
|-------|-----|
| **A** | Every AC has a test. Edge cases (nv=0, sleeping bodies, disabled flags, multi-body chains) have dedicated tests. Interaction tests exist (e.g., xfrc_applied + DISABLE_GRAVITY). |
| **B** | Core ACs covered. Some edge cases and interactions missing. |
| **C** | Happy-path tests only. |
| **D** | Minimal test coverage. |
| **F** | No tests. |

**Current: B** — Core tests exist for all 7 items. Missing: cfrc_ext with contacts,
sleep-state accumulators, multi-body cfrc_int chain, cb_sensor/cb_act_* tests,
RK4+step2 guard test.

---

## P4. Documentation Accuracy

> Spec documents, code comments, and Data field docstrings accurately describe
> the implementation. No stale or contradictory descriptions.

| Grade | Bar |
|-------|-----|
| **A** | All spec docs, docstrings, future_work entries, and builder warnings match the implementation. No contradictions. |
| **B** | Implementation correct but some documentation is stale or misleading. |
| **C** | Multiple contradictions between docs and code. |
| **D** | Documentation actively misleads. |
| **F** | No documentation. |

**Current: C+** — 5 documentation mismatches found (DT-21 stage, S51 cfrc_ext
contents, S53 step() refactoring, S59 element types, ENABLE_FWDINV warning).

---

## P5. API Design

> Public API is idiomatic Rust, matches MuJoCo naming conventions where
> appropriate, and is ergonomic for downstream users.

| Grade | Bar |
|-------|-----|
| **A** | All public methods are documented, have correct visibility, use standard Rust patterns (Option for fallibility, &mut self for mutation), and follow MuJoCo naming where it aids recognition. |
| **B** | API is correct and usable. Minor naming or ergonomic issues. |
| **C** | API works but is inconsistent or hard to discover. |
| **D** | API is confusing or incomplete. |
| **F** | No public API. |

**Current: A** — `inverse()`, `step1()`, `step2()`, `name2id()`, `id2name()`,
`set_*_callback()` are all well-designed with correct signatures and docs.

---

## P6. Pipeline Integration

> Each feature is wired into the forward/inverse pipeline at the correct
> stage, with correct flag guards and sleep-state handling.

| Grade | Bar |
|-------|-----|
| **A** | Every feature fires at the correct pipeline stage. Flag guards (DISABLE_*, ENABLE_*) are complete. Sleep-state interactions are correct. No dead code paths. |
| **B** | Pipeline stages correct. Minor flag guard issues (e.g., missing warning for RK4+step2). |
| **C** | Some features in wrong pipeline stage or missing guards. |
| **D** | Pipeline integration incomplete. |
| **F** | Features not integrated into pipeline. |

**Current: B+** — All features at correct stages. Issues: (1) step2() doesn't
warn on RK4 (G12), (2) `ENABLE_INVDISCRETE` is a no-op (G9), (3) xfrc_applied
projection incorrectly skips sleeping bodies (G19 — MuJoCo projects all bodies),
(4) cb_control DISABLE_ACTUATION gating needs verification (G24).

---

## P7. Code Quality

> Implementation follows codebase conventions: no dead code, no unnecessary
> duplication, correct error handling, idiomatic Rust.

| Grade | Bar |
|-------|-----|
| **A** | No dead code. Duplication is justified (e.g., implicit path xfrc_applied). Error handling is correct. Types are precise. No `unwrap()` in library code. |
| **B** | Code is clean with minor issues. |
| **C** | Some dead code or unnecessary complexity. |
| **D** | Significant code quality issues. |
| **F** | Unacceptable code quality. |

**Current: A** — Newton solver is particularly well-structured (1900+ lines of
tests, 3-phase architecture). Dual xfrc_applied projection is justified and
documented. All callback types use precise trait bounds.

---

## Scoring Summary

| Criterion | Current | Target | Gap |
|-----------|:-------:|:------:|-----|
| P1. MuJoCo Conformance | **B** | A | Sleep gating (G19), inverse formula (G22), fwdinv format (G23) |
| P2. Data Integrity | **B** | A | 5 fields missing from reset() |
| P3. Test Coverage | **B** | A | 5 test groups needed |
| P4. Documentation Accuracy | **C+** | A | 5 doc mismatches to fix |
| P5. API Design | **A** | A | — |
| P6. Pipeline Integration | **B+** | A | RK4 guard, sleep gating, cb_control gating, INVDISCRETE stub |
| P7. Code Quality | **A** | A | — |

**Ship criteria:** All A. Current: 2/7 at A, 5/7 below. The conformance gaps
(G19, G22, G23) are the most impactful — they represent actual behavioral
differences from MuJoCo that affect simulation correctness. See
[PHASE3_AUDIT.md](./PHASE3_AUDIT.md) Remediation Summary for the full fix list.
