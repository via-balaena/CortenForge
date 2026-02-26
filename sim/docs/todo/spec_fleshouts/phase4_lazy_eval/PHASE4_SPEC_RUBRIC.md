# Phase 4 — Spec Quality Rubric

Grades each spec on 8 criteria. Target: A+ on every criterion for every spec
before implementation begins. A+ means "an implementer could build this without
asking a single clarifying question."

---

## Criteria

### Q1. MuJoCo Reference Fidelity

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. No hand-waving.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo function/field/flag cited with source file + behavior. Calling conventions (reference points, spatial layout, frame) explicit. Edge cases (world body, zero mass, sleep) addressed. |
| **A** | MuJoCo behavior described correctly. Minor gaps in edge-case coverage. |
| **B** | Correct at high level, but missing specifics (e.g., "MuJoCo shifts the velocity" without saying from where to where). |
| **C** | Partially correct. Some MuJoCo behavior misunderstood or assumed. |

### Q2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. No "see MuJoCo source"
> or "TBD" gaps. Pseudocode or Rust is line-for-line implementable.

| Grade | Bar |
|-------|-----|
| **A+** | Every loop, every formula, every edge-case guard is written out. An implementer can type it in without reading MuJoCo source. No "verify against source" notes. |
| **A** | Algorithm is complete. One or two minor details left implicit. |
| **B** | Algorithm structure is clear but some steps are hand-waved or deferred. |
| **C** | Skeleton only — "implement this somehow." |

### Q3. Convention Awareness

> Spec explicitly addresses our codebase's conventions where they differ from
> MuJoCo (cvel reference point, SpatialVector layout, field naming, etc.)
> and provides the correct translation.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo→CortenForge translation is called out: cvel reference point, SpatialVector layout, field names, sleep arrays, Model vs mjModel field mapping. No implicit assumptions. |
| **A** | Major conventions documented. Minor field-name mappings left to implementer. |
| **B** | Some conventions noted, others not — risk of silent mismatch during implementation. |
| **C** | MuJoCo code pasted without adaptation to our conventions. |

### Q4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete values,
> tolerances, and model configurations. No "should work correctly."

| Grade | Bar |
|-------|-----|
| **A+** | Every AC has: (1) concrete input model/state, (2) exact expected value or tolerance, (3) what field/sensor to check. Could be copy-pasted into a test function. |
| **A** | ACs are testable. Some lack exact numerical expectations. |
| **B** | ACs are directionally correct but vague ("output should change"). |
| **C** | ACs are aspirational statements, not tests. |

### Q5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions.
> Each AC maps to at least one test. Negative cases (flag NOT triggered)
> are tested.

| Grade | Bar |
|-------|-----|
| **A+** | Every AC has ≥1 test. Edge cases tested: zero mass, world body, sleeping bodies, DISABLE_SENSOR, multi-sensor, reset mid-step. Negative cases (lazy gate NOT triggered) are first-class tests. |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases and negative cases sparse. |
| **C** | Minimal test plan. |

### Q6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions between the three
> specs are explicitly stated. No circular dependencies or implicit ordering.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order is unambiguous. Each spec states what it requires from prior specs. Cross-spec interactions (e.g., "cvel fixes must land before §56 regression tests make sense") are called out. |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced — implementer must infer. |
| **C** | No ordering discussion. |

### Q7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes, and
> every existing test that might break. Surprises are spec bugs.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list. For each change: what breaks, what regresses, how to verify. Staleness guards (Data size check) addressed. Existing test suite impact analyzed. |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. Some regression risk unaddressed. |
| **C** | No blast-radius analysis. |

### Q8. Internal Consistency

> No contradictions between the three specs. Shared concepts (cvel
> convention, lazy flag pattern, field names) use identical terminology.
> The umbrella spec and detail specs agree on scope and design.

| Grade | Bar |
|-------|-----|
| **A+** | All three specs use identical field names, flag names, and conventions. No contradictions. The umbrella spec's summary matches the detail specs exactly. Implementation order is consistent across all docs. |
| **A** | Consistent. Minor terminology drift. |
| **B** | Some contradictions or redundant/conflicting descriptions. |
| **C** | Specs disagree on design decisions. |

---

## Grades: `CVEL_REFERENCE_POINT_FIXES.md`

**Revision 2** — Re-graded after closing gaps C1-C5.

| Criterion | Grade | Evidence |
|-----------|:-----:|----------|
| Q1. MuJoCo Ref | **A+** | Cites `mj_objectVelocity`, correct shift description. Bug 1 now documents the exact MuJoCo calling convention: `flg_local=0` then rotate to sensor frame. |
| Q2. Algorithm | **A+** | Each fix is a single lever-arm shift. Full Rust code provided. Nothing ambiguous. |
| Q3. Convention | **A+** | Explicitly states our cvel is at `xpos[b]`, MuJoCo's at `subtree_com[root]`. Notes `xpos`/`xipos` semantics match MuJoCo's. |
| Q4. AC Rigor | **A+** | All ACs now have concrete model configs with exact expected vectors: B1-AC1 (`[-0.5, 0, 0]`), B1-AC3 (`[-0.8, 0.6, 0]`), B2-AC1 (`[-0.5, 0, 0]`), B3-AC1 (`[0, 0.1, 0]`), B5-AC1 (match within `1e-10`). |
| Q5. Test Plan | **A+** | 10 tests: T1-T6 (happy path + regression) + T7 (world body), T8 (zero-mass), T9 (sleeping body), T10 (DISABLE_SENSOR). Edge cases and negative cases covered. |
| Q6. Dependencies | **A+** | Explicit: "Phase 4 Step 0, prerequisite to both S56 and PHASE4_LAZY_EVAL." Cross-spec links added. |
| Q7. Blast Radius | **A+** | Files listed, ~25 lines changed. Existing test impact analyzed. Data staleness guard note: "No new fields — unaffected." |
| Q8. Consistency | **A+** | Uses identical cvel convention language as other specs. Full fix code shows complete match arms. |

**Overall: A+** — All gaps closed.

---

## Grades: `S56_SUBTREE_VEL_SPEC.md`

**Revision 2** — Re-graded after closing gaps S1-S6.

| Criterion | Grade | Evidence |
|-----------|:-----:|----------|
| Q1. MuJoCo Ref | **A+** | Complete 3-phase algorithm from MuJoCo source. Cites `engine_core_smooth.c`. Field names mapped. Uses `body_inertia` not `cinert`. Derivation of angular momentum transfer formula included. |
| Q2. Algorithm | **A+** | Full Rust implementation provided, Phase 1–3 line-for-line. `subtree_mass` Data vs Model convention documented. Zero-mass guard matches MuJoCo's `MJ_MINVAL` (`1e-15`). `compute_subtree_com()` deletion resolved (sensor reads persistent field). |
| Q3. Convention | **A+** | Documents cvel at `xpos[b]` vs MuJoCo's `subtree_com[root]`, derives simpler shift. Notes `body_inertia` vs `cinert`. New: `subtree_mass` Data vs Model convention. |
| Q4. AC Rigor | **A+** | 13 ACs with concrete model configs and expected values. Structural ACs (AC7, AC12, AC13) tagged as "code review — not a runtime test." |
| Q5. Test Plan | **A+** | 15 tests: T1-T11 (physics, regression, lazy gates, reset, O(n) equivalence, SubtreeCom direct read) + T12 (world body), T13 (zero-mass), T14 (DISABLE_SENSOR), T15 (sleep). |
| Q6. Dependencies | **A+** | Explicit prerequisite: "CVEL_REFERENCE_POINT_FIXES must land first — corrected O(n²) helpers serve as validation baseline for T9." |
| Q7. Blast Radius | **A+** | File list with per-file changes. `compute_subtree_com()` deletion resolved. Dead code list complete. Data staleness guard (Step 9). |
| Q8. Consistency | **A+** | Umbrella spec now defers to this spec for the algorithm (4B.5 replaced with reference). All field names, flag names, and conventions identical across all three specs. |

**Overall: A+** — All gaps closed.

---

## Grades: `PHASE4_LAZY_EVAL_SPEC.md` (Umbrella)

**Revision 2** — Re-graded after closing gaps U1-U8.

| Criterion | Grade | Evidence |
|-----------|:-----:|----------|
| Q1. MuJoCo Ref | **A+** | Complete lazy flag lifecycle. Cites `engine_sensor.c` guard code verbatim. Lists all triggering sensor types. Notes Touch/Contact/Tactile do NOT trigger. |
| Q2. Algorithm | **A+** | 4A (flg_rnepost) fully specified. 4B.5 now defers to S56_SUBTREE_VEL_SPEC for the authoritative algorithm — no stale/incorrect code. Umbrella describes the lazy gate pattern; S56 spec owns the algorithm. |
| Q3. Convention | **A+** | Documents cvel convention difference. 4B.5 no longer contains code with wrong convention — defers to S56 which handles the shift correctly. |
| Q4. AC Rigor | **A+** | 13 ACs covering both lazy gates. AC3 corrected: references the post-cvel-fix baseline. AC4 tagged as code review. AC12/AC13 cover DISABLE_SENSOR and sleep interactions. |
| Q5. Test Plan | **A+** | 12 tests: T1-T10 (physics, lazy gates, inverse, reset, O(n) equivalence) + T11 (DISABLE_SENSOR), T12 (sleep interaction). `forward_skip_sensors` interaction documented in 4A.2. |
| Q6. Dependencies | **A+** | Step 0 (cvel fixes) added to implementation order. S56_SUBTREE_VEL_SPEC explicitly referenced as authoritative algorithm source. Cross-spec links throughout. |
| Q7. Blast Radius | **A+** | File list complete. `compare_fwd_inv` ordering verified and documented in 4A.8 (sensors→inverse, inverse re-runs accumulators, intentional). `forward_skip_sensors` interaction documented. |
| Q8. Consistency | **A+** | No contradictions. 4B.5 defers to S56. AC3 accounts for cvel fix baseline. Field names, flag names, implementation order consistent across all three specs. 4B.7 matches S56 Step 8 (all three helpers deleted). |

**Overall: A+** — All gaps closed. Former B+ elevated to A+ by removing stale algorithm code, fixing AC3 contradiction, adding Step 0 prerequisite, adding missing tests, and documenting pipeline interactions.

---

## Grades: `SENSOR_CACC_CFRC_REFACTOR.md`

**Revision 2** — Re-graded after closing all gaps.

| Criterion | Grade | Evidence |
|-----------|:-----:|----------|
| Q1. MuJoCo Ref | **A+** | Cites `engine_sensor.c`, `engine_core_util.c` (`mj_objectAcceleration`), `engine_util_spatial.c` (`mju_transformSpatial`). Exact C code snippets for all 5 sensor types. Reference point convention difference documented. `flg_local`, `flg_force` calling conventions explicit. |
| Q2. Algorithm | **A+** | Full Rust implementation for all 5 sensor arms (Steps 1-5). Spatial motion transform, Coriolis correction, spatial force transform all line-for-line implementable. Coriolis with `r=0` explained (reduces to `omega × v_at_origin`, no offset contribution). Dead code deletion procedure explicit (Step 6). |
| Q3. Convention | **A+** | Reference point table (cacc/cfrc_int/cvel at xpos[b] vs subtree_com[root]). SpatialVector layout [angular; linear]. Gravity in cacc. cfrc_int formula from mj_body_accumulators. All MuJoCo→CortenForge translations explicit. |
| Q4. AC Rigor | **A+** | 9 ACs with concrete model configs, exact 3D expected vectors, and tolerances. AC1: `[0,0,+9.81]`. AC2: `[-50,0,0]` with algebraic breakdown. AC3: `[0,0,+9.81]` (conformance fix). AC4: `[0,5.0,0]`. AC5: `[0,0,+19.62]`. AC6: `[0,-50,0]` with spatial transport formula. AC7: regression within 1e-6. AC8/AC9: structural. |
| Q5. Test Plan | **A+** | 14 tests: T1-T10 (happy path + regression) + T11 (world body), T12 (zero-mass), T13 (DISABLE_SENSOR), T14 (sleep). Existing test impact analyzed (4 specific tests listed). Negative cases covered (T13: lazy gate not reached). |
| Q6. Dependencies | **A+** | Explicit prerequisite: "flg_rnepost lazy gate (Phase 4A) — already landed (commit 8e8f5f7)." Relationship to umbrella spec documented. No circular dependencies. |
| Q7. Blast Radius | **A+** | 4-file change list with per-file details. Import cleanup specified (Matrix3, derived::*, mod derived). Existing test impact: 4 tests individually analyzed. Data staleness guard: "No new fields — EXPECTED_SIZE unchanged." |
| Q8. Consistency | **A+** | Unified 4A.6 naming (no 4A.6b split). Umbrella spec's Out of Scope updated to reference this spec. Field names, convention language, spatial transform terminology identical to other Phase 4 specs. |

**Overall: A+** — All gaps closed.

---

## Cross-Spec Summary

| Spec | Q1 | Q2 | Q3 | Q4 | Q5 | Q6 | Q7 | Q8 | Overall |
|------|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:-------:|
| CVEL Fixes | A+ | A+ | A+ | A+ | A+ | A+ | A+ | A+ | **A+** |
| S56 Subtree | A+ | A+ | A+ | A+ | A+ | A+ | A+ | A+ | **A+** |
| Umbrella | A+ | A+ | A+ | A+ | A+ | A+ | A+ | A+ | **A+** |
| Sensor Refactor | A+ | A+ | A+ | A+ | A+ | A+ | A+ | A+ | **A+** |

**Ship criteria: All A+. 4/4 specs ready for implementation.**

---

## Gap Closure Record

All 19 gaps have been closed. See revision history below.

### CVEL_REFERENCE_POINT_FIXES — 5 gaps closed

| # | Gap | Resolution |
|---|-----|-----------|
| C1 | ACs lack exact numerical values | Added concrete model configs with expected vectors to all 5 bug ACs |
| C2 | Test plan missing edge cases | Added T7-T10: world body, zero-mass, sleeping, DISABLE_SENSOR |
| C3 | MuJoCo velocimeter convention | Documented in Bug 1: `flg_local=0` then rotate to sensor frame |
| C4 | Dependency to umbrella not stated | Added Dependencies section: "Phase 4 Step 0" |
| C5 | Data staleness guard note | Added: "No new Data fields — staleness guard unaffected" |

### S56_SUBTREE_VEL_SPEC — 6 gaps closed

| # | Gap | Resolution |
|---|-----|-----------|
| S1 | subtree_mass Model vs Data | Added Convention Note section documenting Data vs Model |
| S2 | Zero-mass guard value | Changed `1e-10` → `1e-15` (MJ_MINVAL) in both code instances |
| S3 | compute_subtree_com delete or keep | Resolved: delete. SubtreeCom sensor reads `data.subtree_com` directly |
| S4 | Prerequisite dependency on cvel fixes | Added to Prerequisites header + cross-spec link |
| S5 | Missing edge-case tests | Added T12-T15: world body, zero-mass, DISABLE_SENSOR, sleep |
| S6 | Umbrella spec contradiction | Verified: umbrella now defers to S56, no contradictions remain |

### PHASE4_LAZY_EVAL_SPEC (Umbrella) — 8 gaps closed

| # | Gap | Resolution |
|---|-----|-----------|
| U1 | 4B.5 algorithm wrong/incomplete | Replaced stale code with reference to S56_SUBTREE_VEL_SPEC |
| U2 | 4B.5 code missing cvel shift | Subsumed by U1 |
| U3 | AC3 contradicts cvel fix spec | Rewritten to reference post-fix baseline |
| U4 | Missing Step 0 (cvel fixes) | Added Step 0 to implementation order with cross-spec link |
| U5 | Missing DISABLE_SENSOR test | Added T11 |
| U6 | Missing sleep interaction test | Added T12 |
| U7 | forward_skip_sensors interaction | Documented in 4A.2 |
| U8 | compare_fwd_inv ordering | Verified and documented in 4A.8 |

---

## Next Step

Implement in order:
1. ~~**cvel reference point fixes** (CVEL_REFERENCE_POINT_FIXES.md) — ~25 lines~~ ✓ (commit 444046e)
2. ~~**§56 subtree fields** (S56_SUBTREE_VEL_SPEC.md) — new Data fields + O(n) algorithm~~ ✓ (commit 503ac6d)
3. ~~**lazy gates** (PHASE4_LAZY_EVAL_SPEC.md 4A) — demand-driven body accumulators~~ ✓ (commit 8e8f5f7)
4. ~~**sensor refactor** (SENSOR_CACC_CFRC_REFACTOR.md) — read cacc/cfrc_int, delete derived.rs~~ ✓ (commit 16cfcb3)
5. ~~**DT-103 spatial transport** (DT103_SPATIAL_TRANSPORT.md) — extract transport helpers, rewrite 6 sensor arms~~ ✓ (commit 29501df, review 6/6 A)

**Phase 4 complete (2026-02-26).** All 5 deliverables shipped and audited.
