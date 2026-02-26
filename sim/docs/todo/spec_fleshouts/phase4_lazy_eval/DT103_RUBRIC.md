# DT-103 — Spec Quality Rubric

Grades the DT-103 spatial transport helpers spec on 10 criteria. Target: A+ on
every criterion before implementation begins. A+ means "an implementer could
build this without asking a single clarifying question."

Criteria: all 8 from the Phase 4 family rubric (`PHASE4_SPEC_RUBRIC.md`) plus
two domain-specific additions (P5 API Design, P8 Consumer Completeness). Grade
scale: A+/A/B/C — same as Phase 4 (no F; anything below C is simply "does not
meet the bar").

---

## Criteria

### P1. MuJoCo Reference Fidelity

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. No hand-waving.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo function/field/flag cited with source file + behavior. Transport formulas (`flg_force=0` motion vs `flg_force=1` force) match exactly. Calling conventions (reference points, spatial layout `[angular; linear]`, rotation) explicit. Edge cases (world body, zero-mass, sleep) addressed. |
| **A** | MuJoCo behavior described correctly. Minor gaps in edge-case coverage. |
| **B** | Correct at high level, but missing formula signs or convention specifics (e.g., "MuJoCo shifts the velocity" without saying from where to where). |
| **C** | Partially correct. Some MuJoCo behavior misunderstood or assumed. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. No "see MuJoCo source"
> or "TBD" gaps. Rust code is line-for-line implementable.

| Grade | Bar |
|-------|-----|
| **A+** | Every function body, every formula, every edge-case guard is written out in Rust. An implementer can type it in without reading MuJoCo source. No "verify against source" notes. Coriolis correction fully expanded. Boundary cases (body_id = 0) explicitly addressed — either guarded or shown to be safe without a guard. |
| **A** | Algorithm is complete. One or two minor details left implicit. |
| **B** | Algorithm structure is clear but some steps are hand-waved or deferred. |
| **C** | Skeleton only — "implement this somehow." |

### P3. Convention Awareness

> Spec explicitly addresses our codebase's conventions where they differ from
> MuJoCo (cvel/cacc/cfrc_int reference point, SpatialVector layout, field
> naming) and provides the correct translation. This is the #1 silent-bug risk
> for spatial transport and warrants its own criterion.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo→CortenForge translation called out: reference point (`xpos[b]` vs `subtree_com[root]`), SpatialVector layout `[angular; linear]`, field names. Convention difference table present. Porting rule stated explicitly. Design decisions reference the convention as the central motivation (encoding it once). |
| **A** | Major conventions documented. Minor field-name mappings left to implementer. |
| **B** | Some conventions noted, others not — risk of silent mismatch during implementation. |
| **C** | MuJoCo code pasted without adaptation to our conventions. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete values,
> tolerances, and model configurations. No "should work correctly."

| Grade | Bar |
|-------|-----|
| **A+** | Every AC has: (1) concrete input values or model configuration, (2) exact expected output with 3D vectors, (3) numerical tolerance or "exact." Could be copy-pasted into a test function. Structural ACs (code review) are clearly tagged as such. |
| **A** | ACs are testable. Some lack exact numerical expectations. |
| **B** | ACs directionally correct but vague ("output should change"). |
| **C** | ACs are aspirational statements, not tests. |

### P5. API Design

> Function signatures are idiomatic Rust, correctly scoped, and self-documenting.
> Return types match consumer needs. Backward compatibility maintained.
> (DT-103-specific criterion — the Phase 4 specs modified existing code rather
> than introducing new API surface.)

| Grade | Bar |
|-------|-----|
| **A+** | All signatures shown with doc comments. Return type `(Vector3, Vector3)` justified vs `SpatialVector`/`[f64; 6]`. Visibility (`pub(crate)` vs `pub`) correct and justified. `object_velocity_local` wrapper retains exact existing signature including `_model` param. Parameter choice (`&Data` only) justified. `Option<&Matrix3>` maps cleanly to MuJoCo's `flg_local`. |
| **A** | Signatures correct. Minor doc gaps or one unjustified choice. |
| **B** | Signatures work but are awkward (e.g., returns `[f64; 6]` when callers destructure). |
| **C** | Signatures inconsistent or visibility wrong. |

### P6. Test Plan Coverage

> Tests cover unit (pure math), integration (model-based), regression (existing
> suite), and edge cases. Each AC maps to at least one test. Negative cases
> tested.

| Grade | Bar |
|-------|-----|
| **A+** | Every AC has ≥1 test. Unit tests for pure-math transport kernels (no model needed). Integration tests for composed helpers against physics models. Full regression suites (both domain-specific and full workspace). Edge cases: world body, zero offset, backward compat. Negative cases: DISABLE_SENSOR and sleep interactions addressed (either tested directly or inherited from existing suite with explicit justification). AC→Test traceability matrix present; supplementary tests (if any) justified. |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases and negative cases sparse. |
| **C** | Minimal test plan. |

### P7. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with prior Phase 4 work
> are explicitly stated. No implicit ordering.

| Grade | Bar |
|-------|-----|
| **A+** | Prerequisites stated: 4A.6 sensor refactor already landed (commit hash). Relationship to DT-102 (future consumer, out of scope) documented. Implementation step ordering unambiguous (kernels → composed helpers → wrapper → consumers → tests). Out-of-scope items explicitly listed with rationale. |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced — implementer must infer. |
| **C** | No ordering discussion. |

### P8. Consumer Completeness

> All inlined spatial transport patterns identified. Each consumer's before/after
> shown. No transport site left behind.
> (DT-103-specific criterion — the central risk of a refactor is missing a
> consumer and leaving orphan code.)

| Grade | Bar |
|-------|-----|
| **A+** | Every transport consumer identified with file, line range, and transport type. Before/after code sketches for every rewritten consumer. Explicit list of arms NOT refactored with physics justification. No orphan transport code after refactor (verifiable via grep). Non-sensor consumers (e.g., passive damping, derivatives) identified and confirmed unchanged or backward-compatible. Consumer count in table matches count claimed in problem statement. |
| **A** | All consumers listed. Minor before/after gaps. |
| **B** | Most consumers found. Some missed or before/after unclear. |
| **C** | Partial consumer list. No before/after comparison. |

### P9. Blast Radius & Risk

> Every file touched is listed. Behavioral changes (if any) identified.
> Regression strategy explicit. No surprises.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description and estimated line counts. Files NOT modified explicitly listed with reason. Existing test suites named (Phase 4 39-test, full sim domain 2141+). Data staleness guard verified (`EXPECTED_SIZE` unchanged). Import changes specified. Backward-compat callers (`passive.rs`, `derivatives.rs`) confirmed unaffected. |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. Some regression risk unaddressed. |
| **C** | No blast-radius analysis. |

### P10. Internal Consistency

> No contradictions between spec sections, between the spec and sibling Phase 4
> specs, or between counts/claims and the actual code. Formulas in one section
> don't contradict formulas in another. Numbers add up.
>
> Distinct from P3 (Convention Awareness): P3 grades whether MuJoCo→CortenForge
> *translations* are documented. P10 grades whether the spec *agrees with itself*
> and with its siblings.

| Grade | Bar |
|-------|-----|
| **A+** | Consumer counts in Problem Statement match Consumer Rewrites table. Transport formulas in "New Functions" match "MuJoCo Reference." Implementation steps reference the correct files and functions. Design decision exclusions (D6) match "arms unchanged" table. Terminology identical to sibling specs (CVEL, S56, 4A.6). No section contradicts another. |
| **A** | Consistent. Minor count drift or terminology mismatch. |
| **B** | Some contradictions or numbers that don't add up. |
| **C** | Sections contradict each other or sibling specs. |

---

## Grades: `DT103_SPATIAL_TRANSPORT.md`

**Revision 3** — Re-graded after closing gaps D4–D8.

| Criterion | Grade | Evidence |
|-----------|:-----:|----------|
| P1. MuJoCo Ref Fidelity | **A+** | Cites `mj_objectVelocity`, `mj_objectAcceleration`, `mju_transformSpatial` with source files. `flg_force=0` vs `flg_force=1` transport formulas exact. Force/Torque sensor C code from `engine_sensor.c`. Edge cases: world body (AC11, T13), zero-mass (Edge Cases section), sleep (Edge Cases section), DISABLE_SENSOR (Edge Cases section). |
| P2. Algorithm Completeness | **A+** | Full Rust implementation for all 6 functions. Every formula, every Coriolis correction expanded. `body_id == 0` explicitly addressed: wrapper guards with early return, core functions shown safe without guard (AC11 + Edge Cases section). An implementer can type it in verbatim. No "see MuJoCo source" notes. |
| P3. Convention Awareness | **A+** | Convention difference table (`xpos[b]` vs `subtree_com[root]`) with explicit porting rule. Design decisions D1 motivation references convention encoding. SpatialVector layout `[angular; linear]` documented. Every `object_*` function doc comment states "reads … at `xpos[body_id]` (our convention)." |
| P4. AC Rigor | **A+** | 12 ACs with concrete input vectors, exact expected outputs, and numerical tolerances. AC2: `[0,0,10]×[1,0,0]=[0,10,0]`. AC3: `−[0.5,0,0]×[0,0,100]=[0,−50,0]`. AC5: algebraic breakdown of centripetal `[−50,0,0]`. AC10: force invariance across multiple offsets. AC11: world body expected values. AC9, AC12: tagged as code review (structural). |
| P5. API Design | **A+** | All 6 signatures shown with doc comments. `(Vector3, Vector3)` return justified (D4). `pub(crate)` visibility justified (D1). `&Data`-only justified (D5). `object_velocity_local` wrapper retains exact existing signature + `body_id==0` guard. `Option<&Matrix3>` maps cleanly to `flg_local`. |
| P6. Test Coverage | **A+** | 14 tests: T1–T4 unit (pure kernels), T5–T10 integration (model-based), T11–T12 regression (Phase 4 + full domain), T13–T14 edge cases (world body, backward compat). DISABLE_SENSOR and sleep inherited from 4A.6 regression suite (T11 = 39 Phase 4 tests including sleep/disabled). AC→Test traceability: 12 ACs all mapped. 2 supplementary tests (T6, T10) justified. |
| P7. Dependency Clarity | **A+** | Prerequisites stated in header with commit hashes: 4A.6 (16cfcb3), flg_rnepost (8e8f5f7), cvel fixes (444046e), §56 (503ac6d). DT-102 referenced as future consumer (out of scope). DT-62 referenced for objtype dispatch (out of scope). Implementation steps ordered: kernels (1–2) → composed helpers (3–4) → wrapper (5) → re-exports (6) → sensor rewrites (7–8) → tests (9) → regression (10). |
| P8. Consumer Completeness | **A+** | 7 consumers (6 sensor arms + 1 helper) listed with file, line count, and transport type — matches problem statement's "6 sensor arms + 1 helper." Before/after code sketches for all. 3 excluded arms listed with physics justification. Non-sensor consumers (`forward/passive.rs` ×2, `derivatives.rs` ×2) confirmed unchanged via backward compat (AC8). |
| P9. Blast Radius | **A+** | 5 modified files with per-file change descriptions and estimated line deltas. 5 unmodified files with reasons. `EXPECTED_SIZE` guard (AC12). Import changes noted. Existing test impact analyzed (Phase 4, full domain, derivatives). |
| P10. Internal Consistency | **A+** | Consumer count "6 sensor arms + 1 helper = 7 total" consistent across problem statement, scope line, and consumer table. Transport formulas in "New Functions" match "MuJoCo Reference." Implementation steps 7–8 reference files matching consumer table. D6 exclusions match "3 arms unchanged" table. Terminology identical to sibling specs. |

**Overall: A+** — All gaps closed. All 10 criteria ship-ready.

---

## AC → Test Traceability

| AC | Tests | Coverage |
|----|-------|----------|
| AC1 (transport_motion zero) | T1 | Direct |
| AC2 (transport_motion nonzero) | T2 | Direct |
| AC3 (transport_force nonzero) | T3 | Direct |
| AC4 (object_velocity ≡ object_velocity_local) | T5, T14 | Direct + backward compat |
| AC5 (object_acceleration Coriolis) | T7, T8 | T7 static/gravity, T8 centripetal |
| AC6 (Phase 4 regression) | T11 | Suite (39 tests, includes sleep + DISABLE_SENSOR) |
| AC7 (full sim regression) | T12 | Suite (2,141+ tests) |
| AC8 (object_velocity_local compat) | T5, T14 | Direct (T5 consistency, T14 passive.rs context) |
| AC9 (convention encoded once) | Code review | Manual (`grep` verification post-refactor) |
| AC10 (transport_force invariance) | T4, T9 | T4 unit (pure kernel), T9 integration (model-based) |
| AC11 (world body) | T13 | Direct |
| AC12 (no new Data fields) | Code review | Manual (`EXPECTED_SIZE` unchanged) |

### Supplementary Tests (no dedicated AC)

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T6 (object_velocity world frame) | FrameLinVel equivalence, `None` rotation path | Sanity check; subsumes into AC7 regression |
| T10 (object_force torque transport) | Torque shift against hand-computed value | Complements AC3 with model-based values |

---

## Gap Closure Record

### Rubric: Revision 1 → Revision 2 (rubric self-audit)

| # | Gap | Resolution |
|---|-----|-----------|
| R1 | Missing Algorithm Completeness criterion (Phase 4 Q2) | Added P2. Algorithm Completeness — grades whether Rust code is line-for-line implementable. |
| R2 | Missing Convention Awareness criterion (Phase 4 Q3) | Added P3. Convention Awareness — the `xpos` vs `subtree_com` difference is the central safety concern and warrants standalone criterion. |
| R3 | Missing Dependency Clarity criterion (Phase 4 Q6) | Added P7. Dependency Clarity — grades prerequisites, implementation ordering, out-of-scope rationale. |
| R4 | P4/P6 Test Coverage A+ bar omitted DISABLE_SENSOR and sleep | Updated P6 A+ bar to explicitly name DISABLE_SENSOR and sleep coverage (inherited from Phase 4 regression suite). |
| R5 | Traceability matrix had orphan tests (T6, T7, T9, T10, T13) | Added "Orphan Tests" table explaining why each test exists without a dedicated AC. Fixed AC5 mapping to include T7. Fixed AC3 mapping to include T4. |
| R6 | Grade scale inconsistent with Phase 4 (F grade vs no F) | Removed F grade. Scale now A+/A/B/C matching Phase 4 family. |
| R7 | Rubric said "7 criteria" but Phase 4 pattern uses 8 | Now 10 criteria (all 8 from Phase 4 + 2 DT-103-specific). Explicit note on which are inherited vs added. |
| R8 | Missing Internal Consistency criterion (Phase 4 Q8) | Added P10. Internal Consistency — grades terminology consistency within spec and with sibling Phase 4 specs. |

### Spec: Revision 1 → Revision 2 (gaps exposed by rubric audit)

| # | Gap | Resolution |
|---|-----|-----------|
| D1 | No prerequisite statement with commit hashes | Added `**Prerequisite:**` line in header with 4 commit hashes (4A.6, flg_rnepost, cvel fixes, §56). |
| D2 | No mention of DISABLE_SENSOR or sleep behavior | Added "Edge Cases & Interactions" section documenting DISABLE_SENSOR, sleep, and zero-mass — all structurally unaffected, covered by regression. |
| D3 | Zero-mass edge case not addressed | Covered in Edge Cases section: transport helpers produce correct zero output for zero-mass bodies. World body (body_id=0) is the canonical case, tested in T13. |

### Revision 2 → Revision 3 (stress-test audit)

**Rubric fixes:**

| # | Gap | Resolution |
|---|-----|-----------|
| R9 | P2 A+ bar said "`body_id == 0` guard present" but core functions have no guard | Reworded: "Boundary cases (body_id = 0) explicitly addressed — either guarded or shown safe without a guard." |
| R10 | P3 and P10 overlap on convention terminology | Sharpened P10 scope: now focuses on intra-spec consistency (counts match, formulas agree across sections, implementation steps reference correct files). P3 remains focused on MuJoCo→CortenForge translation. Overlap eliminated. |
| R11 | P6 A+ bar hard-coded function names (`transport_motion`/`transport_force`) | Generalized: "unit tests for pure-math transport kernels," "integration tests for composed helpers." Rubric now evaluates *kind* of coverage, not specific function names. |
| R12 | P8 A+ bar hard-coded "All 7 transport consumers" | Generalized: "Every transport consumer identified." Consumer count verified against problem statement claim as a P10 check. |

**Spec fixes:**

| # | Gap | Resolution |
|---|-----|-----------|
| D4 | Problem statement said "7 sensor arms" — actually 6 arms + 1 helper | Fixed to "6 sensor arms in two files and 1 existing helper in a third file (7 consumers total)." |
| D5 | "7 separate locations" for raw subtraction — actually 6 (Force has no shift) | Fixed to "6 separate locations (5 sensor arms + 1 helper)." |
| D6 | Consumer table heading "7 sensor arms refactored" — object_velocity_local is not a sensor arm | Fixed to "7 consumers refactored (6 sensor arms + 1 helper; 3 arms unchanged)." |
| D7 | 4 orphan tests (T6, T9, T10, T13) had no AC | Promoted T9→AC10 (force invariance), T13→AC11 (world body). T4 now maps to AC10. Orphan table reduced to 2 supplementary tests (T6, T10) — genuine sanity checks. |
| D8 | AC10 was "No new Data fields" — should be AC12 after new ACs inserted | Renumbered: AC10 = force invariance, AC11 = world body, AC12 = no new Data fields. |
