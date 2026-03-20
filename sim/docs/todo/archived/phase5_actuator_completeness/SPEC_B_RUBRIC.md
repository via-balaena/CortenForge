# Spec B — Transmission Types + Slider-Crank: Spec Quality Rubric

Grades the Spec B spec on 10 criteria. Target: A+ on every criterion
before implementation begins. A+ means "an implementer could build this
without asking a single clarifying question — and the result would produce
numerically identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest. A spec that scores A+ on
P2–P10 but has P1 wrong is worse than useless: it would produce a clean,
well-tested implementation of the wrong behavior.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Criteria

> **Criterion priority:** P1 (MuJoCo Reference Fidelity) is the cardinal
> criterion. MuJoCo conformance is the entire reason CortenForge exists.
> A spec can be A+ on P2–P10 and still be worthless if P1 is wrong — because
> an incorrect MuJoCo reference means every algorithm, every AC, and every
> test is verifying the wrong behavior. **Grade P1 first and grade it hardest.**
> If P1 is not A+, do not proceed to grading P2–P10 until it is fixed.

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. No hand-waving. This is the
> single most important criterion: everything else in the spec is downstream
> of getting the MuJoCo reference right.

| Grade | Bar |
|-------|-----|
| **A+** | `mj_transmission()` in `engine_core_smooth.c` cited with exact behavior for: (a) `mjTRN_SLIDERCRANK` case — entirely new case block (crank/slider site geometry, cranklength rod, determinant formula `det = av² + r² - v·v`, chain-rule Jacobian moment via `mj_jacPointAxis` and `mj_jacSite` in `engine_core_util.c`; spec documents `mj_jacPointAxis` algorithm: computes point Jacobian via `mj_jac`, then axis Jacobian via `jacAxis_col = cross(jacr_col, axis)` per DOF column; spec documents Jacobian-to-site assignment: `mj_jacSite` called on **crank** site (translational only, `jacr=NULL`), `mj_jacPointAxis` called on **slider** site (both point + axis Jacobians)), (b) `mjTRN_JOINTINPARENT` — shares Joint's case block via fallthrough; for hinge/slide joints, behaviorally identical to Joint (scalar gear, no rotation). Spec documents ball/free JointInParent behavior from MuJoCo C source (gear rotation via inverse quaternion) but explicitly scopes it out: ball/free joint transmission is a pre-existing gap in the base `Joint` type (`mj_actuator_length` skips `nv > 1`), and fixing it for JointInParent alone would be inconsistent. Every field cited: `actuator_trntype`, `actuator_trnid[2*i]`/`[2*i+1]`, `actuator_cranklength`, `actuator_gear`, `site_xpos`, `site_xmat` (z-column extraction for slider axis), `site_bodyid` (body argument for Jacobian calls). Edge cases addressed with MuJoCo's behavior: `det <= 0` singularity (slider-crank degrades to `length = av`, derivatives degenerate to `dlda = vec`, `dldv = axis`), hinge/slide JointInParent == Joint (identity). C code snippets for non-obvious formulas (length, derivatives dlda/dldv). |
| **A** | Correct behavior from C source. Minor gaps in edge-case coverage (e.g., det=0 graceful degradation not fully described). |
| **B** | Correct at high level but missing specifics (e.g., "slider-crank computes length from geometry" without the exact formula). |
| **C** | Partially correct. Slider-crank algorithm misunderstood or assumed from docs. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. No "see MuJoCo source"
> or "TBD" gaps. Rust code is line-for-line implementable.

| Grade | Bar |
|-------|-----|
| **A+** | Complete Rust implementation for: (1) `mj_jac_point_axis()` helper in `jacobian.rs` — wraps `mj_jac()` to compute point Jacobian + axis Jacobian (`jacAxis_col = cross(jacr_col, axis)` per DOF column), (2) JointInParent for hinge/slide (identical to Joint — scalar gear, same code path; both new types widen existing match arms rather than add novel logic for force application and derivatives), (3) MJCF parsing for `jointinparent` attribute (maps to new enum variant, same trnid semantics as `joint`), (4) MJCF parsing for `cranksite`/`slidersite`/`cranklength` (maps to SliderCrank variant; builder validates `cranklength > 0` — MuJoCo compiler throws `mjCError` if non-positive; builder validates `slidersite` is present when `cranksite` is specified), (5) SliderCrank length formula with determinant, (6) SliderCrank derivative computation (dlda, dldv — note: MuJoCo C reuses `dlda` as temp buffer when computing `dldv`, then overwrites; spec must either reproduce this trick or compute independently), (7) SliderCrank Jacobian composition: `mj_jac_site` on crank site (translational only, rotational=NULL) and `mj_jac_point_axis` on slider site, then `jac = J_crank - J_slider_point` (subtraction), (8) SliderCrank moment = chain-rule J^T application, (9) Gear scaling: `length *= gear[0]` after the complete chain-rule moment computation (gear does NOT affect derivatives or Jacobians); `moment *= gear[0]` when storing into `actuator_moment[i]` (gear is baked into the moment vector, so velocity = `moment.dot(&qvel)` with no additional gear factor). All formulas written out, no "verify against source" notes. Ball/free joint transmission explicitly out of scope (pre-existing gap in base Joint type). |
| **A** | Algorithm is complete. One or two minor details left implicit (e.g., epsilon tolerance for `det <= 0` singularity detection, or moment caching strategy for multiple SliderCrank actuators). |
| **B** | Algorithm structure clear but derivative computation hand-waved. |
| **C** | Skeleton only. |

### P3. Convention Awareness

> Spec explicitly addresses our codebase's conventions where they differ from
> MuJoCo. Convention mismatches are conformance bugs.

| Grade | Bar |
|-------|-----|
| **A+** | Convention table covers: (1) `actuator_trnid` indexing — MuJoCo uses `[2*i]`/`[2*i+1]` flat array, we use `Vec<[usize; 2]>` with `[i][0]`/`[i][1]`; (2) `actuator_trnid` slot semantics for SliderCrank — `trnid[0]` = cranksite ID, `trnid[1]` = slidersite ID (confirmed from MuJoCo compiler `user_objects.cc`); (3) `actuator_gear` — MuJoCo `gear + 6*i` flat, we use `Vec<[f64; 6]>` with `[i]`; (4) `site_xmat` — MuJoCo row-major `9*id + k` indexing, we use `nalgebra::Matrix3` column access; (5) `actuator_moment` — MuJoCo sparse CSR, we use dense `Vec<DVector<f64>>`; (6) Jacobian layout — MuJoCo `3*nv` row-major, we use nalgebra `Matrix3xX` column-major; (7) quaternion conventions (w,x,y,z); (8) function naming — MuJoCo C uses camelCase (`mj_jacPointAxis`, `mj_jacSite`), CortenForge Rust uses snake_case (`mj_jac_point_axis`, `mj_jac_site`); spec uses C names when citing MuJoCo behavior, Rust names when describing implementation. Each convention difference has an explicit porting rule. |
| **A** | Major conventions documented. Minor differences left implicit. |
| **B** | Some conventions noted, others not. |
| **C** | MuJoCo code pasted without adaptation. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable.

| Grade | Bar |
|-------|-----|
| **A+** | Every runtime AC has: (1) concrete MJCF model, (2) exact expected value or tolerance (analytically derived from the geometry), (3) field to check. SliderCrank AC includes a worked example with exact length/moment values from the geometry. JointInParent AC verifies hinge/slide produces identical length/velocity/force to Joint. MJCF parsing ACs verify round-trip for all 4 new attributes (`jointinparent`, `cranksite`, `slidersite`, `cranklength`). At least one AC per new transmission type. |
| **A** | ACs are testable. Some lack exact numerical expectations. |
| **B** | ACs directionally correct but vague ("length should be nonzero"). |
| **C** | ACs are aspirational statements. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions.

| Grade | Bar |
|-------|-----|
| **A+** | AC->Test traceability present. Edge cases tested: slider-crank `det <= 0` singularity, hinge JointInParent == Joint (identity — same length, same velocity, same force), slide JointInParent == Joint (same identity), SliderCrank with various gear values, SliderCrank with non-unit cranklength, MJCF parsing round-trip for `cranklength`/`cranksite`/`slidersite`/`jointinparent`, MJCF error for incomplete SliderCrank (cranksite without slidersite), MJCF error for non-positive cranklength. Negative cases: JointInParent on hinge produces identical result to Joint; SliderCrank with `det <= 0` degrades gracefully (no panic). At least one analytically-verified conformance test per new transmission type. |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and cross-spec interactions stated.

| Grade | Bar |
|-------|-----|
| **A+** | States dependency on Spec A (already landed, commit hash). States that new enum variants require arms in `compute_actuator_params()` / `build_actuator_moment()` in `muscle.rs`. Notes T1-b (dynprm resize to `[f64; 10]`) already landed — no blocking dependency. Flags the dangerous catch-all `_ => {}` in `mj_set_length_range` (line 509) that silently skips new types — must be replaced with explicit arms. Execution order is section-by-section with verification gates. |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes, and every
> existing test that might break.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list covering all exhaustive match sites that break when 2 new enum variants are added. **sim-core:** `enums.rs` (2 new enum variants), `model.rs` (+1 field `actuator_cranklength`), `model_init.rs` (+1 init), `jacobian.rs` (+`mj_jac_point_axis` helper), `actuation.rs` (+`mj_transmission_slidercrank` dispatch, +match arms in `mj_actuator_length` and `mj_fwd_actuation`), `muscle.rs` (+arms in `compute_actuator_params` line 160 and `build_actuator_moment` line 344 for acc0/dampratio; **critical:** replace catch-all `_ => {}` in `mj_set_length_range` line 509 with explicit arms — current catch-all silently skips new types), `derivatives.rs` (+widen match arms in `mjd_actuator_vel` line 563), `sensor/position.rs` (+match arms in `mj_sensor_pos`). **sim-mjcf:** `types.rs` (+4 fields: `jointinparent`, `cranksite`, `slidersite`, `cranklength`), `parser.rs` (+parsing for 4 new attributes), `builder/actuator.rs` (+transmission resolution for new types), `builder/build.rs` (+match arms in sleep policy: SliderCrank must mark **both** crank site and slider site trees as `AutoNever`; JointInParent same as Joint). Names specific existing tests (Phase 4 suite, Spec A tests) and states expected impact (all pass unchanged — extension only, no behavioral changes to existing types). Sim domain test baseline stated. |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Shared concepts use identical terminology.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology uniform (SliderCrank vs slider-crank consistently distinguished as Rust enum name vs mechanism description). File paths match between Specification and Files Affected. AC numbers match Traceability Matrix. Edge cases consistent between MuJoCo Reference and Test Plan. Field name `actuator_cranklength` spelled identically everywhere. |
| **A** | Consistent. One or two minor inconsistencies. |
| **B** | Some sections use different names for the same concept. |
| **C** | Contradictions between sections. |

### P9. Pipeline Integration

> New transmission types must integrate correctly into the existing forward
> dynamics pipeline ordering: position stage -> velocity stage -> force
> application. The spec must show exactly where each new code path plugs in.

| Grade | Bar |
|-------|-----|
| **A+** | Spec explicitly maps each new transmission type to its pipeline stage: (1) SliderCrank moment/length computed in position stage (new dispatch function alongside `mj_transmission_site`) — requires FK results (site_xpos, site_xmat) and Jacobian infrastructure (xpos, xquat, joint tree), (2) SliderCrank velocity from cached moment in `mj_actuator_length`, (3) SliderCrank force application via cached moment in `mj_fwd_actuation`, (4) JointInParent for hinge/slide handled inline in `mj_actuator_length` (identical to Joint — scalar gear, scalar moment shortcut, same `qfrc_actuator[dof_adr] += gear * force` in `mj_fwd_actuation`). Pipeline ordering constraints stated explicitly: "SliderCrank dispatch must run after FK (needs site_xpos/site_xmat, mj_jac)." All match sites that need new arms explicitly enumerated. |
| **A** | Pipeline stages mentioned but ordering constraints not fully explicit. |
| **B** | New functions described but not mapped to pipeline stages. |
| **C** | No pipeline discussion. |

### P10. Geometric Correctness

> The slider-crank mechanism involves non-trivial 3D geometry. The spec must
> derive the geometry correctly and verify it against known analytical results.

| Grade | Bar |
|-------|-----|
| **A+** | Slider-crank geometry fully derived: (1) slider axis = z-column of slider site rotation matrix, (2) vec = cranksite_pos - slidersite_pos, (3) length = a·v - sqrt(det) where det = (a·v)² + r² - v·v, (4) derivatives dlda and dldv derived from chain rule and verified against MuJoCo C code — exact formulas: `dldv = axis*(1 - av/sqrt(det)) + vec/sqrt(det)`, `dlda = vec*(1 - av/sqrt(det))`, (5) Jacobian assembly: `mj_jacSite` on **crank** site (translational only), `mj_jacPointAxis` on **slider** site (both translational + axis), then `jac_vec = J_crank - J_slider_point`; moment = `dlda^T * J_axis + dldv^T * jac_vec`, (6) at least one worked example with numerical values (e.g., known slider/crank site positions, compute expected length and moment analytically). Singularity case (det <= 0) handled with graceful degradation matching MuJoCo: `length = av`, `dlda = vec`, `dldv = axis`. |
| **A** | Geometry correct. Derivatives stated without full derivation. |
| **B** | Length formula correct but derivatives hand-waved. |
| **C** | Geometry partially incorrect or assumed. |

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names specific MuJoCo functions (`mj_transmission`,
      `mj_jacPointAxis`, `mj_jacSite`), specific fields (`actuator_cranklength`,
      `actuator_trnid`), and specific edge cases (`det <= 0`, hinge JointInParent == Joint).

- [x] **Non-overlap:** P1 covers MuJoCo reference accuracy, P2 covers algorithm
      completeness, P3 covers convention translation, P10 covers geometric derivation.
      P1 intentionally shares some content with P2 and P10 (Jacobian-to-site assignment,
      degenerate derivative values, `mj_jacPointAxis` algorithm) because the cardinal
      criterion must be self-contained. The distinction: P1 grades whether the spec GOT
      the MuJoCo reference right, P2 grades whether it translates into complete Rust
      pseudocode, P10 grades whether the geometric derivation is mathematically rigorous
      with a worked example. Same content, different grading angle.

- [x] **Completeness:** Standard P1-P8 + P9 (pipeline integration, critical for
      forward dynamics correctness) + P10 (geometric correctness, critical for
      slider-crank mechanism). No meaningful dimension uncovered.

- [x] **Gradeability:** Each criterion maps to specific spec sections (see mapping below).

- [x] **Conformance primacy:** P1 requires C source verification. P4 requires
      analytically-derived expected values. P5 requires conformance tests with
      MuJoCo-verified values. P10 requires worked geometric examples.

### Criterion -> Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors table, Convention Notes |
| P2 | Specification (S1-S7 or however many sections) |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC->Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius |
| P8 | *Cross-cutting — all sections* |
| P9 | Specification (pipeline stage mapping), Execution Order |
| P10 | MuJoCo Reference (geometry), Specification (SliderCrank formula) |

---

## Scorecard

| Criterion | Grade | Evidence |
|-----------|-------|----------|
| P1. MuJoCo Reference Fidelity | | |
| P2. Algorithm Completeness | | |
| P3. Convention Awareness | | |
| P4. Acceptance Criteria Rigor | | |
| P5. Test Plan Coverage | | |
| P6. Dependency Clarity | | |
| P7. Blast Radius & Risk | | |
| P8. Internal Consistency | | |
| P9. Pipeline Integration | | |
| P10. Geometric Correctness | | |

**Overall: {grade}**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | P1 | "3 new code paths" misleading — JointInParent shares Joint's case block via fallthrough, only diverges for ball/free | Rubric self-audit | Reworded to describe 2 case blocks (SliderCrank new, JointInParent conditional within Joint) | Rev 2 |
| R2 | P2, P7 | Missing `mj_jac_point_axis` prerequisite — SliderCrank calls this but CortenForge doesn't have it | Rubric self-audit (codebase grep) | Added to P2 algorithm list (item 1) and P7 file list (`jacobian.rs`) | Rev 2 |
| R3 | P9 | JointInParent ball/free incorrectly said to need position-stage dispatch — gear rotation reads `qpos` directly, no FK needed | Rubric self-audit (C source analysis) | Corrected: inline in `mj_actuator_length`, key requirement is multi-DOF `actuator_moment` population | Rev 2 |
| R4 | P9 | SliderCrank FK dependency incomplete — also needs Jacobian infrastructure (xpos, xquat, joint tree) | Rubric self-audit | Added Jacobian prerequisites to FK dependency statement | Rev 2 |
| R5 | P1, P2, P4, P5, P9 | Ball/free joint transmission is a pre-existing gap in base Joint type (`mj_actuator_length` has `if nv == 1` guard). Rubric demanded ball/free JointInParent testing but this builds on broken foundation. | Rubric audit round 2 (codebase analysis) | Scoped ball/free OUT of Spec B. P1 documents MuJoCo ball/free behavior but marks it out-of-scope. P2/P4/P5/P9 updated to hinge/slide only. Ball/free joint transmission deferred as separate conformance fix. | Rev 3 |
| R6 | P7 | File list missed 3 sim-core files with exhaustive `ActuatorTransmission` matches: `derivatives.rs`, `sensor/position.rs`, `builder/build.rs`; plus 2 sim-mjcf files: `types.rs`, `parser.rs` | Rubric audit round 2 (exhaustive match grep) | Added all 5 missing files to P7 with specific change descriptions. | Rev 3 |
| R7 | P2, P4 | MJCF parsing not enumerated as algorithm step — `types.rs` (4 new struct fields), `parser.rs` (4 new attribute parsers), `builder/actuator.rs` (transmission resolution) need explicit spec sections | Rubric audit round 2 (MJCF schema analysis) | Added MJCF parsing as P2 items (3) and (4). P4 updated to require parsing round-trip ACs. | Rev 3 |
| R8 | P2 | A-grade bar parenthetical "sparse compression mechanics" is stale — CortenForge uses dense `Vec<DVector<f64>>` | Rubric audit round 3 | Replaced with relevant example: "epsilon tolerance for det ≤ 0, or moment caching strategy" | Rev 4 |
| R9 | P3 | Missing SliderCrank trnid slot semantics: which slot = cranksite, which = slidersite. Confirmed from MuJoCo compiler `user_objects.cc`: `trnid[0]` = cranksite, `trnid[1]` = slidersite. | Rubric audit round 3 (MuJoCo compiler source) | Added as convention table item (2). Table now has 7 items. | Rev 4 |
| R10 | P6, P7 | **Critical:** `muscle.rs` line 509 has catch-all `_ => {}` in `mj_set_length_range()` — will **silently skip** new types instead of compile-failing. Most dangerous match site. | Rubric audit round 3 (muscle.rs exhaustive match audit) | Flagged in P6 and P7 as requiring explicit replacement of catch-all with named arms. | Rev 4 |
| R11 | P6, P7 | Rubric referenced nonexistent function `compute_muscle_params()`. Correct names: `compute_actuator_params()` (line 139) and `build_actuator_moment()` (line 344). | Rubric audit round 3 | Fixed all references to use correct function names. | Rev 4 |
| R12 | P7 | SliderCrank sleep policy in `builder/build.rs` must mark **both** crank site and slider site trees as `AutoNever` — MuJoCo runtime checks both site bodies for wakefulness. | Rubric audit round 3 (MuJoCo `engine_sleep.c` analysis) | Added dual-tree marking detail to P7 `builder/build.rs` entry. | Rev 4 |
| R13 | P7 | Wrong function name in `derivatives.rs` entry: rubric said `mj_dqfrc_du()` — actual function is `mjd_actuator_vel()` at line 529, with match site at line 563. | Rubric audit round 4 (codebase verification) | Fixed P7 A+ bar: `mj_dqfrc_du` → `mjd_actuator_vel` line 563. | Rev 5 |
| R14 | P2, P10 | Spec-writing subtleties for SliderCrank not flagged in rubric: (a) MuJoCo reuses `dlda` as temp buffer when computing `dldv`; (b) gear scaling locations (`length *= gear[0]` after derivatives, `moment *= gear[0]` during storage); (c) `mj_jacSite` on crank site is translational-only (rotational=NULL), `mj_jacPointAxis` on slider site provides both; (d) explicit Jacobian subtraction `jac = J_crank - J_slider_point`. | Rubric audit round 4 (MuJoCo C source character-level verification) | Added to P2 items (6)(7)(9), P10 items (4)(5) with exact derivative formulas and Jacobian assembly details. | Rev 5 |
| R15 | P3 | Implicit C-vs-Rust naming convention for Jacobian functions (`mj_jacPointAxis` vs `mj_jac_point_axis`) followed consistently but never explicitly documented. Implementer seeing P2 and P10 side-by-side would see same function under two names without explanation. | Rubric audit round 5 (internal consistency check) | Added as convention table item (8): C camelCase vs Rust snake_case naming rule. Table now has 8 items. | Rev 6 |
| R16 | P2, P5 | MuJoCo compiler validates `cranklength > 0` (throws `mjCError` if non-positive) and requires `slidersite` when `cranksite` is specified. Rubric P5 tested incomplete SliderCrank but not non-positive cranklength. | Rubric audit round 5 (MuJoCo compiler source `user_objects.cc`) | Added `cranklength > 0` validation to P2 item (4) and non-positive cranklength error test to P5. | Rev 6 |
| R17 | P8, Gap Log | R6 file count arithmetic error: said "4 files" but listed 3; said "6 missing" but 3+2=5. R11 mixes function-definition line (139) vs match-site line (344) in same sentence. | Rubric audit round 5 (internal consistency check) | Fixed R6 counts to "3 sim-core files" and "5 missing files". R11 left as-is (historical, P7 is internally consistent). | Rev 6 |
| R18 | P1 | Gating rule text said "P2–P8" but rubric has 10 criteria (P9, P10 added as domain-specific). Remnant from initial draft. | Rubric audit round 6 (holistic completeness) | Fixed all occurrences: "P2–P8" → "P2–P10" (lines 11, 23, 26). | Rev 7 |
| R19 | P1 | Four precision gaps in P1 A+ bar: (G1) no source file cited for `mj_jacPointAxis` (`engine_core_util.c`); (G3) no requirement to document Jacobian-to-site assignment (which function on which site); (G5) "derivatives degenerate" without exact values; (G9) no requirement to document `mj_jacPointAxis` internal algorithm (`cross(jacr_col, axis)`). Together these would allow a spec to pass P1 while having incorrect Jacobian composition. | Rubric audit round 6 (P1 precision analysis) | Tightened P1 A+ bar: added `engine_core_util.c` citation, `mj_jacPointAxis` algorithm description, Jacobian-to-site assignment (crank=`mj_jacSite` translational-only, slider=`mj_jacPointAxis`), exact degenerate values (`dlda=vec`, `dldv=axis`), and `site_bodyid` field citation. | Rev 7 |
| R20 | P1 | Gap log coherence verified: all 17 prior resolutions (R1-R17) confirmed applied to rubric text. All multi-criteria fix propagation complete. No stale references in active criteria. | Rubric audit round 6 (gap log coherence) | No fix needed — verification pass. | Rev 7 |
| R21 | Self-audit | "Non-overlap" bullet (lines 157-160) was stale: claimed P1/P2/P10 don't overlap, but after R19 expanded P1 to include mathematical formulas and algorithmic details. P1 now intentionally shares content with P2 (Jacobian-to-site assignment, `mj_jacPointAxis` algorithm) and P10 (degenerate derivative values). | Rubric audit round 7 (P1/P2/P10 cross-check) | Rewrote non-overlap bullet to acknowledge intentional overlap and clarify the distinction: same content graded from different angles (MuJoCo accuracy vs Rust completeness vs mathematical rigor). | Rev 8 |
| R22 | P2 | Ambiguous cross product phrasing in P2 item (1): "cross product of rotational Jacobian columns with axis" could be misread as `cross(axis, jacr_col)`. P1 already uses unambiguous `cross(jacr_col, axis)` notation. | Rubric audit round 7 (P1/P2 cross-check) | Changed P2 item (1) to use explicit formula notation: `jacAxis_col = cross(jacr_col, axis)`. | Rev 8 |
| R23 | P2 | Gear scaling phrase "during moment storage" borrowed from MuJoCo sparse idiom; imprecise for dense implementation. Also did not explicitly state that gear is baked into moment (preventing double-scaling in velocity path). | Rubric audit round 7 (gear scaling trace) | Rewrote P2 item (9): "after the complete chain-rule moment computation", "when storing into actuator_moment[i]", explicit no-double-scaling note. | Rev 8 |
