# Spec A — Defaults Completeness: Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_A.md`
**Implementation session(s):** Session 5
**Reviewer:** AI agent
**Date:** 2026-03-02

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

| Behavior | MuJoCo (from spec) | CortenForge Before | CortenForge After | Gap Closed? |
|----------|-------------------|--------------------|-------------------|-------------|
| Equality defaults cascade | `<default><equality solref="...">` applies to all 5 types via `OneEquality()` with `readingdefaults=true` | Not implemented — 5 hardcoded fallback sites, no defaults struct, class field ignored | `MjcfEqualityDefaults` struct (types.rs:1002), parsed in `parse_equality_defaults()` (parser.rs:1057), cascaded via `apply_eq_defaults()` at all 5 builder arms (equality.rs:83/160/218/278/331) | Yes |
| `active` in equality defaults | Parsed as `bool` via `MapValue()+bool_map`, inheritable via class | `active: bool` (not `Option<bool>`) — cannot distinguish "not set" from "default true" | `active: Option<bool>` on all 5 equality structs (types.rs:1603/1695/1794/1907/1998), parsed with `.map(\|v\| v == "true")`, cascaded via equality defaults, resolved with `.unwrap_or(true)` | Yes |
| `qpos_spring` for hinge/slide | `m->qpos_spring[padr] = springref` | Uses `jnt_springref[jnt_id]` — numerically equivalent for hinge/slide but different index space | `qpos_spring: Vec<f64>` on Model (model.rs:745), populated from `springref` (joint.rs:175/180), runtime reads `qpos_spring[qpos_adr]` (passive.rs:804, energy.rs:46) | Yes |
| `qpos_spring` for ball | `[1,0,0,0]` copied from `qpos0` | Not implemented — no `qpos_spring` array exists | Ball pushes `qpos0_ball` identity quaternion (joint.rs:187) — same source variable as qpos0 push | Yes |
| `qpos_spring` for free | `pos[3]+quat[4]` copied from `qpos0` | Not implemented | Free pushes `qpos0_free` 7D pose (joint.rs:196) — same source variable as qpos0 push | Yes |
| Tendon sentinel resolution | Evaluates at `qpos_spring` configuration | Evaluates at `qpos0` — divergent when `springref != 0` | Sentinel resolution reads `model.qpos_spring[dof_adr]` (build.rs:504) | Yes |
| `implicit_springref` init | Reads from `qpos_spring` per-DOF | Reads from `jnt_springref` per-joint | Reads `qpos_spring[jnt_qpos_adr[jnt_id]]` (model_init.rs:824) | Yes |
| Actuator shortcut defaults | `cylinder|muscle|adhesion|damper|intvelocity` dispatched to `OneActuator()` | Missing from `parse_default()` match — silently dropped | All 5 shortcut names added to match arms (parser.rs:522/559), dispatching to `parse_actuator_defaults()` | Yes |
| Type-specific default fields | Stored on single `mjsActuator` per class | `MjcfActuatorDefaults` lacks `area`, `diameter`, `bias`, `timeconst`, muscle params, `gain` | 14 type-specific fields added (types.rs:729-761), parsed (parser.rs:819-854), merged (defaults.rs:893-908), cascaded via sentinel detection (defaults.rs:441-512) | Yes |

**Unclosed gaps:** None

---

## 2. Spec Section Compliance

### S1. DT-2 — Equality Defaults: Struct and Parse

**Grade:** A+

**Spec says:**
Create `MjcfEqualityDefaults` struct with `active: Option<bool>`, `solref: Option<[f64; 2]>`,
`solimp: Option<[f64; 5]>`. Add `equality: Option<MjcfEqualityDefaults>` to `MjcfDefault`.
Change `active` from `bool` to `Option<bool>` on all 5 equality structs. Parse `<equality>`
in `parse_default()`. Create `parse_equality_defaults()`. Update 5 equality `active` parsers
from `!= "false"` to `== "true"`. Export `MjcfEqualityDefaults` in `lib.rs`.

**Implementation does:**
`MjcfEqualityDefaults` created at types.rs:1002-1008 with correct field types.
`equality: Option<MjcfEqualityDefaults>` added to `MjcfDefault` at types.rs:578.
All 5 equality structs have `active: Option<bool>` (types.rs:1603/1695/1794/1907/1998).
`b"equality"` added to `parse_default()` for both Open (parser.rs:541) and Empty (parser.rs:578) events.
`parse_equality_defaults()` created at parser.rs:1057-1076 with correct `v == "true"` comparison.
All 5 equality parsers use `.map(|v| v == "true")` pattern (parser.rs:2400/2449/2488/2527/~2563).
`MjcfEqualityDefaults` exported in lib.rs:189.

**Gaps (if any):** None

**Action:** None

### S2. DT-2 — Equality Defaults: Merge and Cascade

**Grade:** A+

**Spec says:**
Add `merge_equality_defaults()` (parent/child merge with `c.field.or(p.field)`). Add `equality`
to `merge_defaults()`. Add `equality_defaults()` accessor. Add `apply_equality_defaults()` taking
`(&mut Option<bool>, &mut Option<[f64;2]>, &mut Option<[f64;5]>)` — generic across all 5 types.

**Implementation does:**
`merge_equality_defaults()` at defaults.rs:1015-1029 with all four match cases and
correct `c.field.or(p.field)` pattern for all 3 fields.
`equality` added to `merge_defaults()` constructor.
`equality_defaults()` accessor at defaults.rs:127-130.
`apply_equality_defaults()` at defaults.rs:136-154 with correct "fill-None-only" semantics.

**Gaps (if any):** None

**Action:** None

### S3. DT-2 — Equality Defaults: Builder Integration

**Grade:** A+

**Spec says:**
Pass resolver into `process_equality_constraints()`. At each of 5 match arms, extract
`active/solref/solimp` into mutable locals, call `apply_equality_defaults()`, then use
cascaded values. `unwrap_or(DEFAULT_SOLREF/SOLIMP)` fallbacks remain for no-class+no-attr case.
Update `active` from `bool` push to `.unwrap_or(true)` push.

**Implementation does:**
Local helper `apply_eq_defaults()` at equality.rs:15-32 (identical semantics to spec's approach).
Called at all 5 match arms (equality.rs:83/160/218/278/331). Each retrieves equality defaults
via `resolver.equality_defaults()`, calls `apply_eq_defaults()`, then uses cascaded values.
`active` uses `.unwrap_or(true)` at all 5 sites (equality.rs:103/174/232/292/345).
`solref`/`solimp` retain `unwrap_or(DEFAULT_*)` fallbacks for no-class+no-attr case.

**Gaps (if any):** None. Implementation uses a local helper function rather than calling
the resolver method directly at each arm — this is a valid structural choice that reduces
code duplication while preserving identical behavior.

**Action:** None

### S4. DT-13 — `qpos_spring`: Model Array

**Grade:** A+

**Spec says:**
Add `qpos_spring: Vec<f64>` to Model (sized `nq`, same as `qpos0`). Init as empty vec in
`model_init.rs`.

**Implementation does:**
`qpos_spring: Vec<f64>` at model.rs:745-750 with complete doc comment describing per-joint-type
semantics and MuJoCo reference. Empty init at model_init.rs:312.

**Gaps (if any):** None

**Action:** None

### S5. DT-13 — `qpos_spring`: Builder Population

**Grade:** A+

**Spec says:**
Add `qpos_spring_values: Vec<f64>` accumulator to `ModelBuilder`. Populate during joint building:
hinge/slide push `springref` scalar, ball pushes `[1,0,0,0]` from qpos0, free pushes 7D from qpos0.
Populate for flex vertex slide joints (`push(0.0)`). Convert to Model in `build.rs`. Update tendon
sentinel to read `qpos_spring` instead of `qpos0`.

**Implementation does:**
Accumulator `qpos_spring_values: Vec<f64>` at builder/mod.rs:584-585, init at init.rs:170-171.
Joint population (joint.rs:164-198): Hinge pushes `spring_ref_converted` (angle-converted,
line 175). Slide pushes `spring_ref_converted` (line 180). Ball pushes from same `qpos0_ball`
variable (line 187). Free pushes from same `qpos0_free` variable (line 196).
Flex vertex slide joints push 0.0 (flex.rs:219).
Accumulator converted to Model at build.rs:348.
Tendon sentinel uses `model.qpos_spring[dof_adr]` (build.rs:504-505) with MuJoCo reference
comment citing `setSpring()`.

**Gaps (if any):** None

**Action:** None

### S6. DT-13 — `qpos_spring`: Consumer Migration

**Grade:** A+

**Spec says:**
Update `passive.rs` spring force from `jnt_springref[jnt_id]` to `qpos_spring[qpos_adr]`.
Update `energy.rs` spring energy likewise. Update `model_init.rs` `implicit_springref` to read
`qpos_spring[jnt_qpos_adr[jnt_id]]`. Ball/free spring force/energy remain as stubs (Spec B scope).

**Implementation does:**
passive.rs:804 reads `model.qpos_spring[qpos_adr]` (was `jnt_springref[jnt_id]`).
energy.rs:46 reads `model.qpos_spring[qpos_adr]` (was `jnt_springref[jnt_id]`).
model_init.rs:824 reads `self.qpos_spring[self.jnt_qpos_adr[jnt_id]]` (was `self.jnt_springref[jnt_id]`).
Ball/free stubs remain untouched (Spec B scope).

**Gaps (if any):** None

**Action:** None

### S7. DT-13 — `qpos_spring`: Test Model Factory Updates

**Grade:** A+

**Spec says:**
~18 sites that push `jnt_springref` must also push `qpos_spring`. Hinge/slide: same scalar.
Free: 7D `[pos+identity_quat]` or matching `qpos0`. Affects `model_factories.rs` (3 sites),
`sensor/mod.rs` (3 sites), `jacobian.rs` (7 sites), `constraint/jacobian.rs` (2 sites),
`forward/muscle.rs` (6 sites).

**Implementation does:**
23 factory push sites updated across 5 files:
- model_factories.rs: 3 factories (n_link_pendulum:95, spherical_pendulum:197, free_body:287)
- sensor/mod.rs: 3 factories (make_sensor_test_model:106, free:1023, ball:1142)
- jacobian.rs: 9 factories (mixed joint types, free, ball, hinge — lines 390/480/552/719/839/952/1189/1323/1474)
- constraint/jacobian.rs: 2 factories (free joint — lines 390/480)
- forward/muscle.rs: 6 factories (hinge — lines 813/1070/1390/1735/1900/2044)

All correctly sized per joint type (hinge: scalar, ball: 4D quat, free: 7D pos+quat).

**Gaps (if any):** Spec estimated ~18 sites, actual is 23. Extra sites are in jacobian.rs
which has more test model variants than spec estimated. No correctness gap.

**Action:** None

### S8. DT-14 — Actuator Defaults: Parser Extension

**Grade:** A+

**Spec says:**
Extend `parse_default()` match arms to include `b"cylinder"|b"muscle"|b"adhesion"|b"damper"|b"intvelocity"`.
Add 14 type-specific fields to `MjcfActuatorDefaults` (area, diameter, timeconst, bias,
muscle_timeconst, range, force, scale, lmin, lmax, vmax, fpmax, fvmax, gain). Parse them in
`parse_actuator_defaults()`. Handle `timeconst` ambiguity (1 value → cylinder, 2 → muscle).

**Implementation does:**
All 5 shortcut names added to `parse_default()` match for both Start (parser.rs:522-524)
and Empty (parser.rs:559-560) events.
14 type-specific fields added to `MjcfActuatorDefaults` (types.rs:729-761):
cylinder (area/diameter/timeconst/bias), muscle (muscle_timeconst/range/force/scale/lmin/lmax/vmax/fpmax/fvmax), adhesion (gain).
All parsed in `parse_actuator_defaults()` (parser.rs:819-854).
`timeconst` ambiguity handled correctly: 1 value → `timeconst`, 2+ values → `muscle_timeconst`
(parser.rs:823-830).

**Gaps (if any):** None

**Action:** None

### S9. DT-14 — Actuator Defaults: Merge and Cascade

**Grade:** A+

**Spec says:**
Extend `merge_actuator_defaults()` with 14 `c.field.or(p.field)` lines. Extend
`apply_to_actuator()` with sentinel-based detection for non-Option fields (area default 1.0,
bias default [0,0,0], etc.) and Option-based cascade for Option fields. Known limitation:
sentinel detection is imperfect when user explicitly sets field to its default value.

**Implementation does:**
14 fields merged in `merge_actuator_defaults()` (defaults.rs:893-908) using `c.field.or(p.field)`.
14 fields cascaded in `apply_to_actuator()` (defaults.rs:441-512):
- area: sentinel (== 1.0) at line 441
- diameter: Option at line 446
- timeconst: Option at line 449
- bias: sentinel (== [0,0,0]) at line 452
- muscle_timeconst: sentinel (== (0.01, 0.04)) at line 459
- range: sentinel (== (0.75, 1.05)) at line 466
- force: sentinel (== -1.0) at line 471
- scale: sentinel (== 200.0) at line 476
- lmin: sentinel (== 0.5) at line 481
- lmax: sentinel (== 1.6) at line 486
- vmax: sentinel (== 1.5) at line 491
- fpmax: sentinel (== 1.3) at line 496
- fvmax: sentinel (== 1.2) at line 501
- gain: sentinel (== 1.0) at line 508

All sentinel values match MuJoCo defaults exactly. Known limitation documented.

**Gaps (if any):** None

**Action:** None

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | Equality defaults — solref cascade | T1 `t1_equality_defaults_solref_solimp_cascade` | Pass | Asserts `eq_solref[0] == [0.05, 0.8]` on connect + weld |
| AC2 | Equality defaults — solimp cascade | T1 `t1_equality_defaults_solref_solimp_cascade` | Pass | Asserts 5-element solimp on connect + weld |
| AC3 | Equality defaults — active cascade | T2 `t2_equality_defaults_active_false_cascade` | Pass | Asserts `eq_active[0] == false` |
| AC4 | Equality defaults — no class → hardcoded default | T3 `t3_equality_defaults_no_class_builtin` | Pass | Asserts MuJoCo built-in [0.02, 1.0] / [0.9, 0.95, 0.001, 0.5, 2.0] |
| AC5 | `qpos_spring` — hinge joint with springref | T4 `t4_qpos_spring_hinge_springref` | Pass | Asserts `qpos_spring[qpos_adr] == 0.5` |
| AC6 | `qpos_spring` — ball joint | T5 `t5_qpos_spring_ball_identity` | Pass | Asserts identity quaternion [1,0,0,0] |
| AC7 | `qpos_spring` — free joint | T6 `t6_qpos_spring_free_body_pose` | Pass | Non-identity quat proves 7D copy from qpos0 |
| AC8 | `qpos_spring` — hinge/slide spring force regression | T7 `t7_spring_force_regression_hinge` | Pass | `qfrc_spring[0] == -50.0 ± 1e-12` |
| AC9 | Actuator defaults — cylinder area cascade | T8 `t8_cylinder_area_defaults_cascade` | Pass | `gainprm[0][0] == 0.01` |
| AC10 | Actuator defaults — muscle params cascade | T9 `t9_muscle_range_defaults_cascade` | Pass | `gainprm[0] == 0.5, gainprm[1] == 1.2` |
| AC11 | Tendon sentinel resolution uses `qpos_spring` | T10 `t10_tendon_sentinel_uses_qpos_spring` | Pass | `lengthspring == [0.3, 0.3]`, `length0 == 0.0` — proves qpos_spring diverges from qpos0 |
| AC12 | No regression (full domain suite) | Domain test run | Pass | 2,344 tests passed, 0 failures |
| AC13 | Adhesion defaults — gain cascade | T14 `t14_adhesion_gain_defaults_cascade` | Pass | `gainprm[0][0] == 0.5` |
| AC14 | Damper defaults — kv cascade | T15 `t15_damper_kv_defaults_cascade` | Pass | `gainprm[0][2] == -5.0` |
| AC15 | Ball `qpos_spring` copies from `qpos0`, not hardcoded (code review) | — | Pass | joint.rs:184-187: ball uses `qpos0_ball` for both qpos0 and qpos_spring |
| AC16 | `MjcfEqualityDefaults` exported (code review) | — | Pass | lib.rs:189 exports `MjcfEqualityDefaults` |

**Missing or failing ACs:** None

---

## 4. Test Plan Completeness

> **Test numbering note:** Spec uses T1–T15. All 15 tests implemented in
> `sim/L0/tests/integration/phase7_spec_a.rs` (516 lines).

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Equality defaults — solref/solimp cascade (AC1, AC2) | Yes | `t1_equality_defaults_solref_solimp_cascade` | 13 assertions with epsilon 1e-10 |
| T2 | Equality defaults — active=false cascade (AC3) | Yes | `t2_equality_defaults_active_false_cascade` | 1 assertion |
| T3 | Equality defaults — no class → MuJoCo built-in (AC4) | Yes | `t3_equality_defaults_no_class_builtin` | 8 assertions (solref + solimp) |
| T4 | `qpos_spring` — hinge with explicit springref (AC5) | Yes | `t4_qpos_spring_hinge_springref` | 1 assertion |
| T5 | `qpos_spring` — ball joint identity quaternion (AC6) | Yes | `t5_qpos_spring_ball_identity` | 4 assertions (w,x,y,z) |
| T6 | `qpos_spring` — free joint body pose (AC7) | Yes | `t6_qpos_spring_free_body_pose` | 7 assertions (pos+quat); non-identity quat proves copy |
| T7 | Spring force regression — hinge/slide unchanged (AC8) | Yes | `t7_spring_force_regression_hinge` | Runs `forward()`, checks `qfrc_spring` |
| T8 | Cylinder area defaults cascade (AC9) | Yes | `t8_cylinder_area_defaults_cascade` | 1 assertion |
| T9 | Muscle range defaults cascade (AC10) | Yes | `t9_muscle_range_defaults_cascade` | 2 assertions (gainprm[0], gainprm[1]) |
| T10 | Tendon sentinel uses `qpos_spring` (AC11) | Yes | `t10_tendon_sentinel_uses_qpos_spring` | 4 assertions; proves qpos0 vs qpos_spring divergence |
| T11 | Equality defaults — nested class inheritance (supplementary) | Yes | `t11_equality_defaults_nested_class_inheritance` | 8 assertions; verifies merge logic |
| T12 | Multiple actuator shortcuts in one default (supplementary) | Yes | `t12_multiple_actuator_shortcuts_last_wins` | 1 assertion; last-wins semantics |
| T13 | Mixed joint types — `qpos_spring` alignment (supplementary) | Yes | `t13_mixed_joint_types_qpos_spring_alignment` | 14 assertions + length check; hinge+slide+ball+free |
| T14 | Adhesion gain defaults cascade (AC13) | Yes | `t14_adhesion_gain_defaults_cascade` | 1 assertion |
| T15 | Damper kv defaults cascade (AC14) | Yes | `t15_damper_kv_defaults_cascade` | 1 assertion; verifies gainprm[2] = -kv expansion |

### Supplementary Tests

Tests that were added beyond the spec's planned test list — additional
coverage discovered during implementation or review.

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| (none) | All tests match spec plan exactly | — | No supplementary tests needed |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| Equality constraint with NO class, no explicit solref | Must get hardcoded default `[0.02, 1.0]` | Yes | T3 | AC4 direct coverage |
| Nested class inheritance for equality | Child overrides solimp, inherits parent solref | Yes | T11 | Supplementary coverage |
| `qpos_spring` for ball (identity quaternion from qpos0) | Ball ignores `springref`; must copy qpos0 | Yes | T5 | AC6 direct coverage |
| `qpos_spring` for free with non-identity quat | Free copies full 7D pose from qpos0 (non-identity quat proves actual copy) | Yes | T6 | AC7 — non-identity quat is key |
| Hinge springref=0 → `qpos_spring[adr]==0` same as `qpos0[adr]` | Regression: values numerically identical to before | Yes | T7 | AC8 regression |
| Multiple actuator shortcuts in one `<default>` | Last wins (struct replacement at parse level) | Yes | T12 | Supplementary coverage |
| Tendon sentinel with springref!=0 | Only test that proves qpos0→qpos_spring matters | Yes | T10 | AC11 — springref=0.3 diverges from ref=0.0 |
| `<default><equality>` does NOT cascade data/anchor/polycoef | These are type-specific, not in defaults context | Yes | T3 | Structural: `parse_equality_defaults()` only parses 3 fields |
| `active` defaults cascade (Option<bool> migration) | Previous `bool` type prevented cascade | Yes | T2 | AC3 direct coverage |
| Mixed joint types qpos_spring alignment | Different nq per joint type must align | Yes | T13 | Supplementary — 4 joint types in one model |
| Element-level override beats class default | `<connect solref="0.1 0.5" class="X">` should use 0.1/0.5 not X's default | Structural | — | Guaranteed by "fill-None-only" cascade pattern |
| `<default><cylinder area="0.01"/>` then `<cylinder class="...">` | Cylinder-specific defaults cascade | Yes | T8 | AC9 direct coverage |
| `<default><adhesion gain="0.5"/>` then `<adhesion class="...">` | Adhesion-specific defaults cascade | Yes | T14 | AC13 direct coverage |
| `<default><damper kv="5.0"/>` then `<damper class="...">` | Damper-specific defaults cascade; expansion maps kv→gainprm[2]=-kv | Yes | T15 | AC14 direct coverage |
| Ball `qpos_spring` copies from `qpos0` structurally | Prevents hardcoded identity quat diverging from actual qpos0 in future | Code review | — | AC15: joint.rs uses same `qpos0_ball` variable |
| Muscle sentinel detection (`gainprm[0]==1` → 0.75 overwrite) | MuJoCo quirk only manifests for `<general dyntype="muscle">` path; documented as known divergence — see Out of Scope | N/A | — | Deferred (Out of Scope) |

**Missing tests:** None

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| Equality constraints inherit from default classes (hardcoded fallback → class cascade then fallback) | Yes | T1, T2, T3, T11 confirm |
| `active` type on equality structs changes from `bool` to `Option<bool>` | Yes | All 5 structs updated; `.unwrap_or(true)` at builder sites |
| Spring force reads `qpos_spring` instead of `jnt_springref` | Yes | passive.rs:804 confirmed |
| Tendon sentinel resolution uses `qpos_spring` instead of `qpos0` | Yes | build.rs:504 confirmed; T10 proves divergence |
| `implicit_springref` source changes from `jnt_springref[jnt_id]` to `qpos_spring[qpos_adr]` | Yes | model_init.rs:824 confirmed |
| Actuator defaults parse `cylinder|muscle|adhesion|damper|intvelocity` (previously silently dropped) | Yes | parser.rs:522/559 confirmed |
| Actuator type-specific defaults cascade via `apply_to_actuator()` (new code path) | Yes | defaults.rs:441-512 confirmed; T8/T9/T14/T15 verify |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/mjcf/src/types.rs` | Yes (+85 lines) | |
| `sim/L0/mjcf/src/parser.rs` | Yes (+122/-109 lines) | |
| `sim/L0/mjcf/src/defaults.rs` | Yes (+148 lines) | |
| `sim/L0/mjcf/src/builder/equality.rs` | Yes (+107 lines) | |
| `sim/L0/mjcf/src/builder/mod.rs` | Yes (+2 lines) | |
| `sim/L0/mjcf/src/builder/joint.rs` | Yes (+28 lines) | |
| `sim/L0/mjcf/src/builder/flex.rs` | Yes (+2 lines) | |
| `sim/L0/mjcf/src/builder/build.rs` | Yes (+30 lines) | |
| `sim/L0/mjcf/src/builder/init.rs` | Yes (+1 line) | |
| `sim/L0/mjcf/src/lib.rs` | Yes (+9 lines) | |
| `sim/L0/core/src/types/model.rs` | Yes (+6 lines) | |
| `sim/L0/core/src/types/model_init.rs` | Yes (+3 lines) | |
| `sim/L0/core/src/forward/passive.rs` | Yes (+2 lines) | |
| `sim/L0/core/src/energy.rs` | Yes (+2 lines) | |
| `sim/L0/core/src/types/model_factories.rs` | Yes (+5 lines) | |
| Various test files (~15 sites) | Yes (23 sites across 5 files) | |
| Test file(s) for Spec A (~200 lines) | Yes (516 lines) | |

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| `sim/L0/tests/integration/tendon_springlength.rs` (+15 lines) | Pre-existing tendon test needed `qpos_spring` entries added to test model — same pattern as S7 factory updates |
| `sim/L0/tests/integration/mod.rs` (+8 lines) | Registering new `phase7_spec_a` test module |

Both unexpected files are test infrastructure — no production code surprises.

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| `test_equality_connect` | Pass — values unchanged (no class defaults in test models) | Pass | No |
| `test_equality_weld` | Pass — values unchanged | Pass | No |
| Spring force tests | Pass — values unchanged (hinge/slide: `qpos_spring == jnt_springref` when both 0.0) | Pass | No |
| Energy tests | Pass — values unchanged | Pass | No |
| Tendon tests | Pass — values unchanged (test models use springref=0) | Pass | No |
| Model factory tests | Compile error → fix (must add `qpos_spring` field) | Fixed (3 factories) | No |
| Jacobian tests | Compile error → fix (must add `qpos_spring` to test models) | Fixed (9 sites) | No — more sites than predicted (~7 vs 9) |
| Muscle tests | Compile error → fix (must add `qpos_spring` to test models) | Fixed (6 sites) | No |
| Sensor tests | Compile error → fix (must add `qpos_spring` to test models) | Fixed (3 sites) | No |
| Constraint tests | Compile error → fix (must add `qpos_spring` to test models) | Fixed (2 sites) | No |

**Unexpected regressions:** None

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| `qpos_spring` index space: `model.qpos_spring[qpos_adr]` where `qpos_adr = jnt_qpos_adr[jnt_id]` | Direct port — same index space. Use `jnt_qpos_adr` (not `jnt_dof_adr`) | Yes | passive.rs:804, energy.rs:46, model_init.rs:824 all use correct index space |
| `jnt_springref` vs `qpos_spring`: keep `jnt_springref` as builder intermediate; runtime reads `qpos_spring` | Runtime code reads `qpos_spring`; `jnt_springref` stays for builder use only | Yes | `jnt_springref` still pushed in builder; runtime migrated to `qpos_spring` |
| `solref` size: `[f64; 2]` | Direct port | Yes | types.rs:1004 |
| `solimp` size: `[f64; 5]` | Direct port | Yes | types.rs:1006 |
| Equality default struct: single `MjcfEqualityDefaults` per class, applies to all 5 types | Direct port | Yes | Single struct, one `apply_eq_defaults()` for all 5 arms |
| Actuator default struct: single `MjcfActuatorDefaults` per class, all shortcuts write to same struct | Direct port | Yes | All 10 shortcut names dispatch to same `parse_actuator_defaults()` |
| Shortcut expansion timing: deferred to build time | Deferred expansion produces identical results because same mapping logic | Yes | Defaults store ergonomic fields; builder expands to gainprm/biasprm |
| `springref` default: `0.0` (from memset zero; NOT NaN or sentinel) | Direct port — `0.0` is the default when no `springref` attribute specified | Yes | `joint.spring_ref.unwrap_or(0.0)` in builder |
| `bool_map`: only `"true"`/`"false"` — standardize to `== "true"` when touching equality active sites | Standardize to `== "true"` | Yes | All 5 equality parsers use `.map(\|v\| v == "true")` |

---

## 7. Weak Implementation Inventory

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|
| (none) | — | No weak implementations found | — | — |

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| DT-11 (Joint `range` default) — already implemented | Out of Scope, bullet 1 | N/A — already implemented | DT-11 | Yes — verified via rubric EGT-4; `range` on `MjcfJointDefaults` (types.rs:606), parsed (parser.rs:611), cascaded (defaults.rs:169), applied (builder/joint.rs:265) |
| Ball/free spring force computation (`mji_subQuat` path) | Out of Scope, bullet 2 | PHASE7_UMBRELLA.md:113-119 | §64 (Spec B) | Yes |
| Ball/free spring energy computation | Out of Scope, bullet 3 | PHASE7_UMBRELLA.md:113-119 | §64 (Spec B) | Yes |
| `IntVelocity` enum variant (actuator type completeness) | Out of Scope, bullet 4 | future_work_10b.md; ROADMAP_V1.md | DT-123 | Yes — Low priority, T1. Defaults parsing works; concrete elements need enum variant. |
| Muscle sentinel detection for `<general>` path (`gainprm[0]==1`) | Out of Scope, bullet 5 | future_work_10b.md; ROADMAP_V1.md | DT-124 | Yes — Low priority, T1. Known conformance divergence for rare edge case. |
| Migration of `MjcfActuator` type-specific fields from non-Option to Option | Out of Scope, bullet 6 | future_work_10b.md; PHASE7_UMBRELLA.md:461; ROADMAP_V1.md | DT-15 | Yes — Low priority, T1. Spec A added 14 sentinel fields as primary migration candidates. |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| (none) | — | — | — | — |

### Discovered During Review

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| (none) | — | — | — | — |

### Spec Gaps Found During Implementation

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|
| (none) | — | — | — |

---

## 9. Test Coverage Summary

**Domain test results:**
```
cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics
  -p sim-constraint -p sim-muscle -p sim-tendon -p sim-sensor -p sim-urdf
  -p sim-types -p sim-simd

2,344 passed; 0 failed; 22 ignored
```

**New tests added:** 15 (T1–T15 in `sim/L0/tests/integration/phase7_spec_a.rs`, 516 lines)
**Tests modified:** ~23 test model factory sites across 6 files (qpos_spring entries)
**Pre-existing test regressions:** None

**Clippy:** Clean (0 warnings with `-D warnings`)
**Fmt:** Clean

---

## 10. Review Verdict

| Category | Section | Status |
|----------|---------|--------|
| Key behaviors gap closure | 1 | All 9 gaps closed |
| Spec section compliance | 2 | All 9 sections A+ |
| Acceptance criteria | 3 | All 16 ACs pass |
| Test plan completeness | 4 | All 15 tests implemented |
| Blast radius accuracy | 5 | All predictions correct; 2 minor expected extras |
| Convention fidelity | 6 | All 9 conventions followed |
| Weak items | 7 | None found |
| Deferred work tracking | 8 | All 6 items tracked |
| Test health | 9 | 2,344 pass / 0 fail / clean clippy+fmt |

**Overall:** Ship

**Items fixed during review:** None needed

**Items to fix before shipping:** None

**Items tracked for future work:**
- §64 (ball/free spring force + energy) → Spec B scope. Depends on Spec A `qpos_spring`. Updated in future_work_15.md + ROADMAP_V1.md.
- DT-15 (Option migration for actuator fields) → post-v1.0. Updated to cite Spec A's 14 sentinel fields as primary candidates.
- DT-123 (IntVelocity enum variant) → post-v1.0. New entry in future_work_10b.md + ROADMAP_V1.md + index.md.
- DT-124 (Muscle sentinel detection for `<general>` path) → post-v1.0. New entry in future_work_10b.md + ROADMAP_V1.md + index.md.
