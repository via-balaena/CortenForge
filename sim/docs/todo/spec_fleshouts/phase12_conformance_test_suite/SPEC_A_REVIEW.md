# Layer B Per-Stage Reference Comparison — Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/SPEC_A.md`
**Implementation session(s):** Session 7
**Reviewer:** AI agent
**Date:** 2026-03-09

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
| FK body poses from joint angles | `mj_kinematics()` computes xpos/xquat/xipos for all bodies via kinematic tree traversal | `data.forward(model)` → `mj_fwd_position()` → same tree traversal. Fields: `data.xpos: Vec<Vector3<f64>>`, `data.xquat: Vec<UnitQuaternion<f64>>`, `data.xipos: Vec<Vector3<f64>>` | Tests written for all 8 models. contact_scenario passes. 7 models `#[ignore]`d — xipos not computed (body CoM in world frame is zero). xpos and xquat appear correct, but xipos failure causes test abort before full validation. | **Partial** — xipos gap blocks 7/8 models |
| Dense mass matrix | `mj_crb()` computes sparse `qM`, then `mj_fullM()` unpacks to dense nv×nv | `data.qM: DMatrix<f64>` stored dense. Direct comparison via `[(i,j)]` indexing | Tests written for all 8 models. contact_scenario passes. 7 models `#[ignore]`d — parallel axis theorem contribution missing (depends on xipos). | **Partial** — xipos cascades to CRBA for 7/8 models |
| Coriolis + gravity bias | `mj_rne()` backward/forward Newton-Euler on kinematic tree | `data.qfrc_bias: DVector<f64>`. Same algorithm | Tests written for all 8 models. contact_scenario passes. 7 models `#[ignore]`d — gravity torques use wrong CoM (xipos not computed). | **Partial** — xipos cascades to RNE for 7/8 models |
| Passive spring/damper forces | `mj_passive()` computes spring + damper forces per joint | `data.qfrc_passive: DVector<f64>`. Same algorithm | All 8 tests pass at TOL_PASSIVE=1e-10. Full conformance. | **Yes** |
| Contact detection | `mj_collision()` iterates broadphase geom pairs, runs narrowphase | `data.contacts: Vec<Contact>`. Geom pair order may differ — structural comparison needed | Tests written for 2 models. Both `#[ignore]`d — contact.pos z off by 1e-3 (convention difference). Geom pair matching and depth sign flip work correctly. | **Partial** — contact position convention gap |
| Contact depth convention | `d->contact[i].dist`: negative = penetration | `data.contacts[i].depth`: positive = penetration. **Sign flip required** | Sign flip correctly implemented in `run_collision_test()` as `-ref_depth[ri]`. Test infrastructure correct even though tests are `#[ignore]`d. | **Yes** (infrastructure correct) |
| Contact normal | `d->contact[i].frame[0:3]` is contact normal | `data.contacts[i].normal: Vector3<f64>`. Direct match | Comparison implemented. Tests `#[ignore]`d (pos fails before normal comparison completes for contact_scenario). | **Untested** (blocked by pos failure) |
| Constraint Jacobian | `mj_makeConstraint()` assembles efc_J rows. Order: equality → friction loss → joint limits → tendon limits → contacts | `data.efc_J: DMatrix<f64>`. Same row ordering (verified in `assembly.rs:47-53`). Element-wise comparison valid when nefc matches | Tests written for 3 models. All `#[ignore]`d — efc_J row content fundamentally differs. nefc matches but row contents diverge at element [0]. | **No** — constraint Jacobian assembly divergence |
| Constraint forces | `mj_projectConstraint()` iterative solver | `data.efc_force: DVector<f64>`. Looser tolerance (1e-4) for solver convergence | Tests `#[ignore]`d — blocked by efc_J mismatch (test aborts before reaching efc_force comparison). | **Untested** (blocked by efc_J failure) |
| Actuator forces | `mj_fwdActuation()` applies gain/bias → transmission to joint space | `data.qfrc_actuator: DVector<f64>`, `data.actuator_force: Vec<f64>`. Direct computation | Both tests pass at TOL_ACTUATION=1e-10. Full conformance. | **Yes** |
| Sensor data | `mj_sensorPos/Vel/Acc()` writes to contiguous sensordata array | `data.sensordata: DVector<f64>`. Sensor order follows model definition | Tests written for 2 models. Both `#[ignore]`d — sensordata off at specific indices (framepos sensor affected by xipos/subtree_com). | **Partial** — xipos cascades to sensors |
| Tendon kinematics | `mj_tendon()` computes lengths and velocities | `data.ten_length: Vec<f64>`, `data.ten_velocity: Vec<f64>`. Direct computation | Both tests pass at TOL_TENDON=1e-10. Full conformance. | **Yes** |

**Unclosed gaps:**

1. **xipos not computed** — Root cause affecting FK (7 models), CRBA (7 models), RNE (7 models), sensors (2 models). 23 of 28 `#[ignore]`d tests trace back to this. Source phase: Phase 1 (FK).
2. **Contact position convention** — contact.pos z off by 1e-3. 2 `#[ignore]`d tests. Source phase: Phase 3 (collision).
3. **Constraint Jacobian assembly** — efc_J row content fundamentally differs. 3 `#[ignore]`d tests. Source phase: Phase 3 (constraint).

---

## 2. Spec Section Compliance

Walk through every spec section (S1, S2, ...) and grade the implementation
against it. This is the core of the review.

**How to grade each section:**

- Read the spec section (algorithm, file path, MuJoCo equivalent, design
  decision, before/after code).
- Read the actual implementation.
- Compare. Grade honestly.

| Grade | Meaning |
|-------|---------|
| **Pass** | Implementation matches spec. Algorithm, file location, edge cases all correct. |
| **Weak** | Implementation works but deviates from spec, uses shortcuts, has loose tolerances, missing edge-case guards, or TODOs. **Fix before shipping.** |
| **Deviated** | Implementation intentionally diverged from spec (spec gap discovered during implementation). Deviation is documented and justified. Acceptable if the spec was updated. |
| **Missing** | Section not implemented. Must be either fixed or explicitly deferred with tracking. |

### S1. Common Infrastructure (Layer B helpers in `common.rs`)

**Grade:** Pass

**Spec says:**
Extend `common.rs` with Layer B-specific helpers: `parse_npy_i32()`,
`reference_path()`, `load_conformance_model()`, `load_reference_f64()`,
`load_reference_i32()`, `assert_array_eq()`, `assert_quat_eq()`, and
`TOL_CONSTRAINT_JAC` constant.

**Implementation does:**
All 7 functions and 1 constant are present in `common.rs:97-241`. Function
signatures match the spec exactly. `parse_npy_i32()` handles `<i4` dtype
correctly. `assert_array_eq()` produces diagnostic output matching the spec's
format: `[{model}] {stage}.{field}[{index}]: expected {e}, got {a}, diff {d},
tol {t}`. `assert_quat_eq()` uses L∞ norm with sign ambiguity as specified.
`TOL_CONSTRAINT_JAC = 1e-8` at line 95.

**Gaps (if any):** None.

**Action:** None.

### S2. FK Reference Tests

**Grade:** Deviated (positive)

**Spec says:**
Test all 8 models for FK. Each test loads model, calls `forward()`, compares
`xpos`, `xquat`, `xipos` against reference data. Quaternion comparison uses
`assert_quat_eq()` with sign ambiguity. World body (body 0) always tested.
Models: pendulum, double_pendulum, contact_scenario, actuated_system,
tendon_model, sensor_model, equality_model, composite_model.

**Implementation does:**
Extracted `run_fk_test()` helper (lines 15–63) that encapsulates the full FK
comparison pattern. All 8 test functions exist (lines 403–448), each calling
`run_fk_test()` with the correct model name and ctrl values. xpos, xquat
(with `assert_quat_eq`), and xipos are compared for all nbody bodies. World
body (body 0) is tested as part of the full-body loop. 7 tests are
`#[ignore]`d with conformance gap tracking comments. 1 test (contact_scenario)
passes.

**Gaps (if any):** The spec showed inline test code per model; implementation
uses a shared helper. This is a positive deviation — same coverage, less code,
easier maintenance. All test function names match spec convention exactly.

**Action:** None.

### S3. CRBA Reference Tests

**Grade:** Pass

**Spec says:**
Compare dense mass matrix element-wise for all 8 models. Reference data uses
`mj_fullM()` to unpack sparse qM. Tolerance: `TOL_CRBA = 1e-12`.

**Implementation does:**
`run_crba_test()` helper (lines 69–95) loads reference qM, asserts square and
nv-matching, builds flat actual_qm via nested loop `[(i,j)]` indexing, and
calls `assert_array_eq()` at `TOL_CRBA`. All 8 tests present (lines 454–499).
7 `#[ignore]`d, 1 passes (contact_scenario).

**Gaps (if any):** None.

**Action:** None.

### S4. RNE Reference Tests

**Grade:** Pass

**Spec says:**
Compare qfrc_bias element-wise for all 8 models. Tolerance: `TOL_RNE = 1e-10`.

**Implementation does:**
`run_rne_test()` helper (lines 101–118) loads reference qfrc_bias, collects
actual via `(0..model.nv).map()`, calls `assert_array_eq()` at `TOL_RNE`. All
8 tests present (lines 505–550). 7 `#[ignore]`d, 1 passes (contact_scenario).

**Gaps (if any):** None.

**Action:** None.

### S5. Passive Force Reference Tests

**Grade:** Pass

**Spec says:**
Compare qfrc_passive element-wise for all 8 models. Tolerance:
`TOL_PASSIVE = 1e-10`. Non-zero passive forces for models with springs
(pendulum, double_pendulum, tendon_model, sensor_model, composite_model).
Zero passive forces for models without springs (contact_scenario,
actuated_system, equality_model).

**Implementation does:**
`run_passive_test()` helper (lines 124–141) loads reference, collects actual,
calls `assert_array_eq()` at `TOL_PASSIVE`. All 8 tests present (lines 557–
594). **All 8 pass** — full conformance for passive stage.

**Gaps (if any):** None.

**Action:** None.

### S6. Collision Reference Tests

**Grade:** Pass

**Spec says:**
Structural comparison — contacts matched by geom pair, not array index.
Sign flip for depth (`depth_cf ≈ -ref_dist`). Tolerance:
`TOL_COLLISION_DEPTH = 1e-6`. Models: contact_scenario (1 contact),
composite_model (2 contacts).

**Implementation does:**
`run_collision_test()` helper (lines 147–231) loads reference geom pairs
(i32), depth, normal, pos. Builds sorted contacts by `(geom1, geom2)` for
both reference and CortenForge. Asserts same contact count with diagnostic
geom pair listing on mismatch. Compares per matched pair: depth (with sign
flip `-ref_depth[ri]`), normal (3 components), position (3 components), all
at `TOL_COLLISION_DEPTH`. Both tests present (lines 644–654). Both
`#[ignore]`d — contact.pos z off by 1e-3.

**Gaps (if any):** None in the test infrastructure. The test correctly
identifies the conformance gap (pos[2] off).

**Action:** None.

### S7. Constraint Reference Tests

**Grade:** Pass

**Spec says:**
Element-wise comparison of efc_J, efc_b, efc_force with nefc pre-check.
Tolerances: efc_J/efc_b at `TOL_CONSTRAINT_JAC = 1e-8`, efc_force at
`TOL_CONSTRAINT = 1e-4`. Models: contact_scenario (4 efc rows),
equality_model (7 efc rows), composite_model (9 efc rows).

**Implementation does:**
`run_constraint_test()` helper (lines 237–304) loads efc_J (nefc×nv), checks
nefc match with diagnostic breakdown (equality/friction_loss/contact+limit
counts), asserts nv match, compares efc_J at `TOL_CONSTRAINT_JAC`. Then loads
and compares efc_b at `TOL_CONSTRAINT_JAC` and efc_force at `TOL_CONSTRAINT`.
All 3 tests present (lines 660–676). All `#[ignore]`d — efc_J row content
fundamentally diverges.

**Gaps (if any):** None in the test infrastructure. The test correctly
identifies the constraint Jacobian assembly divergence.

**Action:** None.

### S8. Actuation Reference Tests

**Grade:** Pass

**Spec says:**
Compare qfrc_actuator (nv) and actuator_force (nu) element-wise. Tolerance:
`TOL_ACTUATION = 1e-10`. Ctrl values must be set before `forward()`:
actuated_system `ctrl=[1.0, 0.5]`, composite_model `ctrl=[1.0]`.

**Implementation does:**
`run_actuation_test()` helper (lines 310–339) loads qfrc_actuator and
actuator_force references, compares at `TOL_ACTUATION`. Ctrl values set via
helper parameter. Both tests present (lines 601–608) with correct ctrl
values: `&[1.0, 0.5]` for actuated_system, `&[1.0]` for composite_model.
**Both pass.**

**Gaps (if any):** None.

**Action:** None.

### S9. Sensor Reference Tests

**Grade:** Pass

**Spec says:**
Compare sensordata array element-wise. Tolerance: `TOL_SENSOR = 1e-8`.
Models: sensor_model (nsensordata=21, 8 sensors), composite_model
(nsensordata=7, 3 sensors).

**Implementation does:**
`run_sensor_test()` helper (lines 345–363) loads reference sensordata,
collects actual via `(0..nsensordata).map()`, compares at `TOL_SENSOR`. Both
tests present (lines 629–638). Both `#[ignore]`d — sensordata off at
specific indices (sensor_model[12], composite_model[4]) due to
xipos/subtree_com affecting framepos sensor.

**Gaps (if any):** None in the test infrastructure.

**Action:** None.

### S10. Tendon Reference Tests

**Grade:** Pass

**Spec says:**
Compare ten_length and ten_velocity element-wise. Tolerance:
`TOL_TENDON = 1e-10`. Models: tendon_model (ntendon=1),
composite_model (ntendon=1).

**Implementation does:**
`run_tendon_test()` helper (lines 369–397) loads reference length and
velocity, compares directly against `data.ten_length` and
`data.ten_velocity` at `TOL_TENDON`. Both tests present (lines 615–622).
**Both pass.**

**Gaps (if any):** None.

**Action:** None.

---

## 3. Acceptance Criteria Verification

Walk through every AC from the spec. For runtime ACs, confirm the test
exists and passes. For code-review ACs, perform the review now.

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | Layer B infrastructure helpers (code review) | — (code review) | **Pass** | All 7 functions + 1 constant present in `common.rs:97-241`. Signatures and behavior match spec. |
| AC2 | FK pendulum | `layer_b_fk_pendulum` | **#[ignore]** | xipos not computed — Phase 1 FK |
| AC3 | FK double_pendulum | `layer_b_fk_double_pendulum` | **#[ignore]** | xipos not computed — Phase 1 FK |
| AC4 | FK contact_scenario | `layer_b_fk_contact_scenario` | **Pass** | xpos/xquat/xipos all match at 1e-12 |
| AC5 | FK actuated_system | `layer_b_fk_actuated_system` | **#[ignore]** | xipos not computed — Phase 1 FK |
| AC6 | FK tendon_model | `layer_b_fk_tendon_model` | **#[ignore]** | xipos not computed — Phase 1 FK |
| AC7 | FK sensor_model | `layer_b_fk_sensor_model` | **#[ignore]** | xipos not computed — Phase 1 FK |
| AC8 | FK equality_model | `layer_b_fk_equality_model` | **#[ignore]** | xipos not computed — Phase 1 FK |
| AC9 | FK composite_model | `layer_b_fk_composite_model` | **#[ignore]** | xipos not computed — Phase 1 FK |
| AC10 | CRBA pendulum | `layer_b_crba_pendulum` | **#[ignore]** | qM wrong — parallel axis theorem needs xipos — Phase 1 CRBA |
| AC11 | CRBA double_pendulum | `layer_b_crba_double_pendulum` | **#[ignore]** | qM wrong — Phase 1 CRBA |
| AC12 | CRBA contact_scenario | `layer_b_crba_contact_scenario` | **Pass** | qM matches at 1e-12 |
| AC13 | CRBA actuated_system | `layer_b_crba_actuated_system` | **#[ignore]** | qM wrong — Phase 1 CRBA |
| AC14 | CRBA tendon_model | `layer_b_crba_tendon_model` | **#[ignore]** | qM wrong — Phase 1 CRBA |
| AC15 | CRBA sensor_model | `layer_b_crba_sensor_model` | **#[ignore]** | qM wrong — Phase 1 CRBA |
| AC16 | CRBA equality_model | `layer_b_crba_equality_model` | **#[ignore]** | qM wrong — Phase 1 CRBA |
| AC17 | CRBA composite_model | `layer_b_crba_composite_model` | **#[ignore]** | qM wrong — Phase 1 CRBA |
| AC18 | RNE pendulum | `layer_b_rne_pendulum` | **#[ignore]** | qfrc_bias wrong — gravity torques use wrong CoM — Phase 1 RNE |
| AC19 | RNE double_pendulum | `layer_b_rne_double_pendulum` | **#[ignore]** | qfrc_bias wrong — Phase 1 RNE |
| AC20 | RNE contact_scenario | `layer_b_rne_contact_scenario` | **Pass** | qfrc_bias matches at 1e-10 |
| AC21 | RNE actuated_system | `layer_b_rne_actuated_system` | **#[ignore]** | qfrc_bias wrong — Phase 1 RNE |
| AC22 | RNE tendon_model | `layer_b_rne_tendon_model` | **#[ignore]** | qfrc_bias wrong — Phase 1 RNE |
| AC23 | RNE sensor_model | `layer_b_rne_sensor_model` | **#[ignore]** | qfrc_bias wrong — Phase 1 RNE |
| AC24 | RNE equality_model | `layer_b_rne_equality_model` | **#[ignore]** | qfrc_bias wrong — Phase 1 RNE |
| AC25 | RNE composite_model | `layer_b_rne_composite_model` | **#[ignore]** | qfrc_bias wrong — Phase 1 RNE |
| AC26 | Passive pendulum | `layer_b_passive_pendulum` | **Pass** | qfrc_passive matches at 1e-10 |
| AC27 | Passive double_pendulum | `layer_b_passive_double_pendulum` | **Pass** | qfrc_passive matches at 1e-10 |
| AC28 | Passive contact_scenario | `layer_b_passive_contact_scenario` | **Pass** | All zeros as expected |
| AC29 | Passive actuated_system | `layer_b_passive_actuated_system` | **Pass** | qfrc_passive matches at 1e-10 |
| AC30 | Passive tendon_model | `layer_b_passive_tendon_model` | **Pass** | qfrc_passive matches at 1e-10 |
| AC31 | Passive sensor_model | `layer_b_passive_sensor_model` | **Pass** | qfrc_passive matches at 1e-10 |
| AC32 | Passive equality_model | `layer_b_passive_equality_model` | **Pass** | All zeros as expected |
| AC33 | Passive composite_model | `layer_b_passive_composite_model` | **Pass** | qfrc_passive matches at 1e-10 |
| AC34 | Collision contact_scenario | `layer_b_collision_contact_scenario` | **#[ignore]** | contact.pos z off by 1e-3 — Phase 3 collision |
| AC35 | Collision composite_model | `layer_b_collision_composite_model` | **#[ignore]** | contact.pos z off by 1e-3 — Phase 3 collision |
| AC36 | Constraint contact_scenario | `layer_b_constraint_contact_scenario` | **#[ignore]** | efc_J row content diverges — Phase 3 constraint |
| AC37 | Constraint equality_model | `layer_b_constraint_equality_model` | **#[ignore]** | efc_J row content diverges — Phase 3 constraint |
| AC38 | Constraint composite_model | `layer_b_constraint_composite_model` | **#[ignore]** | efc_J row content diverges — Phase 3 constraint |
| AC39 | Actuation actuated_system | `layer_b_actuation_actuated_system` | **Pass** | qfrc_actuator + actuator_force match at 1e-10 |
| AC40 | Actuation composite_model | `layer_b_actuation_composite_model` | **Pass** | qfrc_actuator + actuator_force match at 1e-10 |
| AC41 | Sensor sensor_model | `layer_b_sensor_sensor_model` | **#[ignore]** | sensordata[12] off — framepos affected by xipos — Phase 6 sensor |
| AC42 | Sensor composite_model | `layer_b_sensor_composite_model` | **#[ignore]** | sensordata[4] off — framepos affected by xipos — Phase 6 sensor |
| AC43 | Tendon tendon_model | `layer_b_tendon_tendon_model` | **Pass** | ten_length + ten_velocity match at 1e-10 |
| AC44 | Tendon composite_model | `layer_b_tendon_composite_model` | **Pass** | ten_length + ten_velocity match at 1e-10 |
| AC45 | No production code changes (code review) | — (code review) | **Pass** | `git diff aa2ed08..f6fa988 --stat` shows only test files changed: `common.rs` (+148), `layer_b.rs` (+676 new), `mod.rs` (+1), `SESSION_PLAN.md` (+2/-1). Zero production code changes. |
| AC46 | All existing tests pass (runtime) | — (CI gate) | **Pass** | `cargo test -p sim-conformance-tests`: 43 passed, 0 failed, 28 ignored. Layer A (8 tests) and Layer D (13 tests) all pass. Layer B failures are `#[ignore]`d, not weakened. |
| AC47 | Test naming follows convention (code review) | — (code review) | **Pass** | All 43 Layer B test functions follow `layer_b_{stage}_{model}()` naming convention per umbrella Section 3. |

**Missing or failing ACs:** None missing. 28 ACs are `#[ignore]`d due to
upstream conformance gaps — all correctly tracked with source phase comments.

---

## 4. Test Plan Completeness

Verify every planned test from the spec was actually written. The AC table
above checks that ACs have *some* test — this section checks that the
spec's *full* test plan was executed.

> **Test numbering note:** Spec uses T1–T43. Implementation test functions
> follow the naming convention `layer_b_{stage}_{model}()`.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | FK pendulum — xpos/xquat/xipos vs reference at TOL_FK=1e-12 | Yes | `layer_b_fk_pendulum` | `#[ignore]` — xipos gap |
| T2 | FK double_pendulum — xpos/xquat/xipos vs reference | Yes | `layer_b_fk_double_pendulum` | `#[ignore]` — xipos gap |
| T3 | FK contact_scenario — xpos/xquat/xipos vs reference | Yes | `layer_b_fk_contact_scenario` | **Passes** |
| T4 | FK actuated_system — xpos/xquat/xipos vs reference, ctrl=[1.0,0.5] | Yes | `layer_b_fk_actuated_system` | `#[ignore]` — xipos gap |
| T5 | FK tendon_model — xpos/xquat/xipos vs reference | Yes | `layer_b_fk_tendon_model` | `#[ignore]` — xipos gap |
| T6 | FK sensor_model — xpos/xquat/xipos vs reference | Yes | `layer_b_fk_sensor_model` | `#[ignore]` — xipos gap |
| T7 | FK equality_model — xpos/xquat/xipos vs reference | Yes | `layer_b_fk_equality_model` | `#[ignore]` — xipos gap |
| T8 | FK composite_model — xpos/xquat/xipos vs reference, ctrl=[1.0] | Yes | `layer_b_fk_composite_model` | `#[ignore]` — xipos gap |
| T9 | CRBA pendulum — dense qM vs reference at TOL_CRBA=1e-12 | Yes | `layer_b_crba_pendulum` | `#[ignore]` — parallel axis |
| T10 | CRBA double_pendulum — dense qM vs reference | Yes | `layer_b_crba_double_pendulum` | `#[ignore]` — parallel axis |
| T11 | CRBA contact_scenario — dense qM (6×6) vs reference | Yes | `layer_b_crba_contact_scenario` | **Passes** |
| T12 | CRBA actuated_system — dense qM (1×1) vs reference | Yes | `layer_b_crba_actuated_system` | `#[ignore]` — parallel axis |
| T13 | CRBA tendon_model — dense qM (2×2) vs reference | Yes | `layer_b_crba_tendon_model` | `#[ignore]` — parallel axis |
| T14 | CRBA sensor_model — dense qM (2×2) vs reference | Yes | `layer_b_crba_sensor_model` | `#[ignore]` — parallel axis |
| T15 | CRBA equality_model — dense qM (3×3) vs reference | Yes | `layer_b_crba_equality_model` | `#[ignore]` — parallel axis |
| T16 | CRBA composite_model — dense qM (4×4) vs reference | Yes | `layer_b_crba_composite_model` | `#[ignore]` — parallel axis |
| T17 | RNE pendulum — qfrc_bias vs reference at TOL_RNE=1e-10 | Yes | `layer_b_rne_pendulum` | `#[ignore]` — gravity CoM |
| T18 | RNE double_pendulum — qfrc_bias vs reference | Yes | `layer_b_rne_double_pendulum` | `#[ignore]` — gravity CoM |
| T19 | RNE contact_scenario — qfrc_bias vs reference | Yes | `layer_b_rne_contact_scenario` | **Passes** |
| T20 | RNE actuated_system — qfrc_bias vs reference | Yes | `layer_b_rne_actuated_system` | `#[ignore]` — gravity CoM |
| T21 | RNE tendon_model — qfrc_bias vs reference | Yes | `layer_b_rne_tendon_model` | `#[ignore]` — gravity CoM |
| T22 | RNE sensor_model — qfrc_bias vs reference | Yes | `layer_b_rne_sensor_model` | `#[ignore]` — gravity CoM |
| T23 | RNE equality_model — qfrc_bias vs reference | Yes | `layer_b_rne_equality_model` | `#[ignore]` — gravity CoM |
| T24 | RNE composite_model — qfrc_bias vs reference | Yes | `layer_b_rne_composite_model` | `#[ignore]` — gravity CoM |
| T25 | Passive pendulum — qfrc_passive vs reference at TOL_PASSIVE=1e-10 | Yes | `layer_b_passive_pendulum` | **Passes** |
| T26 | Passive double_pendulum — qfrc_passive vs reference | Yes | `layer_b_passive_double_pendulum` | **Passes** |
| T27 | Passive contact_scenario — qfrc_passive vs reference (expect all zeros) | Yes | `layer_b_passive_contact_scenario` | **Passes** |
| T28 | Passive actuated_system — qfrc_passive vs reference | Yes | `layer_b_passive_actuated_system` | **Passes** |
| T29 | Passive tendon_model — qfrc_passive vs reference | Yes | `layer_b_passive_tendon_model` | **Passes** |
| T30 | Passive sensor_model — qfrc_passive vs reference | Yes | `layer_b_passive_sensor_model` | **Passes** |
| T31 | Passive equality_model — qfrc_passive vs reference (expect all zeros) | Yes | `layer_b_passive_equality_model` | **Passes** |
| T32 | Passive composite_model — qfrc_passive vs reference | Yes | `layer_b_passive_composite_model` | **Passes** |
| T33 | Collision contact_scenario — structural comparison, sign flip, TOL=1e-6 | Yes | `layer_b_collision_contact_scenario` | `#[ignore]` — pos z off |
| T34 | Collision composite_model — 2 contacts, structural comparison | Yes | `layer_b_collision_composite_model` | `#[ignore]` — pos z off |
| T35 | Constraint contact_scenario — nefc=4, efc_J/efc_b at 1e-8, efc_force at 1e-4 | Yes | `layer_b_constraint_contact_scenario` | `#[ignore]` — efc_J diverges |
| T36 | Constraint equality_model — nefc=7, efc_J/efc_b/efc_force | Yes | `layer_b_constraint_equality_model` | `#[ignore]` — efc_J diverges |
| T37 | Constraint composite_model — nefc=9, efc_J/efc_b/efc_force | Yes | `layer_b_constraint_composite_model` | `#[ignore]` — efc_J diverges |
| T38 | Actuation actuated_system — qfrc_actuator + actuator_force, ctrl=[1.0,0.5] | Yes | `layer_b_actuation_actuated_system` | **Passes** |
| T39 | Actuation composite_model — qfrc_actuator + actuator_force, ctrl=[1.0] | Yes | `layer_b_actuation_composite_model` | **Passes** |
| T40 | Sensor sensor_model — sensordata (nsensordata=21) at TOL_SENSOR=1e-8 | Yes | `layer_b_sensor_sensor_model` | `#[ignore]` — framepos off |
| T41 | Sensor composite_model — sensordata (nsensordata=7) at TOL_SENSOR=1e-8 | Yes | `layer_b_sensor_composite_model` | `#[ignore]` — framepos off |
| T42 | Tendon tendon_model — ten_length + ten_velocity at TOL_TENDON=1e-10 | Yes | `layer_b_tendon_tendon_model` | **Passes** |
| T43 | Tendon composite_model — ten_length + ten_velocity at TOL_TENDON=1e-10 | Yes | `layer_b_tendon_composite_model` | **Passes** |

**All 43 planned tests implemented.** 15 pass, 28 `#[ignore]`d.

### Supplementary Tests

Tests that were added beyond the spec's planned test list — additional
coverage discovered during implementation or review. These don't map 1:1
to ACs but strengthen the test suite.

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| (none) | — | — | Spec planned exactly 43 tests; 43 were implemented. No extras needed. |

### Edge Case Inventory

Cross-reference the spec's Edge Case Inventory. Were all edge cases tested?

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| World body (body 0) xpos/xquat | World body must have xpos=[0,0,0], xquat=[1,0,0,0] for all models. If wrong, entire FK chain is wrong. | Yes | T1–T8 (all FK tests compare body 0) | `run_fk_test()` iterates `0..nbody`, body 0 included. contact_scenario confirms world body correct. |
| Quaternion sign ambiguity | q and -q represent the same rotation. MuJoCo may choose different sign. | Yes | T1–T8 (`assert_quat_eq` handles sign) | L∞ min(pos,neg) correctly implemented in `common.rs:218-225`. |
| Contact depth sign flip | MuJoCo `dist` is negative for penetration; CortenForge `depth` is positive. Must negate. | Yes | T33–T34 | `-ref_depth[ri]` at `layer_b.rs:196`. Correct. |
| Models with ncon=0 | No collision test for models without contacts. Absence is intentional. | Yes | N/A | Only contact_scenario and composite_model tested — correct. |
| Models with nu=0 | No actuation test for unactuated models. Absence is correct. | Yes | N/A | Only actuated_system and composite_model tested — correct. |
| Models with nsensor=0 | No sensor test for sensorless models. Absence is correct. | Yes | N/A | Only sensor_model and composite_model tested — correct. |
| Constraint row count (nefc) | If nefc differs, element-wise comparison is invalid. Pre-check required. | Yes | T35–T37 (nefc pre-check) | `run_constraint_test()` checks nefc at `layer_b.rs:251` with diagnostic breakdown. |
| Pyramidal vs elliptic cone | Cone type affects constraint row count. Models must use MuJoCo default (pyramidal). | Yes | T35, T37 | nefc matches reference for all 3 models (no cone type mismatch). |
| Composite model multi-stage | composite_model exercises 9 of 9 stages. Upstream error cascades downstream. | Yes | T8, T16, T24, T32, T34, T37, T39, T41, T43 | All 9 composite_model tests exist. |
| Zero passive force models | contact_scenario, actuated_system, equality_model have no springs → all zeros. | Yes | T27, T28, T31 | All pass — zero passive force correctly verified. |
| Ctrl values for actuated models | actuated_system ctrl=[1.0,0.5], composite_model ctrl=[1.0]. Forgetting ctrl → zero actuation. | Yes | T38, T39 | `&[1.0, 0.5]` and `&[1.0]` correctly passed to helpers. |
| Position servo with activation | actuated_system has dyntype=integrator → na=1. At initial state act=0. | Yes | T38 | Passes — actuator_force=[1.0, 0.0] matches MuJoCo (motor=1.0, servo with act=0 → force=0). |

**Missing tests:** None.

---

## 5. Blast Radius Verification

The spec predicted behavioral changes, files affected, and existing test
impact. Compare predictions against reality. Surprises here reveal spec
gaps worth learning from.

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| None (Spec A adds tests only — no production code changes) | Yes — confirmed | No production code modified. Only test files changed. |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/tests/mujoco_conformance/layer_b.rs` (new, +1,200–1,500 lines) | Yes, +676 lines (new) | — |
| `sim/L0/tests/mujoco_conformance/mod.rs` (add `mod layer_b;`, +1 line) | Yes, +1 line | — |
| `sim/L0/tests/mujoco_conformance/common.rs` (extend with helpers, +120–150 lines) | Yes, +148 lines | — |

The spec predicted +1,200–1,500 lines for `layer_b.rs`; actual was +676.
The helper extraction reduced code by ~50% vs the spec's inline-per-test
pattern. This is a positive deviation — same coverage, less code.

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| `SESSION_PLAN.md` | Session 7 updated its status from Pending to Done and filled in commit hash. Expected administrative change. |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| All Layer A tests (`layer_a.rs`) | Pass (unchanged) — separate module | Pass (8/8) | No |
| All Layer D tests (`layer_d.rs`) | Pass (unchanged) — separate module | Pass (13/13) | No |
| All integration tests (`integration/*.rs`) | Pass (unchanged) — separate test binary | Pass (1,247/1,247) | No |
| `common.rs` existing code | Pass (unchanged) — all changes additive | Pass — existing `parse_npy()`, tolerance constants unchanged | No |

**Unexpected regressions:** None.

---

## 6. Convention Notes Audit

Check every row in the spec's Convention Notes table. Convention mismatches
are the #1 source of silent conformance bugs.

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| `xpos` | Access: `ref_data[i*3+k]` ↔ `data.xpos[i][k]` | Yes | `run_fk_test()` line 30: `data.xpos.iter().flat_map(\|v\| [v[0], v[1], v[2]])` produces flat array matching ref layout. |
| `xquat` | Access: `ref[i*4+0]` ↔ `q.w`, `ref[i*4+1]` ↔ `q.i`, etc. Sign ambiguity: `min(‖q_cf - q_ref‖, ‖q_cf + q_ref‖) < tol` | Yes | `run_fk_test()` lines 42-49: indices `[i*4, i*4+1, i*4+2, i*4+3]` → `[w, x, y, z]`. `assert_quat_eq()` uses `q_cf.w, q_cf.i, q_cf.j, q_cf.k`. L∞ sign ambiguity at `common.rs:219-225`. |
| `xipos` | Same as xpos | Yes | `run_fk_test()` line 54: same `flat_map` pattern as xpos. |
| `qM` | Access: `ref[i*nv+j]` ↔ `data.qM[(i,j)]` | Yes | `run_crba_test()` lines 82-84: nested `for i..nv { for j..nv { data.qM[(i,j)] } }` produces row-major flat array matching ref. |
| `qfrc_bias` | Access: `ref[i]` ↔ `data.qfrc_bias[i]` | Yes | `run_rne_test()` line 109: `(0..model.nv).map(\|i\| data.qfrc_bias[i])`. |
| `qfrc_passive` | Same as qfrc_bias | Yes | `run_passive_test()` line 132: same pattern. |
| `qfrc_actuator` | Same as qfrc_bias | Yes | `run_actuation_test()` line 319: same pattern. |
| `actuator_force` | Access: `ref[i]` ↔ `data.actuator_force[i]` | Yes | `run_actuation_test()` line 336: `&data.actuator_force` directly (Vec<f64> slice). |
| `contact.dist` → `contact.depth` | **Negate:** `depth_cf ≈ -ref_dist` | Yes | `run_collision_test()` line 196: `let ref_d = -ref_depth[ri]`. Correct sign flip. |
| `contact.frame` → `contact.normal` | Access: `ref_normal[k]` ↔ `contact.normal[k]` | Yes | `run_collision_test()` lines 208-217: `ref_normal[ri * 3 + k]` ↔ `data.contacts[ci].normal[k]`. |
| `contact.geom` | Cast: `ref_pair[0] as usize` ↔ `contact.geom1` | Yes | `run_collision_test()` line 190: `ref_g1 as usize` comparison with `cf_g1: usize`. |
| `sensordata` | Access: `ref[i]` ↔ `data.sensordata[i]` | Yes | `run_sensor_test()` line 354: `(0..nsensordata).map(\|i\| data.sensordata[i])`. |
| `ten_length` | Access: `ref[i]` ↔ `data.ten_length[i]` | Yes | `run_tendon_test()` line 383: `&data.ten_length` directly. |
| `ten_velocity` | Same as ten_length | Yes | `run_tendon_test()` line 394: `&data.ten_velocity` directly. |
| `efc_J` | Access: `ref[i*nv+j]` ↔ `data.efc_J[(i,j)]` | Yes | `run_constraint_test()` lines 267-270: nested `for i..nefc { for j..nv { data.efc_J[(i,j)] } }` produces row-major flat array. |
| `efc_b` | Access: `ref[i]` ↔ `data.efc_b[i]` | Yes | `run_constraint_test()` line 283: `(0..nefc).map(\|i\| data.efc_b[i])`. |
| `efc_force` | Access: `ref[i]` ↔ `data.efc_force[i]` | Yes | `run_constraint_test()` line 295: `(0..nefc).map(\|i\| data.efc_force[i])`. |

All 17 convention rules are correctly followed. No convention mismatches found.

---

## 7. Weak Implementation Inventory

Items that technically work but aren't solid. These should be fixed now —
"weak" items left unfixed tend to become permanent technical debt.

**What counts as weak:**

- `TODO` / `FIXME` / `HACK` comments in new code
- Hardcoded values that should come from the spec or MuJoCo
- Loose tolerances (e.g., 1e-3 where MuJoCo conformance demands 1e-10)
- Missing edge-case guards the spec calls for
- Placeholder error handling (e.g., `unwrap()` in non-test code, empty
  `Err(_) => {}` that swallows information)
- Functionality that "passes tests" but uses a different algorithm than
  the spec prescribes
- Dead code or commented-out code from debugging

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|
| (none) | — | — | — | — |

No weak items found. The implementation is clean:
- No TODO/FIXME/HACK comments.
- All tolerances match spec exactly (verified against `common.rs:73-95`).
- No loose tolerances — tests fail or are `#[ignore]`d, never weakened.
- All `#[ignore]` annotations have descriptive tracking comments identifying the conformance gap and source phase.
- No dead code or debugging artifacts.
- No `unwrap()` outside test code (and test panics are appropriate).
- Helper extraction is clean with no leftover dead helpers.

---

## 8. Deferred Work Tracker

Every item that was in the spec's scope but not fully implemented, plus
anything discovered during implementation or review that's out of scope.
**The goal: nothing deferred is untracked.**

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| Integration stage reference tests | Out of Scope, bullet 1 | Spec B (Layer C — trajectory comparison) | Phase 12 Session 12 | Yes — Session 12 prompt explicitly covers trajectory comparison |
| Additional FK fields (xmat, geom_xpos, geom_xmat, site_xpos) | Out of Scope, bullet 2 | `future_work_10j.md`, `ROADMAP_V1.md` | DT-160 | Yes — added during review |
| Fixing conformance failures | Out of Scope, bullet 3 | Spec A specifies `#[ignore]` with tracking comments | Session 15 (Gate Triage) | Yes — all 28 `#[ignore]`d tests have source phase comments. Session 15 will triage. |
| Flag-dependent reference data | Out of Scope, bullet 4 | DT-97 (Session 2) | DT-97 | Yes — Session 2 covered flag golden-file tests |

### Discovered During Implementation

Items that came up during implementation that weren't anticipated by the
spec. These are the most likely to be untracked.

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| xipos not computed (body CoM in world frame) | FK tests fail for 7/8 models at `fk.xipos[3]` — all zero | `#[ignore]` tracking comments identify "Phase 1 FK" | Session 15 (Gate Triage) | Yes |
| Parallel axis theorem missing from CRBA | CRBA qM values too small — missing body inertia contribution that depends on xipos | `#[ignore]` tracking comments identify "Phase 1 CRBA" | Session 15 | Yes |
| contact.pos convention difference | Collision pos z off by 1e-3 for both models | `#[ignore]` tracking comments identify "Phase 3 collision" | Session 15 | Yes |
| Constraint Jacobian assembly divergence | efc_J[0] values swapped/wrong for all 3 constraint models | `#[ignore]` tracking comments identify "Phase 3 constraint" | Session 15 | Yes |
| Sensor framepos affected by xipos/subtree_com | sensordata off at specific indices for framepos-type sensors | `#[ignore]` tracking comments identify "Phase 6 sensor" | Session 15 | Yes |

### Discovered During Review

Items found during this review that were not surfaced during
implementation. Reviews are a discovery mechanism.

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| Additional FK fields not tracked | Out of Scope bullet 2 had no DT or roadmap entry | `future_work_10j.md`, `ROADMAP_V1.md` | DT-160 | **Fixed during review** |

### Spec Gaps Found During Implementation

Items where the spec was wrong or incomplete and was (or should have been)
updated. Verify the spec was actually updated.

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|
| Spec showed inline test code per model; implementation used shared helpers | Implementation extracted `run_{stage}_test()` helpers instead of 43 inline test functions. Reduces code by ~50% with identical coverage. | No (spec not updated) | Positive deviation — does not need spec update. The spec served as a design document; the implementation improved on it. |
| Spec predicted 1,200–1,500 lines for layer_b.rs | Actual: 676 lines (helper extraction) | No | Line count prediction was based on inline pattern. |

---

## 9. Test Coverage Summary

Quick sanity check on test health after the implementation.

**Domain test results:**
```
sim-core:              603 passed, 0 failed, 0 ignored
sim-mjcf:              333 passed, 0 failed, 16 ignored
sim-conformance-tests: 43 passed, 0 failed, 28 ignored
Total:                 979 passed, 0 failed, 44 ignored
```

**New tests added:** 43 (all Layer B — 8 FK + 8 CRBA + 8 RNE + 8 Passive + 2 Collision + 3 Constraint + 2 Actuation + 2 Sensor + 2 Tendon)
**Tests modified:** 0
**Pre-existing test regressions:** 0

**Clippy:** Clean (0 warnings)
**Fmt:** Clean (no formatting violations)

---

## 10. Review Verdict

| Category | Section | Status |
|----------|---------|--------|
| Key behaviors gap closure | 1 | **Partial** — 4/11 behaviors fully closed (passive, depth convention, actuation, tendon). 7 remaining have tests but conformance gaps in upstream stages. |
| Spec section compliance | 2 | **Pass** — All 10 sections pass or positively deviated. No weak or missing sections. |
| Acceptance criteria | 3 | **Pass** — 47/47 ACs accounted for. 15 pass, 3 pass (code review), 1 pass (CI gate), 28 `#[ignore]`d with tracking. |
| Test plan completeness | 4 | **Pass** — 43/43 planned tests implemented. All edge cases covered. |
| Blast radius accuracy | 5 | **Pass** — All predictions correct. No unexpected regressions. Only surprise: layer_b.rs was 676 lines vs predicted 1,200–1,500 (positive deviation from helper extraction). |
| Convention fidelity | 6 | **Pass** — All 17 convention rules correctly followed. |
| Weak items | 7 | **Pass** — No weak items found. |
| Deferred work tracking | 8 | **Pass** — All deferred items tracked. DT-160 created during review for additional FK fields. |
| Test health | 9 | **Pass** — 979 passed, 0 failed, 44 ignored. Clippy clean, fmt clean. |

**Overall: Pass.** The implementation is faithful to the spec, correctly
identifies conformance gaps via `#[ignore]` with tracking comments, and
introduces no regressions. All 43 Layer B tests are written with correct
assertions, tolerances, and diagnostic output. The 28 `#[ignore]`d tests
represent real upstream conformance gaps (not test infrastructure problems)
and are properly tracked for Session 15 Gate Triage.

**Items fixed during review:** Created DT-160 for additional FK fields (xmat, geom_xpos, geom_xmat, site_xpos) — added to `future_work_10j.md` and `ROADMAP_V1.md`.

**Items to fix before shipping:** None. The implementation is clean.

**Items tracked for future work:**
- xipos computation (Phase 1 FK) — unblocks 23 of 28 ignored tests
- Contact position convention (Phase 3 collision) — unblocks 2 tests
- Constraint Jacobian assembly (Phase 3 constraint) — unblocks 3 tests
- Additional FK fields reference data (xmat, geom_xpos, etc.) — DT-160

---

## Preliminary #[ignore] Inventory

This inventory feeds into Session 15 (Gate Triage).

| # | Test | Failure Description | Root Cause | Source Phase | Upstream Fix |
|---|------|-------------------|------------|--------------|-------------|
| 1 | `layer_b_fk_pendulum` | `fk.xipos[3]: expected 1.5e-1, got 0.0` | xipos (body CoM in world frame) not computed — all zeros | Phase 1 FK | Implement `mj_kinematics()` xipos computation |
| 2 | `layer_b_fk_double_pendulum` | `fk.xipos[3]: expected 1.5e-1, got 0.0` | Same as #1 | Phase 1 FK | Same as #1 |
| 3 | `layer_b_fk_actuated_system` | `fk.xipos[3]: expected 1.5e-1, got 0.0` | Same as #1 | Phase 1 FK | Same as #1 |
| 4 | `layer_b_fk_tendon_model` | `fk.xipos[3]: expected 1.5e-1, got 0.0` | Same as #1 | Phase 1 FK | Same as #1 |
| 5 | `layer_b_fk_sensor_model` | `fk.xipos[3]: expected 1.5e-1, got 0.0` | Same as #1 | Phase 1 FK | Same as #1 |
| 6 | `layer_b_fk_equality_model` | `fk.xipos[3]: expected 1.25e-1, got 0.0` | Same as #1 | Phase 1 FK | Same as #1 |
| 7 | `layer_b_fk_composite_model` | `fk.xipos[3]: expected 1.5e-1, got 0.0` | Same as #1 | Phase 1 FK | Same as #1 |
| 8 | `layer_b_crba_pendulum` | `crba.qM[0]: expected 3.15e-2, got 4.41e-3` | Parallel axis theorem contribution missing — depends on xipos for body CoM offset | Phase 1 CRBA | Fix xipos first (cascading dependency) |
| 9 | `layer_b_crba_double_pendulum` | `crba.qM[0]: expected 1.25e-1, got 5.15e-2` | Same as #8 | Phase 1 CRBA | Same as #8 |
| 10 | `layer_b_crba_actuated_system` | `crba.qM[0]: expected 3.15e-2, got 4.41e-3` | Same as #8 | Phase 1 CRBA | Same as #8 |
| 11 | `layer_b_crba_tendon_model` | `crba.qM[0]: expected 1.25e-1, got 5.15e-2` | Same as #8 | Phase 1 CRBA | Same as #8 |
| 12 | `layer_b_crba_sensor_model` | `crba.qM[0]: expected 1.25e-1, got 5.15e-2` | Same as #8 | Phase 1 CRBA | Same as #8 |
| 13 | `layer_b_crba_equality_model` | `crba.qM[0]: expected 1.69e-1, got 9.96e-2` | Same as #8 | Phase 1 CRBA | Same as #8 |
| 14 | `layer_b_crba_composite_model` | `crba.qM[0]: expected 1.25e-1, got 5.15e-2` | Same as #8 | Phase 1 CRBA | Same as #8 |
| 15 | `layer_b_rne_pendulum` | `rne.qfrc_bias[0]: expected -1.47e0, got 0.0` | Gravity torques use wrong CoM (xipos not computed → RNE gravity term incorrect) | Phase 1 RNE | Fix xipos first |
| 16 | `layer_b_rne_double_pendulum` | `rne.qfrc_bias[0]: expected -3.56e0, got -1.47e0` | Same as #15 | Phase 1 RNE | Same as #15 |
| 17 | `layer_b_rne_actuated_system` | `rne.qfrc_bias[0]: expected -1.47e0, got 0.0` | Same as #15 | Phase 1 RNE | Same as #15 |
| 18 | `layer_b_rne_tendon_model` | `rne.qfrc_bias[0]: expected -3.56e0, got -1.47e0` | Same as #15 | Phase 1 RNE | Same as #15 |
| 19 | `layer_b_rne_sensor_model` | `rne.qfrc_bias[0]: expected -3.56e0, got -1.47e0` | Same as #15 | Phase 1 RNE | Same as #15 |
| 20 | `layer_b_rne_equality_model` | `rne.qfrc_bias[0]: expected -4.49e0, got -2.55e0` | Same as #15 | Phase 1 RNE | Same as #15 |
| 21 | `layer_b_rne_composite_model` | `rne.qfrc_bias[0]: expected -3.56e0, got -1.47e0` | Same as #15 | Phase 1 RNE | Same as #15 |
| 22 | `layer_b_sensor_sensor_model` | `sensor.sensordata[12]: expected 2.42e-1, got 1.0e-1` | framepos sensor reads xipos/subtree_com which is wrong | Phase 6 sensor / Phase 1 FK | Fix xipos first |
| 23 | `layer_b_sensor_composite_model` | `sensor.sensordata[4]: expected 2.42e-1, got 1.0e-1` | Same as #22 | Phase 6 sensor / Phase 1 FK | Same as #22 |
| 24 | `layer_b_collision_contact_scenario` | `collision.pos[geom pair (0,2)][2]: expected -1.0e-3, got 0.0` | Contact position z coordinate off by 1e-3 — convention difference in where contact point is placed relative to surface | Phase 3 collision | Audit contact point computation against `mjc_*.c` narrowphase functions |
| 25 | `layer_b_collision_composite_model` | `collision.pos[geom pair (0,3)][2]: expected -1.0e-3, got 0.0` | Same as #24 | Phase 3 collision | Same as #24 |
| 26 | `layer_b_constraint_contact_scenario` | `constraint.efc_J[0]: expected 0.0, got 1.0` | Constraint Jacobian row content fundamentally differs — rows appear reordered or computed differently | Phase 3 constraint | Audit constraint assembly row ordering and per-row Jacobian computation against `engine_core_constraint.c` |
| 27 | `layer_b_constraint_equality_model` | `constraint.efc_J[0]: expected 1.0, got 0.0` | Same as #26 | Phase 3 constraint | Same as #26 |
| 28 | `layer_b_constraint_composite_model` | `constraint.efc_J[2]: expected 1.0, got -3.0e-1` | Same as #26 | Phase 3 constraint | Same as #26 |

**Summary by root cause:**

| Root Cause | Tests Affected | Priority | Cascading? |
|-----------|---------------|----------|-----------|
| xipos not computed | 23 (FK: 7, CRBA: 7, RNE: 7, sensor: 2) | **Critical** | Yes — CRBA, RNE, and sensor all cascade from xipos |
| Contact position convention | 2 | Medium | No |
| Constraint Jacobian assembly | 3 | Medium | No (efc_force comparison blocked but independent) |
