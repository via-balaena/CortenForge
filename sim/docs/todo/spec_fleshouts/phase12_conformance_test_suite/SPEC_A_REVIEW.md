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
| FK body poses from joint angles | `mj_kinematics()` computes xpos/xquat/xipos for all bodies via kinematic tree traversal | `data.forward(model)` → `mj_fwd_position()` → same tree traversal. Fields: `data.xpos: Vec<Vector3<f64>>`, `data.xquat: Vec<UnitQuaternion<f64>>`, `data.xipos: Vec<Vector3<f64>>` | | |
| Dense mass matrix | `mj_crb()` computes sparse `qM`, then `mj_fullM()` unpacks to dense nv×nv | `data.qM: DMatrix<f64>` stored dense. Direct comparison via `[(i,j)]` indexing | | |
| Coriolis + gravity bias | `mj_rne()` backward/forward Newton-Euler on kinematic tree | `data.qfrc_bias: DVector<f64>`. Same algorithm | | |
| Passive spring/damper forces | `mj_passive()` computes spring + damper forces per joint | `data.qfrc_passive: DVector<f64>`. Same algorithm | | |
| Contact detection | `mj_collision()` iterates broadphase geom pairs, runs narrowphase | `data.contacts: Vec<Contact>`. Geom pair order may differ — structural comparison needed | | |
| Contact depth convention | `d->contact[i].dist`: negative = penetration | `data.contacts[i].depth`: positive = penetration. **Sign flip required** | | |
| Contact normal | `d->contact[i].frame[0:3]` is contact normal | `data.contacts[i].normal: Vector3<f64>`. Direct match | | |
| Constraint Jacobian | `mj_makeConstraint()` assembles efc_J rows. Order: equality → friction loss → joint limits → tendon limits → contacts | `data.efc_J: DMatrix<f64>`. Same row ordering (verified in `assembly.rs:47-53`). Element-wise comparison valid when nefc matches | | |
| Constraint forces | `mj_projectConstraint()` iterative solver | `data.efc_force: DVector<f64>`. Looser tolerance (1e-4) for solver convergence | | |
| Actuator forces | `mj_fwdActuation()` applies gain/bias → transmission to joint space | `data.qfrc_actuator: DVector<f64>`, `data.actuator_force: Vec<f64>`. Direct computation | | |
| Sensor data | `mj_sensorPos/Vel/Acc()` writes to contiguous sensordata array | `data.sensordata: DVector<f64>`. Sensor order follows model definition | | |
| Tendon kinematics | `mj_tendon()` computes lengths and velocities | `data.ten_length: Vec<f64>`, `data.ten_velocity: Vec<f64>`. Direct computation | | |

**Unclosed gaps:**

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

**Grade:**

**Spec says:**
Extend `common.rs` with Layer B-specific helpers: `parse_npy_i32()`,
`reference_path()`, `load_conformance_model()`, `load_reference_f64()`,
`load_reference_i32()`, `assert_array_eq()`, `assert_quat_eq()`, and
`TOL_CONSTRAINT_JAC` constant.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. FK Reference Tests

**Grade:**

**Spec says:**
Test all 8 models for FK. Each test loads model, calls `forward()`, compares
`xpos`, `xquat`, `xipos` against reference data. Quaternion comparison uses
`assert_quat_eq()` with sign ambiguity. World body (body 0) always tested.
Models: pendulum, double_pendulum, contact_scenario, actuated_system,
tendon_model, sensor_model, equality_model, composite_model.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. CRBA Reference Tests

**Grade:**

**Spec says:**
Compare dense mass matrix element-wise for all 8 models. Reference data uses
`mj_fullM()` to unpack sparse qM. Tolerance: `TOL_CRBA = 1e-12`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. RNE Reference Tests

**Grade:**

**Spec says:**
Compare qfrc_bias element-wise for all 8 models. Tolerance: `TOL_RNE = 1e-10`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. Passive Force Reference Tests

**Grade:**

**Spec says:**
Compare qfrc_passive element-wise for all 8 models. Tolerance:
`TOL_PASSIVE = 1e-10`. Non-zero passive forces for models with springs
(pendulum, double_pendulum, tendon_model, sensor_model, composite_model).
Zero passive forces for models without springs (contact_scenario,
actuated_system, equality_model).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S6. Collision Reference Tests

**Grade:**

**Spec says:**
Structural comparison — contacts matched by geom pair, not array index.
Sign flip for depth (`depth_cf ≈ -ref_dist`). Tolerance:
`TOL_COLLISION_DEPTH = 1e-6`. Models: contact_scenario (1 contact),
composite_model (2 contacts).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S7. Constraint Reference Tests

**Grade:**

**Spec says:**
Element-wise comparison of efc_J, efc_b, efc_force with nefc pre-check.
Tolerances: efc_J/efc_b at `TOL_CONSTRAINT_JAC = 1e-8`, efc_force at
`TOL_CONSTRAINT = 1e-4`. Models: contact_scenario (4 efc rows),
equality_model (7 efc rows), composite_model (9 efc rows).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S8. Actuation Reference Tests

**Grade:**

**Spec says:**
Compare qfrc_actuator (nv) and actuator_force (nu) element-wise. Tolerance:
`TOL_ACTUATION = 1e-10`. Ctrl values must be set before `forward()`:
actuated_system `ctrl=[1.0, 0.5]`, composite_model `ctrl=[1.0]`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S9. Sensor Reference Tests

**Grade:**

**Spec says:**
Compare sensordata array element-wise. Tolerance: `TOL_SENSOR = 1e-8`.
Models: sensor_model (nsensordata=21, 8 sensors), composite_model
(nsensordata=7, 3 sensors).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S10. Tendon Reference Tests

**Grade:**

**Spec says:**
Compare ten_length and ten_velocity element-wise. Tolerance:
`TOL_TENDON = 1e-10`. Models: tendon_model (ntendon=1),
composite_model (ntendon=1).

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

Walk through every AC from the spec. For runtime ACs, confirm the test
exists and passes. For code-review ACs, perform the review now.

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | Layer B infrastructure helpers (code review) | — (code review) | | |
| AC2 | FK pendulum | `layer_b_fk_pendulum` | | |
| AC3 | FK double_pendulum | `layer_b_fk_double_pendulum` | | |
| AC4 | FK contact_scenario | `layer_b_fk_contact_scenario` | | |
| AC5 | FK actuated_system | `layer_b_fk_actuated_system` | | |
| AC6 | FK tendon_model | `layer_b_fk_tendon_model` | | |
| AC7 | FK sensor_model | `layer_b_fk_sensor_model` | | |
| AC8 | FK equality_model | `layer_b_fk_equality_model` | | |
| AC9 | FK composite_model | `layer_b_fk_composite_model` | | |
| AC10 | CRBA pendulum | `layer_b_crba_pendulum` | | |
| AC11 | CRBA double_pendulum | `layer_b_crba_double_pendulum` | | |
| AC12 | CRBA contact_scenario | `layer_b_crba_contact_scenario` | | |
| AC13 | CRBA actuated_system | `layer_b_crba_actuated_system` | | |
| AC14 | CRBA tendon_model | `layer_b_crba_tendon_model` | | |
| AC15 | CRBA sensor_model | `layer_b_crba_sensor_model` | | |
| AC16 | CRBA equality_model | `layer_b_crba_equality_model` | | |
| AC17 | CRBA composite_model | `layer_b_crba_composite_model` | | |
| AC18 | RNE pendulum | `layer_b_rne_pendulum` | | |
| AC19 | RNE double_pendulum | `layer_b_rne_double_pendulum` | | |
| AC20 | RNE contact_scenario | `layer_b_rne_contact_scenario` | | |
| AC21 | RNE actuated_system | `layer_b_rne_actuated_system` | | |
| AC22 | RNE tendon_model | `layer_b_rne_tendon_model` | | |
| AC23 | RNE sensor_model | `layer_b_rne_sensor_model` | | |
| AC24 | RNE equality_model | `layer_b_rne_equality_model` | | |
| AC25 | RNE composite_model | `layer_b_rne_composite_model` | | |
| AC26 | Passive pendulum | `layer_b_passive_pendulum` | | |
| AC27 | Passive double_pendulum | `layer_b_passive_double_pendulum` | | |
| AC28 | Passive contact_scenario | `layer_b_passive_contact_scenario` | | |
| AC29 | Passive actuated_system | `layer_b_passive_actuated_system` | | |
| AC30 | Passive tendon_model | `layer_b_passive_tendon_model` | | |
| AC31 | Passive sensor_model | `layer_b_passive_sensor_model` | | |
| AC32 | Passive equality_model | `layer_b_passive_equality_model` | | |
| AC33 | Passive composite_model | `layer_b_passive_composite_model` | | |
| AC34 | Collision contact_scenario | `layer_b_collision_contact_scenario` | | |
| AC35 | Collision composite_model | `layer_b_collision_composite_model` | | |
| AC36 | Constraint contact_scenario | `layer_b_constraint_contact_scenario` | | |
| AC37 | Constraint equality_model | `layer_b_constraint_equality_model` | | |
| AC38 | Constraint composite_model | `layer_b_constraint_composite_model` | | |
| AC39 | Actuation actuated_system | `layer_b_actuation_actuated_system` | | |
| AC40 | Actuation composite_model | `layer_b_actuation_composite_model` | | |
| AC41 | Sensor sensor_model | `layer_b_sensor_sensor_model` | | |
| AC42 | Sensor composite_model | `layer_b_sensor_composite_model` | | |
| AC43 | Tendon tendon_model | `layer_b_tendon_tendon_model` | | |
| AC44 | Tendon composite_model | `layer_b_tendon_composite_model` | | |
| AC45 | No production code changes (code review) | — (code review) | | |
| AC46 | All existing tests pass (runtime) | — (CI gate) | | |
| AC47 | Test naming follows convention (code review) | — (code review) | | |

**Missing or failing ACs:**

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
| T1 | FK pendulum — xpos/xquat/xipos vs reference at TOL_FK=1e-12 | | `layer_b_fk_pendulum` | |
| T2 | FK double_pendulum — xpos/xquat/xipos vs reference | | `layer_b_fk_double_pendulum` | |
| T3 | FK contact_scenario — xpos/xquat/xipos vs reference | | `layer_b_fk_contact_scenario` | |
| T4 | FK actuated_system — xpos/xquat/xipos vs reference, ctrl=[1.0,0.5] | | `layer_b_fk_actuated_system` | |
| T5 | FK tendon_model — xpos/xquat/xipos vs reference | | `layer_b_fk_tendon_model` | |
| T6 | FK sensor_model — xpos/xquat/xipos vs reference | | `layer_b_fk_sensor_model` | |
| T7 | FK equality_model — xpos/xquat/xipos vs reference | | `layer_b_fk_equality_model` | |
| T8 | FK composite_model — xpos/xquat/xipos vs reference, ctrl=[1.0] | | `layer_b_fk_composite_model` | |
| T9 | CRBA pendulum — dense qM vs reference at TOL_CRBA=1e-12 | | `layer_b_crba_pendulum` | |
| T10 | CRBA double_pendulum — dense qM vs reference | | `layer_b_crba_double_pendulum` | |
| T11 | CRBA contact_scenario — dense qM (6×6) vs reference | | `layer_b_crba_contact_scenario` | |
| T12 | CRBA actuated_system — dense qM (1×1) vs reference | | `layer_b_crba_actuated_system` | |
| T13 | CRBA tendon_model — dense qM (2×2) vs reference | | `layer_b_crba_tendon_model` | |
| T14 | CRBA sensor_model — dense qM (2×2) vs reference | | `layer_b_crba_sensor_model` | |
| T15 | CRBA equality_model — dense qM (3×3) vs reference | | `layer_b_crba_equality_model` | |
| T16 | CRBA composite_model — dense qM (4×4) vs reference | | `layer_b_crba_composite_model` | |
| T17 | RNE pendulum — qfrc_bias vs reference at TOL_RNE=1e-10 | | `layer_b_rne_pendulum` | |
| T18 | RNE double_pendulum — qfrc_bias vs reference | | `layer_b_rne_double_pendulum` | |
| T19 | RNE contact_scenario — qfrc_bias vs reference | | `layer_b_rne_contact_scenario` | |
| T20 | RNE actuated_system — qfrc_bias vs reference | | `layer_b_rne_actuated_system` | |
| T21 | RNE tendon_model — qfrc_bias vs reference | | `layer_b_rne_tendon_model` | |
| T22 | RNE sensor_model — qfrc_bias vs reference | | `layer_b_rne_sensor_model` | |
| T23 | RNE equality_model — qfrc_bias vs reference | | `layer_b_rne_equality_model` | |
| T24 | RNE composite_model — qfrc_bias vs reference | | `layer_b_rne_composite_model` | |
| T25 | Passive pendulum — qfrc_passive vs reference at TOL_PASSIVE=1e-10 | | `layer_b_passive_pendulum` | |
| T26 | Passive double_pendulum — qfrc_passive vs reference | | `layer_b_passive_double_pendulum` | |
| T27 | Passive contact_scenario — qfrc_passive vs reference (expect all zeros) | | `layer_b_passive_contact_scenario` | |
| T28 | Passive actuated_system — qfrc_passive vs reference | | `layer_b_passive_actuated_system` | |
| T29 | Passive tendon_model — qfrc_passive vs reference | | `layer_b_passive_tendon_model` | |
| T30 | Passive sensor_model — qfrc_passive vs reference | | `layer_b_passive_sensor_model` | |
| T31 | Passive equality_model — qfrc_passive vs reference (expect all zeros) | | `layer_b_passive_equality_model` | |
| T32 | Passive composite_model — qfrc_passive vs reference | | `layer_b_passive_composite_model` | |
| T33 | Collision contact_scenario — structural comparison, sign flip, TOL=1e-6 | | `layer_b_collision_contact_scenario` | |
| T34 | Collision composite_model — 2 contacts, structural comparison | | `layer_b_collision_composite_model` | |
| T35 | Constraint contact_scenario — nefc=4, efc_J/efc_b at 1e-8, efc_force at 1e-4 | | `layer_b_constraint_contact_scenario` | |
| T36 | Constraint equality_model — nefc=7, efc_J/efc_b/efc_force | | `layer_b_constraint_equality_model` | |
| T37 | Constraint composite_model — nefc=9, efc_J/efc_b/efc_force | | `layer_b_constraint_composite_model` | |
| T38 | Actuation actuated_system — qfrc_actuator + actuator_force, ctrl=[1.0,0.5] | | `layer_b_actuation_actuated_system` | |
| T39 | Actuation composite_model — qfrc_actuator + actuator_force, ctrl=[1.0] | | `layer_b_actuation_composite_model` | |
| T40 | Sensor sensor_model — sensordata (nsensordata=21) at TOL_SENSOR=1e-8 | | `layer_b_sensor_sensor_model` | |
| T41 | Sensor composite_model — sensordata (nsensordata=7) at TOL_SENSOR=1e-8 | | `layer_b_sensor_composite_model` | |
| T42 | Tendon tendon_model — ten_length + ten_velocity at TOL_TENDON=1e-10 | | `layer_b_tendon_tendon_model` | |
| T43 | Tendon composite_model — ten_length + ten_velocity at TOL_TENDON=1e-10 | | `layer_b_tendon_composite_model` | |

### Supplementary Tests

Tests that were added beyond the spec's planned test list — additional
coverage discovered during implementation or review. These don't map 1:1
to ACs but strengthen the test suite.

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|

### Edge Case Inventory

Cross-reference the spec's Edge Case Inventory. Were all edge cases tested?

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| World body (body 0) xpos/xquat | World body must have xpos=[0,0,0], xquat=[1,0,0,0] for all models. If wrong, entire FK chain is wrong. | | T1–T8 (all FK tests compare body 0) | |
| Quaternion sign ambiguity | q and -q represent the same rotation. MuJoCo may choose different sign. | | T1–T8 (`assert_quat_eq` handles sign) | |
| Contact depth sign flip | MuJoCo `dist` is negative for penetration; CortenForge `depth` is positive. Must negate. | | T33–T34 | |
| Models with ncon=0 | No collision test for models without contacts. Absence is intentional. | | N/A | |
| Models with nu=0 | No actuation test for unactuated models. Absence is correct. | | N/A | |
| Models with nsensor=0 | No sensor test for sensorless models. Absence is correct. | | N/A | |
| Constraint row count (nefc) | If nefc differs, element-wise comparison is invalid. Pre-check required. | | T35–T37 (nefc pre-check) | |
| Pyramidal vs elliptic cone | Cone type affects constraint row count. Models must use MuJoCo default (pyramidal). | | T35, T37 | |
| Composite model multi-stage | composite_model exercises 9 of 9 stages. Upstream error cascades downstream. | | T8, T16, T24, T32, T34, T37, T39, T41, T43 | |
| Zero passive force models | contact_scenario, actuated_system, equality_model have no springs → all zeros. | | T27, T28, T31 | |
| Ctrl values for actuated models | actuated_system ctrl=[1.0,0.5], composite_model ctrl=[1.0]. Forgetting ctrl → zero actuation. | | T38, T39 | |
| Position servo with activation | actuated_system has dyntype=integrator → na=1. At initial state act=0. | | T38 | |

**Missing tests:**

---

## 5. Blast Radius Verification

The spec predicted behavioral changes, files affected, and existing test
impact. Compare predictions against reality. Surprises here reveal spec
gaps worth learning from.

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| None (Spec A adds tests only — no production code changes) | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/tests/mujoco_conformance/layer_b.rs` (new, +1,200–1,500 lines) | | |
| `sim/L0/tests/mujoco_conformance/mod.rs` (add `mod layer_b;`, +1 line) | | |
| `sim/L0/tests/mujoco_conformance/common.rs` (extend with helpers, +120–150 lines) | | |

{List any files changed that were NOT in the spec's prediction:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| All Layer A tests (`layer_a.rs`) | Pass (unchanged) — separate module | | |
| All Layer D tests (`layer_d.rs`) | Pass (unchanged) — separate module | | |
| All integration tests (`integration/*.rs`) | Pass (unchanged) — separate test binary | | |
| `common.rs` existing code | Pass (unchanged) — all changes additive | | |

**Unexpected regressions:**

---

## 6. Convention Notes Audit

Check every row in the spec's Convention Notes table. Convention mismatches
are the #1 source of silent conformance bugs.

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| `xpos` | Access: `ref_data[i*3+k]` ↔ `data.xpos[i][k]` | | |
| `xquat` | Access: `ref[i*4+0]` ↔ `q.w`, `ref[i*4+1]` ↔ `q.i`, etc. Sign ambiguity: `min(‖q_cf - q_ref‖, ‖q_cf + q_ref‖) < tol` | | |
| `xipos` | Same as xpos | | |
| `qM` | Access: `ref[i*nv+j]` ↔ `data.qM[(i,j)]` | | |
| `qfrc_bias` | Access: `ref[i]` ↔ `data.qfrc_bias[i]` | | |
| `qfrc_passive` | Same as qfrc_bias | | |
| `qfrc_actuator` | Same as qfrc_bias | | |
| `actuator_force` | Access: `ref[i]` ↔ `data.actuator_force[i]` | | |
| `contact.dist` → `contact.depth` | **Negate:** `depth_cf ≈ -ref_dist` | | |
| `contact.frame` → `contact.normal` | Access: `ref_normal[k]` ↔ `contact.normal[k]` | | |
| `contact.geom` | Cast: `ref_pair[0] as usize` ↔ `contact.geom1` | | |
| `sensordata` | Access: `ref[i]` ↔ `data.sensordata[i]` | | |
| `ten_length` | Access: `ref[i]` ↔ `data.ten_length[i]` | | |
| `ten_velocity` | Same as ten_length | | |
| `efc_J` | Access: `ref[i*nv+j]` ↔ `data.efc_J[(i,j)]` | | |
| `efc_b` | Access: `ref[i]` ↔ `data.efc_b[i]` | | |
| `efc_force` | Access: `ref[i]` ↔ `data.efc_force[i]` | | |

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

---

## 8. Deferred Work Tracker

Every item that was in the spec's scope but not fully implemented, plus
anything discovered during implementation or review that's out of scope.
**The goal: nothing deferred is untracked.**

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| Integration stage reference tests | Out of Scope, bullet 1 | Spec B (Layer C — trajectory comparison) | Phase 12 Session 12 | |
| Additional FK fields (xmat, geom_xpos, geom_xmat, site_xpos) | Out of Scope, bullet 2 | | | |
| Fixing conformance failures | Out of Scope, bullet 3 | Spec A specifies `#[ignore]` with tracking comments | Session 15 (Gate Triage) | |
| Flag-dependent reference data | Out of Scope, bullet 4 | DT-97 (Session 2) | DT-97 | |

### Discovered During Implementation

Items that came up during implementation that weren't anticipated by the
spec. These are the most likely to be untracked.

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|

### Discovered During Review

Items found during this review that were not surfaced during
implementation. Reviews are a discovery mechanism.

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|

### Spec Gaps Found During Implementation

Items where the spec was wrong or incomplete and was (or should have been)
updated. Verify the spec was actually updated.

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|

---

## 9. Test Coverage Summary

Quick sanity check on test health after the implementation.

**Domain test results:**
```
sim-core:              N passed, 0 failed, M ignored
sim-mjcf:              N passed, 0 failed, M ignored
sim-conformance-tests: N passed, 0 failed, M ignored
Total:                 N passed, 0 failed, M ignored
```

**New tests added:**
**Tests modified:**
**Pre-existing test regressions:**

**Clippy:**
**Fmt:**

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

**Overall:**

**Items fixed during review:**

**Items to fix before shipping:**

**Items tracked for future work:**
